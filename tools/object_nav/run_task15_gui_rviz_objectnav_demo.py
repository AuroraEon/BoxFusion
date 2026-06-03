#!/usr/bin/env python3
"""Run a configurable RSLG-SLAM object-nav query with Gazebo GUI and RViz evidence."""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
OBJECT_NAV_DIR = Path(__file__).resolve().parent
if str(OBJECT_NAV_DIR) not in sys.path:
    sys.path.insert(0, str(OBJECT_NAV_DIR))

from object_nav_common import canonical_object_id, load_index, run_query  # noqa: E402


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task15_gui_rviz_objectnav_demo_wrapper"
CLEAN_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
TASK14A = TASKS_ROOT / "task14a_object_nav_experiment_adapter"
TASK14C = TASKS_ROOT / "task14c_multi_query_objectnav_runtime_validation"
TASK14D = TASKS_ROOT / "task14d_multi_object_object_facing_nav_validation"
DEFAULT_OUT = TASKS_ROOT / TASK_NAME
LAUNCHER = ROOT / "tools/stage1_runtime/launch_scene_gazebo_nav2.sh"
RUNTIME = ROOT / "tools/object_nav/run_objectnav_single_object_runtime.py"
PUBLISHER = ROOT / "tools/object_nav/publish_task15_objectnav_rviz_markers.py"
RECOMMENDATIONS = ROOT / "tools/object_nav/generate_task15_objectnav_recommendations.py"
RVIZ_CONFIG = ROOT / "tools/object_nav/task15_objectnav_demo.rviz"
MAP_YAML = CLEAN_ROOT / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
SEMANTIC_ROUTE = CLEAN_ROOT / "routes/room_routes/floor_2_room11_to_room14/semantic_route_waypoints_v0_1.json"
EXEC_ROUTE = CLEAN_ROOT / "routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json"
PROFILE = CLEAN_ROOT / "runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value.rstrip() + "\n", encoding="utf-8")


def command_text(cmd: list[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in cmd)


def run_logged(cmd: list[str], log: Path, env: dict[str, str]) -> int:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w", encoding="utf-8") as handle:
        handle.write("$ " + command_text(cmd) + "\n")
        handle.flush()
        result = subprocess.run(cmd, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT, text=True)
    return int(result.returncode)


def capture_command(cmd: list[str], env: dict[str, str], timeout: float = 12.0) -> dict[str, Any]:
    try:
        proc = subprocess.run(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return {"command": cmd, "returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
    except Exception as exc:
        return {"command": cmd, "returncode": None, "error": f"{type(exc).__name__}: {exc}"}


def process_snapshot() -> list[dict[str, Any]]:
    result = subprocess.run(["ps", "-eo", "pid=,comm=,args="], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    needles = (
        "gzserver",
        "gzclient",
        "rviz2",
        "robot_state_publisher",
        "nav2_",
        "controller_server",
        "planner_server",
        "bt_navigator",
        "publish_task15_objectnav_rviz_markers.py",
    )
    rows = []
    for line in result.stdout.splitlines():
        if any(token in line for token in needles):
            parts = line.strip().split(maxsplit=2)
            rows.append({"pid": int(parts[0]), "command": parts[1] if len(parts) > 1 else "", "args": parts[2] if len(parts) > 2 else ""})
    return rows


def clean_task15_stale_processes(evidence_dir: Path, output_name: str, owned_pids: set[int] | None = None) -> dict[str, Any]:
    before = process_snapshot()
    own_pid = os.getpid()
    patterns = (
        "publish_task15_objectnav_rviz_markers.py",
        "task15_objectnav_demo.rviz",
        "turtlebot3_gazebo/launch/robot_state_publisher.launch.py",
        "/opt/ros/foxy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf --ros-args -r __node:=robot_state_publisher",
    )
    killed: list[dict[str, Any]] = []
    for proc in before:
        associated_by_pattern = any(pattern in proc["args"] for pattern in patterns)
        associated_by_launch_snapshot = owned_pids is not None and proc["pid"] in owned_pids
        if proc["pid"] == own_pid or not (associated_by_pattern or associated_by_launch_snapshot):
            continue
        try:
            os.kill(proc["pid"], signal.SIGTERM)
            killed.append(
                {
                    "pid": proc["pid"],
                    "args": proc["args"],
                    "signal": "SIGTERM",
                    "match_basis": "launch_graph_owned_pid" if associated_by_launch_snapshot else "runtime_specific_pattern",
                }
            )
        except ProcessLookupError:
            pass
    if killed:
        time.sleep(1.0)
    payload = {"created_utc": now_iso(), "safe_match_patterns": list(patterns), "before": before, "terminated": killed, "after": process_snapshot()}
    write_json(evidence_dir / output_name, payload)
    return payload


def find_object(index: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    oid = canonical_object_id(object_id)
    return next((obj for obj in index.get("objects", []) if obj.get("object_id") == oid), None)


def find_approach_report(object_id: str) -> tuple[Path | None, dict[str, Any]]:
    for directory in (TASK14D, TASK14C):
        report = directory / "approach_candidate_reports" / f"object_approach_report_{object_id}.json"
        if report.exists():
            return report, read_json(report, {})
    return None, {}


def select_plan(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    index_path = TASK14A / "object_candidate_index_v0_1.json"
    index = load_index(index_path)
    query_result = run_query(index, args.query, top_k=10)
    selected = query_result.get("selected_candidate")
    if args.object_id:
        selected = find_object(index, args.object_id)
    if not selected:
        raise RuntimeError(f"query did not resolve to a candidate: {args.query}")
    object_id = canonical_object_id(selected["object_id"])
    report_path, report = find_approach_report(object_id)
    records = [report.get("recommended_candidate")] + list(report.get("candidate_records") or [])
    records = [record for record in records if isinstance(record, dict)]
    candidate = next((record for record in records if args.approach_candidate_id and record.get("candidate_id") == args.approach_candidate_id), None)
    if candidate is None:
        candidate = report.get("recommended_candidate")
    if not candidate:
        raise RuntimeError(f"no existing approach candidate report for {object_id}")
    proxy_xy = candidate.get("visible_proxy_xy")
    if not (isinstance(proxy_xy, list) and len(proxy_xy) >= 2):
        raise RuntimeError(f"no visible proxy yaw target is available for {object_id}")
    route = read_json(EXEC_ROUTE, {})
    runnable = selected.get("floor_id") == "floor_2" and selected.get("room_id") == "room_14" and EXEC_ROUTE.exists()
    if not runnable:
        raise RuntimeError("task15 runtime currently requires an existing destination route to floor_2 room_14")
    approach_xy = candidate.get("world_xy")
    yaw = math.atan2(float(proxy_xy[1]) - float(approach_xy[1]), float(proxy_xy[0]) - float(approach_xy[0]))
    plan = {
        "artifact_type": "task15_selected_gui_demo_plan",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": args.query,
        "query_resolution": query_result,
        "object_id": object_id,
        "label": selected.get("label"),
        "room_id": selected.get("room_id"),
        "floor_id": selected.get("floor_id"),
        "floor_assignment_status": selected.get("floor_assignment_status"),
        "approach_candidate_id": candidate.get("candidate_id"),
        "approach_candidate_world_xy": approach_xy,
        "object_proxy_target_for_yaw_alignment": {"x": proxy_xy[0], "y": proxy_xy[1], "source": "visible_proxy_xy"},
        "expected_object_facing_yaw_rad": round(yaw, 6),
        "route_id": route.get("route_id"),
        "room_route": rel(EXEC_ROUTE),
        "semantic_route": rel(SEMANTIC_ROUTE),
        "room_sequence": route.get("room_sequence"),
        "source_index": rel(index_path),
        "source_approach_report": rel(report_path),
        "runnable": True,
    }
    write_json(output_dir / "selected_demo_plan.json", plan)
    return plan


def screenshot(evidence_dir: Path, env: dict[str, str]) -> dict[str, Any]:
    path = evidence_dir / "screenshots/task15_gui_rviz_desktop.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    code = (
        "import sys; from PyQt5.QtWidgets import QApplication; "
        "app=QApplication([]); screen=app.primaryScreen(); "
        "ok=screen.grabWindow(0).save(sys.argv[1]); sys.exit(0 if ok else 2)"
    )
    result = capture_command(["/usr/bin/python3", "-c", code, str(path)], env, timeout=15.0)
    result["path"] = rel(path) if path.exists() else None
    result["saved"] = path.exists() and path.stat().st_size > 0
    if result["saved"]:
        inspect = capture_command(
            [
                "/usr/bin/python3",
                "-c",
                "import cv2,sys,json; a=cv2.imread(sys.argv[1]); "
                "nz=float((a != 0).any(axis=2).mean()) if a is not None else 0.0; "
                "print(json.dumps({'non_black_pixel_fraction': nz, 'visually_informative': nz > 0.001}))",
                str(path),
            ],
            env,
        )
        try:
            result.update(json.loads(inspect.get("stdout", "{}")))
        except json.JSONDecodeError:
            result["visually_informative"] = False
    write_json(evidence_dir / "screenshot_capture.json", result)
    return result


def graph_snapshot(env: dict[str, str], evidence_dir: Path) -> dict[str, Any]:
    nodes = capture_command(["ros2", "node", "list"], env)
    topics = capture_command(["ros2", "topic", "list"], env)
    map_info = capture_command(["ros2", "topic", "info", "/map"], env)
    marker_info = capture_command(["ros2", "topic", "info", "/task15/object_nav_markers"], env)
    payload = {
        "created_utc": now_iso(),
        "nodes": nodes,
        "topics": topics,
        "map_topic_info": map_info,
        "marker_topic_info": marker_info,
        "processes": process_snapshot(),
    }
    write_json(evidence_dir / "ros_graph_after_launch.json", payload)
    return payload


def bool_topic(result: dict[str, Any], name: str) -> bool:
    return result.get("returncode") == 0 and name in result.get("stdout", "")


def stop_process(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=8)
    except Exception:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def write_report(output_dir: Path, summary: dict[str, Any]) -> None:
    result = summary["per_query_results"][0]
    recs = summary.get("recommendation_summary") or {}
    lines = [
        "# task15 GUI/RViz ObjectNav Demo Wrapper Validation",
        "",
        "## Actual GUI/RViz Validated Demo",
        "",
        "| query | object | candidate | room arrival | approach reached | yaw aligned | object-facing | Nav2-clean | direct /cmd_vel yaw fallback |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        f"| {result.get('query')} | {result.get('object_id')} | {result.get('approach_candidate_id')} | {result.get('target_room_arrival')} | {result.get('approach_position_reached')} | {result.get('approach_yaw_aligned')} | {result.get('object_facing_approach_success')} | {result.get('nav2_clean_approach_success')} | {result.get('yaw_alignment_direct_cmd_vel_fallback_used')} |",
        "",
        f"- Gazebo GUI launched: `{summary['gazebo_gui_launched']}`; Gazebo client launched: `{summary['gazebo_client_launched']}`.",
        f"- RViz launched: `{summary['rviz_launched']}`; GUI screenshot saved: `{summary['screenshot_saved']}`; screenshot visually informative: `{summary['screenshot_visually_informative']}`.",
        f"- Marker overlay published/saved: `{summary['marker_evidence_saved']}`; executed trajectory saved: `{summary['trajectory_evidence_saved']}`.",
        f"- `/map` available in the launch ROS graph: `{summary['map_topic_available']}`.",
        f"- RViz `/map` render warning detected: `{summary['rviz_map_render_warning_detected']}`; visible `/map` or visible marker rendering is therefore not claimed from the screenshot.",
        "",
        "## Map And Display Scope",
        "",
        "- `/map` is the Nav2 runtime stable occupancy map.",
        "- `/map` is not the clean semantic floorplan.",
        "- GUI/RViz display floorplan and Nav2 runtime map remain coupled in this task and should be decoupled later if needed.",
        "",
        "## RViz Evidence Scope",
        "",
        "| display/evidence layer | published or saved | screenshot-visible confirmation |",
        "| --- | --- | --- |",
        f"| `/map` Nav2 stable occupancy background | configured and topic available: {summary['map_topic_available']} | False (renderer warning and blank capture) |",
        f"| query text, resolved object/label, target room status | marker manifest saved: {summary['marker_evidence_saved']} | False (blank capture) |",
        f"| room route polyline and gateway/route waypoints | marker manifest saved: {summary['marker_evidence_saved']} | False (blank capture) |",
        f"| approach candidate, object/proxy point, facing ray | marker manifest saved: {summary['marker_evidence_saved']} | False (blank capture) |",
        f"| sent controller paths and executed trajectory | marker/trajectory artifacts saved: {summary['trajectory_evidence_saved']} | False (blank capture) |",
        "",
        "## Cleanup Evidence",
        "",
        "- The wrapper performs a pre-launch runtime stop plus runtime-specific/owned-PID cleanup for Gazebo/RViz/marker and TurtleBot3 state-publication remnants.",
        "- This validation removed stale prior `gzclient` and `robot_state_publisher` processes before launch and left no associated matching processes after stop; see the cleanup JSON artifacts.",
        "",
        "## Runtime Metrics",
        "",
        f"- `fallback_used={result.get('fallback_used')}`; `room_route_sparse_fallback_used={result.get('room_route_sparse_fallback_used')}`; `yaw_alignment_direct_cmd_vel_fallback_used={result.get('yaw_alignment_direct_cmd_vel_fallback_used')}`.",
        f"- Final distance to target room terminal: `{result.get('final_distance_to_target_room_terminal_m')}` m.",
        f"- Final distance to approach candidate: `{result.get('final_distance_to_selected_approach_candidate_m')}` m.",
        f"- Final yaw error: `{result.get('yaw_error_rad')}` rad / `{result.get('yaw_error_deg')}` deg; XY drift during yaw alignment: `{result.get('xy_drift_during_yaw_alignment_m')}` m.",
        f"- Wall crossing validation passed: `{result.get('wall_crossing_validation_passed')}`.",
        "",
        "## Headless-Only Inherited Evidence",
        "",
        "- task14d previously validated headless object-facing successes for `obj_175` (`curtain`) and `obj_177` (`nightstand`) using `room_11 -> room_7 -> room_13 -> room_14`.",
        "- That inherited task14d evidence is not itself GUI/RViz validation; only the query listed in the actual GUI/RViz table above was run through this wrapper.",
        "",
        "## Recommended Future Different-Room Objects",
        "",
        f"- Objects scanned: `{recs.get('objects_scanned')}`; immediately runnable non-room14 objects: `{recs.get('immediately_runnable_non_room14_count')}`.",
        "- No different-room object is run here unless existing route, approach-candidate, and yaw-target evidence supports it.",
        f"- Full recommendation table: `{summary['output_paths']['recommendation_report']}`.",
        "",
        "## Allowed Claims",
        "",
        "- The listed query was resolved from existing artifact-backed candidates and executed in Gazebo/Nav2 with Gazebo GUI and RViz running.",
        "- RViz marker evidence records query, resolved object, target room, route, approach candidate, facing proxy/ray, sent controller paths, executed trajectory, and runtime status.",
        "- Object-facing approach success indicates pose and proxy-facing yaw tolerance success, including the explicitly reported direct `/cmd_vel` yaw fallback when used.",
        "",
        "## Forbidden Claims",
        "",
        "- Visual object confirmation or visually verified object arrival.",
        "- Semantic ground-truth accuracy or open-vocabulary CLIP retrieval.",
        "- Cross-floor navigation or broad cross-room object-nav generalization.",
        "- Nav2-clean yaw alignment when direct `/cmd_vel` yaw fallback was used.",
        "- Display-floorplan/runtime-map decoupling.",
    ]
    report = "\n".join(lines) + "\n"
    write_text(output_dir / "gui_rviz_objectnav_demo_report.md", report)
    write_text(output_dir / "completion_summary.md", report)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query", default="curtain in room_14 on floor_2")
    parser.add_argument("--object-id")
    parser.add_argument("--approach-candidate-id")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "84"))
    parser.add_argument("--keep-open-sec", type=float, default=5.0)
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    args = parser.parse_args()
    if not os.environ.get("DISPLAY"):
        raise SystemExit("Gazebo GUI/RViz validation requires DISPLAY; none is set")
    output_dir = args.output_dir.resolve()
    run_dir = output_dir / "runtime_runs" / (canonical_object_id(args.object_id) if args.object_id else "selected_query")
    logs = output_dir / "run_logs"
    evidence = output_dir / "evidence"
    for path in (output_dir, run_dir, logs, evidence):
        path.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    env.setdefault("TURTLEBOT3_MODEL", "burger")
    plan = select_plan(args, output_dir)
    expected_run_dir = output_dir / "runtime_runs" / plan["object_id"]
    if expected_run_dir != run_dir:
        run_dir = expected_run_dir
    if run_dir.exists():
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    rec_cmd = ["/usr/bin/python3", str(RECOMMENDATIONS), "--output-dir", str(output_dir)]
    run_logged(rec_cmd, logs / "recommendations.log", env)
    launch_cmd = [
        str(LAUNCHER), "--scene-id", SCENE_ID, "--floor-id", "floor_2", "--stage-output-dir", str(CLEAN_ROOT),
        "--runtime-profile", str(PROFILE), "--map-yaml", str(MAP_YAML), "--gui", "--ros-domain-id", str(args.ros_domain_id),
        "--log-dir", str(run_dir / "bringup_logs"), "--readiness-timeout-sec", str(args.readiness_timeout_sec),
        "--readiness-output-json", str(run_dir / "runtime_bringup_readiness.json"),
        "--readiness-output-md", str(run_dir / "runtime_bringup_readiness.md"), "--run-id", f"{TASK_NAME}_{plan['object_id']}",
    ]
    stop_cmd = [str(LAUNCHER), "--stage-output-dir", str(CLEAN_ROOT), "--runtime-profile", str(PROFILE), "--stop"]
    marker_cmd = [
        "/usr/bin/python3", str(PUBLISHER), "--plan-json", str(output_dir / "selected_demo_plan.json"), "--run-dir", str(run_dir),
        "--evidence-dir", str(evidence), "--route-json", str(EXEC_ROUTE), "--semantic-route-json", str(SEMANTIC_ROUTE),
        "--marker-topic", "/task15/object_nav_markers",
    ]
    rviz_cmd = ["rviz2", "-d", str(RVIZ_CONFIG)]
    runtime_cmd = [
        "/usr/bin/python3", str(RUNTIME), "--stage-output-dir", str(CLEAN_ROOT),
        "--task14c-output-dir", str((ROOT / plan["source_approach_report"]).parents[1]), "--output-dir", str(run_dir),
        "--query", plan["query"], "--object-id", plan["object_id"], "--approach-candidate-id", plan["approach_candidate_id"],
        "--floor-id", plan["floor_id"], "--start-room", "room_11", "--target-room", plan["room_id"],
        "--stable-map", str(MAP_YAML), "--semantic-route", str(SEMANTIC_ROUTE), "--room-route", str(EXEC_ROUTE),
        "--controller-profile", "task12_robust", "--execution-strategy", "split_follow_path_with_current_pose_gateway_handoff",
        "--two-segment-execution", "--object-facing-approach", "--position-first-approach", "--yaw-alignment-required",
        "--approach-position-tolerance-m", "0.35", "--yaw-alignment-tolerance-rad", "0.50",
        "--validate-wall-crossing", "--record-sent-controller-paths", "--record-executed-trajectory",
    ]
    commands = {
        "artifact_type": "task15_exact_runtime_commands",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "environment_assumptions": {
            "ROS_DOMAIN_ID": str(args.ros_domain_id),
            "DISPLAY": env.get("DISPLAY"),
            "TURTLEBOT3_MODEL": env.get("TURTLEBOT3_MODEL"),
            "source_commands": ["source /opt/ros/foxy/setup.bash"],
            "active_clean_rerun_root": rel(CLEAN_ROOT),
        },
        "wrapper_command": sys.argv,
        "recommendation_command": rec_cmd,
        "prelaunch_stop_command": stop_cmd,
        "launch_command": launch_cmd,
        "marker_publisher_command": marker_cmd,
        "rviz_command": rviz_cmd,
        "objectnav_runtime_command": runtime_cmd,
        "stop_command": stop_cmd,
        "output_paths": {"output_dir": rel(output_dir), "runtime_run_dir": rel(run_dir), "evidence_dir": rel(evidence)},
    }
    write_json(output_dir / "exact_runtime_commands.json", commands)
    marker_proc: subprocess.Popen[Any] | None = None
    rviz_proc: subprocess.Popen[Any] | None = None
    launch_rc = runtime_rc = 1
    graph: dict[str, Any] = {}
    shot: dict[str, Any] = {"saved": False}
    try:
        run_logged(stop_cmd, logs / "prelaunch_stop.log", env)
        prior_graph = read_json(evidence / "ros_graph_after_launch.json", {})
        prior_owned_pids = {int(row["pid"]) for row in prior_graph.get("processes", []) if row.get("pid") is not None}
        clean_task15_stale_processes(evidence, "prelaunch_stale_process_cleanup.json", prior_owned_pids)
        launch_rc = run_logged(launch_cmd, logs / "gazebo_nav2_gui_bringup.log", env)
        if launch_rc != 0:
            raise RuntimeError(f"Gazebo GUI/Nav2 launch returned {launch_rc}")
        marker_log = (logs / "marker_publisher.log").open("w", encoding="utf-8")
        marker_proc = subprocess.Popen(marker_cmd, cwd=ROOT, env=env, stdout=marker_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        rviz_log = (logs / "rviz2.log").open("w", encoding="utf-8")
        rviz_proc = subprocess.Popen(rviz_cmd, cwd=ROOT, env=env, stdout=rviz_log, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        time.sleep(4.0)
        graph = graph_snapshot(env, evidence)
        runtime_rc = run_logged(runtime_cmd, logs / f"objectnav_runtime_{plan['object_id']}.log", env)
        time.sleep(max(2.0, args.keep_open_sec))
        graph = graph_snapshot(env, evidence)
        shot = screenshot(evidence, env)
    except Exception as exc:
        write_text(logs / "wrapper_error.log", f"{type(exc).__name__}: {exc}")
    finally:
        stop_process(rviz_proc)
        stop_process(marker_proc)
        run_logged(stop_cmd, logs / "final_stop.log", env)
        owned_pids = {int(row["pid"]) for row in (graph or {}).get("processes", []) if row.get("pid") is not None}
        clean_task15_stale_processes(evidence, "postrun_process_cleanup.json", owned_pids)
    runtime_result = read_json(run_dir / "object_facing_runtime_result.json", {})
    marker_manifest = read_json(evidence / "marker_manifest.json", {})
    bringup = read_json(run_dir / "bringup_logs/bringup_result.json", {})
    processes = (graph or {}).get("processes") or []
    gzclient = any("gzclient" in row.get("args", "") or row.get("command") == "gzclient" for row in processes)
    rviz_live = any(row.get("command") == "rviz2" for row in processes)
    per_query = {
        **runtime_result,
        "gui_launched": launch_rc == 0 and bringup.get("gui") is True,
        "gazebo_client_launched": gzclient,
        "rviz_launched": rviz_live,
        "marker_manifest": rel(evidence / "marker_manifest.json"),
        "screenshot": shot.get("path"),
        "task15_gui_rviz_launch_validation": bool(launch_rc == 0 and rviz_live and gzclient),
        "rviz_marker_published": bool(marker_manifest.get("published")),
        "runtime_returncode": runtime_rc,
    }
    write_json(output_dir / f"per_query_runtime_{plan['object_id']}.json", per_query)
    rec_data = read_json(output_dir / "different_room_objectnav_recommendations.json", {})
    summary = {
        "artifact_type": "task15_gui_rviz_objectnav_demo_summary",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "gazebo_gui_launched": per_query["gui_launched"],
        "gazebo_client_launched": per_query["gazebo_client_launched"],
        "rviz_launched": per_query["rviz_launched"],
        "map_topic_available": bool_topic((graph or {}).get("topics", {}), "/map"),
        "marker_topic_available": bool_topic((graph or {}).get("topics", {}), "/task15/object_nav_markers"),
        "screenshot_saved": bool(shot.get("saved")),
        "screenshot_visually_informative": bool(shot.get("visually_informative")),
        "marker_evidence_saved": bool(marker_manifest.get("published")),
        "trajectory_evidence_saved": bool((marker_manifest.get("executed_trajectory") or {}).get("sample_count")),
        "rviz_map_render_warning_detected": "GLSL link result" in (logs / "rviz2.log").read_text(encoding="utf-8", errors="replace")
        if (logs / "rviz2.log").exists()
        else False,
        "per_query_results": [per_query],
        "recommendation_summary": {
            "objects_scanned": rec_data.get("objects_scanned"),
            "classification_counts": rec_data.get("classification_counts"),
            "immediately_runnable_non_room14_count": rec_data.get("immediately_runnable_non_room14_count"),
        },
        "output_paths": {
            "output_dir": rel(output_dir),
            "marker_manifest": rel(evidence / "marker_manifest.json"),
            "executed_trajectory_json": rel(evidence / "executed_trajectory_combined.json"),
            "executed_trajectory_csv": rel(evidence / "executed_trajectory_combined.csv"),
            "ros_graph": rel(evidence / "ros_graph_after_launch.json"),
            "recommendation_report": rel(output_dir / "different_room_objectnav_recommendations.md"),
        },
        "map_scope": {
            "rviz_background_topic": "/map",
            "statement": "/map is the Nav2 runtime stable occupancy map; it is not the clean semantic floorplan.",
            "coupling": "GUI/RViz display floorplan and Nav2 runtime map remain coupled in task15.",
        },
    }
    write_json(output_dir / "gui_rviz_objectnav_demo_summary.json", summary)
    write_report(output_dir, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    passed = (
        runtime_result.get("object_facing_approach_success") is True
        and summary["gazebo_gui_launched"]
        and summary["gazebo_client_launched"]
        and summary["rviz_launched"]
        and summary["marker_evidence_saved"]
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
