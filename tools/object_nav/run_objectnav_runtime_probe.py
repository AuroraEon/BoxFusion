#!/usr/bin/env python3
"""Run one task14c object-nav runtime probe through the existing Stage1 runtime."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SCENE_ID = "00843-DYehNKdT76V"
EXPECTED_ROOM_CHAIN = "room_11,room_7,room_13,room_14"
EXPECTED_GATEWAY_SEQUENCE = "00843_floor2_gateway_005,00843_floor2_gateway_004,00843_floor2_gateway_006"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path, default: Any | None = None) -> Any:
    if not path.exists():
        if default is not None:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_md(path: Path, result: dict[str, Any]) -> None:
    lines = [
        f"# Runtime Probe: {result.get('object_id')}",
        "",
        f"- Query: `{result.get('query')}`",
        f"- Attempted: `{result.get('attempted')}`",
        f"- Runtime result: `{result.get('runtime_result')}`",
        f"- Target room arrival: `{result.get('target_room_arrival')}`",
        f"- Approach pose reached: `{result.get('approach_pose_reached')}`",
        f"- FollowPath used: `{result.get('follow_path_used')}`",
        f"- Fallback used: `{result.get('fallback_used')}`",
        f"- Failure layer: `{result.get('failure_layer')}`",
        f"- Failure reason: `{result.get('failure_reason')}`",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def import_rclpy_status() -> tuple[bool, str | None]:
    try:
        import rclpy  # noqa: F401
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"
    return True, None


def candidate_from_report(task14c_output_dir: Path, object_id: str, candidate_id: str) -> dict[str, Any] | None:
    report = read_json(task14c_output_dir / "approach_candidate_reports" / f"object_approach_report_{object_id}.json", {})
    candidate = report.get("recommended_candidate")
    if candidate and candidate.get("candidate_id") == candidate_id:
        return candidate
    for candidate in report.get("candidate_records", []):
        if candidate.get("candidate_id") == candidate_id:
            return candidate
    return None


def append_approach_waypoint(base_route: dict[str, Any], candidate: dict[str, Any], object_id: str, query: str) -> dict[str, Any]:
    waypoints = list(base_route.get("waypoints") or [])
    if not waypoints:
        raise ValueError("base executable route has no waypoints")
    last = dict(waypoints[-1])
    x, y = float(candidate["world_xy"][0]), float(candidate["world_xy"][1])
    yaw = float(candidate.get("yaw") or 0.0)
    approach = {
        **last,
        "x": x,
        "y": y,
        "yaw": yaw,
        "source": "object_approach_pose",
        "room_id": last.get("room_id") or "room_14",
        "target_room": last.get("target_room") or "room_14",
        "object_id": object_id,
        "object_query": query,
        "approach_candidate_id": candidate.get("candidate_id"),
        "waypoint_index": len(waypoints),
    }
    if waypoints:
        prev = waypoints[-1]
        approach["segment_distance_from_route_terminal_m"] = round(
            math.hypot(float(prev["x"]) - x, float(prev["y"]) - y), 6
        )
    return {
        **base_route,
        "artifact_type": "task14c_objectnav_extended_executable_route",
        "created_utc": utc_now(),
        "route_extension_policy": "append selected object approach pose after existing room_11_to_room14 route terminal",
        "object_id": object_id,
        "query": query,
        "approach_candidate_id": candidate.get("candidate_id"),
        "waypoints": waypoints + [approach],
    }


def update_runtime_collection(task14c_output_dir: Path, result: dict[str, Any]) -> None:
    path = task14c_output_dir / "runtime_probe_results.json"
    payload = read_json(path, {
        "artifact_type": "task14c_runtime_probe_results",
        "created_utc": utc_now(),
        "probes": [],
    })
    probes = [p for p in payload.get("probes", []) if p.get("object_id") != result.get("object_id")]
    probes.append(result)
    payload.update(
        {
            "created_utc": utc_now(),
            "ros_gazebo_nav2_rviz_started": any(p.get("ros_gazebo_nav2_started") for p in probes),
            "gui_rviz_visual_validation_available": any(p.get("gui_visual_confirmation") for p in probes),
            "runtime_probe_count": len(probes),
            "runtime_probes_attempted": sum(1 for p in probes if p.get("attempted")),
            "runtime_probes_succeeded": sum(1 for p in probes if p.get("runtime_result") == "succeeded"),
            "probes": probes,
        }
    )
    write_json(path, payload)
    lines = ["# Runtime Probe Results", ""]
    for probe in probes:
        lines.extend(
            [
                f"## {probe.get('object_id')}",
                f"- Query: `{probe.get('query')}`",
                f"- Result: `{probe.get('runtime_result')}`",
                f"- Target room arrival: `{probe.get('target_room_arrival')}`",
                f"- Approach pose reached: `{probe.get('approach_pose_reached')}`",
                f"- Failure: `{probe.get('failure_layer')}` - {probe.get('failure_reason')}",
                "",
            ]
        )
    (task14c_output_dir / "runtime_probe_results.md").write_text("\n".join(lines), encoding="utf-8")


def update_summary(task14c_output_dir: Path) -> None:
    summary_path = task14c_output_dir / "objectnav_validation_summary.json"
    summary = read_json(summary_path, {})
    runtime = read_json(task14c_output_dir / "runtime_probe_results.json", {})
    probes = runtime.get("probes", [])
    summary.update(
        {
            "ros_gazebo_nav2_rviz_started": bool(runtime.get("ros_gazebo_nav2_rviz_started")),
            "gui_rviz_visual_validation_available": bool(runtime.get("gui_rviz_visual_validation_available")),
            "runtime_probes_attempted": sum(1 for p in probes if p.get("attempted")),
            "runtime_probes_succeeded": sum(1 for p in probes if p.get("runtime_result") == "succeeded"),
            "per_runtime_probe_result": [
                {
                    "object_id": p.get("object_id"),
                    "query": p.get("query"),
                    "runtime_result": p.get("runtime_result"),
                    "target_room_arrival": p.get("target_room_arrival"),
                    "approach_pose_reached": p.get("approach_pose_reached"),
                    "failure_layer": p.get("failure_layer"),
                    "failure_reason": p.get("failure_reason"),
                }
                for p in probes
            ],
        }
    )
    write_json(summary_path, summary)
    failed_queries = summary.get("failed_queries", [])
    lines = [
        "# task14c Multi-Query ObjectNav Validation Summary",
        "",
        f"- Stage-A rerun: `{summary.get('stage_a_rerun')}`",
        f"- 00824 modified: `{summary.get('reference_00824_modified')}`",
        f"- ROS/Gazebo/Nav2/RViz started: `{summary.get('ros_gazebo_nav2_rviz_started')}`",
        f"- GUI/RViz visual validation available: `{summary.get('gui_rviz_visual_validation_available')}`",
        f"- Object queries evaluated: `{summary.get('object_queries_evaluated')}`",
        f"- Object queries resolved: `{summary.get('object_queries_resolved')}`",
        f"- Target rooms resolved: `{summary.get('target_rooms_resolved')}`",
        f"- Route-available queries: `{summary.get('route_available_queries')}`",
        f"- Approach-candidate-available queries: `{summary.get('approach_candidate_available_queries')}`",
        f"- Runtime probes attempted: `{summary.get('runtime_probes_attempted')}`",
        f"- Runtime probes succeeded: `{summary.get('runtime_probes_succeeded')}`",
        "",
        "## Per Runtime Probe Result",
        *[
            f"- `{p.get('object_id')}` / `{p.get('query')}`: result=`{p.get('runtime_result')}`, target_room_arrival=`{p.get('target_room_arrival')}`, approach_pose_reached=`{p.get('approach_pose_reached')}`, failure=`{p.get('failure_layer')}` - {p.get('failure_reason')}"
            for p in summary.get("per_runtime_probe_result", [])
        ],
        "",
        "## Failed Queries",
        *[f"- `{item['query']}`: `{item['failure_layer']}` - {item['failure_reason']}" for item in failed_queries],
        "",
        "## Exact Claims Allowed",
        *[f"- {claim}" for claim in summary.get("allowed_claims", [])],
        "",
        "## Exact Claims Not Allowed",
        *[f"- {claim}" for claim in summary.get("claims_not_allowed", [])],
        "",
    ]
    md = "\n".join(lines)
    (task14c_output_dir / "objectnav_validation_summary.md").write_text(md, encoding="utf-8")
    (task14c_output_dir / "completion_summary.md").write_text(md, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--task14c-output-dir", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--approach-candidate-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--start-room", default="room_11")
    parser.add_argument("--target-room", required=True)
    parser.add_argument("--controller-profile", choices=["baseline", "task12_robust"], default="task12_robust")
    parser.add_argument("--execution-strategy", default="split_follow_path_with_current_pose_gateway_handoff")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--skip-route-execution", action="store_true")
    args = parser.parse_args()

    out = args.task14c_output_dir
    object_id = args.object_id if args.object_id.startswith("obj_") else f"obj_{args.object_id}"
    run_dir = out / "runtime_runs" / object_id
    run_dir.mkdir(parents=True, exist_ok=True)
    for subdir in ("sent_controller_paths", "executed_trajectories", "wall_crossing_validation"):
        (out / subdir).mkdir(parents=True, exist_ok=True)

    result: dict[str, Any] = {
        "artifact_type": "task14c_objectnav_runtime_probe_result",
        "created_utc": utc_now(),
        "query": args.query,
        "object_id": object_id,
        "approach_candidate_id": args.approach_candidate_id,
        "floor_id": args.floor_id,
        "start_room": args.start_room,
        "target_room": args.target_room,
        "attempted": False,
        "runtime_result": "failed",
        "target_room_arrival": False,
        "approach_pose_reached": False,
        "follow_path_used": False,
        "fallback_used": None,
        "wall_crossing_validation_run": False,
        "rviz_marker_published": False,
        "gui_visual_confirmation": False,
        "ros_gazebo_nav2_started": False,
        "failure_layer": None,
        "failure_reason": None,
    }
    display_available = bool(os.environ.get("DISPLAY"))
    if args.gui and not display_available:
        result["gui_request_note"] = "GUI requested but DISPLAY is not set; GUI visual confirmation is unavailable."

    candidate = candidate_from_report(out, object_id, args.approach_candidate_id)
    if not candidate:
        result.update({"failure_layer": "approach_planning", "failure_reason": "requested approach candidate not found in task14c report"})
        write_json(run_dir / f"runtime_probe_result_{object_id}.json", result)
        write_md(run_dir / f"runtime_probe_result_{object_id}.md", result)
        update_runtime_collection(out, result)
        update_summary(out)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    route_dir = args.stage_output_dir / "routes/room_routes/floor_2_room11_to_room14"
    base_route_path = route_dir / "executable_route_waypoints_v0_1.json"
    base_route = read_json(base_route_path)
    extended = append_approach_waypoint(base_route, candidate, object_id, args.query)
    extended_route_path = run_dir / f"extended_executable_route_{object_id}_{args.approach_candidate_id}.json"
    write_json(extended_route_path, extended)
    shutil.copyfile(extended_route_path, out / "sent_controller_paths" / f"sent_controller_path_{object_id}.json")

    marker_manifest = read_json(out / "rviz_marker_manifest.json", {})
    marker_manifest.update(
        {
            "marker_publication_attempted": False,
            "gui_requested": bool(args.gui),
            "display_available": display_available,
            "gui_visual_confirmation": False,
            "runtime_probe_extended_route": rel(extended_route_path),
        }
    )
    write_json(out / "rviz_marker_manifest.json", marker_manifest)

    ok, import_error = import_rclpy_status()
    if not ok:
        result.update(
            {
                "failure_layer": "runtime_dependency",
                "failure_reason": f"rclpy unavailable under /usr/bin/python3: {import_error}",
                "extended_route": rel(extended_route_path),
            }
        )
        write_json(run_dir / f"runtime_probe_result_{object_id}.json", result)
        write_md(run_dir / f"runtime_probe_result_{object_id}.md", result)
        update_runtime_collection(out, result)
        update_summary(out)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 3

    if args.skip_route_execution:
        result.update(
            {
                "failure_layer": "runtime_execution",
                "failure_reason": "route execution skipped by flag after extended route generation",
                "extended_route": rel(extended_route_path),
            }
        )
        write_json(run_dir / f"runtime_probe_result_{object_id}.json", result)
        write_md(run_dir / f"runtime_probe_result_{object_id}.md", result)
        update_runtime_collection(out, result)
        update_summary(out)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 4

    profile_name = "floor_2_nav2_task12_controller_robust" if args.controller_profile == "task12_robust" else "floor_2_nav2"
    runtime_profile = args.stage_output_dir / "runtime/profiles" / profile_name / "runtime_profile.json"
    map_yaml = args.stage_output_dir / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
    wall_json = out / "wall_crossing_validation" / f"wall_crossing_{object_id}.json"
    route_result = run_dir / f"route_execution_result_{object_id}.json"
    route_md = run_dir / f"route_execution_result_{object_id}.md"
    trajectory_json = out / "executed_trajectories" / f"executed_trajectory_{object_id}.json"
    latest_slice = run_dir / f"latest_follow_path_slice_{object_id}.json"

    cmd = [
        "/usr/bin/python3",
        str(ROOT / "tools/stage1_runtime/run_scene_route.py"),
        "--scene-id",
        SCENE_ID,
        "--floor-id",
        args.floor_id,
        "--runtime-profile",
        str(runtime_profile),
        "--stage-output-dir",
        str(args.stage_output_dir),
        "--waypoints-json",
        str(extended_route_path),
        "--expected-room-chain",
        EXPECTED_ROOM_CHAIN,
        "--expected-gateway-sequence",
        EXPECTED_GATEWAY_SEQUENCE,
        "--allow-non-scene-truth",
        "--reset-to-route-start",
        "--from-start",
        "--robot-model-name",
        "00843_floor2_turtlebot3",
        "--execution-strategy",
        args.execution_strategy,
        "--split-at-through-room-anchors",
        "--validate-wall-crossing",
        "--wall-crossing-map-yaml",
        str(map_yaml),
        "--wall-crossing-output-json",
        str(wall_json),
        "--current-pose-gateway-handoff",
        "--handoff-target-distance-m",
        "1.0",
        "--handoff-target-waypoint-index",
        "51",
        "--output-json",
        str(route_result),
        "--output-md",
        str(route_md),
        "--trajectory-output-json",
        str(trajectory_json),
        "--latest-slice-output-json",
        str(latest_slice),
    ]
    result["attempted"] = True
    result["ros_gazebo_nav2_started"] = True
    result["extended_route"] = rel(extended_route_path)
    result["runtime_command"] = cmd
    proc = subprocess.run(cmd, text=True)
    route_payload = read_json(route_result, {}) if route_result.exists() else {}
    trajectory_payload = read_json(trajectory_json, {}) if trajectory_json.exists() else {}
    samples = trajectory_payload.get("samples") or route_payload.get("trajectory_samples") or []
    final_pose = samples[-1] if samples else route_payload.get("final_pose")
    approach_xy = candidate.get("world_xy")
    approach_dist = None
    if final_pose and approach_xy:
        approach_dist = math.hypot(float(final_pose["x"]) - float(approach_xy[0]), float(final_pose["y"]) - float(approach_xy[1]))
    approach_reached = bool(approach_dist is not None and approach_dist <= 0.35)
    target_room_arrival = bool(route_payload.get("terminal_reached") or route_payload.get("clean_runtime_success"))
    result.update(
        {
            "runtime_returncode": proc.returncode,
            "runtime_result": "succeeded" if proc.returncode == 0 and approach_reached else "failed",
            "target_room_arrival": target_room_arrival,
            "approach_pose_reached": approach_reached,
            "approach_distance_m": round(approach_dist, 6) if approach_dist is not None else None,
            "follow_path_used": bool(route_payload.get("follow_path_used") or route_payload.get("split_follow_path_used")),
            "fallback_used": bool(route_payload.get("sparse_fallback_used")),
            "wall_crossing_validation_run": wall_json.exists(),
            "wall_crossing_validation": rel(wall_json),
            "route_result": rel(route_result),
            "trajectory": rel(trajectory_json),
            "final_pose": final_pose,
        }
    )
    if result["runtime_result"] != "succeeded":
        result["failure_layer"] = route_payload.get("failure_layer") or "runtime_execution"
        result["failure_reason"] = (
            route_payload.get("failure_reason")
            or ("approach pose not reached" if target_room_arrival else "runtime route command failed before target room/object approach success")
        )
    write_json(run_dir / f"runtime_probe_result_{object_id}.json", result)
    write_md(run_dir / f"runtime_probe_result_{object_id}.md", result)
    update_runtime_collection(out, result)
    update_summary(out)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["runtime_result"] == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
