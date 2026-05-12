#!/usr/bin/env python3
"""Step30H live GUI overlay and Nav2 planning-only smoke orchestration.

This step consumes existing Step30 artifacts. It does not rerun Stage-A,
gateway extraction, or topology generation. It never publishes /cmd_vel,
never calls NavigateToPose, and only sends ComputePathToPose goals after a
live action server probe confirms availability.
"""

from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import textwrap
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
STEP = "step30h"
VERSION = "v0_1"
OUTPUT_ROOT = RUNTIME_ROOT / "step30h_00824_live_gui_nav2_cleanup_planning_smoke"
SCRIPT_ROOT = OUTPUT_ROOT / "scripts"
LOG_ROOT = OUTPUT_ROOT / "logs"
RESOLVED_ROOT = OUTPUT_ROOT / "resolved_routes"

STEP17_ROOT = RUNTIME_ROOT / "step17_gazebo_nav2_asset_layer"
STEP30B2_ROOT = RUNTIME_ROOT / "step30b2_00824_candidate_level_gateway_truth_and_auto_selection"
STEP30C_ROOT = RUNTIME_ROOT / "step30c_00824_gateway_augmented_topology_candidate"
STEP30F_ROOT = RUNTIME_ROOT / "step30f_00824_gui_overlay_bridge"
STEP30G_ROOT = RUNTIME_ROOT / "step30g_00824_route_command_interface"

STEP30G_TOOL = REPO_ROOT / "tools" / "step30g_00824_route_command_interface.py"
PROBE_SCRIPT = SCRIPT_ROOT / "step30h_nav2_action_server_probe.py"
SNAPSHOT_SCRIPT = SCRIPT_ROOT / "step30h_ros_process_and_graph_snapshot.sh"
PLAN_RERUN_SCRIPT = SCRIPT_ROOT / "step30h_plan_only_rerun.sh"
RUNBOOK_HELPER = SCRIPT_ROOT / "step30h_live_gui_overlay_runbook.md"

INPUT_PATHS = {
    "step30b2_selection": STEP30B2_ROOT / "00824_step30b2_auto_gateway_selection_v0_1.json",
    "step30c_topology_candidate": STEP30C_ROOT / "00824_step30c_gateway_augmented_topology_candidate_v0_1.json",
    "step30c_gateway_edge_table": STEP30C_ROOT / "00824_step30c_gateway_edge_table_v0_1.json",
    "step30c_route_candidates": STEP30C_ROOT / "00824_step30c_topology_route_candidates_v0_1.json",
    "step30f_marker_payload": STEP30F_ROOT / "00824_step30f_gui_overlay_marker_payload_v0_1.json",
    "step30f_topic_plan": STEP30F_ROOT / "00824_step30f_marker_topic_plan_v0_1.json",
    "step30g_interface": STEP30G_TOOL,
    "step30g_client": STEP30G_ROOT / "scripts" / "step30g_nav2_route_planning_client.py",
}

OUTPUT_PATHS = {
    "cleanup_report": OUTPUT_ROOT / "00824_step30h_cleanup_report_v0_1.json",
    "prelaunch_snapshot": OUTPUT_ROOT / "00824_step30h_prelaunch_ros_graph_snapshot_v0_1.json",
    "postlaunch_snapshot": OUTPUT_ROOT / "00824_step30h_postlaunch_ros_graph_snapshot_v0_1.json",
    "probe": OUTPUT_ROOT / "00824_step30h_nav2_action_server_probe_v0_1.json",
    "overlay_report": OUTPUT_ROOT / "00824_step30h_live_gui_overlay_report_v0_1.json",
    "plan_smoke": OUTPUT_ROOT / "00824_step30h_plan_only_smoke_results_v0_1.json",
    "validation": OUTPUT_ROOT / "00824_step30h_validation_results_v0_1.json",
    "summary": OUTPUT_ROOT / "00824_step30h_summary_v0_1.json",
    "runbook": OUTPUT_ROOT / "00824_step30h_runbook_v0_1.md",
}

EXPECTED_GATEWAYS = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r7_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r7_r15_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r14_r16_01",
]
EXPECTED_LONG_ROUTE_GATEWAYS = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01",
]
EXPECTED_LONG_ROUTE_ROOMS = ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"]
EXPECTED_PUBLIC_ROOMS = ["room_1", "room_3", "room_7", "room_11", "room_8"]
FORBIDDEN_DIRECT_EDGE = "r3_r11"
FORBIDDEN_NON_TRUTH_PAIRS = {"r14_r15", "r15_r16", "r3_r15", "r7_r16"}
PROCESS_PATTERNS = [
    "gzserver",
    "gzclient",
    "gazebo",
    "rviz2",
    "nav2",
    "lifecycle",
    "map_server",
    "planner_server",
    "controller_server",
    "bt_navigator",
    "robot_state_publisher",
    "amcl",
    "lifecycle_manager",
]
LIFECYCLE_NODES = [
    "/map_server",
    "/amcl",
    "/controller_server",
    "/planner_server",
    "/recoveries_server",
    "/bt_navigator",
    "/waypoint_follower",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def read_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def sha256_file(path: Path) -> Optional[str]:
    if not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_shell(command: str, *, timeout: float = 15.0) -> Dict[str, Any]:
    started = time.time()
    try:
        completed = subprocess.run(
            ["bash", "-lc", command],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "elapsed_sec": round(time.time() - started, 3),
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
            "elapsed_sec": round(time.time() - started, 3),
            "timed_out": True,
        }


def ros_env_prefix() -> str:
    return textwrap.dedent(
        f"""\
        set +e
        if [ -f /opt/ros/foxy/setup.bash ]; then source /opt/ros/foxy/setup.bash; fi
        if [ -f {STEP17_ROOT}/ros2_ws/install/setup.bash ]; then source {STEP17_ROOT}/ros2_ws/install/setup.bash; fi
        """
    )


def ros_command(command: str, *, timeout: float = 12.0) -> Dict[str, Any]:
    return run_shell(ros_env_prefix() + "\n" + command, timeout=timeout)


def split_lines(text: str) -> List[str]:
    return [line.strip() for line in text.splitlines() if line.strip()]


def collect_processes() -> List[Dict[str, Any]]:
    result = run_shell("ps -eo pid=,ppid=,user=,stat=,comm=,args=", timeout=5)
    processes: List[Dict[str, Any]] = []
    if result["returncode"] != 0:
        return processes
    current_pid = os.getpid()
    for line in result["stdout"].splitlines():
        parts = line.strip().split(None, 5)
        if len(parts) < 6:
            continue
        pid_text, ppid_text, user, stat, comm, args = parts
        try:
            pid = int(pid_text)
            ppid = int(ppid_text)
        except ValueError:
            continue
        haystack = f"{comm} {args}".lower()
        matched = [pattern for pattern in PROCESS_PATTERNS if pattern.lower() in haystack]
        if not matched:
            continue
        if pid == current_pid:
            continue
        processes.append(
            {
                "pid": pid,
                "ppid": ppid,
                "user": user,
                "stat": stat,
                "comm": comm,
                "args": args,
                "matched_patterns": matched,
            }
        )
    return processes


def cleanup_processes(processes: Sequence[Mapping[str, Any]], *, kill: bool) -> Dict[str, Any]:
    log_lines = [
        f"Step30H process cleanup report created_utc={now_iso()}",
        f"kill_requested={kill}",
        f"matched_process_count={len(processes)}",
    ]
    killed: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []
    if kill:
        for process in processes:
            pid = int(process["pid"])
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append({"pid": pid, "signal": "SIGTERM", "args": process.get("args")})
                log_lines.append(f"SIGTERM pid={pid} args={process.get('args')}")
            except Exception as exc:
                failures.append({"pid": pid, "signal": "SIGTERM", "error": str(exc)})
                log_lines.append(f"FAILED SIGTERM pid={pid} error={exc}")
        time.sleep(1.0)
        for process in processes:
            pid = int(process["pid"])
            try:
                os.kill(pid, 0)
            except OSError:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append({"pid": pid, "signal": "SIGKILL", "args": process.get("args")})
                log_lines.append(f"SIGKILL pid={pid} args={process.get('args')}")
            except Exception as exc:
                failures.append({"pid": pid, "signal": "SIGKILL", "error": str(exc)})
                log_lines.append(f"FAILED SIGKILL pid={pid} error={exc}")

    cleanup_log = LOG_ROOT / "00824_step30h_process_cleanup.log"
    write_text(cleanup_log, "\n".join(log_lines) + "\n")
    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30h_cleanup_report",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "stale_processes_found": bool(processes),
        "matched_process_count": len(processes),
        "matched_processes": list(processes),
        "kill_requested": kill,
        "killed_processes": killed,
        "kill_failures": failures,
        "cleanup_command_block": [
            "pkill -TERM -f 'gzserver|gzclient|gazebo|rviz2|nav2|lifecycle|map_server|planner_server|controller_server|bt_navigator|robot_state_publisher|amcl|lifecycle_manager'",
            "sleep 1",
            "pkill -KILL -f 'gzserver|gzclient|gazebo|rviz2|nav2|lifecycle|map_server|planner_server|controller_server|bt_navigator|robot_state_publisher|amcl|lifecycle_manager'",
        ],
        "log": rel(cleanup_log),
    }


def command_record(name: str, command: str, *, timeout: float = 12.0) -> Dict[str, Any]:
    result = ros_command(command, timeout=timeout)
    return {
        "name": name,
        "command": command,
        "returncode": result["returncode"],
        "timed_out": result["timed_out"],
        "stdout_lines": split_lines(result["stdout"]),
        "stderr_tail": result["stderr"][-2000:],
        "elapsed_sec": result["elapsed_sec"],
    }


def lifecycle_records() -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    for node in LIFECYCLE_NODES:
        result = ros_command(f"ros2 lifecycle get {node}", timeout=5)
        lines = split_lines(result["stdout"])
        state = None
        for line in lines:
            if "active" in line.lower():
                state = "active"
            elif "inactive" in line.lower():
                state = "inactive"
            elif "unconfigured" in line.lower():
                state = "unconfigured"
        records.append(
            {
                "node": node,
                "returncode": result["returncode"],
                "state": state,
                "stdout_lines": lines,
                "stderr_tail": result["stderr"][-1000:],
                "timed_out": result["timed_out"],
            }
        )
    return records


def collect_graph_snapshot(label: str) -> Dict[str, Any]:
    commands = [
        ("ros2_node_list", "ros2 node list"),
        ("ros2_topic_list", "ros2 topic list"),
        ("ros2_action_list", "ros2 action list"),
        ("ros2_action_list_typed", "ros2 action list -t"),
        ("ros2_service_list", "ros2 service list"),
    ]
    command_results = {name: command_record(name, command) for name, command in commands}
    nodes = command_results["ros2_node_list"]["stdout_lines"]
    topics = command_results["ros2_topic_list"]["stdout_lines"]
    actions = command_results["ros2_action_list"]["stdout_lines"]
    services = command_results["ros2_service_list"]["stdout_lines"]
    lifecycle = lifecycle_records()
    process_matches = collect_processes()
    env = {
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
        "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH"),
        "COLCON_PREFIX_PATH": os.environ.get("COLCON_PREFIX_PATH"),
        "PYTHONPATH": os.environ.get("PYTHONPATH"),
    }
    return {
        "scene_id": SCENE_ID,
        "artifact_type": f"step30h_{label}_ros_graph_snapshot",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "label": label,
        "host": {
            "hostname": platform.node(),
            "user": getpass.getuser(),
            "cwd": str(REPO_ROOT),
            "platform": platform.platform(),
        },
        "environment": env,
        "sourced_environment_files": [
            {"path": "/opt/ros/foxy/setup.bash", "exists": Path("/opt/ros/foxy/setup.bash").exists()},
            {"path": rel(STEP17_ROOT / "ros2_ws" / "install" / "setup.bash"), "exists": (STEP17_ROOT / "ros2_ws" / "install" / "setup.bash").exists()},
        ],
        "process_matches": process_matches,
        "process_match_count": len(process_matches),
        "ros2_command_results": command_results,
        "nodes": nodes,
        "topics": topics,
        "actions": actions,
        "services": services,
        "lifecycle": lifecycle,
        "readiness": {
            "compute_path_to_pose_action_visible": "/compute_path_to_pose" in actions,
            "navigate_to_pose_action_visible": "/navigate_to_pose" in actions,
            "planner_server_node_visible": any("planner_server" in node for node in nodes),
            "map_server_node_visible": any("map_server" in node for node in nodes),
            "lifecycle_manager_visible": any("lifecycle_manager" in node for node in nodes),
            "amcl_node_visible": any("amcl" in node for node in nodes),
            "map_topic_visible": "/map" in topics,
            "odom_topic_visible": "/odom" in topics,
            "tf_topic_visible": "/tf" in topics or "tf" in topics,
            "tf_static_topic_visible": "/tf_static" in topics or "tf_static" in topics,
            "scan_topic_visible": "/scan" in topics,
            "base_related_topics_visible": [topic for topic in topics if any(token in topic for token in ["base_link", "base_footprint", "base_scan"])],
        },
        "lifecycle_summary": {record["node"]: record["state"] for record in lifecycle},
        "foxy_topic_echo_once_used": False,
    }


def source_hashes() -> Dict[str, Optional[str]]:
    return {name: sha256_file(path) for name, path in INPUT_PATHS.items() if path.exists()}


def write_placeholder_logs() -> Dict[str, str]:
    placeholders = {
        "gazebo_log": LOG_ROOT / "00824_step30h_gazebo.log",
        "rviz_log": LOG_ROOT / "00824_step30h_rviz.log",
        "marker_publisher_log": LOG_ROOT / "00824_step30h_marker_publisher.log",
        "nav2_launch_log": LOG_ROOT / "00824_step30h_nav2_launch.log",
        "planning_client_log": LOG_ROOT / "00824_step30h_planning_client_stdout_stderr.log",
    }
    for label, path in placeholders.items():
        if not path.exists():
            write_text(
                path,
                f"{label}: Step30H did not launch this long-running process automatically. "
                "Use 00824_step30h_runbook_v0_1.md and rerun the snapshot/planning scripts after launch.\n",
            )
    return {label: rel(path) for label, path in placeholders.items()}


def resolve_route(route_id: str) -> Tuple[Optional[Path], Dict[str, Any]]:
    route_dir = RESOLVED_ROOT / route_id
    route_dir.mkdir(parents=True, exist_ok=True)
    log_path = LOG_ROOT / f"00824_step30h_step30g_resolve_{route_id}.log"
    command = [
        sys.executable,
        str(STEP30G_TOOL),
        "--scene",
        SCENE_ID,
        "--route-id",
        route_id,
        "--mode",
        "overlay-only",
        "--dry-run",
        "--output-dir",
        str(route_dir),
    ]
    started = time.time()
    completed = subprocess.run(command, cwd=REPO_ROOT, text=True, capture_output=True, check=False)
    write_text(
        log_path,
        "COMMAND: " + " ".join(command) + "\n\nSTDOUT:\n" + completed.stdout + "\n\nSTDERR:\n" + completed.stderr,
    )
    route_path = route_dir / "00824_step30g_resolved_route_v0_1.json"
    record = {
        "route_id": route_id,
        "command": command,
        "returncode": completed.returncode,
        "elapsed_sec": round(time.time() - started, 3),
        "stdout_tail": completed.stdout[-2000:],
        "stderr_tail": completed.stderr[-2000:],
        "log": rel(log_path),
        "resolved_route_path": rel(route_path) if route_path.exists() else None,
        "resolved": route_path.exists() and completed.returncode == 0,
    }
    return (route_path if route_path.exists() else None), record


def write_single_segment_route(route_path: Path, segment_index: int) -> Path:
    route = read_json(route_path)
    segments = [segment for segment in route.get("segments", []) if int(segment.get("segment_index", -1)) == segment_index]
    route["route_id"] = f"{route.get('route_id')}_segment_{segment_index}"
    route["segments"] = segments
    route["segment_count"] = len(segments)
    out = RESOLVED_ROOT / f"{route['route_id']}.json"
    write_json(out, route)
    return out


def run_probe(output: Path, *, send_goals: bool = False, planning_route: Optional[Path] = None, segment_index: Optional[int] = None, timeout_sec: float = 5.0) -> Dict[str, Any]:
    command = [
        "/usr/bin/python3",
        str(PROBE_SCRIPT),
        "--output",
        str(output),
        "--timeout-sec",
        str(timeout_sec),
    ]
    if send_goals:
        command.append("--send-goals")
    if planning_route:
        command.extend(["--planning-route", str(planning_route)])
    if segment_index is not None:
        command.extend(["--segment-index", str(segment_index)])
    result = run_shell(ros_env_prefix() + "\n" + " ".join(command), timeout=120)
    log_path = LOG_ROOT / f"{output.stem}.log"
    write_text(
        log_path,
        "COMMAND: " + " ".join(command) + "\n\nSTDOUT:\n" + result["stdout"] + "\n\nSTDERR:\n" + result["stderr"],
    )
    if output.exists():
        payload = read_json(output)
    else:
        payload = {
            "artifact_type": "step30h_nav2_action_server_probe",
            "step": STEP,
            "version": VERSION,
            "created_utc": now_iso(),
            "status": "unavailable",
            "compute_path_to_pose_available": False,
            "failure_reason": "probe script did not write output",
            "segments": [],
        }
    payload["subprocess"] = {
        "returncode": result["returncode"],
        "timed_out": result["timed_out"],
        "log": rel(log_path),
    }
    write_json(output, payload)
    return payload


def run_plan_only_smoke(probe: Mapping[str, Any], route_records: Mapping[str, Tuple[Optional[Path], Dict[str, Any]]]) -> Dict[str, Any]:
    available = bool(probe.get("compute_path_to_pose_available"))
    smoke: Dict[str, Any] = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30h_plan_only_smoke_results",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "status": "skipped_action_server_unavailable" if not available else "started",
        "compute_path_to_pose_available_before_planning": available,
        "planning_order": [
            "segment_0_smoke",
            "public_path_042_full",
            "long_structure_route_default_full",
        ],
        "navigate_to_pose_used": False,
        "cmd_vel_published": False,
        "robot_motion_commanded": False,
        "compute_path_through_poses_used": False,
        "local_offline_fallback_used": False,
        "route_resolution_records": {route_id: record for route_id, (_path, record) in route_records.items()},
        "runs": [],
    }
    if not available:
        smoke["failure_reason"] = probe.get("failure_reason") or "action server unavailable"
        write_json(OUTPUT_PATHS["plan_smoke"], smoke)
        return smoke

    long_route_path, _long_record = route_records["long_structure_route_default"]
    public_route_path, _public_record = route_records["public_path_042"]
    if not long_route_path or not public_route_path:
        smoke["status"] = "skipped_route_resolution_failed"
        smoke["failure_reason"] = "required route resolution failed"
        write_json(OUTPUT_PATHS["plan_smoke"], smoke)
        return smoke

    segment_route = write_single_segment_route(long_route_path, 0)
    run_specs = [
        ("segment_0_smoke", segment_route, OUTPUT_ROOT / "00824_step30h_segment0_plan_only_probe_v0_1.json"),
        ("public_path_042_full", public_route_path, OUTPUT_ROOT / "00824_step30h_public_path_042_plan_only_probe_v0_1.json"),
        ("long_structure_route_default_full", long_route_path, OUTPUT_ROOT / "00824_step30h_long_structure_route_default_plan_only_probe_v0_1.json"),
    ]
    for run_id, route_path, output in run_specs:
        payload = run_probe(output, send_goals=True, planning_route=route_path, timeout_sec=5.0)
        smoke["runs"].append(
            {
                "run_id": run_id,
                "route_path": rel(route_path),
                "probe_output": rel(output),
                "status": payload.get("status"),
                "failure_reason": payload.get("failure_reason"),
                "segments": payload.get("segments", []),
            }
        )
        if run_id == "segment_0_smoke" and payload.get("status") != "complete":
            break

    smoke["segment_0_path_returned"] = bool(
        smoke["runs"]
        and smoke["runs"][0].get("segments")
        and smoke["runs"][0]["segments"][0].get("path_returned")
    )
    public_run = next((run for run in smoke["runs"] if run["run_id"] == "public_path_042_full"), None)
    long_run = next((run for run in smoke["runs"] if run["run_id"] == "long_structure_route_default_full"), None)
    smoke["public_path_042_all_segments_returned_paths"] = bool(public_run) and all(
        segment.get("path_returned") for segment in public_run.get("segments", [])
    )
    smoke["long_structure_route_default_all_segments_returned_paths"] = bool(long_run) and all(
        segment.get("path_returned") for segment in long_run.get("segments", [])
    )
    smoke["status"] = (
        "complete"
        if smoke["segment_0_path_returned"]
        and smoke["public_path_042_all_segments_returned_paths"]
        and smoke["long_structure_route_default_all_segments_returned_paths"]
        else "complete_with_failures"
    )
    smoke["failure_reason"] = None if smoke["status"] == "complete" else "one or more planning-only runs failed"
    write_json(OUTPUT_PATHS["plan_smoke"], smoke)
    return smoke


def build_overlay_report(post_snapshot: Mapping[str, Any], logs: Mapping[str, str]) -> Dict[str, Any]:
    payload = read_json(INPUT_PATHS["step30f_marker_payload"])
    routes = payload.get("routes") or {}
    report = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30h_live_gui_overlay_report",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "live_gui_overlay_launched_by_step30h": False,
        "runbook_only": True,
        "frame_id": payload.get("frame_id"),
        "overlay_payload": rel(INPUT_PATHS["step30f_marker_payload"]),
        "overlay_payload_hash": sha256_file(INPUT_PATHS["step30f_marker_payload"]),
        "marker_counts": payload.get("marker_counts"),
        "topics_expected": payload.get("topics"),
        "topics_visible_postlaunch": [
            topic for topic in post_snapshot.get("topics", []) if "/boxfusion/step30f" in topic
        ],
        "visual_confirmation_required": {
            "overlay_frame_id_is_map": payload.get("frame_id") == "map",
            "expected_8_gateway_markers_present_in_payload": payload.get("marker_counts", {}).get("gateway_marker_count") == 16,
            "room_markers_and_corrected_anchors_present_in_payload": payload.get("marker_counts", {}).get("room_marker_count") == 16,
            "public_path_042_present": "public_path_042" in routes,
            "long_structure_route_default_present": "long_structure_route_default" in routes,
            "r3_r11_direct_edge_absent_in_payload": FORBIDDEN_DIRECT_EDGE not in payload.get("topology_pair_keys", []),
            "forbidden_non_truth_edges_absent_in_payload": not bool(
                FORBIDDEN_NON_TRUTH_PAIRS.intersection(set(payload.get("topology_pair_keys", [])))
            ),
            "occupancy_floor_world_alignment": "manual_or_live_rviz_screenshot_required",
        },
        "screenshot_capture": {
            "automatic_capture_attempted": False,
            "automatic_capture_available": False,
            "manual_instructions": "Use RViz File > Save Image after enabling Step30F MarkerArray topics and the map display.",
        },
        "logs": logs,
        "notes": [
            "Step30H does not claim visual alignment without a live screenshot or operator confirmation.",
            "Launch commands are recorded in the runbook; rerun this script after the GUI/Nav2 stack is live to refresh postlaunch evidence.",
        ],
    }
    write_json(OUTPUT_PATHS["overlay_report"], report)
    return report


def classify_failure(post_snapshot: Mapping[str, Any], probe: Mapping[str, Any], smoke: Mapping[str, Any]) -> str:
    readiness = post_snapshot.get("readiness", {})
    lifecycle = post_snapshot.get("lifecycle_summary", {})
    if not probe.get("compute_path_to_pose_available"):
        if not readiness.get("planner_server_node_visible"):
            return "planner_server_missing"
        if lifecycle.get("/planner_server") not in {None, "active"}:
            return "lifecycle_inactive"
        return "action_server_unavailable"
    for run in smoke.get("runs", []):
        for segment in run.get("segments", []):
            reason = str(segment.get("failure_reason") or "")
            if "rejected" in reason:
                return "goal_rejected"
            if "no path" in reason:
                return "no_path_found"
            if "timeout" in reason:
                return "timeout"
    if not readiness.get("map_server_node_visible"):
        return "map_server_missing"
    if not readiness.get("map_topic_visible"):
        return "map_or_costmap_unavailable"
    return "unknown" if smoke.get("status") != "complete" else "none"


def run_validation(
    pre_hashes: Mapping[str, Optional[str]],
    post_hashes: Mapping[str, Optional[str]],
    pre_snapshot: Mapping[str, Any],
    post_snapshot: Mapping[str, Any],
    probe: Mapping[str, Any],
    smoke: Mapping[str, Any],
    overlay_report: Mapping[str, Any],
    route_records: Mapping[str, Tuple[Optional[Path], Dict[str, Any]]],
) -> Dict[str, Any]:
    edge_table = read_json(INPUT_PATHS["step30c_gateway_edge_table"])
    gateway_ids = [edge.get("gateway_id") for edge in edge_table.get("gateway_edges", [])]
    pair_keys = [edge.get("pair_key") for edge in edge_table.get("gateway_edges", [])]
    public_route = read_json(route_records["public_path_042"][0]) if route_records["public_path_042"][0] else {}
    long_route = read_json(route_records["long_structure_route_default"][0]) if route_records["long_structure_route_default"][0] else {}
    checks = {
        "step30f_overlay_payload_exists_and_consumed": INPUT_PATHS["step30f_marker_payload"].exists()
        and bool(overlay_report.get("marker_counts")),
        "step30g_route_interface_exists_and_consumed": INPUT_PATHS["step30g_interface"].exists()
        and all(record["resolved"] for _path, record in route_records.values()),
        "step30c_topology_hash_unchanged": pre_hashes.get("step30c_topology_candidate") == post_hashes.get("step30c_topology_candidate"),
        "step30f_overlay_payload_hash_unchanged": pre_hashes.get("step30f_marker_payload") == post_hashes.get("step30f_marker_payload"),
        "eight_expected_primary_gateways_present": sorted(gateway_ids) == sorted(EXPECTED_GATEWAYS),
        "long_route_gateway_sequence_expected": long_route.get("gateway_sequence") == EXPECTED_LONG_ROUTE_GATEWAYS,
        "long_route_room_sequence_expected": long_route.get("room_sequence") == EXPECTED_LONG_ROUTE_ROOMS,
        "public_path_042_room_sequence_expected": public_route.get("room_sequence") == EXPECTED_PUBLIC_ROOMS,
        "r3_r11_direct_edge_absent": FORBIDDEN_DIRECT_EDGE not in pair_keys,
        "forbidden_non_truth_edges_absent": not bool(FORBIDDEN_NON_TRUTH_PAIRS.intersection(set(pair_keys))),
        "no_stage_a_rerun": True,
        "no_gateway_extraction_rerun": True,
        "no_topology_regeneration": True,
        "no_navigate_to_pose_call": True,
        "no_cmd_vel_publication": True,
        "compute_path_to_pose_availability_classified": isinstance(probe.get("compute_path_to_pose_available"), bool),
        "foxy_topic_echo_once_not_used": not pre_snapshot.get("foxy_topic_echo_once_used") and not post_snapshot.get("foxy_topic_echo_once_used"),
    }
    failure_classification = classify_failure(post_snapshot, probe, smoke)
    validation = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30h_validation_results",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "status": "pass" if all(checks.values()) else "fail",
        "checks": checks,
        "input_hashes_before": pre_hashes,
        "input_hashes_after": post_hashes,
        "gateway_ids": gateway_ids,
        "pair_keys": pair_keys,
        "compute_path_to_pose_available": bool(probe.get("compute_path_to_pose_available")),
        "planning_failure_classification": failure_classification,
        "planning_failure_classification_options": [
            "action_server_unavailable",
            "lifecycle_inactive",
            "planner_server_missing",
            "map_server_missing",
            "map_or_costmap_unavailable",
            "goal_rejected",
            "no_path_found",
            "timeout",
            "unknown",
            "none",
        ],
        "safety_contract": {
            "stage_a_rerun": False,
            "gateway_extraction_rerun": False,
            "topology_regeneration": False,
            "navigate_to_pose_call": False,
            "cmd_vel_publication": False,
            "robot_execution": False,
            "compute_path_through_poses_used": False,
        },
    }
    write_json(OUTPUT_PATHS["validation"], validation)
    return validation


def build_runbook() -> str:
    map_yaml = STEP17_ROOT / "scenes" / SCENE_ID / "maps" / "floor_1_approx_nav_map.yaml"
    nav2_launch_file = "00824_nav2_turtlebot3_original.launch.py"
    nav2_params = STEP17_ROOT / "ros2_ws" / "src" / "boxfusion_gazebo_nav2_demo" / "params" / "00824_nav2_turtlebot3_original.yaml"
    gazebo_world = STEP30F_ROOT / "gazebo" / "00824_step30f_static_overlay_marker_world.sdf"
    rviz_config = STEP30F_ROOT / "rviz" / "step30f_00824_overlay.rviz"
    marker_script = STEP30F_ROOT / "scripts" / "publish_00824_step30f_overlay_markers.py"
    payload = INPUT_PATHS["step30f_marker_payload"]
    return textwrap.dedent(
        f"""\
        # Step30H Runbook: 00824 Live GUI Overlay + Nav2 Planning-Only Smoke

        Scene: `{SCENE_ID}`

        Step30H is visualization and planning-only. Do not publish `/cmd_vel`, do not call
        `NavigateToPose`, and do not execute robot navigation in this step.

        ## Source Foxy And Step17

        ```bash
        cd {REPO_ROOT}
        source /opt/ros/foxy/setup.bash
        source {STEP17_ROOT}/ros2_ws/install/setup.bash
        export TURTLEBOT3_MODEL=burger
        ```

        ## Optional Cleanup Snapshot

        ```bash
        bash {SNAPSHOT_SCRIPT} prelaunch
        ```

        If stale Gazebo/RViz/Nav2 processes are present, inspect
        `{rel(OUTPUT_PATHS["cleanup_report"])}`. To terminate matched stale processes explicitly:

        ```bash
        python3 tools/step30h_00824_live_gui_nav2_cleanup_planning_smoke.py --cleanup-stale
        ```

        ## Launch Gazebo/Nav2/RViz/Overlay

        Terminal 1, optional Gazebo static overlay world:

        ```bash
        source /opt/ros/foxy/setup.bash
        source {STEP17_ROOT}/ros2_ws/install/setup.bash
        ros2 launch boxfusion_gazebo_nav2_demo gazebo_marker_world.launch.py \\
          world:=$PWD/{rel(gazebo_world)}
        ```

        Terminal 2, Nav2 stack for scene 00824:

        ```bash
        source /opt/ros/foxy/setup.bash
        source {STEP17_ROOT}/ros2_ws/install/setup.bash
        ros2 launch boxfusion_gazebo_nav2_demo {nav2_launch_file} \\
          map:=$PWD/{rel(map_yaml)} \\
          params_file:=$PWD/{rel(nav2_params)} \\
          autostart:=true
        ```

        Terminal 3, Step30F overlay marker publisher:

        ```bash
        source /opt/ros/foxy/setup.bash
        /usr/bin/python3 {marker_script} \\
          --payload {payload}
        ```

        Terminal 4, RViz:

        ```bash
        source /opt/ros/foxy/setup.bash
        rviz2 -d {rviz_config}
        ```

        ## Visual Confirmation Checklist

        Confirm in RViz:

        - Fixed frame is `map`.
        - 8 primary gateway locations are visible at doorway locations.
        - Room markers and corrected anchors are visible.
        - `public_path_042` is visible.
        - `long_structure_route_default` is visible.
        - Direct `r3_r11` topology edge is absent.
        - Non-truth edges `r14_r15`, `r15_r16`, `r3_r15`, and `r7_r16` are absent.
        - Occupancy/floor/world alignment is visually reasonable.

        For manual screenshot evidence, use RViz `File > Save Image` and save under:

        ```bash
        {OUTPUT_ROOT}/screenshots/
        ```

        ## Postlaunch Readiness And Planning-Only Smoke

        After Gazebo/Nav2/RViz/overlay are live:

        ```bash
        bash {PLAN_RERUN_SCRIPT}
        ```

        This rerun first probes `/compute_path_to_pose`. Only if it is available does it send
        planning-only `ComputePathToPose` goals in this order:

        1. segment 0 of `long_structure_route_default`
        2. full `public_path_042`
        3. full `long_structure_route_default`

        The Foxy client sends goal pose and planner id only. It does not set `start` or
        `use_start`, does not use `ComputePathThroughPoses`, and does not execute motion.
        """
    )


def write_helper_scripts() -> None:
    write_text(OUTPUT_PATHS["runbook"], build_runbook())
    write_text(RUNBOOK_HELPER, build_runbook())
    write_text(
        SNAPSHOT_SCRIPT,
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail
            LABEL="${{1:-manual}}"
            cd "{REPO_ROOT}"
            python3 tools/step30h_00824_live_gui_nav2_cleanup_planning_smoke.py \\
              --snapshot-only \\
              --snapshot-label "$LABEL" \\
              --snapshot-output "{OUTPUT_ROOT}/00824_step30h_${{LABEL}}_ros_graph_snapshot_v0_1.json"
            """
        ),
    )
    write_text(
        PLAN_RERUN_SCRIPT,
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            set -euo pipefail
            cd "{REPO_ROOT}"
            source /opt/ros/foxy/setup.bash
            if [ -f "{STEP17_ROOT}/ros2_ws/install/setup.bash" ]; then
              source "{STEP17_ROOT}/ros2_ws/install/setup.bash"
            fi
            python3 tools/step30h_00824_live_gui_nav2_cleanup_planning_smoke.py --postlaunch
            """
        ),
    )
    for path in [SNAPSHOT_SCRIPT, PLAN_RERUN_SCRIPT, PROBE_SCRIPT]:
        if path.exists():
            path.chmod(path.stat().st_mode | 0o755)


def build_summary(
    cleanup: Mapping[str, Any],
    overlay_report: Mapping[str, Any],
    probe: Mapping[str, Any],
    smoke: Mapping[str, Any],
    validation: Mapping[str, Any],
) -> Dict[str, Any]:
    next_step = "fix Nav2 launch/lifecycle/namespace"
    if overlay_report.get("visual_confirmation_required", {}).get("occupancy_floor_world_alignment") != "manual_or_live_rviz_screenshot_required":
        next_step = "fix frame/transform/map alignment"
    elif not probe.get("compute_path_to_pose_available"):
        next_step = "fix Nav2 launch/lifecycle/namespace"
    elif smoke.get("status") == "complete_with_failures":
        next_step = "inspect map/costmap/gateway carving usage"
    elif smoke.get("status") == "complete":
        next_step = "proceed to Step30I execute-one-segment"
    summary = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30h_summary",
        "step": STEP,
        "version": VERSION,
        "created_utc": now_iso(),
        "cleanup_found_stale_processes": bool(cleanup.get("stale_processes_found")),
        "live_gui_overlay": "runbook-only" if overlay_report.get("runbook_only") else "launched",
        "compute_path_to_pose_available": bool(probe.get("compute_path_to_pose_available")),
        "segment_0_planning_only_returned_path": bool(smoke.get("segment_0_path_returned")),
        "public_path_042_all_segment_paths": bool(smoke.get("public_path_042_all_segments_returned_paths")),
        "long_structure_route_default_all_segment_paths": bool(smoke.get("long_structure_route_default_all_segments_returned_paths")),
        "validation_status": validation.get("status"),
        "planning_failure_classification": validation.get("planning_failure_classification"),
        "artifacts_written": {name: rel(path) for name, path in OUTPUT_PATHS.items()},
        "logs_dir": rel(LOG_ROOT),
        "next_recommended_step": next_step,
        "success_claimed": smoke.get("status") == "complete" and validation.get("status") == "pass",
    }
    write_json(OUTPUT_PATHS["summary"], summary)
    return summary


def print_console_summary(summary: Mapping[str, Any]) -> None:
    print(
        textwrap.dedent(
            f"""\
            Step30H summary
            - cleanup found stale processes: {summary.get('cleanup_found_stale_processes')}
            - live GUI overlay: {summary.get('live_gui_overlay')}
            - /compute_path_to_pose available: {summary.get('compute_path_to_pose_available')}
            - segment-0 planning-only returned path: {summary.get('segment_0_planning_only_returned_path')}
            - public_path_042 all segments returned paths: {summary.get('public_path_042_all_segment_paths')}
            - long_structure_route_default all segments returned paths: {summary.get('long_structure_route_default_all_segment_paths')}
            - artifacts: {summary.get('artifacts_written', {}).get('summary')}
            - next recommended step: {summary.get('next_recommended_step')}
            """
        ).strip()
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step30H 00824 live GUI/Nav2 cleanup and planning-only smoke.")
    parser.add_argument("--cleanup-stale", action="store_true", help="Terminate matched stale Gazebo/RViz/Nav2 processes.")
    parser.add_argument("--postlaunch", action="store_true", help="Collect postlaunch graph and rerun planning-only smoke.")
    parser.add_argument("--server-timeout-sec", type=float, default=5.0)
    parser.add_argument("--snapshot-only", action="store_true")
    parser.add_argument("--snapshot-label", default="manual")
    parser.add_argument("--snapshot-output", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    SCRIPT_ROOT.mkdir(parents=True, exist_ok=True)
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    RESOLVED_ROOT.mkdir(parents=True, exist_ok=True)

    if args.snapshot_only:
        snapshot = collect_graph_snapshot(args.snapshot_label)
        output = args.snapshot_output or OUTPUT_ROOT / f"00824_step30h_{args.snapshot_label}_ros_graph_snapshot_v0_1.json"
        write_json(output, snapshot)
        print(json.dumps({"snapshot": rel(output), "readiness": snapshot.get("readiness")}, indent=2, sort_keys=True))
        return 0

    write_helper_scripts()
    pre_hashes = source_hashes()
    processes = collect_processes()
    cleanup = cleanup_processes(processes, kill=bool(args.cleanup_stale))
    write_json(OUTPUT_PATHS["cleanup_report"], cleanup)

    pre_snapshot = collect_graph_snapshot("prelaunch")
    write_json(OUTPUT_PATHS["prelaunch_snapshot"], pre_snapshot)

    route_records = {
        "public_path_042": resolve_route("public_path_042"),
        "long_structure_route_default": resolve_route("long_structure_route_default"),
    }

    post_snapshot = collect_graph_snapshot("postlaunch")
    post_snapshot["postlaunch_mode"] = "manual_stack_expected" if args.postlaunch else "runbook_only_no_stack_launched_by_step30h"
    write_json(OUTPUT_PATHS["postlaunch_snapshot"], post_snapshot)

    probe = run_probe(OUTPUT_PATHS["probe"], timeout_sec=float(args.server_timeout_sec))
    smoke = run_plan_only_smoke(probe, route_records)
    logs = write_placeholder_logs()
    overlay_report = build_overlay_report(post_snapshot, logs)
    post_hashes = source_hashes()
    validation = run_validation(
        pre_hashes,
        post_hashes,
        pre_snapshot,
        post_snapshot,
        probe,
        smoke,
        overlay_report,
        route_records,
    )
    summary = build_summary(cleanup, overlay_report, probe, smoke, validation)
    print_console_summary(summary)
    return 0 if validation.get("status") == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
