#!/usr/bin/env python3
"""Run single-object object-navigation runtime validation."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
STAGE1_RUNTIME = ROOT / "tools/stage1_runtime"
if str(STAGE1_RUNTIME) not in sys.path:
    sys.path.insert(0, str(STAGE1_RUNTIME))

SCENE_ID = "00843-DYehNKdT76V"
LEGACY_EXPECTED_ROOM_CHAIN = ["room_11", "room_7", "room_13", "room_14"]
LEGACY_EXPECTED_GATEWAY_SEQUENCE = [
    "00843_floor2_gateway_005",
    "00843_floor2_gateway_004",
    "00843_floor2_gateway_006",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, default: Any | None = None) -> Any:
    if not path.exists():
        if default is not None:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except Exception:
        return str(path)


def canonical_object_id(value: str) -> str:
    return value if value.startswith("obj_") else f"obj_{value}"


def dist(a: dict[str, Any] | None, b: dict[str, Any] | None) -> float | None:
    if not a or not b:
        return None
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def angle_error(a: float, b: float) -> float:
    return abs(normalize_angle(float(b) - float(a)))


def round_float(value: Any, digits: int = 6) -> float | None:
    if value is None:
        return None
    try:
        return round(float(value), digits)
    except Exception:
        return None


def candidate_from_report(task14c_output_dir: Path, object_id: str, candidate_id: str) -> dict[str, Any] | None:
    report = read_json(task14c_output_dir / "approach_candidate_reports" / f"object_approach_report_{object_id}.json", {})
    candidate = report.get("recommended_candidate")
    if candidate and candidate.get("candidate_id") == candidate_id:
        return candidate
    for item in report.get("candidate_records", []):
        if item.get("candidate_id") == candidate_id:
            return item
    return None


def pose_from_candidate(candidate: dict[str, Any], object_id: str, query: str, *, yaw: float | None = None, source: str = "object_approach_pose") -> dict[str, Any]:
    x, y = candidate.get("world_xy") or [None, None]
    yaw_value = candidate.get("yaw") if yaw is None else yaw
    return {
        "x": float(x),
        "y": float(y),
        "yaw": float(0.0 if yaw_value is None else yaw_value),
        "source": source,
        "waypoint_index": 0,
        "target_waypoint_index": 0,
        "object_id": object_id,
        "object_query": query,
        "approach_candidate_id": candidate.get("candidate_id"),
        "floor_id": candidate.get("floor_id") or "floor_2",
        "room_id": candidate.get("room_id") or "room_14",
    }


def position_first_yaw(start_pose: dict[str, Any] | None, target: dict[str, Any]) -> tuple[float, str]:
    if start_pose and dist(start_pose, target) and dist(start_pose, target) > 0.05:
        return math.atan2(float(target["y"]) - float(start_pose["y"]), float(target["x"]) - float(start_pose["x"])), "path_heading_yaw_from_live_pose_to_approach_xy"
    if start_pose and start_pose.get("yaw") is not None:
        return float(start_pose["yaw"]), "current_yaw_terminal_orientation"
    return 0.0, "zero_yaw_fallback_terminal_orientation"


def target_yaw_from_pose(candidate: dict[str, Any], current_pose: dict[str, Any] | None) -> tuple[float, str, dict[str, Any] | None]:
    proxy = candidate.get("visible_proxy_xy")
    if current_pose and isinstance(proxy, list) and len(proxy) >= 2:
        target = {"x": float(proxy[0]), "y": float(proxy[1]), "source": "visible_proxy_xy"}
        yaw = math.atan2(target["y"] - float(current_pose["y"]), target["x"] - float(current_pose["x"]))
        return yaw, "visible_proxy_xy_from_committed_candidate_report", target
    yaw = float(candidate.get("yaw") or 0.0)
    target = None
    if isinstance(proxy, list) and len(proxy) >= 2:
        target = {"x": float(proxy[0]), "y": float(proxy[1]), "source": "visible_proxy_xy"}
    return yaw, "candidate_yaw_from_approach_candidate_report", target


def write_plan(args: argparse.Namespace, out: Path, candidate: dict[str, Any]) -> dict[str, Any]:
    object_facing = bool(getattr(args, "object_facing_approach", False))
    room_route = read_json(args.room_route, {})
    room_chain = room_route.get("room_sequence") or LEGACY_EXPECTED_ROOM_CHAIN
    gateway_sequence = room_route.get("gateway_sequence") or LEGACY_EXPECTED_GATEWAY_SEQUENCE
    plan = {
        "artifact_type": "task14c3_object_facing_runtime_plan" if object_facing else "task14c2_single_object_runtime_plan",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "query": args.query,
        "object_id": canonical_object_id(args.object_id),
        "approach_candidate_id": args.approach_candidate_id,
        "floor_id": args.floor_id,
        "start_room": args.start_room,
        "target_room": args.target_room,
        "expected_room_chain": room_chain,
        "expected_gateway_sequence": gateway_sequence,
        "execution_policy": "object_facing_position_then_yaw_alignment" if object_facing else "two_segment_execution",
        "segment_a": {
            "name": "room_route_segment",
            "waypoints_json": rel(args.room_route),
            "controller_profile": args.controller_profile,
            "execution_strategy": args.execution_strategy,
            "object_approach_appended": False,
        },
        "segment_b1": {
            "name": "approach_position_segment" if object_facing else "object_approach_segment",
            "starts_after_target_room_arrival": True,
            "terminal_yaw_policy": "path-heading yaw or current-yaw relaxed terminal orientation" if object_facing else "object-facing candidate yaw",
            "approach_pose": pose_from_candidate(candidate, canonical_object_id(args.object_id), args.query),
        },
        "segment_b2": {
            "name": "object_facing_yaw_alignment",
            "starts_after_approach_position_reached": True,
            "required": object_facing,
            "target_yaw_policy": "live pose to visible_proxy_xy if available, otherwise selected approach candidate yaw",
            "yaw_alignment_tolerance_rad": getattr(args, "yaw_alignment_tolerance_rad", None),
            "direct_cmd_vel_fallback_allowed": not getattr(args, "disable_direct_cmd_vel_yaw_fallback", False),
        } if object_facing else None,
        "forbidden_claims": [
            "semantic ground-truth accuracy",
            "object found",
            "object arrival without visibility/same-side runtime proof",
            "multi-object generalization",
            "cross-floor navigation",
            "RViz GUI validation unless RViz was actually started and visually confirmed",
        ],
    }
    write_json(out / ("object_facing_runtime_plan.json" if object_facing else "obj175_single_object_runtime_plan.json"), plan)
    return plan


def run_logged(cmd: list[str], log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as handle:
        handle.write("$ " + " ".join(cmd) + "\n")
        handle.flush()
        proc = subprocess.run(cmd, stdout=handle, stderr=subprocess.STDOUT, text=True)
    return int(proc.returncode)


def copy_room_segment_paths(out: Path) -> None:
    src_dir = out / "sent_controller_paths"
    dst_dir = out / "sent_controller_paths/room_route_segment"
    dst_dir.mkdir(parents=True, exist_ok=True)
    for path in src_dir.glob("*.json"):
        shutil.copy2(path, dst_dir / path.name)


def load_runtime_helpers() -> Any:
    import run_scene_route as runtime  # type: ignore

    return runtime


def wait_for_current_pose(runtime: Any, node: Any, timeout_sec: float = 12.0) -> dict[str, Any] | None:
    deadline = time.time() + timeout_sec
    pose = None
    while runtime.rclpy.ok() and time.time() < deadline and pose is None:
        runtime.rclpy.spin_once(node, timeout_sec=0.1)
        pose = node.current_pose()
    return pose


def inspect_runtime_interfaces(out: Path) -> dict[str, Any]:
    inspection: dict[str, Any] = {
        "created_utc": now_iso(),
        "actions": [],
        "services": [],
        "inspection_errors": [],
    }
    for kind in ("action", "service"):
        ros2 = shutil.which("ros2") or "/opt/ros/foxy/bin/ros2"
        cmd = [ros2, kind, "list"]
        try:
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
            rows = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
            inspection["actions" if kind == "action" else "services"] = rows
            if proc.returncode != 0:
                inspection["inspection_errors"].append({"command": cmd, "returncode": proc.returncode, "stderr": proc.stderr.strip()})
        except Exception as exc:
            inspection["inspection_errors"].append({"command": cmd, "error": f"{type(exc).__name__}: {exc}"})
    inspection["nav2_compatible_yaw_methods_considered"] = [
        "orientation_only_navigate_to_pose_at_current_xy",
        "short_follow_path_same_xy_target_orientation",
    ]
    inspection["spin_action_available"] = any(item.endswith("/spin") or item == "/spin" for item in inspection["actions"])
    write_json(out / "yaw_alignment_commands/runtime_interface_inspection.json", inspection)
    return inspection


def publish_stop(runtime: Any, publisher: Any) -> None:
    msg = runtime.Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)


def run_direct_cmd_vel_yaw_alignment(
    runtime: Any,
    node: Any,
    out: Path,
    *,
    target_yaw: float,
    tolerance_rad: float,
    timeout_sec: float,
    max_angular_z: float,
    drift_threshold_m: float,
) -> dict[str, Any]:
    publisher = node.create_publisher(runtime.Twist, "/cmd_vel", 10)
    start_pose = node.current_pose()
    result: dict[str, Any] = {
        "attempted": True,
        "method": "bounded_direct_cmd_vel_yaw_alignment",
        "success": False,
        "linear_velocity_policy": "always_zero",
        "max_angular_velocity_rad_s": float(max_angular_z),
        "timeout_sec": float(timeout_sec),
        "xy_drift_abort_threshold_m": float(drift_threshold_m),
        "start_pose": start_pose,
        "commands_path": rel(out / "yaw_alignment_commands/direct_cmd_vel_yaw_alignment_commands.json"),
        "failure_reason": None,
    }
    commands: list[dict[str, Any]] = []
    deadline = time.time() + timeout_sec
    next_sample = 0.0
    previous_context = dict(node.active_sample_context)
    node.active_sample_context = {"segment": "yaw_alignment_segment", "method": "bounded_direct_cmd_vel"}
    try:
        while runtime.rclpy.ok() and time.time() < deadline:
            runtime.rclpy.spin_once(node, timeout_sec=0.05)
            pose = node.current_pose()
            if pose is None:
                result["failure_reason"] = "could not observe robot pose during direct yaw alignment"
                break
            err_signed = normalize_angle(target_yaw - float(pose["yaw"]))
            err_abs = abs(err_signed)
            drift = dist(start_pose, pose) if start_pose else 0.0
            if drift is not None and drift > drift_threshold_m:
                result["failure_reason"] = f"xy drift exceeded {drift_threshold_m} m"
                break
            if err_abs <= tolerance_rad:
                result["success"] = True
                break
            angular = max(-max_angular_z, min(max_angular_z, 0.9 * err_signed))
            if abs(angular) < 0.06:
                angular = 0.06 if err_signed > 0.0 else -0.06
            msg = runtime.Twist()
            msg.linear.x = 0.0
            msg.angular.z = float(angular)
            publisher.publish(msg)
            row = {
                "time_wall_sec": time.time(),
                "x": pose.get("x"),
                "y": pose.get("y"),
                "yaw": pose.get("yaw"),
                "target_yaw": target_yaw,
                "yaw_error_rad": err_abs,
                "cmd_vel_linear_x": 0.0,
                "cmd_vel_angular_z": float(angular),
                "xy_drift_m": drift,
            }
            commands.append(row)
            if time.time() >= next_sample:
                node.sample_pose("yaw_alignment")
                next_sample = time.time() + 0.25
        if not result["success"] and result["failure_reason"] is None:
            result["failure_reason"] = f"direct cmd_vel yaw alignment timed out after {timeout_sec}s"
    finally:
        publish_stop(runtime, publisher)
        for _ in range(5):
            runtime.rclpy.spin_once(node, timeout_sec=0.05)
        node.active_sample_context = previous_context
        write_json(out / "yaw_alignment_commands/direct_cmd_vel_yaw_alignment_commands.json", {
            "artifact_type": "task14c3_direct_cmd_vel_yaw_alignment_commands",
            "created_utc": now_iso(),
            "commands": commands,
            "stop_command_sent": True,
        })
    final_pose = node.current_pose()
    result["final_pose"] = final_pose
    result["final_yaw_rad"] = round_float((final_pose or {}).get("yaw"))
    result["final_xy_drift_m"] = round_float(dist(start_pose, final_pose))
    result["yaw_error_rad"] = round_float(angle_error(float((final_pose or {}).get("yaw", 0.0)), target_yaw) if final_pose else None)
    result["success"] = bool(result["success"] or (result["yaw_error_rad"] is not None and result["yaw_error_rad"] <= tolerance_rad))
    return result


def run_yaw_alignment_segment(args: argparse.Namespace, out: Path, candidate: dict[str, Any]) -> dict[str, Any]:
    runtime = load_runtime_helpers()
    result: dict[str, Any] = {
        "artifact_type": "task14c3_yaw_alignment_segment_result",
        "created_utc": now_iso(),
        "yaw_alignment_attempted": True,
        "yaw_alignment_method": None,
        "direct_cmd_vel_yaw_alignment_used": False,
        "target_object_facing_yaw_rad": None,
        "target_yaw_source": None,
        "final_yaw_rad": None,
        "yaw_error_rad": None,
        "yaw_error_deg": None,
        "yaw_tolerance_rad": float(args.yaw_alignment_tolerance_rad),
        "yaw_alignment_tolerance_rad": float(args.yaw_alignment_tolerance_rad),
        "approach_yaw_aligned": False,
        "yaw_alignment_failure_reason": None,
        "nav2_clean_yaw_alignment": False,
        "action_servers": {},
    }
    if runtime.IMPORT_ERROR:
        result["yaw_alignment_failure_reason"] = runtime.IMPORT_ERROR
        return result
    interface_inspection = inspect_runtime_interfaces(out)
    result["runtime_interface_inspection"] = interface_inspection
    trajectory_json = out / "executed_trajectories/yaw_alignment_segment_trajectory.json"
    runtime.rclpy.init(args=None)
    node = runtime.RouteNode()
    try:
        start_pose = wait_for_current_pose(runtime, node, timeout_sec=args.pose_timeout_sec)
        result["start_pose_observed"] = start_pose
        if start_pose is None:
            result["yaw_alignment_failure_reason"] = "could not observe robot pose before yaw alignment"
            return result
        target_yaw, source, proxy_target = target_yaw_from_pose(candidate, start_pose)
        result["target_object_facing_yaw_rad"] = round_float(target_yaw)
        result["target_yaw_source"] = source
        result["target_object_proxy"] = proxy_target
        nav_ready = bool(node.nav_client.wait_for_server(timeout_sec=args.server_timeout_sec))
        follow_ready = bool(node.follow_path_client.wait_for_server(timeout_sec=args.server_timeout_sec))
        result["action_servers"] = {"/navigate_to_pose": nav_ready, "/follow_path": follow_ready}
        yaw_goal = {
            "x": float(start_pose["x"]),
            "y": float(start_pose["y"]),
            "yaw": target_yaw,
            "source": "orientation_only_current_xy_yaw_goal",
            "waypoint_index": 9100,
            "target_waypoint_index": 9100,
        }
        nav_result: dict[str, Any] = {"attempted": False, "success": False, "failure_reason": "/navigate_to_pose action server not available"}
        if nav_ready:
            runtime.save_controller_path_artifact(
                result,
                out / "yaw_alignment_segment_result.json",
                path_id="yaw_alignment_orientation_only_navigate_to_pose",
                action_type="NavigateToPose",
                points=[start_pose, yaw_goal],
                target=yaw_goal,
                start_pose=start_pose,
                map_yaml=args.stable_map,
                extra={"yaw_alignment_target": True},
            )
            nav_result = node.navigate(yaw_goal, args.yaw_alignment_nav2_timeout_sec, context={"segment": "yaw_alignment_segment", "method": "orientation_only_navigate_to_pose"})
        result["orientation_only_navigate_to_pose"] = nav_result
        final_pose = node.current_pose()
        yaw_err = angle_error(float((final_pose or {}).get("yaw", 0.0)), target_yaw) if final_pose else None
        if yaw_err is not None and yaw_err <= float(args.yaw_alignment_tolerance_rad):
            result["yaw_alignment_method"] = "orientation_only_navigate_to_pose_at_current_xy"
            result["nav2_clean_yaw_alignment"] = bool(nav_result.get("success"))
        else:
            follow_result: dict[str, Any] = {"attempted": False, "success": False, "failure_reason": "/follow_path action server not available"}
            current = node.current_pose()
            if follow_ready and current:
                current_for_path = dict(current)
                current_for_path.setdefault("waypoint_index", 9099)
                current_for_path.setdefault("target_waypoint_index", 9099)
                current_for_path.setdefault("source", "yaw_alignment_current_pose_anchor")
                same_xy_goal = dict(yaw_goal)
                same_xy_goal["x"] = float(current["x"])
                same_xy_goal["y"] = float(current["y"])
                same_xy_goal["source"] = "short_follow_path_same_xy_target_orientation"
                runtime.save_controller_path_artifact(
                    result,
                    out / "yaw_alignment_segment_result.json",
                    path_id="yaw_alignment_short_follow_path_same_xy",
                    action_type="FollowPath",
                    points=[current_for_path, same_xy_goal],
                    target=same_xy_goal,
                    start_pose=current_for_path,
                    map_yaml=args.stable_map,
                    extra={"yaw_alignment_target": True},
                )
                follow_result = node.follow_path([current_for_path, same_xy_goal], args.yaw_alignment_nav2_timeout_sec, context={"segment": "yaw_alignment_segment", "method": "short_follow_path_same_xy"})
            result["short_follow_path_same_xy"] = follow_result
            final_pose = node.current_pose()
            yaw_err = angle_error(float((final_pose or {}).get("yaw", 0.0)), target_yaw) if final_pose else None
            if yaw_err is not None and yaw_err <= float(args.yaw_alignment_tolerance_rad):
                result["yaw_alignment_method"] = "short_follow_path_same_xy_target_orientation"
                result["nav2_clean_yaw_alignment"] = bool(follow_result.get("success"))
            elif not args.disable_direct_cmd_vel_yaw_fallback:
                direct = run_direct_cmd_vel_yaw_alignment(
                    runtime,
                    node,
                    out,
                    target_yaw=target_yaw,
                    tolerance_rad=float(args.yaw_alignment_tolerance_rad),
                    timeout_sec=float(args.direct_yaw_timeout_sec),
                    max_angular_z=float(args.direct_yaw_max_angular_z),
                    drift_threshold_m=float(args.direct_yaw_drift_threshold_m),
                )
                result["direct_cmd_vel_yaw_alignment"] = direct
                result["direct_cmd_vel_yaw_alignment_used"] = True
                result["yaw_alignment_method"] = "bounded_direct_cmd_vel_yaw_alignment"
                final_pose = node.current_pose()
                yaw_err = angle_error(float((final_pose or {}).get("yaw", 0.0)), target_yaw) if final_pose else None
        result["final_pose_observed"] = final_pose
        result["final_yaw_rad"] = round_float((final_pose or {}).get("yaw"))
        result["xy_drift_during_yaw_alignment_m"] = round_float(dist(start_pose, final_pose))
        result["yaw_error_rad"] = round_float(yaw_err)
        result["yaw_error_deg"] = round_float(math.degrees(yaw_err), 3) if yaw_err is not None else None
        result["approach_yaw_aligned"] = bool(yaw_err is not None and yaw_err <= float(args.yaw_alignment_tolerance_rad))
        if not result["approach_yaw_aligned"]:
            reasons = [
                (result.get("orientation_only_navigate_to_pose") or {}).get("failure_reason"),
                (result.get("short_follow_path_same_xy") or {}).get("failure_reason"),
                (result.get("direct_cmd_vel_yaw_alignment") or {}).get("failure_reason"),
            ]
            result["yaw_alignment_failure_reason"] = next((item for item in reasons if item), "final yaw did not align within tolerance")
        return result
    finally:
        write_json(trajectory_json, {
            "artifact_type": "task14c3_yaw_alignment_segment_trajectory",
            "created_utc": now_iso(),
            "samples": node.trajectory_samples,
        })
        result["trajectory"] = rel(trajectory_json)
        try:
            node.destroy_node()
        except Exception:
            pass
        if runtime.rclpy.ok():
            runtime.rclpy.shutdown()
        path_dir = out / "sent_controller_paths"
        dst_dir = path_dir / "yaw_alignment_segment"
        dst_dir.mkdir(parents=True, exist_ok=True)
        for path in path_dir.glob("yaw_alignment*.json"):
            shutil.copy2(path, dst_dir / path.name)


def run_approach_position_segment(args: argparse.Namespace, out: Path, candidate: dict[str, Any]) -> dict[str, Any]:
    runtime = load_runtime_helpers()
    if runtime.IMPORT_ERROR:
        return {
            "artifact_type": "task14c2_approach_segment_result",
            "created_utc": now_iso(),
            "attempted": False,
            "succeeded": False,
            "approach_pose_reached": False,
            "approach_position_reached": False,
            "failure_layer": "rclpy",
            "failure_reason": runtime.IMPORT_ERROR,
        }

    target = pose_from_candidate(candidate, canonical_object_id(args.object_id), args.query)
    target["waypoint_index"] = 9000
    target["target_waypoint_index"] = 9000
    output_json = out / "approach_position_segment_result.json"
    trajectory_json = out / "executed_trajectories/approach_position_segment_trajectory.json"
    wall_json = out / "wall_crossing_validation/approach_position_segment_wall_crossing.json"
    result: dict[str, Any] = {
        "artifact_type": "task14c3_approach_position_segment_result",
        "created_utc": now_iso(),
        "attempted": True,
        "approach_position_segment_attempted": True,
        "succeeded": False,
        "target_room_arrival_required": True,
        "approach_pose_reached": False,
        "approach_position_reached": False,
        "approach_candidate_id": args.approach_candidate_id,
        "approach_position_tolerance_m": float(args.approach_position_tolerance_m),
        "approach_position_terminal_yaw_policy": None,
        "approach_position_failure_reason": None,
        "fallback_used": False,
        "action_servers": {},
        "sent_controller_paths": [],
    }

    runtime.rclpy.init(args=None)
    node = runtime.RouteNode()
    try:
        start_pose = wait_for_current_pose(runtime, node, timeout_sec=args.pose_timeout_sec)
        result["start_pose_observed"] = start_pose
        if start_pose is None:
            result.update({"failure_layer": "TF", "failure_reason": "could not observe robot pose before object approach"})
            return result
        target_yaw, yaw_policy = position_first_yaw(start_pose, target)
        target["yaw"] = target_yaw
        target["source"] = "approach_position_pose_position_first"
        result["approach_position_terminal_yaw_policy"] = yaw_policy
        result["approach_position_target_pose"] = target
        follow_ready = bool(node.follow_path_client.wait_for_server(timeout_sec=args.server_timeout_sec))
        plan_ready = bool(node.plan_client.wait_for_server(timeout_sec=args.server_timeout_sec))
        nav_ready = bool(node.nav_client.wait_for_server(timeout_sec=args.server_timeout_sec))
        result["action_servers"] = {
            "/follow_path": follow_ready,
            "/compute_path_to_pose": plan_ready,
            "/navigate_to_pose": nav_ready,
        }
        if not plan_ready and not nav_ready:
            result.update({"failure_layer": "runtime_readiness", "failure_reason": "neither ComputePathToPose nor NavigateToPose is available"})
            return result

        plan = node.compute_plan(target, args.plan_timeout_sec) if plan_ready else {
            "attempted": False,
            "success": False,
            "failure_reason": "/compute_path_to_pose action server not available",
        }
        result["compute_path_to_pose"] = plan
        computed_path = list(plan.get("path_poses") or [])
        wall_validation = None
        if computed_path and args.stable_map and runtime.validate_point_sequence is not None:
            wall_validation = runtime.validate_point_sequence(computed_path, Path(args.stable_map))
        result["computed_path_wall_validation"] = wall_validation
        if computed_path:
            runtime.save_controller_path_artifact(
                result,
                output_json,
                path_id=f"approach_position_segment_current_pose_to_{args.approach_candidate_id}_full_plan",
                action_type="ComputePathToPose",
                points=computed_path,
                target=target,
                start_pose=start_pose,
                map_yaml=args.stable_map,
            )

        follow_result: dict[str, Any] = {
            "attempted": False,
            "success": False,
            "failure_reason": "not attempted",
        }
        if computed_path and follow_ready and (wall_validation is None or wall_validation.get("wall_crossing_validation_passed")):
            runtime.save_controller_path_artifact(
                result,
                output_json,
                path_id="approach_position_segment_follow_path",
                action_type="FollowPath",
                points=computed_path,
                target=target,
                start_pose=start_pose,
                map_yaml=args.stable_map,
            )
            follow_result = node.follow_path(
                computed_path,
                args.approach_follow_path_timeout_sec,
                context={
                    "segment": "object_approach_segment",
                    "target_waypoint_index": target["waypoint_index"],
                    "target_source": "object_approach_pose",
                    "approach_candidate_id": args.approach_candidate_id,
                },
            )
        elif computed_path and wall_validation is not None and not wall_validation.get("wall_crossing_validation_passed"):
            follow_result = {
                "attempted": False,
                "success": False,
                "failure_reason": "computed object approach path intersects occupied map cells",
            }
        elif not computed_path:
            follow_result = {
                "attempted": False,
                "success": False,
                "failure_reason": plan.get("failure_reason") or "ComputePathToPose produced no object approach path",
            }
        else:
            follow_result = {
                "attempted": False,
                "success": False,
                "failure_reason": "/follow_path action server not available",
            }
        result["follow_path"] = follow_result
        result["approach_position_follow_path_success"] = bool(follow_result.get("success"))

        final_pose = node.current_pose()
        approach_distance = dist(final_pose, target)
        if not follow_result.get("success") and not (approach_distance is not None and approach_distance <= float(args.approach_position_tolerance_m)) and nav_ready:
            result["fallback_used"] = True
            runtime.save_controller_path_artifact(
                result,
                output_json,
                path_id="approach_position_segment_navigate_to_pose_fallback",
                action_type="NavigateToPose",
                points=[p for p in [node.current_pose(), target] if p],
                target=target,
                start_pose=node.current_pose(),
                map_yaml=args.stable_map,
            )
            nav_result = node.navigate(
                target,
                args.approach_goal_timeout_sec,
                context={
                    "segment": "object_approach_segment",
                    "target_source": "object_approach_pose",
                    "approach_candidate_id": args.approach_candidate_id,
                },
            )
            result["navigate_to_pose_fallback"] = nav_result
            final_pose = node.current_pose()
            approach_distance = dist(final_pose, target)

        if runtime.validate_samples is not None:
            wall_report = runtime.validate_samples(node.trajectory_samples, Path(args.stable_map))
            write_json(wall_json, wall_report)
            result["wall_crossing_validation_report"] = rel(wall_json)
            result["wall_crossing_validation_passed"] = bool(wall_report.get("wall_crossing_validation_passed"))
        else:
            result["wall_crossing_validation_passed"] = None
        result["final_pose_observed"] = final_pose
        result["final_distance_to_selected_approach_candidate_m"] = round_float(approach_distance)
        result["final_distance_to_generated_ring_037_m"] = round_float(approach_distance) if args.approach_candidate_id == "generated_ring_037" else None
        result["final_distance_to_approach_position_m"] = round_float(approach_distance)
        result["approach_pose_reached"] = bool(approach_distance is not None and approach_distance <= float(args.approach_position_tolerance_m))
        result["approach_position_reached"] = bool(result["approach_pose_reached"])
        result["succeeded"] = bool(result["approach_pose_reached"] and result.get("wall_crossing_validation_passed") is not False)
        if not result["succeeded"]:
            result["failure_layer"] = "object_approach_segment"
            result["failure_reason"] = (
                follow_result.get("failure_reason")
                or (result.get("navigate_to_pose_fallback") or {}).get("failure_reason")
                or "approach pose not reached within tolerance"
            )
            result["approach_position_failure_reason"] = result["failure_reason"]
        return result
    finally:
        for _ in range(10):
            if runtime.rclpy.ok():
                runtime.rclpy.spin_once(node, timeout_sec=0.05)
        write_json(trajectory_json, {
            "artifact_type": "task14c2_object_approach_segment_trajectory",
            "created_utc": now_iso(),
            "samples": node.trajectory_samples,
        })
        result["trajectory"] = rel(trajectory_json)
        try:
            node.destroy_node()
        except Exception:
            pass
        if runtime.rclpy.ok():
            runtime.rclpy.shutdown()
        path_dir = out / "sent_controller_paths"
        dst_dir = path_dir / "approach_position_segment"
        dst_dir.mkdir(parents=True, exist_ok=True)
        for path in path_dir.glob("approach_position_segment*.json"):
            shutil.copy2(path, dst_dir / path.name)


def write_result_md(path: Path, title: str, payload: dict[str, Any]) -> None:
    approach_candidate_id = payload.get("approach_candidate_id") or "selected approach candidate"
    lines = [
        f"# {title}",
        "",
        f"- Attempted: `{payload.get('attempted')}`",
        f"- Succeeded: `{payload.get('succeeded')}`",
        f"- Target room arrival: `{payload.get('target_room_arrival')}`",
        f"- Approach pose reached: `{payload.get('approach_pose_reached')}`",
        f"- Fallback used: `{payload.get('fallback_used')}`",
        f"- Wall crossing passed: `{payload.get('wall_crossing_validation_passed')}`",
        f"- Failure layer: `{payload.get('failure_layer')}`",
        f"- Failure reason: `{payload.get('failure_reason')}`",
    ]
    if payload.get("final_distance_to_room14_terminal_m") is not None:
        lines.append(f"- Final distance to room_14 terminal: `{payload.get('final_distance_to_room14_terminal_m')}` m")
    if payload.get("final_distance_to_selected_approach_candidate_m") is not None:
        lines.append(f"- Final distance to {approach_candidate_id}: `{payload.get('final_distance_to_selected_approach_candidate_m')}` m")
    if payload.get("final_distance_to_approach_position_m") is not None:
        lines.append(f"- Final distance to approach position: `{payload.get('final_distance_to_approach_position_m')}` m")
    if payload.get("approach_yaw_aligned") is not None:
        lines.append(f"- Object-facing yaw aligned: `{payload.get('approach_yaw_aligned')}`")
    if payload.get("yaw_error_rad") is not None:
        lines.append(f"- Yaw error: `{payload.get('yaw_error_rad')}` rad / `{payload.get('yaw_error_deg')}` deg")
    if payload.get("yaw_alignment_method") is not None:
        lines.append(f"- Yaw alignment method: `{payload.get('yaw_alignment_method')}`")
    write_text(path, "\n".join(lines))


def generate_overlay(
    out: Path,
    room_route: Path,
    map_yaml: Path,
    candidate: dict[str, Any],
    final_pose: dict[str, Any] | None,
    yaw_result: dict[str, Any] | None = None,
    *,
    object_id: str,
    query: str,
) -> dict[str, Any]:
    candidate_id = candidate.get("candidate_id") or "approach_candidate"
    png = out / f"visualizations/{object_id}_object_facing_planned_vs_executed_overlay.png"
    manifest: dict[str, Any] = {"attempted": True, "path": rel(png), "succeeded": False}
    try:
        import matplotlib.pyplot as plt  # type: ignore
        import yaml  # type: ignore
        from PIL import Image  # type: ignore
    except Exception as exc:
        manifest["failure_reason"] = f"plot imports unavailable: {type(exc).__name__}: {exc}"
        return manifest
    try:
        meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        image = Image.open(image_path)
        resolution = float(meta["resolution"])
        origin = meta["origin"]
        width, height = image.size
        extent = [float(origin[0]), float(origin[0]) + width * resolution, float(origin[1]), float(origin[1]) + height * resolution]
        route = read_json(room_route, {}).get("waypoints") or []
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(image, cmap="gray", origin="upper", extent=extent)
        ax.plot([p["x"] for p in route], [p["y"] for p in route], color="#1f77b4", linewidth=1.5, label="original room route")
        colors = {
            "room_route_segment": "#ff7f0e",
            "approach_position_segment": "#2ca02c",
            "yaw_alignment_segment": "#8c564b",
        }
        for segment, color in colors.items():
            for path in sorted((out / "sent_controller_paths" / segment).glob("*.json")):
                payload = read_json(path, {})
                points = payload.get("points") or []
                if points:
                    ax.plot([p["x"] for p in points], [p["y"] for p in points], color=color, alpha=0.7, linewidth=1.0, label=segment)
        for traj_path, color in [
            (out / "executed_trajectories/room_route_segment_trajectory.json", "#d62728"),
            (out / "executed_trajectories/approach_position_segment_trajectory.json", "#9467bd"),
            (out / "executed_trajectories/yaw_alignment_segment_trajectory.json", "#8c564b"),
        ]:
            samples = read_json(traj_path, {}).get("samples") or []
            if samples:
                ax.plot([p["x"] for p in samples], [p["y"] for p in samples], color=color, linewidth=1.0, alpha=0.75, label=traj_path.stem)
        labels = [(39, "wp39 room_13"), (51, "gateway006"), (58, "wp58 room_14")]
        for idx, label in labels:
            if idx < len(route):
                ax.scatter([route[idx]["x"]], [route[idx]["y"]], s=45, marker="o", label=label)
                ax.text(route[idx]["x"], route[idx]["y"], " " + label, fontsize=8)
        approach = pose_from_candidate(candidate, object_id, query)
        ax.scatter([approach["x"]], [approach["y"]], s=80, marker="*", color="#17becf", label=str(candidate_id))
        proxy = candidate.get("visible_proxy_xy")
        if isinstance(proxy, list) and len(proxy) >= 2:
            ax.scatter([float(proxy[0])], [float(proxy[1])], s=60, marker="D", color="#bcbd22", label="object visible proxy")
        if final_pose:
            ax.scatter([final_pose["x"]], [final_pose["y"]], s=65, marker="x", color="#e377c2", label="final robot pose")
            yaw = float(final_pose.get("yaw", 0.0))
            ax.arrow(final_pose["x"], final_pose["y"], 0.45 * math.cos(yaw), 0.45 * math.sin(yaw), width=0.025, color="#e377c2", length_includes_head=True)
        if yaw_result and yaw_result.get("target_object_facing_yaw_rad") is not None and final_pose:
            target_yaw = float(yaw_result["target_object_facing_yaw_rad"])
            ax.arrow(final_pose["x"], final_pose["y"], 0.55 * math.cos(target_yaw), 0.55 * math.sin(target_yaw), width=0.015, color="#2ca02c", alpha=0.75, length_includes_head=True)
            ax.text(final_pose["x"], final_pose["y"] + 0.25, f" yaw err {yaw_result.get('yaw_error_deg')} deg", fontsize=8)
        handles, labels_seen = ax.get_legend_handles_labels()
        dedup: dict[str, Any] = {}
        for handle, label in zip(handles, labels_seen):
            dedup.setdefault(label, handle)
        ax.legend(dedup.values(), dedup.keys(), loc="best", fontsize=8)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"RSLG-SLAM {object_id} planned vs executed overlay")
        png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png, dpi=180, bbox_inches="tight")
        plt.close(fig)
        manifest["succeeded"] = True
    except Exception as exc:
        manifest["failure_reason"] = f"{type(exc).__name__}: {exc}"
    return manifest


def write_completion_reports(out: Path, result: dict[str, Any]) -> None:
    success = bool(result.get("object_facing_approach_success", result.get("runtime_success")))
    partial = bool(result.get("target_room_arrival") and result.get("approach_position_reached") and not result.get("approach_yaw_aligned"))
    object_id = result.get("object_id")
    query = result.get("query")
    approach_candidate_id = result.get("approach_candidate_id") or "selected approach candidate"
    target_room = result.get("target_room") or "target room"
    lines = [
        "# Object-Facing Approach Runtime Validation",
        "",
        f"- Object-facing approach success: `{success}`",
        f"- Nav2-clean approach success: `{result.get('nav2_clean_approach_success')}`",
        f"- Gazebo/Nav2 started: `{result.get('ros_gazebo_nav2_started')}`",
        f"- Target room arrival: `{result.get('target_room_arrival')}`",
        f"- Approach position reached: `{result.get('approach_position_reached')}`",
        f"- Approach yaw aligned: `{result.get('approach_yaw_aligned')}`",
        f"- Fallback used: `{result.get('fallback_used')}`",
        f"- Room-route sparse fallback used: `{result.get('room_route_sparse_fallback_used')}`",
        f"- Direct cmd_vel yaw alignment used: `{result.get('yaw_alignment_direct_cmd_vel_fallback_used')}`",
        f"- Final distance to {target_room} terminal: `{result.get('final_distance_to_target_room_terminal_m', result.get('final_distance_to_room14_terminal_m'))}` m",
        f"- Final distance to {approach_candidate_id}: `{result.get('final_distance_to_approach_position_m')}` m",
        f"- Final yaw error: `{result.get('yaw_error_rad')}` rad / `{result.get('yaw_error_deg')}` deg",
        f"- XY drift during yaw alignment: `{result.get('xy_drift_during_yaw_alignment_m')}` m",
        f"- Wall crossing passed: `{result.get('wall_crossing_validation_passed')}`",
        f"- RViz/GUI validated: `{result.get('rviz_gui_visual_validation')}`",
        f"- Yaw tolerance: `{result.get('yaw_alignment_tolerance_rad')}` rad. This run uses `0.50` rad to be stricter than the task12/Nav2 `0.70` rad yaw-goal tolerance while allowing small in-place controller settling error.",
        "",
        "## Claims Allowed",
        "",
        f"- RSLG-SLAM resolved the artifact-backed object query `{query}` to `{object_id}`.",
        f"- RSLG-SLAM executed target-room navigation to `{target_room}` in Gazebo/Nav2." if result.get("target_room_arrival") else "- Target-room navigation success cannot be claimed.",
        f"- RSLG-SLAM reached the selected approach position `{approach_candidate_id}`." if result.get("approach_position_reached") else f"- Reaching `{approach_candidate_id}` cannot be claimed.",
        "- RSLG-SLAM aligned the robot to face the target object/proxy within configured yaw tolerance." if result.get("approach_yaw_aligned") else "- Object-facing alignment cannot be claimed.",
        "- This is single-object query-driven object-facing approach navigation evidence." if success else "- Full object-facing approach success cannot be claimed.",
        "",
        "## Claims Not Allowed",
        "",
        "- Semantic ground-truth accuracy.",
        "- Open-vocabulary CLIP retrieval.",
        "- Object found or object arrival without visibility/same-side runtime proof.",
        "- Object arrival with verified visual perception.",
        "- Multi-object generalization from this single run.",
        "- Cross-floor navigation or stair/vertical transition execution.",
        "- TurtleBot3 stair capability.",
        "- RViz GUI validation unless an RViz GUI was actually started and visually confirmed.",
    ]
    if success:
        write_text(out / "success_report.md", "\n".join(lines))
    else:
        failure_lines = lines + [
            "",
            "## Failure Evidence",
            "",
            f"- Failure layer: `{result.get('failure_layer')}`",
            f"- Failure reason: `{result.get('failure_reason')}`",
            f"- Room segment result: `{rel(out / 'room_segment_result.json')}`",
            f"- Approach position segment result: `{rel(out / 'approach_position_segment_result.json')}`",
            f"- Yaw alignment segment result: `{rel(out / 'yaw_alignment_segment_result.json')}`",
            "",
            "## Next Actions",
            "",
            "- Inspect the segment logs under `run_logs/`.",
            "- If Segment A failed, repair target-room runtime before attempting object approach.",
            "- If Segment B2 failed after B1 succeeded, tune or replace only the yaw-alignment action.",
        ]
        if partial:
            write_text(out / "partial_success_report.md", "\n".join(lines + [
                "",
                "## Partial Success",
                "",
                f"- Segment A reached `{target_room}` and Segment B1 reached `{approach_candidate_id}`.",
                "- Segment B2 did not satisfy object-facing yaw tolerance, so full object-facing approach success is false.",
            ]))
        write_text(out / "failure_report.md", "\n".join(failure_lines))
    write_text(out / "completion_summary.md", "\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--task14c-output-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--query", required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--approach-candidate-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--target-room", required=True)
    parser.add_argument("--stable-map", type=Path, required=True)
    parser.add_argument("--semantic-route", type=Path, required=True)
    parser.add_argument("--room-route", type=Path, required=True)
    parser.add_argument("--controller-profile", choices=["baseline", "task12_robust"], default="task12_robust")
    parser.add_argument("--execution-strategy", default="split_follow_path_with_current_pose_gateway_handoff")
    parser.add_argument("--two-segment-execution", action="store_true")
    parser.add_argument("--object-facing-approach", action="store_true")
    parser.add_argument("--position-first-approach", action="store_true")
    parser.add_argument("--yaw-alignment-required", action="store_true")
    parser.add_argument("--validate-wall-crossing", action="store_true")
    parser.add_argument("--record-sent-controller-paths", action="store_true")
    parser.add_argument("--record-executed-trajectory", action="store_true")
    parser.add_argument("--server-timeout-sec", type=float, default=25.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=12.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=20.0)
    parser.add_argument("--approach-follow-path-timeout-sec", type=float, default=90.0)
    parser.add_argument("--approach-goal-timeout-sec", type=float, default=90.0)
    parser.add_argument("--approach-tolerance-m", type=float, default=0.35)
    parser.add_argument("--approach-position-tolerance-m", type=float, default=None)
    parser.add_argument("--yaw-alignment-tolerance-rad", type=float, default=0.50)
    parser.add_argument("--yaw-alignment-nav2-timeout-sec", type=float, default=25.0)
    parser.add_argument("--direct-yaw-timeout-sec", type=float, default=30.0)
    parser.add_argument("--direct-yaw-max-angular-z", type=float, default=0.25)
    parser.add_argument("--direct-yaw-drift-threshold-m", type=float, default=0.12)
    parser.add_argument("--disable-direct-cmd-vel-yaw-fallback", action="store_true")
    parser.add_argument("--use-current-robot-pose-if-running", action="store_true")
    args = parser.parse_args()
    if args.approach_position_tolerance_m is None:
        args.approach_position_tolerance_m = float(args.approach_tolerance_m)

    out = args.output_dir
    for subdir in ("run_logs", "sent_controller_paths", "yaw_alignment_commands", "executed_trajectories", "wall_crossing_validation", "visualizations"):
        (out / subdir).mkdir(parents=True, exist_ok=True)
    object_id = canonical_object_id(args.object_id)
    candidate = candidate_from_report(args.task14c_output_dir, object_id, args.approach_candidate_id)
    if candidate is None:
        result = {
            "artifact_type": "task14c3_object_facing_runtime_result" if args.object_facing_approach else "task14c2_single_object_runtime_result",
            "created_utc": now_iso(),
            "project_name": "RSLG-SLAM",
            "query": args.query,
            "object_id": object_id,
            "approach_candidate_id": args.approach_candidate_id,
            "stage_a_rerun": False,
            "reference_00824_modified": False,
            "runtime_success": False,
            "object_facing_approach_success": False,
            "nav2_clean_approach_success": False,
            "failure_layer": "approach_candidate_lookup",
            "failure_reason": "requested approach candidate not found in task14c report",
        }
        write_json(out / ("object_facing_runtime_result.json" if args.object_facing_approach else "obj175_single_object_runtime_result.json"), result)
        write_completion_reports(out, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    plan = write_plan(args, out, candidate)
    route_payload = read_json(args.room_route, {})
    expected_room_chain = route_payload.get("room_sequence") or LEGACY_EXPECTED_ROOM_CHAIN
    expected_gateway_sequence = route_payload.get("gateway_sequence") or LEGACY_EXPECTED_GATEWAY_SEQUENCE
    profile_name = "floor_2_nav2_task12_controller_robust" if args.controller_profile == "task12_robust" else "floor_2_nav2"
    runtime_profile = args.stage_output_dir / "runtime/profiles" / profile_name / "runtime_profile.json"
    object_facing = bool(args.object_facing_approach)
    room_result_json = out / ("room_segment_result.json" if object_facing else "obj175_room_route_segment_result.json")
    room_result_md = out / ("room_segment_result.md" if object_facing else "obj175_room_route_segment_result.md")
    room_traj_json = out / "executed_trajectories/room_route_segment_trajectory.json"
    room_wall_json = out / "wall_crossing_validation/room_route_segment_wall_crossing.json"
    latest_slice = out / "room_route_segment_latest_follow_path_slice.json"
    room_cmd = [
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
        str(args.room_route),
        "--expected-room-chain",
        ",".join(expected_room_chain),
        "--expected-gateway-sequence",
        ",".join(expected_gateway_sequence),
        "--allow-non-scene-truth",
        "--execution-strategy",
        args.execution_strategy,
        "--split-at-through-room-anchors",
        "--validate-wall-crossing",
        "--wall-crossing-map-yaml",
        str(args.stable_map),
        "--wall-crossing-output-json",
        str(room_wall_json),
        "--current-pose-gateway-handoff",
        "--handoff-target-distance-m",
        "1.0",
        "--output-json",
        str(room_result_json),
        "--output-md",
        str(room_result_md),
        "--trajectory-output-json",
        str(room_traj_json),
        "--latest-slice-output-json",
        str(latest_slice),
    ]
    if not args.use_current_robot_pose_if_running:
        room_cmd.append("--from-start")
    exact_commands = {
        "artifact_type": "task14c3_exact_runtime_commands" if object_facing else "task14c2_exact_runtime_commands",
        "created_utc": now_iso(),
        "top_level_command": sys.argv,
        "room_route_segment_command": room_cmd,
        "approach_position_segment_command": "in-process rclpy ComputePathToPose plus FollowPath after target_room_arrival, with position-first terminal yaw",
        "yaw_alignment_segment_command": "in-process rclpy NavigateToPose/FollowPath yaw alignment, with bounded direct cmd_vel fallback if needed",
    }
    write_json(out / "exact_runtime_commands.json", exact_commands)

    room_returncode = run_logged(room_cmd, out / "run_logs/room_segment_execution.log")
    room_payload = read_json(room_result_json, {})
    copy_room_segment_paths(out)
    room_terminal = (read_json(args.room_route, {}).get("waypoints") or [None])[-1]
    final_room_pose = room_payload.get("final_pose_observed")
    room_distance = dist(final_room_pose, room_terminal)
    room_segment_ok = bool(
        room_returncode == 0
        and room_payload.get("terminal_reached") is True
        and room_payload.get("clean_runtime_success") is True
        and room_payload.get("execution_strategy") == "split_follow_path_with_current_pose_gateway_handoff"
        and room_payload.get("sparse_fallback_used") is False
        and room_payload.get("wall_crossing_validation_passed") is True
    )
    room_summary = {
        "artifact_type": "task14c2_room_route_segment_result_summary",
        "created_utc": now_iso(),
        "attempted": True,
        "succeeded": room_segment_ok,
        "target_room_arrival": room_segment_ok,
        "uses_task12_robust_clean_behavior": bool(room_segment_ok),
        "execution_strategy": room_payload.get("execution_strategy"),
        "fallback_used": bool(room_payload.get("sparse_fallback_used")),
        "wall_crossing_validation_passed": room_payload.get("wall_crossing_validation_passed"),
        "final_distance_to_target_room_terminal_m": round_float(room_distance),
        "final_distance_to_room14_terminal_m": round_float(room_distance) if args.target_room == "room_14" else None,
        "returncode": room_returncode,
        "route_execution_result": rel(room_result_json),
        "trajectory": rel(room_traj_json),
        "failure_layer": None if room_segment_ok else room_payload.get("failure_layer", "room_route_segment"),
        "failure_reason": None if room_segment_ok else room_payload.get("failure_reason", "target-room segment did not meet clean task12 behavior criteria"),
    }
    write_json(room_result_json, {**room_payload, "task14c3_segment_summary" if object_facing else "task14c2_segment_summary": room_summary})
    write_result_md(room_result_md, "Room Route Segment Result", room_summary)

    approach_payload: dict[str, Any] = {
        "artifact_type": "task14c3_approach_position_segment_result" if object_facing else "task14c2_approach_segment_result",
        "created_utc": now_iso(),
        "attempted": False,
        "succeeded": False,
        "approach_pose_reached": False,
        "approach_position_segment_attempted": False,
        "approach_position_follow_path_success": False,
        "approach_position_reached": False,
        "final_distance_to_approach_position_m": None,
        "approach_position_terminal_yaw_policy": None,
        "approach_position_failure_reason": "target_room_arrival was not validated",
        "blocked_reason": "target_room_arrival was not validated",
    }
    if room_segment_ok:
        approach_payload = run_approach_position_segment(args, out, candidate)
    approach_result_json = out / ("approach_position_segment_result.json" if object_facing else "obj175_approach_segment_result.json")
    approach_result_md = out / ("approach_position_segment_result.md" if object_facing else "obj175_approach_segment_result.md")
    write_json(approach_result_json, approach_payload)
    write_result_md(approach_result_md, "Approach Position Segment Result", approach_payload)
    write_text(out / "run_logs/approach_position_segment_execution.log", json.dumps(approach_payload, indent=2, sort_keys=True))

    yaw_payload: dict[str, Any] = {
        "artifact_type": "task14c3_yaw_alignment_segment_result",
        "created_utc": now_iso(),
        "yaw_alignment_attempted": False,
        "yaw_alignment_method": None,
        "direct_cmd_vel_yaw_alignment_used": False,
        "target_object_facing_yaw_rad": None,
        "target_yaw_source": None,
        "final_yaw_rad": None,
        "yaw_error_rad": None,
        "yaw_error_deg": None,
        "yaw_tolerance_rad": float(args.yaw_alignment_tolerance_rad),
        "yaw_alignment_tolerance_rad": float(args.yaw_alignment_tolerance_rad),
        "approach_yaw_aligned": False,
        "yaw_alignment_failure_reason": "approach_position_reached was not validated",
    }
    if object_facing and approach_payload.get("approach_position_reached") is True:
        yaw_payload = run_yaw_alignment_segment(args, out, candidate)
    elif not object_facing:
        yaw_payload["yaw_alignment_failure_reason"] = "object-facing approach mode was not requested"
    write_json(out / "yaw_alignment_segment_result.json", yaw_payload)
    write_result_md(out / "yaw_alignment_segment_result.md", "Yaw Alignment Segment Result", yaw_payload)
    write_text(out / "run_logs/yaw_alignment_segment_execution.log", json.dumps(yaw_payload, indent=2, sort_keys=True))

    final_pose = yaw_payload.get("final_pose_observed") or approach_payload.get("final_pose_observed") or final_room_pose
    fallback_used = bool(room_summary.get("fallback_used") or approach_payload.get("fallback_used") or yaw_payload.get("direct_cmd_vel_yaw_alignment_used"))
    wall_passed = bool(room_summary.get("wall_crossing_validation_passed")) and (
        approach_payload.get("wall_crossing_validation_passed") is True if approach_payload.get("attempted") else False
    )
    object_facing_success = bool(
        plan.get("stage_a_rerun") is False
        and plan.get("reference_00824_modified") is False
        and room_segment_ok
        and approach_payload.get("attempted") is True
        and approach_payload.get("approach_position_reached") is True
        and yaw_payload.get("approach_yaw_aligned") is True
        and (approach_payload.get("final_distance_to_approach_position_m") is not None and float(approach_payload["final_distance_to_approach_position_m"]) <= float(args.approach_position_tolerance_m))
        and (yaw_payload.get("yaw_error_rad") is not None and float(yaw_payload["yaw_error_rad"]) <= float(args.yaw_alignment_tolerance_rad))
        and wall_passed
    )
    nav2_clean_approach_success = bool(object_facing_success and not fallback_used and approach_payload.get("approach_position_follow_path_success") is True and yaw_payload.get("nav2_clean_yaw_alignment") is True)
    result = {
        "artifact_type": "task14c3_object_facing_runtime_result" if object_facing else "task14c2_single_object_runtime_result",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "query": args.query,
        "object_id": object_id,
        "approach_candidate_id": args.approach_candidate_id,
        "floor_id": args.floor_id,
        "start_room": args.start_room,
        "target_room": args.target_room,
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "ros_gazebo_nav2_started": True,
        "rviz_gui_visual_validation": False,
        "room_route_segment_attempted": True,
        "room_route_segment_success": bool(room_segment_ok),
        "room_route_segment_uses_task12_robust_clean_behavior": room_summary["uses_task12_robust_clean_behavior"],
        "target_room_arrival": bool(room_summary["target_room_arrival"]),
        "wall_crossing_validation_passed": wall_passed,
        "approach_position_segment_attempted": bool(approach_payload.get("approach_position_segment_attempted", approach_payload.get("attempted"))),
        "approach_position_follow_path_success": bool(approach_payload.get("approach_position_follow_path_success")),
        "approach_position_reached": bool(approach_payload.get("approach_position_reached", approach_payload.get("approach_pose_reached"))),
        "final_distance_to_approach_position_m": approach_payload.get("final_distance_to_approach_position_m", approach_payload.get("final_distance_to_selected_approach_candidate_m")),
        "final_distance_to_selected_approach_candidate_m": approach_payload.get("final_distance_to_selected_approach_candidate_m", approach_payload.get("final_distance_to_approach_position_m")),
        "approach_position_tolerance_m": float(args.approach_position_tolerance_m),
        "approach_position_terminal_yaw_policy": approach_payload.get("approach_position_terminal_yaw_policy"),
        "approach_position_failure_reason": approach_payload.get("approach_position_failure_reason"),
        "yaw_alignment_attempted": bool(yaw_payload.get("yaw_alignment_attempted")),
        "yaw_alignment_method": yaw_payload.get("yaw_alignment_method"),
        "direct_cmd_vel_yaw_alignment_used": bool(yaw_payload.get("direct_cmd_vel_yaw_alignment_used")),
        "yaw_alignment_direct_cmd_vel_fallback_used": bool(yaw_payload.get("direct_cmd_vel_yaw_alignment_used")),
        "target_object_facing_yaw_rad": yaw_payload.get("target_object_facing_yaw_rad"),
        "target_yaw_source": yaw_payload.get("target_yaw_source"),
        "final_yaw_rad": yaw_payload.get("final_yaw_rad"),
        "yaw_error_rad": yaw_payload.get("yaw_error_rad"),
        "yaw_error_deg": yaw_payload.get("yaw_error_deg"),
        "xy_drift_during_yaw_alignment_m": yaw_payload.get("xy_drift_during_yaw_alignment_m")
        if yaw_payload.get("xy_drift_during_yaw_alignment_m") is not None
        else (yaw_payload.get("direct_cmd_vel_yaw_alignment") or {}).get("final_xy_drift_m"),
        "yaw_alignment_tolerance_rad": float(args.yaw_alignment_tolerance_rad),
        "approach_yaw_aligned": bool(yaw_payload.get("approach_yaw_aligned")),
        "yaw_alignment_failure_reason": yaw_payload.get("yaw_alignment_failure_reason"),
        "final_distance_to_room14_terminal_m": room_summary["final_distance_to_room14_terminal_m"],
        "final_distance_to_target_room_terminal_m": room_summary["final_distance_to_target_room_terminal_m"],
        "final_distance_to_generated_ring_037_m": approach_payload.get("final_distance_to_generated_ring_037_m"),
        "room_route_sparse_fallback_used": bool(room_summary.get("fallback_used")),
        "approach_position_navigation_fallback_used": bool(approach_payload.get("fallback_used")),
        "fallback_used": fallback_used,
        "nav2_clean_approach_success": nav2_clean_approach_success,
        "object_facing_approach_success": object_facing_success,
        "clean_runtime_success": bool(object_facing_success and not fallback_used),
        "runtime_success": object_facing_success,
        "rviz_marker_published": False,
        "room_route_segment_result": rel(room_result_json),
        "approach_position_segment_result": rel(approach_result_json),
        "yaw_alignment_segment_result": rel(out / "yaw_alignment_segment_result.json"),
        "allowed_claims": [
            f"RSLG-SLAM resolved the artifact-backed object query `{args.query}` to `{object_id}`.",
            f"RSLG-SLAM executed target-room navigation to `{args.target_room}` in Gazebo/Nav2." if room_segment_ok else "Target-room navigation success cannot be claimed.",
            f"RSLG-SLAM reached the selected approach position `{args.approach_candidate_id}`." if approach_payload.get("approach_position_reached") else "Approach-position success cannot be claimed.",
            "RSLG-SLAM aligned the robot to face the target object/proxy within the configured yaw tolerance." if yaw_payload.get("approach_yaw_aligned") else "Object-facing yaw alignment cannot be claimed.",
            "This is single-object query-driven object-facing approach navigation evidence." if object_facing_success else "Full object-facing approach success cannot be claimed.",
        ],
        "forbidden_claims": [
            "semantic ground-truth accuracy",
            "open-vocabulary CLIP retrieval",
            "object found by visual confirmation",
            "object arrival with verified visual perception",
            "multi-object generalization",
            "RViz GUI validation unless RViz actually started and was visually confirmed",
            "cross-floor or stair navigation",
            "TurtleBot3 stair capability",
        ],
        "failure_layer": None,
        "failure_reason": None,
    }
    if not object_facing_success:
        if not room_segment_ok:
            result["failure_layer"] = room_summary.get("failure_layer") or "room_route_segment"
            result["failure_reason"] = room_summary.get("failure_reason") or "target-room segment did not meet clean success criteria"
        elif not approach_payload.get("approach_position_reached"):
            result["failure_layer"] = approach_payload.get("failure_layer") or "approach_position_segment"
            result["failure_reason"] = approach_payload.get("approach_position_failure_reason") or approach_payload.get("failure_reason") or "approach position not reached within tolerance"
        elif not yaw_payload.get("approach_yaw_aligned"):
            result["failure_layer"] = "yaw_alignment_segment"
            result["failure_reason"] = yaw_payload.get("yaw_alignment_failure_reason") or "final yaw did not face object/proxy within tolerance"
        else:
            result["failure_layer"] = "runtime_validation"
            result["failure_reason"] = "one or more required full-success criteria failed"

    overlay = generate_overlay(out, args.room_route, args.stable_map, candidate, final_pose, yaw_payload, object_id=object_id, query=args.query)
    result["visualization_overlay"] = overlay
    marker_manifest = {
        "artifact_type": "task14c3_rviz_marker_manifest" if object_facing else "task14c2_rviz_marker_manifest",
        "created_utc": now_iso(),
        "rviz_started": False,
        "rviz_gui_visual_confirmation": False,
        "marker_publication_attempted": False,
        "note": "No RViz GUI validation is claimed by this runtime runner.",
        "planned_vs_executed_overlay": overlay,
    }
    write_json(out / "rviz_marker_manifest.json", marker_manifest)
    result_json_path = out / ("object_facing_runtime_result.json" if object_facing else "obj175_single_object_runtime_result.json")
    result_md_path = out / ("object_facing_runtime_result.md" if object_facing else "obj175_single_object_runtime_result.md")
    write_json(result_json_path, result)
    write_result_md(result_md_path, "Object-Facing Runtime Result" if object_facing else "obj175 Single-Object Runtime Result", result)
    write_completion_reports(out, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if object_facing_success else 1


if __name__ == "__main__":
    raise SystemExit(main())
