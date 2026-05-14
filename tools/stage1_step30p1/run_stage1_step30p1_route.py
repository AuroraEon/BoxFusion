#!/usr/bin/env python3
"""Post-restructure Step30P1 route runner using FollowPath plus forward-only fallback."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


os.environ.setdefault("ROS_DOMAIN_ID", "84")
os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"
ALLOWED_GOAL_SOURCES = {
    "segment_start",
    "segment_crossing",
    "segment_goal",
    "snapped_segment_waypoint",
    "target_point_terminal",
    "semantic_target_terminal",
}
FORBIDDEN_NAV_GOAL_SOURCES = {"bridge", "smooth_bridge", "current_pose_path_anchor"}
EXPECTED_ROOM_CHAIN = ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"]
EXPECTED_GATEWAY_SEQUENCE = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01",
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Step30S2 Step30P1 Route Rerun Report",
        "",
        f"Created: `{payload.get('created_utc')}`",
        f"Attempted: `{payload.get('execute_attempted')}`",
        f"Succeeded: `{payload.get('succeeded')}`",
        f"Execution strategy: `{payload.get('execution_strategy')}`",
        f"FollowPath used: `{payload.get('follow_path_used')}`",
        f"Sparse fallback used: `{payload.get('sparse_fallback_used')}`",
        f"Fallback start selected index: `{payload.get('fallback_started_at_selected_list_index')}`",
        f"Fallback start waypoint index: `{payload.get('fallback_started_at_waypoint_index')}`",
        f"Robot moved distance: `{payload.get('robot_moved_distance_m')}` m",
    ]
    if payload.get("failure_layer"):
        lines.append(f"Failure layer: `{payload.get('failure_layer')}`")
    if payload.get("failure_reason"):
        lines.append(f"Failure reason: `{payload.get('failure_reason')}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def yaw_to_quat(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def status_name(status: int | None) -> str | None:
    if status is None or IMPORT_ERROR:
        return None
    return {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }.get(int(status), str(status))


def pose_distance(pose: dict[str, Any] | None, waypoint: dict[str, Any] | None) -> float | None:
    if not pose or not waypoint:
        return None
    return math.hypot(float(pose["x"]) - float(waypoint["x"]), float(pose["y"]) - float(waypoint["y"]))


def nearest_path_index(path_slice: list[dict[str, Any]], current: dict[str, Any] | None) -> tuple[int, float | None]:
    if current is None or not path_slice:
        return 0, None
    distances = [(idx, pose_distance(current, waypoint)) for idx, waypoint in enumerate(path_slice)]
    distances = [(idx, dist) for idx, dist in distances if dist is not None]
    if not distances:
        return 0, None
    idx, dist = min(distances, key=lambda item: item[1])
    return idx, float(dist)


def first_semantic_index_at_or_after_progress(semantic_goals: list[dict[str, Any]], projected_path_index: int | None) -> int:
    if projected_path_index is None:
        return 0
    for idx, waypoint in enumerate(semantic_goals):
        if int(waypoint["waypoint_index"]) >= int(projected_path_index):
            return idx
    return len(semantic_goals)


class RouteNode(Node):  # pragma: no cover - ROS runtime only
    def __init__(self) -> None:
        super().__init__("boxfusion_stage1_step30p1_route_runner")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom: dict[str, Any] | None = None
        self.feedback_count = 0
        self.follow_feedback_count = 0
        self.trajectory_samples: list[dict[str, Any]] = []
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "yaw": yaw_from_quat(q), "frame_id": msg.header.frame_id or "odom"}

    def current_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            t = transform.transform.translation
            q = transform.transform.rotation
            return {"x": float(t.x), "y": float(t.y), "z": float(t.z), "yaw": yaw_from_quat(q), "frame_id": "map", "source": "/tf map->base_footprint"}
        except TransformException:
            if self.latest_odom:
                pose = dict(self.latest_odom)
                pose["frame_id"] = "map"
                pose["source"] = "/odom under static map->odom"
                return pose
            return None

    def sample_pose(self) -> None:
        pose = self.current_pose()
        if pose:
            pose["sample_index"] = len(self.trajectory_samples)
            pose["time_wall_sec"] = time.time()
            self.trajectory_samples.append(pose)

    def make_pose(self, waypoint: dict[str, Any]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])
        q = yaw_to_quat(float(waypoint.get("yaw", 0.0)))
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        return pose

    def make_path(self, waypoints: list[dict[str, Any]]) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(waypoint) for waypoint in waypoints]
        return path

    def follow_path(self, waypoints: list[dict[str, Any]], timeout_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None, "path_pose_count": len(waypoints), "path_waypoint_indices": [int(w["waypoint_index"]) for w in waypoints]}
        if not waypoints:
            result["failure_reason"] = "empty FollowPath path"
            return result
        goal = FollowPath.Goal()
        goal.path = self.make_path(waypoints)
        if hasattr(goal, "controller_id"):
            goal.controller_id = "FollowPath"
        before = self.follow_feedback_count
        send_future = self.follow_path_client.send_goal_async(goal, feedback_callback=lambda _msg: setattr(self, "follow_feedback_count", self.follow_feedback_count + 1))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "FollowPath send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "FollowPath goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        deadline = time.time() + timeout_sec
        next_sample = 0.0
        while rclpy.ok() and time.time() < deadline and not get_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose()
                next_sample = time.time() + 0.5
        if not get_future.done():
            try:
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception:
                pass
            result["status"] = "timeout"
            result["failure_reason"] = f"FollowPath timed out after {timeout_sec}s"
            result["feedback_count_delta"] = self.follow_feedback_count - before
            return result
        wrapped = get_future.result()
        status = int(wrapped.status) if wrapped is not None else None
        result["status"] = status_name(status)
        result["success"] = status == GoalStatus.STATUS_SUCCEEDED
        if not result["success"]:
            result["failure_reason"] = f"FollowPath returned {result['status']}"
        result["feedback_count_delta"] = self.follow_feedback_count - before
        return result

    def dwell(self, seconds: float, label: str) -> dict[str, Any]:
        result = {"attempted": seconds > 0.0, "label": label, "requested_sec": seconds, "sample_count_before": len(self.trajectory_samples)}
        deadline = time.time() + max(0.0, seconds)
        next_sample = 0.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose()
                next_sample = time.time() + 0.25
        result["sample_count_after"] = len(self.trajectory_samples)
        result["recorded_sample_count"] = result["sample_count_after"] - result["sample_count_before"]
        result["success"] = True
        return result

    def compute_plan(self, waypoint: dict[str, Any], timeout_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None}
        goal = ComputePathToPose.Goal()
        goal.pose = self.make_pose(waypoint)
        goal.planner_id = "GridBased"
        send_future = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "ComputePathToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "ComputePathToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_future, timeout_sec=timeout_sec)
        wrapped = get_future.result()
        if wrapped is None:
            result["failure_reason"] = "ComputePathToPose result timeout"
            return result
        result["status"] = status_name(int(wrapped.status))
        result["success"] = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED and len(wrapped.result.path.poses) > 0
        if not result["success"]:
            result["failure_reason"] = f"ComputePathToPose returned {result['status']}"
        return result

    def navigate(self, waypoint: dict[str, Any], timeout_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None}
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(waypoint)
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=lambda _msg: setattr(self, "feedback_count", self.feedback_count + 1))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "NavigateToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "NavigateToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        deadline = time.time() + timeout_sec
        next_sample = 0.0
        while rclpy.ok() and time.time() < deadline and not get_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                self.sample_pose()
                next_sample = time.time() + 0.5
        if not get_future.done():
            try:
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception:
                pass
            result["status"] = "timeout"
            result["failure_reason"] = f"NavigateToPose timed out after {timeout_sec}s"
            return result
        wrapped = get_future.result()
        status = int(wrapped.status) if wrapped is not None else None
        result["status"] = status_name(status)
        result["success"] = status == GoalStatus.STATUS_SUCCEEDED
        if not result["success"]:
            result["failure_reason"] = f"NavigateToPose returned {result['status']}"
        return result


def semantic_goals_from_waypoints(waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
    goals = [
        w for w in waypoints
        if w.get("source") in ALLOWED_GOAL_SOURCES
        or str(w.get("source", "")).endswith("_interior_terminal")
    ]
    forbidden = [w for w in goals if w.get("source") in FORBIDDEN_NAV_GOAL_SOURCES]
    if forbidden:
        indices = [int(w["waypoint_index"]) for w in forbidden]
        raise ValueError(f"bridge/smooth bridge waypoints selected as NavigateToPose goals: {indices}")
    return goals


def gazebo_reset_to_route_start(waypoint: dict[str, Any], model_name: str, timeout_sec: float) -> dict[str, Any]:
    result: dict[str, Any] = {
        "attempted": True,
        "success": False,
        "model_name": model_name,
        "target_pose": {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint.get("z", 0.08)),
            "yaw": float(waypoint.get("yaw", 0.0)),
        },
        "service_used": None,
        "failure_reason": None,
    }
    try:
        services = subprocess.run(
            ["ros2", "service", "list"],
            check=False,
            text=True,
            capture_output=True,
            timeout=max(3.0, min(timeout_sec, 10.0)),
        )
    except Exception as exc:
        result["failure_reason"] = f"could not list ROS services: {type(exc).__name__}: {exc}"
        return result
    service_names = set(services.stdout.splitlines())
    q = yaw_to_quat(float(waypoint.get("yaw", 0.0)))
    entity_yaml = (
        "{state: {"
        f"name: '{model_name}', "
        f"pose: {{position: {{x: {float(waypoint['x'])}, y: {float(waypoint['y'])}, z: {float(waypoint.get('z', 0.08))}}}, "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}}}, "
        "twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, "
        "reference_frame: 'world'}}"
    )
    model_yaml = (
        "{model_state: {"
        f"model_name: '{model_name}', "
        f"pose: {{position: {{x: {float(waypoint['x'])}, y: {float(waypoint['y'])}, z: {float(waypoint.get('z', 0.08))}}}, "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}}}, "
        "twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, "
        "reference_frame: 'world'}}"
    )
    candidates = [
        ("/set_entity_state", "gazebo_msgs/srv/SetEntityState", entity_yaml),
        ("/gazebo/set_entity_state", "gazebo_msgs/srv/SetEntityState", entity_yaml),
        ("/set_model_state", "gazebo_msgs/srv/SetModelState", model_yaml),
        ("/gazebo/set_model_state", "gazebo_msgs/srv/SetModelState", model_yaml),
    ]
    for service, service_type, payload in candidates:
        if service not in service_names:
            continue
        try:
            call = subprocess.run(
                ["ros2", "service", "call", service, service_type, payload],
                check=False,
                text=True,
                capture_output=True,
                timeout=timeout_sec,
            )
        except subprocess.TimeoutExpired:
            result["service_used"] = service
            result["failure_reason"] = f"{service} timed out after {timeout_sec}s"
            return result
        result["service_used"] = service
        result["service_type"] = service_type
        result["returncode"] = call.returncode
        result["stdout_tail"] = call.stdout[-1000:]
        result["stderr_tail"] = call.stderr[-1000:]
        if call.returncode == 0 and "success: True" in call.stdout:
            result["success"] = True
            result["failure_reason"] = None
            return result
        result["failure_reason"] = f"{service} returned {call.returncode}"
    if result["service_used"] is None:
        result["failure_reason"] = "no Gazebo entity/model state reset service was available"
    return result


def validate_route_artifacts(stage_output: Path, waypoints_payload: dict[str, Any] | None = None, expected_room_chain: list[str] | None = None, expected_gateway_sequence: list[str] | None = None) -> dict[str, Any]:
    room_chain = read_json(stage_output / "route/room_chain_v0_1.json")
    gateway_sequence = read_json(stage_output / "route/gateway_sequence_v0_1.json")
    resolved_route = read_json(stage_output / "route/resolved_route_v0_1.json")
    rooms = room_chain.get("room_sequence") or room_chain.get("rooms") or resolved_route.get("room_sequence")
    gateways = gateway_sequence.get("gateway_sequence") or resolved_route.get("gateway_sequence")
    source = "stage_output_route_artifacts"
    if waypoints_payload:
        rooms = waypoints_payload.get("room_sequence") or rooms
        gateways = waypoints_payload.get("gateway_sequence") or gateways
        source = "waypoints_payload"
    expected_rooms = expected_room_chain or EXPECTED_ROOM_CHAIN
    expected_gateways = expected_gateway_sequence or EXPECTED_GATEWAY_SEQUENCE
    return {
        "room_chain": rooms,
        "gateway_sequence": gateways,
        "resolved_route_path": (stage_output / "route/resolved_route_v0_1.json").as_posix(),
        "source": source,
        "expected_room_chain": expected_rooms,
        "expected_gateway_sequence": expected_gateways,
        "matches_step30p1_truth": rooms == EXPECTED_ROOM_CHAIN and gateways == EXPECTED_GATEWAY_SEQUENCE,
        "matches_expected_route": rooms == expected_rooms and gateways == expected_gateways,
    }


def run_route(args: argparse.Namespace, stage_output: Path) -> dict[str, Any]:
    waypoints_path = (args.waypoints_json or stage_output / "execution/step30p1_room_chain_h8r2_continuous_waypoints.json").resolve()
    waypoints_payload = read_json(waypoints_path)
    expected_room_chain = args.expected_room_chain.split(",") if args.expected_room_chain else None
    expected_gateway_sequence = args.expected_gateway_sequence.split(",") if args.expected_gateway_sequence else None
    route_validation = validate_route_artifacts(stage_output, waypoints_payload, expected_room_chain, expected_gateway_sequence)
    waypoints = list(waypoints_payload.get("waypoints") or [])
    semantic_goals = semantic_goals_from_waypoints(waypoints)
    base: dict[str, Any] = {
        "artifact_type": "step30s2_step30p1_route_rerun_report",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "waypoints_source": waypoints_path.as_posix(),
        "route_validation": route_validation,
        "localization_mode": "static_map_to_odom",
        "amcl_used": False,
        "direct_cmd_vel_published_by_harness": False,
        "bridge_waypoints_used_as_goals": False,
        "semantic_goal_waypoint_indices": [int(w["waypoint_index"]) for w in semantic_goals],
        "path_slice_waypoint_indices": [int(w["waypoint_index"]) for w in waypoints],
        "execute_attempted": False,
        "succeeded": False,
        "from_start_requested": bool(getattr(args, "from_start", False)),
        "reset_to_route_start_requested": bool(getattr(args, "reset_to_route_start", False)),
        "route_start_waypoint": {"waypoint_index": int(waypoints[0]["waypoint_index"]), "x": float(waypoints[0]["x"]), "y": float(waypoints[0]["y"]), "yaw": float(waypoints[0].get("yaw", 0.0))} if waypoints else None,
        "follow_path_used": False,
        "sparse_fallback_used": False,
        "fallback_resume_policy": "progress_aware_projected_path_index_forward_only",
        "fallback_started_at_selected_list_index": None,
        "fallback_started_at_waypoint_index": None,
        "fallback_backtracking_detected": False,
        "follow_path_attempts": [],
        "waypoint_results": [],
    }
    route_expected_ok = bool(route_validation["matches_expected_route"])
    if not route_expected_ok and not args.allow_non_step30p1_truth:
        base.update({"failure_layer": "route artifact", "failure_reason": "route/gateway artifacts do not match the expected Step30P1 route"})
        return base
    if IMPORT_ERROR:
        base.update({"failure_layer": "rclpy", "failure_reason": IMPORT_ERROR})
        return base
    if args.reset_to_route_start:
        base["reset_to_route_start_result"] = gazebo_reset_to_route_start(waypoints[0], args.robot_model_name, args.reset_timeout_sec)
        time.sleep(args.reset_settle_sec)

    rclpy.init(args=None)
    node = RouteNode()
    try:
        deadline = time.time() + args.pose_timeout_sec
        start_pose = None
        while rclpy.ok() and time.time() < deadline and start_pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            start_pose = node.current_pose()
        base["start_pose_observed"] = start_pose
        if start_pose is None:
            base.update({"failure_layer": "TF", "failure_reason": "could not observe robot pose"})
            return base
        base["execute_attempted"] = True
        path_start_idx, path_start_distance = nearest_path_index(waypoints, start_pose)
        base["observed_nearest_route_start_slice_index"] = path_start_idx
        base["observed_nearest_route_start_distance_m"] = round(path_start_distance, 6) if path_start_distance is not None else None
        if args.from_start:
            base["from_start_reset_verified"] = bool(
                path_start_distance is not None
                and path_start_idx <= args.from_start_max_start_slice_index
                and path_start_distance <= args.from_start_tolerance_m
            )
            if not base["from_start_reset_verified"]:
                base.update({
                    "failure_layer": "from_start_reset",
                    "failure_reason": (
                        "--from-start requested but robot pose was not near the beginning of the route "
                        f"(nearest_index={path_start_idx}, distance_m={path_start_distance})"
                    ),
                    "path_start_slice_index": path_start_idx,
                    "path_resume_start_slice_index": path_start_idx,
                })
                return base
            path_start_idx = 0
        elif path_start_distance is None or path_start_distance > args.resume_tolerance_m:
            path_start_idx = 0
        follow_path_slice = waypoints[path_start_idx:]
        base["path_start_slice_index"] = path_start_idx
        base["path_resume_start_slice_index"] = path_start_idx
        base["path_resume_start_distance_m"] = round(path_start_distance, 6) if path_start_distance is not None else None
        base["path_pose_count"] = len(follow_path_slice)
        base["follow_path_waypoint_indices"] = [int(w["waypoint_index"]) for w in follow_path_slice]
        write_json(args.latest_slice_output_json or stage_output / "post_restructure_validation/step30s2_latest_follow_path_slice.json", {
            "artifact_type": "step30s2_latest_follow_path_slice",
            "created_utc": now_iso(),
            "path_resume_start_slice_index": path_start_idx,
            "path_start_slice_index": path_start_idx,
            "waypoint_indices": [int(w["waypoint_index"]) for w in follow_path_slice],
            "poses": [{"x": float(w["x"]), "y": float(w["y"]), "source": w.get("source")} for w in follow_path_slice],
        })
        follow_ready = node.follow_path_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        nav_ready = node.nav_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        plan_ready = node.plan_client.wait_for_server(timeout_sec=3.0)
        base["action_servers"] = {"/follow_path": follow_ready, "/navigate_to_pose": nav_ready, "/compute_path_to_pose": plan_ready}
        projected_progress: int | None = None
        if follow_ready:
            base["follow_path_used"] = True
            base["execution_strategy"] = "follow_path"
            split_idx = None
            if args.split_dwell_source:
                for idx, waypoint in enumerate(waypoints):
                    if waypoint.get("source") == args.split_dwell_source:
                        split_idx = idx
                        break
            if split_idx is not None and path_start_idx < split_idx < len(waypoints) - 1:
                base["execution_strategy"] = "split_follow_path_with_through_room_interior_dwell"
                first_slice = waypoints[path_start_idx:split_idx + 1]
                second_slice = waypoints[split_idx:]
                first = node.follow_path(first_slice, args.follow_path_timeout_sec * 0.55)
                first["path_start_slice_index"] = path_start_idx
                first["path_stop_slice_index"] = split_idx
                first["split_dwell_source"] = args.split_dwell_source
                current_after_first = node.current_pose()
                first["distance_to_split_target_after_attempt_m"] = round(pose_distance(current_after_first, waypoints[split_idx]) or 999.0, 6)
                base["follow_path_attempts"].append(first)
                if not first.get("success") and first["distance_to_split_target_after_attempt_m"] > args.goal_tolerance_m:
                    base["follow_path_result"] = first
                    projected_progress, _projected_distance = nearest_path_index(waypoints, current_after_first)
                else:
                    dwell_result = node.dwell(args.split_dwell_sec, args.split_dwell_source)
                    base["split_dwell_result"] = dwell_result
                    second = node.follow_path(second_slice, args.follow_path_timeout_sec * 0.55)
                    second["path_start_slice_index"] = split_idx
                    current_after_second = node.current_pose()
                    second["distance_to_terminal_after_attempt_m"] = round(pose_distance(current_after_second, waypoints[-1]) or 999.0, 6)
                    base["follow_path_attempts"].append(second)
                    base["follow_path_result"] = second
                    if second.get("success") or second["distance_to_terminal_after_attempt_m"] <= args.goal_tolerance_m:
                        base.update({"succeeded": True, "execution_strategy": "split_follow_path_with_through_room_interior_dwell", "final_arrival_success": True})
                        return base
                    projected_progress, _projected_distance = nearest_path_index(waypoints, current_after_second)
            else:
                attempt = node.follow_path(follow_path_slice, args.follow_path_timeout_sec)
                attempt["path_start_slice_index"] = path_start_idx
                current_after = node.current_pose()
                projected_progress, projected_distance = nearest_path_index(waypoints, current_after)
                attempt["projected_next_path_start_slice_index"] = projected_progress
                attempt["projected_next_path_distance_m"] = round(projected_distance, 6) if projected_distance is not None else None
                attempt["distance_to_terminal_after_attempt_m"] = round(pose_distance(current_after, waypoints[-1]) or 999.0, 6)
                base["follow_path_attempts"].append(attempt)
                base["follow_path_result"] = attempt
                if attempt.get("success") or (pose_distance(current_after, waypoints[-1]) is not None and pose_distance(current_after, waypoints[-1]) <= args.goal_tolerance_m):
                    base.update({"succeeded": True, "execution_strategy": "follow_path", "final_arrival_success": True})
                    return base
        if not nav_ready:
            base.update({"failure_layer": "fallback", "failure_reason": "/navigate_to_pose action server not available"})
            return base
        base["sparse_fallback_used"] = True
        base["execution_strategy"] = "follow_path_then_forward_only_sparse_fallback" if base.get("follow_path_used") else "forward_only_sparse_fallback"
        current_for_fallback = node.current_pose()
        pose_progress, pose_distance_m = nearest_path_index(waypoints, current_for_fallback)
        candidates = [value for value in [projected_progress, pose_progress] if value is not None]
        projected_progress = max(candidates) if candidates else None
        start_idx = first_semantic_index_at_or_after_progress(semantic_goals, projected_progress)
        required_interior_indices = [
            idx for idx, goal in enumerate(semantic_goals)
            if str(goal.get("source", "")).endswith("_interior_terminal")
        ]
        if args.enforce_interior_targets and required_interior_indices:
            unvisited_required = [
                idx for idx in required_interior_indices
                if projected_progress is None or int(semantic_goals[idx]["waypoint_index"]) >= max(0, int(projected_progress) - args.interior_target_progress_guard_window)
            ]
            if unvisited_required:
                start_idx = min(start_idx, min(unvisited_required))
        base["projected_next_path_start_slice_index"] = projected_progress
        base["current_pose_projected_path_start_slice_index"] = pose_progress
        base["current_pose_projected_path_distance_m"] = round(pose_distance_m, 6) if pose_distance_m is not None else None
        base["fallback_started_at_selected_list_index"] = start_idx
        base["fallback_started_at_waypoint_index"] = int(semantic_goals[start_idx]["waypoint_index"]) if start_idx < len(semantic_goals) else None
        base["fallback_skipped_completed_semantic_waypoints"] = [int(w["waypoint_index"]) for w in semantic_goals[:start_idx]]
        required_goal_selected_indices = set(required_interior_indices if args.enforce_interior_targets else [])
        for selected_idx, waypoint in enumerate(semantic_goals):
            row = {
                "selected_list_index": selected_idx,
                "waypoint_index": int(waypoint["waypoint_index"]),
                "source": waypoint.get("source"),
                "gateway_id": waypoint.get("gateway_id"),
                "from_room": waypoint.get("from_room"),
                "to_room": waypoint.get("to_room"),
                "plan": {"attempted": False},
                "navigate": {"attempted": False},
            }
            if selected_idx < start_idx:
                row["navigate"] = {"attempted": False, "status": "skipped_forward_progress", "success": True}
                base["waypoint_results"].append(row)
                continue
            if projected_progress is not None and int(waypoint["waypoint_index"]) < int(projected_progress) and selected_idx not in required_goal_selected_indices:
                row["navigate"] = {"attempted": False, "status": "blocked_by_forward_progress_guard", "success": False, "failure_reason": "fallback goal is behind projected FollowPath progress"}
                base["waypoint_results"].append(row)
                base.update({"fallback_backtracking_detected": True, "failure_layer": "fallback", "failure_reason": row["navigate"]["failure_reason"]})
                return base
            if args.plan_before_goal and plan_ready:
                row["plan"] = node.compute_plan(waypoint, args.plan_timeout_sec)
            row["navigate"] = node.navigate(waypoint, args.goal_timeout_sec)
            if row["navigate"].get("success") and waypoint.get("source") == args.split_dwell_source and args.split_dwell_sec > 0.0:
                row["dwell_after_success"] = node.dwell(args.split_dwell_sec, args.split_dwell_source)
            row["final_distance_to_goal_m"] = round(pose_distance(node.current_pose(), waypoint) or 999.0, 6)
            base["waypoint_results"].append(row)
            if not row["navigate"].get("success"):
                recovery_pose = node.current_pose()
                recovery_idx, recovery_dist = nearest_path_index(waypoints, recovery_pose)
                recovery_slice = waypoints[recovery_idx:]
                recovery = {
                    "attempted": bool(follow_ready and len(recovery_slice) >= 2),
                    "reason": "navigate_to_pose_fallback_failed_use_remaining_continuous_follow_path",
                    "path_start_slice_index": recovery_idx,
                    "path_start_distance_m": round(recovery_dist, 6) if recovery_dist is not None else None,
                    "path_pose_count": len(recovery_slice),
                }
                if recovery["attempted"]:
                    recovery.update(node.follow_path(recovery_slice, args.follow_path_timeout_sec))
                    current_after_recovery = node.current_pose()
                    recovery["distance_to_terminal_after_attempt_m"] = round(pose_distance(current_after_recovery, waypoints[-1]) or 999.0, 6)
                    base["follow_path_attempts"].append(recovery)
                    base["remaining_follow_path_recovery"] = recovery
                    if recovery.get("success") or (pose_distance(current_after_recovery, waypoints[-1]) is not None and pose_distance(current_after_recovery, waypoints[-1]) <= args.goal_tolerance_m):
                        base.update({"succeeded": True, "execution_strategy": "follow_path_then_forward_only_sparse_fallback_then_remaining_follow_path_recovery", "final_arrival_success": True})
                        return base
                else:
                    base["remaining_follow_path_recovery"] = recovery
                base.update({"failure_layer": "fallback", "failure_reason": row["navigate"].get("failure_reason") or "NavigateToPose fallback failed", "first_failed_waypoint": row})
                return base
        base["succeeded"] = True
        base["sparse_fallback_clean_resume_passed"] = not base["fallback_backtracking_detected"]
        base["final_arrival_success"] = pose_distance(node.current_pose(), waypoints[-1]) is not None and pose_distance(node.current_pose(), waypoints[-1]) <= args.goal_tolerance_m
        return base
    finally:
        final_pose = None
        for _ in range(10):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
            final_pose = node.current_pose() or final_pose
        base["final_pose_observed"] = final_pose
        if base.get("start_pose_observed") and final_pose:
            base["robot_moved_distance_m"] = round(pose_distance(base["start_pose_observed"], final_pose) or 0.0, 6)
        base["trajectory_sample_count"] = len(node.trajectory_samples)
        base["trajectory_samples"] = node.trajectory_samples
        try:
            node.nav_client.destroy()
            node.plan_client.destroy()
            node.follow_path_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--server-timeout-sec", type=float, default=25.0)
    parser.add_argument("--pose-timeout-sec", type=float, default=12.0)
    parser.add_argument("--follow-path-timeout-sec", type=float, default=240.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=90.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=20.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.30)
    parser.add_argument("--resume-tolerance-m", type=float, default=1.0)
    parser.add_argument("--from-start", action="store_true", help="Require the robot to be at the beginning of the route before executing.")
    parser.add_argument("--reset-to-route-start", action="store_true", help="Try to reset the Gazebo TurtleBot3 entity to route waypoint 0 before executing.")
    parser.add_argument("--robot-model-name", default="turtlebot3_burger")
    parser.add_argument("--from-start-tolerance-m", type=float, default=1.0)
    parser.add_argument("--from-start-max-start-slice-index", type=int, default=3)
    parser.add_argument("--reset-timeout-sec", type=float, default=20.0)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--plan-before-goal", action="store_true", default=True)
    parser.add_argument("--split-dwell-source", default="", help="Split FollowPath at this waypoint source and hold there before continuing.")
    parser.add_argument("--split-dwell-sec", type=float, default=0.0)
    parser.add_argument("--waypoints-json", type=Path, help="Use a generated route waypoint payload instead of the historical Step30P1 waypoint file.")
    parser.add_argument("--expected-room-chain", help="Comma-separated room chain expected for this run.")
    parser.add_argument("--expected-gateway-sequence", help="Comma-separated gateway sequence expected for this run.")
    parser.add_argument("--allow-non-step30p1-truth", action="store_true", help="Allow parametric routes that intentionally differ from the accepted Step30P1 historical room chain.")
    parser.add_argument("--enforce-interior-targets", action="store_true", default=True, help="Do not let progress projection skip explicit through-room/terminal interior targets.")
    parser.add_argument("--interior-target-progress-guard-window", type=int, default=12)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--trajectory-output-json", type=Path)
    parser.add_argument("--latest-slice-output-json", type=Path)
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    out_dir = stage_output / "post_restructure_validation"
    output_json = args.output_json or out_dir / "step30s2_step30p1_route_rerun_report_v0_1.json"
    output_md = args.output_md or out_dir / "step30s2_step30p1_route_rerun_report_v0_1.md"
    trajectory_json = args.trajectory_output_json or out_dir / "step30s2_step30p1_route_rerun_trajectory_v0_1.json"
    payload = run_route(args, stage_output)
    write_json(output_json, payload)
    write_md(output_md, payload)
    write_json(trajectory_json, {"artifact_type": "step30s2_step30p1_route_rerun_trajectory", "created_utc": now_iso(), "samples": payload.get("trajectory_samples", [])})
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("succeeded") else 1


if __name__ == "__main__":
    sys.exit(main())
