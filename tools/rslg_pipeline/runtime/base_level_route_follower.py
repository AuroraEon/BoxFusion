#!/usr/bin/env python3
"""Bounded base-level route follower for RSLG-SLAM Gazebo smoke tests."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from tools.rslg_pipeline.project_truth import NON_TRANSITION_EDGE, TRUE_TRANSITION_EDGE
from tools.rslg_pipeline.runtime.route_result_adapter_common import is_route_result
from tools.rslg_pipeline.runtime.route_result_runtime_adapter import (
    SCHEMA_NAME as PID_RUNTIME_INPUT_SCHEMA,
    build_pid_runtime_input,
)
from tools.rslg_pipeline.runtime.route_result_z_aware_adapter import (
    OVERLAY_SCHEMA,
    build_z_aware_overlay_input,
)
from tools.rslg_pipeline.runtime.z_aware_route_projection import (
    overlay_waypoints,
    visual_z_for_segment,
)
from tools.rslg_pipeline.viz.z_aware_trajectory_markers import (
    build_z_aware_current_waypoint_marker,
    build_z_aware_executed_trajectory_marker,
    build_z_aware_planned_route_marker,
    build_z_aware_robot_markers,
    build_z_aware_transition_band_marker,
)

try:
    from gazebo_msgs.msg import ModelStates
except Exception:  # pragma: no cover - optional fallback dependency
    ModelStates = None  # type: ignore[assignment]

try:
    from tf2_ros import TransformBroadcaster
except Exception:  # pragma: no cover - optional TF dependency
    TransformBroadcaster = None  # type: ignore[assignment]


FLOOR_Z_DEFAULTS = {"floor_1": 0.0, "floor_2": 1.6, "floor_transition": 0.8}
VALID_CLASSIFICATIONS = {
    "gazebo_base_level_route_following_smoke_passed",
    "gazebo_base_level_route_following_partial",
    "gazebo_pose_stepped_route_smoke_passed_with_limits",
    "rviz_only_route_pose_replay_not_enough",
    "failed_environment",
    "failed_pipeline",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "y", "on"}:
        return True
    if normalized in {"false", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def distance_xy(a: dict[str, float], b: dict[str, float]) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def point(x: float, y: float, z: float) -> Point:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def yaw_between(points: list[dict[str, Any]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    before = points[index - 1] if index == len(points) - 1 else points[index]
    after = points[index] if index == len(points) - 1 else points[index + 1]
    return math.atan2(float(after["y"]) - float(before["y"]), float(after["x"]) - float(before["x"]))


def flatten_route_segments(route: dict[str, Any]) -> list[dict[str, Any]]:
    waypoints: list[dict[str, Any]] = []
    waypoint_index = 0
    for segment in route.get("same_floor_segments") or []:
        floor_id = segment.get("floor_id")
        for raw_point in segment.get("waypoints") or []:
            if waypoints and raw_point.get("x") == waypoints[-1].get("x") and raw_point.get("y") == waypoints[-1].get("y"):
                continue
            waypoints.append(
                {
                    "waypoint_index": waypoint_index,
                    "floor_id": floor_id,
                    "segment_id": segment.get("segment_id"),
                    "x": raw_point["x"],
                    "y": raw_point["y"],
                    "source": "layer3_real_route_segment",
                }
            )
            waypoint_index += 1
    return waypoints


def load_runtime_input(source: dict[str, Any]) -> dict[str, Any]:
    """Return a RouteResult-derived PID runtime input payload.

    Accepts the formal ``rslg_route_result_pid_runtime_input`` payload directly, or
    converts an ``rslg_route_result`` in-place through the PID runtime adapter. The
    old task48 route-executor input shape is no longer accepted as a formal input.
    """

    schema = source.get("schema_name")
    if schema == PID_RUNTIME_INPUT_SCHEMA:
        return source
    if is_route_result(source):
        source_ref = (source.get("identity") or {}).get("query_id")
        return build_pid_runtime_input(source, source_ref=source_ref)
    raise ValueError(
        f"expected {PID_RUNTIME_INPUT_SCHEMA!r} or rslg_route_result, got schema_name={schema!r}"
    )


def pid_runtime_waypoints(runtime_input: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the executable PID waypoints (x, y, z, yaw, floor_id) from the input.

    Same-floor and object-approach segments become executable waypoints. The
    vertical connector is a separate semantic handoff and is NOT expanded into a
    physical stair-climb trajectory here.
    """

    floor_z_map = runtime_input.get("floor_z_map") or FLOOR_Z_DEFAULTS
    raw = runtime_input.get("runtime_waypoints") or []
    if not raw:
        raise ValueError("runtime input has no runtime_waypoints")

    waypoints: list[dict[str, Any]] = []
    for index, wp in enumerate(raw):
        floor_id = str(wp.get("floor_id") or "floor_2")
        record = dict(wp)
        record["waypoint_index"] = int(wp.get("waypoint_index", index))
        record["floor_id"] = floor_id
        record["x"] = float(wp["x"])
        record["y"] = float(wp["y"])
        record["z"] = float(wp.get("z", floor_z_map.get(floor_id, 0.0)))
        record["yaw"] = float(wp.get("yaw", 0.0))
        waypoints.append(record)

    goal = runtime_input.get("selected_goal") or runtime_input.get("endpoint")
    if isinstance(goal, dict) and "x" in goal and waypoints:
        cid = goal.get("candidate_id")
        if cid == "generated_ring_037":
            raise ValueError("generated_ring_037 is blocked evidence only and cannot be a runtime goal")
        final = waypoints[-1]
        goal_floor = str(goal.get("floor_id") or "floor_2")
        goal_z = float(goal.get("z", floor_z_map.get(goal_floor, 1.6)))
        if math.hypot(final["x"] - float(goal["x"]), final["y"] - float(goal["y"])) > 0.5:
            waypoints.append(
                {
                    "waypoint_index": int(final["waypoint_index"]) + 1,
                    "floor_id": goal_floor,
                    "segment_id": "selected_object_approach_goal",
                    "x": float(goal["x"]),
                    "y": float(goal["y"]),
                    "z": goal_z,
                    "yaw": float(goal.get("yaw", final.get("yaw", 0.0))),
                    "source": "selected_object_approach_goal",
                    "runtime_goal_candidate_id": cid,
                    "runtime_goal_role": "final_object_approach_goal",
                }
            )
        else:
            final["runtime_goal_candidate_id"] = cid
            final["runtime_goal_role"] = "final_object_approach_goal"
    return waypoints


def pid_route_validation(
    runtime_input: dict[str, Any], waypoints: list[dict[str, Any]]
) -> dict[str, Any]:
    """Validate RSLG-SLAM route truth over the PID runtime input and waypoints."""

    route_type = (runtime_input.get("identity") or {}).get("query_type")
    handoffs = runtime_input.get("connector_handoffs") or []
    transition_edges = [h.get("transition_edge") for h in handoffs if h.get("transition_edge")]
    e001_transition = any(edge == TRUE_TRANSITION_EDGE for edge in transition_edges) if transition_edges else False
    e003_transition = any(edge == NON_TRANSITION_EDGE for edge in transition_edges)

    goal = runtime_input.get("selected_goal") or {}
    ring_037_runtime_goal = any(
        wp.get("runtime_goal_candidate_id") == "generated_ring_037" for wp in waypoints
    ) or (isinstance(goal, dict) and goal.get("candidate_id") == "generated_ring_037")
    final_generated_ring_002 = (
        route_type != "cross_floor_object"
        or (bool(waypoints) and waypoints[-1].get("runtime_goal_candidate_id") == "generated_ring_002")
    )

    validation = {
        "route_type": route_type,
        "waypoint_count": len(waypoints),
        "generated_ring_002_final_target": bool(final_generated_ring_002),
        "generated_ring_037_runtime_goal_used": bool(ring_037_runtime_goal),
        "vt_1_centerline_e001_transition_handoff_edge": bool(e001_transition),
        "vt_1_centerline_e003_transition_used": bool(e003_transition),
        "floor_order": (runtime_input.get("identity") or {}).get("query_type"),
    }
    if ring_037_runtime_goal:
        raise ValueError("generated_ring_037 is blocked evidence only and cannot be a runtime goal")
    if route_type == "cross_floor_object" and not final_generated_ring_002:
        raise ValueError("cross_floor_object final target is not generated_ring_002")
    if handoffs and (e003_transition or not e001_transition):
        raise ValueError("transition edge truth is invalid for this runtime input")
    return validation


def load_z_aware_waypoints(
    *,
    z_aware_overlay: Optional[dict[str, Any]],
    route_result: Optional[dict[str, Any]],
    floor1_z: float,
    floor2_z: float,
    transition_edge: str,
) -> list[dict[str, Any]]:
    """Return z-aware overlay waypoints from an overlay input or a RouteResult."""

    if z_aware_overlay is not None:
        return overlay_waypoints(z_aware_overlay)
    if route_result is not None and is_route_result(route_result):
        overlay = build_z_aware_overlay_input(
            route_result,
            floor_z_map={"floor_1": float(floor1_z), "floor_2": float(floor2_z)},
            transition_edge=transition_edge,
        )
        return list(overlay.get("overlay_waypoints") or [])
    return []


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    source: str
    stamp_sec: float
    z: float = 0.0

    def as_dict(self) -> dict[str, float | str]:
        return {
            "x": round(self.x, 6),
            "y": round(self.y, 6),
            "z": round(self.z, 6),
            "yaw": round(self.yaw, 6),
            "source": self.source,
            "stamp_sec": round(self.stamp_sec, 6),
        }


class BaseLevelRouteFollower(Node):
    def __init__(
        self,
        *,
        waypoints: list[dict[str, Any]],
        route_input: dict[str, Any],
        route_type: str,
        output_dir: Path,
        cmd_vel_topic: str,
        odom_topic: str,
        frame_id: str,
        robot_frame_id: str,
        model_state_topic: str,
        robot_model_name: str,
        goal_tolerance_m: float,
        yaw_tolerance_rad: float,
        max_linear_speed: float,
        max_angular_speed: float,
        max_duration_sec: float,
        require_odom: bool,
        fallback_gazebo_model_state: bool,
        fallback_pose_stepping: bool,
        publish_rate_hz: float,
        z_aware_visualization: bool = False,
        z_aware_waypoints: Optional[list[dict[str, Any]]] = None,
        vertical_transition_edge: str = TRUE_TRANSITION_EDGE,
        floor1_z: float = 0.0,
        floor2_z: float = 1.6,
        transition_mode: str = "visual_marker",
        publish_z_aware_trajectory: bool = True,
        publish_z_aware_robot_marker: bool = True,
        physical_climb_claim: bool = False,
        z_aware_topic: str = "/rslg_slam/z_aware_navigation_markers",
        z_aware_robot_frame_id: str = "z_aware_robot",
    ) -> None:
        super().__init__("rslg_slam_base_level_route_follower")
        self.route_input = route_input
        self.waypoints = waypoints
        self.route_type = route_type
        self.output_dir = output_dir
        self.cmd_vel_topic = cmd_vel_topic
        self.odom_topic = odom_topic
        self.frame_id = frame_id
        self.robot_frame_id = robot_frame_id
        self.model_state_topic = model_state_topic
        self.robot_model_name = robot_model_name
        self.goal_tolerance_m = goal_tolerance_m
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_duration_sec = max_duration_sec
        self.require_odom = require_odom
        self.fallback_gazebo_model_state = fallback_gazebo_model_state
        self.fallback_pose_stepping = fallback_pose_stepping
        self.publish_rate_hz = publish_rate_hz

        # Z-aware vertical-transition visualization overlay (task48j). This lifts a
        # separate visual trajectory/robot marker to a route-derived z; it does NOT
        # move the Gazebo physics robot and does NOT claim physical stair climbing.
        self.z_aware_visualization = z_aware_visualization
        self.z_aware_waypoints = z_aware_waypoints or []
        self.vertical_transition_edge = vertical_transition_edge
        self.floor1_z = float(floor1_z)
        self.floor2_z = float(floor2_z)
        self.transition_mode = transition_mode
        self.publish_z_aware_trajectory = publish_z_aware_trajectory
        self.publish_z_aware_robot_marker = publish_z_aware_robot_marker
        self.physical_climb_claim = bool(physical_climb_claim)
        self.z_aware_topic = z_aware_topic
        self.z_aware_robot_frame_id = z_aware_robot_frame_id
        self.z_aware_trajectory_points: list[tuple[float, float, float]] = []
        self.z_aware_samples: list[dict[str, Any]] = []
        self.z_aware_visual_z_min: Optional[float] = None
        self.z_aware_visual_z_max: Optional[float] = None
        self.physical_z_min: Optional[float] = None
        self.physical_z_max: Optional[float] = None
        self.z_aware_transition_entered = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/rslg_slam/live_navigation_markers", 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.model_state_sub = None
        if fallback_gazebo_model_state and ModelStates is not None:
            self.model_state_sub = self.create_subscription(ModelStates, model_state_topic, self.on_model_states, 20)

        self.z_aware_marker_pub = None
        self.tf_broadcaster = None
        if self.z_aware_visualization:
            self.z_aware_marker_pub = self.create_publisher(MarkerArray, self.z_aware_topic, 10)
            if self.transition_mode == "visual_tf" and TransformBroadcaster is not None:
                self.tf_broadcaster = TransformBroadcaster(self)

        self.current_pose: Optional[Pose2D] = None
        self.first_pose: Optional[Pose2D] = None
        self.last_odom_pose: Optional[Pose2D] = None
        self.last_model_state_pose: Optional[Pose2D] = None
        self.odom_received = False
        self.model_state_received = False
        self.cmd_vel_publish_count = 0
        self.nonzero_cmd_vel_publish_count = 0
        self.current_waypoint_index = 0
        self.waypoint_events: list[dict[str, Any]] = []
        self.trajectory_samples: list[dict[str, Any]] = []
        self.cmd_vel_samples: list[dict[str, Any]] = []
        self.done = False
        self.timed_out = False
        self.abort_reason: Optional[str] = None
        self.started_monotonic = time.monotonic()
        self.last_sample_monotonic = 0.0
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 0.1), self.control_step)

        self.trajectory_path = output_dir / "trajectory_samples.jsonl"
        self.cmd_path = output_dir / "cmd_vel_samples.jsonl"
        self.events_path = output_dir / "waypoint_reach_events.jsonl"
        self.trajectory_handle = self.trajectory_path.open("w", encoding="utf-8")
        self.cmd_handle = self.cmd_path.open("w", encoding="utf-8")
        self.events_handle = self.events_path.open("w", encoding="utf-8")
        self.z_aware_trajectory_path = output_dir / "z_aware_trajectory_samples.jsonl"
        self.z_aware_handle = (
            self.z_aware_trajectory_path.open("w", encoding="utf-8") if self.z_aware_visualization else None
        )

    def on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
            source="odom",
            stamp_sec=self.get_clock().now().nanoseconds / 1e9,
        )
        self.odom_received = True
        self.last_odom_pose = pose
        self.current_pose = pose
        self._track_physical_z(pose.z)
        if self.first_pose is None:
            self.first_pose = pose

    def on_model_states(self, msg: Any) -> None:
        try:
            index = list(msg.name).index(self.robot_model_name)
        except ValueError:
            return
        pose_msg = msg.pose[index]
        orientation = pose_msg.orientation
        pose = Pose2D(
            x=float(pose_msg.position.x),
            y=float(pose_msg.position.y),
            z=float(pose_msg.position.z),
            yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
            source="gazebo_model_state",
            stamp_sec=self.get_clock().now().nanoseconds / 1e9,
        )
        self.model_state_received = True
        self.last_model_state_pose = pose
        if not self.odom_received:
            self.current_pose = pose
            self._track_physical_z(pose.z)
            if self.first_pose is None:
                self.first_pose = pose

    def _track_physical_z(self, z: float) -> None:
        self.physical_z_min = z if self.physical_z_min is None else min(self.physical_z_min, z)
        self.physical_z_max = z if self.physical_z_max is None else max(self.physical_z_max, z)

    def write_jsonl(self, handle: Any, record: dict[str, Any]) -> None:
        handle.write(json.dumps(record, sort_keys=False) + "\n")
        handle.flush()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())
        self.cmd_vel_publish_count += 1
        record = {
            "sample_index": len(self.cmd_vel_samples),
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
            "linear_x": 0.0,
            "angular_z": 0.0,
            "reason": "safe_stop",
        }
        self.cmd_vel_samples.append(record)
        self.write_jsonl(self.cmd_handle, record)

    def publish_cmd(self, linear_x: float, angular_z: float, reason: str) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.cmd_vel_publish_count += 1
        if abs(linear_x) > 1e-5 or abs(angular_z) > 1e-5:
            self.nonzero_cmd_vel_publish_count += 1
        record = {
            "sample_index": len(self.cmd_vel_samples),
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
            "linear_x": round(float(linear_x), 6),
            "angular_z": round(float(angular_z), 6),
            "reason": reason,
            "waypoint_index": self.current_waypoint_index,
        }
        self.cmd_vel_samples.append(record)
        self.write_jsonl(self.cmd_handle, record)

    def log_pose_sample(self, pose: Pose2D) -> None:
        now = time.monotonic()
        if now - self.last_sample_monotonic < 0.2:
            return
        self.last_sample_monotonic = now
        target = self.current_target()
        record = {
            "sample_index": len(self.trajectory_samples),
            "elapsed_sec": round(now - self.started_monotonic, 3),
            "pose": pose.as_dict(),
            "current_waypoint_index": self.current_waypoint_index,
            "target_waypoint_index": target.get("waypoint_index") if target else None,
            "target_floor_id": target.get("floor_id") if target else None,
            "target_x": round(float(target["x"]), 6) if target else None,
            "target_y": round(float(target["y"]), 6) if target else None,
        }
        self.trajectory_samples.append(record)
        self.write_jsonl(self.trajectory_handle, record)

    def log_waypoint_reached(self, waypoint: dict[str, Any], pose: Pose2D, distance_m: float, final_yaw_error: Optional[float] = None) -> None:
        record = {
            "event_index": len(self.waypoint_events),
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
            "waypoint_index": int(waypoint.get("waypoint_index", self.current_waypoint_index)),
            "route_list_index": self.current_waypoint_index,
            "floor_id": waypoint.get("floor_id"),
            "segment_id": waypoint.get("segment_id"),
            "x": round(float(waypoint["x"]), 6),
            "y": round(float(waypoint["y"]), 6),
            "z": round(float(waypoint.get("z", 0.0)), 6),
            "distance_m": round(float(distance_m), 6),
            "pose_source": pose.source,
            "final_yaw_error_rad": round(float(final_yaw_error), 6) if final_yaw_error is not None else None,
            "runtime_goal_candidate_id": waypoint.get("runtime_goal_candidate_id"),
        }
        self.waypoint_events.append(record)
        self.write_jsonl(self.events_handle, record)

    def current_target(self) -> Optional[dict[str, Any]]:
        if self.current_waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_index]

    def marker_common(self, marker: Marker, stamp: Any, ns: str, marker_id: int, marker_type: int) -> None:
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = Duration(seconds=1.0).to_msg()

    def publish_navigation_markers(self, pose: Optional[Pose2D]) -> None:
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        trajectory = Marker()
        self.marker_common(trajectory, stamp, "rslg_base_level_executed_trajectory", 1, Marker.LINE_STRIP)
        trajectory.scale.x = 0.055
        trajectory.color.r = 0.1
        trajectory.color.g = 0.95
        trajectory.color.b = 0.45
        trajectory.color.a = 1.0
        for sample in self.trajectory_samples[-500:]:
            pose_sample = sample["pose"]
            trajectory.points.append(point(pose_sample["x"], pose_sample["y"], 0.08))
        array.markers.append(trajectory)

        if pose is not None:
            robot = Marker()
            self.marker_common(robot, stamp, "rslg_base_level_robot_pose", 2, Marker.ARROW)
            robot.pose.position.x = pose.x
            robot.pose.position.y = pose.y
            robot.pose.position.z = 0.24
            qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
            robot.pose.orientation.x = qx
            robot.pose.orientation.y = qy
            robot.pose.orientation.z = qz
            robot.pose.orientation.w = qw
            robot.scale.x = 0.55
            robot.scale.y = 0.08
            robot.scale.z = 0.08
            robot.color.r = 0.02
            robot.color.g = 0.85
            robot.color.b = 1.0
            robot.color.a = 1.0
            array.markers.append(robot)

        target = self.current_target()
        if target:
            goal = Marker()
            self.marker_common(goal, stamp, "rslg_base_level_current_waypoint", 3, Marker.SPHERE)
            goal.pose.position.x = float(target["x"])
            goal.pose.position.y = float(target["y"])
            goal.pose.position.z = 0.18
            goal.scale.x = 0.24
            goal.scale.y = 0.24
            goal.scale.z = 0.24
            goal.color.r = 1.0
            goal.color.g = 0.9
            goal.color.b = 0.1
            goal.color.a = 0.95
            array.markers.append(goal)

        self.marker_pub.publish(array)

    def _record_z_aware_visual_z(self, z: float) -> None:
        self.z_aware_visual_z_min = z if self.z_aware_visual_z_min is None else min(self.z_aware_visual_z_min, z)
        self.z_aware_visual_z_max = z if self.z_aware_visual_z_max is None else max(self.z_aware_visual_z_max, z)

    def current_visual_z(self, pose: Pose2D) -> float:
        """Route-derived visual z for the current pose (overlay only, not physics)."""
        if not self.z_aware_waypoints:
            return self.floor1_z
        idx = min(self.current_waypoint_index, len(self.z_aware_waypoints) - 1)
        return visual_z_for_segment(self.z_aware_waypoints, idx, pose.x, pose.y)

    def broadcast_z_aware_tf(self, pose: Pose2D, visual_z: float, stamp: Any) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.z_aware_robot_frame_id
        transform.transform.translation.x = float(pose.x)
        transform.transform.translation.y = float(pose.y)
        transform.transform.translation.z = float(visual_z)
        qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def publish_z_aware_markers(self, pose: Optional[Pose2D]) -> None:
        if not self.z_aware_visualization or self.z_aware_marker_pub is None:
            return
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        array.markers.append(build_z_aware_planned_route_marker(self.z_aware_waypoints, frame_id=self.frame_id, stamp=stamp))
        band = build_z_aware_transition_band_marker(self.z_aware_waypoints, frame_id=self.frame_id, stamp=stamp)
        if band is not None:
            array.markers.append(band)

        if pose is not None:
            visual_z = self.current_visual_z(pose)
            self._record_z_aware_visual_z(visual_z)
            phase = None
            idx = min(self.current_waypoint_index, len(self.z_aware_waypoints) - 1)
            if self.z_aware_waypoints:
                phase = self.z_aware_waypoints[idx].get("route_phase")
                if phase == "vertical_transition":
                    self.z_aware_transition_entered = True
            self.z_aware_trajectory_points.append((pose.x, pose.y, visual_z))
            record = {
                "sample_index": len(self.z_aware_samples),
                "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
                "physical_x": round(pose.x, 6),
                "physical_y": round(pose.y, 6),
                "physical_z": round(pose.z, 6),
                "z_aware_visual_z": round(visual_z, 6),
                "current_waypoint_index": self.current_waypoint_index,
                "route_phase": phase,
                "transition_edge_id": self.vertical_transition_edge if phase == "vertical_transition" else None,
            }
            self.z_aware_samples.append(record)
            if self.z_aware_handle is not None:
                self.write_jsonl(self.z_aware_handle, record)

            if self.publish_z_aware_trajectory and self.z_aware_trajectory_points:
                array.markers.append(
                    build_z_aware_executed_trajectory_marker(self.z_aware_trajectory_points, frame_id=self.frame_id, stamp=stamp)
                )
            if self.publish_z_aware_robot_marker:
                for marker in build_z_aware_robot_markers(pose.x, pose.y, visual_z, pose.yaw, frame_id=self.frame_id, stamp=stamp):
                    array.markers.append(marker)
            target = self.current_target()
            if target is not None and self.z_aware_waypoints:
                z_target = dict(self.z_aware_waypoints[idx])
                z_target["x"] = float(target["x"])
                z_target["y"] = float(target["y"])
                array.markers.append(build_z_aware_current_waypoint_marker(z_target, frame_id=self.frame_id, stamp=stamp))
            self.broadcast_z_aware_tf(pose, visual_z, stamp)

        self.z_aware_marker_pub.publish(array)

    def control_step(self) -> None:
        if self.done:
            return
        elapsed = time.monotonic() - self.started_monotonic
        if elapsed >= self.max_duration_sec:
            self.timed_out = True
            self.abort_reason = "max_duration_sec_reached"
            self.publish_stop()
            self.done = True
            return

        pose = self.current_pose
        if pose is None:
            self.publish_cmd(0.0, 0.0, "waiting_for_pose")
            if elapsed > 10.0 and self.require_odom and not self.odom_received:
                self.abort_reason = "required_odom_not_received"
                self.done = True
                self.publish_stop()
            elif elapsed > 20.0 and not self.require_odom and not self.fallback_gazebo_model_state:
                self.abort_reason = "no_pose_source_available"
                self.done = True
                self.publish_stop()
            self.publish_navigation_markers(None)
            self.publish_z_aware_markers(None)
            return

        self.log_pose_sample(pose)
        self.publish_navigation_markers(pose)
        self.publish_z_aware_markers(pose)
        target = self.current_target()
        if target is None:
            self.publish_stop()
            self.done = True
            return

        dx = float(target["x"]) - pose.x
        dy = float(target["y"]) - pose.y
        dist = math.hypot(dx, dy)
        is_final = self.current_waypoint_index >= len(self.waypoints) - 1
        if dist <= self.goal_tolerance_m:
            if is_final:
                yaw_error = normalize_angle(float(target.get("yaw", pose.yaw)) - pose.yaw)
                if abs(yaw_error) <= self.yaw_tolerance_rad:
                    self.log_waypoint_reached(target, pose, dist, yaw_error)
                    self.current_waypoint_index += 1
                    self.publish_stop()
                    self.done = True
                    return
                angular = clamp(1.6 * yaw_error, self.max_angular_speed)
                self.publish_cmd(0.0, angular, "final_yaw_alignment")
                return

            self.log_waypoint_reached(target, pose, dist)
            self.current_waypoint_index += 1
            self.publish_cmd(0.0, 0.0, "waypoint_reached")
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - pose.yaw)
        angular = clamp(1.7 * heading_error, self.max_angular_speed)
        if abs(heading_error) > 0.85:
            linear = 0.0
            reason = "turning_to_waypoint"
        else:
            slowdown = max(0.2, math.cos(heading_error))
            linear = min(self.max_linear_speed, max(0.035, 0.7 * dist)) * slowdown
            reason = "tracking_waypoint"
        self.publish_cmd(linear, angular, reason)

    def close(self) -> None:
        self.trajectory_handle.close()
        self.cmd_handle.close()
        self.events_handle.close()
        if self.z_aware_handle is not None:
            self.z_aware_handle.close()

    def z_aware_summary(self) -> dict[str, Any]:
        z_min = round(self.z_aware_visual_z_min, 6) if self.z_aware_visual_z_min is not None else None
        z_max = round(self.z_aware_visual_z_max, 6) if self.z_aware_visual_z_max is not None else None
        transition_detected = bool(
            self.z_aware_visualization
            and self.z_aware_transition_entered
            and z_min is not None
            and z_max is not None
            and (z_max - z_min) > 0.5
        )
        return {
            "z_aware_visualization_enabled": self.z_aware_visualization,
            "transition_mode": self.transition_mode if self.z_aware_visualization else None,
            "vertical_transition_edge": self.vertical_transition_edge,
            "transition_edge_used": self.vertical_transition_edge if self.z_aware_visualization else None,
            "non_transition_edge_used": self.vertical_transition_edge == NON_TRANSITION_EDGE,
            "floor1_z": self.floor1_z,
            "floor2_z": self.floor2_z,
            "z_aware_topic": self.z_aware_topic if self.z_aware_visualization else None,
            "z_aware_sample_count": len(self.z_aware_samples),
            "physical_trajectory_z_min": round(self.physical_z_min, 6) if self.physical_z_min is not None else None,
            "physical_trajectory_z_max": round(self.physical_z_max, 6) if self.physical_z_max is not None else None,
            "z_aware_visual_trajectory_z_min": z_min,
            "z_aware_visual_trajectory_z_max": z_max,
            "z_aware_visual_transition_detected": transition_detected,
            "physical_climb_claimed": bool(self.physical_climb_claim),
            "z_aware_visual_overlay": self.z_aware_visualization,
            "z_aware_trajectory_samples_file": self.z_aware_trajectory_path.as_posix() if self.z_aware_visualization else None,
        }

    def route_completed(self) -> bool:
        return self.current_waypoint_index >= len(self.waypoints)

    def pose_changed_distance(self) -> float:
        if not self.first_pose or not self.current_pose:
            return 0.0
        return math.hypot(self.current_pose.x - self.first_pose.x, self.current_pose.y - self.first_pose.y)

    def endpoint_distance(self) -> Optional[float]:
        if not self.current_pose or not self.waypoints:
            return None
        final = self.waypoints[-1]
        return math.hypot(self.current_pose.x - float(final["x"]), self.current_pose.y - float(final["y"]))

    def classification(self) -> str:
        pose_changed = self.pose_changed_distance() > 0.05
        cmd_vel_published = self.nonzero_cmd_vel_publish_count > 0
        several_attempted = self.current_waypoint_index >= 3 or len(self.cmd_vel_samples) >= 10
        if self.fallback_pose_stepping:
            return "gazebo_pose_stepped_route_smoke_passed_with_limits" if pose_changed else "failed_environment"
        if cmd_vel_published and pose_changed and several_attempted and self.trajectory_samples:
            if self.route_completed():
                return "gazebo_base_level_route_following_smoke_passed"
            return "gazebo_base_level_route_following_partial"
        if self.abort_reason and "odom" in self.abort_reason:
            return "failed_environment"
        if cmd_vel_published and not pose_changed:
            return "failed_environment"
        return "failed_pipeline"

    def summary(self, route_validation: dict[str, Any]) -> dict[str, Any]:
        endpoint = self.endpoint_distance()
        classification = self.classification()
        assert classification in VALID_CLASSIFICATIONS
        return {
            "schema_name": "rslg_base_level_route_follower_summary",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "route_type": self.route_type,
            "output_dir": self.output_dir.as_posix(),
            "cmd_vel_topic": self.cmd_vel_topic,
            "odom_topic": self.odom_topic,
            "frame_id": self.frame_id,
            "robot_frame_id": self.robot_frame_id,
            "motion_basis": "base_velocity_cmd_vel",
            "pose_source_used": self.current_pose.source if self.current_pose else None,
            "require_odom": self.require_odom,
            "fallback_gazebo_model_state_enabled": self.fallback_gazebo_model_state,
            "fallback_pose_stepping_enabled": self.fallback_pose_stepping,
            "pose_stepping_used": False,
            "cmd_vel_published": self.cmd_vel_publish_count > 0,
            "nonzero_cmd_vel_published": self.nonzero_cmd_vel_publish_count > 0,
            "cmd_vel_publish_count": self.cmd_vel_publish_count,
            "nonzero_cmd_vel_publish_count": self.nonzero_cmd_vel_publish_count,
            "odom_received": self.odom_received,
            "gazebo_model_state_received": self.model_state_received,
            "robot_pose_changed": self.pose_changed_distance() > 0.05,
            "pose_changed_distance_m": round(self.pose_changed_distance(), 6),
            "trajectory_sample_count": len(self.trajectory_samples),
            "cmd_vel_sample_count": len(self.cmd_vel_samples),
            "waypoints_total": len(self.waypoints),
            "waypoints_attempted": min(len(self.waypoints), max(self.current_waypoint_index + (0 if self.route_completed() else 1), 0)),
            "waypoints_reached": len(self.waypoint_events),
            "route_completed": self.route_completed(),
            "timed_out": self.timed_out,
            "abort_reason": self.abort_reason,
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 3),
            "endpoint_distance_to_generated_ring_002": round(endpoint, 6) if endpoint is not None else None,
            "final_target_candidate_id": "generated_ring_002" if self.route_type == "cross_floor_object" else None,
            "generated_ring_037_runtime_goal_used": route_validation["generated_ring_037_runtime_goal_used"],
            "vt_1_centerline_e001_transition_handoff_edge": route_validation["vt_1_centerline_e001_transition_handoff_edge"],
            "vt_1_centerline_e003_transition_used": route_validation["vt_1_centerline_e003_transition_used"],
            "z_aware_visualization": self.z_aware_summary(),
            "navigation_classification": classification,
            "files": {
                "trajectory_samples": self.trajectory_path.as_posix(),
                "cmd_vel_samples": self.cmd_path.as_posix(),
                "waypoint_reach_events": self.events_path.as_posix(),
            },
            "claim_boundary": {
                "base_level_route_following": classification.startswith("gazebo_base_level_route_following"),
                "pose_stepped_fallback": False,
                "pose_replay_visualization": False,
                "nav2_success_claimed": False,
                "amcl_success_claimed": False,
                "legged_gait_control_claimed": False,
                "real_robot_claimed": False,
                "collision_free_guarantee_claimed": False,
                "z_aware_vertical_transition_is_visualization_overlay": self.z_aware_visualization,
                "physical_stair_climbing_claimed": bool(self.physical_climb_claim),
                "unitree_go2_real_control_claimed": False,
            },
        }


def write_validation_markdown(path: Path, summary: dict[str, Any]) -> None:
    write_text(
        path,
        "# Base-Level Route Follower Validation\n\n"
        f"- navigation_classification: `{summary['navigation_classification']}`\n"
        f"- motion_basis: `{summary['motion_basis']}`\n"
        f"- cmd_vel_published: `{summary['cmd_vel_published']}`\n"
        f"- nonzero_cmd_vel_published: `{summary['nonzero_cmd_vel_published']}`\n"
        f"- odom_received: `{summary['odom_received']}`\n"
        f"- pose_source_used: `{summary['pose_source_used']}`\n"
        f"- robot_pose_changed: `{summary['robot_pose_changed']}`\n"
        f"- pose_changed_distance_m: `{summary['pose_changed_distance_m']}`\n"
        f"- waypoints_attempted: `{summary['waypoints_attempted']}`\n"
        f"- waypoints_reached: `{summary['waypoints_reached']}` / `{summary['waypoints_total']}`\n"
        f"- endpoint_distance_to_generated_ring_002: `{summary['endpoint_distance_to_generated_ring_002']}`\n"
        f"- final_target_candidate_id: `{summary['final_target_candidate_id']}`\n"
        f"- generated_ring_037_runtime_goal_used: `{summary['generated_ring_037_runtime_goal_used']}`\n"
        f"- vt_1_centerline_e001_transition_handoff_edge: `{summary['vt_1_centerline_e001_transition_handoff_edge']}`\n"
        f"- vt_1_centerline_e003_transition_used: `{summary['vt_1_centerline_e003_transition_used']}`\n"
        f"- z_aware_visualization_enabled: `{summary['z_aware_visualization']['z_aware_visualization_enabled']}`\n"
        f"- z_aware_visual_trajectory_z_min: `{summary['z_aware_visualization']['z_aware_visual_trajectory_z_min']}`\n"
        f"- z_aware_visual_trajectory_z_max: `{summary['z_aware_visualization']['z_aware_visual_trajectory_z_max']}`\n"
        f"- z_aware_visual_transition_detected: `{summary['z_aware_visualization']['z_aware_visual_transition_detected']}`\n"
        f"- physical_trajectory_z_min: `{summary['z_aware_visualization']['physical_trajectory_z_min']}`\n"
        f"- physical_trajectory_z_max: `{summary['z_aware_visualization']['physical_trajectory_z_max']}`\n"
        f"- physical_climb_claimed: `{summary['z_aware_visualization']['physical_climb_claimed']}`\n"
        f"- timed_out: `{summary['timed_out']}`\n"
        f"- abort_reason: `{summary['abort_reason']}`\n\n"
        "This is a bounded Gazebo route-navigation smoke using base-level velocity commands where available. "
        "The z-aware vertical-transition layer is a visualization overlay only; the Gazebo physics robot stays "
        "base-level and no physical stair climbing is performed. "
        "It does not claim Nav2 success, AMCL success, full embodied navigation benchmarking, real robot deployment, "
        "legged gait control, Unitree Go2 real-robot control, or a collision-free guarantee.\n",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    # Formal input: RouteResult-derived PID runtime input.
    source.add_argument("--runtime-input-json", type=Path, default=None)
    # Optional: an rslg_route_result, converted internally through the PID adapter.
    source.add_argument("--route-result-json", type=Path, default=None)
    parser.add_argument("--z-aware-overlay-json", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--robot-frame-id", default="base_link")
    parser.add_argument("--model-state-topic", default="/model_states")
    parser.add_argument("--robot-model-name", default="quadruped_visual_base_robot")
    parser.add_argument("--max-duration-sec", type=float, default=180.0)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.30)
    parser.add_argument("--yaw-tolerance-rad", type=float, default=0.35)
    parser.add_argument("--max-linear-speed", type=float, default=0.15)
    parser.add_argument("--max-angular-speed", type=float, default=0.5)
    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--require-odom", type=parse_bool, default=True)
    parser.add_argument("--fallback-gazebo-model-state", action="store_true")
    parser.add_argument("--fallback-pose-stepping", action="store_true")
    parser.add_argument("--z-aware-visualization", action="store_true")
    parser.add_argument("--vertical-transition-edge", default=TRUE_TRANSITION_EDGE)
    parser.add_argument("--floor1-z", type=float, default=0.0)
    parser.add_argument("--floor2-z", type=float, default=1.6)
    parser.add_argument(
        "--transition-mode",
        choices=("visual_marker", "visual_tf", "gazebo_pose_step_optional"),
        default="visual_marker",
    )
    parser.add_argument("--publish-z-aware-trajectory", action="store_true")
    parser.add_argument("--publish-z-aware-robot-marker", action="store_true")
    parser.add_argument("--no-physical-climb-claim", action="store_true")
    parser.add_argument("--z-aware-topic", default="/rslg_slam/z_aware_navigation_markers")
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args()

    if args.no_canonical_write and "canonical" in args.output_dir.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    source_path = args.runtime_input_json or args.route_result_json
    source_payload = read_json(source_path)
    runtime_input = load_runtime_input(source_payload)
    route_type = (runtime_input.get("identity") or {}).get("query_type") or "cross_floor_object"
    waypoints = pid_runtime_waypoints(runtime_input)
    route_validation = pid_route_validation(runtime_input, waypoints)

    z_aware_waypoints: list[dict[str, Any]] = []
    z_aware_overlay = read_json(args.z_aware_overlay_json) if args.z_aware_overlay_json else None
    route_result_for_overlay = source_payload if is_route_result(source_payload) else None
    if args.z_aware_visualization:
        z_aware_waypoints = load_z_aware_waypoints(
            z_aware_overlay=z_aware_overlay,
            route_result=route_result_for_overlay,
            floor1_z=args.floor1_z,
            floor2_z=args.floor2_z,
            transition_edge=args.vertical_transition_edge,
        )
        if z_aware_waypoints:
            write_json(
                args.output_dir / "z_aware_route_projection.json",
                {
                    "schema_name": "rslg_route_follower_z_aware_overlay_snapshot",
                    "source_runtime_input": source_path.as_posix(),
                    "vertical_transition_edge": args.vertical_transition_edge,
                    "overlay_waypoints": z_aware_waypoints,
                },
            )

    write_json(
        args.output_dir / "route_follower_startup_summary.json",
        {
            "schema_name": "rslg_base_level_route_follower_startup",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "runtime_input": source_path.as_posix(),
            "runtime_input_schema": runtime_input.get("schema_name"),
            "route_type": route_type,
            "waypoint_count": len(waypoints),
            "first_waypoint": waypoints[0],
            "last_waypoint": waypoints[-1],
            "route_validation": route_validation,
            "no_canonical_write": bool(args.no_canonical_write),
            "z_aware_visualization_enabled": bool(args.z_aware_visualization),
            "vertical_transition_edge": args.vertical_transition_edge,
            "transition_mode": args.transition_mode if args.z_aware_visualization else None,
            "floor1_z": args.floor1_z,
            "floor2_z": args.floor2_z,
            "z_aware_waypoint_count": len(z_aware_waypoints),
            "physical_climb_claimed": False,
        },
    )

    rclpy.init(args=None)
    node = BaseLevelRouteFollower(
        waypoints=waypoints,
        route_input=runtime_input,
        route_type=route_type,
        output_dir=args.output_dir,
        cmd_vel_topic=args.cmd_vel_topic,
        odom_topic=args.odom_topic,
        frame_id=args.frame_id,
        robot_frame_id=args.robot_frame_id,
        model_state_topic=args.model_state_topic,
        robot_model_name=args.robot_model_name,
        goal_tolerance_m=args.goal_tolerance_m,
        yaw_tolerance_rad=args.yaw_tolerance_rad,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        max_duration_sec=args.max_duration_sec,
        require_odom=args.require_odom,
        fallback_gazebo_model_state=args.fallback_gazebo_model_state,
        fallback_pose_stepping=args.fallback_pose_stepping,
        publish_rate_hz=args.publish_rate_hz,
        z_aware_visualization=args.z_aware_visualization,
        z_aware_waypoints=z_aware_waypoints,
        vertical_transition_edge=args.vertical_transition_edge,
        floor1_z=args.floor1_z,
        floor2_z=args.floor2_z,
        transition_mode=args.transition_mode,
        publish_z_aware_trajectory=args.publish_z_aware_trajectory or args.z_aware_visualization,
        publish_z_aware_robot_marker=args.publish_z_aware_robot_marker or args.z_aware_visualization,
        physical_climb_claim=False,
        z_aware_topic=args.z_aware_topic,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.publish_stop()
        summary = node.summary(route_validation)
        write_json(args.output_dir / "base_level_route_follower_summary.json", summary)
        write_validation_markdown(args.output_dir / "base_level_route_follower_validation.md", summary)
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
