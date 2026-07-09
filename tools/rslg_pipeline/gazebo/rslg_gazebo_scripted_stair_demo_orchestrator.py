#!/usr/bin/env python3
"""Segmented Gazebo/RViz executor for the task60 scripted stair-transition demo."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rslg_gazebo_pid_follower import (  # noqa: E402
    DEFAULT_CMD_VEL_TOPIC,
    DEFAULT_ODOM_TOPIC,
    DEFAULT_PROFILE_ID,
    PidParams,
    clamp,
    load_profile,
    yaw_from_quaternion,
)
from scripted_stair_transition_common import (  # noqa: E402
    BLOCKED_CANDIDATE_ID,
    FORBIDDEN_NON_TRANSITION_EDGE,
    PROJECT_NAME,
    SELECTED_APPROACH_ID,
    TOPICS,
    TRUE_TRANSITION_EDGE,
    AnchorTransform,
    Pose3D,
    count_route_segments,
    distance_xy,
    normalize_angle,
    pose_to_point_dict,
    read_json,
    route_points,
    stair_points,
    transform_points,
    utc_now,
    write_json,
    write_text,
    yaw_to_quaternion,
)


STATE_IDLE = "idle_wait_for_odom"
STATE_FLOOR_1 = "follow_floor_1"
STATE_STAIR = "scripted_stair_transition"
STATE_FLOOR_2 = "follow_floor_2"
STATE_DONE = "done"
STATE_FAILED = "failed"


@dataclass
class OdomPose:
    x: float
    y: float
    z: float
    yaw: float

    def as_pose3d(self, segment: str = "odom") -> Pose3D:
        return Pose3D(self.x, self.y, self.z, self.yaw, segment=segment)


def query_id_from_runtime_input(runtime_input: dict[str, Any], fallback: str | None) -> str:
    return fallback or str(runtime_input.get("source_query_id") or "unknown_query")


def path_summary_markdown(summary: dict[str, Any], *, title: str) -> str:
    if "ok" in summary:
        checks = summary.get("checks") or {}
        lines = [
            f"# {title}",
            "",
            f"- status: `{summary.get('status')}`",
            f"- ok: `{summary.get('ok')}`",
            f"- query_id: `{summary.get('query_id')}`",
            f"- frame_id: `{summary.get('frame_id')}`",
            f"- floor_1 waypoints: `{summary.get('floor_1_waypoints')}`",
            f"- scripted stair keyframes: `{summary.get('scripted_stair_keyframes')}`",
            f"- floor_2 waypoints: `{summary.get('floor_2_waypoints')}`",
            f"- composite point count: `{summary.get('composite_point_count')}`",
            f"- world exists: `{checks.get('world_file_exists')}`",
            f"- RViz config exists: `{checks.get('rviz_config_exists')}`",
            f"- vt_1_centerline_e001 transition: `{checks.get('vt_1_centerline_e001_transition')}`",
            f"- vt_1_centerline_e003 not transition: `{checks.get('vt_1_centerline_e003_not_transition')}`",
            f"- generated_ring_037 not goal: `{checks.get('generated_ring_037_not_goal')}`",
            f"- generated_ring_002 final approach: `{checks.get('generated_ring_002_final_approach')}`",
            f"- composite planned topic: `{summary.get('topics', {}).get('composite_planned_path_odom')}`",
            f"- composite executed topic: `{summary.get('topics', {}).get('composite_executed_path')}`",
            f"- physical stair-climbing claim: `{summary.get('physical_stair_climbing_claim')}`",
            "",
            "Dry-run does not start Gazebo, RViz, Nav2, AMCL, map_server, Stage-A, raw RGB-D inference, or physical robot code.",
            "",
        ]
        return "\n".join(lines)
    lines = [
        f"# {title}",
        "",
        f"- status: `{summary.get('status')}`",
        f"- query_id: `{summary.get('query_id')}`",
        f"- frame_id: `{summary.get('frame_id')}`",
        f"- floor_1 status: `{summary.get('floor_1_status')}`",
        f"- stair transition status: `{summary.get('stair_transition_status')}`",
        f"- floor_2 status: `{summary.get('floor_2_status')}`",
        f"- final target status: `{summary.get('final_target_status')}`",
        f"- odom received: `{summary.get('odom_received')}`",
        f"- cmd_vel published: `{summary.get('cmd_vel_published')}`",
        f"- set-entity-state service available: `{summary.get('set_entity_state_service_available')}`",
        f"- execution mode: `{summary.get('execution_mode')}`",
        f"- timeout: `{summary.get('timeout')}`",
        f"- final error: `{summary.get('final_error')}`",
        f"- trajectory CSV: `{summary.get('output_trajectory_csv')}`",
        "",
        "The stair transition segment is scripted animation. It is not physical stair-climbing validation.",
        "",
    ]
    if summary.get("failure_reason"):
        lines.insert(-2, f"- failure reason: `{summary.get('failure_reason')}`")
    return "\n".join(lines)


def pose_stamped_msg(PoseStamped: Any, frame_id: str, stamp: Any, pose: Pose3D) -> Any:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    msg.pose.position.z = float(pose.z)
    qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def path_msg(RosPath: Any, PoseStamped: Any, frame_id: str, stamp: Any, points: list[Pose3D]) -> Any:
    msg = RosPath()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.poses = [pose_stamped_msg(PoseStamped, frame_id, stamp, point) for point in points]
    return msg


def add_line_marker(
    markers: list[Any],
    Marker: Any,
    frame_id: str,
    stamp: Any,
    *,
    marker_id: int,
    ns: str,
    points: list[Pose3D],
    color: tuple[float, float, float, float],
    width: float,
) -> None:
    if len(points) < 2:
        return
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    for point in points:
        p = type(marker.pose.position)()
        p.x = float(point.x)
        p.y = float(point.y)
        p.z = float(point.z + 0.04)
        marker.points.append(p)
    markers.append(marker)


def add_text_marker(
    markers: list[Any],
    Marker: Any,
    frame_id: str,
    stamp: Any,
    *,
    marker_id: int,
    ns: str,
    pose: Pose3D,
    text: str,
    color: tuple[float, float, float, float],
    size: float = 0.18,
) -> None:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = float(pose.x)
    marker.pose.position.y = float(pose.y)
    marker.pose.position.z = float(pose.z)
    marker.pose.orientation.w = 1.0
    marker.scale.z = size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.text = text
    markers.append(marker)


def add_sphere_marker(
    markers: list[Any],
    Marker: Any,
    frame_id: str,
    stamp: Any,
    *,
    marker_id: int,
    ns: str,
    pose: Pose3D,
    color: tuple[float, float, float, float],
    size: float = 0.28,
) -> None:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(pose.x)
    marker.pose.position.y = float(pose.y)
    marker.pose.position.z = float(pose.z)
    marker.pose.orientation.w = 1.0
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    markers.append(marker)


def marker_array_msg(MarkerArray: Any, Marker: Any, frame_id: str, stamp: Any, points: list[Pose3D]) -> Any:
    msg = MarkerArray()
    floor_1 = [point for point in points if point.segment == "floor_1"]
    stair = [point for point in points if point.segment == "scripted_stair"]
    floor_2 = [point for point in points if point.segment == "floor_2"]
    markers: list[Any] = []
    add_line_marker(
        markers,
        Marker,
        frame_id,
        stamp,
        marker_id=1,
        ns="rslg_scripted_stair_floor_1_pid_segment",
        points=floor_1,
        color=(0.18, 0.63, 0.95, 1.0),
        width=0.055,
    )
    add_line_marker(
        markers,
        Marker,
        frame_id,
        stamp,
        marker_id=2,
        ns="rslg_scripted_stair_transition_vt_1_centerline_e001",
        points=stair,
        color=(0.85, 0.62, 1.0, 1.0),
        width=0.07,
    )
    add_line_marker(
        markers,
        Marker,
        frame_id,
        stamp,
        marker_id=3,
        ns="rslg_scripted_stair_floor_2_pid_segment",
        points=floor_2,
        color=(0.25, 0.90, 0.42, 1.0),
        width=0.055,
    )
    if stair:
        mid = stair[len(stair) // 2]
        add_text_marker(
            markers,
            Marker,
            frame_id,
            stamp,
            marker_id=4,
            ns="rslg_scripted_transition_label",
            pose=Pose3D(mid.x, mid.y, mid.z + 0.36, mid.yaw),
            text=f"{TRUE_TRANSITION_EDGE} scripted transition",
            color=(0.92, 0.80, 1.0, 1.0),
        )
        add_text_marker(
            markers,
            Marker,
            frame_id,
            stamp,
            marker_id=5,
            ns="rslg_forbidden_non_transition_guard",
            pose=Pose3D(mid.x + 0.45, mid.y, mid.z + 0.12, mid.yaw),
            text=f"{FORBIDDEN_NON_TRANSITION_EDGE} forbidden/non-transition",
            color=(1.0, 0.35, 0.22, 1.0),
            size=0.16,
        )
    if floor_2:
        final = floor_2[-1]
        add_sphere_marker(
            markers,
            Marker,
            frame_id,
            stamp,
            marker_id=6,
            ns="rslg_generated_ring_002_final_approach",
            pose=Pose3D(final.x, final.y, final.z + 0.16, final.yaw),
            color=(0.05, 0.88, 0.30, 1.0),
        )
        add_text_marker(
            markers,
            Marker,
            frame_id,
            stamp,
            marker_id=7,
            ns="rslg_blocked_candidate_guard",
            pose=Pose3D(final.x, final.y - 0.58, final.z + 0.46, final.yaw),
            text=f"{BLOCKED_CANDIDATE_ID} blocked/rejected only",
            color=(1.0, 0.42, 0.35, 1.0),
            size=0.16,
        )
        add_text_marker(
            markers,
            Marker,
            frame_id,
            stamp,
            marker_id=8,
            ns="rslg_scripted_stair_claim_boundary",
            pose=Pose3D(final.x, final.y - 1.02, final.z + 0.72, final.yaw),
            text="scripted stair animation; no physical stair-climbing claim",
            color=(1.0, 0.95, 0.72, 1.0),
            size=0.16,
        )
    msg.markers = markers
    return msg


class ScriptedStairDemo:
    def __init__(
        self,
        *,
        rclpy: Any,
        Node: Any,
        Twist: Any,
        Odometry: Any,
        RosPath: Any,
        PoseStamped: Any,
        MarkerArray: Any,
        Marker: Any,
        SetEntityState: Any | None,
        EntityState: Any | None,
        args: argparse.Namespace,
        runtime_input: dict[str, Any],
        profile: dict[str, Any],
        params: PidParams,
    ) -> None:
        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.Twist = Twist
        self.RosPath = RosPath
        self.PoseStamped = PoseStamped
        self.MarkerArray = MarkerArray
        self.Marker = Marker
        self.SetEntityState = SetEntityState
        self.EntityState = EntityState
        self.node = _Node("rslg_gazebo_scripted_stair_demo_orchestrator")
        self.args = args
        self.runtime_input = runtime_input
        self.profile = profile
        self.params = params
        self.query_id = query_id_from_runtime_input(runtime_input, args.query_id)
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.summary_json = self.output_dir / f"{self.query_id}_scripted_stair_demo_execution_summary.json"
        self.trajectory_csv = self.output_dir / f"{self.query_id}_scripted_stair_composite_trajectory.csv"
        self.csv_handle = self.trajectory_csv.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=[
                "wall_time_utc",
                "elapsed_sec",
                "state",
                "event",
                "odom_x",
                "odom_y",
                "odom_z",
                "odom_yaw",
                "target_index",
                "target_x",
                "target_y",
                "target_z",
                "target_yaw",
                "target_segment",
                "source_edge_id",
                "scripted_transition",
                "distance_to_target_xy",
                "linear_x",
                "angular_z",
            ],
        )
        self.csv_writer.writeheader()

        self.pose: OdomPose | None = None
        self.visual_pose: Pose3D | None = None
        self.odom_received = False
        self.cmd_vel_publish_count = 0
        self.state = STATE_IDLE
        self.failure_reason: list[str] = []
        self.timeout = False
        self.start_monotonic = time.monotonic()
        self.first_odom_monotonic: float | None = None
        self.first_control_monotonic: float | None = None
        self.end_monotonic: float | None = None
        self.anchor_transform: AnchorTransform | None = None
        self.anchor_applied = False
        self.original_points = route_points(runtime_input)
        self.original_floor_1 = [point for point in self.original_points if point.segment == "floor_1"]
        self.original_stair = [point for point in self.original_points if point.segment == "scripted_stair"]
        self.original_floor_2 = [point for point in self.original_points if point.segment == "floor_2"]
        self.floor_1 = self.original_floor_1
        self.stair = self.original_stair
        self.floor_2 = self.original_floor_2
        self.planned_points = self.original_points
        self.executed_points: list[Pose3D] = []
        self.floor_1_index = 0
        self.floor_2_index = 0
        self.stair_index = 0
        self.floor_1_status = "pending"
        self.stair_transition_status = "pending"
        self.floor_2_status = "pending"
        self.final_target_status = "pending"
        self.gazebo_msg_import_available = SetEntityState is not None and EntityState is not None
        self.set_entity_state_service_available = False
        self.set_entity_state_service_used = False
        self.execution_mode = "gazebo_service_if_available"

        self.cmd_vel_pub = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.planned_pub = self.node.create_publisher(RosPath, TOPICS["composite_planned_path_odom"], 10)
        self.executed_pub = self.node.create_publisher(RosPath, TOPICS["composite_executed_path"], 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, TOPICS["robot_pose"], 10)
        self.stair_path_pub = self.node.create_publisher(RosPath, TOPICS["stair_transition_path"], 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, TOPICS["marker_array"], 10)
        self.odom_sub = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.set_state_client = None
        if self.gazebo_msg_import_available:
            self.set_state_client = self.node.create_client(SetEntityState, args.set_entity_state_service)
            self.set_entity_state_service_available = bool(self.set_state_client.wait_for_service(timeout_sec=args.service_wait_sec))
        if not self.set_entity_state_service_available and args.rviz_only_stair_fallback:
            self.execution_mode = "rviz_only_stair_fallback"
        self.timer = self.node.create_timer(1.0 / float(args.rate_hz), self._on_timer)
        self.node.get_logger().info(
            f"RSLG-SLAM scripted stair demo ready: query={self.query_id}, "
            f"floor_1={len(self.floor_1)}, stair={len(self.stair)}, floor_2={len(self.floor_2)}, "
            f"set_entity_state={self.set_entity_state_service_available}"
        )

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        self.pose = OdomPose(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        self.odom_received = True
        if self.first_odom_monotonic is None:
            self.first_odom_monotonic = time.monotonic()

    def _publish_stop(self) -> None:
        twist = self.Twist()
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_publish_count += 1

    def _publish_paths(self) -> None:
        stamp = self.node.get_clock().now().to_msg()
        self.planned_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.planned_points))
        self.executed_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.executed_points))
        self.stair_path_pub.publish(path_msg(self.RosPath, self.PoseStamped, self.args.frame_id, stamp, self.stair))
        self.marker_pub.publish(marker_array_msg(self.MarkerArray, self.Marker, self.args.frame_id, stamp, self.planned_points))
        pose = self.visual_pose
        if pose is None and self.pose is not None:
            pose = self.pose.as_pose3d(segment=self.state)
        if pose is not None:
            self.robot_pose_pub.publish(pose_stamped_msg(self.PoseStamped, self.args.frame_id, stamp, pose))

    def _apply_anchor_if_needed(self) -> None:
        if self.anchor_applied or self.pose is None:
            return
        if self.args.anchor_first_waypoint_to_odom_start:
            first = self.original_points[0]
            self.anchor_transform = AnchorTransform(
                route_x=first.x,
                route_y=first.y,
                route_z=first.z,
                route_yaw=first.yaw,
                odom_x=self.pose.x,
                odom_y=self.pose.y,
                odom_z=self.pose.z,
                odom_yaw=self.pose.yaw,
            )
        self.floor_1 = transform_points(self.original_floor_1, self.anchor_transform)
        self.stair = transform_points(self.original_stair, self.anchor_transform)
        self.floor_2 = transform_points(self.original_floor_2, self.anchor_transform)
        self.planned_points = [*self.floor_1, *self.stair, *self.floor_2]
        self.anchor_applied = True

    def _write_row(self, *, event: str, target: Pose3D | None, linear: float, angular: float, distance: float) -> None:
        pose = self.pose.as_pose3d(segment=self.state) if self.pose is not None else self.visual_pose
        if pose is None:
            pose = Pose3D(0.0, 0.0, 0.0, 0.0, segment=self.state)
        elapsed = time.monotonic() - self.start_monotonic
        self.csv_writer.writerow(
            {
                "wall_time_utc": utc_now(),
                "elapsed_sec": f"{elapsed:.6f}",
                "state": self.state,
                "event": event,
                "odom_x": f"{pose.x:.6f}",
                "odom_y": f"{pose.y:.6f}",
                "odom_z": f"{pose.z:.6f}",
                "odom_yaw": f"{pose.yaw:.6f}",
                "target_index": "" if target is None else target.index,
                "target_x": "" if target is None else f"{target.x:.6f}",
                "target_y": "" if target is None else f"{target.y:.6f}",
                "target_z": "" if target is None else f"{target.z:.6f}",
                "target_yaw": "" if target is None else f"{target.yaw:.6f}",
                "target_segment": "" if target is None else target.segment,
                "source_edge_id": "" if target is None or target.source_edge_id is None else target.source_edge_id,
                "scripted_transition": "" if target is None else str(target.scripted_transition),
                "distance_to_target_xy": "" if math.isnan(distance) else f"{distance:.6f}",
                "linear_x": f"{linear:.6f}",
                "angular_z": f"{angular:.6f}",
            }
        )
        self.csv_handle.flush()

    def _append_executed(self, point: Pose3D, *, event: str) -> None:
        self.executed_points.append(point)
        self.visual_pose = point
        self._write_row(event=event, target=point, linear=0.0, angular=0.0, distance=0.0)

    def _call_set_entity_state(self, pose: Pose3D) -> None:
        if self.set_state_client is None or self.SetEntityState is None or self.EntityState is None:
            return
        request = self.SetEntityState.Request()
        state = self.EntityState()
        state.name = self.args.robot_model_name
        state.pose.position.x = float(pose.x)
        state.pose.position.y = float(pose.y)
        state.pose.position.z = float(pose.z)
        qx, qy, qz, qw = yaw_to_quaternion(pose.yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.reference_frame = "world"
        request.state = state
        self.set_state_client.call_async(request)
        self.set_entity_state_service_used = True

    def _follow_segment(self, segment: list[Pose3D], index_attr: str) -> bool:
        if self.pose is None:
            return False
        index = int(getattr(self, index_attr))
        if index >= len(segment):
            return True
        target = segment[index]
        pose = self.pose
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx) if distance > 1e-9 else pose.yaw
        heading_error = normalize_angle(heading - pose.yaw)
        yaw_error = normalize_angle(target.yaw - pose.yaw)
        final_target = self.state == STATE_FLOOR_2 and index == len(segment) - 1
        if distance <= self.params.waypoint_tolerance:
            if not final_target or abs(yaw_error) <= self.params.yaw_tolerance:
                self._append_executed(pose.as_pose3d(segment=target.segment), event="waypoint_reached")
                setattr(self, index_attr, index + 1)
                return index + 1 >= len(segment)

        twist = self.Twist()
        if final_target and distance <= self.params.waypoint_tolerance:
            linear = 0.0
            angular = clamp(self.params.angular_gain * yaw_error, -self.params.max_angular_velocity, self.params.max_angular_velocity)
        else:
            angular = clamp(self.params.angular_gain * heading_error, -self.params.max_angular_velocity, self.params.max_angular_velocity)
            if abs(heading_error) > math.pi / 2.0:
                linear = 0.0
            else:
                linear = (
                    clamp(self.params.linear_gain * distance, 0.0, self.params.max_linear_velocity)
                    * max(0.0, math.cos(heading_error))
                )
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)
        self.cmd_vel_publish_count += 1
        self._write_row(event="cmd_vel", target=target, linear=linear, angular=angular, distance=distance)
        return False

    def _run_stair_tick(self) -> bool:
        if self.stair_index == 0:
            self._publish_stop()
        if not self.set_entity_state_service_available and not self.args.rviz_only_stair_fallback:
            self._fail("gazebo_set_entity_state_unavailable")
            return False
        if self.stair_index >= len(self.stair):
            return True
        pose = self.stair[self.stair_index]
        if self.set_entity_state_service_available:
            self._call_set_entity_state(pose)
        self._append_executed(pose, event="scripted_stair_keyframe")
        self.stair_index += 1
        if self.stair_index >= len(self.stair):
            self.stair_transition_status = "passed_scripted"
            return True
        return False

    def _timeout_limit(self) -> float:
        if self.args.duration_sec and self.args.duration_sec > 0:
            return float(self.args.duration_sec)
        return float(self.params.timeout_sec)

    def _on_timer(self) -> None:
        if self.state in {STATE_DONE, STATE_FAILED}:
            return
        now = time.monotonic()
        if now - self.start_monotonic > self._timeout_limit():
            self.timeout = True
            self._fail("timeout")
            return
        self._publish_paths()
        if self.pose is None:
            return
        self._apply_anchor_if_needed()
        if self.first_control_monotonic is None:
            self.first_control_monotonic = now
            self.state = STATE_FLOOR_1
            self.floor_1_status = "running"

        if self.state == STATE_FLOOR_1:
            if self._follow_segment(self.floor_1, "floor_1_index"):
                self.floor_1_status = "passed"
                self.state = STATE_STAIR
                self.stair_transition_status = "running_scripted"
                self._publish_stop()
            return
        if self.state == STATE_STAIR:
            if self._run_stair_tick():
                if self.set_entity_state_service_available:
                    self.state = STATE_FLOOR_2
                    self.floor_2_status = "running"
                else:
                    self.floor_2_status = "skipped_rviz_only_stair_fallback"
                    self.final_target_status = "skipped_rviz_only_stair_fallback"
                    self._finish()
            return
        if self.state == STATE_FLOOR_2:
            if self._follow_segment(self.floor_2, "floor_2_index"):
                self.floor_2_status = "passed"
                self.final_target_status = "passed"
                self._finish()
            return

    def _fail(self, reason: str) -> None:
        self.failure_reason.append(reason)
        if self.floor_1_status == "running":
            self.floor_1_status = "failed"
        if self.stair_transition_status == "running_scripted":
            self.stair_transition_status = "failed"
        if self.floor_2_status == "running":
            self.floor_2_status = "failed"
        if self.final_target_status == "pending":
            self.final_target_status = "failed"
        self.state = STATE_FAILED
        self._publish_stop()
        self._close_outputs()

    def _finish(self) -> None:
        self.state = STATE_DONE
        self._publish_stop()
        if self.final_target_status == "pending":
            self.final_target_status = "passed"
        self._close_outputs()

    def _close_outputs(self) -> None:
        if self.end_monotonic is not None:
            return
        self.end_monotonic = time.monotonic()
        self._write_row(event="finish", target=None, linear=0.0, angular=0.0, distance=float("nan"))
        self.csv_handle.close()
        summary = self.summary()
        write_json(self.summary_json, summary)
        if self.args.summary_json:
            write_json(Path(self.args.summary_json), summary)
        if self.args.summary_md:
            write_text(Path(self.args.summary_md), path_summary_markdown(summary, title="Headless or Service Demo Execution Summary"))

    def summary(self) -> dict[str, Any]:
        final_target = self.floor_2[-1] if self.floor_2 else (self.planned_points[-1] if self.planned_points else None)
        pose = self.pose.as_pose3d(segment=self.state) if self.pose is not None else self.visual_pose
        final_error = None
        if final_target is not None and pose is not None and self.final_target_status == "passed":
            final_error = round(distance_xy(pose, final_target), 6)
        end = self.end_monotonic or time.monotonic()
        status = "passed" if self.state == STATE_DONE and self.final_target_status == "passed" else "failed"
        if self.state == STATE_DONE and self.final_target_status.startswith("skipped"):
            status = "partial_rviz_only_fallback"
        return {
            "schema_name": "rslg_scripted_stair_transition_demo_execution_summary",
            "schema_version": "0.1",
            "project_name": PROJECT_NAME,
            "generated_utc": utc_now(),
            "status": status,
            "query_id": self.query_id,
            "profile_id": self.args.profile_id,
            "runtime_input_json": str(self.args.runtime_input_json),
            "profile_json": str(self.args.profile_json),
            "frame_id": self.args.frame_id,
            "odom_topic": self.args.odom_topic,
            "cmd_vel_topic": self.args.cmd_vel_topic,
            "topics": TOPICS,
            "anchor_first_waypoint_to_odom_start": bool(self.args.anchor_first_waypoint_to_odom_start),
            "anchor_applied": self.anchor_applied,
            "execution_mode": self.execution_mode,
            "robot_model_name": self.args.robot_model_name,
            "set_entity_state_service": self.args.set_entity_state_service,
            "gazebo_msg_import_available": self.gazebo_msg_import_available,
            "set_entity_state_service_available": self.set_entity_state_service_available,
            "set_entity_state_service_used": self.set_entity_state_service_used,
            "floor_1_status": self.floor_1_status,
            "stair_transition_status": self.stair_transition_status,
            "floor_2_status": self.floor_2_status,
            "final_target_status": self.final_target_status,
            "floor_1_waypoints_total": len(self.floor_1),
            "floor_1_waypoints_reached": min(self.floor_1_index, len(self.floor_1)),
            "scripted_stair_keyframes_total": len(self.stair),
            "scripted_stair_keyframes_reached": min(self.stair_index, len(self.stair)),
            "floor_2_waypoints_total": len(self.floor_2),
            "floor_2_waypoints_reached": min(self.floor_2_index, len(self.floor_2)),
            "planned_composite_point_count": len(self.planned_points),
            "executed_composite_point_count": len(self.executed_points),
            "segment_waypoints_total": count_route_segments(self.planned_points),
            "final_error": final_error,
            "timeout": self.timeout,
            "failure_reason": self.failure_reason,
            "odom_received": self.odom_received,
            "cmd_vel_published": self.cmd_vel_publish_count > 0,
            "cmd_vel_publish_count": self.cmd_vel_publish_count,
            "simulated_duration": round(end - self.start_monotonic, 6),
            "output_trajectory_csv": str(self.trajectory_csv),
            "physical_stair_climbing_claim": False,
            "claim_boundary": self.runtime_input.get("claim_boundary"),
        }


def build_dry_run_summary(args: argparse.Namespace, runtime_input: dict[str, Any], profile: dict[str, Any], params: PidParams) -> dict[str, Any]:
    points = route_points(runtime_input)
    world = Path(runtime_input.get("gazebo_world") or "")
    rviz = Path(runtime_input.get("rviz_config") or "")
    guards = runtime_input.get("guards") or {}
    topics = runtime_input.get("topics") or {}
    checks = {
        "runtime_input_exists": Path(args.runtime_input_json).is_file(),
        "world_file_exists": world.is_file(),
        "rviz_config_exists": rviz.is_file(),
        "floor_1_segment_exists": len((runtime_input.get("floor_1_segment") or {}).get("waypoints") or []) > 0,
        "scripted_stair_keyframes_exist": len(runtime_input.get("scripted_stair_keyframes") or []) > 0,
        "floor_2_segment_exists": len((runtime_input.get("floor_2_segment") or {}).get("waypoints") or []) > 0,
        "vt_1_centerline_e001_transition": runtime_input.get("transition_edge_id") == TRUE_TRANSITION_EDGE,
        "vt_1_centerline_e003_not_transition": not bool(guards.get("vt_1_centerline_e003_used_as_transition")),
        "generated_ring_037_not_goal": not bool(guards.get("generated_ring_037_selected_as_runtime_goal")),
        "generated_ring_002_final_approach": runtime_input.get("target_approach_id") == SELECTED_APPROACH_ID,
        "frame_id_odom": str(runtime_input.get("frame_id") or args.frame_id) == "odom",
        "composite_planned_topic_configured": topics.get("composite_planned_path_odom") == TOPICS["composite_planned_path_odom"],
        "composite_executed_topic_configured": topics.get("composite_executed_path") == TOPICS["composite_executed_path"],
        "no_nav2_dependency": guards.get("requires_nav2") is False,
        "no_amcl_dependency": guards.get("requires_amcl") is False,
        "no_map_server_dependency": guards.get("requires_map_server") is False,
        "physical_stair_climbing_claim_false": runtime_input.get("physical_stair_climbing_claim") is False,
    }
    ok = all(checks.values())
    return {
        "schema_name": "rslg_scripted_stair_transition_dry_run_summary",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "generated_utc": utc_now(),
        "status": "passed" if ok else "failed",
        "ok": ok,
        "query_id": query_id_from_runtime_input(runtime_input, args.query_id),
        "profile_id": args.profile_id,
        "runtime_input_json": str(args.runtime_input_json),
        "profile_json": str(args.profile_json),
        "world": str(world),
        "rviz_config": str(rviz),
        "frame_id": args.frame_id,
        "topics": TOPICS,
        "floor_1_waypoints": len((runtime_input.get("floor_1_segment") or {}).get("waypoints") or []),
        "scripted_stair_keyframes": len(runtime_input.get("scripted_stair_keyframes") or []),
        "floor_2_waypoints": len((runtime_input.get("floor_2_segment") or {}).get("waypoints") or []),
        "composite_point_count": len(points),
        "segment_counts": count_route_segments(points),
        "first_point": points[0].__dict__ if points else None,
        "last_point": points[-1].__dict__ if points else None,
        "profile_params": params.__dict__,
        "checks": checks,
        "replay_topic_primary_display": False,
        "physical_stair_climbing_claim": False,
    }


def dry_run(args: argparse.Namespace, runtime_input: dict[str, Any], profile: dict[str, Any], params: PidParams) -> int:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary = build_dry_run_summary(args, runtime_input, profile, params)
    query_id = summary["query_id"]
    output = args.output_dir / f"{query_id}_scripted_stair_dry_run_summary.json"
    write_json(output, summary)
    if args.summary_json:
        write_json(Path(args.summary_json), summary)
    if args.summary_md:
        write_text(Path(args.summary_md), path_summary_markdown(summary, title="Scripted Stair Dry Run Summary"))
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0 if summary["ok"] else 2


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cmd-vel-topic", default=DEFAULT_CMD_VEL_TOPIC)
    parser.add_argument("--robot-model-name", default="turtlebot3_burger")
    parser.add_argument("--set-entity-state-service", default="/set_entity_state")
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=420.0)
    parser.add_argument("--service-wait-sec", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rviz-only-stair-fallback", action="store_true")
    parser.add_argument("--stop-at-end", action="store_true")
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--summary-md", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime_input = read_json(args.runtime_input_json)
    profile, params = load_profile(args.profile_json, args.profile_id)
    if args.dry_run:
        return dry_run(args, runtime_input, profile, params)

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry, Path as RosPath
        from rclpy.node import Node
        from visualization_msgs.msg import Marker, MarkerArray
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise SystemExit(f"ROS 2 Python dependencies are unavailable: {exc}") from exc

    try:
        from gazebo_msgs.msg import EntityState
        from gazebo_msgs.srv import SetEntityState
    except Exception:
        EntityState = None
        SetEntityState = None

    rclpy.init(args=None)
    demo = ScriptedStairDemo(
        rclpy=rclpy,
        Node=Node,
        Twist=Twist,
        Odometry=Odometry,
        RosPath=RosPath,
        PoseStamped=PoseStamped,
        MarkerArray=MarkerArray,
        Marker=Marker,
        SetEntityState=SetEntityState,
        EntityState=EntityState,
        args=args,
        runtime_input=runtime_input,
        profile=profile,
        params=params,
    )
    try:
        while rclpy.ok() and demo.state not in {STATE_DONE, STATE_FAILED}:
            rclpy.spin_once(demo.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        demo._fail("keyboard_interrupt")
    finally:
        if demo.state not in {STATE_DONE, STATE_FAILED}:
            demo._fail("shutdown_before_completion")
        demo.node.destroy_node()
        rclpy.shutdown()
    summary = demo.summary()
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)
    return 0 if summary["status"] in {"passed", "partial_rviz_only_fallback"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
