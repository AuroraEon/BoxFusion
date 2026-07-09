#!/usr/bin/env python3
"""ROS 2 waypoint follower for task59 RSLG-SLAM multi-floor ramp simulation.

The node follows XY/yaw waypoints from a task59 ramp-surrogate runtime input.
Gazebo collision geometry supplies the height transition; this follower does
not command z and does not import or launch Nav2, AMCL, map_server, Stage-A,
RGB-D inference, navigation actions, or physical robot code.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
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
    normalize_angle,
    read_json,
    write_json,
    yaw_from_quaternion,
)


TRUE_TRANSITION_EDGE = "vt_1_centerline_e001"
FORBIDDEN_NON_TRANSITION_EDGE = "vt_1_centerline_e003"
BLOCKED_CANDIDATE_ID = "generated_ring_037"
SELECTED_APPROACH_ID = "generated_ring_002"


@dataclass(frozen=True)
class Waypoint:
    index: int
    source_index: int | None
    x: float
    y: float
    z: float
    yaw: float
    floor_id: str
    segment_id: str
    ramp_segment_id: str | None
    semantic_source: str
    is_ramp_waypoint: bool
    source_edge_id: str | None


@dataclass
class Pose3D:
    x: float
    y: float
    z: float
    yaw: float


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def query_id_from_runtime_input(runtime_input: dict[str, Any], fallback: str | None) -> str:
    if fallback:
        return fallback
    return str(runtime_input.get("source_query_id") or "unknown_query")


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def extract_waypoints(runtime_input: dict[str, Any], floor_id: str | None) -> list[Waypoint]:
    raw_waypoints = runtime_input.get("waypoints") or runtime_input.get("runtime_waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("runtime input does not contain waypoints")

    waypoints: list[Waypoint] = []
    previous_yaw = 0.0
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            continue
        wp_floor = str(raw.get("floor_id") or "")
        if floor_id and floor_id != "all" and wp_floor != floor_id:
            continue
        yaw = as_float(raw.get("yaw"), previous_yaw)
        previous_yaw = yaw
        source_index_raw = raw.get("source_index")
        source_index = None if source_index_raw in {None, ""} else int(source_index_raw)
        waypoints.append(
            Waypoint(
                index=int(raw.get("index", raw.get("waypoint_index", index))),
                source_index=source_index,
                x=as_float(raw.get("x")),
                y=as_float(raw.get("y")),
                z=as_float(raw.get("z")),
                yaw=yaw,
                floor_id=wp_floor,
                segment_id=str(raw.get("segment_id") or ""),
                ramp_segment_id=raw.get("ramp_segment_id"),
                semantic_source=str(raw.get("semantic_source") or raw.get("source") or ""),
                is_ramp_waypoint=bool(raw.get("is_ramp_waypoint")),
                source_edge_id=raw.get("source_edge_id"),
            )
        )
    if not waypoints:
        detail = f" for floor_id={floor_id}" if floor_id else ""
        raise ValueError(f"No executable waypoints found{detail}")
    return waypoints


def transform_waypoints_to_odom(waypoints: list[Waypoint], odom_start: Pose3D, *, anchor: bool) -> list[Waypoint]:
    if not anchor:
        return waypoints
    first = waypoints[0]
    yaw_offset = normalize_angle(odom_start.yaw - first.yaw)
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    transformed: list[Waypoint] = []
    for waypoint in waypoints:
        dx = waypoint.x - first.x
        dy = waypoint.y - first.y
        transformed.append(
            Waypoint(
                index=waypoint.index,
                source_index=waypoint.source_index,
                x=odom_start.x + dx * cos_yaw - dy * sin_yaw,
                y=odom_start.y + dx * sin_yaw + dy * cos_yaw,
                z=odom_start.z + (waypoint.z - first.z),
                yaw=normalize_angle(waypoint.yaw + yaw_offset),
                floor_id=waypoint.floor_id,
                segment_id=waypoint.segment_id,
                ramp_segment_id=waypoint.ramp_segment_id,
                semantic_source=waypoint.semantic_source,
                is_ramp_waypoint=waypoint.is_ramp_waypoint,
                source_edge_id=waypoint.source_edge_id,
            )
        )
    return transformed


def segment_key(waypoint: Waypoint) -> str:
    if waypoint.is_ramp_waypoint:
        return "ramp"
    if waypoint.floor_id == "floor_1":
        return "floor_1"
    if waypoint.floor_id == "floor_2":
        return "floor_2"
    return waypoint.floor_id or "unknown"


def count_by_segment(waypoints: list[Waypoint], reached_count: int | None = None) -> dict[str, int]:
    counts = {"floor_1": 0, "ramp": 0, "floor_2": 0, "other": 0}
    selected = waypoints if reached_count is None else waypoints[: max(0, min(reached_count, len(waypoints)))]
    for waypoint in selected:
        key = segment_key(waypoint)
        if key in counts:
            counts[key] += 1
        else:
            counts["other"] += 1
    return counts


class MultifloorRampFollower:
    def __init__(
        self,
        *,
        rclpy: Any,
        Node: Any,
        Twist: Any,
        Odometry: Any,
        args: argparse.Namespace,
        runtime_input: dict[str, Any],
        profile: dict[str, Any],
        params: PidParams,
        waypoints: list[Waypoint],
    ) -> None:
        class _Node(Node):
            pass

        self.node = _Node("rslg_gazebo_multifloor_ramp_follower")
        self.rclpy = rclpy
        self.Twist = Twist
        self.args = args
        self.runtime_input = runtime_input
        self.profile = profile
        self.params = params
        self.original_waypoints = waypoints
        self.waypoints = waypoints
        self.query_id = query_id_from_runtime_input(runtime_input, args.query_id)
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trajectory_csv = self.output_dir / f"{self.query_id}_multifloor_ramp_gazebo_pid_trajectory.csv"
        self.summary_json = self.output_dir / f"{self.query_id}_multifloor_ramp_gazebo_pid_summary.json"
        self.csv_handle = self.trajectory_csv.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=[
                "wall_time_utc",
                "elapsed_sec",
                "odom_x",
                "odom_y",
                "odom_z",
                "odom_yaw",
                "target_index",
                "target_source_index",
                "target_x",
                "target_y",
                "target_z",
                "target_yaw",
                "target_floor_id",
                "target_segment_id",
                "target_ramp_segment_id",
                "target_is_ramp_waypoint",
                "distance_to_target_xy",
                "z_delta_to_target",
                "linear_x",
                "angular_z",
                "event",
            ],
        )
        self.csv_writer.writeheader()
        self.pose: Pose3D | None = None
        self.odom_received = False
        self.odom_z_observed = False
        self.odom_z_min: float | None = None
        self.odom_z_max: float | None = None
        self.cmd_vel_publish_count = 0
        self.current_index = 0
        self.done = False
        self.timeout = False
        self.failure_reason: list[str] = []
        self.start_monotonic = time.monotonic()
        self.first_control_monotonic: float | None = None
        self.end_monotonic: float | None = None
        self.anchor_applied = False
        self.publisher = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.subscription = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.timer = self.node.create_timer(1.0 / float(args.rate_hz), self._on_timer)
        self.node.get_logger().info(
            f"RSLG-SLAM multifloor ramp follower ready: query={self.query_id}, "
            f"waypoints={len(self.waypoints)}, profile={args.profile_id}"
        )

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        z = float(position.z)
        self.pose = Pose3D(
            x=float(position.x),
            y=float(position.y),
            z=z,
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        self.odom_received = True
        self.odom_z_observed = True
        self.odom_z_min = z if self.odom_z_min is None else min(self.odom_z_min, z)
        self.odom_z_max = z if self.odom_z_max is None else max(self.odom_z_max, z)

    def _publish_stop(self) -> None:
        twist = self.Twist()
        self.publisher.publish(twist)
        self.cmd_vel_publish_count += 1

    def _write_row(self, *, target: Waypoint | None, distance: float, linear: float, angular: float, event: str) -> None:
        pose = self.pose or Pose3D(0.0, 0.0, 0.0, 0.0)
        elapsed = time.monotonic() - self.start_monotonic
        self.csv_writer.writerow(
            {
                "wall_time_utc": utc_now(),
                "elapsed_sec": f"{elapsed:.6f}",
                "odom_x": f"{pose.x:.6f}",
                "odom_y": f"{pose.y:.6f}",
                "odom_z": f"{pose.z:.6f}",
                "odom_yaw": f"{pose.yaw:.6f}",
                "target_index": self.current_index,
                "target_source_index": "" if target is None or target.source_index is None else target.source_index,
                "target_x": "" if target is None else f"{target.x:.6f}",
                "target_y": "" if target is None else f"{target.y:.6f}",
                "target_z": "" if target is None else f"{target.z:.6f}",
                "target_yaw": "" if target is None else f"{target.yaw:.6f}",
                "target_floor_id": "" if target is None else target.floor_id,
                "target_segment_id": "" if target is None else target.segment_id,
                "target_ramp_segment_id": "" if target is None or target.ramp_segment_id is None else target.ramp_segment_id,
                "target_is_ramp_waypoint": "" if target is None else str(target.is_ramp_waypoint),
                "distance_to_target_xy": "" if math.isnan(distance) else f"{distance:.6f}",
                "z_delta_to_target": "" if target is None else f"{target.z - pose.z:.6f}",
                "linear_x": f"{linear:.6f}",
                "angular_z": f"{angular:.6f}",
                "event": event,
            }
        )
        self.csv_handle.flush()

    def _finish(self, *, timeout: bool, reason: str | None = None) -> None:
        if self.done:
            return
        self.timeout = timeout
        if reason:
            self.failure_reason.append(reason)
        if self.args.stop_at_end:
            self._publish_stop()
        self.end_monotonic = time.monotonic()
        self._write_row(target=None, distance=float("nan"), linear=0.0, angular=0.0, event="finish")
        self.done = True

    def _timeout_limit(self) -> float:
        timeout_limit = float(self.args.goal_timeout_sec)
        if self.args.duration_sec is not None and float(self.args.duration_sec) > 0:
            timeout_limit = float(self.args.duration_sec)
        if timeout_limit <= 0:
            timeout_limit = float(self.params.timeout_sec)
        return timeout_limit

    def _on_timer(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if now - self.start_monotonic < float(self.args.start_delay_sec):
            return
        if self.pose is None:
            if now - self.start_monotonic > self._timeout_limit():
                self._finish(timeout=True, reason="odom_not_received_before_timeout")
            return

        if self.first_control_monotonic is None:
            self.first_control_monotonic = now
            if self.args.anchor_first_waypoint_to_odom_start:
                self.waypoints = transform_waypoints_to_odom(self.original_waypoints, self.pose, anchor=True)
                self.anchor_applied = True

        if now - (self.first_control_monotonic or now) > self._timeout_limit():
            self._finish(timeout=True, reason="goal_timeout_sec_exceeded")
            return

        if self.current_index >= len(self.waypoints):
            self._finish(timeout=False)
            return

        pose = self.pose
        target = self.waypoints[self.current_index]
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx) if distance > 1e-9 else pose.yaw
        heading_error = normalize_angle(heading - pose.yaw)
        yaw_error = normalize_angle(target.yaw - pose.yaw)
        final_target = self.current_index == len(self.waypoints) - 1

        if distance <= self.params.waypoint_tolerance:
            if not final_target or abs(yaw_error) <= self.params.yaw_tolerance:
                self._write_row(target=target, distance=distance, linear=0.0, angular=0.0, event="waypoint_reached")
                self.current_index += 1
                if self.current_index >= len(self.waypoints):
                    self._finish(timeout=False)
                return

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
        self.publisher.publish(twist)
        self.cmd_vel_publish_count += 1
        self._write_row(target=target, distance=distance, linear=linear, angular=angular, event="cmd_vel")

    def summary(self) -> dict[str, Any]:
        pose = self.pose
        final_target = self.waypoints[-1]
        final_error = None if pose is None else round(math.hypot(final_target.x - pose.x, final_target.y - pose.y), 6)
        end = self.end_monotonic or time.monotonic()
        totals = count_by_segment(self.waypoints)
        reached = count_by_segment(self.waypoints, self.current_index)
        odom_z_changed = (
            self.odom_z_min is not None
            and self.odom_z_max is not None
            and abs(float(self.odom_z_max) - float(self.odom_z_min)) > 0.02
        )
        return {
            "schema_name": "rslg_multifloor_ramp_gazebo_execution_summary",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "query_id": self.query_id,
            "profile_id": self.args.profile_id,
            "runtime_input_json": str(Path(self.args.runtime_input_json)),
            "profile_json": str(Path(self.args.profile_json)),
            "frame_id": self.args.frame_id,
            "odom_topic": self.args.odom_topic,
            "cmd_vel_topic": self.args.cmd_vel_topic,
            "anchor_first_waypoint_to_odom_start": bool(self.args.anchor_first_waypoint_to_odom_start),
            "anchor_applied": self.anchor_applied,
            "waypoints_total": len(self.waypoints),
            "waypoints_reached": min(self.current_index, len(self.waypoints)),
            "segment_waypoints_total": totals,
            "segment_waypoints_reached": reached,
            "floor_1_waypoints_reached": reached["floor_1"] >= totals["floor_1"] and totals["floor_1"] > 0,
            "ramp_waypoints_reached": reached["ramp"] >= totals["ramp"] and totals["ramp"] > 0,
            "floor_2_waypoints_reached": reached["floor_2"] >= totals["floor_2"] and totals["floor_2"] > 0,
            "final_approach_reached": self.current_index >= len(self.waypoints) and not self.timeout,
            "final_error": final_error,
            "timeout": self.timeout,
            "failure_reason": self.failure_reason,
            "odom_received": self.odom_received,
            "odom_z_observed": self.odom_z_observed,
            "odom_z_min": None if self.odom_z_min is None else round(self.odom_z_min, 6),
            "odom_z_max": None if self.odom_z_max is None else round(self.odom_z_max, 6),
            "odom_z_changed": odom_z_changed,
            "odom_z_note": "Gazebo/TurtleBot odom may remain planar; z constancy is reported but is not a standalone failure.",
            "cmd_vel_published": self.cmd_vel_publish_count > 0,
            "cmd_vel_publish_count": self.cmd_vel_publish_count,
            "simulated_duration": round(end - self.start_monotonic, 6),
            "output_trajectory_csv": str(self.trajectory_csv),
            "claim_boundary": {
                "gazebo_simulation_only": True,
                "ramp_surrogate": True,
                "physical_stair_climbing_claimed": False,
                "physical_robot_claimed": False,
                "nav2_required": False,
                "amcl_required": False,
                "map_server_required": False,
                "global_collision_free_guarantee_claimed": False,
            },
        }

    def close(self) -> None:
        try:
            self.csv_handle.close()
        finally:
            write_json(self.summary_json, self.summary())


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--goal-timeout-sec", type=float, default=420.0)
    parser.add_argument("--duration-sec", type=float, default=None)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--start-delay-sec", type=float, default=1.0)
    parser.add_argument("--stop-at-end", action="store_true")
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--floor-id", default="all")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cmd-vel-topic", default=DEFAULT_CMD_VEL_TOPIC)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    return parser


def dry_run(
    args: argparse.Namespace,
    runtime_input: dict[str, Any],
    profile: dict[str, Any],
    params: PidParams,
    waypoints: list[Waypoint],
) -> int:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    query_id = query_id_from_runtime_input(runtime_input, args.query_id)
    totals = count_by_segment(waypoints)
    payload = {
        "schema_name": "rslg_multifloor_ramp_follower_dry_run",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id,
        "profile_id": args.profile_id,
        "runtime_input_json": str(args.runtime_input_json),
        "profile_json": str(args.profile_json),
        "selected_waypoint_count": len(waypoints),
        "segment_waypoints_total": totals,
        "floor_id_filter": args.floor_id,
        "first_waypoint": waypoints[0].__dict__,
        "last_waypoint": waypoints[-1].__dict__,
        "profile_params": params.__dict__,
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "ramp_connector_edge_id": runtime_input.get("ramp_connector_edge_id"),
        "forbidden_transition_edge_ids": runtime_input.get("forbidden_transition_edge_ids"),
        "blocked_goal_candidate_ids": runtime_input.get("blocked_goal_candidate_ids"),
        "target_approach_id": runtime_input.get("target_approach_id"),
        "requires_nav2": False,
        "requires_amcl": False,
        "requires_map_server": False,
        "claim_boundary": runtime_input.get("claim_boundary"),
        "profile_evidence_summary": profile.get("evidence_summary") if isinstance(profile, dict) else None,
    }
    output = args.output_dir / f"{query_id}_multifloor_ramp_follower_dry_run_summary.json"
    write_json(output, payload)
    print(json.dumps(payload, indent=2, sort_keys=False), flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    runtime_input = read_json(args.runtime_input_json)
    profile, params = load_profile(args.profile_json, args.profile_id)
    waypoints = extract_waypoints(runtime_input, args.floor_id)

    if args.dry_run:
        return dry_run(args, runtime_input, profile, params, waypoints)

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise SystemExit(f"ROS 2 Python dependencies are unavailable: {exc}") from exc

    rclpy.init(args=None)
    follower = MultifloorRampFollower(
        rclpy=rclpy,
        Node=Node,
        Twist=Twist,
        Odometry=Odometry,
        args=args,
        runtime_input=runtime_input,
        profile=profile,
        params=params,
        waypoints=waypoints,
    )
    try:
        while rclpy.ok() and not follower.done:
            rclpy.spin_once(follower.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        follower._finish(timeout=True, reason="keyboard_interrupt")
    finally:
        follower._publish_stop()
        follower.close()
        follower.node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(follower.summary(), indent=2, sort_keys=False), flush=True)
    return 0 if not follower.timeout else 2


if __name__ == "__main__":
    raise SystemExit(main())
