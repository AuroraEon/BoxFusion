#!/usr/bin/env python3
"""ROS 2 PID waypoint follower for bounded RSLG-SLAM Gazebo smoke tests.

This node consumes a RouteResult-derived PID runtime input, subscribes to odom,
publishes cmd_vel, and logs the executed trajectory. It intentionally does not
import or launch Nav2, AMCL, map_server, Stage-A, RGB-D inference, navigation
actions, or physical robot code.
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
from typing import Any, Optional


DEFAULT_PROFILE_ID = "practical_zero_collision"
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"
DEFAULT_ODOM_TOPIC = "/odom"


@dataclass(frozen=True)
class PidParams:
    dt: float
    max_linear_velocity: float
    max_angular_velocity: float
    linear_gain: float
    angular_gain: float
    waypoint_tolerance: float
    yaw_tolerance: float
    timeout_sec: float


@dataclass(frozen=True)
class Waypoint:
    source_index: int
    x: float
    y: float
    z: float
    yaw: float
    floor_id: str
    segment_id: str
    source: str


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_profile(profile_json: Path, profile_id: str) -> tuple[dict[str, Any], PidParams]:
    payload = read_json(profile_json)
    profiles = payload.get("profiles") or {}
    profile = profiles.get(profile_id)
    if not isinstance(profile, dict):
        raise KeyError(f"profile_id not found in {profile_json}: {profile_id}")
    params = profile.get("params") or {}
    parsed = PidParams(
        dt=float(params.get("dt", 0.1)),
        max_linear_velocity=float(params.get("max_linear_velocity", 0.15)),
        max_angular_velocity=float(params.get("max_angular_velocity", 0.8)),
        linear_gain=float(params.get("linear_gain", 0.8)),
        angular_gain=float(params.get("angular_gain", 2.0)),
        waypoint_tolerance=float(params.get("waypoint_tolerance", 0.08)),
        yaw_tolerance=float(params.get("yaw_tolerance", 0.25)),
        timeout_sec=float(params.get("timeout_sec", 240.0)),
    )
    return profile, parsed


def _waypoint_yaw(raw: dict[str, Any], fallback: float = 0.0) -> float:
    value = raw.get("yaw")
    if value is None:
        return fallback
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def extract_waypoints(pid_input: dict[str, Any], floor_id: Optional[str]) -> list[Waypoint]:
    raw_waypoints = pid_input.get("runtime_waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("PID input does not contain runtime_waypoints")

    waypoints: list[Waypoint] = []
    previous_yaw = 0.0
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, dict):
            continue
        wp_floor = str(raw.get("floor_id") or "")
        if floor_id and wp_floor != floor_id:
            continue
        yaw = _waypoint_yaw(raw, previous_yaw)
        previous_yaw = yaw
        waypoints.append(
            Waypoint(
                source_index=int(raw.get("waypoint_index", index)),
                x=float(raw["x"]),
                y=float(raw["y"]),
                z=float(raw.get("z", 0.0)),
                yaw=yaw,
                floor_id=wp_floor,
                segment_id=str(raw.get("segment_id") or ""),
                source=str(raw.get("source") or ""),
            )
        )
    if not waypoints:
        detail = f" for floor_id={floor_id}" if floor_id else ""
        raise ValueError(f"No executable waypoints found{detail}")
    return waypoints


def query_id_from_pid_input(pid_input: dict[str, Any], fallback: Optional[str]) -> str:
    if fallback:
        return fallback
    identity = pid_input.get("identity") or {}
    return str(identity.get("query_id") or "unknown_query")


def floor_counts(pid_input: dict[str, Any]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for raw in pid_input.get("runtime_waypoints") or []:
        if isinstance(raw, dict):
            floor_id = str(raw.get("floor_id") or "")
            counts[floor_id] = counts.get(floor_id, 0) + 1
    return dict(sorted(counts.items()))


def transform_waypoints_to_odom(
    waypoints: list[Waypoint],
    odom_start: Pose2D,
    *,
    anchor: bool,
) -> list[Waypoint]:
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
        x = odom_start.x + dx * cos_yaw - dy * sin_yaw
        y = odom_start.y + dx * sin_yaw + dy * cos_yaw
        yaw = normalize_angle(waypoint.yaw + yaw_offset)
        transformed.append(
            Waypoint(
                source_index=waypoint.source_index,
                x=x,
                y=y,
                z=waypoint.z,
                yaw=yaw,
                floor_id=waypoint.floor_id,
                segment_id=waypoint.segment_id,
                source=waypoint.source,
            )
        )
    return transformed


class GazeboPidFollower:
    def __init__(
        self,
        *,
        rclpy: Any,
        Node: Any,
        Twist: Any,
        Odometry: Any,
        args: argparse.Namespace,
        pid_input: dict[str, Any],
        profile: dict[str, Any],
        params: PidParams,
        waypoints: list[Waypoint],
    ) -> None:
        class _Node(Node):
            pass

        self.node = _Node("rslg_gazebo_pid_follower")
        self.rclpy = rclpy
        self.Twist = Twist
        self.args = args
        self.pid_input = pid_input
        self.profile = profile
        self.params = params
        self.original_waypoints = waypoints
        self.waypoints = waypoints
        self.query_id = query_id_from_pid_input(pid_input, args.query_id)
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trajectory_csv = self.output_dir / f"{self.query_id}_gazebo_pid_trajectory.csv"
        self.summary_json = self.output_dir / f"{self.query_id}_gazebo_pid_summary.json"
        self.csv_handle = self.trajectory_csv.open("w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=[
                "wall_time_utc",
                "elapsed_sec",
                "odom_x",
                "odom_y",
                "odom_yaw",
                "target_index",
                "target_source_index",
                "target_x",
                "target_y",
                "target_yaw",
                "target_floor_id",
                "target_segment_id",
                "distance_to_target",
                "linear_x",
                "angular_z",
                "event",
            ],
        )
        self.csv_writer.writeheader()
        self.pose: Optional[Pose2D] = None
        self.odom_received = False
        self.cmd_vel_publish_count = 0
        self.current_index = 0
        self.done = False
        self.timeout = False
        self.failure_reason: list[str] = []
        self.start_monotonic = time.monotonic()
        self.first_control_monotonic: Optional[float] = None
        self.end_monotonic: Optional[float] = None
        self.anchor_applied = False
        self.publisher = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.subscription = self.node.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.timer = self.node.create_timer(1.0 / float(args.rate_hz), self._on_timer)
        self.node.get_logger().info(
            f"RSLG-SLAM Gazebo PID follower ready: query={self.query_id}, "
            f"waypoints={len(self.waypoints)}, profile={args.profile_id}, "
            f"floor_filter={args.floor_id or 'none'}"
        )

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        self.pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        self.odom_received = True

    def _publish_stop(self) -> None:
        twist = self.Twist()
        self.publisher.publish(twist)
        self.cmd_vel_publish_count += 1

    def _write_row(self, *, target: Optional[Waypoint], distance: float, linear: float, angular: float, event: str) -> None:
        pose = self.pose or Pose2D(0.0, 0.0, 0.0)
        elapsed = time.monotonic() - self.start_monotonic
        self.csv_writer.writerow(
            {
                "wall_time_utc": utc_now(),
                "elapsed_sec": f"{elapsed:.6f}",
                "odom_x": f"{pose.x:.6f}",
                "odom_y": f"{pose.y:.6f}",
                "odom_yaw": f"{pose.yaw:.6f}",
                "target_index": self.current_index,
                "target_source_index": "" if target is None else target.source_index,
                "target_x": "" if target is None else f"{target.x:.6f}",
                "target_y": "" if target is None else f"{target.y:.6f}",
                "target_yaw": "" if target is None else f"{target.yaw:.6f}",
                "target_floor_id": "" if target is None else target.floor_id,
                "target_segment_id": "" if target is None else target.segment_id,
                "distance_to_target": "" if math.isnan(distance) else f"{distance:.6f}",
                "linear_x": f"{linear:.6f}",
                "angular_z": f"{angular:.6f}",
                "event": event,
            }
        )
        self.csv_handle.flush()

    def _finish(self, *, timeout: bool, reason: Optional[str] = None) -> None:
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

    def _on_timer(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if now - self.start_monotonic < float(self.args.start_delay_sec):
            return
        if self.pose is None:
            if now - self.start_monotonic > float(self.args.goal_timeout_sec):
                self._finish(timeout=True, reason="odom_not_received_before_timeout")
            return

        if self.first_control_monotonic is None:
            self.first_control_monotonic = now
            if self.args.anchor_first_waypoint_to_odom_start:
                self.waypoints = transform_waypoints_to_odom(self.original_waypoints, self.pose, anchor=True)
                self.anchor_applied = True

        elapsed_control = now - (self.first_control_monotonic or now)
        timeout_limit = float(self.args.goal_timeout_sec)
        if timeout_limit <= 0:
            timeout_limit = float(self.params.timeout_sec)
        if elapsed_control > timeout_limit:
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
            angular = clamp(
                self.params.angular_gain * yaw_error,
                -self.params.max_angular_velocity,
                self.params.max_angular_velocity,
            )
        else:
            angular = clamp(
                self.params.angular_gain * heading_error,
                -self.params.max_angular_velocity,
                self.params.max_angular_velocity,
            )
            if abs(heading_error) > math.pi / 2.0:
                linear = 0.0
            else:
                linear = clamp(
                    self.params.linear_gain * distance,
                    0.0,
                    self.params.max_linear_velocity,
                ) * max(0.0, math.cos(heading_error))
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)
        self.cmd_vel_publish_count += 1
        self._write_row(target=target, distance=distance, linear=linear, angular=angular, event="cmd_vel")

    def summary(self) -> dict[str, Any]:
        pose = self.pose
        final_target = self.waypoints[-1]
        if pose is None:
            final_error = None
        else:
            final_error = round(math.hypot(final_target.x - pose.x, final_target.y - pose.y), 6)
        end = self.end_monotonic or time.monotonic()
        return {
            "schema_name": "rslg_gazebo_pid_smoke_execution_summary",
            "schema_version": "0.1",
            "project_name": "RSLG-SLAM",
            "generated_utc": utc_now(),
            "query_id": self.query_id,
            "profile_id": self.args.profile_id,
            "pid_input_json": str(Path(self.args.pid_input_json)),
            "profile_json": str(Path(self.args.profile_json)),
            "floor_id_filter": self.args.floor_id,
            "frame_id": self.args.frame_id,
            "odom_topic": self.args.odom_topic,
            "cmd_vel_topic": self.args.cmd_vel_topic,
            "anchor_first_waypoint_to_odom_start": bool(self.args.anchor_first_waypoint_to_odom_start),
            "anchor_applied": self.anchor_applied,
            "waypoints_total": len(self.waypoints),
            "waypoints_reached": min(self.current_index, len(self.waypoints)),
            "final_error": final_error,
            "timeout": self.timeout,
            "failure_reason": self.failure_reason,
            "odom_received": self.odom_received,
            "cmd_vel_published": self.cmd_vel_publish_count > 0,
            "cmd_vel_publish_count": self.cmd_vel_publish_count,
            "simulated_duration": round(end - self.start_monotonic, 6),
            "output_trajectory_csv": str(self.trajectory_csv),
        }

    def close(self) -> None:
        try:
            self.csv_handle.close()
        finally:
            write_json(self.summary_json, self.summary())


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-input-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--profile-id", default=DEFAULT_PROFILE_ID)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--goal-timeout-sec", type=float, default=60.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--start-delay-sec", type=float, default=1.0)
    parser.add_argument("--stop-at-end", action="store_true")
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--floor-id", default=None)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cmd-vel-topic", default=DEFAULT_CMD_VEL_TOPIC)
    parser.add_argument("--anchor-first-waypoint-to-odom-start", action="store_true")
    return parser


def dry_run(args: argparse.Namespace, pid_input: dict[str, Any], profile: dict[str, Any], params: PidParams, waypoints: list[Waypoint]) -> int:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    query_id = query_id_from_pid_input(pid_input, args.query_id)
    payload = {
        "schema_name": "rslg_gazebo_pid_follower_dry_run",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "ok": True,
        "query_id": query_id,
        "profile_id": args.profile_id,
        "pid_input_json": str(args.pid_input_json),
        "profile_json": str(args.profile_json),
        "runtime_waypoint_count": len(pid_input.get("runtime_waypoints") or []),
        "selected_waypoint_count": len(waypoints),
        "floor_id_filter": args.floor_id,
        "floor_counts": floor_counts(pid_input),
        "first_waypoint": waypoints[0].__dict__,
        "last_waypoint": waypoints[-1].__dict__,
        "profile_params": params.__dict__,
        "anchor_first_waypoint_to_odom_start": bool(args.anchor_first_waypoint_to_odom_start),
        "requires_nav2": bool(pid_input.get("requires_nav2")),
        "requires_amcl": bool(pid_input.get("requires_amcl")),
        "runtime_policy": pid_input.get("runtime_policy"),
        "profile_evidence_summary": profile.get("evidence_summary"),
    }
    output = args.output_dir / f"{query_id}_gazebo_pid_dry_run_summary.json"
    write_json(output, payload)
    print(json.dumps(payload, indent=2, sort_keys=False), flush=True)
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    pid_input = read_json(args.pid_input_json)
    profile, params = load_profile(args.profile_json, args.profile_id)
    waypoints = extract_waypoints(pid_input, args.floor_id)

    if args.dry_run:
        return dry_run(args, pid_input, profile, params, waypoints)

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise SystemExit(f"ROS 2 Python dependencies are unavailable: {exc}") from exc

    rclpy.init(args=None)
    follower = GazeboPidFollower(
        rclpy=rclpy,
        Node=Node,
        Twist=Twist,
        Odometry=Odometry,
        args=args,
        pid_input=pid_input,
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

