#!/usr/bin/env python3
"""Planar visual-kinematic proxy for a Gazebo quadruped model.

This node deliberately does not command joints or implement a gait controller.
It integrates bounded planar Twist commands, publishes odometry and TF, and
updates a Gazebo entity pose when a SetEntityState service is available.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .profile_loader import load_robot_profile


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class QuadrupedKinematicProxyNode(Node):
    """Bounded planar kinematic proxy with Gazebo visual pose updates."""

    SERVICE_CANDIDATES = (
        "/set_entity_state",
        "/demo/set_entity_state",
        "/gazebo/set_entity_state",
    )

    def __init__(
        self,
        profile: dict[str, Any],
        *,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        log_path: Path | None = None,
        manifest_path: Path | None = None,
    ) -> None:
        super().__init__("quadruped_kinematic_proxy")
        self.profile = profile
        self.x = float(initial_x)
        self.y = float(initial_y)
        self.yaw = float(initial_yaw)
        self.v = 0.0
        self.wz = 0.0
        self.command_count = 0
        self.integration_count = 0
        self.gazebo_update_success_count = 0
        self.gazebo_update_failure_count = 0
        self.last_command_ns: int | None = None
        self.command_timeout_sec = float(profile.get("command_timeout_sec", 0.5))
        self.entity_z = float(profile.get("gazebo_entity_z", 0.35))
        self.log_path = log_path
        self.manifest_path = manifest_path
        self.shutdown_requested = False
        self.service_name: str | None = None
        self.service_client = None
        self.pending_future = None

        self.cmd_topic = str(profile.get("cmd_topic", "/cmd_vel"))
        self.odom_topic = str(profile.get("odom_topic", "/odom"))
        self.map_frame = str(profile.get("pose_tf_parent") or profile["map_frame"])
        self.base_frame = str(profile.get("pose_tf_child") or profile["base_frame"])
        self.entity_name = str(profile.get("gazebo_entity_name") or profile["robot_name"])
        self.max_linear = float(profile["max_linear_speed"])
        self.max_angular = float(profile["max_angular_speed"])
        self.rate_hz = float(profile["control_rate_hz"])
        self.dt = 1.0 / self.rate_hz

        self.create_subscription(Twist, self.cmd_topic, self._on_command, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self._tick)
        self.discovery_timer = self.create_timer(1.0, self._discover_service)
        self._record("startup", claim_boundary="visual_kinematic_proxy_only")
        self._write_manifest()

    def _record(self, event: str, **fields: Any) -> None:
        payload = {"timestamp_utc": now_iso(), "event": event, **fields}
        self.get_logger().info(json.dumps(payload, sort_keys=True))
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, sort_keys=True) + "\n")

    def _write_manifest(self) -> None:
        if self.manifest_path is None:
            return
        payload = {
            "timestamp_utc": now_iso(),
            "node": self.get_name(),
            "robot_name": self.profile["robot_name"],
            "robot_type": self.profile["robot_type"],
            "claim_boundary": self.profile.get("claim_boundary"),
            "is_physical_locomotion": self.profile.get("is_physical_locomotion"),
            "cmd_topic": self.cmd_topic,
            "odom_topic": self.odom_topic,
            "tf_parent": self.map_frame,
            "tf_child": self.base_frame,
            "gazebo_entity_name": self.entity_name,
            "gazebo_set_entity_state_service": self.service_name,
            "command_count": self.command_count,
            "integration_count": self.integration_count,
            "gazebo_update_success_count": self.gazebo_update_success_count,
            "gazebo_update_failure_count": self.gazebo_update_failure_count,
            "last_pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
        }
        self.manifest_path.parent.mkdir(parents=True, exist_ok=True)
        self.manifest_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _on_command(self, msg: Twist) -> None:
        requested_v = float(msg.linear.x)
        requested_wz = float(msg.angular.z)
        self.v = clamp(requested_v, -self.max_linear, self.max_linear)
        self.wz = clamp(requested_wz, -self.max_angular, self.max_angular)
        self.last_command_ns = self.get_clock().now().nanoseconds
        self.command_count += 1
        self._record(
            "command",
            requested_linear_x=requested_v,
            requested_angular_z=requested_wz,
            applied_linear_x=self.v,
            applied_angular_z=self.wz,
        )

    def _discover_service(self) -> None:
        if self.service_client is not None:
            return
        available = {
            name
            for name, types in self.get_service_names_and_types()
            if "gazebo_msgs/srv/SetEntityState" in types
        }
        selected = next((name for name in self.SERVICE_CANDIDATES if name in available), None)
        if selected is None and available:
            selected = sorted(available)[0]
        if selected is not None:
            self.service_name = selected
            self.service_client = self.create_client(SetEntityState, selected)
            self._record("gazebo_service_detected", service=selected)
            self._write_manifest()

    def _publish_pose(self, stamp: Any) -> None:
        qx, qy, qz, qw = yaw_quaternion(self.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

    def _update_gazebo(self) -> None:
        if self.service_client is None or not self.service_client.service_is_ready():
            return
        if self.pending_future is not None and not self.pending_future.done():
            return
        if self.pending_future is not None:
            try:
                response = self.pending_future.result()
                if response is not None and response.success:
                    self.gazebo_update_success_count += 1
                else:
                    self.gazebo_update_failure_count += 1
            except Exception as exc:
                self.gazebo_update_failure_count += 1
                self._record("gazebo_update_exception", error=str(exc))

        request = SetEntityState.Request()
        request.state.name = self.entity_name
        request.state.reference_frame = "world"
        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = self.entity_z
        qx, qy, qz, qw = yaw_quaternion(self.yaw)
        request.state.pose.orientation.x = qx
        request.state.pose.orientation.y = qy
        request.state.pose.orientation.z = qz
        request.state.pose.orientation.w = qw
        request.state.twist.linear.x = 0.0
        request.state.twist.angular.z = 0.0
        self.pending_future = self.service_client.call_async(request)

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.last_command_ns is not None:
            age_sec = (now.nanoseconds - self.last_command_ns) / 1e9
            if age_sec > self.command_timeout_sec and (self.v != 0.0 or self.wz != 0.0):
                self.v = 0.0
                self.wz = 0.0
                self._record("safe_stop_timeout", command_age_sec=age_sec)

        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * self.dt), math.cos(self.yaw + self.wz * self.dt))
        self.integration_count += 1
        self._publish_pose(now.to_msg())
        self._update_gazebo()
        if self.integration_count % max(1, int(self.rate_hz)) == 0:
            self._record("integrated_pose", x=self.x, y=self.y, yaw=self.yaw, linear_x=self.v, angular_z=self.wz)
            self._write_manifest()

    def safe_stop(self) -> None:
        self.v = 0.0
        self.wz = 0.0
        stamp = self.get_clock().now().to_msg()
        self._publish_pose(stamp)
        self._update_gazebo()
        self._record("safe_stop_shutdown", x=self.x, y=self.y, yaw=self.yaw)
        self._write_manifest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-profile", type=Path, required=True)
    parser.add_argument("--initial-x", type=float, default=0.0)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--log-path", type=Path)
    parser.add_argument("--manifest-path", type=Path)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = QuadrupedKinematicProxyNode(
        load_robot_profile(args.robot_profile),
        initial_x=args.initial_x,
        initial_y=args.initial_y,
        initial_yaw=args.initial_yaw,
        log_path=args.log_path,
        manifest_path=args.manifest_path,
    )

    def request_shutdown(_signum: int, _frame: Any) -> None:
        node.shutdown_requested = True

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.safe_stop()
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
