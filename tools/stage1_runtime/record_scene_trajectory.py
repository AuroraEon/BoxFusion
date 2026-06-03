#!/usr/bin/env python3
"""Record map-frame robot trajectory samples for Stage1 route debugging."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def rgba(r: float, g: float, b: float, a: float) -> Any:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def yaw_from_quat(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def marker_qos() -> Any:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class TrajectoryRecorder(Node):  # pragma: no cover - requires ROS graph
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("boxfusion_scene_trajectory_recorder")
        self.args = args
        self.samples: list[dict[str, Any]] = []
        self.last_odom: dict[str, Any] | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, marker_qos()) if args.marker_topic else None
        self.path_pub = self.create_publisher(NavPath, args.path_topic, marker_qos()) if args.path_topic else None
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
        self.timer = self.create_timer(1.0 / max(args.rate_hz, 0.1), self.sample)

    def on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        self.last_odom = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw": yaw_from_quat(pose.orientation),
            "frame_id": msg.header.frame_id or "odom",
            "source": self.args.odom_topic,
        }

    def tf_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform(self.args.fixed_frame, self.args.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except (TransformException, Exception):
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return {
            "x": float(t.x),
            "y": float(t.y),
            "z": float(t.z),
            "yaw": yaw_from_quat(q),
            "frame_id": self.args.fixed_frame,
            "source": f"/tf {self.args.fixed_frame}->{self.args.base_frame}",
        }

    def sample(self) -> None:
        pose = self.tf_pose() or self.last_odom
        if pose is None:
            return
        row = dict(pose)
        row["sample_index"] = len(self.samples)
        row["time_wall_sec"] = time.time()
        self.samples.append(row)
        self.publish_debug()

    def publish_debug(self) -> None:
        if not self.samples:
            return
        stamp = self.get_clock().now().to_msg()
        if self.path_pub is not None:
            path = NavPath()
            path.header.frame_id = self.samples[-1].get("frame_id", self.args.fixed_frame)
            path.header.stamp = stamp
            for sample in self.samples:
                pose = PoseStamped()
                pose.header.frame_id = path.header.frame_id
                pose.header.stamp = stamp
                pose.pose.position.x = float(sample["x"])
                pose.pose.position.y = float(sample["y"])
                yaw = float(sample.get("yaw", 0.0))
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_pub.publish(path)
        if self.marker_pub is not None:
            marker = Marker()
            marker.header.frame_id = self.samples[-1].get("frame_id", self.args.fixed_frame)
            marker.header.stamp = stamp
            marker.ns = self.args.marker_namespace
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.06
            marker.color = rgba(1.0, 0.25, 0.08, 1.0)
            for sample in self.samples:
                point = Point()
                point.x = float(sample["x"])
                point.y = float(sample["y"])
                point.z = 0.72
                marker.points.append(point)
            self.marker_pub.publish(MarkerArray(markers=[marker]))

    def write_outputs(self) -> None:
        payload = {
            "artifact_type": "stage1_scene_trajectory_recording",
            "created_wall_sec": time.time(),
            "fixed_frame": self.args.fixed_frame,
            "base_frame": self.args.base_frame,
            "sample_count": len(self.samples),
            "samples": self.samples,
        }
        write_json(self.args.output_json, payload)
        if self.args.output_csv:
            self.args.output_csv.parent.mkdir(parents=True, exist_ok=True)
            with self.args.output_csv.open("w", encoding="utf-8", newline="") as handle:
                fields = ["sample_index", "time_wall_sec", "x", "y", "z", "yaw", "frame_id", "source"]
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                for sample in self.samples:
                    writer.writerow({field: sample.get(field) for field in fields})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--fixed-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--marker-topic", default="")
    parser.add_argument("--path-topic", default="")
    parser.add_argument("--marker-namespace", default="stage1_nav/executed_trajectory_recorder")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if IMPORT_ERROR:
        print(json.dumps({"rclpy_import_error": IMPORT_ERROR}), file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = TrajectoryRecorder(args)
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration_sec > 0 and time.monotonic() - start >= args.duration_sec:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.write_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
