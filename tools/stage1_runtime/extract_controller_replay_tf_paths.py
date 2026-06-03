#!/usr/bin/env python3
"""Replay a controller probe bag and extract controller path topics in map frame."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

os.environ.setdefault("ROS_DOMAIN_ID", "84")
os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


PATH_TOPICS = ["/local_plan", "/transformed_global_plan", "/received_global_plan", "/plan"]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class Extractor(Node):
    def __init__(self, topics: list[str], max_messages_per_topic: int, sample_every: int) -> None:
        super().__init__("boxfusion_controller_replay_path_extractor")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.max_messages_per_topic = max_messages_per_topic
        self.sample_every = max(1, sample_every)
        self.message_counts = {topic: 0 for topic in topics}
        self.paths: dict[str, list[dict[str, Any]]] = {topic: [] for topic in topics}
        self.cmd_vel: list[dict[str, Any]] = []
        for topic in topics:
            self.create_subscription(NavPath, topic, lambda msg, t=topic: self.path_cb(t, msg), 50)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 50)

    def cmd_cb(self, msg: Twist) -> None:
        if len(self.cmd_vel) >= self.max_messages_per_topic:
            return
        self.cmd_vel.append({
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
            "spin_like": abs(float(msg.linear.x)) < 0.03 and abs(float(msg.angular.z)) > 0.35,
            "moving_turn": abs(float(msg.linear.x)) >= 0.03 and abs(float(msg.angular.z)) > 0.35,
        })

    def transform_xy(self, x: float, y: float, frame_id: str, stamp: Any) -> tuple[float, float, str]:
        if frame_id in {"", "map"}:
            return x, y, "already_map"
        for when, mode in [(stamp, "tf_at_path_stamp"), (rclpy.time.Time(), "tf_latest")]:
            try:
                transform = self.tf_buffer.lookup_transform("map", frame_id, when, timeout=Duration(seconds=0.05))
                t = transform.transform.translation
                yaw = yaw_from_quat(transform.transform.rotation)
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                return (
                    float(t.x) + cos_yaw * x - sin_yaw * y,
                    float(t.y) + sin_yaw * x + cos_yaw * y,
                    mode,
                )
            except TransformException:
                continue
        if frame_id == "odom":
            return x, y, "identity_map_from_odom_fallback"
        return x, y, "untransformed_no_tf"

    def path_cb(self, topic: str, msg: NavPath) -> None:
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
        count = self.message_counts[topic]
        if count % self.sample_every != 0:
            return
        if len(self.paths.setdefault(topic, [])) >= self.max_messages_per_topic:
            return
        frame_id = msg.header.frame_id or "map"
        points = []
        modes: dict[str, int] = {}
        for pose in msg.poses:
            x, y, mode = self.transform_xy(float(pose.pose.position.x), float(pose.pose.position.y), frame_id, msg.header.stamp)
            modes[mode] = modes.get(mode, 0) + 1
            points.append({"x": x, "y": y})
        self.paths[topic].append({
            "message_index": count,
            "frame_id": frame_id,
            "pose_count": len(points),
            "transform_modes": modes,
            "points": points,
        })


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--topics", default=",".join(PATH_TOPICS))
    parser.add_argument("--duration-sec", type=float, default=180.0)
    parser.add_argument("--startup-wait-sec", type=float, default=1.0)
    parser.add_argument("--max-messages-per-topic", type=int, default=2000)
    parser.add_argument("--sample-every", type=int, default=1)
    args = parser.parse_args()

    topics = [topic.strip() for topic in args.topics.split(",") if topic.strip()]
    rclpy.init()
    node = Extractor(topics, args.max_messages_per_topic, args.sample_every)
    play = subprocess.Popen(["ros2", "bag", "play", args.bag_dir.as_posix()], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    deadline = time.time() + float(args.duration_sec)
    time.sleep(float(args.startup_wait_sec))
    try:
        while rclpy.ok() and time.time() < deadline and play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if play.poll() is None:
            play.terminate()
            try:
                play.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                play.kill()
        stdout, stderr = play.communicate(timeout=3.0) if play.poll() is not None else ("", "")
        node.destroy_node()
        rclpy.shutdown()

    cmd_vel_summary = {
        "message_count": len(node.cmd_vel),
        "spin_like_count": sum(1 for row in node.cmd_vel if row["spin_like"]),
        "moving_turn_count": sum(1 for row in node.cmd_vel if row["moving_turn"]),
    }
    payload = {
        "artifact_type": "controller_replay_tf_map_paths",
        "version": "v0_1",
        "created_utc": now_iso(),
        "bag_dir": args.bag_dir.as_posix(),
        "topics": topics,
        "message_counts": node.message_counts,
        "paths": node.paths,
        "cmd_vel_summary": cmd_vel_summary,
        "ros2_bag_play_returncode": play.returncode,
        "ros2_bag_play_stdout_tail": stdout[-4000:],
        "ros2_bag_play_stderr_tail": stderr[-4000:],
    }
    write_json(args.output_json, payload)
    print(json.dumps({"output_json": args.output_json.as_posix(), "message_counts": node.message_counts, "cmd_vel_summary": cmd_vel_summary}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
