#!/usr/bin/env python3
"""Probe the post-restructure Stage1 Step30P1 Gazebo/TurtleBot3/Nav2 data plane."""

from __future__ import annotations

import argparse
import json
import math
import os
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
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_md(path: Path, payload: dict[str, Any]) -> None:
    readiness = payload.get("readiness", {})
    topics = payload.get("topics", {})
    actions = payload.get("action_servers", {})
    lines = [
        "# Step30S2 Bringup Dataplane Report",
        "",
        f"Created: `{payload.get('created_utc')}`",
        "",
        f"Dataplane ready: `{readiness.get('staticloc_dataplane_ready')}`",
        "",
        "## Topics",
        "",
    ]
    for topic in ["/clock", "/odom", "/scan", "/tf", "/tf_static", "/map", "/cmd_vel"]:
        item = topics.get(topic, {})
        lines.append(f"- `{topic}`: publishers={item.get('publisher_count')} subscribers={item.get('subscription_count')} messages={item.get('message_count_observed')} rate_hz={item.get('observed_rate_hz')}")
    lines.extend(["", "## TF And Actions", ""])
    for key, value in payload.get("tf_frames", {}).items():
        lines.append(f"- `{key}`: `{value}`")
    for key, value in actions.items():
        role = (payload.get("action_server_roles") or {}).get(key)
        lines.append(f"- action `{key}`: `{value}` role=`{role}`")
    if payload.get("missing_conditions"):
        lines.extend(["", "## Missing Conditions", ""])
        lines.extend(f"- `{item}`" for item in payload["missing_conditions"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class DataplaneProbe(Node):  # pragma: no cover - ROS runtime only
    def __init__(self) -> None:
        super().__init__("boxfusion_stage1_step30p1_dataplane_probe")
        self.counts = {topic: 0 for topic in ["/clock", "/scan", "/odom", "/map", "/tf", "/tf_static", "/cmd_vel"]}
        self.first_seen: dict[str, float] = {}
        self.last_seen: dict[str, float] = {}
        self.tf_edges = {"/tf": set(), "/tf_static": set()}
        self.latest_odom: dict[str, Any] | None = None
        self.latest_scan: dict[str, Any] | None = None
        self.latest_map: dict[str, Any] | None = None

        qos_default = QoSProfile(depth=50)
        qos_default.reliability = ReliabilityPolicy.RELIABLE
        qos_sensor = QoSProfile(depth=50)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_map = QoSProfile(depth=1)
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_static = QoSProfile(depth=50)
        qos_static.reliability = ReliabilityPolicy.RELIABLE
        qos_static.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Clock, "/clock", lambda _msg: self.bump("/clock"), 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_sensor)
        self.create_subscription(Odometry, "/odom", self.odom_cb, qos_default)
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos_map)
        self.create_subscription(TFMessage, "/tf", self.tf_cb, qos_default)
        self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb, qos_static)
        self.create_subscription(Twist, "/cmd_vel", lambda _msg: self.bump("/cmd_vel"), qos_default)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.compute_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")

    def bump(self, topic: str) -> None:
        now = time.time()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.first_seen.setdefault(topic, now)
        self.last_seen[topic] = now

    def scan_cb(self, msg: LaserScan) -> None:
        self.bump("/scan")
        finite = [float(r) for r in msg.ranges if math.isfinite(float(r))]
        self.latest_scan = {
            "frame_id": msg.header.frame_id,
            "range_count": len(msg.ranges),
            "finite_count": len(finite),
            "finite_min": round(min(finite), 6) if finite else None,
            "finite_max": round(max(finite), 6) if finite else None,
        }

    def odom_cb(self, msg: Odometry) -> None:
        self.bump("/odom")
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": round(float(p.x), 6),
            "y": round(float(p.y), 6),
            "yaw": round(yaw_from_quat(q), 6),
        }

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.bump("/map")
        self.latest_map = {
            "frame_id": msg.header.frame_id,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {"x": float(msg.info.origin.position.x), "y": float(msg.info.origin.position.y)},
        }

    def tf_cb(self, msg: TFMessage) -> None:
        self.bump("/tf")
        for transform in msg.transforms:
            self.tf_edges["/tf"].add((transform.header.frame_id, transform.child_frame_id))

    def tf_static_cb(self, msg: TFMessage) -> None:
        self.bump("/tf_static")
        for transform in msg.transforms:
            self.tf_edges["/tf_static"].add((transform.header.frame_id, transform.child_frame_id))

    def topic_summary(self, topic: str, timeout_sec: float) -> dict[str, Any]:
        count = self.counts.get(topic, 0)
        span = max(0.001, self.last_seen.get(topic, time.time()) - self.first_seen.get(topic, time.time()))
        return {
            "publisher_count": len(self.get_publishers_info_by_topic(topic)),
            "subscription_count": len(self.get_subscriptions_info_by_topic(topic)),
            "message_count_observed": count,
            "message_received": count > 0,
            "observed_rate_hz": round((count - 1) / span, 3) if count > 1 else 0.0,
            "probe_window_sec": timeout_sec,
        }

    def tf_ready(self, parent: str, child: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.25))
            return True
        except TransformException:
            return False


def run_probe(
    timeout_sec: float,
    stage_output: Path,
    require_compute_path_to_pose: bool = False,
    require_navigate_to_pose: bool = False,
) -> dict[str, Any]:
    if IMPORT_ERROR:
        return {
            "artifact_type": "step30s2_bringup_dataplane_report",
            "created_utc": now_iso(),
            "rclpy_import_error": IMPORT_ERROR,
            "readiness": {"staticloc_dataplane_ready": False},
            "missing_conditions": ["rclpy_import_error"],
        }
    rclpy.init(args=None)
    node = DataplaneProbe()
    try:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        topics = {topic: node.topic_summary(topic, timeout_sec) for topic in ["/clock", "/odom", "/scan", "/tf", "/tf_static", "/map", "/cmd_vel"]}
        tf_frames = {
            "map_to_odom": node.tf_ready("map", "odom"),
            "odom_to_base_footprint": node.tf_ready("odom", "base_footprint"),
            "odom_to_base_link": node.tf_ready("odom", "base_link"),
            "base_footprint_to_base_link": node.tf_ready("base_footprint", "base_link"),
            "base_link_to_base_scan": node.tf_ready("base_link", "base_scan"),
            "map_to_base_footprint": node.tf_ready("map", "base_footprint"),
            "map_to_base_link": node.tf_ready("map", "base_link"),
        }
        action_servers = {
            "/compute_path_to_pose": node.compute_client.wait_for_server(timeout_sec=1.0),
            "/navigate_to_pose": node.navigate_client.wait_for_server(timeout_sec=1.0),
            "/follow_path": node.follow_path_client.wait_for_server(timeout_sec=1.0),
        }
        readiness = {
            "clock_publishing": topics["/clock"]["message_received"],
            "odom_nonzero_rate": topics["/odom"]["observed_rate_hz"] > 0.0,
            "scan_nonzero_rate": topics["/scan"]["observed_rate_hz"] > 0.0,
            "tf_has_frames": topics["/tf"]["message_received"],
            "tf_static_has_frames": topics["/tf_static"]["message_received"],
            "map_received": topics["/map"]["message_received"],
            "cmd_vel_exists_with_subscribers": topics["/cmd_vel"]["subscription_count"] > 0,
            "tf_map_to_odom_exists": tf_frames["map_to_odom"],
            "tf_odom_to_robot_exists": tf_frames["odom_to_base_footprint"] or tf_frames["odom_to_base_link"],
            "tf_base_footprint_to_base_link_exists_or_recoverable": tf_frames["base_footprint_to_base_link"] or tf_frames["odom_to_base_link"],
            "tf_base_link_to_base_scan_exists": tf_frames["base_link_to_base_scan"],
            "navigate_to_pose_action_server_ready": action_servers["/navigate_to_pose"],
            "follow_path_action_server_ready": action_servers["/follow_path"],
            "compute_path_to_pose_action_server_ready": action_servers["/compute_path_to_pose"],
        }
        required = [
            "clock_publishing",
            "odom_nonzero_rate",
            "scan_nonzero_rate",
            "tf_has_frames",
            "tf_static_has_frames",
            "map_received",
            "cmd_vel_exists_with_subscribers",
            "tf_map_to_odom_exists",
            "tf_odom_to_robot_exists",
            "tf_base_footprint_to_base_link_exists_or_recoverable",
            "tf_base_link_to_base_scan_exists",
            "follow_path_action_server_ready",
        ]
        if require_compute_path_to_pose:
            required.append("compute_path_to_pose_action_server_ready")
        if require_navigate_to_pose:
            required.append("navigate_to_pose_action_server_ready")
        readiness["staticloc_dataplane_ready"] = all(bool(readiness[key]) for key in required)
        missing = [key for key in required if not readiness[key]]
        return {
            "artifact_type": "step30s2_bringup_dataplane_report",
            "version": "v0_1",
            "created_utc": now_iso(),
            "stage_output_dir": stage_output.as_posix(),
            "localization_mode": "static_map_to_odom",
            "amcl_used": False,
            "topics": topics,
            "tf_frames": tf_frames,
            "tf_edges_observed": {key: sorted([f"{a}->{b}" for a, b in value]) for key, value in node.tf_edges.items()},
            "action_servers": action_servers,
            "action_server_roles": {
                "/follow_path": "hard_blocker_for_current_route_execution",
                "/compute_path_to_pose": "non_blocking_diagnostic_unless_planning_gate_requested",
                "/navigate_to_pose": "non_blocking_diagnostic_for_follow_path_runtime",
            },
            "required_conditions": required,
            "require_compute_path_to_pose": require_compute_path_to_pose,
            "require_navigate_to_pose": require_navigate_to_pose,
            "node_names": sorted(set(node.get_node_names())),
            "samples": {"odom": node.latest_odom, "scan": node.latest_scan, "map": node.latest_map},
            "readiness": readiness,
            "missing_conditions": missing,
        }
    finally:
        try:
            node.compute_client.destroy()
            node.navigate_client.destroy()
            node.follow_path_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--require-compute-path-to-pose", action="store_true")
    parser.add_argument("--require-navigate-to-pose", action="store_true")
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    out_dir = stage_output / "post_restructure_validation"
    output_json = args.output_json or out_dir / "step30s2_bringup_dataplane_report_v0_1.json"
    output_md = args.output_md or out_dir / "step30s2_bringup_dataplane_report_v0_1.md"
    payload = run_probe(
        args.timeout_sec,
        stage_output,
        require_compute_path_to_pose=args.require_compute_path_to_pose,
        require_navigate_to_pose=args.require_navigate_to_pose,
    )
    write_json(output_json, payload)
    write_md(output_md, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("readiness", {}).get("staticloc_dataplane_ready") else 1


if __name__ == "__main__":
    sys.exit(main())
