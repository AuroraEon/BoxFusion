#!/usr/bin/env python3
"""Publish task42b 3D RViz MarkerArray replay for RSLG-SLAM."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
from pathlib import Path
from typing import Any


DEFAULT_ROOT = Path("/home/ws/workspace/BoxFusion")
DEFAULT_MARKERS_JSON = DEFAULT_ROOT / (
    "stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/"
    "rviz_3d_dynamic/task42b_3d_static_markers_v0_1.json"
)
DEFAULT_FRAMES_JSON = DEFAULT_ROOT / (
    "stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/"
    "rviz_3d_dynamic/task42b_dynamic_replay_frames_v0_1.json"
)
MARKER_TOPIC = "/task42b_3d_demo_markers"
POSE_TOPIC = "/task42b_replay_pose"
PATH_TOPIC = "/task42b_replay_path"


def import_ros() -> dict[str, Any]:
    try:
        import rclpy
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point, PoseStamped, TransformStamped
        from nav_msgs.msg import Path as PathMsg
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Header
        from visualization_msgs.msg import Marker, MarkerArray
    except Exception as exc:  # pragma: no cover - depends on ROS shell
        print(
            "Failed to import ROS2 Python packages. Run with /usr/bin/python3 in a ROS2 Foxy environment "
            "after `source /opt/ros/foxy/setup.bash`.",
            file=sys.stderr,
        )
        print(f"Import error: {type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(3) from exc
    try:
        from tf2_ros import TransformBroadcaster
    except Exception:
        TransformBroadcaster = None
    return locals()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def marker_type_value(marker_cls: Any, marker_type: str) -> int:
    mapping = {
        "ARROW": marker_cls.ARROW,
        "CUBE": marker_cls.CUBE,
        "SPHERE": marker_cls.SPHERE,
        "CYLINDER": marker_cls.CYLINDER,
        "LINE_STRIP": marker_cls.LINE_STRIP,
        "LINE_LIST": marker_cls.LINE_LIST,
        "TEXT_VIEW_FACING": marker_cls.TEXT_VIEW_FACING,
    }
    return mapping.get(marker_type, marker_cls.SPHERE)


def make_point(point_cls: Any, payload: dict[str, Any]) -> Any:
    point = point_cls()
    point.x = float(payload.get("x", 0.0))
    point.y = float(payload.get("y", 0.0))
    point.z = float(payload.get("z", 0.0))
    return point


def apply_pose(marker: Any, payload: dict[str, Any]) -> None:
    position = (payload.get("pose") or {}).get("position")
    if payload.get("points") and payload.get("type") in {"ARROW", "LINE_STRIP", "LINE_LIST"}:
        return
    if position:
        marker.pose.position.x = float(position.get("x", 0.0))
        marker.pose.position.y = float(position.get("y", 0.0))
        marker.pose.position.z = float(position.get("z", 0.0))
    yaw = float(payload.get("yaw") or 0.0)
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw


def build_marker(ros: dict[str, Any], payload: dict[str, Any], stamp: Any, marker_id_offset: int = 0) -> Any:
    Marker = ros["Marker"]
    Point = ros["Point"]
    Duration = ros["Duration"]
    marker = Marker()
    marker.header.frame_id = payload.get("frame_id", "map")
    marker.header.stamp = stamp
    marker.ns = payload.get("ns", "task42b")
    marker.id = int(payload.get("id", 0)) + marker_id_offset
    marker.type = marker_type_value(Marker, payload.get("type", "SPHERE"))
    marker.action = Marker.ADD
    marker.lifetime = Duration(sec=0, nanosec=0)
    color = payload.get("color_rgba", [1.0, 1.0, 1.0, 1.0])
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])
    scale = payload.get("scale") or {}
    marker.scale.x = float(scale.get("x", 0.1))
    marker.scale.y = float(scale.get("y", scale.get("x", 0.1)))
    marker.scale.z = float(scale.get("z", scale.get("x", 0.1)))
    marker.text = payload.get("text") or ""
    marker.points = [make_point(Point, point) for point in payload.get("points", [])]
    apply_pose(marker, payload)
    return marker


def dynamic_markers(ros: dict[str, Any], frame: dict[str, Any], tail: list[dict[str, Any]], stamp: Any) -> list[Any]:
    x = float(frame["x"])
    y = float(frame["y"])
    z = float(frame["z"])
    yaw = float(frame.get("yaw", 0.0))
    tip = {"x": x + 0.55 * math.cos(yaw), "y": y + 0.55 * math.sin(yaw), "z": z + 0.12}
    base = {"x": x, "y": y, "z": z + 0.12}
    pose_arrow = {
        "ns": "task42b_current_pose",
        "id": 9001,
        "type": "ARROW",
        "frame_id": "map",
        "points": [base, tip],
        "scale": {"x": 0.08, "y": 0.18, "z": 0.18},
        "color_rgba": [1.0, 1.0, 0.12, 1.0],
    }
    body = {
        "ns": "task42b_current_robot_body",
        "id": 9002,
        "type": "SPHERE",
        "frame_id": "map",
        "pose": {"position": {"x": x, "y": y, "z": z + 0.09}},
        "scale": {"x": 0.24, "y": 0.24, "z": 0.16},
        "color_rgba": [1.0, 0.95, 0.12, 1.0],
    }
    label = {
        "ns": "task42b_current_stage_label",
        "id": 9003,
        "type": "TEXT_VIEW_FACING",
        "frame_id": "map",
        "pose": {"position": {"x": x, "y": y, "z": z + 0.48}},
        "scale": {"x": 0.0, "y": 0.0, "z": 0.22},
        "color_rgba": [1.0, 1.0, 1.0, 1.0],
        "text": str(frame.get("stage_name", "replay")),
    }
    tail_marker = {
        "ns": "task42b_dynamic_tail",
        "id": 9004,
        "type": "LINE_STRIP",
        "frame_id": "map",
        "points": [{"x": t["x"], "y": t["y"], "z": t["z"] + 0.08} for t in tail],
        "scale": {"x": 0.045, "y": 0.045, "z": 0.045},
        "color_rgba": [1.0, 0.95, 0.12, 0.78],
    }
    return [build_marker(ros, payload, stamp) for payload in (pose_arrow, body, label, tail_marker)]


def make_pose_stamped(ros: dict[str, Any], frame: dict[str, Any], stamp: Any) -> Any:
    PoseStamped = ros["PoseStamped"]
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp
    msg.pose.position.x = float(frame["x"])
    msg.pose.position.y = float(frame["y"])
    msg.pose.position.z = float(frame["z"])
    qx, qy, qz, qw = quaternion_from_yaw(float(frame.get("yaw", 0.0)))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def make_transform(ros: dict[str, Any], frame: dict[str, Any], stamp: Any) -> Any:
    TransformStamped = ros["TransformStamped"]
    msg = TransformStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp
    msg.child_frame_id = "task42b_replay_base"
    msg.transform.translation.x = float(frame["x"])
    msg.transform.translation.y = float(frame["y"])
    msg.transform.translation.z = float(frame["z"])
    qx, qy, qz, qw = quaternion_from_yaw(float(frame.get("yaw", 0.0)))
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=12.0, help="Publish timer rate in Hz.")
    parser.add_argument("--loop", action="store_true", help="Loop the replay after the last frame.")
    parser.add_argument("--static-only", action="store_true", help="Publish only static markers.")
    parser.add_argument("--speed", type=float, default=1.0, help="Frames advanced per timer tick multiplier.")
    parser.add_argument("--frames-json", type=Path, default=DEFAULT_FRAMES_JSON)
    parser.add_argument("--markers-json", type=Path, default=DEFAULT_MARKERS_JSON)
    parser.add_argument("--tail-length", type=int, default=28)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ros = import_ros()
    rclpy = ros["rclpy"]
    Node = ros["Node"]
    QoSProfile = ros["QoSProfile"]
    DurabilityPolicy = ros["DurabilityPolicy"]
    ReliabilityPolicy = ros["ReliabilityPolicy"]
    MarkerArray = ros["MarkerArray"]
    PathMsg = ros["PathMsg"]
    TransformBroadcaster = ros["TransformBroadcaster"]

    marker_payload = load_json(args.markers_json)
    frames = load_json(args.frames_json).get("frames", [])
    static_payloads = marker_payload.get("markers", [])

    rclpy.init()
    node = Node("task42b_3d_replay_publisher")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    marker_pub = node.create_publisher(MarkerArray, MARKER_TOPIC, qos)
    pose_pub = node.create_publisher(ros["PoseStamped"], POSE_TOPIC, 10)
    path_pub = node.create_publisher(PathMsg, PATH_TOPIC, 10)
    tf_pub = TransformBroadcaster(node) if TransformBroadcaster is not None else None

    state = {"index_float": 0.0, "tail": [], "path": PathMsg(), "running": True}
    state["path"].header.frame_id = "map"

    def publish_once() -> None:
        stamp = node.get_clock().now().to_msg()
        array = MarkerArray()
        array.markers = [build_marker(ros, payload, stamp) for payload in static_payloads]
        if frames and not args.static_only:
            index = min(int(state["index_float"]), len(frames) - 1)
            frame = frames[index]
            state["tail"].append(frame)
            state["tail"] = state["tail"][-max(1, args.tail_length):]
            array.markers.extend(dynamic_markers(ros, frame, state["tail"], stamp))
            pose = make_pose_stamped(ros, frame, stamp)
            pose_pub.publish(pose)
            state["path"].header.stamp = stamp
            state["path"].poses.append(pose)
            state["path"].poses = state["path"].poses[-500:]
            path_pub.publish(state["path"])
            if tf_pub is not None:
                tf_pub.sendTransform(make_transform(ros, frame, stamp))
            state["index_float"] += max(0.05, args.speed)
            if state["index_float"] >= len(frames):
                if args.loop:
                    state["index_float"] = 0.0
                    state["tail"] = []
                    state["path"].poses = []
                else:
                    state["index_float"] = len(frames) - 1
        marker_pub.publish(array)

    def handle_signal(signum: int, _frame: Any) -> None:
        state["running"] = False
        node.get_logger().info(f"received signal {signum}, shutting down task42b replay publisher")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    timer = node.create_timer(1.0 / max(args.rate, 0.1), publish_once)
    publish_once()
    try:
        while rclpy.ok() and state["running"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
