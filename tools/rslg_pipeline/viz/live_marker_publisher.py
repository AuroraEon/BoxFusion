#!/usr/bin/env python3
"""Bounded RSLG-SLAM live MarkerArray publisher for visualization smoke tests.

Its formal input is the RouteResult-derived RViz marker input
(schema ``rslg_route_result_rviz_marker_input``) produced by
``tools/rslg_pipeline/runtime/route_result_marker_adapter.py``. It may optionally
accept an ``rslg_route_result`` directly and convert it through that adapter. The
old task48 route-executor input shape is no longer a formal input here.

This publisher does not launch RViz and requires no ``map_server`` /
``nav2_map_server`` display. It only republishes marker records; it is a Layer 4
visualization adapter only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.runtime.route_result_adapter_common import is_route_result
from tools.rslg_pipeline.runtime.route_result_marker_adapter import (
    SCHEMA_NAME as MARKER_INPUT_SCHEMA,
    build_rviz_marker_input,
)


TYPE_MAP = {
    "ARROW": Marker.ARROW,
    "CUBE": Marker.CUBE,
    "SPHERE": Marker.SPHERE,
    "CYLINDER": Marker.CYLINDER,
    "LINE_STRIP": Marker.LINE_STRIP,
    "LINE_LIST": Marker.LINE_LIST,
    "CUBE_LIST": Marker.CUBE_LIST,
    "SPHERE_LIST": Marker.SPHERE_LIST,
    "TEXT_VIEW_FACING": Marker.TEXT_VIEW_FACING,
    "MESH_RESOURCE": Marker.MESH_RESOURCE,
    "TRIANGLE_LIST": Marker.TRIANGLE_LIST,
}

FLOOR_Z_DEFAULTS = {"floor_1": 0.0, "floor_2": 1.6, "floor_transition": 0.8}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def bool_text(value: bool) -> str:
    return "true" if value else "false"


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def point_from_dict(data: dict[str, Any]) -> Point:
    point = Point()
    point.x = float(data.get("x", 0.0))
    point.y = float(data.get("y", 0.0))
    point.z = float(data.get("z", 0.0))
    return point


def marker_from_dict(record: dict[str, Any], node: Node, default_frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = str(record.get("frame_id") or default_frame_id)
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = str(record.get("ns", "rslg_slam_live"))
    marker.id = int(record.get("id", 0))
    marker.type = TYPE_MAP.get(str(record.get("type", "SPHERE")), Marker.SPHERE)
    marker.action = Marker.ADD

    scale = record.get("scale") or {}
    marker.scale.x = float(scale.get("x", 0.1))
    marker.scale.y = float(scale.get("y", marker.scale.x))
    marker.scale.z = float(scale.get("z", marker.scale.x))

    color = record.get("color_rgba") or [1.0, 1.0, 1.0, 1.0]
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])

    for point in record.get("points") or []:
        marker.points.append(point_from_dict(point))

    pose = (record.get("pose") or {}).get("position")
    if pose:
        marker.pose.position = point_from_dict(pose)
    elif marker.points:
        marker.pose.orientation.w = 1.0
    else:
        marker.pose.orientation.w = 1.0

    orientation = (record.get("pose") or {}).get("orientation") or {}
    if orientation:
        marker.pose.orientation.x = float(orientation.get("x", 0.0))
        marker.pose.orientation.y = float(orientation.get("y", 0.0))
        marker.pose.orientation.z = float(orientation.get("z", 0.0))
        marker.pose.orientation.w = float(orientation.get("w", 1.0))
    elif record.get("yaw") is not None:
        qx, qy, qz, qw = quaternion_from_yaw(float(record["yaw"]))
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
    elif marker.pose.orientation.w == 0.0:
        marker.pose.orientation.w = 1.0

    marker.text = str(record.get("text") or "")
    marker.lifetime = Duration(seconds=0).to_msg()
    return marker


def marker_blob(markers: list[dict[str, Any]]) -> list[str]:
    return [json.dumps(marker, sort_keys=True) for marker in markers]


def semantic_summary(payload: dict[str, Any], markers: list[dict[str, Any]]) -> dict[str, Any]:
    blobs = marker_blob(markers)
    z_levels = payload.get("floor_z_map") or payload.get("floor_z_levels_m") or FLOOR_Z_DEFAULTS
    non_transition_edge_id = "vt_1_centerline_e003"
    ring_037_goal = any(
        "generated_ring_037" in blob
        and (
            '"used_as_runtime_goal": true' in blob
            or '"used_as_runtime_goal":true' in blob
            or '"runtime_goal_used": true' in blob
            or '"object_runtime_goal": true' in blob
        )
        for blob in blobs
    )
    e003_as_transition = any(
        (marker.get("metadata") or {}).get("transition_edge") == non_transition_edge_id for marker in markers
    )
    return {
        "schema_name": "rslg_live_marker_publisher_semantic_summary",
        "schema_version": "0.1",
        "generated_utc": utc_now(),
        "source_route_result": payload.get("source_route_result"),
        "topic": payload.get("topic"),
        "frame_id": payload.get("frame_id"),
        "marker_count": len(markers),
        "namespaces": [marker.get("ns") for marker in markers],
        "rviz_map_display_required": bool(payload.get("rviz_map_display_required", False)),
        "generated_ring_002_present": any("generated_ring_002" in blob for blob in blobs),
        "generated_ring_002_selected_approach_marker": any(
            "generated_ring_002" in blob and "selected_object_approach_goal" in blob for blob in blobs
        ),
        "generated_ring_037_present": any("generated_ring_037" in blob for blob in blobs),
        "generated_ring_037_runtime_goal_used": ring_037_goal,
        "generated_ring_037_blocked_only": any("generated_ring_037" in blob for blob in blobs) and not ring_037_goal,
        "vt_1_centerline_e001_present": any("vt_1_centerline_e001" in blob for blob in blobs),
        "vt_1_centerline_e001_connector_handoff_marker": any(
            "vt_1_centerline_e001" in blob and ("connector_handoff" in blob or "handoff" in blob) for blob in blobs
        ),
        "vt_1_centerline_e003_transition_used": e003_as_transition,
        "floor_z_map": z_levels,
        "floor_1_z_0p0_represented": float(z_levels.get("floor_1", -1.0)) == 0.0,
        "floor_2_z_1p6_represented": float(z_levels.get("floor_2", -1.0)) == 1.6,
    }


class LiveMarkerPublisher(Node):
    def __init__(self, payload: dict[str, Any], markers: list[dict[str, Any]], topic: str, frame_id: str, rate_hz: float) -> None:
        super().__init__("rslg_slam_live_marker_publisher")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, topic, qos)
        self.payload = payload
        self.markers = markers
        self.topic = topic
        self.frame_id = frame_id
        self.publish_count = 0
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.publish_once)

    def publish_once(self) -> None:
        array = MarkerArray()
        for index, record in enumerate(self.markers):
            if "id" not in record:
                record = dict(record)
                record["id"] = index
            array.markers.append(marker_from_dict(record, self, self.frame_id))
        self.publisher.publish(array)
        self.publish_count += 1


def load_payload(args: argparse.Namespace) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Load a RouteResult-derived RViz marker input payload and its marker list."""

    if args.marker_input_json:
        payload = read_json(args.marker_input_json)
        if payload.get("schema_name") != MARKER_INPUT_SCHEMA:
            raise SystemExit(
                f"--marker-input-json must be a {MARKER_INPUT_SCHEMA} payload, "
                f"got {payload.get('schema_name')!r}"
            )
        markers = list(payload.get("markers") or [])
    elif args.route_result_json:
        route_result = read_json(args.route_result_json)
        if not is_route_result(route_result):
            raise SystemExit("--route-result-json must be an rslg_route_result payload")
        payload = build_rviz_marker_input(
            route_result,
            source_ref=args.route_result_json.as_posix(),
            frame_id=args.frame_id,
            topic=args.topic,
        )
        markers = list(payload.get("markers") or [])
    else:
        raise SystemExit("Provide --marker-input-json or --route-result-json.")

    payload.setdefault("frame_id", args.frame_id)
    payload["topic"] = args.topic or payload.get("topic") or "/rslg_slam/live_route_markers"
    return payload, markers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--marker-input-json", type=Path, default=None)
    source.add_argument("--route-result-json", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--topic", default="/rslg_slam/live_route_markers")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--publish-rate-hz", type=float, default=2.0)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args()

    if args.no_canonical_write and "canonical" in args.output_dir.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    payload, markers = load_payload(args)
    topic = args.topic or payload.get("topic") or "/rslg_slam/live_route_markers"
    summary = semantic_summary(payload, markers)
    summary.update(
        {
            "publisher_status": "starting",
            "bounded_duration_sec": args.duration_sec,
            "publish_rate_hz": args.publish_rate_hz,
            "marker_input_json": str(args.marker_input_json) if args.marker_input_json else None,
            "route_result_json": str(args.route_result_json) if args.route_result_json else None,
            "no_canonical_write": bool(args.no_canonical_write),
        }
    )
    write_json(args.output_dir / "live_marker_publisher_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=False), flush=True)

    rclpy.init(args=None)
    node = LiveMarkerPublisher(payload, markers, topic, args.frame_id, args.publish_rate_hz)
    deadline = time.monotonic() + max(0.5, args.duration_sec)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        publish_count = node.publish_count
        node.destroy_node()
        rclpy.shutdown()

    summary["publisher_status"] = "completed"
    summary["publish_count"] = publish_count
    summary["elapsed_sec"] = round(time.monotonic() - started, 3)
    summary["topic"] = topic
    summary["generated_ring_037_runtime_goal_used"] = bool(summary["generated_ring_037_runtime_goal_used"])
    summary["generated_ring_037_runtime_goal_used_text"] = bool_text(summary["generated_ring_037_runtime_goal_used"])
    write_json(args.output_dir / "live_marker_publisher_summary.json", summary)
    print(json.dumps({"publisher_status": "completed", "topic": topic, "publish_count": publish_count}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
