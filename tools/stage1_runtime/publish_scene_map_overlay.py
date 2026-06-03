#!/usr/bin/env python3
"""Publish an RViz MarkerArray occupancy-map overlay without using the Map display."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
    Node = object  # type: ignore[assignment]

from scene_runtime_common import load_nav_map, now_iso, write_json


def rgba(r: float, g: float, b: float, a: float) -> Any:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def point(x: float, y: float, z: float = 0.0) -> Any:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def marker_qos() -> Any:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def occupied_points_from_map(map_yaml: Path, downsample: int, occupied_value_max: int) -> tuple[list[tuple[float, float]], dict[str, Any]]:
    grid, resolution, origin, meta = load_nav_map(map_yaml)
    occupied = np.argwhere(grid <= int(occupied_value_max))
    downsample = max(int(downsample), 1)
    if downsample > 1 and len(occupied):
        occupied = occupied[::downsample]
    points = [
        (
            float(origin[0] + (col + 0.5) * resolution),
            float(origin[1] + (row + 0.5) * resolution),
        )
        for row, col in occupied
    ]
    rows = occupied[:, 0] if len(occupied) else np.array([], dtype=int)
    cols = occupied[:, 1] if len(occupied) else np.array([], dtype=int)
    bounds = None
    if len(occupied):
        bounds = {
            "min_x": float(origin[0] + (int(cols.min()) + 0.5) * resolution),
            "max_x": float(origin[0] + (int(cols.max()) + 0.5) * resolution),
            "min_y": float(origin[1] + (int(rows.min()) + 0.5) * resolution),
            "max_y": float(origin[1] + (int(rows.max()) + 0.5) * resolution),
        }
    summary = {
        "source_map_yaml": map_yaml.as_posix(),
        "map_resolution_m": float(resolution),
        "map_origin_xy": [float(origin[0]), float(origin[1])],
        "map_width_cells": int(grid.shape[1]),
        "map_height_cells": int(grid.shape[0]),
        "occupied_value_max": int(occupied_value_max),
        "map_overlay_downsample": downsample,
        "map_overlay_contour_count": None,
        "map_overlay_point_count": len(points),
        "occupied_bounds_xy": bounds,
        "map_meta": meta,
    }
    return points, summary


def build_marker_array(args: argparse.Namespace, occupied_points: list[tuple[float, float]], summary: dict[str, Any]) -> Any:
    marker_array = MarkerArray()
    marker_id = 1
    ns_prefix = args.namespace.rstrip("/")

    if args.publish_background:
        bounds = summary.get("occupied_bounds_xy") or {
            "min_x": -10.0,
            "max_x": 10.0,
            "min_y": -10.0,
            "max_y": 10.0,
        }
        pad = float(args.background_padding_m)
        background = Marker()
        background.header.frame_id = "map"
        background.ns = f"{ns_prefix}/free_space_background"
        background.id = marker_id
        background.type = Marker.CUBE
        background.action = Marker.ADD
        background.pose.position.x = (float(bounds["min_x"]) + float(bounds["max_x"])) / 2.0
        background.pose.position.y = (float(bounds["min_y"]) + float(bounds["max_y"])) / 2.0
        background.pose.position.z = -0.015
        background.pose.orientation.w = 1.0
        background.scale.x = max(0.1, float(bounds["max_x"]) - float(bounds["min_x"]) + 2.0 * pad)
        background.scale.y = max(0.1, float(bounds["max_y"]) - float(bounds["min_y"]) + 2.0 * pad)
        background.scale.z = 0.01
        background.color = rgba(0.58, 0.60, 0.62, float(args.background_alpha))
        marker_array.markers.append(background)
        marker_id += 1

    walls = Marker()
    walls.header.frame_id = "map"
    walls.ns = f"{ns_prefix}/occupied_walls"
    walls.id = marker_id
    walls.type = Marker.CUBE_LIST
    walls.action = Marker.ADD
    walls.pose.orientation.w = 1.0
    cell_size = float(summary["map_resolution_m"]) * max(int(summary["map_overlay_downsample"]), 1)
    walls.scale.x = cell_size
    walls.scale.y = cell_size
    walls.scale.z = float(args.wall_height_m)
    walls.color = rgba(0.02, 0.025, 0.03, float(args.wall_alpha))
    walls.points = [point(x, y, float(args.wall_height_m) / 2.0) for x, y in occupied_points]
    marker_array.markers.append(walls)
    return marker_array


class MapOverlayPublisher(Node):  # pragma: no cover - ROS runtime only
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("boxfusion_scene_map_marker_overlay")
        self.args = args
        self.occupied_points, self.summary = occupied_points_from_map(args.map_yaml, args.downsample, args.occupied_value_max)
        self.marker_array = build_marker_array(args, self.occupied_points, self.summary)
        self.publisher = self.create_publisher(MarkerArray, args.marker_topic, marker_qos())
        self.timer = self.create_timer(1.0 / max(float(args.rate_hz), 0.1), self.publish_all)
        self.publish_all()
        self.get_logger().info(json.dumps(self.validation_summary(), sort_keys=True))

    def validation_summary(self) -> dict[str, Any]:
        namespaces = sorted({marker.ns for marker in self.marker_array.markers})
        payload = dict(self.summary)
        payload.update({
            "created_utc": now_iso(),
            "marker_topic": self.args.marker_topic,
            "marker_count": len(self.marker_array.markers),
            "marker_namespaces": namespaces,
            "marker_array_non_empty": bool(self.marker_array.markers),
            "passed": bool(self.marker_array.markers and self.summary["map_overlay_point_count"] > 0),
        })
        return payload

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.marker_array)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--marker-topic", required=True)
    parser.add_argument("--namespace", default="map_overlay")
    parser.add_argument("--downsample", type=int, default=1)
    parser.add_argument("--occupied-value-max", type=int, default=100)
    parser.add_argument("--wall-height-m", type=float, default=0.08)
    parser.add_argument("--wall-alpha", type=float, default=0.95)
    parser.add_argument("--publish-background", action="store_true", default=True)
    parser.add_argument("--background-alpha", type=float, default=0.22)
    parser.add_argument("--background-padding-m", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--publish-once", action="store_true")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--metadata-only", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    points, summary = occupied_points_from_map(args.map_yaml, args.downsample, args.occupied_value_max)
    if args.metadata_only:
        summary.update({
            "created_utc": now_iso(),
            "marker_topic": args.marker_topic,
            "marker_count": 2 if args.publish_background else 1,
            "marker_namespaces": [f"{args.namespace.rstrip('/')}/free_space_background", f"{args.namespace.rstrip('/')}/occupied_walls"] if args.publish_background else [f"{args.namespace.rstrip('/')}/occupied_walls"],
            "marker_array_non_empty": True,
            "passed": bool(points),
        })
        if args.output_json:
            write_json(args.output_json, summary)
        print(json.dumps(summary, sort_keys=True))
        return 0 if points else 1
    if IMPORT_ERROR:
        payload = {"passed": False, "rclpy_import_error": IMPORT_ERROR, **summary}
        if args.output_json:
            write_json(args.output_json, payload)
        print(json.dumps(payload, sort_keys=True), file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = MapOverlayPublisher(args)
    start = time.monotonic()
    try:
        if args.publish_once:
            for _ in range(12):
                node.publish_all()
                rclpy.spin_once(node, timeout_sec=0.1)
            payload = node.validation_summary()
            if args.output_json:
                write_json(args.output_json, payload)
            print(json.dumps(payload, sort_keys=True))
            return 0 if payload["passed"] else 1
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration_sec > 0 and time.monotonic() - start >= args.duration_sec:
                break
        payload = node.validation_summary()
        if args.output_json:
            write_json(args.output_json, payload)
        print(json.dumps(payload, sort_keys=True))
        return 0 if payload["passed"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
