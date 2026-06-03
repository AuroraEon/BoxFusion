#!/usr/bin/env python3
"""Publish non-empty scene/floor semantic overlay markers for RViz."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"

from scene_runtime_common import derive_paths, read_json


def rgba(r: float, g: float, b: float, a: float) -> Any:
    msg = ColorRGBA()
    msg.r, msg.g, msg.b, msg.a = r, g, b, a
    return msg


def pt(x: float, y: float, z: float) -> Any:
    msg = Point()
    msg.x, msg.y, msg.z = float(x), float(y), float(z)
    return msg


class OverlayPublisher(Node):  # pragma: no cover
    def __init__(self, args: argparse.Namespace, paths: dict[str, Any]) -> None:
        super().__init__("boxfusion_scene_rviz_overlay_publisher")
        self.args = args
        self.paths = paths
        self.topic = args.overlay_topic or paths.get("overlay_topic")
        self.publisher = self.create_publisher(MarkerArray, self.topic, 1)
        self.markers = self.build_markers()
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()
        namespaces = sorted({m.ns for m in self.markers.markers})
        self.get_logger().info(json.dumps({
            "topic": self.topic,
            "marker_count": len(self.markers.markers),
            "namespaces": namespaces,
        }))

    def marker(self, marker_id: int, marker_type: int, klass: str) -> Any:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        scene_short = str(self.args.scene_id).split("-")[0]
        marker.ns = f"scene_{scene_short}/{self.args.floor_id}/{klass}"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def build_markers(self) -> Any:
        topology = read_json(Path(self.paths["topology_json"]))
        route_query = read_json(Path(self.args.route_query_json))
        waypoints = read_json(Path(self.args.waypoints_json)).get("waypoints", [])
        rooms = [room for room in topology.get("rooms", []) if room.get("floor_id") == self.args.floor_id]
        marker_array = MarkerArray()
        marker_id = 1

        floorplan = self.marker(marker_id, Marker.LINE_LIST, "floorplan_rooms")
        floorplan.scale.x = 0.035
        floorplan.color = rgba(0.42, 0.72, 1.0, 0.95)
        for room in rooms:
            poly = room.get("polygon") or []
            for a, b in zip(poly, poly[1:] + poly[:1]):
                floorplan.points.append(pt(a[0], a[1], 0.05))
                floorplan.points.append(pt(b[0], b[1], 0.05))
        marker_array.markers.append(floorplan)
        marker_id += 1

        route_room_set = set(route_query.get("selected_route") or route_query.get("requested_route") or [])
        room_centers = self.marker(marker_id, Marker.SPHERE_LIST, "room_centers")
        room_centers.scale.x = room_centers.scale.y = room_centers.scale.z = 0.18
        room_centers.color = rgba(0.95, 0.78, 0.18, 1.0)
        for room in rooms:
            if room["id"] in route_room_set:
                x, y = room["center"]
                room_centers.points.append(pt(x, y, 0.20))
        marker_array.markers.append(room_centers)
        marker_id += 1

        route = self.marker(marker_id, Marker.LINE_STRIP, "route_path")
        route.scale.x = 0.07
        route.color = rgba(0.05, 1.0, 0.95, 1.0)
        route.points = [pt(w["x"], w["y"], 0.28) for w in waypoints]
        marker_array.markers.append(route)
        marker_id += 1

        gateways = self.marker(marker_id, Marker.SPHERE_LIST, "gateway_crossings")
        gateways.scale.x = gateways.scale.y = gateways.scale.z = 0.26
        gateways.color = rgba(0.2, 1.0, 0.35, 1.0)
        gateways.points = [pt(w["x"], w["y"], 0.36) for w in waypoints if w.get("source") == "gateway" or w.get("gateway_id")]
        marker_array.markers.append(gateways)
        marker_id += 1

        labels = self.marker(marker_id, Marker.TEXT_VIEW_FACING, "route_summary")
        labels.pose.position = pt(waypoints[0]["x"] if waypoints else 0.0, waypoints[0]["y"] if waypoints else 0.0, 0.85)
        labels.scale.z = 0.22
        labels.color = rgba(1.0, 1.0, 1.0, 1.0)
        labels.text = f"{self.args.scene_id}/{self.args.floor_id}: " + " -> ".join(route_query.get("selected_route") or route_query.get("requested_route") or [])
        marker_array.markers.append(labels)
        return marker_array

    def publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--stage-a-output-dir", type=Path)
    parser.add_argument("--route-query-json", type=Path, required=True)
    parser.add_argument("--waypoints-json", type=Path, required=True)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--overlay-topic", required=True)
    parser.add_argument("--run-id", default="manual")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    if IMPORT_ERROR:
        print(json.dumps({"rclpy_import_error": IMPORT_ERROR}), file=sys.stderr)
        return 2
    paths = derive_paths(args)
    rclpy.init(args=None)
    node = OverlayPublisher(args, paths)
    try:
        if args.once:
            for _ in range(12):
                node.publish_markers()
                rclpy.spin_once(node, timeout_sec=0.1)
            return 0
        rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
