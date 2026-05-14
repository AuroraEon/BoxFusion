#!/usr/bin/env python3
"""Publish Stage1 BEV semantic mask, route, gateway, trajectory, and validation markers."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

os.environ.setdefault("ROS_DOMAIN_ID", "84")
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


TOPIC = "/stage1_nav/semantic_overlay_markers"
COMPAT_TOPIC = "/step30s7_rviz_overlay_markers"
ROOM_COLORS = {
    "room_1": (0.22, 0.58, 1.00, 0.22),
    "room_3": (0.10, 0.78, 0.62, 0.22),
    "room_7": (0.95, 0.70, 0.18, 0.22),
    "room_8": (0.90, 0.36, 0.28, 0.24),
    "room_11": (0.55, 0.72, 1.00, 0.22),
    "room_14": (0.52, 0.86, 0.36, 0.22),
    "room_15": (1.00, 0.20, 0.82, 0.26),
    "room_16": (0.78, 0.50, 1.00, 0.24),
}


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def color(r: float, g: float, b: float, a: float = 1.0) -> Any:
    msg = ColorRGBA()
    msg.r, msg.g, msg.b, msg.a = r, g, b, a
    return msg


def point(x: float, y: float, z: float = 0.05) -> Any:
    msg = Point()
    msg.x, msg.y, msg.z = float(x), float(y), float(z)
    return msg


def room_center(room: dict[str, Any]) -> tuple[float, float]:
    center = room.get("center") or [0.0, 0.0]
    return float(center[0]), float(center[1])


class OverlayPublisher(Node):  # pragma: no cover - ROS runtime only
    def __init__(
        self,
        stage_output: Path,
        route_query: Path | None,
        waypoints: Path | None,
        trajectory: Path | None,
        wall_validation: Path | None,
        topic: str,
        run_id: str,
        room_stride: int,
        max_trajectory_segments: int,
        clear_only: bool,
    ) -> None:
        super().__init__("boxfusion_stage1_nav_overlay_publisher")
        self.publisher = self.create_publisher(MarkerArray, topic, 1)
        self.compat_publisher = self.create_publisher(MarkerArray, COMPAT_TOPIC, 1) if topic != COMPAT_TOPIC else None
        self.stage_output = stage_output
        self.route_query_path = route_query
        self.waypoints_path = waypoints
        self.trajectory_path = trajectory
        self.wall_validation_path = wall_validation
        self.run_id = run_id
        self.room_stride = max(1, int(room_stride))
        self.max_trajectory_segments = max(10, int(max_trajectory_segments))
        self.clear_only = clear_only
        self.cached_markers = self.build_clear_markers() if clear_only else self.build_markers()
        self.timer = self.create_timer(5.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info(
            f"Stage1 overlay run_id={run_id} marker_count={len(self.cached_markers.markers)} "
            f"topic={topic} room_stride={self.room_stride}"
        )

    def base_marker(self, marker_id: int, marker_type: int, ns: str) -> Any:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.run_id}/{ns}" if ns != "clear" else ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def build_clear_markers(self) -> Any:
        markers = MarkerArray()
        clear = self.base_marker(0, Marker.CUBE, "clear")
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        return markers

    def build_markers(self) -> Any:
        route_query = read_json(self.route_query_path) if self.route_query_path else {}
        waypoint_payload = read_json(self.waypoints_path) if self.waypoints_path else {}
        waypoints = waypoint_payload.get("waypoints") or []
        rooms_payload = read_json(self.stage_output / "stage1_committed_public/topology_v0_1.json")
        rooms = {room["id"]: room for room in rooms_payload.get("rooms", [])}
        layered_meta = read_json(self.stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.json")
        resolution = float(layered_meta["resolution"])
        origin = (float(layered_meta["origin"][0]), float(layered_meta["origin"][1]))
        room_mask = None
        try:
            import numpy as np
            room_mask = np.load(self.stage_output / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy")
        except Exception:
            room_mask = None

        markers = self.build_clear_markers()
        marker_id = 1

        if room_mask is not None:
            for room_id, room in sorted(rooms.items(), key=lambda item: int(item[0].split("_")[1])):
                room_num = int(room_id.split("_")[1])
                rgba = ROOM_COLORS.get(room_id, (0.45, 0.65, 0.95, 0.18))
                fill = self.base_marker(marker_id, Marker.CUBE_LIST, f"stable_semantic_{room_id}")
                fill.scale.x = fill.scale.y = max(0.18, resolution * self.room_stride)
                fill.scale.z = 0.014
                fill.color = color(*rgba)
                rows, cols = (room_mask == room_num).nonzero()
                for row, col in zip(rows[:: self.room_stride], cols[:: self.room_stride]):
                    fill.points.append(point(origin[0] + int(col) * resolution, origin[1] + int(row) * resolution, 0.012))
                markers.markers.append(fill)
                marker_id += 1

                outline = self.base_marker(marker_id, Marker.LINE_LIST, f"stable_outline_{room_id}")
                outline.scale.x = 0.025
                outline.color = color(rgba[0], rgba[1], rgba[2], 0.85)
                poly = room.get("polygon") or []
                for a, b in zip(poly, poly[1:] + poly[:1]):
                    outline.points.append(point(a[0], a[1], 0.07))
                    outline.points.append(point(b[0], b[1], 0.07))
                markers.markers.append(outline)
                marker_id += 1

            through_rooms = route_query.get("through_rooms") or []
            for through_room in through_rooms:
                if through_room not in rooms:
                    continue
                rgba = ROOM_COLORS.get(through_room, (1.0, 0.2, 0.8, 1.0))
                highlight = self.base_marker(marker_id, Marker.LINE_LIST, f"request_highlight_{through_room}")
                highlight.scale.x = 0.065
                highlight.color = color(rgba[0], rgba[1], rgba[2], 1.0)
                poly = rooms[through_room].get("polygon") or []
                for a, b in zip(poly, poly[1:] + poly[:1]):
                    highlight.points.append(point(a[0], a[1], 0.16))
                    highlight.points.append(point(b[0], b[1], 0.16))
                markers.markers.append(highlight)
                marker_id += 1

        route_line = self.base_marker(marker_id, Marker.LINE_STRIP, "planned_topology_route")
        route_line.scale.x = 0.055
        route_line.color = color(0.0, 0.75, 1.0, 1.0)
        route_line.points = [point(w["x"], w["y"], 0.18) for w in waypoints]
        markers.markers.append(route_line)
        marker_id += 1

        gateway_points = self.base_marker(marker_id, Marker.SPHERE_LIST, "gateway_crossings")
        gateway_points.scale.x = gateway_points.scale.y = gateway_points.scale.z = 0.16
        gateway_points.color = color(0.2, 1.0, 0.35, 1.0)
        gateway_points.points = [point(w["x"], w["y"], 0.22) for w in waypoints if w.get("source") == "segment_crossing"]
        markers.markers.append(gateway_points)
        marker_id += 1

        interior_points = self.base_marker(marker_id, Marker.SPHERE_LIST, "interior_targets")
        interior_points.scale.x = interior_points.scale.y = interior_points.scale.z = 0.26
        interior_points.color = color(1.0, 0.15, 0.9, 1.0)
        interior_points.points = [point(v["x"], v["y"], 0.26) for v in (route_query.get("interior_targets") or {}).values()]
        markers.markers.append(interior_points)
        marker_id += 1

        for idx, room_id in enumerate(route_query.get("room_sequence") or []):
            if room_id not in rooms:
                continue
            x, y = room_center(rooms[room_id])
            text = self.base_marker(marker_id, Marker.TEXT_VIEW_FACING, "room_chain_labels")
            text.pose.position = point(x, y, 0.65)
            text.scale.z = 0.26
            text.color = color(0.92, 0.97, 1.0, 1.0)
            text.text = f"{idx + 1}: {room_id}"
            markers.markers.append(text)
            marker_id += 1

        summary = self.base_marker(marker_id, Marker.TEXT_VIEW_FACING, "request_summary")
        anchor = (route_query.get("interior_targets") or {}).get("room_16") or (waypoints[-1] if waypoints else {"x": 0.0, "y": 0.0})
        summary.pose.position = point(anchor["x"], anchor["y"], 1.12)
        summary.scale.z = 0.18
        summary.color = color(1.0, 1.0, 1.0, 1.0)
        summary.text = "\n".join([
            f"run_id: {self.run_id}",
            f"start: {route_query.get('start_room')}  goal: {route_query.get('goal_room')}",
            "through: " + ", ".join(route_query.get("through_rooms") or []),
            "rooms: " + " -> ".join(route_query.get("room_sequence") or []),
        ])
        markers.markers.append(summary)
        marker_id += 1

        samples: list[dict[str, Any]] = []
        if self.trajectory_path and self.trajectory_path.exists():
            try:
                samples = read_json(self.trajectory_path).get("samples") or []
            except Exception:
                samples = []
        if len(samples) >= 2:
            pairs = list(zip(samples, samples[1:]))
            if len(pairs) > self.max_trajectory_segments:
                stride = max(1, math.ceil(len(pairs) / self.max_trajectory_segments))
                pairs = pairs[::stride]
            traj = self.base_marker(marker_id, Marker.LINE_LIST, "executed_trajectory")
            traj.scale.x = 0.045
            traj.color = color(1.0, 0.22, 0.12, 1.0)
            for a, b in pairs:
                traj.points.append(point(a["x"], a["y"], 0.24))
                traj.points.append(point(b["x"], b["y"], 0.24))
            markers.markers.append(traj)
            marker_id += 1

        if self.wall_validation_path and self.wall_validation_path.exists():
            try:
                wall = read_json(self.wall_validation_path)
                bad = self.base_marker(marker_id, Marker.LINE_LIST, "wall_diagnostic_segments")
                bad.scale.x = 0.08
                bad.color = color(1.0, 0.0, 0.0, 1.0)
                for seg in (wall.get("suspicious_wall_crossing_segments") or [])[:50]:
                    a = seg.get("from_xy") or {}
                    b = seg.get("to_xy") or {}
                    bad.points.append(point(a.get("x", 0.0), a.get("y", 0.0), 0.34))
                    bad.points.append(point(b.get("x", 0.0), b.get("y", 0.0), 0.34))
                markers.markers.append(bad)
            except Exception:
                pass

        return markers

    def publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.cached_markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.cached_markers)
        if self.compat_publisher is not None:
            self.compat_publisher.publish(self.cached_markers)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--route-query-json", type=Path)
    parser.add_argument("--waypoints-json", type=Path)
    parser.add_argument("--trajectory-json", type=Path)
    parser.add_argument("--wall-validation-json", type=Path)
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument("--run-id", default="manual")
    parser.add_argument("--room-stride", type=int, default=8)
    parser.add_argument("--max-trajectory-segments", type=int, default=500)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--clear-only", action="store_true")
    args = parser.parse_args()
    if IMPORT_ERROR:
        print(json.dumps({"rclpy_import_error": IMPORT_ERROR}), file=sys.stderr)
        return 2
    if not args.clear_only and (not args.route_query_json or not args.waypoints_json):
        print("--route-query-json and --waypoints-json are required unless --clear-only is set", file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = OverlayPublisher(
        args.stage_output_dir.resolve(),
        args.route_query_json.resolve() if args.route_query_json else None,
        args.waypoints_json.resolve() if args.waypoints_json else None,
        args.trajectory_json.resolve() if args.trajectory_json else None,
        args.wall_validation_json.resolve() if args.wall_validation_json else None,
        args.topic,
        args.run_id,
        args.room_stride,
        args.max_trajectory_segments,
        args.clear_only,
    )
    try:
        if args.once or args.clear_only:
            for _ in range(8):
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
