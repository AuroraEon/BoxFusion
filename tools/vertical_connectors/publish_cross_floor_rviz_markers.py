#!/usr/bin/env python3
"""Publish RSLG-SLAM cross-floor topology markers as a ROS2 MarkerArray."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:  # ROS2 Foxy compatibility on some installations.
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy


DEFAULT_MARKER_JSON = (
    "/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/"
    "00843-DYehNKdT76V/tasks/"
    "task24d_cross_floor_rviz_overlay_adapter_with_transition_semantics_fix/"
    "cross_floor_rviz_markers_v0_1.json"
)
DEFAULT_TOPIC = "/rslg/cross_floor_markers"
CLAIM_BOUNDARY_TEXT = "\n".join(
    [
        "RSLG-SLAM cross-floor topology overlay",
        "topological_vertical_transition_only",
        "physical_execution_supported=false",
        "no Gazebo/Nav2/Habitat/robot execution in task24e",
    ]
)
Z_SCALE_TEXT = "z scaled for visualization only"
ROUTE_ROOMS = {"room_3", "room_7", "room_13", "room_14"}
FLOOR_CONTEXT_NAMESPACE = "floor_context"
FLOOR_CONTEXT_MARKER_TYPE = "floor_context_marker"


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def marker_key(marker: dict[str, Any], index: int) -> str:
    return str(marker.get("marker_id") or f"marker_{index:04d}")


def get_frame_id(spec: dict[str, Any], marker: dict[str, Any]) -> str:
    return str(marker.get("frame_id") or spec.get("frames", {}).get("default_frame_id") or "map")


def raw_xyz(item: dict[str, Any]) -> list[float]:
    value: Any
    if "position" in item:
        value = item["position"]
        if isinstance(value, dict):
            value = value.get("position") or value.get("xyz") or value.get("position_xyz")
    elif "position_xyz" in item:
        value = item["position_xyz"]
    elif "xyz" in item:
        value = item["xyz"]
    elif "position_xy" in item:
        value = item["position_xy"]
    else:
        value = [0.0, 0.0, 0.0]
    if not isinstance(value, list):
        value = [0.0, 0.0, 0.0]
    xyz = [as_float(value[0]) if len(value) > 0 else 0.0, as_float(value[1]) if len(value) > 1 else 0.0]
    xyz.append(as_float(value[2]) if len(value) > 2 else 0.0)
    return xyz


def marker_position_payload(marker: dict[str, Any]) -> dict[str, Any]:
    position = marker.get("position")
    if isinstance(position, dict):
        return position
    return {"position": position or [0.0, 0.0, 0.0], "floor_id": marker.get("floor_id")}


def point_payloads(marker: dict[str, Any]) -> list[dict[str, Any]]:
    points = marker.get("points")
    if isinstance(points, list):
        return [point for point in points if isinstance(point, dict)]
    return []


def floor_reference_z(markers: list[dict[str, Any]]) -> dict[str, float]:
    """Use 3D connector endpoints as available floor levels for 2D room nodes."""
    by_floor: dict[str, list[float]] = {}
    for marker in markers:
        for payload in [marker_position_payload(marker), *point_payloads(marker)]:
            floor_id = payload.get("floor_id") or marker.get("floor_id")
            xyz = raw_xyz(payload)
            if floor_id and abs(xyz[2]) > 1.0e-9:
                by_floor.setdefault(str(floor_id), []).append(xyz[2])
    references: dict[str, float] = {}
    for floor_id, values in by_floor.items():
        if floor_id == "floor_1":
            references[floor_id] = min(values)
        elif floor_id == "floor_2":
            references[floor_id] = max(values)
        else:
            references[floor_id] = sum(values) / len(values)
    return references


def needs_floor_lift(marker: dict[str, Any], payload: dict[str, Any], xyz: list[float]) -> bool:
    if abs(xyz[2]) > 1.0e-9:
        return False
    node_id = str(payload.get("node_id") or "")
    marker_type = str(marker.get("marker_type") or "")
    if marker_type in {"floor_1_room_marker", "floor_2_room_marker", "route_node_label_marker"}:
        return node_id.startswith("room_") or node_id in ROUTE_ROOMS
    if marker_type == "same_floor_route_marker":
        return node_id.startswith("room_")
    return False


def transformed_xyz(
    marker: dict[str, Any],
    payload: dict[str, Any],
    floor_z: dict[str, float],
    z_visual_scale: float,
) -> list[float]:
    xyz = raw_xyz(payload)
    floor_id = payload.get("floor_id") or marker.get("floor_id")
    if floor_id and needs_floor_lift(marker, payload, xyz):
        xyz[2] = floor_z.get(str(floor_id), xyz[2])
    xyz[2] *= z_visual_scale
    return xyz


def point_from_xyz(xyz: list[float]) -> Point:
    point = Point()
    point.x = xyz[0]
    point.y = xyz[1]
    point.z = xyz[2]
    return point


def color_from_marker(marker: dict[str, Any]) -> ColorRGBA:
    hint = marker.get("color_hint") if isinstance(marker.get("color_hint"), dict) else {}
    color = ColorRGBA()
    color.r = as_float(hint.get("r"), 1.0)
    color.g = as_float(hint.get("g"), 1.0)
    color.b = as_float(hint.get("b"), 1.0)
    color.a = as_float(hint.get("a"), 1.0)
    return color


def scale_tuple(marker: dict[str, Any], default: tuple[float, float, float]) -> tuple[float, float, float]:
    hint = marker.get("scale_hint") if isinstance(marker.get("scale_hint"), dict) else {}
    return (
        as_float(hint.get("x"), default[0]),
        as_float(hint.get("y"), default[1]),
        as_float(hint.get("z"), default[2]),
    )


def marker_type(marker: dict[str, Any]) -> int:
    offline_type = str(marker.get("marker_type") or "")
    if offline_type in {"same_floor_route_marker", "stair_connector_centerline_marker"}:
        return Marker.LINE_STRIP
    if offline_type in {"connector_entry_marker", "connector_exit_marker", "floor_transition_marker"}:
        return Marker.SPHERE
    if offline_type in {"claim_boundary_text_marker", "route_node_label_marker", "z_visual_scale_text_marker"}:
        return Marker.TEXT_VIEW_FACING
    if offline_type in {"floor_1_room_marker", "floor_2_room_marker", FLOOR_CONTEXT_MARKER_TYPE}:
        return Marker.CUBE
    return Marker.SPHERE


def set_pose_position(marker_msg: Marker, xyz: list[float]) -> None:
    marker_msg.pose.position.x = xyz[0]
    marker_msg.pose.position.y = xyz[1]
    marker_msg.pose.position.z = xyz[2]
    marker_msg.pose.orientation.w = 1.0


def to_ros_marker(
    spec: dict[str, Any],
    marker: dict[str, Any],
    index: int,
    floor_z: dict[str, float],
    z_visual_scale: float,
) -> Marker:
    msg = Marker()
    msg.header.frame_id = get_frame_id(spec, marker)
    msg.ns = str(marker.get("namespace") or marker.get("marker_type") or "rslg_cross_floor")
    msg.id = index
    msg.action = Marker.ADD
    msg.type = marker_type(marker)
    msg.color = color_from_marker(marker)
    msg.lifetime = Duration(sec=0, nanosec=0)
    sx, sy, sz = scale_tuple(marker, (0.18, 0.18, 0.18))

    if msg.type == Marker.LINE_STRIP:
        msg.scale.x = max(sx, 0.01)
        msg.points = [
            point_from_xyz(transformed_xyz(marker, payload, floor_z, z_visual_scale))
            for payload in point_payloads(marker)
        ]
        msg.pose.orientation.w = 1.0
        return msg

    if msg.type == Marker.TEXT_VIEW_FACING:
        label = str(marker.get("label") or marker_key(marker, index))
        if marker.get("marker_type") == "claim_boundary_text_marker":
            label = CLAIM_BOUNDARY_TEXT
        msg.text = label
        msg.scale.z = max(sz, sx, 0.12)
        xyz = transformed_xyz(marker, marker_position_payload(marker), floor_z, z_visual_scale)
        if marker.get("marker_type") == "claim_boundary_text_marker" and "floor_2" in floor_z:
            xyz[2] = floor_z["floor_2"] * z_visual_scale + 0.8
        else:
            xyz[2] += 0.32
        set_pose_position(msg, xyz)
        return msg

    msg.scale.x = max(sx, 0.01)
    msg.scale.y = max(sy, 0.01)
    msg.scale.z = max(sz, 0.01)
    set_pose_position(msg, transformed_xyz(marker, marker_position_payload(marker), floor_z, z_visual_scale))
    return msg


def synthetic_text_marker(marker_id: str, label: str, position: list[float]) -> dict[str, Any]:
    return {
        "marker_id": marker_id,
        "marker_type": "z_visual_scale_text_marker",
        "namespace": "claim_boundary",
        "frame_id": "map",
        "label": label,
        "position": {"position": position},
        "scale_hint": {"x": 0.22, "y": 0.22, "z": 0.22},
        "color_hint": {"r": 0.15, "g": 0.05, "b": 0.0, "a": 1.0},
    }


def floor_extent_payloads(markers: list[dict[str, Any]]) -> dict[str, list[list[float]]]:
    floor_points: dict[str, list[list[float]]] = {}
    extent_marker_types = {
        "floor_1_room_marker",
        "floor_2_room_marker",
        "same_floor_route_marker",
        "route_node_label_marker",
    }
    for marker in markers:
        if str(marker.get("marker_type") or "") not in extent_marker_types:
            continue
        for payload in [marker_position_payload(marker), *point_payloads(marker)]:
            floor_id = payload.get("floor_id") or marker.get("floor_id")
            if not floor_id:
                continue
            floor_points.setdefault(str(floor_id), []).append(raw_xyz(payload))
    return floor_points


def synthetic_floor_context_markers(markers: list[dict[str, Any]], floor_z: dict[str, float]) -> list[dict[str, Any]]:
    floor_points = floor_extent_payloads(markers)
    context_markers: list[dict[str, Any]] = []
    colors = {
        "floor_1": {"r": 0.05, "g": 0.38, "b": 0.86, "a": 0.13},
        "floor_2": {"r": 0.08, "g": 0.58, "b": 0.24, "a": 0.13},
    }
    for floor_id in ("floor_1", "floor_2"):
        points = floor_points.get(floor_id, [])
        if floor_id not in floor_z or not points:
            continue
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        padding = 0.7
        min_x = min(xs) - padding
        max_x = max(xs) + padding
        min_y = min(ys) - padding
        max_y = max(ys) + padding
        context_markers.append(
            {
                "marker_id": f"task24e2_floor_context_{floor_id}",
                "marker_type": FLOOR_CONTEXT_MARKER_TYPE,
                "namespace": FLOOR_CONTEXT_NAMESPACE,
                "frame_id": "map",
                "floor_id": floor_id,
                "label": f"{floor_id} context plane",
                "position": {
                    "floor_id": floor_id,
                    "position": [
                        (min_x + max_x) / 2.0,
                        (min_y + max_y) / 2.0,
                        floor_z[floor_id] - 0.03,
                    ],
                },
                "scale_hint": {"x": max(max_x - min_x, 0.2), "y": max(max_y - min_y, 0.2), "z": 0.03},
                "color_hint": colors.get(floor_id, {"r": 0.5, "g": 0.5, "b": 0.5, "a": 0.13}),
            }
        )
    return context_markers


def position_from_node(node: dict[str, Any]) -> list[float]:
    if "position_xyz" in node:
        return [as_float(v) for v in node["position_xyz"][:3]]
    if "xyz" in node:
        return [as_float(v) for v in node["xyz"][:3]]
    if "position_xy" in node:
        return [as_float(node["position_xy"][0]), as_float(node["position_xy"][1]), 0.0]
    room = node.get("room") if isinstance(node.get("room"), dict) else {}
    center = room.get("center") if isinstance(room.get("center"), list) else [0.0, 0.0]
    return [as_float(center[0]), as_float(center[1]), 0.0]


def derive_marker_spec(topology: dict[str, Any], route_report: dict[str, Any]) -> dict[str, Any]:
    nodes = {str(node.get("node_id")): node for node in topology.get("nodes", []) if node.get("node_id")}
    route_nodes = [str(node_id) for node_id in route_report.get("route_nodes", [])]
    markers: list[dict[str, Any]] = []
    for node_id, node in sorted(nodes.items()):
        if node.get("node_type") == "room":
            floor_id = str(node.get("floor_id") or "unknown_floor")
            markers.append(
                {
                    "marker_id": f"{floor_id}_room_marker_{node_id}",
                    "marker_type": f"{floor_id}_room_marker",
                    "namespace": f"{floor_id}_rooms",
                    "frame_id": "map",
                    "floor_id": floor_id,
                    "label": node_id,
                    "position": {"node_id": node_id, "floor_id": floor_id, "position": position_from_node(node)},
                    "scale_hint": {"x": 0.16, "y": 0.16, "z": 0.16},
                    "color_hint": {"r": 0.21, "g": 0.54, "b": 0.82, "a": 0.65}
                    if floor_id == "floor_1"
                    else {"r": 0.35, "g": 0.67, "b": 0.38, "a": 0.65},
                }
            )
    for floor_id in ("floor_1", "floor_2"):
        points = [
            {"node_id": node_id, "floor_id": nodes[node_id].get("floor_id"), "position": position_from_node(nodes[node_id])}
            for node_id in route_nodes
            if node_id in nodes and nodes[node_id].get("floor_id") == floor_id
        ]
        markers.append(
            {
                "marker_id": f"same_floor_route_marker_{floor_id}",
                "marker_type": "same_floor_route_marker",
                "namespace": "same_floor_route",
                "frame_id": "map",
                "floor_id": floor_id,
                "label": f"{floor_id} route segment",
                "points": points,
                "scale_hint": {"x": 0.08, "y": 0.08, "z": 0.08},
                "color_hint": {"r": 0.06, "g": 0.35, "b": 0.73, "a": 1.0}
                if floor_id == "floor_1"
                else {"r": 0.08, "g": 0.46, "b": 0.2, "a": 1.0},
            }
        )
    centerline_ids = [node_id for node_id in route_nodes if node_id.startswith("vt_1_centerline_")]
    markers.append(
        {
            "marker_id": "stair_connector_centerline_marker_vc_vt_1",
            "marker_type": "stair_connector_centerline_marker",
            "namespace": "stair_connector_centerline",
            "frame_id": "map",
            "connector_id": "vc_vt_1",
            "label": "vc_vt_1 fitted centerline",
            "points": [
                {"node_id": node_id, "floor_id": nodes[node_id].get("floor_id"), "position": position_from_node(nodes[node_id])}
                for node_id in centerline_ids
                if node_id in nodes
            ],
            "scale_hint": {"x": 0.1, "y": 0.1, "z": 0.1},
            "color_hint": {"r": 0.83, "g": 0.43, "b": 0.08, "a": 1.0},
        }
    )
    for edge in route_report.get("route_edges", []):
        source = str(edge.get("source"))
        target = str(edge.get("target"))
        source_floor = nodes.get(source, {}).get("floor_id")
        target_floor = nodes.get(target, {}).get("floor_id")
        if source_floor and target_floor and source_floor != target_floor:
            source_xyz = position_from_node(nodes[source])
            target_xyz = position_from_node(nodes[target])
            midpoint = [(source_xyz[i] + target_xyz[i]) / 2.0 for i in range(3)]
            markers.append(
                {
                    "marker_id": f"floor_transition_marker_{edge.get('edge_id', 'derived')}",
                    "marker_type": "floor_transition_marker",
                    "namespace": "floor_transition_markers",
                    "frame_id": "map",
                    "connector_id": edge.get("connector_id"),
                    "route_edge_id": edge.get("edge_id"),
                    "label": f"floor transition: {source} to {target}",
                    "position": {
                        "source_node_id": source,
                        "target_node_id": target,
                        "position": midpoint,
                    },
                    "scale_hint": {"x": 0.34, "y": 0.34, "z": 0.34},
                    "color_hint": {"r": 0.86, "g": 0.12, "b": 0.12, "a": 1.0},
                }
            )
    for index, node_id in enumerate(route_nodes):
        if node_id in nodes:
            markers.append(
                {
                    "marker_id": f"route_node_label_marker_{index:03d}_{node_id}",
                    "marker_type": "route_node_label_marker",
                    "namespace": "route_node_labels",
                    "frame_id": "map",
                    "floor_id": nodes[node_id].get("floor_id"),
                    "connector_id": nodes[node_id].get("connector_id"),
                    "label": node_id,
                    "position": {
                        "node_id": node_id,
                        "floor_id": nodes[node_id].get("floor_id"),
                        "position": position_from_node(nodes[node_id]),
                    },
                    "scale_hint": {"x": 0.18, "y": 0.18, "z": 0.18},
                    "color_hint": {"r": 0.08, "g": 0.08, "b": 0.08, "a": 1.0},
                }
            )
    markers.append(
        {
            "marker_id": "claim_boundary_text_marker_task24e_derived",
            "marker_type": "claim_boundary_text_marker",
            "namespace": "claim_boundary",
            "frame_id": "map",
            "label": CLAIM_BOUNDARY_TEXT,
            "position": {"position": [-9.6, 7.5, 0.0]},
            "scale_hint": {"x": 0.22, "y": 0.22, "z": 0.22},
            "color_hint": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 1.0},
        }
    )
    return {
        "schema_version": "task24e_derived_marker_spec_v0_1",
        "frames": {"default_frame_id": "map"},
        "markers": markers,
        "claim_boundary": "topological_vertical_transition_only",
        "physical_execution_supported": False,
    }


def load_marker_spec(marker_json: Path) -> tuple[dict[str, Any], str]:
    payload = read_json(marker_json)
    if isinstance(payload, dict) and isinstance(payload.get("markers"), list):
        return payload, "task24d_marker_like_entries"
    base = marker_json.parent
    topology_path = base / "corrected_cross_floor_topology_v0_2.json"
    route_path = base / "corrected_cross_floor_route_query_report_v0_2.json"
    topology = read_json(topology_path)
    route_report = read_json(route_path)
    return derive_marker_spec(topology, route_report), "derived_from_corrected_topology_and_route_report"


def add_task24e_text_markers(spec: dict[str, Any], z_visual_scale: float) -> dict[str, Any]:
    markers = list(spec.get("markers", []))
    existing_types = {str(marker.get("marker_type")) for marker in markers if isinstance(marker, dict)}
    if "claim_boundary_text_marker" not in existing_types:
        markers.append(
            {
                "marker_id": "claim_boundary_text_marker_task24e",
                "marker_type": "claim_boundary_text_marker",
                "namespace": "claim_boundary",
                "frame_id": "map",
                "label": CLAIM_BOUNDARY_TEXT,
                "position": {"position": [-9.6, 7.5, 0.0]},
                "scale_hint": {"x": 0.22, "y": 0.22, "z": 0.22},
                "color_hint": {"r": 0.0, "g": 0.0, "b": 0.0, "a": 1.0},
            }
        )
    if abs(z_visual_scale - 1.0) > 1.0e-9:
        markers.append(synthetic_text_marker("z_visual_scale_text_marker_task24e", Z_SCALE_TEXT, [-9.6, 6.9, 0.0]))
    updated = dict(spec)
    updated["markers"] = markers
    return updated


def build_marker_array(spec: dict[str, Any], z_visual_scale: float) -> tuple[MarkerArray, dict[str, Any]]:
    spec = add_task24e_text_markers(spec, z_visual_scale)
    offline_markers = [marker for marker in spec.get("markers", []) if isinstance(marker, dict)]
    floor_z = floor_reference_z(offline_markers)
    input_marker_count = len(offline_markers)
    synthetic_context_markers = synthetic_floor_context_markers(offline_markers, floor_z)
    offline_markers.extend(synthetic_context_markers)
    array = MarkerArray()
    for index, marker in enumerate(offline_markers):
        array.markers.append(to_ros_marker(spec, marker, index, floor_z, z_visual_scale))
    summary = {
        "input_marker_count": input_marker_count,
        "synthetic_floor_context_marker_count": len(synthetic_context_markers),
        "published_marker_count": len(array.markers),
        "namespace_counts": dict(Counter(marker.ns for marker in array.markers)),
        "type_counts": dict(Counter(marker.type for marker in array.markers)),
        "offline_marker_type_counts": dict(Counter(str(marker.get("marker_type")) for marker in offline_markers)),
        "floor_reference_z": floor_z,
        "z_visual_scale": z_visual_scale,
        "claim_boundary_text_marker_exists": any(marker.text == CLAIM_BOUNDARY_TEXT for marker in array.markers),
        "z_visual_scale_text_marker_exists": any(marker.text == Z_SCALE_TEXT for marker in array.markers),
        "floor_context_marker_ids": [str(marker.get("marker_id")) for marker in synthetic_context_markers],
        "floor_context_floor_ids": [str(marker.get("floor_id")) for marker in synthetic_context_markers],
    }
    return array, summary


class CrossFloorMarkerPublisher(Node):
    def __init__(self, topic_name: str, marker_array: MarkerArray, publish_rate: float, summary: dict[str, Any]):
        super().__init__("rslg_cross_floor_markerarray_publisher")
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, topic_name, qos)
        self.marker_array = marker_array
        self.topic_name = topic_name
        self.timer = self.create_timer(1.0 / max(publish_rate, 0.05), self.publish_markers)
        self.summary = summary
        self.publish_count = 0
        self.publish_markers()
        self.get_logger().info(
            f"publishing {len(self.marker_array.markers)} cross-floor markers on "
            f"{self.topic_name} at {publish_rate:.3f} Hz; physical_execution_supported=false"
        )
        self.get_logger().info(f"marker_summary={json.dumps(summary, sort_keys=True)}")

    def publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.marker_array)
        self.publish_count += 1


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker-json", default=DEFAULT_MARKER_JSON, help="task24d marker JSON path")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="MarkerArray topic name")
    parser.add_argument("--z-visual-scale", type=float, default=1.0, help="visual-only multiplier applied to z coordinates")
    parser.add_argument("--publish-rate", type=float, default=0.5, help="low-rate periodic publish rate in Hz")
    parser.add_argument("--print-summary", action="store_true", help="print conversion summary and exit without ROS spin")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    marker_json = Path(args.marker_json).expanduser().resolve()
    spec, interpretation = load_marker_spec(marker_json)
    marker_array, summary = build_marker_array(spec, args.z_visual_scale)
    summary["schema_interpretation"] = interpretation
    summary["marker_json"] = marker_json.as_posix()
    summary["topic"] = args.topic
    if args.print_summary:
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    rclpy.init(args=None)
    node = CrossFloorMarkerPublisher(args.topic, marker_array, args.publish_rate, summary)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"shutting down after {node.publish_count} publishes")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
