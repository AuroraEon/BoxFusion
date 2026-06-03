#!/usr/bin/env python3
"""Publish route-debug RViz markers for sparse, executable, and executed paths."""

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
    from nav_msgs.msg import Path as NavPath
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"

from scene_runtime_common import derive_paths, read_json


def rgba(r: float, g: float, b: float, a: float = 1.0) -> Any:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def color_tuple(color: Any) -> tuple[float, float, float, float]:
    return (float(color.r), float(color.g), float(color.b), float(color.a))


def point(x: float, y: float, z: float = 0.0) -> Any:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def pose_stamped(x: float, y: float, yaw: float | None, frame_id: str = "map") -> Any:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.w = 1.0
    if yaw is not None:
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return msg


def compact_anchor_name(waypoint: dict[str, Any]) -> str:
    if waypoint.get("gateway_id"):
        return str(waypoint["gateway_id"]).replace("00843_floor2_", "")
    room = waypoint.get("from_room") or waypoint.get("to_room") or "unknown"
    if waypoint.get("source") == "room_center":
        return f"{room}_center"
    return str(room)


def source_label(waypoint: dict[str, Any]) -> str:
    if waypoint.get("gateway_id"):
        return str(waypoint["gateway_id"])
    room = waypoint.get("from_room") or waypoint.get("to_room") or "unknown_room"
    if waypoint.get("source") == "room_center":
        return f"{room}_center"
    return f"{waypoint.get('source', 'waypoint')}:{room}"


def load_trajectory(path: Path | None) -> tuple[list[dict[str, Any]], Path | None]:
    if path is None:
        return [], None
    if not path.exists():
        return [], None
    if path.suffix.lower() == ".jsonl":
        samples = []
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(row, dict) and "x" in row and "y" in row:
                    samples.append(row)
        return samples, path
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8", newline="") as handle:
            rows = [dict(row) for row in csv.DictReader(handle)]
        samples = []
        for row in rows:
            try:
                samples.append({"x": float(row["x"]), "y": float(row["y"]), "yaw": float(row.get("yaw", "nan"))})
            except (KeyError, ValueError):
                continue
        return samples, path
    payload = read_json(path)
    samples = payload.get("samples") if isinstance(payload, dict) else payload
    if not isinstance(samples, list):
        return [], None
    return [s for s in samples if isinstance(s, dict) and "x" in s and "y" in s], path


def locate_latest_trajectory(stage_output_dir: Path) -> Path | None:
    patterns = [
        "runs/active/*task4*/trajectory_sample_result_v0_1.json",
        "runs/active/*task4*/trajectory_sample_result.json",
        "runs/active/*/trajectory_sample_result_v0_1.json",
        "runs/active/*/trajectory_sample_result.json",
    ]
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(stage_output_dir.glob(pattern))
    candidates = [path for path in candidates if path.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def run_artifact(run_dir: Path | None, name: str) -> Path | None:
    if run_dir is None:
        return None
    candidate = run_dir / name
    return candidate if candidate.exists() else None


def load_route_execution(path: Path | None, trajectory_path: Path | None) -> tuple[dict[str, Any], Path | None]:
    candidates: list[Path] = []
    if path is not None:
        candidates.append(path)
    if trajectory_path is not None:
        candidates.extend([
            trajectory_path.parent / "route_execution_result_v0_1.json",
            trajectory_path.parent / "route_execution_result.json",
        ])
    for candidate in candidates:
        if candidate.exists():
            payload = read_json(candidate)
            return payload if isinstance(payload, dict) else {}, candidate
    return {}, None


def load_sent_controller_paths(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.exists():
        return []
    payloads = []
    for candidate in sorted(path.glob("*.json")):
        try:
            payload = read_json(candidate)
        except Exception:
            continue
        if not isinstance(payload, dict):
            continue
        points = payload.get("points")
        if isinstance(points, list) and points:
            payload["_source_path"] = candidate.as_posix()
            payloads.append(payload)
    return payloads


def waypoint_by_index(waypoints: list[dict[str, Any]], waypoint_index: int) -> dict[str, Any] | None:
    for waypoint in waypoints:
        if int(waypoint.get("waypoint_index", -1)) == waypoint_index:
            return waypoint
    return None


def derived_phase_name(sample: dict[str, Any]) -> str:
    phase = str(sample.get("phase") or "unknown")
    route_index = sample.get("route_progress_index")
    try:
        progress = int(route_index)
    except (TypeError, ValueError):
        progress = None
    if phase == "post":
        return "room13_dwell_post" if progress is None or progress <= 51 else "terminal_post"
    if progress is None:
        return phase
    if progress <= 39:
        return "slice0"
    if progress <= 51:
        return "slice1"
    return "slice2"


def grouped_phase_samples(samples: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    groups: dict[str, list[dict[str, Any]]] = {}
    for sample in samples:
        groups.setdefault(derived_phase_name(sample), []).append(sample)
    return groups


def semantic_anchor_for_index(semantic_waypoints: list[dict[str, Any]], index: int) -> dict[str, Any]:
    if 0 <= index < len(semantic_waypoints):
        return semantic_waypoints[index]
    return {}


def segment_namespace(scene_short: str, floor_id: str, segment: dict[str, Any], semantic_waypoints: list[dict[str, Any]]) -> str:
    seg_idx = int(segment.get("segment_index", 0))
    start = semantic_anchor_for_index(semantic_waypoints, int(segment.get("start_waypoint_index", seg_idx)))
    target = semantic_anchor_for_index(semantic_waypoints, int(segment.get("target_waypoint_index", seg_idx + 1)))
    return f"scene_{scene_short}/{floor_id}/segment_{seg_idx:02d}_{compact_anchor_name(start)}_to_{compact_anchor_name(target)}"


def segment_label(segment: dict[str, Any], semantic_waypoints: list[dict[str, Any]]) -> str:
    seg_idx = int(segment.get("segment_index", 0))
    start = semantic_anchor_for_index(semantic_waypoints, int(segment.get("start_waypoint_index", seg_idx)))
    target = semantic_anchor_for_index(semantic_waypoints, int(segment.get("target_waypoint_index", seg_idx + 1)))
    extra = []
    if segment.get("bridge_waypoint_count") is not None:
        extra.append(f"bridge={segment['bridge_waypoint_count']}")
    if segment.get("detour_ratio") is not None:
        extra.append(f"detour={segment['detour_ratio']}")
    if segment.get("min_clearance_m") is not None:
        extra.append(f"clearance={segment['min_clearance_m']}m")
    suffix = " " + " ".join(extra) if extra else ""
    return f"seg{seg_idx} {source_label(start)} -> {source_label(target)}{suffix}"


def collect_segment_points(
    segment: dict[str, Any],
    semantic_waypoints: list[dict[str, Any]],
    executable_waypoints: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    seg_idx = int(segment.get("segment_index", 0))
    start = semantic_anchor_for_index(semantic_waypoints, int(segment.get("start_waypoint_index", seg_idx)))
    target = semantic_anchor_for_index(semantic_waypoints, int(segment.get("target_waypoint_index", seg_idx + 1)))
    bridge = [wp for wp in executable_waypoints if wp.get("segment_index") == seg_idx]
    points = []
    if start:
        points.append(start)
    points.extend(bridge)
    if target:
        points.append(target)
    return points


def marker_qos() -> Any:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class RouteDebugOverlayPublisher(Node):  # pragma: no cover - exercised by ROS validation
    def __init__(self, args: argparse.Namespace, paths: dict[str, Any]) -> None:
        super().__init__("boxfusion_scene_route_debug_overlay")
        self.args = args
        self.paths = paths
        self.scene_short = str(args.scene_id).split("-")[0]
        self.marker_topic = args.marker_topic
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, marker_qos())
        self.sparse_path_pub = self.create_publisher(NavPath, f"/stage1_nav/scene_{self.scene_short}/{args.floor_id}/debug_sparse_semantic_path", marker_qos())
        self.executable_path_pub = self.create_publisher(NavPath, f"/stage1_nav/scene_{self.scene_short}/{args.floor_id}/debug_executable_path", marker_qos())
        self.trajectory_path_pub = self.create_publisher(NavPath, f"/stage1_nav/scene_{self.scene_short}/{args.floor_id}/debug_executed_trajectory_path", marker_qos())
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.semantic_payload = read_json(args.semantic_waypoints_json)
        self.executable_payload = read_json(args.executable_waypoints_json)
        self.diagnostics = read_json(args.diagnostics_json) if args.diagnostics_json else {}
        self.semantic_waypoints = self.semantic_payload.get("waypoints", [])
        self.executable_waypoints = self.executable_payload.get("waypoints", [])
        self.run_dir = args.run_dir.resolve() if args.run_dir else None
        trace_path = args.trajectory_trace_jsonl or run_artifact(self.run_dir, "route_execution_trace_v0_1.jsonl")
        trajectory_path = args.trajectory_json or trace_path or run_artifact(self.run_dir, "trajectory_sample_result_v0_1.json") or locate_latest_trajectory(Path(paths["stage_output"]))
        self.trajectory_samples, self.trajectory_path = load_trajectory(trajectory_path)
        route_execution_json = args.route_execution_json or run_artifact(self.run_dir, "route_execution_result_v0_1.json")
        self.route_execution, self.route_execution_path = load_route_execution(route_execution_json, self.trajectory_path)
        self.sent_controller_paths_dir = args.sent_controller_paths_dir or (self.run_dir / "sent_controller_paths" if self.run_dir else None)
        self.sent_controller_paths = load_sent_controller_paths(self.sent_controller_paths_dir)
        self.marker_array = self.build_markers()
        self.sparse_path = self.build_path(self.semantic_waypoints)
        self.executable_path = self.build_path(self.executable_waypoints)
        self.trajectory_path_msg = self.build_path(self.trajectory_samples)
        self.timer = self.create_timer(1.0 / max(args.rate_hz, 0.1), self.publish_all)
        self.publish_all()
        self.get_logger().info(json.dumps({
            "marker_topic": self.marker_topic,
            "marker_count": len(self.marker_array.markers),
            "namespaces": sorted({marker.ns for marker in self.marker_array.markers}),
            "semantic_waypoint_count": len(self.semantic_waypoints),
            "executable_waypoint_count": len(self.executable_waypoints),
            "trajectory_sample_count": len(self.trajectory_samples),
            "trajectory_path": self.trajectory_path.as_posix() if self.trajectory_path else None,
            "route_execution_path": self.route_execution_path.as_posix() if self.route_execution_path else None,
            "sent_controller_paths_dir": self.sent_controller_paths_dir.as_posix() if self.sent_controller_paths_dir else None,
            "sent_controller_path_count": len(self.sent_controller_paths),
        }, sort_keys=True))

    def ns(self, name: str) -> str:
        return f"scene_{self.scene_short}/{self.args.floor_id}/{name}"

    def marker(self, marker_id: int, marker_type: int, namespace: str) -> Any:
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = namespace
        msg.id = marker_id
        msg.type = marker_type
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        return msg

    def text_marker(self, marker_id: int, namespace: str, x: float, y: float, z: float, text: str, color: Any, scale: float = 0.22) -> Any:
        msg = self.marker(marker_id, Marker.TEXT_VIEW_FACING, namespace)
        msg.pose.position = point(x, y, z)
        msg.scale.z = scale
        msg.color = color
        msg.text = text
        return msg

    def line_marker(self, marker_id: int, namespace: str, samples: list[dict[str, Any]], color: Any, z: float, width: float) -> Any:
        msg = self.marker(marker_id, Marker.LINE_STRIP, namespace)
        msg.scale.x = width
        msg.color = color
        msg.points = [point(sample["x"], sample["y"], z) for sample in samples if "x" in sample and "y" in sample]
        return msg

    def build_path(self, waypoints: list[dict[str, Any]]) -> Any:
        msg = NavPath()
        msg.header.frame_id = "map"
        for waypoint in waypoints:
            try:
                msg.poses.append(pose_stamped(float(waypoint["x"]), float(waypoint["y"]), waypoint.get("yaw")))
            except (KeyError, TypeError, ValueError):
                continue
        return msg

    def build_markers(self) -> Any:
        marker_array = MarkerArray()
        marker_id = 1

        sparse_points = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("sparse_anchor_points"))
        sparse_points.scale.x = sparse_points.scale.y = sparse_points.scale.z = 0.22
        sparse_points.color = rgba(1.0, 0.84, 0.18, 1.0)
        sparse_points.points = [point(wp["x"], wp["y"], 0.35) for wp in self.semantic_waypoints]
        marker_array.markers.append(sparse_points)
        marker_id += 1

        sparse_line = self.marker(marker_id, Marker.LINE_STRIP, self.ns("sparse_anchor_line"))
        sparse_line.scale.x = 0.045
        sparse_line.color = rgba(1.0, 0.84, 0.18, 0.92)
        sparse_line.points = [point(wp["x"], wp["y"], 0.30) for wp in self.semantic_waypoints]
        marker_array.markers.append(sparse_line)
        marker_id += 1

        for index, wp in enumerate(self.semantic_waypoints):
            label = f"s{index} {wp.get('source')} {source_label(wp)}"
            marker_array.markers.append(self.text_marker(marker_id, self.ns("sparse_anchor_labels"), wp["x"], wp["y"], 0.72, label, rgba(1.0, 0.95, 0.52, 1.0)))
            marker_id += 1

        room_centers = [wp for wp in self.semantic_waypoints if wp.get("source") == "room_center"]
        room_center_markers = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("room_center_markers"))
        room_center_markers.scale.x = room_center_markers.scale.y = room_center_markers.scale.z = 0.28
        room_center_markers.color = rgba(1.0, 0.62, 0.10, 1.0)
        room_center_markers.points = [point(wp["x"], wp["y"], 0.64) for wp in room_centers]
        marker_array.markers.append(room_center_markers)
        marker_id += 1

        executable_line = self.marker(marker_id, Marker.LINE_STRIP, self.ns("executable_route_line"))
        executable_line.scale.x = 0.075
        executable_line.color = rgba(0.08, 0.72, 1.0, 1.0)
        executable_line.points = [point(wp["x"], wp["y"], 0.42) for wp in self.executable_waypoints]
        marker_array.markers.append(executable_line)
        marker_id += 1

        executable_points = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("executable_route_points"))
        executable_points.scale.x = executable_points.scale.y = executable_points.scale.z = 0.07
        executable_points.color = rgba(0.12, 0.95, 1.0, 0.92)
        executable_points.points = [point(wp["x"], wp["y"], 0.50) for wp in self.executable_waypoints]
        marker_array.markers.append(executable_points)
        marker_id += 1

        label_stride = max(int(self.args.executable_label_stride), 1)
        for index, wp in enumerate(self.executable_waypoints):
            if index % label_stride != 0 and not wp.get("gateway_id") and wp.get("source") != "room_center":
                continue
            label = f"e{index} {wp.get('source')}"
            if wp.get("gateway_id") or wp.get("source") == "room_center":
                label += f" {source_label(wp)}"
            marker_array.markers.append(self.text_marker(marker_id, self.ns("executable_route_labels"), wp["x"], wp["y"], 0.88, label, rgba(0.70, 0.94, 1.0, 1.0), scale=0.18))
            marker_id += 1

        for segment in self.diagnostics.get("segments", []):
            ns = segment_namespace(self.scene_short, self.args.floor_id, segment, self.semantic_waypoints)
            points = collect_segment_points(segment, self.semantic_waypoints, self.executable_waypoints)
            line = self.marker(marker_id, Marker.LINE_STRIP, ns)
            line.scale.x = 0.10
            segment_index = int(segment.get("segment_index", 0))
            if segment_index == 3:
                line.color = rgba(1.0, 0.48, 0.04, 0.95)
            elif segment_index == 4:
                line.color = rgba(0.0, 0.78, 1.0, 0.95)
            else:
                line.color = rgba(0.65, 0.65, 0.65, 0.62)
            line.points = [point(wp["x"], wp["y"], 0.58) for wp in points]
            marker_array.markers.append(line)
            marker_id += 1
            if points:
                mid = points[len(points) // 2]
                label_color = rgba(1.0, 0.68, 0.28, 1.0) if segment_index == 3 else rgba(0.46, 0.92, 1.0, 1.0) if segment_index == 4 else rgba(0.86, 0.86, 0.86, 1.0)
                marker_array.markers.append(self.text_marker(marker_id, self.ns("segment_labels"), mid["x"], mid["y"], 1.08, segment_label(segment, self.semantic_waypoints), label_color, scale=0.18))
                marker_id += 1

        gateway_waypoints = [wp for wp in self.semantic_waypoints if wp.get("gateway_id")]
        gateways = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("gateway_markers"))
        gateways.scale.x = gateways.scale.y = gateways.scale.z = 0.24
        gateways.color = rgba(0.95, 0.95, 0.95, 1.0)
        gateways.points = [point(wp["x"], wp["y"], 0.76) for wp in gateway_waypoints if wp.get("gateway_id") != "00843_floor2_gateway_006"]
        marker_array.markers.append(gateways)
        marker_id += 1

        gateway_006 = [wp for wp in gateway_waypoints if wp.get("gateway_id") == "00843_floor2_gateway_006"]
        if gateway_006:
            highlighted = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("gateway_006_marker"))
            highlighted.scale.x = highlighted.scale.y = highlighted.scale.z = 0.48
            highlighted.color = rgba(1.0, 0.28, 0.18, 1.0)
            highlighted.points = [point(wp["x"], wp["y"], 0.86) for wp in gateway_006]
            marker_array.markers.append(highlighted)
            marker_id += 1
            halo = self.marker(marker_id, Marker.SPHERE_LIST, self.ns("gateway_006_halo"))
            halo.scale.x = halo.scale.y = halo.scale.z = 0.62
            halo.color = rgba(1.0, 1.0, 1.0, 0.72)
            halo.points = [point(wp["x"], wp["y"], 0.84) for wp in gateway_006]
            marker_array.markers.append(halo)
            marker_id += 1

        gateway_registry = self.load_gateway_registry()
        for wp in gateway_waypoints:
            gateway = gateway_registry.get(wp.get("gateway_id"), {})
            pair = gateway.get("pair_key") or wp.get("pair_key") or ""
            label = f"{wp.get('gateway_id')} {pair}".strip()
            label_color = rgba(1.0, 0.32, 0.24, 1.0) if wp.get("gateway_id") == "00843_floor2_gateway_006" else rgba(0.96, 0.96, 0.96, 1.0)
            marker_array.markers.append(self.text_marker(marker_id, self.ns("gateway_labels"), wp["x"], wp["y"], 1.16, label, label_color, scale=0.18))
            marker_id += 1

        if self.trajectory_samples and (self.args.show_success_trajectory or not self.run_dir):
            trajectory = self.line_marker(marker_id, self.ns("executed_trajectory"), self.trajectory_samples, rgba(1.0, 0.26, 0.10, 1.0), 0.66, 0.065)
            marker_array.markers.append(trajectory)
            marker_id += 1
            start = self.trajectory_samples[0]
            end = self.trajectory_samples[-1]
            marker_array.markers.append(self.text_marker(marker_id, self.ns("executed_trajectory_labels"), start["x"], start["y"], 1.10, "trajectory start", rgba(1.0, 0.54, 0.38, 1.0), scale=0.18))
            marker_id += 1
            marker_array.markers.append(self.text_marker(marker_id, self.ns("executed_trajectory_labels"), end["x"], end["y"], 1.10, "trajectory end", rgba(1.0, 0.54, 0.38, 1.0), scale=0.18))
            marker_id += 1

        if self.trajectory_samples and self.args.show_phase_trajectory:
            phase_colors = {
                "slice0": rgba(0.95, 0.18, 0.90, 1.0),
                "room13_dwell_post": rgba(1.0, 0.92, 0.18, 1.0),
                "slice1": rgba(0.18, 0.98, 0.82, 1.0),
                "slice2": rgba(0.64, 0.38, 1.0, 1.0),
                "terminal_post": rgba(1.0, 0.52, 0.12, 1.0),
                "follow_path": rgba(0.95, 0.18, 0.90, 1.0),
                "post": rgba(1.0, 0.92, 0.18, 1.0),
                "unknown": rgba(0.90, 0.90, 0.90, 0.95),
            }
            phase_groups = grouped_phase_samples(self.trajectory_samples)
            for phase_name in ("slice0", "room13_dwell_post", "slice1", "slice2", "terminal_post", "follow_path", "post", "unknown"):
                samples = phase_groups.pop(phase_name, [])
                if len(samples) < 2:
                    continue
                color = phase_colors.get(phase_name, rgba(0.90, 0.90, 0.90, 0.95))
                marker_array.markers.append(self.line_marker(marker_id, self.ns(f"phase_trajectory/{phase_name}"), samples, color, 0.74, 0.09))
                marker_id += 1
                mid = samples[len(samples) // 2]
                marker_array.markers.append(self.text_marker(marker_id, self.ns("phase_trajectory_labels"), mid["x"], mid["y"], 1.22, f"phase {phase_name}", color, scale=0.17))
                marker_id += 1
            for phase_name, samples in sorted(phase_groups.items()):
                if len(samples) < 2:
                    continue
                color = rgba(0.90, 0.90, 0.90, 0.95)
                marker_array.markers.append(self.line_marker(marker_id, self.ns(f"phase_trajectory/{phase_name}"), samples, color, 0.74, 0.09))
                marker_id += 1

        if self.sent_controller_paths and self.args.show_sent_controller_paths:
            sent_colors = {
                "slice0_followpath_path": rgba(0.18, 0.62, 1.0, 1.0),
                "slice1a_current_pose_to_gateway006_full_plan": rgba(0.86, 0.42, 1.0, 0.88),
                "slice1a_handoff_staging_prefix_path": rgba(1.0, 0.45, 0.14, 1.0),
                "slice1b_gateway006_remaining_path": rgba(0.06, 0.90, 0.52, 1.0),
                "slice2_current_pose_to_room_14_full_plan": rgba(0.95, 0.22, 0.55, 0.88),
                "slice2_current_pose_to_room_14_path": rgba(0.44, 1.0, 0.18, 1.0),
            }
            for payload in self.sent_controller_paths:
                path_id = str(payload.get("path_id") or Path(str(payload.get("_source_path", "sent_path"))).stem)
                if (not self.args.show_handoff_paths) and "handoff" in path_id:
                    continue
                samples = [p for p in payload.get("points", []) if isinstance(p, dict) and "x" in p and "y" in p]
                if len(samples) < 2:
                    continue
                color = sent_colors.get(path_id, rgba(0.82, 0.82, 1.0, 0.92))
                marker_array.markers.append(self.line_marker(marker_id, self.ns(f"sent_controller_path/{path_id}"), samples, color, 0.98, 0.075))
                marker_id += 1
                mid = samples[len(samples) // 2]
                action_type = payload.get("action_type") or "controller"
                label = f"{path_id}\n{action_type} poses={len(samples)} wall_safe={(payload.get('wall_crossing') or {}).get('wall_crossing_validation_passed')}"
                marker_array.markers.append(self.text_marker(marker_id, self.ns("sent_controller_path_labels"), mid["x"], mid["y"], 1.42, label, color, scale=0.15))
                marker_id += 1

        if self.args.show_key_waypoints:
            for waypoint_index in (33, 39, 51, 58):
                wp = waypoint_by_index(self.executable_waypoints, waypoint_index)
                if not wp:
                    continue
                key = self.marker(marker_id, Marker.SPHERE, self.ns(f"key_waypoint/wp{waypoint_index}"))
                key.pose.position = point(wp["x"], wp["y"], 1.04)
                key.scale.x = key.scale.y = key.scale.z = 0.34
                key.color = rgba(1.0, 1.0, 1.0, 1.0) if waypoint_index != 51 else rgba(1.0, 0.18, 0.12, 1.0)
                marker_array.markers.append(key)
                marker_id += 1
                label = f"wp{waypoint_index} {wp.get('source')}"
                if wp.get("gateway_id"):
                    label += f"\n{wp.get('gateway_id')}"
                elif wp.get("from_room") or wp.get("to_room"):
                    label += f"\n{wp.get('from_room') or wp.get('to_room')}"
                marker_array.markers.append(self.text_marker(marker_id, self.ns("key_waypoint_labels"), wp["x"], wp["y"], 1.55, label, key.color, scale=0.17))
                marker_id += 1

        if self.route_execution:
            runtime = self.marker(marker_id, Marker.TEXT_VIEW_FACING, self.ns("follow_path_fallback_labels"))
            anchor = self.semantic_waypoints[-1] if self.semantic_waypoints else {"x": 0.0, "y": 0.0}
            runtime.pose.position = point(float(anchor["x"]) + 0.35, float(anchor["y"]) + 0.45, 1.35)
            runtime.scale.z = 0.18
            clean = bool(self.route_execution.get("clean_runtime_success"))
            fallback = bool(self.route_execution.get("sparse_fallback_used"))
            wall_passed = self.route_execution.get("wall_crossing_validation_passed")
            runtime.color = rgba(0.18, 1.0, 0.52, 1.0) if clean and not fallback else rgba(1.0, 0.76, 0.20, 1.0) if fallback else rgba(1.0, 0.24, 0.16, 1.0)
            follow_status = self.route_execution.get("follow_path_status") or (self.route_execution.get("follow_path_result") or {}).get("status")
            runtime.text = (
                f"runtime: succeeded={self.route_execution.get('succeeded')}\n"
                f"FollowPath used={self.route_execution.get('follow_path_used')} status={follow_status}\n"
                f"clean_runtime_success={self.route_execution.get('clean_runtime_success')}\n"
                f"sparse_fallback_used={self.route_execution.get('sparse_fallback_used')}\n"
                f"wall_crossing_validation_passed={wall_passed}"
            )
            marker_array.markers.append(runtime)
            marker_id += 1

        summary = self.marker(marker_id, Marker.TEXT_VIEW_FACING, self.ns("summary_labels"))
        anchor = self.semantic_waypoints[0] if self.semantic_waypoints else {"x": 0.0, "y": 0.0}
        summary.pose.position = point(float(anchor["x"]) + 0.2, float(anchor["y"]) - 0.55, 1.45)
        summary.scale.z = 0.18
        summary.color = rgba(1.0, 1.0, 1.0, 1.0)
        summary.text = (
            "sparse anchors = semantic constraints\n"
            "dense route = occupancy-grid A* executable route\n"
            "trajectory = actual robot motion when artifact is available\n"
            "FollowPath may abort; sparse fallback may replan"
        )
        marker_array.markers.append(summary)
        return marker_array

    def load_gateway_registry(self) -> dict[str, dict[str, Any]]:
        path = Path(self.paths["gateway_registry_json"])
        if not path.exists():
            return {}
        payload = read_json(path)
        gateways = payload.get("gateways", []) if isinstance(payload, dict) else []
        return {str(gateway.get("gateway_id")): gateway for gateway in gateways if isinstance(gateway, dict)}

    def validation_summary(self) -> dict[str, Any]:
        namespaces = sorted({marker.ns for marker in self.marker_array.markers})
        text = "\n".join(str(getattr(marker, "text", "")) for marker in self.marker_array.markers if getattr(marker, "text", ""))
        return {
            "marker_topic": self.marker_topic,
            "marker_count": len(self.marker_array.markers),
            "marker_namespaces": namespaces,
            "semantic_waypoint_count": len(self.semantic_waypoints),
            "executable_waypoint_count": len(self.executable_waypoints),
            "trajectory_sample_count": len(self.trajectory_samples),
            "trajectory_path": self.trajectory_path.as_posix() if self.trajectory_path else None,
            "route_execution_path": self.route_execution_path.as_posix() if self.route_execution_path else None,
            "run_dir": self.run_dir.as_posix() if self.run_dir else None,
            "sent_controller_paths_dir": self.sent_controller_paths_dir.as_posix() if self.sent_controller_paths_dir else None,
            "sent_controller_path_count": len(self.sent_controller_paths),
            "contains_executed_trajectory_markers": any(ns.endswith("/executed_trajectory") for ns in namespaces),
            "contains_phase_trajectory_markers": any("/phase_trajectory/" in ns for ns in namespaces),
            "contains_slice0_phase_marker": any(ns.endswith("/phase_trajectory/slice0") for ns in namespaces),
            "contains_room13_dwell_post_phase_marker": any(ns.endswith("/phase_trajectory/room13_dwell_post") for ns in namespaces),
            "contains_slice1_phase_marker": any(ns.endswith("/phase_trajectory/slice1") for ns in namespaces),
            "contains_sent_controller_path_markers": any("/sent_controller_path/" in ns for ns in namespaces),
            "contains_handoff_path_marker": any("slice1a_handoff_staging_prefix_path" in ns for ns in namespaces),
            "contains_key_waypoint_labels": all(f"wp{idx}" in text for idx in (33, 39, 51, 58)),
            "contains_gateway006_highlight": any(ns.endswith("/gateway_006_marker") for ns in namespaces),
            "contains_clean_status_label": all(key in text for key in ("clean_runtime_success", "sparse_fallback_used", "wall_crossing_validation_passed")),
            "passed": bool(self.marker_array.markers),
        }

    def current_robot_marker(self) -> Any | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", self.args.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.02))
        except (TransformException, Exception):
            return None
        marker = self.marker(999001, Marker.ARROW, self.ns("current_robot_pose"))
        marker.pose.position.x = float(transform.transform.translation.x)
        marker.pose.position.y = float(transform.transform.translation.y)
        marker.pose.position.z = 0.90
        marker.pose.orientation = transform.transform.rotation
        marker.scale.x = 0.42
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color = rgba(1.0, 1.0, 1.0, 1.0)
        return marker

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_array.markers = list(self.marker_array.markers)
        robot_marker = self.current_robot_marker()
        if robot_marker is not None:
            marker_array.markers.append(robot_marker)
        for marker in marker_array.markers:
            marker.header.stamp = stamp
        for path_msg in (self.sparse_path, self.executable_path, self.trajectory_path_msg):
            path_msg.header.stamp = stamp
            for pose in path_msg.poses:
                pose.header.stamp = stamp
        self.marker_pub.publish(marker_array)
        self.sparse_path_pub.publish(self.sparse_path)
        self.executable_path_pub.publish(self.executable_path)
        if self.trajectory_path_msg.poses:
            self.trajectory_path_pub.publish(self.trajectory_path_msg)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--semantic-waypoints-json", type=Path, required=True)
    parser.add_argument("--executable-waypoints-json", type=Path, required=True)
    parser.add_argument("--diagnostics-json", type=Path, required=True)
    parser.add_argument("--trajectory-json", type=Path)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--trajectory-trace-jsonl", type=Path)
    parser.add_argument("--sent-controller-paths-dir", type=Path)
    parser.add_argument("--show-sent-controller-paths", action="store_true")
    parser.add_argument("--show-handoff-paths", action="store_true")
    parser.add_argument("--show-success-trajectory", action="store_true")
    parser.add_argument("--show-key-waypoints", action="store_true")
    parser.add_argument("--show-phase-trajectory", action="store_true")
    parser.add_argument("--route-execution-json", type=Path)
    parser.add_argument("--marker-topic", required=True)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--publish-once", action="store_true")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--executable-label-stride", type=int, default=10)
    parser.add_argument("--base-frame", default="base_footprint")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if IMPORT_ERROR:
        print(json.dumps({"rclpy_import_error": IMPORT_ERROR}), file=sys.stderr)
        return 2
    paths = derive_paths(args)
    rclpy.init(args=None)
    node = RouteDebugOverlayPublisher(args, paths)
    start = time.monotonic()
    try:
        if args.publish_once:
            for _ in range(12):
                node.publish_all()
                rclpy.spin_once(node, timeout_sec=0.1)
            if args.output_json:
                from scene_runtime_common import write_json
                write_json(args.output_json, node.validation_summary())
            return 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration_sec > 0 and time.monotonic() - start >= args.duration_sec:
                break
        if args.output_json:
            from scene_runtime_common import write_json
            write_json(args.output_json, node.validation_summary())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
