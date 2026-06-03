#!/usr/bin/env python3
"""Publish task15 object-nav pipeline evidence markers and persist their manifest."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
IMPORT_ERROR: str | None = None
try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return default


def rel(path: Path | None) -> str | None:
    if path is None:
        return None
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def point(x: float, y: float, z: float = 0.0) -> Any:
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def points(rows: list[dict[str, Any]], z: float) -> list[Any]:
    return [point(row["x"], row["y"], z) for row in rows if "x" in row and "y" in row]


def path_points(path: Path) -> list[dict[str, Any]]:
    data = read_json(path, {})
    raw = data.get("samples") if isinstance(data, dict) and data.get("samples") is not None else data.get("points", []) if isinstance(data, dict) else []
    return [r for r in (raw or []) if isinstance(r, dict) and r.get("x") is not None and r.get("y") is not None]


def all_trajectory_samples(run_dir: Path) -> tuple[list[dict[str, Any]], list[Path]]:
    files = [
        run_dir / "executed_trajectories/room_route_segment_trajectory.json",
        run_dir / "executed_trajectories/approach_position_segment_trajectory.json",
        run_dir / "executed_trajectories/yaw_alignment_segment_trajectory.json",
    ]
    present = [p for p in files if p.exists()]
    samples: list[dict[str, Any]] = []
    for path in present:
        for row in path_points(path):
            samples.append({**row, "source_segment_file": rel(path)})
    return samples, present


def save_combined_trajectory(run_dir: Path, evidence_dir: Path) -> tuple[list[dict[str, Any]], list[Path]]:
    samples, sources = all_trajectory_samples(run_dir)
    if samples:
        write_json(
            evidence_dir / "executed_trajectory_combined.json",
            {"artifact_type": "task15_executed_trajectory", "created_utc": now_iso(), "source_files": [rel(p) for p in sources], "samples": samples},
        )
        with (evidence_dir / "executed_trajectory_combined.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["x", "y", "yaw", "source_segment_file"], extrasaction="ignore")
            writer.writeheader()
            writer.writerows(samples)
    return samples, sources


def sent_paths(run_dir: Path) -> list[dict[str, Any]]:
    found = []
    for path in sorted((run_dir / "sent_controller_paths").glob("*.json")):
        rows = path_points(path)
        if rows:
            found.append({"path": path, "points": rows})
    return found


class Task15Publisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rslg_slam_task15_objectnav_markers")
        self.args = args
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.timer = self.create_timer(1.0 / args.rate_hz, self.publish_markers)
        self.marker_inventory: list[dict[str, Any]] = []
        self.publish_count = 0

    def basic_marker(self, marker_id: int, namespace: str, marker_type: int, color: tuple[float, float, float, float]) -> Any:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = 0
        return marker

    def line(self, marker_id: int, namespace: str, rows: list[dict[str, Any]], color: tuple[float, float, float, float], width: float, z: float) -> Any:
        marker = self.basic_marker(marker_id, namespace, Marker.LINE_STRIP, color)
        marker.scale.x = width
        marker.points = points(rows, z)
        return marker

    def sphere(self, marker_id: int, namespace: str, xy: list[float] | tuple[float, float], color: tuple[float, float, float, float], scale: float, z: float) -> Any:
        marker = self.basic_marker(marker_id, namespace, Marker.SPHERE, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(xy[0]), float(xy[1]), z
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        return marker

    def text(self, marker_id: int, namespace: str, xy: tuple[float, float], value: str, color: tuple[float, float, float, float], z: float, scale: float = 0.18) -> Any:
        marker = self.basic_marker(marker_id, namespace, Marker.TEXT_VIEW_FACING, color)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xy[0], xy[1], z
        marker.scale.z = scale
        marker.text = value
        return marker

    def publish_markers(self) -> None:
        plan = read_json(self.args.plan_json, {})
        route = read_json(self.args.route_json, {})
        semantic = read_json(self.args.semantic_route_json, {})
        runtime = read_json(self.args.run_dir / "object_facing_runtime_result.json", {})
        approach = plan.get("approach_candidate_world_xy") or []
        proxy = plan.get("object_proxy_target_for_yaw_alignment") or {}
        semantic_points = semantic.get("waypoints") or []
        route_points = route.get("waypoints") or []
        markers: list[Any] = []
        inventory: list[dict[str, Any]] = []

        def add(marker: Any, evidence: str) -> None:
            markers.append(marker)
            inventory.append({"namespace": marker.ns, "marker_id": marker.id, "evidence": evidence})

        if route_points:
            add(self.line(1, "task15/room_route_dense", route_points, (0.10, 0.60, 1.0, 0.82), 0.055, 0.03), "room route polyline")
        if semantic_points:
            add(self.line(2, "task15/room_route_semantic", semantic_points, (1.0, 0.75, 0.12, 1.0), 0.11, 0.06), "room route semantic polyline")
            for index, waypoint in enumerate(semantic_points):
                xy = [waypoint["x"], waypoint["y"]]
                add(self.sphere(100 + index, "task15/gateway_waypoints", xy, (1.0, 0.55, 0.06, 1.0), 0.18, 0.12), "gateway/route waypoint")
                label = waypoint.get("gateway_id") or waypoint.get("from_room") or "route point"
                add(self.text(200 + index, "task15/gateway_labels", (xy[0], xy[1] + 0.20), str(label), (1.0, 0.80, 0.25, 1.0), 0.23, 0.12), "gateway/route waypoint label")
        if len(approach) >= 2:
            add(self.sphere(10, "task15/approach_candidate", approach, (0.15, 1.0, 0.25, 1.0), 0.24, 0.18), "approach candidate point")
            add(self.text(11, "task15/approach_candidate", (approach[0], approach[1] + 0.26), f"approach: {plan.get('approach_candidate_id')}", (0.10, 1.0, 0.25, 1.0), 0.28), "approach candidate label")
        if proxy.get("x") is not None and proxy.get("y") is not None:
            target_xy = [proxy["x"], proxy["y"]]
            add(self.sphere(12, "task15/object_proxy", target_xy, (1.0, 0.12, 0.60, 1.0), 0.26, 0.18), "object/proxy facing target point")
            add(self.text(13, "task15/object_proxy", (target_xy[0], target_xy[1] + 0.30), f"{plan.get('object_id')} {plan.get('label')} proxy", (1.0, 0.22, 0.70, 1.0), 0.30), "resolved object id and label")
            if len(approach) >= 2:
                ray = [{"x": approach[0], "y": approach[1]}, {"x": target_xy[0], "y": target_xy[1]}]
                marker = self.basic_marker(14, "task15/yaw_facing_ray", Marker.ARROW, (0.18, 1.0, 0.42, 1.0))
                marker.scale.x, marker.scale.y, marker.scale.z = 0.06, 0.14, 0.14
                marker.points = points(ray, 0.20)
                add(marker, "yaw-facing ray from approach candidate to proxy target")
        anchor = tuple(approach) if len(approach) >= 2 else (-7.0, 2.0)
        status = [
            f"query: {plan.get('query')}",
            f"resolved: {plan.get('object_id')} ({plan.get('label')}) -> {plan.get('room_id')}",
            f"target_room_arrival={runtime.get('target_room_arrival')}",
            f"approach_position_reached={runtime.get('approach_position_reached')}",
            f"approach_yaw_aligned={runtime.get('approach_yaw_aligned')}",
            f"object_facing_approach_success={runtime.get('object_facing_approach_success')}",
            f"nav2_clean_approach_success={runtime.get('nav2_clean_approach_success')}",
            f"room_route_sparse_fallback_used={runtime.get('room_route_sparse_fallback_used')}",
            f"yaw_alignment_direct_cmd_vel_fallback_used={runtime.get('yaw_alignment_direct_cmd_vel_fallback_used')}",
        ]
        add(self.text(20, "task15/status", (anchor[0] + 0.35, anchor[1] + 1.55), "\n".join(status), (1.0, 1.0, 1.0, 1.0), 0.8, 0.14), "query, resolved target room, and per-object runtime status text")
        paths = sent_paths(self.args.run_dir)
        for index, entry in enumerate(paths):
            add(self.line(300 + index, "task15/sent_controller_paths", entry["points"], (0.65, 0.36, 1.0, 0.50), 0.045, 0.10), "sent controller path")
        trajectory, trajectory_sources = save_combined_trajectory(self.args.run_dir, self.args.evidence_dir)
        if trajectory:
            add(self.line(400, "task15/executed_trajectory", trajectory, (1.0, 0.10, 0.10, 1.0), 0.085, 0.16), "executed trajectory")
        msg = MarkerArray()
        msg.markers = markers
        self.publisher.publish(msg)
        self.publish_count += 1
        write_json(
            self.args.evidence_dir / "marker_manifest.json",
            {
                "artifact_type": "task15_rviz_objectnav_marker_manifest",
                "updated_utc": now_iso(),
                "project_name": "RSLG-SLAM",
                "marker_topic": self.args.marker_topic,
                "frame_id": "map",
                "published": True,
                "publish_count": self.publish_count,
                "marker_count": len(markers),
                "markers": inventory,
                "status_values": {key: runtime.get(key) for key in [
                    "target_room_arrival",
                    "approach_position_reached",
                    "approach_yaw_aligned",
                    "object_facing_approach_success",
                    "nav2_clean_approach_success",
                    "room_route_sparse_fallback_used",
                    "yaw_alignment_direct_cmd_vel_fallback_used",
                ]},
                "sent_controller_paths": [{"path": rel(p["path"]), "point_count": len(p["points"])} for p in paths],
                "executed_trajectory": {
                    "combined_json": rel(self.args.evidence_dir / "executed_trajectory_combined.json") if trajectory else None,
                    "combined_csv": rel(self.args.evidence_dir / "executed_trajectory_combined.csv") if trajectory else None,
                    "sample_count": len(trajectory),
                    "source_files": [rel(path) for path in trajectory_sources],
                },
                "source_plan": rel(self.args.plan_json),
                "source_runtime_result": rel(self.args.run_dir / "object_facing_runtime_result.json"),
                "map_display_note": "/map is the Nav2 runtime stable occupancy map, not the clean semantic floorplan.",
            },
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-json", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--route-json", type=Path, required=True)
    parser.add_argument("--semantic-route-json", type=Path, required=True)
    parser.add_argument("--marker-topic", default="/task15/object_nav_markers")
    parser.add_argument("--rate-hz", type=float, default=1.0)
    args = parser.parse_args()
    if IMPORT_ERROR:
        print(IMPORT_ERROR, file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = Task15Publisher(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
