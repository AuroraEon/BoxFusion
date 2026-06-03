#!/usr/bin/env python3
"""Publish task16 dynamic object-nav route, target, and runtime evidence markers."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:  # pragma: no cover - reported by CLI
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
else:
    IMPORT_ERROR = None

ROOT = Path(__file__).resolve().parents[2]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def xy_point(x: float, y: float, z: float) -> Any:
    point = Point()
    point.x, point.y, point.z = float(x), float(y), float(z)
    return point


def rows_from(path: Path) -> list[dict[str, Any]]:
    data = read_json(path)
    return [row for row in (data.get("waypoints") or data.get("points") or data.get("samples") or []) if row.get("x") is not None]


class DynamicMarkerPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rslg_slam_task16_dynamic_objectnav_markers")
        self.args = args
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.count = 0
        self.create_timer(0.8, self.publish_evidence)

    def base(self, marker_id: int, namespace: str, kind: int, rgba: tuple[float, float, float, float]) -> Any:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = namespace, marker_id, kind, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        return marker

    def line(self, marker_id: int, namespace: str, rows: list[dict[str, Any]], rgba: tuple[float, float, float, float], width: float, z: float) -> Any:
        marker = self.base(marker_id, namespace, Marker.LINE_STRIP, rgba)
        marker.scale.x = width
        marker.points = [xy_point(row["x"], row["y"], z) for row in rows]
        return marker

    def sphere(self, marker_id: int, namespace: str, xy: list[float], rgba: tuple[float, float, float, float]) -> Any:
        marker = self.base(marker_id, namespace, Marker.SPHERE, rgba)
        marker.pose.position = xy_point(xy[0], xy[1], 0.14)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.24
        return marker

    def text(self, marker_id: int, namespace: str, xy: list[float], value: str) -> Any:
        marker = self.base(marker_id, namespace, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        marker.pose.position = xy_point(xy[0], xy[1], 0.60)
        marker.scale.z = 0.15
        marker.text = value
        return marker

    def publish_evidence(self) -> None:
        plan = read_json(self.args.plan_json)
        runtime = read_json(self.args.run_dir / "object_facing_runtime_result.json")
        executable = rows_from(self.args.route_json)
        semantic = rows_from(self.args.semantic_route_json)
        candidate = plan.get("approach_candidate_world_xy") or []
        proxy = plan.get("object_proxy_target_for_yaw_alignment") or {}
        items: list[Any] = []
        evidence: list[str] = []

        def add(marker: Any, description: str) -> None:
            items.append(marker)
            evidence.append(description)

        if executable:
            add(self.line(1, "task16/generated_executable_route", executable, (0.05, 0.62, 1.0, 0.9), 0.06, 0.05), "generated executable route")
        if semantic:
            add(self.line(2, "task16/generated_semantic_route", semantic, (1.0, 0.65, 0.05, 1.0), 0.11, 0.09), "generated semantic route and gateway waypoints")
            for idx, waypoint in enumerate(semantic):
                add(self.sphere(100 + idx, "task16/semantic_waypoints", [waypoint["x"], waypoint["y"]], (1.0, 0.55, 0.08, 1.0)), "gateway/room waypoint")
        if len(candidate) >= 2:
            add(self.sphere(10, "task16/approach_candidate", candidate, (0.05, 1.0, 0.25, 1.0)), "selected approach candidate")
        if proxy.get("x") is not None:
            proxy_xy = [proxy["x"], proxy["y"]]
            add(self.sphere(11, "task16/yaw_proxy", proxy_xy, (1.0, 0.1, 0.55, 1.0)), "object/yaw proxy target")
            if len(candidate) >= 2:
                arrow = self.base(12, "task16/facing_ray", Marker.ARROW, (0.2, 1.0, 0.4, 1.0))
                arrow.points = [xy_point(candidate[0], candidate[1], 0.18), xy_point(proxy_xy[0], proxy_xy[1], 0.18)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.13, 0.13
                add(arrow, "facing ray")
        status_anchor = candidate if len(candidate) >= 2 else [executable[-1]["x"], executable[-1]["y"]]
        status = (
            f"query: {plan.get('query')}\n"
            f"resolved: {plan.get('resolved_object_id')} {plan.get('label')}\n"
            f"route: {' -> '.join(plan.get('room_sequence') or [])}\n"
            f"room_arrival={runtime.get('target_room_arrival')} approach={runtime.get('approach_position_reached')}\n"
            f"yaw_aligned={runtime.get('approach_yaw_aligned')} fallback={runtime.get('fallback_used')}"
        )
        add(self.text(20, "task16/runtime_status", [status_anchor[0] + 0.3, status_anchor[1] + 0.8], status), "query and runtime status text")
        sent_count = 0
        for path in sorted((self.args.run_dir / "sent_controller_paths").glob("*.json")):
            rows = rows_from(path)
            if rows:
                add(self.line(300 + sent_count, "task16/sent_controller_paths", rows, (0.62, 0.35, 1.0, 0.65), 0.04, 0.12), "sent controller path")
                sent_count += 1
        trajectory: list[dict[str, Any]] = []
        for path in sorted((self.args.run_dir / "executed_trajectories").glob("*.json")):
            trajectory.extend(rows_from(path))
        if trajectory:
            add(self.line(400, "task16/executed_trajectory", trajectory, (1.0, 0.12, 0.1, 1.0), 0.08, 0.16), "executed trajectory")
        message = MarkerArray()
        message.markers = items
        self.publisher.publish(message)
        self.count += 1
        write_json(self.args.evidence_dir / "marker_manifest.json", {
            "artifact_type": "task16_dynamic_objectnav_marker_manifest",
            "updated_utc": now_iso(),
            "marker_topic": self.args.marker_topic,
            "frame_id": "map",
            "published": True,
            "publish_count": self.count,
            "marker_count": len(items),
            "evidence_layers": evidence,
            "sent_controller_path_count": sent_count,
            "executed_trajectory_sample_count": len(trajectory),
            "plan": rel(self.args.plan_json),
            "runtime_result": rel(self.args.run_dir / "object_facing_runtime_result.json"),
        })


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-json", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--route-json", type=Path, required=True)
    parser.add_argument("--semantic-route-json", type=Path, required=True)
    parser.add_argument("--marker-topic", default="/task16/dynamic_object_nav_markers")
    args = parser.parse_args()
    if IMPORT_ERROR:
        print(IMPORT_ERROR)
        return 2
    rclpy.init()
    node = DynamicMarkerPublisher(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
