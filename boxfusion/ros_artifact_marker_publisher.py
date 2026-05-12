from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from boxfusion.ros_artifact_bridge import DEFAULT_FRAME_ID, CommittedArtifactBundle, build_marker_specs
from boxfusion.route_to_waypoints import route_to_waypoints

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # pragma: no cover - exercised only in ROS-enabled environments
    rclpy = None
    Node = None
    Point = None
    QoSProfile = None
    DurabilityPolicy = None
    remove_ros_args = None
    String = None
    Marker = None
    MarkerArray = None


DEFAULT_MARKER_TOPIC_PREFIX = "/boxfusion/markers"
DEFAULT_STATUS_TOPIC = "/boxfusion/artifacts/status"
DEFAULT_PUBLISH_PERIOD_SEC = 2.0


def _default_route(bundle: CommittedArtifactBundle) -> Dict[str, Any]:
    first_route = bundle.query_report.get("first_route")
    if isinstance(first_route, dict):
        return dict(first_route)
    room_ids = [room.get("id") or room.get("room_id") for room in bundle.topology_rooms]
    room_ids = [room_id for room_id in room_ids if room_id]
    return {"room_sequence": room_ids[:2]}


def _bool_param(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _marker_type(value: Any) -> int:
    mapping = {
        "ARROW": Marker.ARROW,
        "CUBE": Marker.CUBE,
        "SPHERE": Marker.SPHERE,
        "LINE_STRIP": Marker.LINE_STRIP,
        "LINE_LIST": Marker.LINE_LIST,
        "TEXT_VIEW_FACING": Marker.TEXT_VIEW_FACING,
    }
    return mapping.get(str(value or "").upper(), Marker.SPHERE)


def _point(payload: Mapping[str, Any]) -> Any:
    point = Point()
    point.x = float(payload.get("x", 0.0) or 0.0)
    point.y = float(payload.get("y", 0.0) or 0.0)
    point.z = float(payload.get("z", 0.0) or 0.0)
    return point


def _apply_color(marker: Any, color: Mapping[str, Any]) -> None:
    marker.color.r = float(color.get("r", 1.0) or 0.0)
    marker.color.g = float(color.get("g", 1.0) or 0.0)
    marker.color.b = float(color.get("b", 1.0) or 0.0)
    marker.color.a = float(color.get("a", 1.0) or 0.0)


def _apply_scale(marker: Any, scale: Mapping[str, Any], marker_type_name: str) -> None:
    if marker_type_name in {"LINE_STRIP", "LINE_LIST"}:
        marker.scale.x = float(scale.get("x", 0.04) or 0.04)
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        return
    if marker_type_name == "TEXT_VIEW_FACING":
        marker.scale.z = float(scale.get("z", 0.18) or 0.18)
        return
    marker.scale.x = float(scale.get("x", 0.15) or 0.15)
    marker.scale.y = float(scale.get("y", 0.15) or 0.15)
    marker.scale.z = float(scale.get("z", 0.10) or 0.10)


def marker_spec_to_ros_marker(spec: Mapping[str, Any], *, marker_id: int, stamp: Any) -> Any:
    marker = Marker()
    marker.header.frame_id = str(spec.get("frame_id") or DEFAULT_FRAME_ID)
    marker.header.stamp = stamp
    marker.ns = str(spec.get("namespace") or spec.get("group") or "boxfusion")
    marker.id = int(marker_id)
    marker.type = _marker_type(spec.get("type"))
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker_type_name = str(spec.get("type") or "").upper()
    if isinstance(spec.get("position"), Mapping):
        position = _point(spec["position"])
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
    if isinstance(spec.get("points"), list):
        marker.points = [_point(point) for point in spec["points"] if isinstance(point, Mapping)]
    if marker_type_name == "TEXT_VIEW_FACING":
        marker.text = str(spec.get("text") or spec.get("label") or "")

    _apply_scale(marker, dict(spec.get("scale") or {}), marker_type_name)
    _apply_color(marker, dict(spec.get("color") or {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}))
    return marker


def marker_specs_to_arrays(marker_specs: Mapping[str, Any], *, stamp: Any) -> Dict[str, Any]:
    groups = dict(marker_specs.get("groups") or {})
    topic_groups = {
        "rooms": list(groups.get("rooms") or []) + list(groups.get("room_labels") or []),
        "topology": list(groups.get("topology_edges") or []),
        "overlays": list(groups.get("gateways") or []) + list(groups.get("vertical_transitions") or []),
        "objects": list(groups.get("objects") or []) + list(groups.get("anchors") or []),
        "route": list(groups.get("route") or []) + list(groups.get("waypoints") or []),
    }
    if groups.get("debug"):
        topic_groups["debug/audit"] = list(groups["debug"])

    arrays: Dict[str, Any] = {}
    next_marker_id = 1
    for topic_leaf, specs in topic_groups.items():
        array = MarkerArray()
        for spec in specs:
            array.markers.append(marker_spec_to_ros_marker(spec, marker_id=next_marker_id, stamp=stamp))
            next_marker_id += 1
        arrays[topic_leaf] = array
    return arrays


if Node is not None:  # pragma: no branch - definition-only split

    class BoxFusionArtifactMarkerPublisherNode(Node):  # pragma: no cover - requires ROS runtime
        def __init__(
            self,
            *,
            scene_root: Optional[Path] = None,
            frame_id: str = DEFAULT_FRAME_ID,
            marker_topic_prefix: str = DEFAULT_MARKER_TOPIC_PREFIX,
            status_topic: str = DEFAULT_STATUS_TOPIC,
            publish_period_sec: float = DEFAULT_PUBLISH_PERIOD_SEC,
            include_debug: bool = False,
        ) -> None:
            super().__init__("boxfusion_artifact_marker_publisher")
            self.declare_parameter("scene_root", "" if scene_root is None else str(scene_root))
            self.declare_parameter("frame_id", frame_id)
            self.declare_parameter("marker_topic_prefix", marker_topic_prefix)
            self.declare_parameter("status_topic", status_topic)
            self.declare_parameter("publish_period_sec", float(publish_period_sec))
            self.declare_parameter("include_debug", bool(include_debug))

            scene_root = str(self.get_parameter("scene_root").value or "").strip()
            if not scene_root:
                raise ValueError("scene_root parameter is required for the artifact marker publisher.")
            self.frame_id = str(self.get_parameter("frame_id").value or DEFAULT_FRAME_ID)
            self.marker_topic_prefix = str(
                self.get_parameter("marker_topic_prefix").value or DEFAULT_MARKER_TOPIC_PREFIX
            ).rstrip("/")
            status_topic = str(self.get_parameter("status_topic").value or DEFAULT_STATUS_TOPIC)
            publish_period_sec = float(self.get_parameter("publish_period_sec").value or DEFAULT_PUBLISH_PERIOD_SEC)
            include_debug = _bool_param(self.get_parameter("include_debug").value)

            self.bundle = CommittedArtifactBundle.from_scene_root(Path(scene_root))
            route_preview = route_to_waypoints(_default_route(self.bundle), self.bundle, frame_id=self.frame_id)
            self.marker_specs = build_marker_specs(
                self.bundle,
                route_waypoints=route_preview,
                frame_id=self.frame_id,
                include_debug=include_debug,
            )

            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publishers_by_leaf = {
                leaf: self.create_publisher(MarkerArray, f"{self.marker_topic_prefix}/{leaf}", qos)
                for leaf in ("rooms", "topology", "overlays", "objects", "route")
            }
            if include_debug:
                self.publishers_by_leaf["debug/audit"] = self.create_publisher(
                    MarkerArray,
                    f"{self.marker_topic_prefix}/debug/audit",
                    qos,
                )
            self.status_publisher = self.create_publisher(String, status_topic, qos)
            self.timer = self.create_timer(max(0.1, publish_period_sec), self.publish_once)
            self.get_logger().info(
                "Loaded committed/public BoxFusion artifacts for marker publication: "
                + json.dumps(self.bundle.describe(), sort_keys=True)
            )

        def publish_once(self) -> None:
            stamp = self.get_clock().now().to_msg()
            arrays = marker_specs_to_arrays(self.marker_specs, stamp=stamp)
            for leaf, publisher in self.publishers_by_leaf.items():
                publisher.publish(arrays[leaf])
            status = String()
            status.data = json.dumps(
                {
                    "scene_id": self.bundle.scene_id,
                    "frame_id": self.frame_id,
                    "committed_public_only": True,
                    "counts": self.bundle.counts(),
                    "marker_counts": dict(self.marker_specs.get("counts") or {}),
                    "paper_safety_label": self.marker_specs.get("paper_safety_label"),
                    "warnings": list(self.marker_specs.get("warnings") or []),
                },
                sort_keys=True,
            )
            self.status_publisher.publish(status)

else:

    class BoxFusionArtifactMarkerPublisherNode:  # pragma: no cover - import-time fallback
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise ImportError("rclpy is not installed; the artifact marker publisher cannot be created.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish committed BoxFusion artifact marker specs as ROS 2 MarkerArray topics.")
    parser.add_argument("--scene-root", required=True)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--marker-topic-prefix", default=DEFAULT_MARKER_TOPIC_PREFIX)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--publish-period-sec", type=float, default=DEFAULT_PUBLISH_PERIOD_SEC)
    parser.add_argument("--include-debug", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    if rclpy is None:
        raise ImportError("rclpy is not installed; install ROS 2 Python bindings to run the marker publisher.")
    ros_args = None if argv is None else list(argv)
    raw_cli_args = list(remove_ros_args(args=ros_args)) if remove_ros_args is not None else list(argv or [])
    cli_args = raw_cli_args[1:] if argv is None else raw_cli_args
    args = build_arg_parser().parse_args(cli_args)
    rclpy.init(args=ros_args)
    node = BoxFusionArtifactMarkerPublisherNode(
        scene_root=Path(args.scene_root),
        frame_id=args.frame_id,
        marker_topic_prefix=args.marker_topic_prefix,
        status_topic=args.status_topic,
        publish_period_sec=args.publish_period_sec,
        include_debug=args.include_debug,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())
