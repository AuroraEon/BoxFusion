"""ROS-independent bridge helpers for committed BoxFusion artifacts.

This module intentionally returns plain Python dictionaries.  ROS nodes can
turn those dictionaries into MarkerArray messages, but the artifact policy,
coordinate validation, and marker provenance stay testable without ROS.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from boxfusion.committed_artifact_ids import (
    CommittedArtifactIdNormalizer,
    canonical_object_id,
    canonical_room_id,
)


DEFAULT_FRAME_ID = "boxfusion_map"
DEFAULT_FLOOR_LAYER_GAP_M = 0.35
PAPER_SAFETY_LABEL = (
    "Committed/public artifact visualization only; not a collision-free plan, "
    "local planner, Nav2 path, robot-control command, or BEV output."
)

TOPOLOGY_FILENAME = "topology_v0_1.json"
QUERY_REPORT_FILENAME = "topology_query_report.json"
WORLD_MODEL_FILENAME = "committed_room_world_model_v0_1.json"
SNAPSHOT_FILENAME = "committed_room_world_snapshot_v0_1.json"


class ArtifactBridgeError(RuntimeError):
    """Raised when a committed/public artifact bundle cannot be loaded."""


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ArtifactBridgeError(f"Expected a JSON object in {path}")
    return payload


def _clean_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _records(value: Any) -> List[Dict[str, Any]]:
    if not isinstance(value, list):
        return []
    return [dict(item) for item in value if isinstance(item, dict)]


def _as_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(result) or math.isinf(result):
        return None
    return result


def _xy(value: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    x = _as_float(value[0])
    y = _as_float(value[1])
    if x is None or y is None:
        return None
    return x, y


def _xyz(value: Any, *, default_z: float = 0.0) -> Optional[Tuple[float, float, float]]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    xy = _xy(value)
    if xy is None:
        return None
    z = _as_float(value[2]) if len(value) >= 3 else default_z
    return xy[0], xy[1], default_z if z is None else z


def _point_dict(x: float, y: float, z: float = 0.0) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "z": float(z)}


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _polygon_points(value: Any, *, z: float) -> Optional[List[Dict[str, float]]]:
    if not isinstance(value, list) or len(value) < 3:
        return None
    points: List[Dict[str, float]] = []
    for raw_point in value:
        point = _xy(raw_point)
        if point is None:
            return None
        points.append(_point_dict(point[0], point[1], z))
    if points and points[0] != points[-1]:
        points.append(dict(points[0]))
    return points


def normalize_room_id(value: Any) -> Optional[str]:
    """Normalize numeric and canonical room ids to the public ``room_*`` form."""

    return canonical_room_id(value)


def normalize_object_id(value: Any) -> Optional[str]:
    return canonical_object_id(value)


@dataclass
class ValidationIssue:
    severity: str
    code: str
    message: str
    source_artifact: str
    source_ids: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "severity": self.severity,
            "code": self.code,
            "message": self.message,
            "source_artifact": self.source_artifact,
            "source_ids": dict(self.source_ids),
        }


@dataclass
class CommittedArtifactBundle:
    """Read-only loader for the four committed/public bridge artifacts."""

    scene_root: Path
    topology_path: Path
    query_report_path: Path
    world_model_path: Path
    snapshot_path: Path
    topology: Dict[str, Any]
    query_report: Dict[str, Any]
    world_model: Dict[str, Any]
    snapshot: Dict[str, Any]
    id_normalizer: CommittedArtifactIdNormalizer = field(init=False)

    def __post_init__(self) -> None:
        self.scene_root = Path(self.scene_root).resolve()
        self.topology_path = Path(self.topology_path).resolve()
        self.query_report_path = Path(self.query_report_path).resolve()
        self.world_model_path = Path(self.world_model_path).resolve()
        self.snapshot_path = Path(self.snapshot_path).resolve()
        self.id_normalizer = CommittedArtifactIdNormalizer.from_snapshot(self.snapshot)

    @classmethod
    def from_scene_root(cls, scene_root: Path) -> "CommittedArtifactBundle":
        root = Path(scene_root).resolve()
        logs_dir = root / "logs"
        paths = {
            "topology": logs_dir / TOPOLOGY_FILENAME,
            "query_report": logs_dir / QUERY_REPORT_FILENAME,
            "world_model": logs_dir / WORLD_MODEL_FILENAME,
            "snapshot": logs_dir / SNAPSHOT_FILENAME,
        }
        missing = [str(path) for path in paths.values() if not path.exists()]
        if missing:
            raise ArtifactBridgeError(
                "Missing committed/public artifact(s): " + ", ".join(missing)
            )
        return cls(
            scene_root=root,
            topology_path=paths["topology"],
            query_report_path=paths["query_report"],
            world_model_path=paths["world_model"],
            snapshot_path=paths["snapshot"],
            topology=_load_json(paths["topology"]),
            query_report=_load_json(paths["query_report"]),
            world_model=_load_json(paths["world_model"]),
            snapshot=_load_json(paths["snapshot"]),
        )

    @property
    def scene_id(self) -> Optional[str]:
        return _clean_text(self.topology.get("sequence_id")) or _clean_text(self.scene_root.name)

    @property
    def topology_rooms(self) -> List[Dict[str, Any]]:
        return _records(self.topology.get("rooms"))

    @property
    def topology_edges(self) -> List[Dict[str, Any]]:
        return _records(self.topology.get("edges"))

    @property
    def topology_floors(self) -> List[Dict[str, Any]]:
        return _records(self.topology.get("floors"))

    @property
    def snapshot_gateways(self) -> List[Dict[str, Any]]:
        return _records(self.snapshot.get("gateways"))

    @property
    def snapshot_vertical_transitions(self) -> List[Dict[str, Any]]:
        return _records(self.snapshot.get("vertical_transitions"))

    @property
    def snapshot_anchors(self) -> List[Dict[str, Any]]:
        return _records(self.snapshot.get("anchors"))

    @property
    def snapshot_objects(self) -> List[Dict[str, Any]]:
        return _records(self.snapshot.get("objects"))

    def counts(self) -> Dict[str, int]:
        return {
            "rooms": len(self.topology_rooms),
            "edges": len(self.topology_edges),
            "floors": len(self.topology_floors),
            "gateways": len(self.snapshot_gateways),
            "vertical_transitions": len(self.snapshot_vertical_transitions),
            "anchors": len(self.snapshot_anchors),
            "objects": len(self.snapshot_objects),
        }

    def room_index(self) -> Dict[str, Dict[str, Any]]:
        index: Dict[str, Dict[str, Any]] = {}
        for room in self.topology_rooms:
            room_id = normalize_room_id(room.get("id") or room.get("room_id"))
            if room_id is not None:
                index[room_id] = room
        return index

    def floor_index(self) -> Dict[str, Dict[str, Any]]:
        index: Dict[str, Dict[str, Any]] = {}
        for floor in self.topology_floors:
            floor_id = _clean_text(floor.get("floor_id"))
            if floor_id is not None:
                index[floor_id] = floor
        return index

    def floor_display_z(self, floor_id: Any, *, floor_layer_gap_m: float = DEFAULT_FLOOR_LAYER_GAP_M) -> float:
        floor = self.floor_index().get(str(floor_id or ""))
        display_order = _as_float((floor or {}).get("display_order"))
        if display_order is None:
            return 0.0
        return max(0.0, display_order - 1.0) * float(floor_layer_gap_m)

    def describe(self) -> Dict[str, Any]:
        return {
            "scene_root": str(self.scene_root),
            "scene_id": self.scene_id,
            "artifact_paths": {
                "topology": str(self.topology_path),
                "query_report": str(self.query_report_path),
                "world_model": str(self.world_model_path),
                "snapshot": str(self.snapshot_path),
            },
            "counts": self.counts(),
            "frame_id_default": DEFAULT_FRAME_ID,
            "paper_safety_label": PAPER_SAFETY_LABEL,
        }


def _issue(
    issues: List[ValidationIssue],
    severity: str,
    code: str,
    message: str,
    source_artifact: str,
    **source_ids: Any,
) -> None:
    issues.append(
        ValidationIssue(
            severity=severity,
            code=code,
            message=message,
            source_artifact=source_artifact,
            source_ids={key: value for key, value in source_ids.items() if value is not None},
        )
    )


def validate_coordinates(bundle: CommittedArtifactBundle) -> List[Dict[str, Any]]:
    """Validate coordinate availability without mutating the loaded artifacts."""

    issues: List[ValidationIssue] = []
    room_index = bundle.room_index()

    for room in bundle.topology_rooms:
        room_id = normalize_room_id(room.get("id") or room.get("room_id"))
        if _xy(room.get("center")) is None:
            _issue(
                issues,
                "error",
                "missing_topology_room_center",
                "Topology room is missing a usable center; room centers are required for route preview.",
                TOPOLOGY_FILENAME,
                room_id=room_id,
            )
        z = bundle.floor_display_z(room.get("floor_id"))
        if _polygon_points(room.get("polygon"), z=z) is None:
            _issue(
                issues,
                "error",
                "missing_topology_room_polygon",
                "Topology room is missing a usable polygon; topology polygons are the primary room geometry.",
                TOPOLOGY_FILENAME,
                room_id=room_id,
            )
        if _clean_text(room.get("floor_id")) is None or _clean_text(room.get("display_floor_id")) is None:
            _issue(
                issues,
                "warning",
                "missing_room_floor_metadata",
                "Topology room has incomplete floor metadata.",
                TOPOLOGY_FILENAME,
                room_id=room_id,
            )

    if not bundle.topology_floors:
        _issue(
            issues,
            "warning",
            "missing_topology_floors",
            "Topology artifact does not expose floor records; z display layers will fall back to 0.0.",
            TOPOLOGY_FILENAME,
        )
    for floor in bundle.topology_floors:
        if _clean_text(floor.get("floor_id")) is None or _clean_text(floor.get("display_floor_id")) is None:
            _issue(
                issues,
                "warning",
                "missing_floor_metadata",
                "A topology floor record is missing floor_id or display_floor_id.",
                TOPOLOGY_FILENAME,
                floor_id=floor.get("floor_id"),
            )

    _detect_room_center_sign_mismatch(bundle, issues, room_index)

    for row in bundle.id_normalizer.summary().get("warnings", []):
        _issue(
            issues,
            "warning",
            "room_id_normalization_warning",
            str(row.get("message") or "Room id normalization warning."),
            SNAPSHOT_FILENAME,
            context=row.get("context"),
            field=row.get("field"),
            value=row.get("value"),
        )

    for idx, gateway in enumerate(bundle.snapshot_gateways):
        if _xy(gateway.get("pos_world")) is None:
            _issue(
                issues,
                "warning",
                "missing_gateway_pos_world",
                "Snapshot gateway is missing a usable pos_world marker coordinate.",
                SNAPSHOT_FILENAME,
                gateway_index=idx,
                connects=gateway.get("connects"),
            )

    for idx, transition in enumerate(bundle.snapshot_vertical_transitions):
        if _xy(transition.get("from_position_xy")) is None:
            _issue(
                issues,
                "warning",
                "missing_vertical_transition_from_position",
                "Vertical transition is missing from_position_xy.",
                SNAPSHOT_FILENAME,
                transition_id=transition.get("transition_id") or idx,
            )
        if _xy(transition.get("to_position_xy")) is None:
            _issue(
                issues,
                "warning",
                "missing_vertical_transition_to_position",
                "Vertical transition is missing to_position_xy.",
                SNAPSHOT_FILENAME,
                transition_id=transition.get("transition_id") or idx,
            )

    for idx, anchor in enumerate(bundle.snapshot_anchors):
        if _xyz(anchor.get("position"), default_z=bundle.floor_display_z(anchor.get("floor_id"))) is None:
            _issue(
                issues,
                "warning",
                "missing_anchor_position",
                "Snapshot anchor is missing a usable position marker coordinate.",
                SNAPSHOT_FILENAME,
                anchor_id=anchor.get("id") or idx,
            )

    for idx, obj in enumerate(bundle.snapshot_objects):
        if _xyz(obj.get("pose_3d"), default_z=bundle.floor_display_z(obj.get("floor_id"))) is None and _xyz(
            obj.get("pose"), default_z=bundle.floor_display_z(obj.get("floor_id"))
        ) is None:
            _issue(
                issues,
                "warning",
                "missing_object_pose",
                "Snapshot object is missing a usable pose_3d or pose marker coordinate.",
                SNAPSHOT_FILENAME,
                object_id=normalize_object_id(obj.get("id")) or obj.get("id") or idx,
            )

    return [issue.to_dict() for issue in issues]


def _detect_room_center_sign_mismatch(
    bundle: CommittedArtifactBundle,
    issues: List[ValidationIssue],
    topology_rooms: Mapping[str, Dict[str, Any]],
) -> None:
    def check_records(records: Iterable[Dict[str, Any]], source: str, center_field: str) -> None:
        checked = 0
        sign_like = 0
        direct_like = 0
        for record in records:
            room_id = normalize_room_id(record.get("room_id") or record.get("id") or record.get("stable_room_id"))
            if room_id is None or room_id not in topology_rooms:
                continue
            topology_center = _xy(topology_rooms[room_id].get("center"))
            candidate_center = _xy(record.get(center_field))
            if topology_center is None or candidate_center is None:
                continue
            direct_distance = _distance(topology_center, candidate_center)
            sign_distance = _distance(topology_center, (-candidate_center[0], -candidate_center[1]))
            checked += 1
            if sign_distance + 1.0e-6 < direct_distance and sign_distance < 0.75:
                sign_like += 1
            elif direct_distance <= sign_distance:
                direct_like += 1
        if checked >= 2 and sign_like > direct_like and sign_like / checked >= 0.5:
            _issue(
                issues,
                "warning",
                "risky_room_center_sign_mismatch",
                (
                    f"{source} room centers appear sign-flipped relative to topology room centers; "
                    "the bridge will keep topology_v0_1.json rooms[].center/polygon as primary room geometry."
                ),
                source,
                checked_rooms=checked,
                sign_like_rooms=sign_like,
                direct_like_rooms=direct_like,
            )

    check_records(bundle.snapshot.get("rooms") or [], SNAPSHOT_FILENAME, "center")
    check_records(bundle.world_model.get("rooms") or [], WORLD_MODEL_FILENAME, "centroid_xy")


def _floor_payload(record: Mapping[str, Any]) -> Dict[str, Any]:
    return {
        "floor_id": record.get("floor_id"),
        "display_floor_id": record.get("display_floor_id"),
        "display_order": record.get("display_order"),
    }


def _base_marker(
    *,
    marker_id: str,
    group: str,
    marker_type: str,
    frame_id: str,
    namespace: str,
    source_artifact: str,
    source_ids: Mapping[str, Any],
    floor_id: Any = None,
    display_floor_id: Any = None,
    label: Optional[str] = None,
    text: Optional[str] = None,
    visualization_only: bool = True,
    not_collision_free: bool = False,
    color: Optional[Mapping[str, float]] = None,
    scale: Optional[Mapping[str, float]] = None,
) -> Dict[str, Any]:
    marker = {
        "marker_id": marker_id,
        "group": group,
        "type": marker_type,
        "frame_id": frame_id,
        "namespace": namespace,
        "floor_id": floor_id,
        "display_floor_id": display_floor_id,
        "source_artifact": source_artifact,
        "source_ids": dict(source_ids),
        "paper_safety_label": PAPER_SAFETY_LABEL,
        "visualization_only": bool(visualization_only),
        "not_collision_free": bool(not_collision_free),
    }
    if label is not None:
        marker["label"] = label
    if text is not None:
        marker["text"] = text
    if color is not None:
        marker["color"] = dict(color)
    if scale is not None:
        marker["scale"] = dict(scale)
    return marker


def _floor_color(display_order: Any, *, alpha: float = 1.0) -> Dict[str, float]:
    palette = [
        (0.24, 0.58, 0.92),
        (0.35, 0.73, 0.48),
        (0.89, 0.58, 0.20),
        (0.72, 0.43, 0.86),
    ]
    order = int(display_order or 1)
    r, g, b = palette[(max(1, order) - 1) % len(palette)]
    return {"r": r, "g": g, "b": b, "a": alpha}


def build_marker_specs(
    bundle: CommittedArtifactBundle,
    *,
    route_waypoints: Optional[Mapping[str, Any]] = None,
    frame_id: str = DEFAULT_FRAME_ID,
    include_debug: bool = False,
    floor_layer_gap_m: float = DEFAULT_FLOOR_LAYER_GAP_M,
) -> Dict[str, Any]:
    """Build ROS-independent marker dictionaries grouped by RViz layer."""

    groups: Dict[str, List[Dict[str, Any]]] = {
        "rooms": [],
        "room_labels": [],
        "topology_edges": [],
        "gateways": [],
        "vertical_transitions": [],
        "objects": [],
        "anchors": [],
        "route": [],
        "waypoints": [],
    }
    if include_debug:
        groups["debug"] = []

    room_index = bundle.room_index()
    model_rooms = {
        normalize_room_id(room.get("room_id") or room.get("stable_room_id")): room
        for room in _records(bundle.world_model.get("rooms"))
    }

    for idx, room in enumerate(bundle.topology_rooms):
        room_id = normalize_room_id(room.get("id") or room.get("room_id")) or f"room_{idx}"
        z = bundle.floor_display_z(room.get("floor_id"), floor_layer_gap_m=floor_layer_gap_m)
        polygon = _polygon_points(room.get("polygon"), z=z)
        color = _floor_color(room.get("display_order"), alpha=0.9)
        if polygon is not None:
            marker = _base_marker(
                marker_id=f"room_polygon_{room_id}",
                group="rooms",
                marker_type="LINE_STRIP",
                frame_id=frame_id,
                namespace=f"rooms/{room.get('display_floor_id') or room.get('floor_id') or 'unknown_floor'}",
                source_artifact=TOPOLOGY_FILENAME,
                source_ids={"room_id": room_id},
                floor_id=room.get("floor_id"),
                display_floor_id=room.get("display_floor_id"),
                label=room_id,
                color=color,
                scale={"x": 0.04, "y": 0.04, "z": 0.04},
            )
            marker["points"] = polygon
            groups["rooms"].append(marker)

        center_xy = _xy(room.get("center"))
        if center_xy is not None:
            center = _point_dict(center_xy[0], center_xy[1], z)
            center_marker = _base_marker(
                marker_id=f"room_center_{room_id}",
                group="rooms",
                marker_type="SPHERE",
                frame_id=frame_id,
                namespace=f"room_centers/{room.get('display_floor_id') or room.get('floor_id') or 'unknown_floor'}",
                source_artifact=TOPOLOGY_FILENAME,
                source_ids={"room_id": room_id},
                floor_id=room.get("floor_id"),
                display_floor_id=room.get("display_floor_id"),
                label=f"{room_id} center",
                text="Topological room center; not a guaranteed navigable pose.",
                not_collision_free=True,
                color=_floor_color(room.get("display_order"), alpha=1.0),
                scale={"x": 0.16, "y": 0.16, "z": 0.08},
            )
            center_marker["position"] = center
            groups["rooms"].append(center_marker)

            semantic = dict((model_rooms.get(room_id) or {}).get("semantic_summary") or {})
            dominant = ", ".join(list(semantic.get("dominant_object_labels") or [])[:3])
            label_text = f"{room_id} {room.get('display_floor_id') or room.get('floor_id') or ''}".strip()
            if dominant:
                label_text = f"{label_text}: {dominant}"
            label_marker = _base_marker(
                marker_id=f"room_label_{room_id}",
                group="room_labels",
                marker_type="TEXT_VIEW_FACING",
                frame_id=frame_id,
                namespace="room_labels",
                source_artifact=f"{TOPOLOGY_FILENAME}+{WORLD_MODEL_FILENAME}",
                source_ids={"room_id": room_id},
                floor_id=room.get("floor_id"),
                display_floor_id=room.get("display_floor_id"),
                label=room_id,
                text=label_text,
                color={"r": 0.95, "g": 0.95, "b": 0.95, "a": 1.0},
                scale={"x": 0.0, "y": 0.0, "z": 0.18},
            )
            label_marker["position"] = _point_dict(center_xy[0], center_xy[1], z + 0.16)
            groups["room_labels"].append(label_marker)

    for idx, edge in enumerate(bundle.topology_edges):
        source_room_id = normalize_room_id(edge.get("source"))
        target_room_id = normalize_room_id(edge.get("target"))
        source_room = room_index.get(source_room_id or "")
        target_room = room_index.get(target_room_id or "")
        source_xy = _xy((source_room or {}).get("center"))
        target_xy = _xy((target_room or {}).get("center"))
        if source_xy is None or target_xy is None:
            continue
        source_z = bundle.floor_display_z((source_room or {}).get("floor_id"), floor_layer_gap_m=floor_layer_gap_m)
        target_z = bundle.floor_display_z((target_room or {}).get("floor_id"), floor_layer_gap_m=floor_layer_gap_m)
        relation_type = str(edge.get("relation_type") or "unknown")
        marker = _base_marker(
            marker_id=f"topology_edge_{idx}_{source_room_id}_{target_room_id}",
            group="topology_edges",
            marker_type="LINE_LIST",
            frame_id=frame_id,
            namespace=f"topology_edges/{relation_type}",
            source_artifact=TOPOLOGY_FILENAME,
            source_ids={
                "edge_index": idx,
                "source_room_id": source_room_id,
                "target_room_id": target_room_id,
                "evidence_ids": list(edge.get("evidence_ids") or []),
            },
            floor_id=(source_room or {}).get("floor_id"),
            display_floor_id=(source_room or {}).get("display_floor_id"),
            label=f"{source_room_id}->{target_room_id} {relation_type}",
            text=f"{relation_type}, conf={edge.get('confidence')}, support={edge.get('support_count')}",
            not_collision_free=True,
            color={"r": 0.65, "g": 0.72, "b": 0.78, "a": 0.85},
            scale={"x": 0.03, "y": 0.03, "z": 0.03},
        )
        if relation_type == "vertical_transition":
            marker["color"] = {"r": 0.74, "g": 0.35, "b": 0.95, "a": 0.95}
        elif relation_type == "transition":
            marker["color"] = {"r": 0.27, "g": 0.66, "b": 0.94, "a": 0.9}
        marker["points"] = [
            _point_dict(source_xy[0], source_xy[1], source_z + 0.04),
            _point_dict(target_xy[0], target_xy[1], target_z + 0.04),
        ]
        groups["topology_edges"].append(marker)

    for normalized in bundle.id_normalizer.normalized_gateways:
        gateway = normalized["record"]
        point = _xy(gateway.get("pos_world"))
        if point is None:
            continue
        z = bundle.floor_display_z(gateway.get("floor_id"), floor_layer_gap_m=floor_layer_gap_m) + 0.08
        marker = _base_marker(
            marker_id=f"gateway_{normalized['index']}",
            group="gateways",
            marker_type="CUBE",
            frame_id=frame_id,
            namespace=f"gateways/{gateway.get('display_floor_id') or gateway.get('floor_id') or 'unknown_floor'}",
            source_artifact=SNAPSHOT_FILENAME,
            source_ids={
                "gateway_index": normalized["index"],
                "room_a": normalized.get("room_a"),
                "room_b": normalized.get("room_b"),
                "raw_connects": normalized.get("raw_connects"),
            },
            floor_id=gateway.get("floor_id"),
            display_floor_id=gateway.get("display_floor_id"),
            label=f"gateway {normalized.get('room_a')}<->{normalized.get('room_b')}",
            text="Gateway evidence marker; not a clearance guarantee.",
            not_collision_free=True,
            color={"r": 0.95, "g": 0.55, "b": 0.18, "a": 0.95},
            scale={"x": 0.18, "y": 0.18, "z": 0.08},
        )
        marker["position"] = _point_dict(point[0], point[1], z)
        marker["yaw"] = gateway.get("yaw")
        groups["gateways"].append(marker)

    for normalized in bundle.id_normalizer.normalized_vertical_transitions:
        transition = normalized["record"]
        from_xy = _xy(transition.get("from_position_xy"))
        to_xy = _xy(transition.get("to_position_xy"))
        from_z = bundle.floor_display_z(transition.get("from_floor_id"), floor_layer_gap_m=floor_layer_gap_m) + 0.12
        to_z = bundle.floor_display_z(transition.get("to_floor_id"), floor_layer_gap_m=floor_layer_gap_m) + 0.12
        transition_id = normalized.get("transition_id") or f"vertical_transition_{normalized['index']}"
        endpoint_points = [("from", from_xy, from_z), ("to", to_xy, to_z)]
        for endpoint_name, xy_value, z in endpoint_points:
            if xy_value is None:
                continue
            marker = _base_marker(
                marker_id=f"{transition_id}_{endpoint_name}",
                group="vertical_transitions",
                marker_type="SPHERE",
                frame_id=frame_id,
                namespace="vertical_transitions/endpoints",
                source_artifact=SNAPSHOT_FILENAME,
                source_ids={
                    "transition_id": transition_id,
                    "endpoint": endpoint_name,
                    "room_a": normalized.get("room_a"),
                    "room_b": normalized.get("room_b"),
                },
                floor_id=transition.get(f"{endpoint_name}_floor_id"),
                display_floor_id=transition.get(f"{endpoint_name}_display_floor_id"),
                label=f"{transition_id} {endpoint_name}",
                text="Vertical transition endpoint marker only; not a traversal command.",
                visualization_only=True,
                not_collision_free=True,
                color={"r": 0.72, "g": 0.32, "b": 0.94, "a": 0.95},
                scale={"x": 0.20, "y": 0.20, "z": 0.10},
            )
            marker["position"] = _point_dict(xy_value[0], xy_value[1], z)
            groups["vertical_transitions"].append(marker)
        if from_xy is not None and to_xy is not None:
            marker = _base_marker(
                marker_id=f"{transition_id}_connector",
                group="vertical_transitions",
                marker_type="LINE_LIST",
                frame_id=frame_id,
                namespace="vertical_transitions/connectors",
                source_artifact=SNAPSHOT_FILENAME,
                source_ids={"transition_id": transition_id},
                floor_id=transition.get("from_floor_id"),
                display_floor_id=transition.get("from_display_floor_id"),
                label=f"{transition_id} connector",
                text="Cross-floor topology marker only.",
                visualization_only=True,
                not_collision_free=True,
                color={"r": 0.72, "g": 0.32, "b": 0.94, "a": 0.75},
                scale={"x": 0.035, "y": 0.035, "z": 0.035},
            )
            marker["points"] = [
                _point_dict(from_xy[0], from_xy[1], from_z),
                _point_dict(to_xy[0], to_xy[1], to_z),
            ]
            groups["vertical_transitions"].append(marker)

    for idx, obj in enumerate(bundle.snapshot_objects):
        z_default = bundle.floor_display_z(obj.get("floor_id"), floor_layer_gap_m=floor_layer_gap_m) + 0.10
        point = _xyz(obj.get("pose_3d"), default_z=z_default) or _xyz(obj.get("pose"), default_z=z_default)
        if point is None:
            continue
        object_id = normalize_object_id(obj.get("id")) or str(obj.get("id") or idx)
        marker = _base_marker(
            marker_id=f"object_{object_id}",
            group="objects",
            marker_type="SPHERE",
            frame_id=frame_id,
            namespace=f"objects/{obj.get('display_floor_id') or obj.get('floor_id') or 'unknown_floor'}",
            source_artifact=SNAPSHOT_FILENAME,
            source_ids={"object_id": object_id, "room_id": normalize_room_id(obj.get("room_id"))},
            floor_id=obj.get("floor_id"),
            display_floor_id=obj.get("display_floor_id"),
            label=str(obj.get("label") or obj.get("category") or object_id),
            text="Object semantic marker from committed snapshot.",
            visualization_only=True,
            not_collision_free=True,
            color={"r": 0.90, "g": 0.42, "b": 0.48, "a": 0.85},
            scale={"x": 0.10, "y": 0.10, "z": 0.10},
        )
        marker["position"] = _point_dict(point[0], point[1], point[2])
        groups["objects"].append(marker)

    for idx, anchor in enumerate(bundle.snapshot_anchors):
        z_default = bundle.floor_display_z(anchor.get("floor_id"), floor_layer_gap_m=floor_layer_gap_m) + 0.12
        point = _xyz(anchor.get("position"), default_z=z_default)
        if point is None:
            continue
        anchor_id = str(anchor.get("id") or f"anchor_{idx}")
        marker = _base_marker(
            marker_id=f"anchor_{anchor_id}",
            group="anchors",
            marker_type="SPHERE",
            frame_id=frame_id,
            namespace=f"anchors/{anchor.get('display_floor_id') or anchor.get('floor_id') or 'unknown_floor'}",
            source_artifact=SNAPSHOT_FILENAME,
            source_ids={"anchor_id": anchor_id, "room_id": normalize_room_id(anchor.get("room_id"))},
            floor_id=anchor.get("floor_id"),
            display_floor_id=anchor.get("display_floor_id"),
            label=anchor_id,
            text="Anchor marker from committed snapshot.",
            visualization_only=True,
            not_collision_free=True,
            color={"r": 0.20, "g": 0.82, "b": 0.78, "a": 0.85},
            scale={"x": 0.12, "y": 0.12, "z": 0.08},
        )
        marker["position"] = _point_dict(point[0], point[1], point[2])
        groups["anchors"].append(marker)

    if route_waypoints:
        _append_route_marker_specs(groups, route_waypoints, frame_id=frame_id)

    return {
        "frame_id": frame_id,
        "scene_id": bundle.scene_id,
        "groups": groups,
        "counts": {group: len(markers) for group, markers in groups.items()},
        "paper_safety_label": PAPER_SAFETY_LABEL,
        "warnings": validate_coordinates(bundle),
    }


def _append_route_marker_specs(
    groups: Dict[str, List[Dict[str, Any]]],
    route_waypoints: Mapping[str, Any],
    *,
    frame_id: str,
) -> None:
    waypoints = [dict(item) for item in route_waypoints.get("waypoints") or [] if isinstance(item, Mapping)]
    points: List[Dict[str, float]] = []
    for waypoint in waypoints:
        position = waypoint.get("position")
        if isinstance(position, Mapping):
            x = _as_float(position.get("x"))
            y = _as_float(position.get("y"))
            z = _as_float(position.get("z")) or 0.0
            if x is not None and y is not None:
                points.append(_point_dict(x, y, z + 0.08))
    if len(points) >= 2:
        marker = _base_marker(
            marker_id="selected_route_line",
            group="route",
            marker_type="LINE_STRIP",
            frame_id=frame_id,
            namespace="route/topological_demo",
            source_artifact="route_to_waypoints",
            source_ids={"waypoint_count": len(points)},
            label="selected route",
            text="Committed room-level route visualized as topological/demo waypoints.",
            visualization_only=True,
            not_collision_free=True,
            color={"r": 1.0, "g": 0.22, "b": 0.12, "a": 0.95},
            scale={"x": 0.06, "y": 0.06, "z": 0.06},
        )
        marker["points"] = points
        groups["route"].append(marker)

    for waypoint in waypoints:
        position = waypoint.get("position")
        if not isinstance(position, Mapping):
            continue
        x = _as_float(position.get("x"))
        y = _as_float(position.get("y"))
        z = _as_float(position.get("z")) or 0.0
        if x is None or y is None:
            continue
        waypoint_id = str(waypoint.get("waypoint_id") or f"waypoint_{waypoint.get('index')}")
        marker = _base_marker(
            marker_id=waypoint_id,
            group="waypoints",
            marker_type="SPHERE",
            frame_id=frame_id,
            namespace="waypoints/topological_demo",
            source_artifact=str((waypoint.get("source") or {}).get("artifact") or "route_to_waypoints"),
            source_ids=dict((waypoint.get("source") or {}).get("source_ids") or {}),
            floor_id=(waypoint.get("floor") or {}).get("floor_id"),
            display_floor_id=(waypoint.get("floor") or {}).get("display_floor_id"),
            label=waypoint_id,
            text=str(waypoint.get("paper_safety_label") or PAPER_SAFETY_LABEL),
            visualization_only=True,
            not_collision_free=True,
            color={"r": 1.0, "g": 0.72, "b": 0.14, "a": 0.98},
            scale={"x": 0.18, "y": 0.18, "z": 0.10},
        )
        marker["position"] = _point_dict(x, y, z + 0.12)
        groups["waypoints"].append(marker)

