import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx
import numpy as np


DEFAULT_LABEL_NORMALIZATION: Dict[str, str] = {
    "couch": "sofa",
    "loveseat": "sofa",
    "coffee_table": "table",
    "dining_table": "table",
    "side_table": "table",
    "bedside_table": "nightstand",
    "night_stand": "nightstand",
    "bed_side_table": "nightstand",
    "desk_table": "desk",
    "work_table": "desk",
    "bookshelf": "shelf",
    "book_shelf": "shelf",
    "tv_stand": "cabinet",
    "trash bin": "trash_can",
    "garbage_bin": "trash_can",
}

DEFAULT_SEMANTIC_FUSION_CONFIG: Dict[str, float] = {
    "min_observation_weight": 0.05,
    "semantic_gap_scale": 0.30,
    "semantic_stability_floor": 0.0,
}

DEFAULT_ANCHOR_CONFIG: Dict[str, float] = {
    "room_anchor_wall_clearance": 0.35,
    "room_anchor_obstacle_clearance": 0.30,
    "room_anchor_search_step": 0.25,
    "room_anchor_search_rings": 4,
    "room_anchor_search_angles": 16,
    "object_anchor_clearance": 0.45,
    "object_anchor_safety_margin": 0.15,
    "object_anchor_extent_scale": 0.25,
    "object_anchor_preferred_distance": 0.75,
    "object_anchor_object_clearance": 0.20,
    "object_anchor_wall_clearance": 0.20,
    "anchor_candidate_line_samples": 6,
    "clearance_score_scale": 1.20,
    "interiority_weight": 0.35,
    "clearance_weight": 0.40,
    "preferred_distance_weight": 0.25,
    "object_anchor_min_obs": 1,
    "object_anchor_min_stability": 0.20,
}

DEFAULT_LANDMARK_RULES: Dict[str, set[str]] = {
    "categories": {"furniture", "appliance", "fixture", "landmark"},
    "labels": {
        "bed",
        "cabinet",
        "chair",
        "couch",
        "desk",
        "door",
        "dresser",
        "fridge",
        "microwave",
        "nightstand",
        "oven",
        "shelf",
        "sink",
        "sofa",
        "stove",
        "table",
        "television",
        "toilet",
        "tv",
        "wardrobe",
        "washing_machine",
    },
    "excluded_categories": {"food", "utensil", "decor", "small_object", "clutter"},
    "excluded_labels": {
        "apple",
        "bottle",
        "book",
        "cup",
        "keyboard",
        "mouse",
        "mug",
        "plant",
        "remote",
    },
}


def _merge_config(defaults: Dict[str, Any], override: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    merged = dict(defaults)
    if override:
        merged.update(override)
    return merged


def _merge_rule_sets(defaults: Dict[str, set[str]], override: Optional[Dict[str, Iterable[str]]]) -> Dict[str, set[str]]:
    merged = {key: set(values) for key, values in defaults.items()}
    if not override:
        return merged
    for key, values in override.items():
        merged[key] = {str(v).strip().lower() for v in values}
    return merged


def _sanitize_token(value: Optional[str]) -> str:
    if value is None:
        return "unknown"
    text = str(value).strip().lower()
    if not text:
        return "unknown"
    return text.replace(" ", "_")


def normalize_open_vocab_label(label: Optional[str], mapping: Optional[Dict[str, str]] = None) -> str:
    normalized = _sanitize_token(label)
    lookup = mapping or DEFAULT_LABEL_NORMALIZATION
    return lookup.get(normalized, normalized)


def _to_numpy_vector(value: Optional[Sequence[float]]) -> Optional[np.ndarray]:
    if value is None:
        return None
    array = np.asarray(value, dtype=np.float32)
    if array.size == 0:
        return None
    return array


def _polygon_signed_area(polygon: Sequence[Tuple[float, float]]) -> float:
    if len(polygon) < 3:
        return 0.0
    pts = np.asarray(polygon, dtype=np.float32)
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _polygon_centroid(polygon: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    if not polygon:
        return (0.0, 0.0)
    pts = np.asarray(polygon, dtype=np.float32)
    if len(pts) < 3:
        return (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
    signed_area = _polygon_signed_area(polygon)
    if abs(signed_area) < 1e-6:
        return (float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])))
    factor = 1.0 / (6.0 * signed_area)
    cross = pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]
    cx = factor * np.sum((pts[:, 0] + np.roll(pts[:, 0], -1)) * cross)
    cy = factor * np.sum((pts[:, 1] + np.roll(pts[:, 1], -1)) * cross)
    return (float(cx), float(cy))


def _point_in_polygon(point_xy: Sequence[float], polygon: Sequence[Tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False
    x, y = float(point_xy[0]), float(point_xy[1])
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-8) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _distance_point_to_segment(point_xy: Sequence[float], start_xy: Sequence[float], end_xy: Sequence[float]) -> float:
    point = np.asarray(point_xy, dtype=np.float32)
    start = np.asarray(start_xy, dtype=np.float32)
    end = np.asarray(end_xy, dtype=np.float32)
    segment = end - start
    denom = float(np.dot(segment, segment))
    if denom < 1e-8:
        return float(np.linalg.norm(point - start))
    t = float(np.dot(point - start, segment) / denom)
    t = max(0.0, min(1.0, t))
    projection = start + t * segment
    return float(np.linalg.norm(point - projection))


def _distance_point_to_polygon_boundary(point_xy: Sequence[float], polygon: Sequence[Tuple[float, float]]) -> float:
    if len(polygon) < 2:
        return 0.0
    return min(
        _distance_point_to_segment(point_xy, polygon[idx], polygon[(idx + 1) % len(polygon)])
        for idx in range(len(polygon))
    )


def _segment_inside_polygon(start_xy: Sequence[float], end_xy: Sequence[float], polygon: Sequence[Tuple[float, float]], samples: int) -> bool:
    if len(polygon) < 3:
        return False
    start = np.asarray(start_xy, dtype=np.float32)
    end = np.asarray(end_xy, dtype=np.float32)
    for alpha in np.linspace(0.1, 1.0, max(samples, 2)):
        sample = start + alpha * (end - start)
        if not _point_in_polygon(sample, polygon):
            return False
    return True


def _safe_unit(vector_xy: Sequence[float]) -> np.ndarray:
    vector = np.asarray(vector_xy, dtype=np.float32)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        return np.zeros(2, dtype=np.float32)
    return vector / norm


def _build_box_footprint(center_xyz: Tuple[float, float, float], bbox_xyz: Tuple[float, float, float], yaw: float = 0.0) -> List[Tuple[float, float]]:
    dx, dy = float(bbox_xyz[0]), float(bbox_xyz[1])
    local = np.array(
        [[dx / 2.0, dy / 2.0], [-dx / 2.0, dy / 2.0], [-dx / 2.0, -dy / 2.0], [dx / 2.0, -dy / 2.0]],
        dtype=np.float32,
    )
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))
    rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float32)
    rotated = (rotation @ local.T).T + np.asarray(center_xyz[:2], dtype=np.float32)
    return [tuple(map(float, pt)) for pt in rotated]


@dataclass
class SemanticObservation:
    label: Optional[str]
    category: Optional[str]
    clip_feature: Optional[Sequence[float]] = None
    detection_confidence: float = 1.0
    semantic_confidence: Optional[float] = None
    association_confidence: float = 1.0
    semantic_gap: Optional[float] = None
    view_quality: Optional[float] = None
    raw_label: Optional[str] = None
    raw_category: Optional[str] = None

    def normalized_label(self, mapping: Dict[str, str]) -> str:
        return normalize_open_vocab_label(self.label, mapping)

    def normalized_category(self) -> str:
        return _sanitize_token(self.category)

    def fusion_weight(self, cfg: Dict[str, float]) -> float:
        semantic_conf = self.semantic_confidence
        if semantic_conf is None:
            semantic_conf = self.detection_confidence
        gap_term = 1.0
        if self.semantic_gap is not None:
            gap_term += min(max(float(self.semantic_gap), 0.0), cfg["semantic_gap_scale"]) / max(
                cfg["semantic_gap_scale"], 1e-6
            )
        view_quality = 1.0 if self.view_quality is None else max(0.25, min(1.0, float(self.view_quality)))
        weight = (
            max(0.0, float(self.detection_confidence))
            * max(0.0, float(semantic_conf))
            * max(0.0, float(self.association_confidence))
            * gap_term
            * view_quality
        )
        return max(cfg["min_observation_weight"], float(weight))


@dataclass
class FloorNode:
    """Explicit floor node for the world-graph truth layer."""

    id: str
    floor_index: int
    z_min: float
    z_max: float
    z_center: float
    confidence: float
    status: str
    support_statistics: Dict[str, Any] = field(default_factory=dict)
    node_type: str = "floor"

    def get_attributes(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "node_type": self.node_type,
            "floor_index": int(self.floor_index),
            "z_min": float(self.z_min),
            "z_max": float(self.z_max),
            "z_center": float(self.z_center),
            "confidence": float(self.confidence),
            "status": self.status,
            "support_statistics": dict(self.support_statistics),
        }


@dataclass
class RoomNode:
    """Room node schema used by the VLN scene graph."""

    id: str
    room_type: str
    polygon: List[Tuple[float, float]] = field(default_factory=list)
    center: Optional[Tuple[float, float]] = None
    floor_id: Optional[str] = None
    floor_assignment_confidence: float = 1.0
    status: str = "confirmed"
    node_type: str = "room"

    def __post_init__(self) -> None:
        self.polygon = [tuple(map(float, pt)) for pt in self.polygon]
        if self.center is None:
            self.center = _polygon_centroid(self.polygon) if self.polygon else (0.0, 0.0)
        else:
            self.center = tuple(map(float, self.center))

    def get_attributes(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "node_type": self.node_type,
            "room_type": self.room_type,
            "polygon": self.polygon,
            "center": list(self.center),
            "floor_id": self.floor_id,
            "floor_assignment_confidence": float(self.floor_assignment_confidence),
            "status": self.status,
        }


@dataclass
class AnchorNode:
    """Lightweight navigable anchor for rooms or objects."""

    id: str
    anchor_type: str
    position: Tuple[float, float, float]
    room_id: str
    target_id: str
    valid: bool
    score: float
    floor_id: Optional[str] = None
    candidate_positions: Optional[List[Tuple[float, float]]] = None
    node_type: str = "anchor"

    def get_attributes(self) -> Dict[str, Any]:
        attrs: Dict[str, Any] = {
            "id": self.id,
            "node_type": self.node_type,
            "anchor_type": self.anchor_type,
            "position": [float(v) for v in self.position],
            "room_id": self.room_id,
            "target_id": self.target_id,
            "valid": bool(self.valid),
            "score": float(self.score),
            "floor_id": self.floor_id,
        }
        if self.candidate_positions:
            attrs["candidate_positions"] = [list(map(float, pt)) for pt in self.candidate_positions]
        return attrs


@dataclass
class ObjectNode:
    """Object node with lightweight multi-observation semantic fusion state."""

    id: str
    center: Tuple[float, float, float]
    bbox: Tuple[float, float, float]
    label: str
    category: str
    clip_feature: Optional[Sequence[float]]
    confidence: float
    room_id: str
    floor_id: Optional[str] = None
    yaw: float = 0.0
    footprint: Optional[List[Tuple[float, float]]] = None
    semantic_observations: Optional[List[Any]] = None
    detection_confidence: Optional[float] = None
    semantic_confidence: Optional[float] = None
    association_confidence: float = 1.0
    semantic_gap: Optional[float] = None
    view_quality: Optional[float] = None
    node_type: str = "object"
    label_normalization: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_LABEL_NORMALIZATION), repr=False)

    def __post_init__(self) -> None:
        self.center = tuple(float(v) for v in self.center)
        self.bbox = tuple(float(v) for v in self.bbox)
        self.yaw = float(self.yaw)
        self.clip_feature = _to_numpy_vector(self.clip_feature)
        if self.detection_confidence is None:
            self.detection_confidence = float(self.confidence)
        if self.semantic_confidence is None:
            self.semantic_confidence = float(self.confidence)

        self.canonical_label = normalize_open_vocab_label(self.label, self.label_normalization)
        self.canonical_category = _sanitize_token(self.category)
        self.label_scores: Dict[str, float] = {}
        self.category_scores: Dict[str, float] = {}
        self.raw_label_scores: Dict[str, float] = {}
        self.obs_count = 0
        self.semantic_stability = 0.0
        self.clip_feature_fused: Optional[np.ndarray] = None
        self._fused_feature_weight = 0.0
        self._total_semantic_weight = 0.0
        self._refresh_geometry()
        self._initialize_semantic_state()

    def _refresh_geometry(self) -> None:
        self._center_np = np.asarray(self.center, dtype=np.float32)
        self._bbox_np = np.asarray(self.bbox, dtype=np.float32)
        half_size = self._bbox_np / 2.0
        self.min_pt = self._center_np - half_size
        self.max_pt = self._center_np + half_size
        if self.footprint:
            self.footprint = [tuple(map(float, pt)) for pt in self.footprint]
        else:
            self.footprint = _build_box_footprint(self.center, self.bbox, self.yaw)

    def _initial_observation(self) -> SemanticObservation:
        return SemanticObservation(
            label=self.label,
            category=self.category,
            clip_feature=self.clip_feature,
            detection_confidence=float(self.detection_confidence or self.confidence),
            semantic_confidence=float(self.semantic_confidence or self.confidence),
            association_confidence=float(self.association_confidence),
            semantic_gap=self.semantic_gap,
            view_quality=self.view_quality,
            raw_label=self.label,
            raw_category=self.category,
        )

    def _initialize_semantic_state(self) -> None:
        self.label_scores.clear()
        self.category_scores.clear()
        self.raw_label_scores.clear()
        self.obs_count = 0
        self.semantic_stability = 0.0
        self.clip_feature_fused = None
        self._fused_feature_weight = 0.0
        self._total_semantic_weight = 0.0
        observations = self.semantic_observations or [self._initial_observation()]
        for obs in observations:
            self.fuse_observation(obs)

    def update_geometry(
        self,
        center: Optional[Tuple[float, float, float]] = None,
        bbox: Optional[Tuple[float, float, float]] = None,
        yaw: Optional[float] = None,
        footprint: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        if center is not None:
            self.center = tuple(float(v) for v in center)
        if bbox is not None:
            self.bbox = tuple(float(v) for v in bbox)
        if yaw is not None:
            self.yaw = float(yaw)
        if footprint is not None:
            self.footprint = [tuple(map(float, pt)) for pt in footprint]
        self._refresh_geometry()

    def _to_observation(self, observation: Any) -> SemanticObservation:
        if isinstance(observation, SemanticObservation):
            return observation
        if isinstance(observation, dict):
            return SemanticObservation(**observation)
        raise TypeError(f"Unsupported observation type: {type(observation)!r}")

    def fuse_observation(self, observation: Any, cfg: Optional[Dict[str, float]] = None) -> None:
        obs = self._to_observation(observation)
        fusion_cfg = _merge_config(DEFAULT_SEMANTIC_FUSION_CONFIG, cfg)
        weight = obs.fusion_weight(fusion_cfg)
        normalized_label = obs.normalized_label(self.label_normalization)
        normalized_category = obs.normalized_category()
        raw_label = _sanitize_token(obs.raw_label or obs.label)

        self.label_scores[normalized_label] = self.label_scores.get(normalized_label, 0.0) + weight
        self.category_scores[normalized_category] = self.category_scores.get(normalized_category, 0.0) + weight
        self.raw_label_scores[raw_label] = self.raw_label_scores.get(raw_label, 0.0) + weight

        feature = _to_numpy_vector(obs.clip_feature)
        if feature is not None:
            if self.clip_feature_fused is None:
                self.clip_feature_fused = feature.astype(np.float32)
                self._fused_feature_weight = weight
            else:
                total_weight = self._fused_feature_weight + weight
                self.clip_feature_fused = (
                    (self.clip_feature_fused * self._fused_feature_weight) + feature * weight
                ) / max(total_weight, 1e-6)
                self._fused_feature_weight = total_weight

        self.obs_count += 1
        self._total_semantic_weight += weight
        self.canonical_label = max(self.label_scores.items(), key=lambda item: item[1])[0]
        self.canonical_category = max(self.category_scores.items(), key=lambda item: item[1])[0]
        self._update_semantic_stability(fusion_cfg)
        self.label = self.canonical_label
        self.category = self.canonical_category
        self.clip_feature = self.clip_feature_fused
        semantic_conf = obs.semantic_confidence
        if semantic_conf is None:
            semantic_conf = obs.detection_confidence
        self.semantic_confidence = float(semantic_conf)
        self.detection_confidence = float(obs.detection_confidence)
        self.association_confidence = float(obs.association_confidence)
        self.confidence = float(self.semantic_confidence)

    def _update_semantic_stability(self, cfg: Dict[str, float]) -> None:
        sorted_scores = sorted(self.label_scores.values(), reverse=True)
        if not sorted_scores:
            self.semantic_stability = cfg["semantic_stability_floor"]
            return
        top1 = sorted_scores[0]
        top2 = sorted_scores[1] if len(sorted_scores) > 1 else 0.0
        total = max(sum(sorted_scores), 1e-6)
        margin = (top1 - top2) / total
        self.semantic_stability = max(cfg["semantic_stability_floor"], float(margin))

    def fuse_from_object(self, other: "ObjectNode", cfg: Optional[Dict[str, float]] = None) -> None:
        self.update_geometry(center=other.center, bbox=other.bbox, yaw=other.yaw, footprint=other.footprint)
        observations = other.semantic_observations or [other._initial_observation()]
        for observation in observations:
            self.fuse_observation(observation, cfg=cfg)
        if other.room_id:
            self.room_id = other.room_id
        if other.floor_id:
            self.floor_id = other.floor_id

    def get_attributes(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "node_type": self.node_type,
            "center": list(self.center),
            "bbox": list(self.bbox),
            "yaw": float(self.yaw),
            "footprint": [list(map(float, pt)) for pt in (self.footprint or [])],
            "label": self.label,
            "category": self.category,
            "canonical_label": self.canonical_label,
            "canonical_category": self.canonical_category,
            "label_scores": dict(self.label_scores),
            "category_scores": dict(self.category_scores),
            "raw_label_scores": dict(self.raw_label_scores),
            "obs_count": int(self.obs_count),
            "semantic_stability": float(self.semantic_stability),
            "clip_feature": self.clip_feature_fused,
            "clip_feature_fused": self.clip_feature_fused,
            "confidence": float(self.confidence),
            "detection_confidence": float(self.detection_confidence or self.confidence),
            "semantic_confidence": float(self.semantic_confidence or self.confidence),
            "association_confidence": float(self.association_confidence),
            "room_id": self.room_id,
            "floor_id": self.floor_id,
        }


class SemanticSceneGraph:
    """Navigation-oriented semantic scene graph with fused object semantics and anchor nodes."""

    def __init__(
        self,
        semantic_fusion_config: Optional[Dict[str, Any]] = None,
        anchor_config: Optional[Dict[str, Any]] = None,
        label_normalization: Optional[Dict[str, str]] = None,
        landmark_rules: Optional[Dict[str, Iterable[str]]] = None,
    ):
        self.graph = nx.MultiDiGraph()
        self.semantic_fusion_config = _merge_config(DEFAULT_SEMANTIC_FUSION_CONFIG, semantic_fusion_config)
        self.anchor_config = _merge_config(DEFAULT_ANCHOR_CONFIG, anchor_config)
        self.label_normalization = dict(DEFAULT_LABEL_NORMALIZATION)
        if label_normalization:
            self.label_normalization.update(
                {normalize_open_vocab_label(key): normalize_open_vocab_label(value) for key, value in label_normalization.items()}
            )
        self.landmark_rules = _merge_rule_sets(DEFAULT_LANDMARK_RULES, landmark_rules)
        self.objects: List[ObjectNode] = []
        self.object_index: Dict[str, ObjectNode] = {}
        self.floors: Dict[str, FloorNode] = {}
        self.rooms: Dict[str, RoomNode] = {}
        self.anchors: Dict[str, AnchorNode] = {}
        self._anchor_profile: Dict[str, float] = {}
        self.reset_anchor_profile()

    def reset_anchor_profile(self) -> None:
        self._anchor_profile = {
            "room_anchor_total_sec": 0.0,
            "object_anchor_total_sec": 0.0,
            "candidate_generation_sec": 0.0,
            "validate_sec": 0.0,
            "score_sec": 0.0,
            "fallback_sec": 0.0,
            "insert_sec": 0.0,
            "room_anchor_candidate_count": 0.0,
            "object_anchor_candidate_count": 0.0,
            "valid_candidate_count": 0.0,
            "fallback_candidate_count": 0.0,
        }

    def _accumulate_anchor_profile(self, key: str, value: float) -> None:
        self._anchor_profile[key] = float(self._anchor_profile.get(key, 0.0) or 0.0) + float(value)

    def get_anchor_profile(self) -> Dict[str, float]:
        return dict(self._anchor_profile)

    def add_floor_node(self, floor: FloorNode) -> None:
        self.floors[floor.id] = floor
        self.graph.add_node(floor.id, **floor.get_attributes())

    def add_room_node(self, room: RoomNode) -> None:
        self.rooms[room.id] = room
        self.graph.add_node(room.id, **room.get_attributes())
        if room.floor_id is not None and room.floor_id in self.graph.nodes:
            self._add_relation_edge(room.id, room.floor_id, "ON_FLOOR")

    def _relation_exists(self, source_id: str, target_id: str, relation: str) -> bool:
        edge_data = self.graph.get_edge_data(source_id, target_id, default={})
        if not self.graph.is_multigraph():
            return bool(edge_data) and edge_data.get("relation") == relation
        return any(data.get("relation") == relation for data in edge_data.values())

    def _add_relation_edge(self, source_id: str, target_id: str, relation: str) -> None:
        if not self._relation_exists(source_id, target_id, relation):
            self.graph.add_edge(source_id, target_id, relation=relation)

    def get_relations_between(self, source_id: str, target_id: str) -> List[str]:
        edge_data = self.graph.get_edge_data(source_id, target_id, default={})
        if not edge_data:
            return []
        if not self.graph.is_multigraph():
            relation = edge_data.get("relation")
            return [relation] if relation else []
        return [data["relation"] for data in edge_data.values() if "relation" in data]

    def _sync_object_cache(self) -> None:
        self.objects = list(self.object_index.values())

    def _sync_object_node(self, obj: ObjectNode) -> None:
        self.graph.add_node(obj.id, **obj.get_attributes())
        self._sync_object_cache()

    def add_object_node(self, obj: ObjectNode) -> ObjectNode:
        obj.label_normalization = dict(self.label_normalization)
        if obj.id in self.object_index:
            existing = self.object_index[obj.id]
            existing.fuse_from_object(obj, cfg=self.semantic_fusion_config)
            self._sync_object_node(existing)
            return existing
        obj._initialize_semantic_state()
        self.object_index[obj.id] = obj
        self._sync_object_node(obj)
        if obj.floor_id is not None and obj.floor_id in self.graph.nodes:
            self._add_relation_edge(obj.id, obj.floor_id, "ON_FLOOR")
        return obj

    def fuse_object_observation(
        self,
        object_id: str,
        observation: Any,
        center: Optional[Tuple[float, float, float]] = None,
        bbox: Optional[Tuple[float, float, float]] = None,
        yaw: Optional[float] = None,
        footprint: Optional[List[Tuple[float, float]]] = None,
        room_id: Optional[str] = None,
    ) -> None:
        if object_id not in self.object_index:
            raise KeyError(f"Object node not found: {object_id}")
        obj = self.object_index[object_id]
        obj.fuse_observation(observation, cfg=self.semantic_fusion_config)
        obj.update_geometry(center=center, bbox=bbox, yaw=yaw, footprint=footprint)
        if room_id is not None:
            obj.room_id = room_id
        self._sync_object_node(obj)

    def add_inside_relation(self, object_id: str, room_id: str) -> None:
        if object_id not in self.graph.nodes:
            raise KeyError(f"Object node not found: {object_id}")
        if room_id not in self.graph.nodes:
            raise KeyError(f"Room node not found: {room_id}")
        self._add_relation_edge(object_id, room_id, "INSIDE")

    def add_room(self, room: RoomNode) -> None:
        self.add_room_node(room)

    def add_object(self, obj: ObjectNode) -> None:
        self.add_object_node(obj)

    def compute_spatial_relations(self, dist_threshold: float = 1.0, z_tolerance: float = 0.15) -> None:
        num_objs = len(self.objects)
        for i in range(num_objs):
            for j in range(i + 1, num_objs):
                obj_a = self.objects[i]
                obj_b = self.objects[j]
                if obj_a.room_id != obj_b.room_id:
                    continue
                overlap_x = max(obj_a.min_pt[0], obj_b.min_pt[0]) < min(obj_a.max_pt[0], obj_b.max_pt[0])
                overlap_y = max(obj_a.min_pt[1], obj_b.min_pt[1]) < min(obj_a.max_pt[1], obj_b.max_pt[1])
                xy_overlap = overlap_x and overlap_y

                relation_found = False
                if xy_overlap:
                    if abs(obj_a.min_pt[2] - obj_b.max_pt[2]) < z_tolerance:
                        self._add_relation_edge(obj_a.id, obj_b.id, "ON")
                        self._add_relation_edge(obj_b.id, obj_a.id, "UNDER")
                        relation_found = True
                    elif abs(obj_b.min_pt[2] - obj_a.max_pt[2]) < z_tolerance:
                        self._add_relation_edge(obj_b.id, obj_a.id, "ON")
                        self._add_relation_edge(obj_a.id, obj_b.id, "UNDER")
                        relation_found = True

                if not relation_found:
                    dist_xy = float(np.linalg.norm(obj_a._center_np[:2] - obj_b._center_np[:2]))
                    z_diff = abs(float(obj_a._center_np[2] - obj_b._center_np[2]))
                    if dist_xy < dist_threshold and z_diff < 0.5:
                        self._add_relation_edge(obj_a.id, obj_b.id, "NEXT_TO")
                        self._add_relation_edge(obj_b.id, obj_a.id, "NEXT_TO")

    def _objects_in_room(self, room_id: str) -> List[ObjectNode]:
        return [obj for obj in self.objects if obj.room_id == room_id]

    def _distance_to_objects(self, point_xy: Sequence[float], objects: Sequence[ObjectNode], ignore_ids: Optional[set[str]] = None) -> float:
        ignore_ids = ignore_ids or set()
        min_distance = float("inf")
        for obj in objects:
            if obj.id in ignore_ids:
                continue
            if obj.footprint and _point_in_polygon(point_xy, obj.footprint):
                return 0.0
            distance = _distance_point_to_polygon_boundary(point_xy, obj.footprint or [])
            min_distance = min(min_distance, distance)
        return float(min_distance)

    def validate_anchor(
        self,
        position_xyz: Sequence[float],
        room_id: str,
        target_id: Optional[str] = None,
        target_type: str = "room",
    ) -> Tuple[bool, Dict[str, Any]]:
        room = self.rooms.get(room_id)
        if room is None or len(room.polygon) < 3:
            return False, {"reason": "missing_room_polygon"}  # type: ignore[return-value]

        point_xy = np.asarray(position_xyz[:2], dtype=np.float32)
        room_objects = self._objects_in_room(room_id)
        wall_clearance = _distance_point_to_polygon_boundary(point_xy, room.polygon)
        if not _point_in_polygon(point_xy, room.polygon):
            return False, {"reason": "outside_room", "wall_clearance": wall_clearance}

        min_wall_clearance = self.anchor_config["room_anchor_wall_clearance"]
        if target_type == "object":
            min_wall_clearance = self.anchor_config["object_anchor_wall_clearance"]
        if wall_clearance < min_wall_clearance:
            return False, {"reason": "too_close_to_wall", "wall_clearance": wall_clearance}

        object_clearance = float("inf")
        scoring_object_clearance = float("inf")
        target_object = self.object_index.get(target_id or "")
        for obj in room_objects:
            if obj.footprint is None:
                continue
            if target_type == "room":
                margin = self.anchor_config["room_anchor_obstacle_clearance"]
            elif obj.id == target_id:
                margin = self.anchor_config["object_anchor_object_clearance"]
            else:
                margin = self.anchor_config["object_anchor_clearance"]
            if _point_in_polygon(point_xy, obj.footprint):
                return False, {"reason": "inside_object", "object_id": obj.id}
            distance = _distance_point_to_polygon_boundary(point_xy, obj.footprint)
            object_clearance = min(object_clearance, distance)
            if target_type == "room" or obj.id != target_id:
                scoring_object_clearance = min(scoring_object_clearance, distance)
            if distance < margin:
                reason = "too_close_to_target" if obj.id == target_id else "too_close_to_other_object"
                return False, {"reason": reason, "object_id": obj.id, "object_clearance": distance}

        if target_type == "object" and target_object is not None and target_object.footprint:
            direction_start = np.asarray(_polygon_centroid(target_object.footprint), dtype=np.float32)
            if not _segment_inside_polygon(
                direction_start,
                point_xy,
                room.polygon,
                samples=int(self.anchor_config["anchor_candidate_line_samples"]),
            ):
                return False, {"reason": "crosses_room_boundary"}

        if not math.isfinite(object_clearance):
            object_clearance = wall_clearance
        if not math.isfinite(scoring_object_clearance):
            scoring_object_clearance = wall_clearance

        return True, {
            "wall_clearance": wall_clearance,
            "object_clearance": object_clearance,
            "scoring_object_clearance": scoring_object_clearance,
        }

    def _score_room_anchor(
        self,
        point_xy: np.ndarray,
        room: RoomNode,
        room_objects: Sequence[ObjectNode],
        *,
        wall_clearance: Optional[float] = None,
        obstacle_clearance: Optional[float] = None,
    ) -> float:
        if wall_clearance is None:
            wall_clearance = _distance_point_to_polygon_boundary(point_xy, room.polygon)
        if obstacle_clearance is None:
            obstacle_clearance = self._distance_to_objects(point_xy, room_objects)
        if not math.isfinite(obstacle_clearance):
            obstacle_clearance = wall_clearance
        center = np.asarray(room.center or _polygon_centroid(room.polygon), dtype=np.float32)
        center_bias = 1.0 / (1.0 + float(np.linalg.norm(point_xy - center)))
        return float(0.45 * wall_clearance + 0.35 * obstacle_clearance + 0.20 * center_bias)

    def _score_clearance(self, point_xy: np.ndarray, room: RoomNode, room_objects: Sequence[ObjectNode], ignore_ids: Optional[set[str]] = None) -> float:
        wall_clearance = _distance_point_to_polygon_boundary(point_xy, room.polygon)
        object_clearance = self._distance_to_objects(point_xy, room_objects, ignore_ids=ignore_ids)
        return self._clearance_score_from_distances(
            wall_clearance=wall_clearance,
            object_clearance=object_clearance,
        )

    def _clearance_score_from_distances(self, *, wall_clearance: float, object_clearance: float) -> float:
        min_clearance = min(wall_clearance, object_clearance)
        return min(1.0, max(0.0, min_clearance / self.anchor_config["clearance_score_scale"]))

    def _score_interiority(self, candidate_xy: np.ndarray, object_xy: np.ndarray, room_center_xy: np.ndarray) -> float:
        outward = _safe_unit(candidate_xy - object_xy)
        inward = _safe_unit(room_center_xy - object_xy)
        alignment = float(np.dot(outward, inward))
        return max(0.0, 0.5 * (alignment + 1.0))

    def _score_preferred_distance(self, candidate_xy: np.ndarray, object_xy: np.ndarray) -> float:
        actual = float(np.linalg.norm(candidate_xy - object_xy))
        preferred = self.anchor_config["object_anchor_preferred_distance"]
        error = abs(actual - preferred)
        return max(0.0, 1.0 - error / max(preferred, 1e-6))

    def _object_anchor_candidate_offset(self, obj: ObjectNode) -> float:
        footprint_extent = 0.5 * min(float(obj.bbox[0]), float(obj.bbox[1]))
        return (
            self.anchor_config["object_anchor_clearance"]
            + self.anchor_config["object_anchor_safety_margin"]
            + self.anchor_config["object_anchor_extent_scale"] * footprint_extent
        )

    def _generate_side_candidates(self, obj: ObjectNode) -> List[Tuple[np.ndarray, np.ndarray]]:
        if not obj.footprint or len(obj.footprint) < 4:
            obj.footprint = _build_box_footprint(obj.center, obj.bbox, obj.yaw)
        polygon = obj.footprint[:4]
        orientation = _polygon_signed_area(polygon)
        candidates: List[Tuple[np.ndarray, np.ndarray]] = []
        offset = self._object_anchor_candidate_offset(obj)
        for idx in range(4):
            start = np.asarray(polygon[idx], dtype=np.float32)
            end = np.asarray(polygon[(idx + 1) % 4], dtype=np.float32)
            edge = end - start
            midpoint = 0.5 * (start + end)
            if abs(orientation) < 1e-8:
                normal = _safe_unit(midpoint - np.asarray(obj.center[:2], dtype=np.float32))
            elif orientation > 0:
                normal = _safe_unit(np.array([edge[1], -edge[0]], dtype=np.float32))
            else:
                normal = _safe_unit(np.array([-edge[1], edge[0]], dtype=np.float32))
            if float(np.linalg.norm(normal)) < 1e-8:
                normal = _safe_unit(midpoint - np.asarray(obj.center[:2], dtype=np.float32))
            candidate = midpoint + normal * offset
            candidates.append((candidate, midpoint))
        return candidates

    def _fallback_object_anchor(self, obj: ObjectNode, room: RoomNode, room_objects: Sequence[ObjectNode]) -> Optional[AnchorNode]:
        object_xy = np.asarray(obj.center[:2], dtype=np.float32)
        room_center = np.asarray(room.center or _polygon_centroid(room.polygon), dtype=np.float32)
        direction = _safe_unit(room_center - object_xy)
        if float(np.linalg.norm(direction)) < 1e-8:
            direction = np.array([1.0, 0.0], dtype=np.float32)
        base_distance = 0.5 * max(float(obj.bbox[0]), float(obj.bbox[1])) + self.anchor_config["object_anchor_preferred_distance"]
        best_anchor: Optional[AnchorNode] = None
        for scale in (1.0, 1.3, 1.6):
            candidate_xy = object_xy + direction * base_distance * scale
            self._accumulate_anchor_profile("fallback_candidate_count", 1.0)
            valid, _ = self.validate_anchor((candidate_xy[0], candidate_xy[1], 0.0), room.id, obj.id, "object")
            if not valid:
                continue
            score = self._score_object_anchor_candidate(candidate_xy, obj, room, room_objects)
            semantic_gate = min(1.0, max(0.25, obj.semantic_stability))
            anchor = AnchorNode(
                id=f"anchor_{obj.id}",
                anchor_type="object",
                position=(float(candidate_xy[0]), float(candidate_xy[1]), 0.0),
                room_id=room.id,
                target_id=obj.id,
                valid=True,
                score=float(score * semantic_gate),
                floor_id=obj.floor_id or room.floor_id,
            )
            if best_anchor is None or anchor.score > best_anchor.score:
                best_anchor = anchor
        return best_anchor

    def _score_object_anchor_candidate(
        self,
        candidate_xy: np.ndarray,
        obj: ObjectNode,
        room: RoomNode,
        room_objects: Sequence[ObjectNode],
        *,
        clearance_score: Optional[float] = None,
    ) -> float:
        room_center = np.asarray(room.center or _polygon_centroid(room.polygon), dtype=np.float32)
        object_xy = np.asarray(obj.center[:2], dtype=np.float32)
        if clearance_score is None:
            clearance_score = self._score_clearance(candidate_xy, room, room_objects, ignore_ids={obj.id})
        interiority_score = self._score_interiority(candidate_xy, object_xy, room_center)
        distance_score = self._score_preferred_distance(candidate_xy, object_xy)
        return float(
            self.anchor_config["clearance_weight"] * clearance_score
            + self.anchor_config["interiority_weight"] * interiority_score
            + self.anchor_config["preferred_distance_weight"] * distance_score
        )

    def is_anchor_worthy_object(self, obj: ObjectNode) -> bool:
        canonical_label = normalize_open_vocab_label(obj.canonical_label, self.label_normalization)
        canonical_category = _sanitize_token(obj.canonical_category)
        if canonical_category in self.landmark_rules["excluded_categories"]:
            return False
        if canonical_label in self.landmark_rules["excluded_labels"]:
            return False
        if canonical_category in self.landmark_rules["categories"]:
            return True
        return canonical_label in self.landmark_rules["labels"]

    def generate_room_anchor(self, room_id: str, debug: bool = False) -> Optional[AnchorNode]:
        room_t0 = time.perf_counter()
        room = self.rooms.get(room_id)
        if room is None or len(room.polygon) < 3:
            return None
        room_objects = self._objects_in_room(room_id)
        centroid = np.asarray(room.center or _polygon_centroid(room.polygon), dtype=np.float32)
        candidates: List[np.ndarray] = [centroid]
        for ring in range(1, int(self.anchor_config["room_anchor_search_rings"]) + 1):
            radius = ring * self.anchor_config["room_anchor_search_step"]
            for angle_idx in range(int(self.anchor_config["room_anchor_search_angles"])):
                angle = (2.0 * math.pi * angle_idx) / self.anchor_config["room_anchor_search_angles"]
                offset = np.array([math.cos(angle), math.sin(angle)], dtype=np.float32) * radius
                candidates.append(centroid + offset)
        self._accumulate_anchor_profile("room_anchor_candidate_count", float(len(candidates)))

        best_anchor: Optional[AnchorNode] = None
        debug_candidates: List[Tuple[float, float]] = []
        for candidate_xy in candidates:
            validate_t0 = time.perf_counter()
            valid, details = self.validate_anchor((candidate_xy[0], candidate_xy[1], 0.0), room_id, target_type="room")
            self._accumulate_anchor_profile("validate_sec", time.perf_counter() - validate_t0)
            debug_candidates.append((float(candidate_xy[0]), float(candidate_xy[1])))
            if not valid:
                continue
            self._accumulate_anchor_profile("valid_candidate_count", 1.0)
            score_t0 = time.perf_counter()
            score = self._score_room_anchor(
                candidate_xy,
                room,
                room_objects,
                wall_clearance=float(details["wall_clearance"]),
                obstacle_clearance=float(details["scoring_object_clearance"]),
            )
            self._accumulate_anchor_profile("score_sec", time.perf_counter() - score_t0)
            anchor = AnchorNode(
                id=f"anchor_{room_id}",
                anchor_type="room",
                position=(float(candidate_xy[0]), float(candidate_xy[1]), 0.0),
                room_id=room_id,
                target_id=room_id,
                valid=True,
                score=score,
                floor_id=room.floor_id,
                candidate_positions=debug_candidates if debug else None,
            )
            if best_anchor is None or anchor.score > best_anchor.score:
                best_anchor = anchor
        self._accumulate_anchor_profile("room_anchor_total_sec", time.perf_counter() - room_t0)
        return best_anchor

    def generate_object_anchor(self, object_id: str, debug: bool = False) -> Optional[AnchorNode]:
        object_t0 = time.perf_counter()
        obj = self.object_index.get(object_id)
        if obj is None:
            return None
        room = self.rooms.get(obj.room_id)
        if room is None or len(room.polygon) < 3:
            return None
        if not self.is_anchor_worthy_object(obj):
            return None
        if obj.obs_count < int(self.anchor_config["object_anchor_min_obs"]) and obj.semantic_stability < self.anchor_config["object_anchor_min_stability"]:
            return None

        room_objects = self._objects_in_room(room.id)
        best_anchor: Optional[AnchorNode] = None
        debug_candidates: List[Tuple[float, float]] = []
        semantic_gate = min(1.0, max(0.25, obj.semantic_stability))
        if obj.obs_count <= 1:
            semantic_gate *= 0.85

        candidate_gen_t0 = time.perf_counter()
        side_candidates = self._generate_side_candidates(obj)
        self._accumulate_anchor_profile("candidate_generation_sec", time.perf_counter() - candidate_gen_t0)
        self._accumulate_anchor_profile("object_anchor_candidate_count", float(len(side_candidates)))
        for candidate_xy, boundary_midpoint in side_candidates:
            debug_candidates.append((float(candidate_xy[0]), float(candidate_xy[1])))
            if not _segment_inside_polygon(
                boundary_midpoint,
                candidate_xy,
                room.polygon,
                samples=int(self.anchor_config["anchor_candidate_line_samples"]),
            ):
                continue
            validate_t0 = time.perf_counter()
            valid, details = self.validate_anchor((candidate_xy[0], candidate_xy[1], 0.0), room.id, obj.id, "object")
            self._accumulate_anchor_profile("validate_sec", time.perf_counter() - validate_t0)
            if not valid:
                continue
            self._accumulate_anchor_profile("valid_candidate_count", 1.0)
            score_t0 = time.perf_counter()
            score = self._score_object_anchor_candidate(
                candidate_xy,
                obj,
                room,
                room_objects,
                clearance_score=self._clearance_score_from_distances(
                    wall_clearance=float(details["wall_clearance"]),
                    object_clearance=float(details["scoring_object_clearance"]),
                ),
            ) * semantic_gate
            self._accumulate_anchor_profile("score_sec", time.perf_counter() - score_t0)
            anchor = AnchorNode(
                id=f"anchor_{obj.id}",
                anchor_type="object",
                position=(float(candidate_xy[0]), float(candidate_xy[1]), 0.0),
                room_id=room.id,
                target_id=obj.id,
                valid=True,
                score=score,
                floor_id=obj.floor_id or room.floor_id,
                candidate_positions=debug_candidates if debug else None,
            )
            if best_anchor is None or anchor.score > best_anchor.score:
                best_anchor = anchor

        if best_anchor is not None:
            self._accumulate_anchor_profile("object_anchor_total_sec", time.perf_counter() - object_t0)
            return best_anchor

        fallback_t0 = time.perf_counter()
        fallback_anchor = self._fallback_object_anchor(obj, room, room_objects)
        self._accumulate_anchor_profile("fallback_sec", time.perf_counter() - fallback_t0)
        if fallback_anchor and debug:
            fallback_anchor.candidate_positions = debug_candidates
        self._accumulate_anchor_profile("object_anchor_total_sec", time.perf_counter() - object_t0)
        return fallback_anchor

    def _remove_existing_anchor_layer(self) -> None:
        anchor_ids = [node_id for node_id, data in self.graph.nodes(data=True) if data.get("node_type") == "anchor"]
        self.graph.remove_nodes_from(anchor_ids)
        self.anchors.clear()

    def insert_anchor_nodes_into_graph(self, anchors: Sequence[AnchorNode]) -> None:
        insert_t0 = time.perf_counter()
        for anchor in anchors:
            self.anchors[anchor.id] = anchor
            self.graph.add_node(anchor.id, **anchor.get_attributes())
            self._add_relation_edge(anchor.id, anchor.room_id, "IN_ROOM")
            self._add_relation_edge(anchor.id, anchor.target_id, "FOR")
            if anchor.floor_id is not None and anchor.floor_id in self.graph.nodes:
                self._add_relation_edge(anchor.id, anchor.floor_id, "ON_FLOOR")
        self._accumulate_anchor_profile("insert_sec", time.perf_counter() - insert_t0)

    def generate_room_anchors(self, debug: bool = False) -> List[AnchorNode]:
        anchors: List[AnchorNode] = []
        for room_id in self.rooms:
            anchor = self.generate_room_anchor(room_id, debug=debug)
            if anchor is not None:
                anchors.append(anchor)
        return anchors

    def generate_object_anchors(self, debug: bool = False) -> List[AnchorNode]:
        anchors: List[AnchorNode] = []
        for object_id in self.object_index:
            anchor = self.generate_object_anchor(object_id, debug=debug)
            if anchor is not None:
                anchors.append(anchor)
        return anchors

    def build_anchor_layer(self, debug: bool = False) -> List[AnchorNode]:
        self._remove_existing_anchor_layer()
        anchors = self.generate_room_anchors(debug=debug) + self.generate_object_anchors(debug=debug)
        self.insert_anchor_nodes_into_graph(anchors)
        return anchors

    def export_anchor_data(self) -> List[Dict[str, Any]]:
        return [anchor.get_attributes() for anchor in self.anchors.values()]

    def print_graph(self) -> None:
        print("=== Scene Graph Nodes ===")
        for node, data in self.graph.nodes(data=True):
            node_type = data["node_type"]
            if node_type == "floor":
                print(f"🏢 {node} (Floor) | index={data.get('floor_index')} | z=[{data.get('z_min')}, {data.get('z_max')}]")
            elif node_type == "room":
                print(f"📍 {node} (Room) | type={data['room_type']} | floor={data.get('floor_id')} | center={data['center']}")
            elif node_type == "anchor":
                print(
                    f"🧭 {node} (Anchor) | anchor_type={data['anchor_type']} | "
                    f"target={data['target_id']} | pos={[round(p, 2) for p in data['position']]}"
                )
            else:
                print(
                    f"📦 {node} | Label: {data['label']:<10} | "
                    f"Center: {[round(p, 2) for p in data['center']]} | obs={data.get('obs_count', 1)}"
                )

        print("\n=== Scene Graph Edges (Relationships) ===")
        for source, target, data in self.graph.edges(data=True):
            rel = data["relation"]
            print(f"{source:<14} --[{rel:<8}]--> {target}")

    def print_node_attributes(self) -> None:
        print("\n=== Scene Graph Node Attributes ===")
        for node, data in self.graph.nodes(data=True):
            data_view = dict(data)
            for key in ("clip_feature", "clip_feature_fused"):
                if isinstance(data_view.get(key), np.ndarray):
                    feature = data_view[key]
                    data_view[key] = f"ndarray(shape={feature.shape}, dtype={feature.dtype})"
            print(f"{node}: {data_view}")

    def visualize_bev_graph(
        self,
        save_path: Optional[str] = "scene_graph_bev.png",
        show_anchor_links: bool = True,
        show_anchor_candidates: bool = False,
        show_relations: bool = True,
        verbose: bool = False,
    ) -> None:
        import matplotlib.pyplot as plt

        try:
            from adjustText import adjust_text

            use_adjust_text = True
        except ImportError:
            use_adjust_text = False

        fig, ax = plt.subplots(figsize=(14, 12))
        texts = []

        for room in self.rooms.values():
            if room.id == "room_unknown" or len(room.polygon) < 3:
                continue
            polygon = np.asarray(room.polygon, dtype=np.float32)
            ax.fill(polygon[:, 0], polygon[:, 1], color="#d9e9f6", alpha=0.35, zorder=1)
            ax.plot(
                np.r_[polygon[:, 0], polygon[0, 0]],
                np.r_[polygon[:, 1], polygon[0, 1]],
                color="#4c78a8",
                linewidth=1.5,
                zorder=2,
            )
            texts.append(ax.text(room.center[0], room.center[1], room.room_type, fontsize=11, weight="bold", zorder=8))

        for obj in self.objects:
            if not obj.footprint:
                continue
            polygon = np.asarray(obj.footprint, dtype=np.float32)
            ax.fill(polygon[:, 0], polygon[:, 1], color="#91d39a", alpha=0.25, zorder=3)
            ax.plot(
                np.r_[polygon[:, 0], polygon[0, 0]],
                np.r_[polygon[:, 1], polygon[0, 1]],
                color="#3a7f42",
                linewidth=1.1,
                zorder=4,
            )
            ax.scatter(obj.center[0], obj.center[1], c="#2f5d34", s=20, zorder=5)
            texts.append(
                ax.text(
                    obj.center[0],
                    obj.center[1],
                    f"{obj.canonical_label} ({obj.obs_count})",
                    fontsize=8,
                    color="#1f2d1f",
                    zorder=8,
                )
            )

        if show_relations:
            for source, target, data in self.graph.edges(data=True):
                relation = data.get("relation")
                if relation not in {"ON", "NEXT_TO"}:
                    continue
                if source not in self.object_index or target not in self.object_index:
                    continue
                src = self.object_index[source]
                dst = self.object_index[target]
                style = ":" if relation == "NEXT_TO" else "-"
                color = "#c98f70" if relation == "ON" else "#8f8fd8"
                ax.plot([src.center[0], dst.center[0]], [src.center[1], dst.center[1]], linestyle=style, color=color, alpha=0.6, zorder=2)

        room_anchor_points = []
        object_anchor_points = []
        for anchor in self.anchors.values():
            if anchor.anchor_type == "room":
                room_anchor_points.append(anchor)
            else:
                object_anchor_points.append(anchor)
            if show_anchor_links:
                target_center: Optional[Tuple[float, float]] = None
                if anchor.target_id in self.rooms:
                    target_center = self.rooms[anchor.target_id].center
                elif anchor.target_id in self.object_index:
                    target_center = (self.object_index[anchor.target_id].center[0], self.object_index[anchor.target_id].center[1])
                if target_center is not None:
                    ax.plot(
                        [anchor.position[0], target_center[0]],
                        [anchor.position[1], target_center[1]],
                        color="#7a7a7a",
                        linewidth=0.8,
                        linestyle="--",
                        alpha=0.7,
                        zorder=4,
                    )
            if show_anchor_candidates and anchor.candidate_positions:
                candidates = np.asarray(anchor.candidate_positions, dtype=np.float32)
                ax.scatter(candidates[:, 0], candidates[:, 1], c="#f0b14a", s=15, alpha=0.5, zorder=5)

        if room_anchor_points:
            room_pts = np.asarray([anchor.position[:2] for anchor in room_anchor_points], dtype=np.float32)
            ax.scatter(room_pts[:, 0], room_pts[:, 1], c="#d95f02", marker="*", s=110, label="Room Anchor", zorder=6)

        if object_anchor_points:
            obj_pts = np.asarray([anchor.position[:2] for anchor in object_anchor_points], dtype=np.float32)
            ax.scatter(obj_pts[:, 0], obj_pts[:, 1], c="#1b9e77", marker="D", s=55, label="Object Anchor", zorder=6)

        if use_adjust_text and texts:
            adjust_text(texts, ax=ax, arrowprops=dict(arrowstyle="-", color="gray", lw=0.4, alpha=0.6))

        ax.set_title("BEV Semantic Scene Graph")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, linestyle=":", alpha=0.35)
        ax.legend(loc="upper right")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches="tight")
            if verbose:
                print(f"✅ BEV Scene Graph 已保存至 {save_path}")
            plt.close(fig)
        else:
            plt.show()

    def visualize_2d_graph(
        self,
        save_path: Optional[str] = "scene_graph_2d.png",
        show_anchor_links: bool = True,
        show_anchor_candidates: bool = False,
        verbose: bool = False,
    ) -> None:
        self.visualize_bev_graph(
            save_path=save_path,
            show_anchor_links=show_anchor_links,
            show_anchor_candidates=show_anchor_candidates,
            show_relations=True,
            verbose=verbose,
        )


if __name__ == "__main__":
    sg = SemanticSceneGraph()

    sg.add_room_node(RoomNode(id="room_1", room_type="living_room", polygon=[(0, 0), (5, 0), (5, 5), (0, 5)]))
    sg.add_room_node(RoomNode(id="room_2", room_type="bedroom", polygon=[(6, 0), (10, 0), (10, 4), (6, 4)]))

    table = ObjectNode(
        id="obj_101",
        center=(2.5, 2.5, 0.4),
        bbox=(1.2, 0.8, 0.8),
        label="desk",
        category="furniture",
        clip_feature=np.random.rand(8),
        confidence=0.65,
        room_id="room_1",
        semantic_observations=[
            {"label": "desk", "category": "furniture", "detection_confidence": 0.65, "semantic_confidence": 0.60},
            {"label": "table", "category": "furniture", "detection_confidence": 0.93, "semantic_confidence": 0.92, "semantic_gap": 0.22},
        ],
    )
    apple = ObjectNode(
        id="obj_102",
        center=(2.5, 2.5, 0.85),
        bbox=(0.1, 0.1, 0.1),
        label="apple",
        category="food",
        clip_feature=np.random.rand(8),
        confidence=0.81,
        room_id="room_1",
    )
    sofa = ObjectNode(
        id="obj_201",
        center=(7.5, 2.0, 0.35),
        bbox=(1.8, 0.9, 0.7),
        label="couch",
        category="furniture",
        clip_feature=np.random.rand(8),
        confidence=0.86,
        room_id="room_2",
        semantic_observations=[
            {"label": "couch", "category": "furniture", "detection_confidence": 0.86, "semantic_confidence": 0.80},
            {"label": "sofa", "category": "furniture", "detection_confidence": 0.92, "semantic_confidence": 0.89, "semantic_gap": 0.18},
        ],
    )

    for obj in (table, apple, sofa):
        sg.add_object_node(obj)
        sg.add_inside_relation(obj.id, obj.room_id)

    sg.compute_spatial_relations(dist_threshold=1.5, z_tolerance=0.1)
    sg.build_anchor_layer(debug=True)
    sg.print_graph()
    sg.print_node_attributes()
    sg.visualize_bev_graph(save_path="demo_scene_graph.png", show_anchor_candidates=True)
