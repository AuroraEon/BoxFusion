from __future__ import annotations

import html
import json
import math
import csv
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx

from boxfusion.committed_artifact_ids import CommittedArtifactIdNormalizer, canonical_room_id
from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopology, _canonical_room_id
from boxfusion.template_grounding.normalizer import normalize_reference_slot


ROOM_GRAPH_VLN_DEMO_VERSION = "0.1"
ROOM_GRAPH_VLN_ENHANCED_VERSION = "0.2"
AUDIT_OVERLAY_DISCLAIMER = (
    "Audit overlay is non-authoritative, does not change routing, is not a ground-truth topology error label, "
    "and weak geometry candidates should not drive repair by themselves."
)
DEFAULT_RELATION_PRIORITY: Dict[str, int] = {
    "transition": 0,
    "vertical_transition": 1,
    "adjacent": 2,
    "possible_connection": 3,
}
DEFAULT_FLOOR_COLORS: Tuple[str, ...] = (
    "#f2c14e",
    "#5aa9e6",
    "#7fc8a9",
    "#ef7d57",
    "#b084cc",
    "#8d99ae",
)
ROOM_NAME_PLACEHOLDERS = {"", "unknown", "none", "null", "room", "other", "unlabeled", "unassigned"}
SEMANTIC_SYNONYMS: Dict[str, Tuple[str, ...]] = {
    "sofa": ("couch",),
    "couch": ("sofa",),
    "bathroom": ("toilet", "sink"),
    "restroom": ("toilet", "sink"),
    "bedroom": ("bed", "nightstand", "pillow"),
    "living_room": ("sofa", "couch", "chair", "coffee_table"),
    "living room": ("sofa", "couch", "chair", "coffee_table"),
    "kitchen": ("sink",),
}
POLYGON_PROXIMITY_EVIDENCE_REFS = {"boundary_contact", "polygon_proximity"}
TRAJECTORY_EVIDENCE_REFS = {"trajectory_transition", "repeated_crossing"}


def _unique_sorted_strings(values: Iterable[Any]) -> List[str]:
    return sorted({str(value).strip() for value in values if str(value).strip()})


def _lower_string_set(values: Iterable[Any]) -> set[str]:
    return {str(value).strip().lower() for value in values if str(value).strip()}


def _has_gateway_hint(values: Iterable[Any]) -> bool:
    return any("gateway" in str(value).strip().lower() for value in values if str(value).strip())


def _presentation_semantics_payload() -> Dict[str, Any]:
    return {
        "legend_items": [
            {
                "label": "committed_topology_edge",
                "meaning": "Public downstream graph edge from topology_v0_1.json.",
            },
            {
                "label": "gateway_backed_passage",
                "meaning": "Stronger physical passage relation backed by gateway evidence.",
            },
            {
                "label": "polygon_proximity_adjacent_edge",
                "meaning": "Weaker same-floor adjacency derived from boundary contact or polygon proximity, not a gateway-backed passage.",
            },
            {
                "label": "possible_connection",
                "meaning": "Weak or provisional fallback relation unless stronger evidence also exists.",
            },
            {
                "label": "trajectory_supported_transition",
                "meaning": "Same-floor movement relation supported by trajectory or repeated-transition evidence.",
            },
            {
                "label": "vertical_transition",
                "meaning": "Cross-floor connector only; same-floor weak adjacency does not require vertical-transition evidence.",
            },
            {
                "label": "room_summary_neighbor_relation",
                "meaning": "neighbor_room_ids reflect room-summary or spatial-neighbor semantics and are not guaranteed to equal committed topology edges.",
            },
            {
                "label": "audit_overlay",
                "meaning": "Non-authoritative diagnostic overlay; it does not change routing and does not prove a topology repair target by itself.",
            },
            {
                "label": "route_selected_relation_matching",
                "meaning": "Route-selected labels use relation-specific matching where metadata suffices; same-pair visibility alone is shown separately.",
            },
        ],
        "audit_overlay_disclaimer": AUDIT_OVERLAY_DISCLAIMER,
        "route_matching_policy": (
            "Actual route-selected relation labels require relation-specific matching. "
            "Same-pair alternate relations remain visible for inspection but are not labeled as route-selected."
        ),
    }


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _load_json_if_exists(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None or not Path(path).exists():
        return None
    return _load_json(Path(path))


def _load_csv_rows(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return [dict(row) for row in csv.DictReader(f)]


def _truthy_csv(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _room_sort_key(room_record: Dict[str, Any], room_id: str) -> Tuple[int, int, str]:
    display_order = room_record.get("display_order")
    try:
        display_order_value = int(display_order)
    except (TypeError, ValueError):
        display_order_value = 10**6
    numeric_room_id = 10**6
    canonical_room_id = _canonical_room_id(room_id)
    if canonical_room_id and canonical_room_id.startswith("room_"):
        suffix = canonical_room_id.split("room_", 1)[-1]
        if suffix.isdigit():
            numeric_room_id = int(suffix)
    return (display_order_value, numeric_room_id, str(room_id))


def _preferred_relation_type(relation_types: Iterable[str]) -> Optional[str]:
    filtered = [str(item) for item in relation_types if item]
    if not filtered:
        return None
    return min(filtered, key=lambda item: (DEFAULT_RELATION_PRIORITY.get(item, 99), item))


def _relation_color(relation_type: Optional[str], *, highlight: bool = False) -> str:
    if highlight:
        return "#e4572e"
    if relation_type == "vertical_transition":
        return "#7f5af0"
    if relation_type == "transition":
        return "#2e86ab"
    if relation_type == "adjacent":
        return "#7a8a99"
    return "#b0b7bf"


def _escape(value: Any) -> str:
    return html.escape("" if value is None else str(value), quote=True)


def _round_float(value: Any, digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def _as_int(value: Any) -> Optional[int]:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None


def _normalize_term(value: Any) -> str:
    return normalize_reference_slot(value)


def _best_public_room_ids(
    topology: RoomTopology,
    room_model_by_id: Dict[str, Dict[str, Any]],
) -> List[str]:
    room_ids = list(topology.list_room_ids())
    return sorted(room_ids, key=lambda room_id: _room_sort_key(room_model_by_id.get(room_id, {}), room_id))


def _dominant_labels(room_model: Dict[str, Any], limit: int = 5) -> List[str]:
    summary = dict(room_model.get("semantic_summary") or {})
    dominant = [str(item) for item in summary.get("dominant_object_labels", []) if str(item).strip()]
    if dominant:
        return dominant[:limit]
    labels = sorted((summary.get("object_label_counts") or {}).items(), key=lambda item: (-int(item[1]), str(item[0])))
    return [str(label) for label, _ in labels[:limit]]


def _room_display_name(room_id: str, room_record: Dict[str, Any], room_model: Optional[Dict[str, Any]]) -> str:
    room_type = _normalize_term(room_record.get("room_type"))
    if room_type and room_type not in ROOM_NAME_PLACEHOLDERS:
        return f"{room_id} ({room_type.replace('_', ' ')})"
    dominant = _dominant_labels(room_model or {}, limit=2)
    if dominant:
        return f"{room_id} ({', '.join(dominant)})"
    return room_id


def _room_semantic_lines(room_model: Dict[str, Any]) -> List[str]:
    if not room_model:
        return []
    summary = dict(room_model.get("semantic_summary") or {})
    counts = dict(summary.get("object_label_counts") or {})
    if counts:
        ordered = sorted(counts.items(), key=lambda item: (-int(item[1]), str(item[0])))
        return [f"{label} x{count}" for label, count in ordered[:4]]
    dominant = [str(item) for item in summary.get("dominant_object_labels", []) if str(item).strip()]
    return dominant[:4]


def _pair_key(a: Any, b: Any) -> Tuple[str, str]:
    return tuple(sorted((str(a), str(b))))  # type: ignore[return-value]


def _point_xy(value: Any) -> Optional[List[float]]:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
    return None


def _room_center(room_record: Dict[str, Any], room_model: Optional[Dict[str, Any]] = None) -> Optional[List[float]]:
    for candidate in (
        room_record.get("center"),
        room_record.get("centroid_xy"),
        (room_model or {}).get("centroid_xy"),
        ((room_model or {}).get("bev_vln_hook") or {}).get("centroid_xy"),
    ):
        point = _point_xy(candidate)
        if point is not None:
            return point
    polygon = room_record.get("polygon") or (room_model or {}).get("footprint_polygon_xy")
    points = [_point_xy(item) for item in polygon or []]
    points = [point for point in points if point is not None]
    if points:
        return [
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        ]
    return None


@dataclass
class DemoArtifactPaths:
    scene_root: Optional[Path]
    summary_json: Optional[Path]
    topology_json: Path
    committed_room_world_model_json: Path
    committed_room_world_snapshot_json: Optional[Path] = None


class RoomGraphVLNDemo:
    def __init__(
        self,
        artifact_paths: DemoArtifactPaths,
    ) -> None:
        self.artifact_paths = artifact_paths
        self.summary_payload = (
            None if artifact_paths.summary_json is None else _load_json(artifact_paths.summary_json)
        )
        self.topology_payload = _load_json(artifact_paths.topology_json)
        self.committed_room_world_model_payload = _load_json(artifact_paths.committed_room_world_model_json)
        self.committed_room_world_snapshot_payload = _load_json_if_exists(
            artifact_paths.committed_room_world_snapshot_json
        )
        self.snapshot_normalizer = CommittedArtifactIdNormalizer.from_snapshot(
            self.committed_room_world_snapshot_payload
        )
        self.topology = RoomTopology.from_json(artifact_paths.topology_json)
        self.query_api = RoomTopologyQueryAPI(self.topology)
        self.evidence_by_id = {
            str(item.get("evidence_id")): dict(item)
            for item in self.topology_payload.get("evidences", [])
            if isinstance(item, dict) and item.get("evidence_id")
        }
        self.room_model_by_id = {
            str(room.get("room_id")): dict(room) for room in self.committed_room_world_model_payload.get("rooms", [])
        }
        self.public_room_ids = _best_public_room_ids(self.topology, self.room_model_by_id)
        self.floor_records = {
            str(item.get("floor_id")): dict(item)
            for item in self.topology_payload.get("floors", [])
            if item.get("floor_id") is not None
        }
        self.topology_room_ids = set(self.public_room_ids)
        self.topology_edges_by_pair: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
        for edge_index, raw_edge in enumerate(self.topology_payload.get("edges", []) or []):
            source = canonical_room_id(raw_edge.get("source"))
            target = canonical_room_id(raw_edge.get("target"))
            if not source or not target:
                continue
            self.topology_edges_by_pair[_pair_key(source, target)].append(
                {
                    "edge_index": edge_index,
                    "source_room": source,
                    "target_room": target,
                    "relation_type": raw_edge.get("relation_type"),
                    "confidence": _round_float(raw_edge.get("confidence")),
                    "status": raw_edge.get("status"),
                    "support_count": _as_int(raw_edge.get("support_count")),
                    "evidence_ids": list(raw_edge.get("evidence_ids") or []),
                    "metadata": dict(raw_edge.get("metadata") or {}),
                }
            )

    @classmethod
    def from_inputs(
        cls,
        *,
        scene_root: Optional[Path] = None,
        summary_json: Optional[Path] = None,
        topology_json: Optional[Path] = None,
        committed_room_world_model_json: Optional[Path] = None,
        committed_room_world_snapshot_json: Optional[Path] = None,
    ) -> "RoomGraphVLNDemo":
        artifact_paths = resolve_demo_artifact_paths(
            scene_root=scene_root,
            summary_json=summary_json,
            topology_json=topology_json,
            committed_room_world_model_json=committed_room_world_model_json,
            committed_room_world_snapshot_json=committed_room_world_snapshot_json,
        )
        return cls(artifact_paths)

    @property
    def sequence_id(self) -> Optional[str]:
        return self.topology.sequence_id or self.topology_payload.get("sequence_id")

    def _default_start_room_id(self) -> Optional[str]:
        return self.public_room_ids[0] if self.public_room_ids else None

    def resolve_room_selector(self, selector: Optional[str], *, label: str) -> Dict[str, Any]:
        if selector is None:
            return {
                "resolved": False,
                "failure_reason": f"{label}_missing",
                "notes": [f"No {label.replace('_', ' ')} selector was provided."],
            }
        selector_text = str(selector).strip()
        canonical_selector = _canonical_room_id(selector_text)
        if canonical_selector and canonical_selector in self.public_room_ids:
            room_record = self.topology.get_room(canonical_selector) or {}
            return {
                "resolved": True,
                "input_text": selector_text,
                "match_type": "room_id",
                "resolved_room_id": canonical_selector,
                "resolved_room_record": room_record,
                "notes": [f"Matched {label.replace('_', ' ')} by public room id."],
            }

        normalized_selector = _normalize_term(selector_text)
        exact_name_matches = []
        contains_matches = []
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            room_type = _normalize_term(room_record.get("room_type"))
            if not room_type or room_type in ROOM_NAME_PLACEHOLDERS:
                continue
            if room_type == normalized_selector:
                exact_name_matches.append(room_id)
            elif normalized_selector and (normalized_selector in room_type or room_type in normalized_selector):
                contains_matches.append(room_id)
        candidate_ids = exact_name_matches or contains_matches
        if len(candidate_ids) == 1:
            room_id = candidate_ids[0]
            room_record = self.topology.get_room(room_id) or {}
            return {
                "resolved": True,
                "input_text": selector_text,
                "match_type": "room_name",
                "resolved_room_id": room_id,
                "resolved_room_record": room_record,
                "notes": [f"Matched {label.replace('_', ' ')} by public room_type."],
            }
        if len(candidate_ids) > 1:
            return {
                "resolved": False,
                "failure_reason": f"{label}_ambiguous",
                "candidate_room_ids": candidate_ids,
                "notes": [f"Multiple public rooms matched {selector_text!r}."],
            }
        return {
            "resolved": False,
            "failure_reason": f"{label}_not_found",
            "notes": [f"No public room matched {selector_text!r} by id or room_type."],
        }

    def resolve_start_room(self, start_room: Optional[str]) -> Dict[str, Any]:
        if start_room is None:
            default_room_id = self._default_start_room_id()
            if default_room_id is None:
                return {
                    "resolved": False,
                    "failure_reason": "start_room_unavailable",
                    "notes": ["The public topology export has no rooms."],
                }
            room_record = self.topology.get_room(default_room_id) or {}
            return {
                "resolved": True,
                "input_text": None,
                "match_type": "default_public_room",
                "resolved_room_id": default_room_id,
                "resolved_room_record": room_record,
                "notes": ["No start room was provided; defaulted to the first public room in display order."],
            }
        return self.resolve_room_selector(start_room, label="start_room")

    def resolve_goal_room(self, goal_room: Optional[str]) -> Dict[str, Any]:
        payload = self.resolve_room_selector(goal_room, label="goal_room")
        payload["goal_mode"] = "explicit_room"
        return payload

    def _iter_semantic_terms(self, room_id: str) -> List[Tuple[str, float, str]]:
        room_record = self.topology.get_room(room_id) or {}
        room_model = self.room_model_by_id.get(room_id, {})
        summary = dict(room_model.get("semantic_summary") or {})
        terms: List[Tuple[str, float, str]] = []
        room_type = _normalize_term(room_record.get("room_type"))
        if room_type and room_type not in ROOM_NAME_PLACEHOLDERS:
            terms.append((room_type, 12.0, "room_type"))
        for label, count in (summary.get("object_label_counts") or {}).items():
            normalized = _normalize_term(label)
            if normalized:
                terms.append((normalized, 6.0 + min(float(count), 5.0), "object_label"))
        for label in summary.get("dominant_object_labels", []) or []:
            normalized = _normalize_term(label)
            if normalized:
                terms.append((normalized, 8.0, "dominant_label"))
        for label in (room_model.get("bev_vln_hook", {}) or {}).get("semantic_landmarks", []) or []:
            normalized = _normalize_term(label)
            if normalized:
                terms.append((normalized, 5.0, "semantic_landmark"))
        return terms

    def _score_semantic_match(self, room_id: str, semantic_target: str) -> Optional[Dict[str, Any]]:
        normalized_target = _normalize_term(semantic_target)
        if not normalized_target:
            return None
        target_aliases = {normalized_target}
        target_aliases.update(_normalize_term(item) for item in SEMANTIC_SYNONYMS.get(normalized_target, ()))
        target_tokens = {token for alias in target_aliases for token in alias.split("_") if token}
        score = 0.0
        matched_terms: List[Dict[str, Any]] = []
        for term, weight, source in self._iter_semantic_terms(room_id):
            term_tokens = {token for token in term.split("_") if token}
            exact_match = term in target_aliases
            reverse_match = normalized_target in term or term in target_aliases
            overlap_count = len(term_tokens & target_tokens)
            if not exact_match and not reverse_match and overlap_count == 0:
                continue
            match_score = 0.0
            if exact_match:
                match_score += weight * 3.0
            elif reverse_match:
                match_score += weight * 1.75
            if overlap_count:
                match_score += overlap_count * 1.5
            score += match_score
            matched_terms.append(
                {
                    "term": term,
                    "source": source,
                    "score": _round_float(match_score),
                }
            )
        if score <= 0.0:
            return None
        room_record = self.topology.get_room(room_id) or {}
        return {
            "room_id": room_id,
            "score": _round_float(score),
            "matched_terms": matched_terms,
            "room_type": room_record.get("room_type"),
            "display_name": _room_display_name(room_id, room_record, self.room_model_by_id.get(room_id)),
        }

    def resolve_semantic_goal(self, semantic_target: Optional[str]) -> Dict[str, Any]:
        if semantic_target is None:
            return {
                "resolved": False,
                "goal_mode": "semantic_room_summary",
                "failure_reason": "semantic_target_missing",
                "notes": ["No semantic target was provided."],
            }
        candidates = []
        for room_id in self.public_room_ids:
            candidate = self._score_semantic_match(room_id, semantic_target)
            if candidate is not None:
                candidates.append(candidate)
        candidates = sorted(
            candidates,
            key=lambda item: (-float(item.get("score", 0.0)), _room_sort_key(self.room_model_by_id.get(item["room_id"], {}), item["room_id"])),
        )
        if not candidates:
            return {
                "resolved": False,
                "goal_mode": "semantic_room_summary",
                "failure_reason": "semantic_target_unresolved",
                "semantic_target": semantic_target,
                "notes": [
                    "No public room summary produced a positive score for the requested semantic target.",
                ],
                "candidate_matches": [],
            }
        resolved_room_id = candidates[0]["room_id"]
        room_record = self.topology.get_room(resolved_room_id) or {}
        return {
            "resolved": True,
            "goal_mode": "semantic_room_summary",
            "semantic_target": semantic_target,
            "resolved_room_id": resolved_room_id,
            "resolved_room_record": room_record,
            "candidate_matches": candidates[:5],
            "notes": [
                "Resolved against committed room semantic summaries, restricted to rooms present in the public topology export.",
            ],
        }

    def _path_pair_set(self, room_sequence: Sequence[str]) -> set[Tuple[str, str]]:
        return {
            tuple(sorted((room_sequence[idx], room_sequence[idx + 1])))
            for idx in range(max(0, len(room_sequence) - 1))
        }

    def _normalization_warnings(self) -> List[Dict[str, Any]]:
        warnings = [dict(item) for item in self.snapshot_normalizer.summary().get("warnings", [])]
        for collection_name, rows in (
            ("gateways", self.snapshot_normalizer.normalized_gateways),
            ("vertical_transitions", self.snapshot_normalizer.normalized_vertical_transitions),
        ):
            for idx, row in enumerate(rows):
                for field in ("room_a", "room_b"):
                    room_id = row.get(field)
                    if room_id and room_id not in self.topology_room_ids:
                        warnings.append(
                            {
                                "context": f"snapshot.{collection_name}[{idx}]",
                                "field": field,
                                "value": room_id,
                                "message": "normalized room id is not present in public topology rooms",
                            }
                        )
        return warnings

    def _route_pair_set(self, demo_result: Dict[str, Any]) -> set[Tuple[str, str]]:
        return {tuple(sorted(item)) for item in demo_result.get("path_pair_set", [])}

    def _gateway_records(self, route_pair_set: set[Tuple[str, str]]) -> List[Dict[str, Any]]:
        records: List[Dict[str, Any]] = []
        for row in self.snapshot_normalizer.normalized_gateways:
            room_a = row.get("room_a")
            room_b = row.get("room_b")
            raw_record = dict(row.get("record") or {})
            pair = _pair_key(room_a, room_b) if room_a and room_b else None
            point = _point_xy(raw_record.get("pos_world"))
            geometry_source = "pos_world" if point is not None else None
            if point is None:
                point = _point_xy(raw_record.get("grid_pos"))
                geometry_source = "grid_pos" if point is not None else None
            if point is None and room_a and room_b:
                center_a = _room_center(self.topology.get_room(room_a) or {}, self.room_model_by_id.get(room_a))
                center_b = _room_center(self.topology.get_room(room_b) or {}, self.room_model_by_id.get(room_b))
                if center_a and center_b:
                    point = [(center_a[0] + center_b[0]) / 2.0, (center_a[1] + center_b[1]) / 2.0]
                    geometry_source = "approximate_room_midpoint"
            records.append(
                {
                    "index": row.get("index"),
                    "room_a": room_a,
                    "room_b": room_b,
                    "room_pair": list(pair) if pair else [],
                    "gateway_type": raw_record.get("type"),
                    "width_m": raw_record.get("width_m"),
                    "floor_id": raw_record.get("floor_id"),
                    "display_floor_id": raw_record.get("display_floor_id"),
                    "pos_world": raw_record.get("pos_world"),
                    "grid_pos": raw_record.get("grid_pos"),
                    "plot_xy": point,
                    "geometry_source": geometry_source or "unavailable",
                    "is_approximate": geometry_source == "approximate_room_midpoint",
                    "on_selected_route": bool(pair and pair in route_pair_set),
                }
            )
        return records

    def _vertical_transition_records(self, route_pair_set: set[Tuple[str, str]]) -> List[Dict[str, Any]]:
        records: List[Dict[str, Any]] = []
        for row in self.snapshot_normalizer.normalized_vertical_transitions:
            room_a = row.get("room_a")
            room_b = row.get("room_b")
            raw_record = dict(row.get("record") or {})
            pair = _pair_key(room_a, room_b) if room_a and room_b else None
            from_xy = _point_xy(raw_record.get("from_position_xy"))
            to_xy = _point_xy(raw_record.get("to_position_xy"))
            if from_xy is None and room_a:
                from_xy = _room_center(self.topology.get_room(room_a) or {}, self.room_model_by_id.get(room_a))
            if to_xy is None and room_b:
                to_xy = _room_center(self.topology.get_room(room_b) or {}, self.room_model_by_id.get(room_b))
            records.append(
                {
                    "index": row.get("index"),
                    "transition_id": row.get("transition_id"),
                    "type": raw_record.get("type"),
                    "status": raw_record.get("status"),
                    "confidence": raw_record.get("confidence"),
                    "from_floor_id": raw_record.get("from_floor_id"),
                    "to_floor_id": raw_record.get("to_floor_id"),
                    "from_display_floor_id": raw_record.get("from_display_floor_id"),
                    "to_display_floor_id": raw_record.get("to_display_floor_id"),
                    "from_room_id": room_a,
                    "to_room_id": room_b,
                    "room_pair": list(pair) if pair else [],
                    "connector_label": raw_record.get("connector_label"),
                    "evidence_summary": raw_record.get("evidence_summary"),
                    "from_position_xy": from_xy,
                    "to_position_xy": to_xy,
                    "on_selected_route": bool(pair and pair in route_pair_set),
                }
            )
        return records

    def _edge_evidence_details(self, edge_record: Dict[str, Any]) -> Dict[str, Any]:
        evidence_ids = [str(item) for item in edge_record.get("evidence_ids") or [] if str(item).strip()]
        resolved = [self.evidence_by_id[evidence_id] for evidence_id in evidence_ids if evidence_id in self.evidence_by_id]
        metadata = dict(edge_record.get("metadata") or {})
        metadata_breakdown = {
            str(key): value
            for key, value in (metadata.get("evidence_type_breakdown") or {}).items()
            if str(key).strip()
        }
        evidence_types = _unique_sorted_strings(
            list(metadata_breakdown) + [item.get("evidence_type") for item in resolved]
        )
        source_refs = _unique_sorted_strings(item.get("source_ref") for item in resolved)
        return {
            "evidence_ids": evidence_ids,
            "resolved_evidence": resolved,
            "evidence_types": evidence_types,
            "evidence_source_refs": source_refs,
            "evidence_type_breakdown": metadata_breakdown,
            "evidence_type_summary": ", ".join(evidence_types) or "none",
            "evidence_source_summary": ", ".join(source_refs) or "none",
        }

    def _edge_semantic_labels(
        self,
        *,
        relation_type: Optional[str],
        evidence_types: Sequence[str],
        evidence_source_refs: Sequence[str],
        has_gateway_match: bool,
        has_vertical_transition_match: bool,
        cross_floor: bool,
    ) -> List[str]:
        lowered_refs = _lower_string_set(list(evidence_types) + list(evidence_source_refs))
        labels = ["committed_topology_edge"]
        if relation_type == "vertical_transition" or has_vertical_transition_match or cross_floor:
            labels.append("vertical_transition")
        elif relation_type == "transition":
            labels.append("trajectory_supported_transition")
        elif relation_type == "possible_connection":
            labels.append("possible_connection")
        elif relation_type == "adjacent":
            if lowered_refs & POLYGON_PROXIMITY_EVIDENCE_REFS:
                labels.append("polygon_proximity_adjacent_edge")
            else:
                labels.append("adjacent")
        if has_gateway_match or _has_gateway_hint(list(evidence_types) + list(evidence_source_refs)):
            labels.append("gateway_backed_passage")
        return labels

    def _semantic_summary_for_edge(
        self,
        edge_record: Dict[str, Any],
        *,
        has_gateway_match: bool,
        has_vertical_transition_match: bool,
        cross_floor: bool,
    ) -> Dict[str, Any]:
        evidence = self._edge_evidence_details(edge_record)
        labels = self._edge_semantic_labels(
            relation_type=str(edge_record.get("relation_type") or ""),
            evidence_types=evidence["evidence_types"],
            evidence_source_refs=evidence["evidence_source_refs"],
            has_gateway_match=has_gateway_match,
            has_vertical_transition_match=has_vertical_transition_match,
            cross_floor=cross_floor,
        )
        return {
            **evidence,
            "semantic_labels": labels,
            "semantic_summary": "; ".join(labels) or "committed_topology_edge",
        }

    def _match_route_edge_to_topology_edge(
        self,
        *,
        route_edge: Dict[str, Any],
        pair_edges: Sequence[Dict[str, Any]],
    ) -> Tuple[Optional[Dict[str, Any]], str, str]:
        if not pair_edges:
            return None, "pair_unavailable", "No committed topology edge was found for this room pair."
        route_evidence_ids = [str(item) for item in route_edge.get("evidence_ids") or [] if str(item).strip()]
        route_relation_type = str(route_edge.get("relation_type") or "")
        route_confidence = _round_float(route_edge.get("confidence"))
        route_support_count = _as_int(route_edge.get("support_count"))
        route_status = str(route_edge.get("status") or "")
        route_metadata = dict(route_edge.get("metadata") or {})
        scored: List[Tuple[int, Dict[str, Any]]] = []
        for candidate in pair_edges:
            score = 0
            if str(candidate.get("relation_type") or "") == route_relation_type:
                score += 40
            if list(candidate.get("evidence_ids") or []) == route_evidence_ids and route_evidence_ids:
                score += 50
            elif set(candidate.get("evidence_ids") or []) & set(route_evidence_ids):
                score += 20
            if _as_int(candidate.get("support_count")) == route_support_count and route_support_count is not None:
                score += 8
            if _round_float(candidate.get("confidence")) == route_confidence and route_confidence is not None:
                score += 8
            if str(candidate.get("status") or "") == route_status and route_status:
                score += 5
            if dict(candidate.get("metadata") or {}).get("evidence_type_breakdown") == route_metadata.get(
                "evidence_type_breakdown"
            ) and route_metadata.get("evidence_type_breakdown"):
                score += 10
            scored.append((score, candidate))
        if not scored:
            return None, "pair_unavailable", "No committed topology edge was found for this room pair."
        scored.sort(key=lambda item: (-item[0], int(item[1].get("edge_index", 10**9))))
        best_score, best = scored[0]
        top_matches = [item for item in scored if item[0] == best_score]
        if best_score >= 90 and len(top_matches) == 1:
            return best, "exact_topology_edge_match", "Selected route relation matched a committed topology edge by relation and evidence metadata."
        if best_score >= 40 and len(top_matches) == 1:
            return best, "best_effort_relation_match", "Selected route relation matched a committed topology edge by best available relation metadata."
        if best_score >= 40:
            return None, "same_pair_relation_visible", "This room pair is route-visible, but same-pair metadata is insufficient for a stable relation-specific match."
        return None, "pair_visible_only", "Only room-pair visibility is available for this route step; relation-specific matching is insufficient."

    def _route_edge_explanations(
        self,
        route: Dict[str, Any],
        gateway_records: Sequence[Dict[str, Any]],
        vertical_transition_records: Sequence[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        gateway_pairs = {
            _pair_key(item.get("room_a"), item.get("room_b"))
            for item in gateway_records
            if item.get("room_a") and item.get("room_b")
        }
        vertical_pairs = {
            _pair_key(item.get("from_room_id"), item.get("to_room_id"))
            for item in vertical_transition_records
            if item.get("from_room_id") and item.get("to_room_id")
        }
        explanations = []
        for idx, edge in enumerate(route.get("edges", []) or [], start=1):
            source = canonical_room_id(edge.get("source_room_id") or edge.get("source"))
            target = canonical_room_id(edge.get("target_room_id") or edge.get("target"))
            pair = _pair_key(source, target) if source and target else None
            source_room = self.topology.get_room(source) or {}
            target_room = self.topology.get_room(target) or {}
            source_floor = source_room.get("floor_id")
            target_floor = target_room.get("floor_id")
            evidence_ids = list(edge.get("evidence_ids") or [])
            support_count = _as_int(edge.get("support_count"))
            confidence = _round_float(edge.get("confidence"))
            has_gateway_match = bool(pair and pair in gateway_pairs)
            has_vertical_transition_match = bool(pair and pair in vertical_pairs) or bool(
                (edge.get("metadata") or {}).get("transition_ids")
            )
            matched_edge, match_precision, match_note = self._match_route_edge_to_topology_edge(
                route_edge=edge,
                pair_edges=self.topology_edges_by_pair.get(pair or ("", ""), []),
            )
            semantic_summary = self._semantic_summary_for_edge(
                edge,
                has_gateway_match=has_gateway_match,
                has_vertical_transition_match=has_vertical_transition_match,
                cross_floor=bool(source_floor != target_floor),
            )
            selected_edge_index = None if matched_edge is None else matched_edge.get("edge_index")
            alternate_relations = []
            for candidate in self.topology_edges_by_pair.get(pair or ("", ""), []):
                if selected_edge_index is not None and candidate.get("edge_index") == selected_edge_index:
                    continue
                candidate_semantics = self._semantic_summary_for_edge(
                    candidate,
                    has_gateway_match=has_gateway_match,
                    has_vertical_transition_match=has_vertical_transition_match,
                    cross_floor=bool(source_floor != target_floor),
                )
                alternate_relations.append(
                    {
                        "edge_index": candidate.get("edge_index"),
                        "relation_type": candidate.get("relation_type"),
                        "confidence": candidate.get("confidence"),
                        "support_count": candidate.get("support_count"),
                        "status": candidate.get("status"),
                        "evidence_ids": list(candidate.get("evidence_ids") or []),
                        "evidence_type_summary": candidate_semantics.get("evidence_type_summary"),
                        "semantic_labels": list(candidate_semantics.get("semantic_labels") or []),
                        "semantic_summary": candidate_semantics.get("semantic_summary"),
                    }
                )
            notes = []
            if support_count is None or support_count <= 1 or (confidence is not None and confidence < 0.35):
                notes.append("low-support route edge; keep as committed topology, not a repair target")
            if source_floor != target_floor:
                notes.append("cross-floor edge")
            if match_precision in {"same_pair_relation_visible", "pair_visible_only"}:
                notes.append(match_note)
            if alternate_relations:
                notes.append(
                    f"same room pair also has {len(alternate_relations)} alternate committed relation(s); these are visible for audit but are not automatically route-selected"
                )
            explanations.append(
                {
                    "step_index": idx,
                    "source_room": source,
                    "target_room": target,
                    "relation_type": edge.get("relation_type"),
                    "selected_edge_index": selected_edge_index,
                    "selected_relation_match_precision": match_precision,
                    "selected_relation_match_note": match_note,
                    "confidence": confidence,
                    "support_count": support_count,
                    "status": edge.get("status"),
                    "evidence_id_count": len(evidence_ids),
                    "evidence_ids": evidence_ids,
                    "evidence_types": semantic_summary.get("evidence_types"),
                    "evidence_source_refs": semantic_summary.get("evidence_source_refs"),
                    "evidence_type_summary": semantic_summary.get("evidence_type_summary"),
                    "evidence_source_summary": semantic_summary.get("evidence_source_summary"),
                    "selected_relation_semantic_labels": semantic_summary.get("semantic_labels"),
                    "selected_relation_semantic_summary": semantic_summary.get("semantic_summary"),
                    "same_floor": bool(source_floor == target_floor),
                    "cross_floor": bool(source_floor != target_floor),
                    "source_floor_id": source_floor,
                    "target_floor_id": target_floor,
                    "has_gateway_match": has_gateway_match,
                    "has_vertical_transition_match": has_vertical_transition_match,
                    "same_pair_relation_count": len(self.topology_edges_by_pair.get(pair or ("", ""), [])),
                    "same_pair_alternate_relation_count": len(alternate_relations),
                    "same_pair_alternate_relations": alternate_relations,
                    "same_pair_alternate_relation_present": bool(alternate_relations),
                    "visible_audit_overlay_count": 0,
                    "visible_selected_relation_overlay_count": 0,
                    "visible_same_pair_alternate_overlay_count": 0,
                    "audit_overlay_summary": "Audit overlays were not requested.",
                    "notes": notes,
                }
            )
        return explanations

    def _annotate_route_edge_explanations_with_audit_overlays(
        self,
        route_edge_explanations: Sequence[Dict[str, Any]],
        audit_overlay_records: Sequence[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        overlay_by_pair: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
        for item in audit_overlay_records:
            room_a = item.get("room_a")
            room_b = item.get("room_b")
            if room_a and room_b:
                overlay_by_pair[_pair_key(room_a, room_b)].append(item)
        annotated = []
        for item in route_edge_explanations:
            pair = _pair_key(item.get("source_room"), item.get("target_room"))
            overlays = overlay_by_pair.get(pair, [])
            selected_count = sum(1 for record in overlays if record.get("route_visibility_label") == "route_selected_weak_relation")
            alternate_count = sum(
                1
                for record in overlays
                if record.get("route_visibility_label") in {"same_pair_alternate_relation_visible", "same_pair_relation_visible"}
            )
            updated = dict(item)
            updated["visible_audit_overlay_count"] = len(overlays)
            updated["visible_selected_relation_overlay_count"] = selected_count
            updated["visible_same_pair_alternate_overlay_count"] = alternate_count
            if not overlays:
                updated["audit_overlay_summary"] = "No visible audit overlay for this route relation."
            elif selected_count:
                updated["audit_overlay_summary"] = (
                    f"{selected_count} overlay row(s) match the selected weak relation; {alternate_count} same-pair alternate overlay row(s) remain non-authoritative."
                )
            elif alternate_count:
                updated["audit_overlay_summary"] = (
                    f"Only same-pair overlay rows are visible ({alternate_count}); they are not selected route relations."
                )
            else:
                updated["audit_overlay_summary"] = (
                    f"{len(overlays)} overlay row(s) are visible for this pair, but none changes routing or becomes authoritative."
                )
            annotated.append(updated)
        return annotated

    def _semantic_room_summary_records(
        self,
        *,
        start_room_id: Optional[str],
        goal_room_id: Optional[str],
        room_sequence: Sequence[str],
        semantic_candidates: Sequence[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        candidate_ids = {str(item.get("room_id")) for item in semantic_candidates}
        records = []
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            room_model = self.room_model_by_id.get(room_id, {})
            summary = dict(room_model.get("semantic_summary") or {})
            label_counts = dict(summary.get("object_label_counts") or {})
            records.append(
                {
                    "room_id": room_id,
                    "floor_id": room_record.get("floor_id") or room_model.get("floor_id"),
                    "display_floor_id": room_record.get("display_floor_id"),
                    "room_type": room_record.get("room_type") or room_model.get("room_type"),
                    "object_count": summary.get("object_count", len(room_model.get("object_ids") or [])),
                    "anchor_count": summary.get("anchor_count", len(room_model.get("anchor_ids") or [])),
                    "dominant_object_labels": _dominant_labels(room_model, limit=8),
                    "object_label_counts": label_counts,
                    "neighbor_room_ids": [canonical_room_id(item) for item in (room_model.get("neighbor_room_ids") or [])],
                    "is_on_selected_route": room_id in room_sequence,
                    "is_start": room_id == start_room_id,
                    "is_goal": room_id == goal_room_id,
                    "is_semantic_target_candidate": room_id in candidate_ids,
                }
            )
        return records

    def _semantic_target_evidence(
        self,
        *,
        semantic_target: Optional[str],
        goal_resolution: Dict[str, Any],
        route: Dict[str, Any],
    ) -> Dict[str, Any]:
        candidates = list(goal_resolution.get("candidate_matches") or [])
        candidate_records = []
        for candidate in candidates:
            room_id = str(candidate.get("room_id"))
            room_model = self.room_model_by_id.get(room_id, {})
            summary = dict(room_model.get("semantic_summary") or {})
            matched_labels = sorted(
                {
                    str(term.get("term"))
                    for term in candidate.get("matched_terms", [])
                    if str(term.get("source")) in {"object_label", "dominant_label", "semantic_landmark"}
                }
            )
            candidate_records.append(
                {
                    "room_id": room_id,
                    "ranking_score": candidate.get("score"),
                    "matched_object_labels": matched_labels,
                    "object_label_counts": summary.get("object_label_counts") or {},
                    "object_count": summary.get("object_count", len(room_model.get("object_ids") or [])),
                }
            )
        return {
            "query_target": semantic_target,
            "candidate_rooms": candidate_records,
            "selected_goal_room": goal_resolution.get("resolved_room_id"),
            "route_room_sequence": list(route.get("room_sequence") or []),
        }

    def _audit_overlay_records(
        self,
        audit_dir: Optional[Path],
        route_edge_explanations: Sequence[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        if audit_dir is None:
            return []
        audit_dir = Path(audit_dir)
        records: List[Dict[str, Any]] = []
        route_by_pair = {
            _pair_key(item.get("source_room"), item.get("target_room")): dict(item)
            for item in route_edge_explanations
            if item.get("source_room") and item.get("target_room")
        }
        for row in _load_csv_rows(audit_dir / "topology_spurious_edge_candidates.csv"):
            if row.get("candidate_type") != "low_support_edge":
                continue
            room_a = canonical_room_id(row.get("source_room"))
            room_b = canonical_room_id(row.get("target_room"))
            pair = None if not room_a or not room_b else _pair_key(room_a, room_b)
            route_item = route_by_pair.get(pair or ("", ""))
            route_visibility_label = "off_route_overlay"
            route_visibility_note = "This low-support committed edge is not on the selected route."
            if route_item is not None:
                selected_edge_index = _as_int(route_item.get("selected_edge_index"))
                row_edge_index = _as_int(row.get("edge_index"))
                if selected_edge_index is not None and row_edge_index == selected_edge_index:
                    route_visibility_label = "route_selected_weak_relation"
                    route_visibility_note = "This overlay row matches the actual selected weak route relation."
                elif route_item.get("selected_relation_match_precision") in {"same_pair_relation_visible", "pair_visible_only"}:
                    route_visibility_label = "same_pair_relation_visible"
                    route_visibility_note = (
                        "This overlay row shares a selected route room pair, but relation-specific metadata is insufficient for a stronger match."
                    )
                else:
                    route_visibility_label = "same_pair_alternate_relation_visible"
                    route_visibility_note = "This overlay row is a same-pair alternate relation, not the selected route relation."
            records.append(
                {
                    "overlay_kind": "low_support_committed_edge",
                    "room_a": room_a,
                    "room_b": room_b,
                    "relation_type": row.get("relation_type"),
                    "candidate_type": row.get("candidate_type"),
                    "presentation_candidate_type": "weak_low_support_committed_topology_edge",
                    "candidate_label": route_visibility_label,
                    "severity": row.get("severity"),
                    "evidence_source": str(audit_dir / "topology_spurious_edge_candidates.csv"),
                    "recommended_visual_check": "inspect committed edge support only; this overlay does not add, remove, or reroute topology",
                    "notes": row.get("notes"),
                    "route_visibility_label": route_visibility_label,
                    "route_visibility_note": route_visibility_note,
                    "authoritative_status": "non_authoritative_overlay_only",
                    "adds_topology_edge": False,
                }
            )
        for row in _load_csv_rows(audit_dir / "topology_missing_edge_candidates.csv"):
            candidate_type = row.get("candidate_type")
            if candidate_type == "geometry_close_no_edge":
                presentation_candidate_type = "geometry_close_no_edge"
                label = "weak_geometry_candidate_non_authoritative"
                overlay_kind = "geometry_close_no_edge_candidate"
                paper_safety_note = "Weak geometry candidate only; do not treat as a topology repair target by itself."
            elif candidate_type in {"missing_edge_candidate_neighbor_mismatch", "world_model_neighbor_without_topology_edge"}:
                presentation_candidate_type = "world_model_neighbor_without_topology_edge"
                label = "world_model_neighbor_without_topology_edge"
                overlay_kind = "world_model_neighbor_no_topology_edge_candidate"
                paper_safety_note = "Room-summary or spatial-neighbor relation only; not automatically a missing committed topology edge."
            elif candidate_type in {
                "missing_edge_candidate_gateway_without_topology_edge",
                "navigable_gateway_missing_topology_edge",
            }:
                presentation_candidate_type = "navigable_gateway_missing_topology_edge"
                label = "navigable_gateway_missing_topology_edge"
                overlay_kind = "missing_connectivity_candidate"
                paper_safety_note = "Gateway-backed navigability evidence deserves inspection, but the overlay remains non-authoritative."
            elif candidate_type in {
                "missing_edge_candidate_vertical_transition_without_topology_edge",
                "vertical_transition_missing_topology_edge",
            }:
                presentation_candidate_type = "vertical_transition_missing_topology_edge"
                label = "vertical_transition_missing_topology_edge"
                overlay_kind = "missing_connectivity_candidate"
                paper_safety_note = "Cross-floor vertical-transition evidence deserves inspection, but same-floor weak adjacency does not require this evidence."
            else:
                presentation_candidate_type = candidate_type or "missing_connectivity_candidate"
                label = presentation_candidate_type
                overlay_kind = "missing_connectivity_candidate"
                paper_safety_note = "Diagnostic candidate only; manual review is still required."
            records.append(
                {
                    "overlay_kind": overlay_kind,
                    "room_a": canonical_room_id(row.get("room_a")),
                    "room_b": canonical_room_id(row.get("room_b")),
                    "candidate_type": candidate_type,
                    "presentation_candidate_type": presentation_candidate_type,
                    "candidate_label": label,
                    "severity": row.get("severity"),
                    "evidence_source": row.get("evidence_source") or str(audit_dir / "topology_missing_edge_candidates.csv"),
                    "recommended_visual_check": "manual inspection only; this candidate is not authoritative topology and does not change routing",
                    "notes": row.get("notes"),
                    "route_visibility_label": "not_a_selected_route_relation",
                    "route_visibility_note": "Missing-edge overlays are inspection aids only and are not selected route relations.",
                    "paper_safety_note": paper_safety_note,
                    "authoritative_status": "non_authoritative_overlay_only",
                    "has_gateway_record": _truthy_csv(row.get("has_gateway_record")),
                    "has_vertical_transition_record": _truthy_csv(row.get("has_vertical_transition_record")),
                    "adds_topology_edge": False,
                }
            )
        return records

    def build_demo(
        self,
        *,
        start_room: Optional[str] = None,
        goal_room: Optional[str] = None,
        semantic_target: Optional[str] = None,
        route_policy: str = "balanced",
        title: Optional[str] = None,
        include_snapshot_overlays: bool = False,
        include_gateway_overlays: bool = False,
        include_vertical_transition_overlays: bool = False,
        include_route_edge_explanation: bool = False,
        include_semantic_room_summary: bool = False,
        include_audit_overlays: bool = False,
        audit_dir: Optional[Path] = None,
    ) -> Dict[str, Any]:
        if bool(goal_room) == bool(semantic_target):
            raise ValueError("Provide exactly one of goal_room or semantic_target.")
        if route_policy not in self.query_api.route_policy_presets:
            raise ValueError(f"Unknown route_policy {route_policy!r}.")

        start_resolution = self.resolve_start_room(start_room)
        if not start_resolution.get("resolved"):
            return {
                "ok": False,
                "sequence_id": self.sequence_id,
                "failure_reason": start_resolution.get("failure_reason"),
                "start_resolution": start_resolution,
            }

        goal_resolution = (
            self.resolve_goal_room(goal_room)
            if goal_room is not None
            else self.resolve_semantic_goal(semantic_target)
        )
        if not goal_resolution.get("resolved"):
            return {
                "ok": False,
                "sequence_id": self.sequence_id,
                "failure_reason": goal_resolution.get("failure_reason"),
                "start_resolution": start_resolution,
                "goal_resolution": goal_resolution,
            }

        start_room_id = str(start_resolution["resolved_room_id"])
        goal_room_id = str(goal_resolution["resolved_room_id"])
        policy = dict(self.query_api.route_policy_presets[route_policy])
        route = self.topology.find_room_path(
            start_room_id,
            goal_room_id,
            allowed_relations=policy.get("allowed_relations"),
            min_conf=float(policy.get("min_conf", 0.0)),
            method="shortest",
        )
        room_sequence = list(route.get("room_sequence", []))
        next_hop_room_id = room_sequence[1] if len(room_sequence) > 1 else (room_sequence[0] if room_sequence else None)
        path_pair_set = self._path_pair_set(room_sequence)
        snapshot_requested = include_snapshot_overlays or include_gateway_overlays or include_vertical_transition_overlays
        gateway_records = self._gateway_records(path_pair_set) if (snapshot_requested or include_gateway_overlays) else []
        vertical_transition_records = (
            self._vertical_transition_records(path_pair_set)
            if (snapshot_requested or include_vertical_transition_overlays)
            else []
        )
        route_edge_explanation = (
            self._route_edge_explanations(route, gateway_records, vertical_transition_records)
            if include_route_edge_explanation
            else []
        )
        semantic_candidates = list(goal_resolution.get("candidate_matches") or [])
        semantic_room_summaries = (
            self._semantic_room_summary_records(
                start_room_id=start_room_id,
                goal_room_id=goal_room_id,
                room_sequence=room_sequence,
                semantic_candidates=semantic_candidates,
            )
            if include_semantic_room_summary
            else []
        )
        semantic_target_evidence = self._semantic_target_evidence(
            semantic_target=semantic_target,
            goal_resolution=goal_resolution,
            route=route,
        )
        audit_overlay_records = (
            self._audit_overlay_records(audit_dir, route_edge_explanation) if include_audit_overlays else []
        )
        if route_edge_explanation:
            route_edge_explanation = self._annotate_route_edge_explanations_with_audit_overlays(
                route_edge_explanation,
                audit_overlay_records,
            )
        unresolved_id_warnings = self._normalization_warnings() if snapshot_requested else []
        presentation_semantics = _presentation_semantics_payload()
        route_matching_limitations = [
            item.get("selected_relation_match_note")
            for item in route_edge_explanation
            if item.get("selected_relation_match_precision") in {"same_pair_relation_visible", "pair_visible_only"}
            and item.get("selected_relation_match_note")
        ]
        per_room_cards = []
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            room_model = self.room_model_by_id.get(room_id, {})
            per_room_cards.append(
                {
                    "room_id": room_id,
                    "display_name": _room_display_name(room_id, room_record, room_model),
                    "floor_id": room_record.get("floor_id"),
                    "room_type": room_record.get("room_type"),
                    "semantic_lines": _room_semantic_lines(room_model),
                    "is_start": room_id == start_room_id,
                    "is_goal": room_id == goal_room_id,
                    "is_next_hop": room_id == next_hop_room_id,
                    "on_path": room_id in room_sequence,
                }
            )
        title_text = title or (
            f"Room-Graph VLN Demo: {self.sequence_id} | {start_room_id} -> {goal_room_id}"
            if goal_room is not None
            else f"Room-Graph VLN Demo: {self.sequence_id} | {start_room_id} -> {semantic_target}"
        )
        return {
            "ok": bool(route.get("found")),
            "version": ROOM_GRAPH_VLN_DEMO_VERSION,
            "enhanced_version": ROOM_GRAPH_VLN_ENHANCED_VERSION,
            "title": title_text,
            "sequence_id": self.sequence_id,
            "scene_id": self.sequence_id,
            "scene_root": None if self.artifact_paths.scene_root is None else str(self.artifact_paths.scene_root),
            "route_policy": route_policy,
            "artifacts": {
                "scene_root": None if self.artifact_paths.scene_root is None else str(self.artifact_paths.scene_root),
                "summary_json": None if self.artifact_paths.summary_json is None else str(self.artifact_paths.summary_json),
                "topology_json": str(self.artifact_paths.topology_json),
                "committed_room_world_model_json": str(self.artifact_paths.committed_room_world_model_json),
                "committed_room_world_snapshot_json": (
                    None
                    if self.artifact_paths.committed_room_world_snapshot_json is None
                    else str(self.artifact_paths.committed_room_world_snapshot_json)
                ),
                "audit_dir": None if audit_dir is None else str(audit_dir),
            },
            "contract": {
                "public_topology_source": "topology_v0_1.json",
                "semantic_summary_source": "committed_room_world_model_v0_1.json",
                "snapshot_overlay_source": "committed_room_world_snapshot_v0_1.json" if snapshot_requested else None,
                "audit_overlay_source": "topology_audit CSV files" if include_audit_overlays else None,
                "audit_overlay_authoritative": False,
                "working_state_used_for_routing": False,
                "continuous_navigation_control": False,
                "bev_used": False,
            },
            "enhanced_features": {
                "include_snapshot_overlays": bool(snapshot_requested),
                "include_gateway_overlays": bool(include_gateway_overlays or snapshot_requested),
                "include_vertical_transition_overlays": bool(include_vertical_transition_overlays or snapshot_requested),
                "include_route_edge_explanation": bool(include_route_edge_explanation),
                "include_semantic_room_summary": bool(include_semantic_room_summary),
                "include_audit_overlays": bool(include_audit_overlays),
            },
            "start_resolution": start_resolution,
            "goal_resolution": goal_resolution,
            "selected_start_room": start_room_id,
            "selected_goal_room": goal_room_id,
            "route": route,
            "route_room_sequence": room_sequence,
            "next_hop": {
                "room_id": next_hop_room_id,
                "relation_type": None if len(route.get("edges", [])) == 0 else route["edges"][0].get("relation_type"),
                "summary": (
                    None
                    if next_hop_room_id is None
                    else (
                        "Already in goal room."
                        if len(room_sequence) <= 1 and next_hop_room_id == goal_room_id
                        else f"Go next to {next_hop_room_id}."
                    )
                ),
            },
            "public_topology_summary": {
                "room_count": len(self.public_room_ids),
                "edge_count": len(self.topology_payload.get("edges", [])),
                "public_room_ids": self.public_room_ids,
            },
            "presentation_semantics": presentation_semantics,
            "route_relation_matching_limitations": route_matching_limitations,
            "per_room_cards": per_room_cards,
            "path_pair_set": [list(item) for item in sorted(path_pair_set)],
            "route_edge_explanation": route_edge_explanation,
            "gateway_overlay_records": gateway_records,
            "vertical_transition_overlay_records": vertical_transition_records,
            "semantic_room_summary_records": semantic_room_summaries,
            "semantic_target_evidence": semantic_target_evidence,
            "audit_overlay_records": audit_overlay_records,
            "audit_overlay_disclaimer": AUDIT_OVERLAY_DISCLAIMER if include_audit_overlays else None,
            "unresolved_id_warnings": unresolved_id_warnings,
        }

    def _floor_color_lookup(self) -> Dict[str, str]:
        ordered_floor_ids = sorted(
            self.floor_records,
            key=lambda floor_id: (
                int((self.floor_records.get(floor_id) or {}).get("display_order", 10**6)),
                str(floor_id),
            ),
        )
        return {
            floor_id: DEFAULT_FLOOR_COLORS[idx % len(DEFAULT_FLOOR_COLORS)]
            for idx, floor_id in enumerate(ordered_floor_ids)
        }

    def _map_geometry(self, width: int, height: int) -> Dict[str, Any]:
        polygons: List[List[List[float]]] = []
        centers: List[List[float]] = []
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            polygon = [list(map(float, point)) for point in room_record.get("polygon", []) if len(point) >= 2]
            center = room_record.get("center")
            if polygon:
                polygons.append(polygon)
            if isinstance(center, (list, tuple)) and len(center) >= 2:
                centers.append([float(center[0]), float(center[1])])
        if polygons:
            xs = [point[0] for polygon in polygons for point in polygon]
            ys = [point[1] for polygon in polygons for point in polygon]
        elif centers:
            xs = [point[0] for point in centers]
            ys = [point[1] for point in centers]
        else:
            xs = [0.0, 1.0]
            ys = [0.0, 1.0]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(0.001, max_x - min_x)
        span_y = max(0.001, max_y - min_y)
        padding = 38.0
        scale = min((width - 2.0 * padding) / span_x, (height - 2.0 * padding) / span_y)
        scaled_width = span_x * scale
        scaled_height = span_y * scale
        x_offset = (width - scaled_width) / 2.0
        y_offset = (height - scaled_height) / 2.0

        def project(point: Sequence[float]) -> Tuple[float, float]:
            x = x_offset + (float(point[0]) - min_x) * scale
            y = height - (y_offset + (float(point[1]) - min_y) * scale)
            return (round(x, 2), round(y, 2))

        return {"project": project, "bounds": [min_x, min_y, max_x, max_y]}

    def render_map_svg(self, demo_result: Dict[str, Any], *, width: int = 900, height: int = 620) -> str:
        floor_colors = self._floor_color_lookup()
        geometry = self._map_geometry(width, height)
        project = geometry["project"]
        room_sequence = list((demo_result.get("route") or {}).get("room_sequence", []))
        path_pair_set = {
            tuple(sorted(item)) for item in demo_result.get("path_pair_set", [])
        }
        start_room_id = (demo_result.get("start_resolution") or {}).get("resolved_room_id")
        goal_room_id = (demo_result.get("goal_resolution") or {}).get("resolved_room_id")
        next_hop_room_id = (demo_result.get("next_hop") or {}).get("room_id")

        elements: List[str] = [
            f'<svg class="viz-svg" viewBox="0 0 {width} {height}" role="img" aria-label="Spatial room graph view">'
        ]

        relation_by_pair: Dict[Tuple[str, str], str] = {}
        for edge in self.topology_payload.get("edges", []):
            source = str(edge.get("source"))
            target = str(edge.get("target"))
            pair = tuple(sorted((source, target)))
            existing = relation_by_pair.get(pair)
            preferred = _preferred_relation_type((existing, edge.get("relation_type")))
            if preferred is not None:
                relation_by_pair[pair] = preferred

        for pair, relation_type in sorted(relation_by_pair.items()):
            source_room = self.topology.get_room(pair[0]) or {}
            target_room = self.topology.get_room(pair[1]) or {}
            source_center = source_room.get("center")
            target_center = target_room.get("center")
            if not source_center or not target_center:
                continue
            x1, y1 = project(source_center)
            x2, y2 = project(target_center)
            is_path = pair in path_pair_set
            dash = ' stroke-dasharray="8 6"' if relation_type == "vertical_transition" else ""
            elements.append(
                '<line class="edge" '
                f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{_relation_color(relation_type, highlight=is_path)}" '
                f'stroke-width="{"7" if is_path else "3"}" opacity="0.8"{dash} />'
            )

        for item in demo_result.get("audit_overlay_records", []) or []:
            room_a = item.get("room_a")
            room_b = item.get("room_b")
            if not room_a or not room_b:
                continue
            source_center = _room_center(self.topology.get_room(room_a) or {}, self.room_model_by_id.get(room_a))
            target_center = _room_center(self.topology.get_room(room_b) or {}, self.room_model_by_id.get(room_b))
            if not source_center or not target_center:
                continue
            x1, y1 = project(source_center)
            x2, y2 = project(target_center)
            if item.get("overlay_kind") == "low_support_committed_edge":
                color = "#b42318"
                dash = ' stroke-dasharray="4 4"'
                width_value = "5"
            elif item.get("overlay_kind") == "geometry_close_no_edge_candidate":
                color = "#8a6f15"
                dash = ' stroke-dasharray="2 8"'
                width_value = "2"
            else:
                color = "#6b7280"
                dash = ' stroke-dasharray="10 7"'
                width_value = "2.5"
            elements.append(
                '<line class="audit-overlay-line" '
                f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{color}" stroke-width="{width_value}" opacity="0.72"{dash} />'
            )

        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            polygon = [list(map(float, point)) for point in room_record.get("polygon", []) if len(point) >= 2]
            if not polygon:
                continue
            points = " ".join(f"{x},{y}" for x, y in (project(point) for point in polygon))
            floor_color = floor_colors.get(str(room_record.get("floor_id")), "#d8dee9")
            is_start = room_id == start_room_id
            is_goal = room_id == goal_room_id
            on_path = room_id in room_sequence
            stroke = "#1f2937"
            stroke_width = 2
            if on_path:
                stroke = "#e4572e"
                stroke_width = 4
            if is_start:
                stroke = "#1b9e77"
                stroke_width = 5
            if is_goal:
                stroke = "#d7263d"
                stroke_width = 5
            fill_opacity = "0.42" if on_path else "0.24"
            elements.append(
                f'<polygon points="{points}" fill="{floor_color}" fill-opacity="{fill_opacity}" '
                f'stroke="{stroke}" stroke-width="{stroke_width}" />'
            )

        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            center = room_record.get("center")
            if not center:
                continue
            x, y = project(center)
            is_start = room_id == start_room_id
            is_goal = room_id == goal_room_id
            is_next_hop = room_id == next_hop_room_id
            node_fill = "#ffffff"
            node_stroke = "#334155"
            node_radius = 9
            if is_start:
                node_fill = "#1b9e77"
                node_stroke = "#0b5d46"
                node_radius = 12
            elif is_goal:
                node_fill = "#d7263d"
                node_stroke = "#7c1d2b"
                node_radius = 12
            elif is_next_hop:
                node_fill = "#ff9f1c"
                node_stroke = "#9a5c00"
                node_radius = 11
            elements.append(
                f'<circle cx="{x}" cy="{y}" r="{node_radius}" fill="{node_fill}" stroke="{node_stroke}" stroke-width="3" />'
            )
            room_model = self.room_model_by_id.get(room_id, {})
            line_1 = _escape(room_id)
            semantic_preview = ", ".join(_dominant_labels(room_model, limit=2))
            line_2 = _escape(semantic_preview) if semantic_preview else _escape(str(room_record.get("floor_id") or ""))
            elements.append(
                f'<text x="{x + 12}" y="{y - 4}" class="room-label-primary">{line_1}</text>'
                f'<text x="{x + 12}" y="{y + 14}" class="room-label-secondary">{line_2}</text>'
            )

        for item in demo_result.get("vertical_transition_overlay_records", []) or []:
            from_xy = item.get("from_position_xy")
            to_xy = item.get("to_position_xy")
            if not from_xy or not to_xy:
                continue
            x1, y1 = project(from_xy)
            x2, y2 = project(to_xy)
            stroke = "#6d28d9" if item.get("on_selected_route") else "#8b5cf6"
            elements.append(
                '<line class="vertical-transition-overlay" '
                f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{stroke}" stroke-width="{"7" if item.get("on_selected_route") else "4"}" '
                'opacity="0.86" stroke-dasharray="3 8" />'
            )
            mid_x = round((x1 + x2) / 2.0, 2)
            mid_y = round((y1 + y2) / 2.0, 2)
            elements.append(
                f'<rect x="{mid_x - 12}" y="{mid_y - 12}" width="24" height="24" rx="5" fill="#faf5ff" stroke="{stroke}" stroke-width="3" />'
                f'<text x="{mid_x}" y="{mid_y + 4}" text-anchor="middle" class="gateway-label">VT</text>'
            )

        for item in demo_result.get("gateway_overlay_records", []) or []:
            point = item.get("plot_xy")
            if not point:
                continue
            x, y = project(point)
            fill = "#0f766e" if item.get("on_selected_route") else "#14b8a6"
            elements.append(
                f'<rect x="{x - 8}" y="{y - 8}" width="16" height="16" rx="3" fill="{fill}" stroke="#134e4a" stroke-width="2" opacity="0.92" />'
                f'<text x="{x + 12}" y="{y + 4}" class="gateway-label">{_escape(item.get("gateway_type") or "gateway")}</text>'
            )

        elements.append("</svg>")
        return "\n".join(elements)

    def render_floor_topology_sections(self, demo_result: Dict[str, Any], *, width: int = 760, height: int = 430) -> str:
        geometry = self._map_geometry(width, height)
        project = geometry["project"]
        floor_colors = self._floor_color_lookup()
        room_sequence = list((demo_result.get("route") or {}).get("room_sequence", []))
        path_pair_set = self._route_pair_set(demo_result)
        floors: Dict[str, List[str]] = defaultdict(list)
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            floors[str(room_record.get("floor_id") or "unknown_floor")].append(room_id)
        panels = []
        for floor_id, room_ids in sorted(floors.items()):
            elements = [
                f'<svg class="viz-svg floor-svg" viewBox="0 0 {width} {height}" role="img" aria-label="Floor-separated committed topology for {floor_id}">'
            ]
            for edge in self.topology_payload.get("edges", []) or []:
                source = canonical_room_id(edge.get("source"))
                target = canonical_room_id(edge.get("target"))
                if source not in room_ids or target not in room_ids:
                    continue
                source_center = _room_center(self.topology.get_room(source) or {}, self.room_model_by_id.get(source))
                target_center = _room_center(self.topology.get_room(target) or {}, self.room_model_by_id.get(target))
                if not source_center or not target_center:
                    continue
                x1, y1 = project(source_center)
                x2, y2 = project(target_center)
                pair = _pair_key(source, target)
                elements.append(
                    '<line '
                    f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                    f'stroke="{_relation_color(edge.get("relation_type"), highlight=pair in path_pair_set)}" '
                    f'stroke-width="{"6" if pair in path_pair_set else "2.5"}" opacity="0.78" />'
                )
            for room_id in room_ids:
                room_record = self.topology.get_room(room_id) or {}
                polygon = [list(map(float, point)) for point in room_record.get("polygon", []) if len(point) >= 2]
                center = _room_center(room_record, self.room_model_by_id.get(room_id))
                if polygon:
                    points = " ".join(f"{x},{y}" for x, y in (project(point) for point in polygon))
                    elements.append(
                        f'<polygon points="{points}" fill="{floor_colors.get(floor_id, "#d8dee9")}" '
                        f'fill-opacity="{"0.44" if room_id in room_sequence else "0.22"}" '
                        f'stroke="{"#e4572e" if room_id in room_sequence else "#334155"}" stroke-width="{"4" if room_id in room_sequence else "2"}" />'
                    )
                if center:
                    x, y = project(center)
                    elements.append(
                        f'<circle cx="{x}" cy="{y}" r="8" fill="#ffffff" stroke="#243b53" stroke-width="2" />'
                        f'<text x="{x + 11}" y="{y + 4}" class="room-label-primary">{_escape(room_id)}</text>'
                    )
            elements.append("</svg>")
            panels.append(
                '<article class="floor-panel">'
                f'<h3>{_escape(floor_id)}</h3>'
                f'<div class="note">Committed rooms: {_escape(len(room_ids))}. Route rooms on this floor: {_escape(len([room_id for room_id in room_ids if room_id in room_sequence]))}.</div>'
                f'{"".join(elements)}'
                "</article>"
            )
        return "".join(panels)

    def render_topology_svg(self, demo_result: Dict[str, Any], *, width: int = 900, height: int = 620) -> str:
        graph = nx.Graph()
        relation_by_pair: Dict[Tuple[str, str], List[str]] = {}
        for room_id in self.public_room_ids:
            graph.add_node(room_id)
        for edge in self.topology_payload.get("edges", []):
            source = str(edge.get("source"))
            target = str(edge.get("target"))
            if source not in graph.nodes or target not in graph.nodes:
                continue
            pair = tuple(sorted((source, target)))
            relation_by_pair.setdefault(pair, []).append(str(edge.get("relation_type") or ""))
            graph.add_edge(source, target)
        if graph.number_of_nodes() == 0:
            return f'<svg class="viz-svg" viewBox="0 0 {width} {height}"></svg>'
        positions = nx.spring_layout(graph, seed=7, k=max(0.5, 2.8 / math.sqrt(max(1, graph.number_of_nodes()))))
        xs = [float(point[0]) for point in positions.values()]
        ys = [float(point[1]) for point in positions.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(0.001, max_x - min_x)
        span_y = max(0.001, max_y - min_y)
        padding = 70.0

        def project(point: Sequence[float]) -> Tuple[float, float]:
            x = padding + (float(point[0]) - min_x) * ((width - 2.0 * padding) / span_x)
            y = padding + (float(point[1]) - min_y) * ((height - 2.0 * padding) / span_y)
            return (round(x, 2), round(y, 2))

        start_room_id = (demo_result.get("start_resolution") or {}).get("resolved_room_id")
        goal_room_id = (demo_result.get("goal_resolution") or {}).get("resolved_room_id")
        next_hop_room_id = (demo_result.get("next_hop") or {}).get("room_id")
        room_sequence = list((demo_result.get("route") or {}).get("room_sequence", []))
        path_pair_set = {
            tuple(sorted(item)) for item in demo_result.get("path_pair_set", [])
        }
        floor_colors = self._floor_color_lookup()
        elements: List[str] = [
            f'<svg class="viz-svg" viewBox="0 0 {width} {height}" role="img" aria-label="Abstract topology graph view">'
        ]
        for pair, relation_types in sorted(relation_by_pair.items()):
            x1, y1 = project(positions[pair[0]])
            x2, y2 = project(positions[pair[1]])
            relation_type = _preferred_relation_type(relation_types)
            is_path = pair in path_pair_set
            dash = ' stroke-dasharray="10 8"' if relation_type == "vertical_transition" else ""
            elements.append(
                '<line '
                f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{_relation_color(relation_type, highlight=is_path)}" '
                f'stroke-width="{"8" if is_path else "3"}" opacity="0.84"{dash} />'
            )
        for room_id in self.public_room_ids:
            room_record = self.topology.get_room(room_id) or {}
            x, y = project(positions[room_id])
            floor_color = floor_colors.get(str(room_record.get("floor_id")), "#d8dee9")
            radius = 34
            stroke = "#243b53"
            fill = floor_color
            if room_id == start_room_id:
                stroke = "#1b9e77"
                radius = 40
            elif room_id == goal_room_id:
                stroke = "#d7263d"
                radius = 40
            elif room_id == next_hop_room_id:
                stroke = "#ff9f1c"
                radius = 37
            elif room_id in room_sequence:
                stroke = "#e4572e"
                radius = 36
            elements.append(
                f'<circle cx="{x}" cy="{y}" r="{radius}" fill="{fill}" fill-opacity="0.8" stroke="{stroke}" stroke-width="5" />'
            )
            room_model = self.room_model_by_id.get(room_id, {})
            semantic_preview = ", ".join(_dominant_labels(room_model, limit=1))
            elements.append(
                f'<text x="{x}" y="{y - 4}" text-anchor="middle" class="topology-label-primary">{_escape(room_id)}</text>'
                f'<text x="{x}" y="{y + 15}" text-anchor="middle" class="topology-label-secondary">{_escape(semantic_preview)}</text>'
            )
        elements.append("</svg>")
        return "\n".join(elements)

    def render_html(self, demo_result: Dict[str, Any]) -> str:
        start_room_id = (demo_result.get("start_resolution") or {}).get("resolved_room_id")
        goal_room_id = (demo_result.get("goal_resolution") or {}).get("resolved_room_id")
        goal_mode = (demo_result.get("goal_resolution") or {}).get("goal_mode")
        route = dict(demo_result.get("route") or {})
        room_sequence = list(route.get("room_sequence", []))
        candidate_matches = list((demo_result.get("goal_resolution") or {}).get("candidate_matches", []))
        presentation_semantics = dict(demo_result.get("presentation_semantics") or {})
        map_svg = self.render_map_svg(demo_result)
        topology_svg = self.render_topology_svg(demo_result)
        floor_topology_sections = self.render_floor_topology_sections(demo_result)
        room_cards = []
        semantic_summary_lookup = {
            item.get("room_id"): item for item in demo_result.get("semantic_room_summary_records", [])
        }
        for item in demo_result.get("per_room_cards", []):
            badges = []
            if item.get("is_start"):
                badges.append("start")
            if item.get("is_goal"):
                badges.append("goal")
            if item.get("is_next_hop"):
                badges.append("next-hop")
            if item.get("on_path") and not badges:
                badges.append("path")
            summary_record = semantic_summary_lookup.get(item.get("room_id"), {})
            if summary_record.get("is_semantic_target_candidate"):
                badges.append("semantic-target")
            badge_html = "".join(f'<span class="badge">{_escape(badge)}</span>' for badge in badges)
            semantic_lines = item.get("semantic_lines") or ["no semantic summary"]
            if summary_record:
                semantic_lines = [
                    f"objects: {summary_record.get('object_count')}",
                    f"anchors: {summary_record.get('anchor_count')}",
                    f"summary neighbors: {', '.join(summary_record.get('neighbor_room_ids') or []) or 'none'}",
                ] + semantic_lines
            semantic_html = "".join(f"<li>{_escape(line)}</li>" for line in semantic_lines)
            room_cards.append(
                '<div class="room-card">'
                f'<div class="room-card-title">{_escape(item.get("display_name"))}</div>'
                f'<div class="room-card-meta">{_escape(item.get("floor_id"))} {badge_html}</div>'
                f'<ul class="room-card-list">{semantic_html}</ul>'
                "</div>"
            )
        candidate_rows = []
        for item in candidate_matches:
            matched_terms = ", ".join(
                f"{entry.get('term')} ({entry.get('source')})" for entry in item.get("matched_terms", [])
            )
            candidate_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('room_id'))}</td>"
                f"<td>{_escape(item.get('score'))}</td>"
                f"<td>{_escape(matched_terms)}</td>"
                "</tr>"
            )
        if not candidate_rows:
            candidate_rows.append('<tr><td colspan="3">No semantic ranking table for this query.</td></tr>')
        route_steps = []
        if room_sequence:
            for idx, room_id in enumerate(room_sequence):
                prefix = "start" if idx == 0 else ("goal" if idx == len(room_sequence) - 1 else f"hop {idx}")
                route_steps.append(f"<li><strong>{_escape(prefix)}:</strong> {_escape(room_id)}</li>")
        else:
            route_steps.append("<li>No path found.</li>")
        edge_rows = []
        for item in demo_result.get("route_edge_explanation", []) or []:
            notes = "; ".join(item.get("notes") or [])
            alternate_bits = []
            for alternate in item.get("same_pair_alternate_relations") or []:
                alternate_bits.append(
                    f"{alternate.get('relation_type')} (edge {alternate.get('edge_index')}, {alternate.get('semantic_summary')})"
                )
            alternate_summary = "; ".join(alternate_bits) or "none"
            audit_overlay_summary = item.get("audit_overlay_summary") or "none"
            edge_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('step_index'))}</td>"
                f"<td>{_escape(item.get('source_room'))} -> {_escape(item.get('target_room'))}</td>"
                f"<td>{_escape(item.get('relation_type'))}<br><span class='inline-note'>{_escape(item.get('selected_relation_match_precision'))}</span></td>"
                f"<td>{_escape(item.get('selected_relation_semantic_summary'))}</td>"
                f"<td>{_escape(item.get('evidence_type_summary'))}<br><span class='inline-note'>refs: {_escape(item.get('evidence_source_summary'))}</span></td>"
                f"<td>{_escape(', '.join(item.get('evidence_ids') or []) or 'none')}</td>"
                f"<td>{_escape(item.get('confidence'))}</td>"
                f"<td>{_escape(item.get('support_count'))}</td>"
                f"<td>{_escape(item.get('status'))}</td>"
                f"<td>{_escape('cross-floor' if item.get('cross_floor') else 'same-floor')}</td>"
                f"<td>{_escape(item.get('has_gateway_match'))}</td>"
                f"<td>{_escape(item.get('has_vertical_transition_match'))}</td>"
                f"<td>{_escape(alternate_summary)}</td>"
                f"<td>{_escape(audit_overlay_summary)}</td>"
                f"<td>{_escape(notes)}</td>"
                "</tr>"
            )
        if not edge_rows:
            edge_rows.append('<tr><td colspan="15">Route edge explanation was not requested or no path edge exists.</td></tr>')
        gateway_rows = []
        for item in demo_result.get("gateway_overlay_records", []) or []:
            gateway_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('room_a'))} - {_escape(item.get('room_b'))}</td>"
                f"<td>{_escape(item.get('gateway_type'))}</td>"
                f"<td>{_escape(item.get('width_m'))}</td>"
                f"<td>{_escape(item.get('floor_id'))}</td>"
                f"<td>{_escape(item.get('geometry_source'))}</td>"
                f"<td>{_escape(item.get('on_selected_route'))}</td>"
                "</tr>"
            )
        if not gateway_rows:
            gateway_rows.append('<tr><td colspan="6">No gateway overlay records loaded.</td></tr>')
        vertical_rows = []
        for item in demo_result.get("vertical_transition_overlay_records", []) or []:
            vertical_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('transition_id'))}</td>"
                f"<td>{_escape(item.get('type'))}</td>"
                f"<td>{_escape(item.get('status'))}</td>"
                f"<td>{_escape(item.get('confidence'))}</td>"
                f"<td>{_escape(item.get('from_room_id'))} ({_escape(item.get('from_floor_id'))}) -> {_escape(item.get('to_room_id'))} ({_escape(item.get('to_floor_id'))})</td>"
                f"<td>{_escape(item.get('connector_label'))}</td>"
                f"<td>{_escape(item.get('on_selected_route'))}</td>"
                f"<td>{_escape(item.get('evidence_summary'))}</td>"
                "</tr>"
            )
        if not vertical_rows:
            vertical_rows.append('<tr><td colspan="8">No vertical transition overlay records loaded.</td></tr>')
        semantic_target = demo_result.get("semantic_target_evidence") or {}
        semantic_evidence_rows = []
        for item in semantic_target.get("candidate_rooms", []) or []:
            semantic_evidence_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('room_id'))}</td>"
                f"<td>{_escape(item.get('ranking_score'))}</td>"
                f"<td>{_escape(', '.join(item.get('matched_object_labels') or []))}</td>"
                f"<td>{_escape(item.get('object_count'))}</td>"
                "</tr>"
            )
        if not semantic_evidence_rows:
            semantic_evidence_rows.append('<tr><td colspan="4">No semantic/object target evidence for this explicit-room query.</td></tr>')
        audit_rows = []
        for item in demo_result.get("audit_overlay_records", []) or []:
            audit_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('presentation_candidate_type') or item.get('overlay_kind'))}</td>"
                f"<td>{_escape(item.get('room_a'))} - {_escape(item.get('room_b'))}</td>"
                f"<td>{_escape(item.get('candidate_label'))}</td>"
                f"<td>{_escape(item.get('route_visibility_note'))}</td>"
                f"<td>{_escape(item.get('severity'))}</td>"
                f"<td>{_escape(item.get('evidence_source'))}</td>"
                f"<td>{_escape(item.get('recommended_visual_check'))}</td>"
                "</tr>"
            )
        if not audit_rows:
            audit_rows.append('<tr><td colspan="7">Audit candidate overlays are disabled or unavailable.</td></tr>')
        warning_rows = []
        for item in demo_result.get("unresolved_id_warnings", []) or []:
            warning_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('context'))}</td>"
                f"<td>{_escape(item.get('field'))}</td>"
                f"<td>{_escape(item.get('value'))}</td>"
                f"<td>{_escape(item.get('message'))}</td>"
                "</tr>"
            )
        if not warning_rows:
            warning_rows.append('<tr><td colspan="4">No unresolved ID warnings.</td></tr>')
        audit_disclaimer = demo_result.get("audit_overlay_disclaimer") or ""
        semantics_rows = []
        for item in presentation_semantics.get("legend_items") or []:
            semantics_rows.append(
                "<tr>"
                f"<td>{_escape(item.get('label'))}</td>"
                f"<td>{_escape(item.get('meaning'))}</td>"
                "</tr>"
            )
        if not semantics_rows:
            semantics_rows.append('<tr><td colspan="2">No presentation semantics metadata.</td></tr>')
        matching_limitations = [
            f"<li>{_escape(note)}</li>" for note in (demo_result.get("route_relation_matching_limitations") or [])
        ]
        if not matching_limitations:
            matching_limitations.append("<li>Route-selected labels had sufficient relation-specific metadata for the rendered path.</li>")
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>{_escape(demo_result.get("title"))}</title>
  <style>
    :root {{
      --bg: #f7f3ea;
      --panel: #fffdf8;
      --ink: #1f2937;
      --muted: #5b6471;
      --line: #d9d2c3;
      --accent: #e4572e;
      --accent-2: #1b9e77;
      --accent-3: #d7263d;
      --badge: #f4efe4;
      --shadow: 0 12px 32px rgba(31, 41, 55, 0.08);
      --font-sans: "Avenir Next", "Segoe UI", sans-serif;
      --font-mono: "SFMono-Regular", Consolas, monospace;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 24px;
      background:
        radial-gradient(circle at top left, rgba(242, 193, 78, 0.22), transparent 34%),
        radial-gradient(circle at top right, rgba(90, 169, 230, 0.18), transparent 28%),
        var(--bg);
      color: var(--ink);
      font-family: var(--font-sans);
    }}
    .shell {{
      max-width: 1600px;
      margin: 0 auto;
      display: grid;
      gap: 18px;
    }}
    .hero, .panel {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      box-shadow: var(--shadow);
    }}
    .hero {{
      padding: 24px;
    }}
    .hero h1 {{
      margin: 0 0 10px 0;
      font-size: 30px;
      line-height: 1.1;
    }}
    .subtitle {{
      color: var(--muted);
      font-size: 15px;
      max-width: 980px;
    }}
    .hero-grid {{
      margin-top: 18px;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 14px;
    }}
    .metric {{
      padding: 14px;
      border-radius: 16px;
      background: #faf6ed;
      border: 1px solid #ebe2d0;
    }}
    .metric-label {{
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }}
    .metric-value {{
      margin-top: 6px;
      font-size: 20px;
      font-weight: 700;
    }}
    .layout {{
      display: grid;
      grid-template-columns: minmax(0, 1.45fr) minmax(320px, 0.85fr);
      gap: 18px;
    }}
    .stack {{
      display: grid;
      gap: 18px;
    }}
    .panel {{
      padding: 18px;
    }}
    .panel h2 {{
      margin: 0 0 6px 0;
      font-size: 19px;
    }}
    .panel .note {{
      color: var(--muted);
      font-size: 14px;
      margin-bottom: 12px;
    }}
    .viz-svg {{
      width: 100%;
      height: auto;
      border-radius: 14px;
      background:
        linear-gradient(180deg, rgba(255,255,255,0.96), rgba(248,245,238,0.92)),
        repeating-linear-gradient(0deg, transparent, transparent 23px, rgba(31, 41, 55, 0.04) 24px),
        repeating-linear-gradient(90deg, transparent, transparent 23px, rgba(31, 41, 55, 0.04) 24px);
      border: 1px solid #ece5d6;
    }}
    .room-label-primary {{
      font-size: 13px;
      font-weight: 700;
      fill: #14213d;
    }}
    .room-label-secondary {{
      font-size: 11px;
      fill: #415164;
    }}
    .topology-label-primary {{
      font-size: 15px;
      font-weight: 700;
      fill: #14213d;
    }}
    .topology-label-secondary {{
      font-size: 11px;
      fill: #415164;
    }}
    .gateway-label {{
      font-size: 10px;
      font-weight: 700;
      fill: #12343b;
    }}
    .pill-row {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-top: 10px;
    }}
    .pill, .badge {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      background: var(--badge);
      border: 1px solid #e3d9c8;
      padding: 5px 9px;
      color: #354052;
      font-size: 12px;
      line-height: 1;
    }}
    .badge {{
      margin-left: 6px;
    }}
    .artifact-list, .route-list {{
      margin: 0;
      padding-left: 18px;
      color: var(--muted);
    }}
    .inline-note {{
      color: var(--muted);
      font-size: 11px;
    }}
    .artifact-list code, .route-list code {{
      font-family: var(--font-mono);
      font-size: 12px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    th, td {{
      text-align: left;
      padding: 8px 10px;
      border-bottom: 1px solid #eee5d8;
      vertical-align: top;
    }}
    th {{
      color: var(--muted);
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.04em;
      font-size: 11px;
    }}
    .room-card-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
    }}
    .room-card {{
      border: 1px solid #eadfcd;
      border-radius: 8px;
      background: #fcfaf5;
      padding: 14px;
    }}
    .room-card-title {{
      font-size: 15px;
      font-weight: 700;
    }}
    .room-card-meta {{
      margin-top: 4px;
      color: var(--muted);
      font-size: 12px;
    }}
    .room-card-list {{
      margin: 10px 0 0 18px;
      padding: 0;
      color: #425166;
      font-size: 13px;
    }}
    .floor-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
      gap: 12px;
    }}
    .floor-panel {{
      border: 1px solid #eadfcd;
      border-radius: 8px;
      padding: 12px;
      background: #fcfaf5;
    }}
    .floor-panel h3 {{
      margin: 0 0 6px 0;
      font-size: 16px;
    }}
    .audit-disclaimer {{
      border: 1px solid #f59e0b;
      background: #fffbeb;
      color: #713f12;
      border-radius: 8px;
      padding: 10px 12px;
      font-weight: 700;
      margin-bottom: 12px;
    }}
    @media (max-width: 1080px) {{
      .layout {{
        grid-template-columns: 1fr;
      }}
      body {{
        padding: 14px;
      }}
    }}
  </style>
</head>
<body>
  <div class="shell">
    <section class="hero">
      <h1>{_escape(demo_result.get("title"))}</h1>
      <div class="subtitle">
        Lightweight room-graph VLN visualization over the committed/public export only. Routing comes from the existing public topology; semantic goal resolution comes from committed room summaries.
      </div>
      <div class="hero-grid">
        <div class="metric"><div class="metric-label">Sequence</div><div class="metric-value">{_escape(demo_result.get("sequence_id"))}</div></div>
        <div class="metric"><div class="metric-label">Start Room</div><div class="metric-value">{_escape(start_room_id)}</div></div>
        <div class="metric"><div class="metric-label">Goal Mode</div><div class="metric-value">{_escape(goal_mode)}</div></div>
        <div class="metric"><div class="metric-label">Resolved Goal</div><div class="metric-value">{_escape(goal_room_id)}</div></div>
        <div class="metric"><div class="metric-label">Next Hop</div><div class="metric-value">{_escape((demo_result.get("next_hop") or {}).get("room_id"))}</div></div>
        <div class="metric"><div class="metric-label">Path</div><div class="metric-value">{_escape(" -> ".join(room_sequence) if room_sequence else "no path")}</div></div>
      </div>
      <div class="pill-row">
        <span class="pill">policy: {_escape(demo_result.get("route_policy"))}</span>
        <span class="pill">rooms: {_escape((demo_result.get("public_topology_summary") or {}).get("room_count"))}</span>
        <span class="pill">edges: {_escape((demo_result.get("public_topology_summary") or {}).get("edge_count"))}</span>
        <span class="pill">continuous control: disabled</span>
        <span class="pill">BEV: not used</span>
      </div>
    </section>

    <div class="layout">
      <div class="stack">
        <section class="panel">
          <h2>Spatial Public Room Graph</h2>
          <div class="note">Public rooms are drawn from <code>topology_v0_1.json</code>. Colored polygons show room footprints; lines show committed topology edges; the highlighted sequence is the inferred room path. Gateway, vertical-transition, and audit layers remain overlays rather than authoritative graph changes.</div>
          {map_svg}
        </section>
        <section class="panel">
          <h2>Abstract Topology Route</h2>
          <div class="note">This abstract graph highlights the same public room path and makes the next-hop decision easier to read in a paper/demo setting. Same-pair alternate relations may exist in the committed topology without being the actual selected route relation.</div>
          {topology_svg}
        </section>
        <section class="panel">
          <h2>Floor-Separated Committed Topology View</h2>
          <div class="note">Each floor panel uses committed topology rooms and edges only. Cross-floor vertical transitions are rendered separately as purple connector overlays. Same-floor weak adjacency does not require vertical-transition evidence.</div>
          <div class="floor-grid">
            {floor_topology_sections}
          </div>
        </section>
        <section class="panel">
          <h2>Semantic Room Summary</h2>
          <div class="note">Semantic snippets come from <code>committed_room_world_model_v0_1.json</code>, restricted to rooms that also exist in the public topology export. Listed summary neighbors are room-summary or spatial-neighbor cues and are not guaranteed to equal committed topology edges.</div>
          <div class="room-card-grid">
            {''.join(room_cards)}
          </div>
        </section>
      </div>

      <div class="stack">
        <section class="panel">
          <h2>Route Explanation</h2>
          <div class="note">Thin topology-only pathing. No continuous navigation controller is added here, and audit overlays do not alter route selection.</div>
          <ul class="route-list">
            {''.join(route_steps)}
          </ul>
          <div class="pill-row">
            <span class="pill">hops: {_escape(route.get("hop_count"))}</span>
            <span class="pill">route confidence: {_escape(route.get("route_confidence"))}</span>
            <span class="pill">total cost: {_escape(route.get("total_cost"))}</span>
            <span class="pill">next-hop relation: {_escape((demo_result.get("next_hop") or {}).get("relation_type"))}</span>
          </div>
        </section>
        <section class="panel">
          <h2>Legend and Semantics</h2>
          <div class="note">These labels separate public committed graph edges from stronger gateway-backed passages, weaker geometry-derived adjacency, trajectory-supported same-floor transitions, cross-floor vertical transitions, and non-authoritative audit overlays.</div>
          <table>
            <thead>
              <tr><th>Label</th><th>Meaning</th></tr>
            </thead>
            <tbody>{''.join(semantics_rows)}</tbody>
          </table>
          <div class="note">{_escape(presentation_semantics.get('route_matching_policy'))}</div>
          <ul class="route-list">
            {''.join(matching_limitations)}
          </ul>
        </section>
        <section class="panel">
          <h2>Route Edge Explanation</h2>
          <div class="note">Edge-by-edge route evidence is derived from <code>topology_v0_1.json</code> plus normalized committed snapshot gateway/vertical records when requested. Route-selected labels below are relation-specific when artifact metadata suffices; same-pair alternate relations remain separate.</div>
          <table>
            <thead>
              <tr><th>Step</th><th>Rooms</th><th>Selected Relation</th><th>Semantics</th><th>Evidence Summary</th><th>Evidence IDs</th><th>Conf.</th><th>Support</th><th>Status</th><th>Floor</th><th>Gateway</th><th>Vertical</th><th>Same-Pair Alternates</th><th>Audit Overlay View</th><th>Notes</th></tr>
            </thead>
            <tbody>{''.join(edge_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Object/Semantic Target Evidence</h2>
          <div class="note">Only populated for semantic/object-style requests. Ranking is based on committed room summaries, not raw working-state entities.</div>
          <div class="pill-row">
            <span class="pill">target: {_escape(semantic_target.get("query_target"))}</span>
            <span class="pill">selected goal: {_escape(semantic_target.get("selected_goal_room"))}</span>
          </div>
          <table>
            <thead>
              <tr><th>Room</th><th>Score</th><th>Matched Terms</th></tr>
            </thead>
            <tbody>
              {''.join(candidate_rows)}
            </tbody>
          </table>
          <table>
            <thead><tr><th>Room</th><th>Ranking Score</th><th>Matched Object Labels</th><th>Object Count</th></tr></thead>
            <tbody>{''.join(semantic_evidence_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Gateway Overlay</h2>
          <div class="note">Gateway endpoints are normalized with <code>committed_artifact_ids.py</code>. These markers indicate gateway-backed passage evidence only; they do not redefine every same-room-pair relation as gateway-backed. Approximate markers use room midpoints only when exact marker geometry is unavailable.</div>
          <table>
            <thead><tr><th>Rooms</th><th>Type</th><th>Width m</th><th>Floor</th><th>Geometry</th><th>On Route</th></tr></thead>
            <tbody>{''.join(gateway_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Vertical Transition Overlay</h2>
          <div class="note">Vertical transitions are rendered as a visually distinct overlay and do not change routing. They are cross-floor connectors only and are not required evidence for same-floor weak adjacency.</div>
          <table>
            <thead><tr><th>ID</th><th>Type</th><th>Status</th><th>Conf.</th><th>Rooms/Floors</th><th>Connector</th><th>On Route</th><th>Evidence</th></tr></thead>
            <tbody>{''.join(vertical_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Audit Candidate Overlay</h2>
          {'<div class="audit-disclaimer">' + _escape(audit_disclaimer) + '</div>' if audit_disclaimer else ''}
          <div class="note">Optional audit candidates are visual inspection aids only. They are not used for routing, are not treated as confirmed topology errors, and same-pair visibility alone does not imply the candidate relation was selected by the route.</div>
          <table>
            <thead><tr><th>Type</th><th>Rooms/Edge</th><th>Candidate Label</th><th>Route Visibility</th><th>Severity</th><th>Evidence Source</th><th>Recommended Visual Check</th></tr></thead>
            <tbody>{''.join(audit_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>ID Normalization Warnings</h2>
          <div class="note">Unresolved snapshot or overlay IDs are surfaced here and in the JSON sidecar.</div>
          <table>
            <thead><tr><th>Context</th><th>Field</th><th>Value</th><th>Message</th></tr></thead>
            <tbody>{''.join(warning_rows)}</tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Artifacts Consumed</h2>
          <ul class="artifact-list">
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("topology_json"))}</code></li>
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("committed_room_world_model_json"))}</code></li>
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("committed_room_world_snapshot_json"))}</code></li>
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("audit_dir"))}</code></li>
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("summary_json"))}</code></li>
          </ul>
        </section>
      </div>
    </div>
  </div>
</body>
</html>
"""


def resolve_demo_artifact_paths(
    *,
    scene_root: Optional[Path] = None,
    summary_json: Optional[Path] = None,
    topology_json: Optional[Path] = None,
    committed_room_world_model_json: Optional[Path] = None,
    committed_room_world_snapshot_json: Optional[Path] = None,
) -> DemoArtifactPaths:
    resolved_scene_root = None if scene_root is None else Path(scene_root)
    resolved_summary_json = None if summary_json is None else Path(summary_json)
    resolved_topology_json = None if topology_json is None else Path(topology_json)
    resolved_world_model_json = (
        None if committed_room_world_model_json is None else Path(committed_room_world_model_json)
    )
    resolved_snapshot_json = (
        None if committed_room_world_snapshot_json is None else Path(committed_room_world_snapshot_json)
    )

    if resolved_scene_root is not None:
        logs_dir = resolved_scene_root / "logs"
        if resolved_summary_json is None:
            resolved_summary_json = logs_dir / "summary.json"
        if resolved_topology_json is None:
            resolved_topology_json = logs_dir / "topology_v0_1.json"
        if resolved_world_model_json is None:
            resolved_world_model_json = logs_dir / "committed_room_world_model_v0_1.json"
        if resolved_snapshot_json is None:
            resolved_snapshot_json = logs_dir / "committed_room_world_snapshot_v0_1.json"

    if resolved_summary_json is not None and (resolved_topology_json is None or resolved_world_model_json is None):
        summary_payload = _load_json(resolved_summary_json)
        if resolved_topology_json is None:
            summary_topology = summary_payload.get("topology_v0_1_json")
            if summary_topology:
                resolved_topology_json = Path(summary_topology)
        if resolved_world_model_json is None:
            summary_world_model = summary_payload.get("committed_room_world_model_json")
            if summary_world_model:
                resolved_world_model_json = Path(summary_world_model)
        if resolved_snapshot_json is None:
            summary_snapshot = summary_payload.get("committed_room_world_snapshot_json")
            if summary_snapshot:
                resolved_snapshot_json = Path(summary_snapshot)
        if resolved_scene_root is None:
            resolved_scene_root = resolved_summary_json.parent.parent

    if resolved_topology_json is None or resolved_world_model_json is None:
        raise FileNotFoundError(
            "Could not resolve topology_v0_1.json and committed_room_world_model_v0_1.json."
        )
    if not resolved_topology_json.exists():
        raise FileNotFoundError(f"Missing topology export: {resolved_topology_json}")
    if not resolved_world_model_json.exists():
        raise FileNotFoundError(f"Missing committed room world model export: {resolved_world_model_json}")
    if resolved_summary_json is not None and not resolved_summary_json.exists():
        resolved_summary_json = None
    if resolved_snapshot_json is not None and not resolved_snapshot_json.exists():
        resolved_snapshot_json = None

    return DemoArtifactPaths(
        scene_root=resolved_scene_root,
        summary_json=resolved_summary_json,
        topology_json=resolved_topology_json,
        committed_room_world_model_json=resolved_world_model_json,
        committed_room_world_snapshot_json=resolved_snapshot_json,
    )


def build_room_graph_vln_demo(
    *,
    scene_root: Optional[Path] = None,
    summary_json: Optional[Path] = None,
    topology_json: Optional[Path] = None,
    committed_room_world_model_json: Optional[Path] = None,
    committed_room_world_snapshot_json: Optional[Path] = None,
    start_room: Optional[str] = None,
    goal_room: Optional[str] = None,
    semantic_target: Optional[str] = None,
    route_policy: str = "balanced",
    title: Optional[str] = None,
    include_snapshot_overlays: bool = False,
    include_gateway_overlays: bool = False,
    include_vertical_transition_overlays: bool = False,
    include_route_edge_explanation: bool = False,
    include_semantic_room_summary: bool = False,
    include_audit_overlays: bool = False,
    audit_dir: Optional[Path] = None,
) -> Dict[str, Any]:
    demo = RoomGraphVLNDemo.from_inputs(
        scene_root=scene_root,
        summary_json=summary_json,
        topology_json=topology_json,
        committed_room_world_model_json=committed_room_world_model_json,
        committed_room_world_snapshot_json=committed_room_world_snapshot_json,
    )
    return demo.build_demo(
        start_room=start_room,
        goal_room=goal_room,
        semantic_target=semantic_target,
        route_policy=route_policy,
        title=title,
        include_snapshot_overlays=include_snapshot_overlays,
        include_gateway_overlays=include_gateway_overlays,
        include_vertical_transition_overlays=include_vertical_transition_overlays,
        include_route_edge_explanation=include_route_edge_explanation,
        include_semantic_room_summary=include_semantic_room_summary,
        include_audit_overlays=include_audit_overlays,
        audit_dir=audit_dir,
    )


def write_room_graph_vln_demo(
    demo_result: Dict[str, Any],
    *,
    demo: RoomGraphVLNDemo,
    html_out: Path,
    json_out: Optional[Path] = None,
) -> Dict[str, str]:
    html_path = Path(html_out)
    html_path.parent.mkdir(parents=True, exist_ok=True)
    html_path.write_text(demo.render_html(demo_result), encoding="utf-8")
    outputs = {"html_out": str(html_path)}
    if json_out is not None:
        json_path = Path(json_out)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(demo_result, indent=2), encoding="utf-8")
        outputs["json_out"] = str(json_path)
    return outputs
