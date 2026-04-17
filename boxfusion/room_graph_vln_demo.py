from __future__ import annotations

import html
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import networkx as nx

from boxfusion.query_api import RoomTopologyQueryAPI
from boxfusion.room_topology import RoomTopology, _canonical_room_id
from boxfusion.template_grounding.normalizer import normalize_reference_slot


ROOM_GRAPH_VLN_DEMO_VERSION = "0.1"
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


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


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
    return round(float(value), digits)


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


@dataclass
class DemoArtifactPaths:
    scene_root: Optional[Path]
    summary_json: Optional[Path]
    topology_json: Path
    committed_room_world_model_json: Path


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
        self.topology = RoomTopology.from_json(artifact_paths.topology_json)
        self.query_api = RoomTopologyQueryAPI(self.topology)
        self.room_model_by_id = {
            str(room.get("room_id")): dict(room) for room in self.committed_room_world_model_payload.get("rooms", [])
        }
        self.public_room_ids = _best_public_room_ids(self.topology, self.room_model_by_id)
        self.floor_records = {
            str(item.get("floor_id")): dict(item)
            for item in self.topology_payload.get("floors", [])
            if item.get("floor_id") is not None
        }

    @classmethod
    def from_inputs(
        cls,
        *,
        scene_root: Optional[Path] = None,
        summary_json: Optional[Path] = None,
        topology_json: Optional[Path] = None,
        committed_room_world_model_json: Optional[Path] = None,
    ) -> "RoomGraphVLNDemo":
        artifact_paths = resolve_demo_artifact_paths(
            scene_root=scene_root,
            summary_json=summary_json,
            topology_json=topology_json,
            committed_room_world_model_json=committed_room_world_model_json,
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

    def build_demo(
        self,
        *,
        start_room: Optional[str] = None,
        goal_room: Optional[str] = None,
        semantic_target: Optional[str] = None,
        route_policy: str = "balanced",
        title: Optional[str] = None,
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
            "title": title_text,
            "sequence_id": self.sequence_id,
            "route_policy": route_policy,
            "artifacts": {
                "scene_root": None if self.artifact_paths.scene_root is None else str(self.artifact_paths.scene_root),
                "summary_json": None if self.artifact_paths.summary_json is None else str(self.artifact_paths.summary_json),
                "topology_json": str(self.artifact_paths.topology_json),
                "committed_room_world_model_json": str(self.artifact_paths.committed_room_world_model_json),
            },
            "contract": {
                "public_topology_source": "topology_v0_1.json",
                "semantic_summary_source": "committed_room_world_model_v0_1.json",
                "working_state_used_for_routing": False,
                "continuous_navigation_control": False,
                "bev_used": False,
            },
            "start_resolution": start_resolution,
            "goal_resolution": goal_resolution,
            "route": route,
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
            "per_room_cards": per_room_cards,
            "path_pair_set": [list(item) for item in sorted(path_pair_set)],
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

        elements.append("</svg>")
        return "\n".join(elements)

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
        map_svg = self.render_map_svg(demo_result)
        topology_svg = self.render_topology_svg(demo_result)
        room_cards = []
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
            badge_html = "".join(f'<span class="badge">{_escape(badge)}</span>' for badge in badges)
            semantic_lines = item.get("semantic_lines") or ["no semantic summary"]
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
      border-radius: 16px;
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
          <div class="note">Public rooms are drawn from <code>topology_v0_1.json</code>. Colored polygons show room footprints; lines show public graph edges; the highlighted sequence is the inferred room path.</div>
          {map_svg}
        </section>
        <section class="panel">
          <h2>Abstract Topology Route</h2>
          <div class="note">This abstract graph highlights the same public room path and makes the next-hop decision easier to read in a paper/demo setting.</div>
          {topology_svg}
        </section>
        <section class="panel">
          <h2>Public Room Summary Cards</h2>
          <div class="note">Semantic snippets come from <code>committed_room_world_model_v0_1.json</code>, restricted to rooms that also exist in the public topology export.</div>
          <div class="room-card-grid">
            {''.join(room_cards)}
          </div>
        </section>
      </div>

      <div class="stack">
        <section class="panel">
          <h2>Route Explanation</h2>
          <div class="note">Thin topology-only pathing. No continuous navigation controller is added here.</div>
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
          <h2>Semantic Goal Ranking</h2>
          <div class="note">Only populated for semantic/object-style requests. Ranking is based on committed room summaries, not raw working-state entities.</div>
          <table>
            <thead>
              <tr><th>Room</th><th>Score</th><th>Matched Terms</th></tr>
            </thead>
            <tbody>
              {''.join(candidate_rows)}
            </tbody>
          </table>
        </section>
        <section class="panel">
          <h2>Artifacts Consumed</h2>
          <ul class="artifact-list">
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("topology_json"))}</code></li>
            <li><code>{_escape((demo_result.get("artifacts") or {}).get("committed_room_world_model_json"))}</code></li>
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
) -> DemoArtifactPaths:
    resolved_scene_root = None if scene_root is None else Path(scene_root)
    resolved_summary_json = None if summary_json is None else Path(summary_json)
    resolved_topology_json = None if topology_json is None else Path(topology_json)
    resolved_world_model_json = (
        None if committed_room_world_model_json is None else Path(committed_room_world_model_json)
    )

    if resolved_scene_root is not None:
        logs_dir = resolved_scene_root / "logs"
        if resolved_summary_json is None:
            resolved_summary_json = logs_dir / "summary.json"
        if resolved_topology_json is None:
            resolved_topology_json = logs_dir / "topology_v0_1.json"
        if resolved_world_model_json is None:
            resolved_world_model_json = logs_dir / "committed_room_world_model_v0_1.json"

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

    return DemoArtifactPaths(
        scene_root=resolved_scene_root,
        summary_json=resolved_summary_json,
        topology_json=resolved_topology_json,
        committed_room_world_model_json=resolved_world_model_json,
    )


def build_room_graph_vln_demo(
    *,
    scene_root: Optional[Path] = None,
    summary_json: Optional[Path] = None,
    topology_json: Optional[Path] = None,
    committed_room_world_model_json: Optional[Path] = None,
    start_room: Optional[str] = None,
    goal_room: Optional[str] = None,
    semantic_target: Optional[str] = None,
    route_policy: str = "balanced",
    title: Optional[str] = None,
) -> Dict[str, Any]:
    demo = RoomGraphVLNDemo.from_inputs(
        scene_root=scene_root,
        summary_json=summary_json,
        topology_json=topology_json,
        committed_room_world_model_json=committed_room_world_model_json,
    )
    return demo.build_demo(
        start_room=start_room,
        goal_room=goal_room,
        semantic_target=semantic_target,
        route_policy=route_policy,
        title=title,
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
