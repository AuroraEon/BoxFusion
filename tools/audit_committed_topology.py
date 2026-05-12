#!/usr/bin/env python3
"""Audit committed/public Stage-A topology artifacts.

This is an offline sanity-check tool only.  It reads committed/public artifacts
by default and never mutates topology construction, export semantics, or runtime
state.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
import sys
import tempfile
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from boxfusion.committed_artifact_ids import (
    CommittedArtifactIdNormalizer,
    canonical_room_id,
)


KNOWN_RELATION_TYPES = {"adjacent", "transition", "possible_connection", "vertical_transition"}
DEBUG_ONLY_ARTIFACTS = [
    "working_topology_v0_1.json",
    "working_vs_committed_topology_report_v0_1.json",
    "working_vs_committed_topology_timeline_v0_1.json",
    "online_topology_lifecycle_v0_1.json",
    "room_scoped_runtime_state_v0_1.json",
    "final_vector_map_snapshot.json",
]


def _load_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _json_default(value: Any) -> Any:
    if isinstance(value, set):
        return sorted(value)
    return str(value)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, default=_json_default)


def _write_csv(path: Path, rows: List[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in fieldnames})


def _csv_value(value: Any) -> Any:
    if isinstance(value, (dict, list, tuple, set)):
        return json.dumps(value, sort_keys=True, default=_json_default)
    if value is None:
        return ""
    return value


def _pair_key(a: Any, b: Any) -> Tuple[str, str]:
    return tuple(sorted((str(a), str(b))))  # type: ignore[return-value]


def _edge_pair(edge: Dict[str, Any]) -> Tuple[str, str]:
    return _pair_key(edge.get("source"), edge.get("target"))


def _as_float(value: Any) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
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


def _severity(flags: Iterable[str]) -> str:
    error_flags = {"missing_endpoint", "self_loop"}
    warning_flags = {
        "duplicate_edge",
        "cross_floor_non_vertical_edge",
        "vertical_edge_missing_transition_match",
        "same_floor_edge_without_gateway_or_evidence",
        "low_support_edge",
        "missing_floor_assignment",
        "unresolved_snapshot_id",
        "unknown_relation_type",
    }
    flag_set = set(flags)
    if flag_set & error_flags:
        return "error"
    if flag_set & warning_flags:
        return "warning"
    if flag_set:
        return "info"
    return "ok"


def _missing_candidate_presentation(candidate_type: str) -> Tuple[str, str]:
    mapping = {
        "missing_edge_candidate_neighbor_mismatch": (
            "world_model_neighbor_without_topology_edge",
            "Room-summary or spatial-neighbor relation only; not automatically a missing committed topology edge.",
        ),
        "missing_edge_candidate_gateway_without_topology_edge": (
            "navigable_gateway_missing_topology_edge",
            "Gateway-backed navigability evidence deserves inspection, but the row remains non-authoritative.",
        ),
        "missing_edge_candidate_vertical_transition_without_topology_edge": (
            "vertical_transition_missing_topology_edge",
            "Cross-floor vertical-transition evidence deserves inspection, but same-floor weak adjacency does not require vertical-transition evidence.",
        ),
        "geometry_close_no_edge": (
            "geometry_close_no_edge",
            "Weak same-floor geometry candidate only; do not drive topology repair from this row by itself.",
        ),
    }
    return mapping.get(
        candidate_type,
        (candidate_type or "missing_connectivity_candidate", "Diagnostic candidate only; manual review is still required."),
    )


def _spurious_candidate_presentation(candidate_type: str) -> Tuple[str, str]:
    mapping = {
        "low_support_edge": (
            "weak_low_support_committed_topology_edge",
            "Low-support committed edge for inspection only; route visibility must be interpreted relation-specifically elsewhere.",
        ),
        "cross_floor_non_vertical_edge": (
            "cross_floor_non_vertical_edge",
            "Cross-floor committed edge should be inspected for vertical-transition semantics.",
        ),
        "vertical_transition_edge_without_snapshot_transition": (
            "vertical_transition_missing_snapshot_support",
            "Vertical-transition edge lacks expected committed snapshot support.",
        ),
        "same_floor_edge_without_gateway_or_evidence": (
            "same_floor_edge_without_gateway_or_evidence",
            "Same-floor committed edge lacks gateway or explicit evidence support.",
        ),
        "edge_endpoint_missing_from_topology_rooms": (
            "edge_endpoint_missing_from_topology_rooms",
            "Committed topology edge references a room endpoint not present in topology rooms.",
        ),
        "unknown_relation_type": (
            "unknown_relation_type",
            "Committed topology edge uses a relation outside the known public vocabulary.",
        ),
    }
    return mapping.get(
        candidate_type,
        (candidate_type or "spurious_connectivity_candidate", "Diagnostic candidate only; manual review is still required."),
    )


def _has_vertical_metadata(edge: Dict[str, Any]) -> bool:
    metadata = edge.get("metadata") or {}
    if not isinstance(metadata, dict):
        return False
    return bool(
        metadata.get("transition_ids")
        or metadata.get("transition_records")
        or metadata.get("vertical_transition_id")
        or metadata.get("display_floor_pairs")
    )


def _center_from_room(room: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    candidates = [room.get("center"), room.get("centroid_xy")]
    hook = room.get("bev_vln_hook") or {}
    if isinstance(hook, dict):
        candidates.append(hook.get("centroid_xy"))
    for value in candidates:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            try:
                return float(value[0]), float(value[1])
            except (TypeError, ValueError):
                continue
    polygon = room.get("polygon") or room.get("footprint_polygon_xy")
    if isinstance(polygon, list) and polygon:
        pts = []
        for item in polygon:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                try:
                    pts.append((float(item[0]), float(item[1])))
                except (TypeError, ValueError):
                    pass
        if pts:
            return sum(pt[0] for pt in pts) / len(pts), sum(pt[1] for pt in pts) / len(pts)
    return None


def _bbox_from_room(room: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
    bbox = room.get("extent_bbox_xy")
    if isinstance(bbox, dict):
        try:
            return (
                float(bbox["min_x"]),
                float(bbox["min_y"]),
                float(bbox["max_x"]),
                float(bbox["max_y"]),
            )
        except (KeyError, TypeError, ValueError):
            pass
    polygon = room.get("polygon") or room.get("footprint_polygon_xy")
    if isinstance(polygon, list) and polygon:
        xs, ys = [], []
        for item in polygon:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                try:
                    xs.append(float(item[0]))
                    ys.append(float(item[1]))
                except (TypeError, ValueError):
                    pass
        if xs and ys:
            return min(xs), min(ys), max(xs), max(ys)
    return None


def _bbox_gap(a: Optional[Tuple[float, float, float, float]], b: Optional[Tuple[float, float, float, float]]) -> Optional[float]:
    if a is None or b is None:
        return None
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    dx = max(0.0, ax1 - bx2, bx1 - ax2)
    dy = max(0.0, ay1 - by2, by1 - ay2)
    return math.hypot(dx, dy)


def _distance(a: Optional[Tuple[float, float]], b: Optional[Tuple[float, float]]) -> Optional[float]:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _connected_components(room_ids: Set[str], pairs: Iterable[Tuple[str, str]]) -> List[List[str]]:
    adjacency: Dict[str, Set[str]] = {room_id: set() for room_id in room_ids}
    for a, b in pairs:
        if a in room_ids and b in room_ids and a != b:
            adjacency[a].add(b)
            adjacency[b].add(a)
    seen: Set[str] = set()
    components: List[List[str]] = []
    for room_id in sorted(room_ids):
        if room_id in seen:
            continue
        queue = deque([room_id])
        seen.add(room_id)
        component = []
        while queue:
            current = queue.popleft()
            component.append(current)
            for nxt in sorted(adjacency[current]):
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        components.append(sorted(component))
    return components


def _artifact_paths(scene_root: Path) -> Dict[str, Path]:
    logs = scene_root / "logs"
    return {
        "topology": logs / "topology_v0_1.json",
        "world_model": logs / "committed_room_world_model_v0_1.json",
        "snapshot": logs / "committed_room_world_snapshot_v0_1.json",
        "query_report": logs / "topology_query_report.json",
    }


def _scene_id(scene_root: Path, topology: Optional[Dict[str, Any]], world_model: Optional[Dict[str, Any]], snapshot: Optional[Dict[str, Any]]) -> str:
    for payload in (topology, world_model, snapshot):
        if isinstance(payload, dict) and payload.get("sequence_id"):
            return str(payload["sequence_id"])
    return scene_root.name


def _availability(paths: Dict[str, Path]) -> List[Dict[str, Any]]:
    return [
        {
            "artifact": name,
            "path": str(path),
            "available": path.exists(),
            "authoritative_status": "committed_public_sample_report" if name == "query_report" else "committed_public",
        }
        for name, path in paths.items()
    ]


def _build_topology_indexes(topology: Dict[str, Any]) -> Dict[str, Any]:
    rooms = {}
    for room in topology.get("rooms", []) or []:
        if isinstance(room, dict):
            room_id = canonical_room_id(room.get("id"))
            if room_id:
                rooms[room_id] = dict(room, id=room_id)
    room_ids = set(rooms)
    floor_by_room = {room_id: room.get("floor_id") for room_id, room in rooms.items()}
    valid_pairs = []
    edge_pair_set: Set[Tuple[str, str]] = set()
    vertical_pair_set: Set[Tuple[str, str]] = set()
    relation_by_pair: Dict[Tuple[str, str], Set[str]] = defaultdict(set)
    edge_key_counts: Counter = Counter()
    for edge in topology.get("edges", []) or []:
        if not isinstance(edge, dict):
            continue
        source = canonical_room_id(edge.get("source"))
        target = canonical_room_id(edge.get("target"))
        relation = str(edge.get("relation_type"))
        if source and target:
            edge_key_counts[(tuple(sorted((source, target))), relation)] += 1
            if source in room_ids and target in room_ids and source != target:
                valid_pairs.append((source, target))
                edge_pair_set.add(_pair_key(source, target))
                relation_by_pair[_pair_key(source, target)].add(relation)
                if relation == "vertical_transition":
                    vertical_pair_set.add(_pair_key(source, target))
    return {
        "rooms": rooms,
        "room_ids": room_ids,
        "floor_by_room": floor_by_room,
        "valid_pairs": valid_pairs,
        "edge_pair_set": edge_pair_set,
        "vertical_pair_set": vertical_pair_set,
        "relation_by_pair": relation_by_pair,
        "edge_key_counts": edge_key_counts,
    }


def _matching_pairs(rows: List[Dict[str, Any]]) -> Dict[Tuple[str, str], List[Dict[str, Any]]]:
    out: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        a, b = row.get("room_a"), row.get("room_b")
        if a and b:
            out[_pair_key(a, b)].append(row)
    return out


def _audit_edges(
    scene_id: str,
    topology: Dict[str, Any],
    indexes: Dict[str, Any],
    normalizer: CommittedArtifactIdNormalizer,
    low_confidence_threshold: float,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    room_ids = indexes["room_ids"]
    floor_by_room = indexes["floor_by_room"]
    duplicate_edge_keys = {key for key, count in indexes["edge_key_counts"].items() if count > 1}
    gateway_by_pair = _matching_pairs(normalizer.normalized_gateways)
    vertical_by_pair = _matching_pairs(normalizer.normalized_vertical_transitions)
    unresolved_snapshot = normalizer.summary()["unresolved_gateway_endpoints"] + normalizer.summary()["unresolved_vertical_transition_endpoints"]
    evidence_lookup = {
        str(item.get("evidence_id")): item
        for item in topology.get("evidences", []) or []
        if isinstance(item, dict) and item.get("evidence_id")
    }
    edge_rows = []
    spurious_rows = []
    for idx, raw_edge in enumerate(topology.get("edges", []) or []):
        if not isinstance(raw_edge, dict):
            continue
        source = canonical_room_id(raw_edge.get("source"))
        target = canonical_room_id(raw_edge.get("target"))
        relation = str(raw_edge.get("relation_type"))
        source_floor = floor_by_room.get(source)
        target_floor = floor_by_room.get(target)
        pair = _pair_key(source, target) if source and target else ("", "")
        gateway_matches = gateway_by_pair.get(pair, [])
        vertical_matches = vertical_by_pair.get(pair, [])
        confidence = _as_float(raw_edge.get("confidence"))
        support_count = _as_int(raw_edge.get("support_count"))
        evidence_ids = list(raw_edge.get("evidence_ids") or [])
        evidence_id_count = len(evidence_ids)
        evidence_records = [evidence_lookup[evidence_id] for evidence_id in evidence_ids if evidence_id in evidence_lookup]
        topology_gateway_evidence_count = sum(
            1
            for record in evidence_records
            if "gateway" in str(record.get("source_ref", "")).lower()
            or "gateway" in str(record.get("evidence_type", "")).lower()
        )
        topology_vertical_evidence_count = sum(
            1
            for record in evidence_records
            if "vertical" in str(record.get("source_ref", "")).lower()
            or "vertical" in str(record.get("evidence_type", "")).lower()
        )
        has_gateway_match = bool(gateway_matches) or topology_gateway_evidence_count > 0
        matched_gateway_count = len(gateway_matches) + topology_gateway_evidence_count
        has_vertical_transition_match = bool(vertical_matches) or topology_vertical_evidence_count > 0 or _has_vertical_metadata(raw_edge)
        matched_vertical_transition_count = len(vertical_matches) + topology_vertical_evidence_count
        is_cross_floor = bool(source_floor and target_floor and source_floor != target_floor)
        flags: List[str] = []
        notes: List[str] = []
        if not source or not target or source not in room_ids or target not in room_ids:
            flags.append("missing_endpoint")
            notes.append("edge endpoint is absent from topology rooms")
        if source and target and source == target:
            flags.append("self_loop")
            notes.append("edge connects a room to itself")
        if source and target and (tuple(sorted((source, target))), relation) in duplicate_edge_keys:
            flags.append("duplicate_edge")
            notes.append("same unordered room pair and relation appears more than once")
        if source and target and (source_floor is None or target_floor is None):
            flags.append("missing_floor_assignment")
            notes.append("source or target floor is missing")
        if relation not in KNOWN_RELATION_TYPES:
            flags.append("unknown_relation_type")
            notes.append("relation_type is not in the committed topology relation vocabulary")
        if is_cross_floor and relation != "vertical_transition" and not _has_vertical_metadata(raw_edge):
            flags.append("cross_floor_non_vertical_edge")
            notes.append("cross-floor edge is not explicitly marked as a vertical transition")
        if relation == "vertical_transition" and not has_vertical_transition_match:
            flags.append("vertical_edge_missing_transition_match")
            notes.append("no normalized snapshot or topology vertical-transition evidence matches this pair")
        if (
            source_floor
            and target_floor
            and source_floor == target_floor
            and relation != "vertical_transition"
            and evidence_id_count == 0
            and (support_count is None or support_count <= 0)
            and not has_gateway_match
        ):
            flags.append("same_floor_edge_without_gateway_or_evidence")
            notes.append("same-floor edge has neither evidence_ids nor normalized gateway evidence")
        if (confidence is not None and confidence < low_confidence_threshold) or support_count is None or support_count <= 0:
            flags.append("low_support_edge")
            notes.append("edge has very low confidence or zero/missing support_count")
        if unresolved_snapshot:
            flags.append("unresolved_snapshot_id")
        severity = _severity(flags)
        row = {
            "scene_id": scene_id,
            "edge_id_or_index": raw_edge.get("id", idx),
            "source_room": source,
            "target_room": target,
            "source_floor": source_floor,
            "target_floor": target_floor,
            "relation_type": relation,
            "status": raw_edge.get("status"),
            "confidence": confidence,
            "support_count": support_count,
            "evidence_id_count": evidence_id_count,
            "has_gateway_match": has_gateway_match,
            "matched_gateway_count": matched_gateway_count,
            "has_vertical_transition_match": has_vertical_transition_match,
            "matched_vertical_transition_count": matched_vertical_transition_count,
            "is_cross_floor": is_cross_floor,
            "issue_flags": sorted(set(flags)),
            "severity": severity,
            "notes": "; ".join(dict.fromkeys(notes)) if notes else "",
        }
        edge_rows.append(row)
        for flag in sorted(set(flags)):
            candidate_type = {
                "cross_floor_non_vertical_edge": "cross_floor_non_vertical_edge",
                "vertical_edge_missing_transition_match": "vertical_transition_edge_without_snapshot_transition",
                "same_floor_edge_without_gateway_or_evidence": "same_floor_edge_without_gateway_or_evidence",
                "missing_endpoint": "edge_endpoint_missing_from_topology_rooms",
                "unknown_relation_type": "unknown_relation_type",
                "low_support_edge": "low_support_edge",
            }.get(flag)
            if candidate_type:
                presentation_candidate_type, paper_safety_note = _spurious_candidate_presentation(candidate_type)
                spurious_rows.append(
                    {
                        "scene_id": scene_id,
                        "edge_index": idx,
                        "source_room": source,
                        "target_room": target,
                        "relation_type": relation,
                        "candidate_type": candidate_type,
                        "presentation_candidate_type": presentation_candidate_type,
                        "confidence": confidence,
                        "support_count": support_count,
                        "evidence_id_count": evidence_id_count,
                        "has_gateway_match": has_gateway_match,
                        "has_vertical_transition_match": has_vertical_transition_match,
                        "severity": "error" if flag == "missing_endpoint" else "warning",
                        "notes": row["notes"],
                        "paper_safety_note": paper_safety_note,
                    }
                )
    return edge_rows, spurious_rows


def _graph_summary(scene_id: str, topology: Dict[str, Any], indexes: Dict[str, Any], edge_rows: List[Dict[str, Any]]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    rooms = indexes["rooms"]
    room_ids = indexes["room_ids"]
    floor_by_room = indexes["floor_by_room"]
    components = _connected_components(room_ids, indexes["valid_pairs"])
    degree = Counter()
    for a, b in indexes["valid_pairs"]:
        degree[a] += 1
        degree[b] += 1
    isolated = sorted([room_id for room_id in room_ids if degree[room_id] == 0])
    duplicate_count = sum(1 for count in indexes["edge_key_counts"].values() if count > 1)
    self_loop_count = sum(1 for row in edge_rows if "self_loop" in row.get("issue_flags", []))
    missing_endpoint_count = sum(1 for row in edge_rows if "missing_endpoint" in row.get("issue_flags", []))
    rooms_missing_floor = sorted([room_id for room_id, floor in floor_by_room.items() if floor is None])
    degrees = [degree[room_id] for room_id in room_ids]
    if len(degrees) >= 2:
        mean_degree = statistics.mean(degrees)
        stdev_degree = statistics.pstdev(degrees)
        threshold = max(8.0, mean_degree + 3.0 * stdev_degree)
    else:
        threshold = 8.0
    high_degree_rooms = sorted(
        [
            {"room_id": room_id, "degree": degree[room_id]}
            for room_id in room_ids
            if degree[room_id] > threshold
        ],
        key=lambda item: (-item["degree"], item["room_id"]),
    )
    relation_counts = Counter(str(edge.get("relation_type")) for edge in topology.get("edges", []) or [] if isinstance(edge, dict))
    status_counts = Counter(str(edge.get("status")) for edge in topology.get("edges", []) or [] if isinstance(edge, dict))
    support_values = [_as_int(edge.get("support_count")) for edge in topology.get("edges", []) or [] if isinstance(edge, dict)]
    support_values = [value for value in support_values if value is not None]
    confidence_values = [_as_float(edge.get("confidence")) for edge in topology.get("edges", []) or [] if isinstance(edge, dict)]
    confidence_values = [value for value in confidence_values if value is not None]
    floor_groups: Dict[str, Set[str]] = defaultdict(set)
    for room_id, floor_id in floor_by_room.items():
        floor_groups[str(floor_id)].add(room_id)
    per_floor_components = {
        floor_id: _connected_components(floor_room_ids, [(a, b) for a, b in indexes["valid_pairs"] if a in floor_room_ids and b in floor_room_ids])
        for floor_id, floor_room_ids in floor_groups.items()
    }
    summary = {
        "scene_id": scene_id,
        "room_count": len(room_ids),
        "floor_count": len(topology.get("floors", []) or set(filter(None, floor_by_room.values()))),
        "edge_count": len(topology.get("edges", []) or []),
        "connected_components": components,
        "connected_component_count": len(components),
        "connected_components_per_floor": {key: len(value) for key, value in per_floor_components.items()},
        "isolated_rooms": isolated,
        "duplicate_edges": duplicate_count,
        "self_loop_edges": self_loop_count,
        "edges_with_missing_source_or_target": missing_endpoint_count,
        "abnormal_high_degree_rooms": high_degree_rooms,
        "rooms_missing_floor_id": rooms_missing_floor,
        "edges_crossing_floors_without_vertical_transition": sum(
            1 for row in edge_rows if "cross_floor_non_vertical_edge" in row.get("issue_flags", [])
        ),
        "vertical_transition_edges_with_missing_transition_metadata": sum(
            1 for row in edge_rows if row.get("relation_type") == "vertical_transition" and not row.get("has_vertical_transition_match")
        ),
        "relation_type_distribution": dict(sorted(relation_counts.items())),
        "edge_status_distribution": dict(sorted(status_counts.items())),
        "edge_confidence_summary": _numeric_summary(confidence_values),
        "edge_support_count_summary": _numeric_summary(support_values),
    }
    rows = [
        {"scene_id": scene_id, "metric": key, "value": value}
        for key, value in summary.items()
        if key not in {"connected_components", "abnormal_high_degree_rooms"}
    ]
    return summary, rows


def _numeric_summary(values: List[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"count": 0, "min": None, "max": None, "mean": None}
    return {
        "count": len(values),
        "min": round(min(values), 4),
        "max": round(max(values), 4),
        "mean": round(statistics.mean(values), 4),
    }


def _missing_candidates(
    scene_id: str,
    topology_indexes: Dict[str, Any],
    world_model: Optional[Dict[str, Any]],
    normalizer: CommittedArtifactIdNormalizer,
    centroid_close_threshold_m: float,
    bbox_gap_threshold_m: float,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    edge_pairs = topology_indexes["edge_pair_set"]
    vertical_pairs = topology_indexes["vertical_pair_set"]
    gateway_pairs = set(_matching_pairs(normalizer.normalized_gateways))
    vertical_snapshot_pairs = set(_matching_pairs(normalizer.normalized_vertical_transitions))
    floor_by_room = dict(topology_indexes["floor_by_room"])
    world_rooms = {}
    world_neighbor_pairs: Set[Tuple[str, str]] = set()
    if world_model:
        for room in world_model.get("rooms", []) or []:
            if not isinstance(room, dict):
                continue
            room_id = canonical_room_id(room.get("room_id") or room.get("stable_room_id"))
            if not room_id:
                continue
            world_rooms[room_id] = room
            floor_by_room.setdefault(room_id, room.get("floor_id"))
            for neighbor in room.get("neighbor_room_ids") or []:
                neighbor_id = canonical_room_id(neighbor)
                if neighbor_id and neighbor_id != room_id:
                    world_neighbor_pairs.add(_pair_key(room_id, neighbor_id))
            for conn in room.get("connectivity") or []:
                if isinstance(conn, dict):
                    neighbor_id = canonical_room_id(conn.get("room_id"))
                    if neighbor_id and neighbor_id != room_id:
                        world_neighbor_pairs.add(_pair_key(room_id, neighbor_id))
    for pair in sorted(world_neighbor_pairs):
        if pair not in edge_pairs:
            rows.append(_missing_row(scene_id, pair, floor_by_room, "missing_edge_candidate_neighbor_mismatch", "committed_room_world_model_v0_1.json", None, pair in gateway_pairs, pair in vertical_snapshot_pairs, True, False, "warning", "world model neighbor/connectivity relation has no committed topology edge"))
    for pair in sorted(gateway_pairs):
        if pair not in edge_pairs:
            rows.append(_missing_row(scene_id, pair, floor_by_room, "missing_edge_candidate_gateway_without_topology_edge", "committed_room_world_snapshot_v0_1.json gateways", None, True, pair in vertical_snapshot_pairs, pair in world_neighbor_pairs, False, "warning", "normalized snapshot gateway connects this pair but topology has no edge"))
    for pair in sorted(vertical_snapshot_pairs):
        if pair not in vertical_pairs:
            rows.append(_missing_row(scene_id, pair, floor_by_room, "missing_edge_candidate_vertical_transition_without_topology_edge", "committed_room_world_snapshot_v0_1.json vertical_transitions", None, pair in gateway_pairs, True, pair in world_neighbor_pairs, pair in edge_pairs, "warning", "normalized vertical transition exists without a committed vertical_transition edge"))
    world_room_ids = sorted(world_rooms)
    for idx, room_a in enumerate(world_room_ids):
        for room_b in world_room_ids[idx + 1 :]:
            pair = _pair_key(room_a, room_b)
            if pair in edge_pairs:
                continue
            floor_a, floor_b = floor_by_room.get(room_a), floor_by_room.get(room_b)
            if not floor_a or floor_a != floor_b:
                continue
            center_gap = _distance(_center_from_room(world_rooms[room_a]), _center_from_room(world_rooms[room_b]))
            bbox_gap = _bbox_gap(_bbox_from_room(world_rooms[room_a]), _bbox_from_room(world_rooms[room_b]))
            qualifies = (center_gap is not None and center_gap <= centroid_close_threshold_m) or (
                bbox_gap is not None and bbox_gap <= bbox_gap_threshold_m
            )
            if qualifies:
                rows.append(
                    _missing_row(
                        scene_id,
                        pair,
                        floor_by_room,
                        "geometry_close_no_edge",
                        "committed_room_world_model_v0_1.json room geometry",
                        round(bbox_gap if bbox_gap is not None else center_gap, 4),
                        pair in gateway_pairs,
                        pair in vertical_snapshot_pairs,
                        pair in world_neighbor_pairs,
                        False,
                        "info",
                        "weak same-floor geometry proximity candidate; not trajectory-backed ground truth",
                    )
                )
    return rows


def _missing_row(
    scene_id: str,
    pair: Tuple[str, str],
    floor_by_room: Dict[str, Any],
    candidate_type: str,
    evidence_source: str,
    distance_or_gap: Optional[float],
    has_gateway: bool,
    has_vertical: bool,
    has_world_neighbor: bool,
    has_topology_edge: bool,
    severity: str,
    notes: str,
) -> Dict[str, Any]:
    a, b = pair
    presentation_candidate_type, paper_safety_note = _missing_candidate_presentation(candidate_type)
    return {
        "scene_id": scene_id,
        "room_a": a,
        "room_b": b,
        "floor_a": floor_by_room.get(a),
        "floor_b": floor_by_room.get(b),
        "candidate_type": candidate_type,
        "presentation_candidate_type": presentation_candidate_type,
        "evidence_source": evidence_source,
        "distance_or_gap": distance_or_gap,
        "has_gateway_record": has_gateway,
        "has_vertical_transition_record": has_vertical,
        "has_world_model_neighbor_relation": has_world_neighbor,
        "has_topology_edge": has_topology_edge,
        "severity": severity,
        "notes": notes,
        "paper_safety_note": paper_safety_note,
    }


def _route_consistency(
    scene_id: str,
    topology_path: Path,
    topology: Dict[str, Any],
    query_report: Optional[Dict[str, Any]],
    indexes: Dict[str, Any],
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    room_ids = sorted(indexes["room_ids"])
    relation_by_pair = indexes["relation_by_pair"]
    floor_by_room = indexes["floor_by_room"]
    first_route = (query_report or {}).get("first_route") if isinstance(query_report, dict) else None
    if isinstance(first_route, dict):
        rows.append(_validate_route_row(scene_id, "query_report_first_route", first_route.get("room_sequence") or [], relation_by_pair, floor_by_room, "topology_query_report.json sample first_route"))
    if len(room_ids) >= 2:
        route_specs = [("deterministic_first_to_last", room_ids[0], room_ids[-1])]
        isolated = [room_id for room_id in room_ids if not any(room_id in pair for pair in indexes["edge_pair_set"])]
        route_specs.extend((f"deterministic_first_to_isolated_{room_id}", room_ids[0], room_id) for room_id in isolated if room_id != room_ids[0])
        vertical_pairs = sorted(indexes["vertical_pair_set"])
        if vertical_pairs:
            route_specs.append(("deterministic_vertical_pair", vertical_pairs[0][0], vertical_pairs[0][1]))
        route_api = None
        route_api_error = None
        try:
            from boxfusion.room_topology import RoomTopology

            route_api = RoomTopology.from_json(topology_path)
        except Exception as exc:  # pragma: no cover - defensive import path
            route_api_error = str(exc)
        for route_id, source, target in route_specs:
            result = (
                route_api.find_room_path(source, target, min_conf=0.0)
                if route_api is not None
                else _find_room_path_raw(source, target, relation_by_pair, room_ids)
            )
            if not result.get("found"):
                rows.append(
                    {
                        "scene_id": scene_id,
                        "route_id": route_id,
                        "source_room": source,
                        "target_room": target,
                        "found": False,
                        "room_sequence": [],
                        "all_route_rooms_exist": source in room_ids and target in room_ids,
                        "all_adjacent_pairs_have_edges": False,
                        "cross_floor_uses_vertical_transition": False,
                        "unreachable": True,
                        "severity": "warning",
                        "notes": (
                            f"route API returned {result.get('failure_reason')}"
                            if route_api is not None
                            else f"raw committed-edge fallback returned {result.get('failure_reason')}; route API unavailable: {route_api_error}"
                        ),
                    }
                )
            else:
                notes = "RoomTopology.find_room_path deterministic probe" if route_api is not None else "raw committed-edge deterministic fallback probe"
                rows.append(_validate_route_row(scene_id, route_id, result.get("room_sequence") or [], relation_by_pair, floor_by_room, notes))
    return rows


def _find_room_path_raw(
    source: str,
    target: str,
    relation_by_pair: Dict[Tuple[str, str], Set[str]],
    room_ids: Sequence[str],
) -> Dict[str, Any]:
    room_set = set(room_ids)
    if source not in room_set:
        return {"found": False, "failure_reason": "start_room_not_in_topology", "room_sequence": []}
    if target not in room_set:
        return {"found": False, "failure_reason": "goal_room_not_in_topology", "room_sequence": []}
    if source == target:
        return {"found": True, "room_sequence": [source]}
    adjacency: Dict[str, Set[str]] = {room_id: set() for room_id in room_set}
    for a, b in relation_by_pair:
        if a in room_set and b in room_set:
            adjacency[a].add(b)
            adjacency[b].add(a)
    queue = deque([(source, [source])])
    seen = {source}
    while queue:
        current, path = queue.popleft()
        for nxt in sorted(adjacency[current]):
            if nxt in seen:
                continue
            next_path = path + [nxt]
            if nxt == target:
                return {"found": True, "room_sequence": next_path}
            seen.add(nxt)
            queue.append((nxt, next_path))
    return {"found": False, "failure_reason": "no_path_found", "room_sequence": []}


def _validate_route_row(
    scene_id: str,
    route_id: str,
    sequence: Sequence[Any],
    relation_by_pair: Dict[Tuple[str, str], Set[str]],
    floor_by_room: Dict[str, Any],
    notes_prefix: str,
) -> Dict[str, Any]:
    rooms = [canonical_room_id(room) for room in sequence]
    all_rooms_exist = all(room in floor_by_room for room in rooms if room is not None)
    pair_checks = []
    cross_floor_ok = True
    for a, b in zip(rooms, rooms[1:]):
        pair = _pair_key(a, b)
        relations = relation_by_pair.get(pair, set())
        pair_checks.append(bool(relations))
        if floor_by_room.get(a) and floor_by_room.get(b) and floor_by_room.get(a) != floor_by_room.get(b):
            cross_floor_ok = "vertical_transition" in relations
    all_pairs_have_edges = all(pair_checks) if pair_checks else True
    severity = "ok" if all_rooms_exist and all_pairs_have_edges and cross_floor_ok else "warning"
    return {
        "scene_id": scene_id,
        "route_id": route_id,
        "source_room": rooms[0] if rooms else None,
        "target_room": rooms[-1] if rooms else None,
        "found": bool(rooms),
        "room_sequence": rooms,
        "all_route_rooms_exist": all_rooms_exist,
        "all_adjacent_pairs_have_edges": all_pairs_have_edges,
        "cross_floor_uses_vertical_transition": cross_floor_ok,
        "unreachable": False,
        "severity": severity,
        "notes": notes_prefix,
    }


def _markdown_summary(summary: Dict[str, Any], artifact_availability: List[Dict[str, Any]], output_paths: Dict[str, str]) -> str:
    graph = summary["graph_structural_summary"]
    counts = summary["candidate_counts"]
    lines = [
        f"# Committed Topology Audit Summary: {summary['scene_id']}",
        "",
        "This audit uses committed/public artifacts by default. Findings are diagnostic audit candidates, not topology repair actions or ground-truth error labels.",
        "World-model neighbor rows reflect room-summary or spatial-neighbor semantics, and weak geometry rows are not authoritative missing-edge claims.",
        "",
        "## Artifact Availability",
        "",
        "| artifact | available | path |",
        "| --- | --- | --- |",
    ]
    for item in artifact_availability:
        lines.append(f"| {item['artifact']} | {item['available']} | `{item['path']}` |")
    lines.extend(
        [
            "",
            "## Structural Summary",
            "",
            f"- Rooms: {graph['room_count']}",
            f"- Floors: {graph['floor_count']}",
            f"- Edges: {graph['edge_count']}",
            f"- Connected components: {graph['connected_component_count']}",
            f"- Isolated rooms: {len(graph['isolated_rooms'])}",
            f"- Duplicate edge groups: {graph['duplicate_edges']}",
            f"- Cross-floor non-vertical edge candidates: {graph['edges_crossing_floors_without_vertical_transition']}",
            "",
            "## Candidate Counts",
            "",
            f"- Edge audit warnings/errors: {counts['edge_warning_or_error_count']}",
            f"- Missing-connectivity candidates: {counts['missing_edge_candidate_count']}",
            f"- Spurious-connectivity candidates: {counts['spurious_edge_candidate_count']}",
            f"- Route consistency warnings/errors: {counts['route_warning_or_error_count']}",
            "",
            "## Outputs",
            "",
        ]
    )
    for name, path in output_paths.items():
        lines.append(f"- `{name}`: `{path}`")
    lines.extend(
        [
            "",
            "## CLI Smoke Command",
            "",
            "```bash",
            "python tools/audit_committed_topology.py --self-test",
            "```",
        ]
    )
    return "\n".join(lines) + "\n"


def _html_table(rows: List[Dict[str, Any]], columns: Sequence[str], limit: int = 200) -> str:
    head = "".join(f"<th>{html.escape(col)}</th>" for col in columns)
    body = []
    for row in rows[:limit]:
        body.append(
            "<tr>"
            + "".join(f"<td>{html.escape(str(_csv_value(row.get(col))))}</td>" for col in columns)
            + "</tr>"
        )
    if len(rows) > limit:
        body.append(f"<tr><td colspan='{len(columns)}'>Showing first {limit} of {len(rows)} rows.</td></tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def _html_graph(edge_rows: List[Dict[str, Any]], missing_rows: List[Dict[str, Any]], floor_by_room: Dict[str, Any]) -> str:
    rooms = sorted(floor_by_room)
    if not rooms:
        return "<p>No room nodes available.</p>"
    floors = sorted({str(floor_by_room.get(room) or "unknown") for room in rooms})
    width = 900
    height = max(260, 160 * len(floors))
    positions = {}
    for floor_idx, floor in enumerate(floors):
        floor_rooms = [room for room in rooms if str(floor_by_room.get(room) or "unknown") == floor]
        y = 80 + floor_idx * 150
        for idx, room in enumerate(floor_rooms):
            x = 80 + idx * max(80, min(150, (width - 160) // max(1, len(floor_rooms) - 1 or 1)))
            positions[room] = (x, y)
    parts = [f"<svg viewBox='0 0 {width} {height}' role='img' aria-label='topology graph'>"]
    for floor_idx, floor in enumerate(floors):
        y = 30 + floor_idx * 150
        parts.append(f"<text x='20' y='{y}' class='floor-label'>{html.escape(floor)}</text>")
        parts.append(f"<line x1='20' y1='{y + 50}' x2='{width - 20}' y2='{y + 50}' class='floor-line' />")
    for edge in edge_rows:
        a, b = edge.get("source_room"), edge.get("target_room")
        if a not in positions or b not in positions:
            continue
        x1, y1 = positions[a]
        x2, y2 = positions[b]
        cls = "edge suspicious" if edge.get("severity") in {"warning", "error"} else "edge"
        parts.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' class='{cls}' />")
    for row in missing_rows[:80]:
        a, b = row.get("room_a"), row.get("room_b")
        if a not in positions or b not in positions:
            continue
        x1, y1 = positions[a]
        x2, y2 = positions[b]
        parts.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' class='missing' />")
    for room, (x, y) in positions.items():
        parts.append(f"<circle cx='{x}' cy='{y}' r='18' class='node' />")
        parts.append(f"<text x='{x}' y='{y + 4}' text-anchor='middle' class='node-label'>{html.escape(room.replace('room_', 'R'))}</text>")
    parts.append("</svg>")
    return "".join(parts)


def _write_html(path: Path, payload: Dict[str, Any]) -> None:
    edge_columns = ["edge_id_or_index", "source_room", "target_room", "relation_type", "severity", "issue_flags", "notes"]
    missing_columns = ["room_a", "room_b", "presentation_candidate_type", "severity", "notes", "paper_safety_note"]
    spurious_columns = ["edge_index", "source_room", "target_room", "presentation_candidate_type", "severity", "notes", "paper_safety_note"]
    route_columns = ["route_id", "source_room", "target_room", "found", "severity", "notes"]
    graph_html = _html_graph(payload["edge_audit_rows"], payload["missing_edge_candidates"], payload["topology_indexes"]["floor_by_room"])
    doc = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8" />
<title>Committed Topology Audit - {html.escape(payload['scene_id'])}</title>
<style>
body {{ font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 24px; color: #1f2933; }}
h1, h2 {{ margin: 24px 0 12px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px; margin-bottom: 20px; }}
th, td {{ border: 1px solid #d8dee9; padding: 6px 8px; vertical-align: top; }}
th {{ background: #f2f5f8; text-align: left; }}
.summary {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; }}
.metric {{ border: 1px solid #d8dee9; border-radius: 6px; padding: 10px; background: #fbfcfe; }}
svg {{ width: 100%; max-height: 560px; border: 1px solid #d8dee9; background: #fff; }}
.edge {{ stroke: #536878; stroke-width: 2; opacity: 0.75; }}
.suspicious {{ stroke: #b45309; stroke-width: 3; }}
.missing {{ stroke: #b91c1c; stroke-width: 2; stroke-dasharray: 6 4; opacity: 0.7; }}
.node {{ fill: #e6f4f1; stroke: #136f63; stroke-width: 2; }}
.node-label {{ font-size: 11px; fill: #16332f; }}
.floor-label {{ font-size: 13px; fill: #52616b; }}
.floor-line {{ stroke: #edf1f5; stroke-width: 2; }}
</style>
</head>
<body>
<h1>Committed Topology Audit: {html.escape(payload['scene_id'])}</h1>
<p>This page audits committed/public artifacts. Candidate rows are diagnostic, not topology repairs or ground-truth labels. World-model neighbor rows reflect room-summary or spatial-neighbor semantics, and weak geometry rows are not authoritative missing-edge claims.</p>
<h2>Scene Summary</h2>
<div class="summary">
<div class="metric"><strong>Rooms</strong><br>{payload['graph_structural_summary']['room_count']}</div>
<div class="metric"><strong>Floors</strong><br>{payload['graph_structural_summary']['floor_count']}</div>
<div class="metric"><strong>Edges</strong><br>{payload['graph_structural_summary']['edge_count']}</div>
<div class="metric"><strong>Components</strong><br>{payload['graph_structural_summary']['connected_component_count']}</div>
</div>
<h2>Artifact Availability</h2>
{_html_table(payload['artifact_availability'], ['artifact', 'available', 'authoritative_status', 'path'])}
<h2>ID Normalization Summary</h2>
{_html_table([payload['id_normalization_summary']], ['total_snapshot_rooms', 'total_numeric_room_ids_mapped', 'total_canonical_room_ids_mapped', 'unresolved_gateway_endpoints', 'unresolved_vertical_transition_endpoints', 'unresolved_object_room_assignments'])}
<h2>Graph Structural Summary</h2>
{_html_table(payload['graph_summary_rows'], ['metric', 'value'])}
<h2>Topology Graph</h2>
{graph_html}
<h2>Edge Issue Table</h2>
{_html_table(payload['edge_audit_rows'], edge_columns)}
<h2>Missing-Connectivity Candidates</h2>
{_html_table(payload['missing_edge_candidates'], missing_columns)}
<h2>Spurious-Connectivity Candidates</h2>
{_html_table(payload['spurious_edge_candidates'], spurious_columns)}
<h2>Vertical Transition Summary</h2>
{_html_table(payload['normalizer'].normalized_vertical_transitions, ['index', 'transition_id', 'room_a', 'room_b', 'from_floor_id', 'to_floor_id'])}
<h2>Gateway Summary</h2>
{_html_table(payload['normalizer'].normalized_gateways, ['index', 'room_a', 'room_b', 'floor_id'])}
<h2>Route Consistency Summary</h2>
{_html_table(payload['route_consistency_rows'], route_columns)}
</body>
</html>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(doc, encoding="utf-8")


def audit_scene(args: argparse.Namespace, scene_root: Path, out_dir: Path) -> Dict[str, Any]:
    paths = _artifact_paths(scene_root)
    topology = _load_json(paths["topology"])
    world_model = _load_json(paths["world_model"])
    snapshot = _load_json(paths["snapshot"]) or {}
    query_report = _load_json(paths["query_report"])
    if topology is None:
        raise FileNotFoundError(f"missing required committed topology artifact: {paths['topology']}")
    scene_id = _scene_id(scene_root, topology, world_model, snapshot)
    artifact_availability = _availability(paths)
    normalizer = CommittedArtifactIdNormalizer.from_snapshot(snapshot)
    indexes = _build_topology_indexes(topology)
    edge_rows, spurious_rows = _audit_edges(scene_id, topology, indexes, normalizer, args.low_confidence_threshold)
    graph_summary, graph_rows = _graph_summary(scene_id, topology, indexes, edge_rows)
    missing_rows = _missing_candidates(
        scene_id,
        indexes,
        world_model,
        normalizer,
        args.centroid_close_threshold_m,
        args.bbox_gap_threshold_m,
    )
    route_rows = _route_consistency(scene_id, paths["topology"], topology, query_report, indexes)
    debug_diagnostics = []
    if args.include_debug_diagnostics:
        for name in DEBUG_ONLY_ARTIFACTS:
            path = scene_root / "logs" / name
            debug_diagnostics.append(
                {
                    "artifact": name,
                    "path": str(path),
                    "available": path.exists(),
                    "diagnostic_status": "non_authoritative_debug_only",
                }
            )
    candidate_counts = {
        "edge_warning_or_error_count": sum(1 for row in edge_rows if row["severity"] in {"warning", "error"}),
        "missing_edge_candidate_count": len(missing_rows),
        "spurious_edge_candidate_count": len(spurious_rows),
        "route_warning_or_error_count": sum(1 for row in route_rows if row["severity"] in {"warning", "error"}),
    }
    summary = {
        "scene_id": scene_id,
        "scene_root": str(scene_root),
        "artifact_availability": artifact_availability,
        "id_normalization_summary": normalizer.summary(),
        "graph_structural_summary": graph_summary,
        "candidate_counts": candidate_counts,
        "debug_diagnostics": debug_diagnostics,
        "notes": [
            "Uses committed/public artifacts by default.",
            "topology_query_report.json is treated only as a sample queryability report.",
            "Missing and spurious connectivity rows are audit candidates, not confirmed topology errors.",
        ],
    }
    output_paths = {
        "topology_audit_summary.md": str(out_dir / "topology_audit_summary.md"),
        "topology_audit_summary.json": str(out_dir / "topology_audit_summary.json"),
        "topology_graph_summary.csv": str(out_dir / "topology_graph_summary.csv"),
        "topology_edge_audit.csv": str(out_dir / "topology_edge_audit.csv"),
        "topology_missing_edge_candidates.csv": str(out_dir / "topology_missing_edge_candidates.csv"),
        "topology_spurious_edge_candidates.csv": str(out_dir / "topology_spurious_edge_candidates.csv"),
        "topology_route_consistency.csv": str(out_dir / "topology_route_consistency.csv"),
        "topology_id_normalization_summary.json": str(out_dir / "topology_id_normalization_summary.json"),
        "topology_audit.html": str(out_dir / "topology_audit.html"),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "topology_audit_summary.md").write_text(_markdown_summary(summary, artifact_availability, output_paths), encoding="utf-8")
    if args.emit_json:
        _write_json(out_dir / "topology_audit_summary.json", summary)
        _write_json(out_dir / "topology_id_normalization_summary.json", normalizer.summary())
        _write_json(out_dir / "topology_edge_audit.json", edge_rows)
        _write_json(out_dir / "topology_missing_edge_candidates.json", missing_rows)
        _write_json(out_dir / "topology_spurious_edge_candidates.json", spurious_rows)
        _write_json(out_dir / "topology_route_consistency.json", route_rows)
    if args.emit_csv:
        _write_csv(out_dir / "topology_graph_summary.csv", graph_rows, ["scene_id", "metric", "value"])
        _write_csv(out_dir / "topology_edge_audit.csv", edge_rows, EDGE_FIELDS)
        _write_csv(out_dir / "topology_missing_edge_candidates.csv", missing_rows, MISSING_FIELDS)
        _write_csv(out_dir / "topology_spurious_edge_candidates.csv", spurious_rows, SPURIOUS_FIELDS)
        _write_csv(out_dir / "topology_route_consistency.csv", route_rows, ROUTE_FIELDS)
    if args.emit_html:
        html_payload = {
            "scene_id": scene_id,
            "artifact_availability": artifact_availability,
            "id_normalization_summary": normalizer.summary(),
            "graph_structural_summary": graph_summary,
            "graph_summary_rows": graph_rows,
            "edge_audit_rows": edge_rows,
            "missing_edge_candidates": missing_rows,
            "spurious_edge_candidates": spurious_rows,
            "route_consistency_rows": route_rows,
            "normalizer": normalizer,
            "topology_indexes": indexes,
        }
        _write_html(out_dir / "topology_audit.html", html_payload)
    summary["output_dir"] = str(out_dir)
    return summary


EDGE_FIELDS = [
    "scene_id",
    "edge_id_or_index",
    "source_room",
    "target_room",
    "source_floor",
    "target_floor",
    "relation_type",
    "status",
    "confidence",
    "support_count",
    "evidence_id_count",
    "has_gateway_match",
    "matched_gateway_count",
    "has_vertical_transition_match",
    "matched_vertical_transition_count",
    "is_cross_floor",
    "issue_flags",
    "severity",
    "notes",
]
MISSING_FIELDS = [
    "scene_id",
    "room_a",
    "room_b",
    "floor_a",
    "floor_b",
    "candidate_type",
    "presentation_candidate_type",
    "evidence_source",
    "distance_or_gap",
    "has_gateway_record",
    "has_vertical_transition_record",
    "has_world_model_neighbor_relation",
    "has_topology_edge",
    "severity",
    "notes",
    "paper_safety_note",
]
SPURIOUS_FIELDS = [
    "scene_id",
    "edge_index",
    "source_room",
    "target_room",
    "relation_type",
    "candidate_type",
    "presentation_candidate_type",
    "confidence",
    "support_count",
    "evidence_id_count",
    "has_gateway_match",
    "has_vertical_transition_match",
    "severity",
    "notes",
    "paper_safety_note",
]
ROUTE_FIELDS = [
    "scene_id",
    "route_id",
    "source_room",
    "target_room",
    "found",
    "room_sequence",
    "all_route_rooms_exist",
    "all_adjacent_pairs_have_edges",
    "cross_floor_uses_vertical_transition",
    "unreachable",
    "severity",
    "notes",
]


def _read_scene_roots_file(path: Path) -> List[Path]:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return []
    if path.suffix.lower() == ".json":
        data = json.loads(text)
        if isinstance(data, dict):
            data = data.get("scene_roots") or data.get("scenes") or []
        return [Path(str(item)) for item in data]
    return [Path(line.strip()) for line in text.splitlines() if line.strip() and not line.strip().startswith("#")]


def _write_batch_outputs(batch_out: Path, summaries: List[Dict[str, Any]]) -> None:
    rows = []
    lines = ["# Batch Committed Topology Audit Summary", ""]
    for summary in summaries:
        counts = summary["candidate_counts"]
        graph = summary["graph_structural_summary"]
        rows.append(
            {
                "scene_id": summary["scene_id"],
                "scene_root": summary["scene_root"],
                "output_dir": summary["output_dir"],
                "room_count": graph["room_count"],
                "floor_count": graph["floor_count"],
                "edge_count": graph["edge_count"],
                "connected_component_count": graph["connected_component_count"],
                "edge_warning_or_error_count": counts["edge_warning_or_error_count"],
                "missing_edge_candidate_count": counts["missing_edge_candidate_count"],
                "spurious_edge_candidate_count": counts["spurious_edge_candidate_count"],
                "route_warning_or_error_count": counts["route_warning_or_error_count"],
            }
        )
        lines.append(
            f"- {summary['scene_id']}: rooms={graph['room_count']}, edges={graph['edge_count']}, "
            f"edge_warnings={counts['edge_warning_or_error_count']}, output=`{summary['output_dir']}`"
        )
    batch_out.mkdir(parents=True, exist_ok=True)
    (batch_out / "batch_topology_audit_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    _write_csv(
        batch_out / "batch_topology_audit_summary.csv",
        rows,
        [
            "scene_id",
            "scene_root",
            "output_dir",
            "room_count",
            "floor_count",
            "edge_count",
            "connected_component_count",
            "edge_warning_or_error_count",
            "missing_edge_candidate_count",
            "spurious_edge_candidate_count",
            "route_warning_or_error_count",
        ],
    )
    links = "".join(
        f"<li><a href='{html.escape(Path(row['output_dir']).name)}/topology_audit.html'>{html.escape(row['scene_id'])}</a></li>"
        for row in rows
    )
    (batch_out / "index.html").write_text(
        f"<!doctype html><html><head><meta charset='utf-8'><title>Batch Topology Audit</title></head><body><h1>Batch Topology Audit</h1><ul>{links}</ul></body></html>",
        encoding="utf-8",
    )


def run_self_test() -> None:
    snapshot = {
        "rooms": [
            {"id": 1, "room_id": "room_1", "floor_id": "floor_1"},
            {"id": 2, "room_id": "room_2", "floor_id": "floor_1"},
        ],
        "gateways": [{"connects": [1, 2]}, {"connects": [1, 99]}],
        "vertical_transitions": [
            {"transition_id": "vt_ok", "from_room_id": 1, "to_room_id": 2},
            {"transition_id": "vt_bad", "from_room_id": 1, "to_room_id": 42},
        ],
        "objects": [{"id": 7, "room_id": "room_1"}],
    }
    normalizer = CommittedArtifactIdNormalizer.from_snapshot(snapshot)
    assert normalizer.normalize_room_id(1, "test", "room") == "room_1"
    assert normalizer.normalize_room_id("room_2", "test", "room") == "room_2"
    summary = normalizer.summary()
    assert summary["unresolved_gateway_endpoints"] == 1
    assert summary["unresolved_vertical_transition_endpoints"] == 1
    assert normalizer.normalized_gateways[0]["room_a"] == "room_1"
    assert normalizer.normalized_gateways[0]["room_b"] == "room_2"
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / "scene"
        logs = root / "logs"
        logs.mkdir(parents=True)
        topology = {
            "version": "0.1",
            "sequence_id": "synthetic_scene",
            "floors": [{"floor_id": "floor_1"}],
            "rooms": [
                {"id": "room_1", "floor_id": "floor_1", "center": [0, 0], "polygon": []},
                {"id": "room_2", "floor_id": "floor_1", "center": [1, 0], "polygon": []},
            ],
            "edges": [
                {
                    "source": "room_1",
                    "target": "room_2",
                    "relation_type": "adjacent",
                    "confidence": 0.9,
                    "status": "confirmed",
                    "support_count": 2,
                    "evidence_ids": ["ev_1"],
                    "metadata": {},
                }
            ],
            "indices": {},
            "entities": {},
            "evidences": [{"evidence_id": "ev_1", "source_ref": "gateways:0"}],
        }
        world_model = {
            "sequence_id": "synthetic_scene",
            "rooms": [
                {"room_id": "room_1", "floor_id": "floor_1", "centroid_xy": [0, 0], "neighbor_room_ids": ["room_2"]},
                {"room_id": "room_2", "floor_id": "floor_1", "centroid_xy": [1, 0], "neighbor_room_ids": ["room_1"]},
            ],
        }
        (logs / "topology_v0_1.json").write_text(json.dumps(topology), encoding="utf-8")
        (logs / "committed_room_world_model_v0_1.json").write_text(json.dumps(world_model), encoding="utf-8")
        (logs / "committed_room_world_snapshot_v0_1.json").write_text(json.dumps(snapshot), encoding="utf-8")
        args = parse_args(
            [
                "--scene-root",
                str(root),
                "--out-dir",
                str(root / "logs" / "topology_audit"),
                "--emit-json",
                "--emit-csv",
                "--emit-html",
            ]
        )
        result = audit_scene(args, root, root / "logs" / "topology_audit")
        assert result["candidate_counts"]["edge_warning_or_error_count"] == 1
        assert (root / "logs" / "topology_audit" / "topology_edge_audit.csv").exists()
        assert (root / "logs" / "topology_audit" / "topology_audit.html").exists()
    print("self-test passed")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit committed/public Stage-A topology artifacts.")
    parser.add_argument("--scene-root", action="append", default=[], help="Scene root containing logs/*.json committed artifacts. May be repeated.")
    parser.add_argument("--scene-roots-file", help="Optional JSON or text file listing scene roots.")
    parser.add_argument("--out-dir", help="Output directory. Single-scene writes directly here; batch writes per-scene subdirectories here.")
    parser.add_argument("--emit-json", action="store_true", help="Emit JSON audit outputs.")
    parser.add_argument("--emit-csv", action="store_true", help="Emit CSV audit outputs.")
    parser.add_argument("--emit-html", action="store_true", help="Emit standalone HTML audit page.")
    parser.add_argument("--include-debug-diagnostics", action="store_true", help="List debug-only artifacts as non-authoritative diagnostic evidence.")
    parser.add_argument("--centroid-close-threshold-m", type=float, default=2.0, help="Weak geometry candidate centroid threshold.")
    parser.add_argument("--bbox-gap-threshold-m", type=float, default=0.5, help="Weak geometry candidate bbox gap threshold.")
    parser.add_argument("--low-confidence-threshold", type=float, default=0.15, help="Threshold for low-confidence spurious-edge candidates.")
    parser.add_argument("--self-test", action="store_true", help="Run lightweight normalization and synthetic artifact smoke tests.")
    args = parser.parse_args(argv)
    if args.self_test:
        return args
    roots = [Path(item) for item in args.scene_root]
    if args.scene_roots_file:
        roots.extend(_read_scene_roots_file(Path(args.scene_roots_file)))
    args.scene_roots = roots
    if not args.scene_roots:
        parser.error("--scene-root or --scene-roots-file is required unless --self-test is used")
    if not args.out_dir:
        parser.error("--out-dir is required unless --self-test is used")
    if not (args.emit_json or args.emit_csv or args.emit_html):
        args.emit_json = True
        args.emit_csv = True
        args.emit_html = True
    return args


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    if args.self_test:
        run_self_test()
        return
    out_dir = Path(args.out_dir)
    summaries = []
    batch = len(args.scene_roots) > 1
    for scene_root in args.scene_roots:
        scene_root = Path(scene_root)
        scene_out = out_dir / scene_root.name if batch else out_dir
        summaries.append(audit_scene(args, scene_root, scene_out))
    if batch:
        _write_batch_outputs(out_dir, summaries)
    print(json.dumps({"scene_count": len(summaries), "output_dir": str(out_dir), "scene_ids": [item["scene_id"] for item in summaries]}, indent=2))


if __name__ == "__main__":
    main()
