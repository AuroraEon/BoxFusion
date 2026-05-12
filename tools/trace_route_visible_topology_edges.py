#!/usr/bin/env python3
"""Trace Step 7 route-visible topology edge candidates.

This is a read-only diagnostic helper. It reads Step 7 target rows, committed
public artifacts, Step 5 route sidecars, and Step 4 audit CSVs. It does not
modify topology construction, runtime behavior, export semantics, or routing.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


SCENE_ROOTS = {
    "00843-DYehNKdT76V": Path("runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V"),
    "00824-Dd4bFSTQ8gi": Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi"),
    "00862-LT9Jq6dN3Ea": Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea"),
    "00829-QaLdnwvtxbs": Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs"),
}

STEP5_ROOT = Path("runtime_stage1_frozen_evidence/step5_enhanced_vln_visualization")
STEP4_AUDIT_ROOT = Path("runtime_stage1_frozen_evidence/step4_topology_audit_batch")

TRACE_FIELDS = [
    "scene_id",
    "candidate_id",
    "edge_index",
    "source_room",
    "target_room",
    "relation_type",
    "confidence",
    "support_count",
    "status",
    "evidence_ids",
    "resolved_evidence_types",
    "resolved_evidence_source_refs",
    "resolved_evidence_notes",
    "has_snapshot_gateway_pair",
    "snapshot_gateway_count",
    "gateway_details",
    "has_snapshot_vertical_transition_pair",
    "snapshot_vertical_transition_count",
    "vertical_transition_details",
    "world_model_neighbor_relation",
    "world_model_connectivity_relation",
    "route_step_index",
    "route_visibility_label",
    "relation_match_precision",
    "selected_route_relation_type",
    "selected_route_edge_index",
    "same_pair_alternate_relation_count",
    "route_edge_notes",
    "diagnosis",
    "recommended_action",
    "notes",
]


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _load_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return [dict(row) for row in csv.DictReader(f)]


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")


def _csv_value(value: Any) -> Any:
    if isinstance(value, (dict, list, tuple, set)):
        return json.dumps(value, sort_keys=True)
    if value is None:
        return ""
    return value


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(fields))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _csv_value(row.get(field)) for field in fields})


def canonical_room_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if text.startswith("room_"):
        return text
    if text.lstrip("-").isdigit():
        number = int(text)
        if number < 0:
            return None
        return f"room_{number}"
    return text


def pair_key(a: Any, b: Any) -> Tuple[str, str]:
    room_a = canonical_room_id(a) or ""
    room_b = canonical_room_id(b) or ""
    return tuple(sorted((room_a, room_b)))  # type: ignore[return-value]


def parse_edge_descriptor(value: str) -> Dict[str, Any]:
    match = re.search(r"(.+?)\s+-\s+(.+?)\s+\((.+?),\s*edge_index=(\d+)\)", value or "")
    if not match:
        return {"source_room": "", "target_room": "", "relation_type": "", "edge_index": ""}
    source, target, relation, edge_index = match.groups()
    return {
        "source_room": canonical_room_id(source),
        "target_room": canonical_room_id(target),
        "relation_type": relation,
        "edge_index": int(edge_index),
    }


def snapshot_gateway_matches(snapshot: Dict[str, Any], source: str, target: str) -> List[Dict[str, Any]]:
    wanted = pair_key(source, target)
    matches = []
    for idx, gateway in enumerate(snapshot.get("gateways", []) or []):
        connects = list(gateway.get("connects") or [])
        if len(connects) < 2:
            continue
        if pair_key(connects[0], connects[1]) == wanted:
            matches.append({"index": idx, **dict(gateway)})
    return matches


def snapshot_vertical_matches(snapshot: Dict[str, Any], source: str, target: str) -> List[Dict[str, Any]]:
    wanted = pair_key(source, target)
    matches = []
    for idx, transition in enumerate(snapshot.get("vertical_transitions", []) or []):
        if pair_key(transition.get("from_room_id"), transition.get("to_room_id")) == wanted:
            matches.append({"index": idx, **dict(transition)})
    return matches


def world_model_pair_fields(world_model: Dict[str, Any], source: str, target: str) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    wanted = {source, target}
    neighbors: List[Dict[str, Any]] = []
    connectivity: List[Dict[str, Any]] = []
    for room in world_model.get("rooms", []) or []:
        room_id = canonical_room_id(room.get("room_id") or room.get("stable_room_id"))
        if room_id not in wanted:
            continue
        for neighbor in room.get("neighbor_room_ids") or []:
            neighbor_id = canonical_room_id(neighbor)
            if neighbor_id in wanted and neighbor_id != room_id:
                neighbors.append({"room_id": room_id, "neighbor_room_id": neighbor_id})
        for conn in room.get("connectivity") or []:
            if not isinstance(conn, dict):
                continue
            neighbor_id = canonical_room_id(conn.get("room_id"))
            if neighbor_id in wanted and neighbor_id != room_id:
                connectivity.append({"room_id": room_id, **dict(conn, room_id=neighbor_id)})
    return neighbors, connectivity


def sidecar_candidates(scene_id: str, sidecar_root: Path) -> Iterable[Path]:
    scene_slug = scene_id.lower()
    return sorted((sidecar_root / "scenes" / scene_slug / "queries").glob("*.json"))


def route_matches(
    scene_id: str,
    source: str,
    target: str,
    relation_type: str,
    edge_index: int,
    sidecar_root: Path,
) -> List[Dict[str, Any]]:
    wanted = pair_key(source, target)
    matches: List[Dict[str, Any]] = []
    for sidecar in sidecar_candidates(scene_id, sidecar_root):
        data = _load_json(sidecar)
        for explanation in data.get("route_edge_explanation", []) or []:
            if pair_key(explanation.get("source_room"), explanation.get("target_room")) != wanted:
                continue
            selected_edge_index = explanation.get("selected_edge_index")
            relation_match_precision = str(explanation.get("selected_relation_match_precision") or "")
            relation_selected = False
            route_visibility_label = "same_pair_alternate_relation_visible"
            if selected_edge_index is not None and str(selected_edge_index) == str(edge_index):
                relation_selected = True
                route_visibility_label = "route_selected_weak_relation"
            elif relation_match_precision in {"same_pair_relation_visible", "pair_visible_only"}:
                route_visibility_label = "same_pair_relation_visible"
                relation_selected = False
            elif str(explanation.get("relation_type") or "") == str(relation_type) and selected_edge_index is None:
                route_visibility_label = "same_pair_relation_visible"
            matches.append(
                {
                    "json_sidecar_path": str(sidecar),
                    "html_path": str(sidecar.with_suffix(".html")),
                    "route_step_index": explanation.get("step_index"),
                    "selected_relation_type": explanation.get("relation_type"),
                    "selected_edge_index": selected_edge_index,
                    "relation_match_precision": relation_match_precision or "legacy_relation_type_only",
                    "candidate_relation_selected": relation_selected,
                    "route_visibility_label": route_visibility_label,
                    "explanation": explanation,
                }
            )
    return matches


def audit_rows(scene_id: str, source: str, target: str, relation_type: str, edge_index: int) -> List[Dict[str, Any]]:
    path = STEP4_AUDIT_ROOT / scene_id / "topology_spurious_edge_candidates.csv"
    rows = []
    for row in _load_csv(path):
        if str(row.get("edge_index")) == str(edge_index):
            rows.append(row)
            continue
        if (
            pair_key(row.get("source_room"), row.get("target_room")) == pair_key(source, target)
            and str(row.get("relation_type") or "") == str(relation_type)
        ):
            rows.append(row)
    return rows


def compact_gateway(gateway: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "index": gateway.get("index"),
        "type": gateway.get("type"),
        "width_m": gateway.get("width_m"),
        "pos_world": gateway.get("pos_world"),
        "grid_pos": gateway.get("grid_pos"),
        "floor_id": gateway.get("floor_id"),
        "relation_scope": gateway.get("relation_scope"),
        "connects": gateway.get("connects"),
    }


def compact_transition(transition: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "index": transition.get("index"),
        "transition_id": transition.get("transition_id"),
        "type": transition.get("type"),
        "confidence": transition.get("confidence"),
        "status": transition.get("status"),
        "from_room_id": canonical_room_id(transition.get("from_room_id")),
        "to_room_id": canonical_room_id(transition.get("to_room_id")),
        "from_floor_id": transition.get("from_floor_id"),
        "to_floor_id": transition.get("to_floor_id"),
        "connector_label": transition.get("connector_label"),
    }


def trace_target(row: Dict[str, str], sidecar_root: Path) -> Dict[str, Any]:
    parsed = parse_edge_descriptor(row.get("rooms_or_edge", ""))
    scene_id = row["scene_id"]
    source = parsed["source_room"]
    target = parsed["target_room"]
    relation_type = parsed["relation_type"]
    edge_index = int(parsed["edge_index"])
    scene_root = SCENE_ROOTS[scene_id]
    logs = scene_root / "logs"
    topology = _load_json(logs / "topology_v0_1.json")
    snapshot = _load_json(logs / "committed_room_world_snapshot_v0_1.json")
    world_model = _load_json(logs / "committed_room_world_model_v0_1.json")

    edge = dict((topology.get("edges") or [])[edge_index])
    evidence_lookup = {
        str(item.get("evidence_id")): item
        for item in topology.get("evidences", []) or []
        if isinstance(item, dict) and item.get("evidence_id")
    }
    resolved_evidence = [
        evidence_lookup[evidence_id]
        for evidence_id in edge.get("evidence_ids", []) or []
        if evidence_id in evidence_lookup
    ]
    gateways = snapshot_gateway_matches(snapshot, source, target)
    verticals = snapshot_vertical_matches(snapshot, source, target)
    neighbors, connectivity = world_model_pair_fields(world_model, source, target)
    routes = route_matches(scene_id, source, target, relation_type, edge_index, sidecar_root)
    selected_routes = [item for item in routes if item.get("candidate_relation_selected")]
    pair_visible_routes = routes
    first_route = (selected_routes or pair_visible_routes or [{}])[0]
    explanation = dict(first_route.get("explanation") or {})
    evidence_source_refs = [str(item.get("source_ref") or "") for item in resolved_evidence]
    evidence_types = [str(item.get("evidence_type") or "") for item in resolved_evidence]
    route_visibility_label = str(first_route.get("route_visibility_label") or "not_route_visible")
    relation_match_precision = str(first_route.get("relation_match_precision") or "unavailable")
    same_pair_alternate_relation_count = int(explanation.get("same_pair_alternate_relation_count") or 0)

    if selected_routes and not gateways and relation_type == "adjacent":
        diagnosis = "route-selected weak geometric adjacency without gateway-backed passage; expected export semantics, but route scoring/caption merit review"
        recommended_action = "improve_visualization_caption"
    elif route_visibility_label == "same_pair_alternate_relation_visible":
        diagnosis = "candidate pair is on a selected route, but this specific relation is a same-pair alternate rather than the selected route edge"
        recommended_action = "improve_audit_matching"
    elif route_visibility_label == "same_pair_relation_visible":
        diagnosis = "candidate pair is route-visible, but available artifact metadata is insufficient for a stronger relation-specific match"
        recommended_action = "improve_visualization_caption"
    elif relation_type == "possible_connection" and not gateways:
        diagnosis = "possible_connection is weak/provisional and is not expected to require a gateway record"
        recommended_action = "improve_visualization_caption"
    elif not gateways and any(ref == "polygon_proximity" for ref in evidence_source_refs):
        diagnosis = "gateway absence matches polygon-proximity topology evidence rather than gateway generation"
        recommended_action = "improve_visualization_caption"
    else:
        diagnosis = "unclear_needs_manual_review"
        recommended_action = "investigate_gateway_generation_deeper"

    route_notes = {
        "pair_visible_route_count": len(pair_visible_routes),
        "candidate_relation_selected_route_count": len(selected_routes),
        "route_pages": [
            {
                "json_sidecar_path": item.get("json_sidecar_path"),
                "html_path": item.get("html_path"),
                "route_step_index": item.get("route_step_index"),
                "selected_relation_type": item.get("selected_relation_type"),
                "candidate_relation_selected": item.get("candidate_relation_selected"),
            }
            for item in pair_visible_routes
        ],
    }
    notes = {
        "scene_root": str(scene_root),
        "topology_metadata": edge.get("metadata"),
        "audit_rows": audit_rows(scene_id, source, target, relation_type, edge_index),
        "known_future_bev_path": "/home/ws/workspace/ws_geom",
    }
    return {
        "scene_id": scene_id,
        "candidate_id": row.get("candidate_id"),
        "edge_index": edge_index,
        "source_room": source,
        "target_room": target,
        "relation_type": relation_type,
        "confidence": edge.get("confidence"),
        "support_count": edge.get("support_count"),
        "status": edge.get("status"),
        "evidence_ids": edge.get("evidence_ids") or [],
        "resolved_evidence_types": evidence_types,
        "resolved_evidence_source_refs": evidence_source_refs,
        "resolved_evidence_notes": [str(item.get("notes") or "") for item in resolved_evidence],
        "has_snapshot_gateway_pair": bool(gateways),
        "snapshot_gateway_count": len(gateways),
        "gateway_details": [compact_gateway(item) for item in gateways],
        "has_snapshot_vertical_transition_pair": bool(verticals),
        "snapshot_vertical_transition_count": len(verticals),
        "vertical_transition_details": [compact_transition(item) for item in verticals],
        "world_model_neighbor_relation": neighbors,
        "world_model_connectivity_relation": connectivity,
        "route_step_index": explanation.get("step_index"),
        "route_visibility_label": route_visibility_label,
        "relation_match_precision": relation_match_precision,
        "selected_route_relation_type": explanation.get("relation_type"),
        "selected_route_edge_index": explanation.get("selected_edge_index"),
        "same_pair_alternate_relation_count": same_pair_alternate_relation_count,
        "route_edge_notes": route_notes,
        "diagnosis": diagnosis,
        "recommended_action": recommended_action,
        "notes": notes,
    }


def write_markdown(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    lines = [
        "# Route-Visible Edge Trace",
        "",
        "Read-only trace of Step 7 target topology edge candidates against committed/public artifacts and Step 5 sidecars.",
        "",
    ]
    for row in rows:
        lines.extend(
            [
                f"## {row['scene_id']} / {row['candidate_id']}",
                "",
                f"- Edge: {row['source_room']} - {row['target_room']} ({row['relation_type']}, edge_index={row['edge_index']})",
                f"- Confidence/support/status: {row['confidence']} / {row['support_count']} / {row['status']}",
                f"- Evidence types: {', '.join(row['resolved_evidence_types']) or 'none'}",
                f"- Evidence refs: {', '.join(row['resolved_evidence_source_refs']) or 'none'}",
                f"- Snapshot gateways: {row['snapshot_gateway_count']}",
                f"- Snapshot vertical transitions: {row['snapshot_vertical_transition_count']}",
                f"- Route visibility label: {row['route_visibility_label']}",
                f"- Relation match precision: {row['relation_match_precision']}",
                f"- Diagnosis: {row['diagnosis']}",
                f"- Recommended action: {row['recommended_action']}",
                "",
            ]
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--targets-csv", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sidecar-root", type=Path, default=STEP5_ROOT)
    args = parser.parse_args()

    targets = _load_csv(args.targets_csv)
    rows = [trace_target(row, args.sidecar_root) for row in targets]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.out_dir / "route_visible_edge_trace.csv", rows, TRACE_FIELDS)
    _write_json(args.out_dir / "route_visible_edge_trace.json", rows)
    write_markdown(args.out_dir / "route_visible_edge_trace.md", rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
