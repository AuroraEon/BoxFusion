#!/usr/bin/env python3
"""Shared offline helpers for task14a object-navigation artifacts."""

from __future__ import annotations

import csv
import difflib
import json
import re
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task14a_object_nav_experiment_adapter"
CLEAN_RERUN = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
PUBLIC_DIR = CLEAN_RERUN / "committed_public"
TASK_DIR = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks" / TASK_NAME
FLOOR2_MAP_YAML = CLEAN_RERUN / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
FLOOR2_ROUTE_DIR = CLEAN_RERUN / "routes/room_routes/floor_2_room11_to_room14"
FLOOR2_SEMANTIC_ROUTE = FLOOR2_ROUTE_DIR / "semantic_route_waypoints_v0_1.json"
FLOOR2_EXEC_ROUTE = FLOOR2_ROUTE_DIR / "executable_route_waypoints_v0_1.json"

TOPOLOGY_JSON = PUBLIC_DIR / "topology_v0_1.json"
TOPOLOGY_QUERY_REPORT_JSON = PUBLIC_DIR / "topology_query_report.json"
COMMITTED_SNAPSHOT_JSON = PUBLIC_DIR / "committed_room_world_snapshot_v0_1.json"
COMMITTED_MODEL_JSON = PUBLIC_DIR / "committed_room_world_model_v0_1.json"
FINAL_VECTOR_JSON = PUBLIC_DIR / "final_vector_map_snapshot.json"

NOISY_LABELS = {
    "sky",
    "snow",
    "oyster",
    "floor",
    "ceiling",
    "wall_wood",
    "roof",
}

ALIAS_MAP = {
    "sofa": "couch",
    "settee": "couch",
    "plant": "potted_plant",
    "potted plant": "potted_plant",
    "frame": "picture_frame",
    "picture": "picture_frame",
    "picture frame": "picture_frame",
    "photo": "picture_frame",
    "lamp": "light",
    "bedside table": "nightstand",
    "night stand": "nightstand",
    "outlet": "power_outlet",
    "power socket": "power_outlet",
    "aircon": "air_conditioner",
    "air conditioner": "air_conditioner",
    "toiletpaper": "toilet_paper",
    "toilet paper": "toilet_paper",
    "coffee table": "coffee_table",
    "window other": "window_other",
    "window-other": "window_other",
    "cigarette": "cigar_cigarette",
    "cigar": "cigar_cigarette",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def load_json(path: Path, default: Any | None = None) -> Any:
    if not path.exists():
        if default is not None:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def normalize_label(text: Any) -> str:
    text = "" if text is None else str(text)
    text = text.lower().strip()
    text = text.replace("&", " and ")
    text = re.sub(r"[^a-z0-9]+", "_", text)
    return re.sub(r"_+", "_", text).strip("_")


def canonical_label(text: Any) -> str:
    normalized = normalize_label(text)
    return ALIAS_MAP.get(normalized.replace("_", " "), ALIAS_MAP.get(normalized, normalized))


def canonical_object_id(value: Any) -> str:
    text = str(value)
    if text.startswith("obj_"):
        return text
    return f"obj_{text}"


def object_id_number(value: Any) -> int | None:
    text = str(value)
    if text.startswith("obj_"):
        text = text[4:]
    try:
        return int(text)
    except ValueError:
        return None


def score_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def route_assets_status() -> dict[str, Any]:
    map_exists = FLOOR2_MAP_YAML.exists()
    semantic_exists = FLOOR2_SEMANTIC_ROUTE.exists()
    exec_exists = FLOOR2_EXEC_ROUTE.exists()
    room_sequence: list[str] = []
    route_id = "floor_2_room11_to_room14"
    if exec_exists:
        route = load_json(FLOOR2_EXEC_ROUTE, {})
        room_sequence = list(route.get("room_sequence") or [])
        route_id = route.get("route_id") or route_id
    return {
        "floor_id": "floor_2",
        "map_yaml": str(FLOOR2_MAP_YAML),
        "map_yaml_exists": map_exists,
        "semantic_route_waypoints": str(FLOOR2_SEMANTIC_ROUTE),
        "semantic_route_waypoints_exists": semantic_exists,
        "executable_route_waypoints": str(FLOOR2_EXEC_ROUTE),
        "executable_route_waypoints_exists": exec_exists,
        "route_id": route_id,
        "room_sequence": room_sequence,
        "available": map_exists and semantic_exists and exec_exists and bool(room_sequence),
    }


def room_lookup(topology: dict[str, Any], snapshot: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rooms = {}
    for room in topology.get("rooms", []) + snapshot.get("rooms", []):
        room_id = room.get("id") or room.get("room_id")
        if room_id:
            rooms[room_id] = room
    return rooms


def embedding_vector_status(artifacts: list[Path]) -> dict[str, Any]:
    vector_paths: list[str] = []
    refs_seen = 0
    refs_resolved = 0
    for artifact in artifacts:
        if not artifact.exists():
            continue
        data = load_json(artifact, {})
        for obj in data.get("objects", []):
            if obj.get("embedding_ref"):
                refs_seen += 1
            if any(k in obj for k in ("embedding", "embedding_vector", "clip_embedding")):
                refs_resolved += 1
    for path in CLEAN_RERUN.rglob("*"):
        if path.is_file() and re.search(r"(embed|clip|faiss)", path.name, re.I):
            vector_paths.append(rel(path))
    return {
        "embedding_refs_seen": refs_seen,
        "embedding_vectors_inline_seen": refs_resolved,
        "candidate_embedding_files": sorted(vector_paths),
        "embedding_vectors_resolved": refs_resolved > 0 or bool(vector_paths),
    }


def build_candidate_index_data(
    topology_path: Path = TOPOLOGY_JSON,
    snapshot_path: Path = COMMITTED_SNAPSHOT_JSON,
    final_vector_path: Path = FINAL_VECTOR_JSON,
) -> dict[str, Any]:
    topology = load_json(topology_path, {})
    snapshot = load_json(snapshot_path, {})
    final_vector = load_json(final_vector_path, {}) if final_vector_path.exists() else {}
    rooms = room_lookup(topology, snapshot)
    route_status = route_assets_status()
    route_rooms = set(route_status.get("room_sequence") or [])
    topo_by_id = {
        canonical_object_id(o.get("id")): o
        for o in topology.get("entities", {}).get("objects", [])
        if o.get("id") is not None
    }
    snap_by_id = {
        canonical_object_id(o.get("id")): o
        for o in snapshot.get("objects", [])
        if o.get("id") is not None
    }
    raw_by_id = {
        canonical_object_id(o.get("id")): o
        for o in final_vector.get("objects", [])
        if o.get("id") is not None
    }
    topology_object_to_room = topology.get("indices", {}).get("object_to_room", {})
    object_ids = sorted(set(topo_by_id) | set(snap_by_id), key=lambda x: object_id_number(x) or 10**9)
    objects: list[dict[str, Any]] = []
    for oid in object_ids:
        topo = topo_by_id.get(oid, {})
        snap = snap_by_id.get(oid, {})
        raw = raw_by_id.get(oid, {})
        label = snap.get("label") or topo.get("label") or raw.get("label")
        normalized = topo.get("normalized_label") or normalize_label(label)
        room_id = snap.get("room_id") or topo.get("room_id") or raw.get("room_id")
        floor_id = snap.get("floor_id") or topo.get("floor_id") or raw.get("floor_id")
        room = rooms.get(room_id or "")
        floor_assignment = snap.get("floor_assignment") or raw.get("floor_assignment") or {}
        floor_assignment_status = floor_assignment.get("status") or "unknown"
        binding_status = "explicit_valid"
        binding_notes: list[str] = []
        if not room_id:
            binding_status = "missing_room"
            binding_notes.append("object has no room_id")
        elif not room:
            binding_status = "explicit_invalid"
            binding_notes.append("room_id does not resolve to a committed room")
        if not floor_id:
            binding_status = "missing_floor" if binding_status == "explicit_valid" else binding_status
            binding_notes.append("object has no floor_id")
        if room and floor_id and room.get("floor_id") and room.get("floor_id") != floor_id:
            binding_status = "room_floor_mismatch"
            binding_notes.append(f"room floor {room.get('floor_id')} differs from object floor {floor_id}")
        if oid in topology_object_to_room and room_id and topology_object_to_room[oid] != room_id:
            binding_status = "topology_index_mismatch"
            binding_notes.append(f"topology object_to_room has {topology_object_to_room[oid]}")
        warnings: list[str] = []
        if normalized in NOISY_LABELS:
            warnings.append("noisy_or_scene_surface_label")
        if floor_assignment_status not in ("stable", "confirmed"):
            warnings.append(f"floor_assignment_{floor_assignment_status}")
        if binding_status != "explicit_valid":
            warnings.append(f"binding_{binding_status}")
        if floor_id == "floor_2":
            if route_status["available"] and room_id in route_rooms:
                route_availability_status = "floor2_runtime_candidate"
            elif route_status["available"]:
                route_availability_status = "missing_map_or_bridge"
                warnings.append("floor2_room_not_in_existing_route_bridge")
            else:
                route_availability_status = "missing_map_or_bridge"
        elif floor_id:
            route_availability_status = "symbolic_only"
        else:
            route_availability_status = "unknown"
        objects.append(
            {
                "object_id": oid,
                "source_ids": {
                    "topology_id": topo.get("id"),
                    "snapshot_id": snap.get("id"),
                    "final_vector_map_id": raw.get("id"),
                },
                "label": label,
                "normalized_label": normalized,
                "category": snap.get("category") or topo.get("category") or raw.get("category"),
                "confidence": score_float(snap.get("score", topo.get("score", raw.get("score")))),
                "score": score_float(snap.get("score", topo.get("score", raw.get("score")))),
                "detection_confidence": score_float(
                    snap.get("detection_confidence", topo.get("detection_confidence", raw.get("detection_confidence")))
                ),
                "semantic_confidence": score_float(
                    snap.get("semantic_confidence", topo.get("semantic_confidence", raw.get("semantic_confidence")))
                ),
                "association_confidence": score_float(snap.get("association_confidence", raw.get("association_confidence")), 0.0),
                "room_id": room_id,
                "floor_id": floor_id,
                "floor_index": snap.get("floor_index", topo.get("floor_index", raw.get("floor_index"))),
                "display_floor_id": snap.get("display_floor_id", topo.get("display_floor_id", raw.get("display_floor_id"))),
                "room_uuid": snap.get("room_uuid", raw.get("room_uuid")),
                "pose_xy": snap.get("pose") or raw.get("pose"),
                "pose_xyz": snap.get("pose_3d") or raw.get("pose_3d"),
                "footprint_2d": snap.get("footprint_2d") or raw.get("footprint_2d"),
                "size": snap.get("size") or raw.get("size"),
                "embedding_ref": snap.get("embedding_ref") or raw.get("embedding_ref"),
                "semantic_observations": snap.get("semantic_observations") or raw.get("semantic_observations") or [],
                "room_assignment": snap.get("room_assignment") or raw.get("room_assignment"),
                "floor_assignment": floor_assignment,
                "floor_assignment_status": floor_assignment_status,
                "binding_status": binding_status,
                "binding_notes": binding_notes,
                "route_availability_status": route_availability_status,
                "is_noisy_label": normalized in NOISY_LABELS,
                "is_uncertain_floor_assignment": floor_assignment_status not in ("stable", "confirmed"),
                "warnings": warnings,
                "source_artifact_provenance": {
                    "topology_v0_1": oid in topo_by_id,
                    "committed_room_world_snapshot_v0_1": oid in snap_by_id,
                    "final_vector_map_snapshot": oid in raw_by_id,
                },
            }
        )
    raw_count = len(final_vector.get("objects", [])) if isinstance(final_vector, dict) else 0
    raw_bound = sum(1 for o in final_vector.get("objects", []) if o.get("room_id")) if isinstance(final_vector, dict) else 0
    return {
        "version": "v0_1",
        "artifact_type": "object_candidate_index",
        "scene_id": SCENE_ID,
        "created_utc": utc_now(),
        "source_artifacts": {
            "topology_v0_1": str(topology_path),
            "committed_room_world_snapshot_v0_1": str(snapshot_path),
            "final_vector_map_snapshot_comparison_only": str(final_vector_path) if final_vector_path.exists() else None,
        },
        "route_asset_status": route_status,
        "counts": {
            "topology_public_objects": len(topo_by_id),
            "committed_snapshot_objects": len(snap_by_id),
            "indexed_objects": len(objects),
            "raw_final_vector_objects": raw_count,
            "raw_final_vector_objects_with_room_id": raw_bound,
            "raw_final_vector_objects_without_room_id": raw_count - raw_bound,
        },
        "objects": objects,
        "notes": [
            "Index is built from committed/public artifacts only.",
            "final_vector_map_snapshot is comparison-only and is not authoritative for object navigation.",
            "route_availability_status does not imply Nav2 execution or object-approach validation.",
        ],
    }


def load_index(path: Path) -> dict[str, Any]:
    return load_json(path, {})


def label_histogram(objects: list[dict[str, Any]]) -> dict[str, int]:
    return dict(sorted(Counter(o.get("normalized_label") or normalize_label(o.get("label")) for o in objects).items()))


def group_counts(objects: list[dict[str, Any]], key: str) -> dict[str, int]:
    return dict(sorted(Counter(str(o.get(key) or "missing") for o in objects).items()))


def route_eligible(obj: dict[str, Any]) -> bool:
    return (
        obj.get("binding_status") == "explicit_valid"
        and obj.get("route_availability_status") == "floor2_runtime_candidate"
        and not obj.get("is_noisy_label")
    )


def parse_query_constraints(query_text: str, preferred_room_id: str | None = None, preferred_floor_id: str | None = None) -> dict[str, Any]:
    text = query_text.strip()
    room_match = re.search(r"\broom[_\s-]*(\d+)\b", text, re.I)
    floor_match = re.search(r"\bfloor[_\s-]*(\d+)\b", text, re.I)
    room_id = preferred_room_id or (f"room_{room_match.group(1)}" if room_match else None)
    floor_id = preferred_floor_id or (f"floor_{floor_match.group(1)}" if floor_match else None)
    label_text = re.sub(r"\b(in|inside|at|near|on)?\s*room[_\s-]*\d+\b", " ", text, flags=re.I)
    label_text = re.sub(r"\b(on|at|in)?\s*floor[_\s-]*\d+\b", " ", label_text, flags=re.I)
    label_text = re.sub(r"\b(find|go to|navigate to|object|the|a|an)\b", " ", label_text, flags=re.I)
    label_text = re.sub(r"\s+", " ", label_text).strip()
    normalized = normalize_label(label_text or text)
    alias_normalized = canonical_label(normalized)
    query_type = "object_only"
    if room_id and floor_id:
        query_type = "object_room_floor"
    elif room_id:
        query_type = "object_room"
    elif floor_id:
        query_type = "object_floor"
    return {
        "query_type": query_type,
        "object_label_text": label_text or text,
        "normalized_label": normalized,
        "canonical_label": alias_normalized,
        "preferred_room_id": room_id,
        "preferred_floor_id": floor_id,
    }


def candidate_match(candidate_label: str, query_label: str) -> tuple[str | None, str, float]:
    if not query_label:
        return None, "empty query label", 0.0
    cand = normalize_label(candidate_label)
    cand_canonical = canonical_label(cand)
    query = normalize_label(query_label)
    canonical = canonical_label(query)
    if cand_canonical == canonical:
        return "exact_or_alias", f"{cand} resolves to canonical label {cand_canonical}", 1.0
    if cand == query:
        return "exact_normalized", f"{cand} equals normalized query label {query}", 0.98
    if canonical in cand_canonical or cand_canonical in canonical:
        return "substring", f"{cand_canonical} and {canonical} overlap by substring", 0.82
    ratio = difflib.SequenceMatcher(None, cand_canonical, canonical).ratio()
    if ratio >= 0.72:
        return "fuzzy", f"{cand} fuzzy-matches {canonical} at {ratio:.2f}", 0.65 + ratio * 0.1
    return None, f"{cand} did not match {canonical}", ratio


def run_query(index: dict[str, Any], query_text: str, preferred_room_id: str | None = None, preferred_floor_id: str | None = None, top_k: int = 10) -> dict[str, Any]:
    constraints = parse_query_constraints(query_text, preferred_room_id, preferred_floor_id)
    ranked: list[dict[str, Any]] = []
    for obj in index.get("objects", []):
        match_type, reason, base = candidate_match(obj.get("normalized_label") or obj.get("label") or "", constraints["canonical_label"])
        if not match_type:
            continue
        room_ok = constraints["preferred_room_id"] in (None, obj.get("room_id"))
        floor_ok = constraints["preferred_floor_id"] in (None, obj.get("floor_id"))
        hard_ok = room_ok and floor_ok
        constraint_bonus = (0.18 if room_ok else -0.4) + (0.18 if floor_ok else -0.4)
        binding_bonus = 0.1 if obj.get("binding_status") == "explicit_valid" else -0.2
        route_bonus = 0.08 if obj.get("route_availability_status") == "floor2_runtime_candidate" else 0.0
        confidence_bonus = score_float(obj.get("confidence")) * 0.18
        noisy_penalty = -0.12 if obj.get("is_noisy_label") else 0.0
        rank_score = base + constraint_bonus + binding_bonus + route_bonus + confidence_bonus + noisy_penalty
        if not hard_ok:
            rank_score -= 1.0
        ranked.append(
            {
                "object_id": obj.get("object_id"),
                "label": obj.get("label"),
                "normalized_label": obj.get("normalized_label"),
                "room_id": obj.get("room_id"),
                "floor_id": obj.get("floor_id"),
                "confidence": obj.get("confidence"),
                "binding_status": obj.get("binding_status"),
                "floor_assignment_status": obj.get("floor_assignment_status"),
                "route_availability_status": obj.get("route_availability_status"),
                "warnings": obj.get("warnings", []),
                "match_type": match_type,
                "match_reason": reason,
                "constraint_satisfaction": {
                    "room": room_ok,
                    "floor": floor_ok,
                    "all_constraints": hard_ok,
                },
                "rank_score": round(rank_score, 6),
            }
        )
    ranked.sort(key=lambda r: (r["constraint_satisfaction"]["all_constraints"], r["rank_score"], r.get("confidence") or 0), reverse=True)
    limited = ranked[:top_k]
    selected = limited[0] if limited and limited[0]["constraint_satisfaction"]["all_constraints"] else None
    failure = None
    if not ranked:
        failure = "no_label_match"
    elif not selected:
        failure = "label_matches_failed_constraints"
    ambiguous = False
    if selected:
        same_label = [
            r for r in ranked
            if r["constraint_satisfaction"]["all_constraints"]
            and r.get("normalized_label") == selected.get("normalized_label")
        ]
        ambiguous = len(same_label) > 1
    return {
        "version": "v0_1",
        "artifact_type": "object_query_result",
        "scene_id": index.get("scene_id", SCENE_ID),
        "query_id": f"query_{normalize_label(query_text)[:64] or 'empty'}",
        "query_text": query_text,
        "parsed_constraints": constraints,
        "ranked_candidates": limited,
        "total_matching_candidates": len(ranked),
        "selected_candidate": selected,
        "ambiguous_query": ambiguous,
        "failure_reason": failure,
        "metric_scope_note": "Artifact-derived retrieval result; not semantic GT accuracy.",
    }


def inventory_from_index(index: dict[str, Any]) -> dict[str, Any]:
    objects = index.get("objects", [])
    floor2 = [o for o in objects if o.get("floor_id") == "floor_2"]
    noisy = [o for o in objects if o.get("is_noisy_label")]
    uncertain = [o for o in objects if o.get("is_uncertain_floor_assignment")]
    missing = [o for o in objects if o.get("binding_status") != "explicit_valid"]
    eligible = [o for o in objects if route_eligible(o)]
    label_counts = Counter(o.get("normalized_label") for o in objects)
    unique_labels = sum(1 for _, n in label_counts.items() if n == 1)
    duplicate_labels = sum(1 for _, n in label_counts.items() if n > 1)
    return {
        "scene_id": index.get("scene_id", SCENE_ID),
        "created_utc": utc_now(),
        "total_public_objects": index.get("counts", {}).get("topology_public_objects"),
        "total_committed_objects": index.get("counts", {}).get("committed_snapshot_objects"),
        "indexed_objects": len(objects),
        "object_labels_histogram": label_histogram(objects),
        "objects_by_floor": group_counts(objects, "floor_id"),
        "objects_by_room": group_counts(objects, "room_id"),
        "floor_2_object_candidates": slim_objects(floor2),
        "noisy_label_candidates": slim_objects(noisy),
        "uncertain_floor_assignment_candidates": slim_objects(uncertain),
        "missing_binding_candidates": slim_objects(missing),
        "route_eligible_candidates": slim_objects(eligible),
        "query_case_counts_by_type": {
            "object_only": len(objects),
            "object_room": sum(1 for o in objects if o.get("room_id")),
            "object_floor": sum(1 for o in objects if o.get("floor_id")),
            "object_room_floor": sum(1 for o in objects if o.get("room_id") and o.get("floor_id")),
            "unique_label_object_only": unique_labels,
            "duplicate_or_ambiguous_label_object_only": duplicate_labels,
            "floor2_runtime_candidate": len(eligible),
        },
    }


def slim_objects(objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "object_id": o.get("object_id"),
            "label": o.get("label"),
            "normalized_label": o.get("normalized_label"),
            "room_id": o.get("room_id"),
            "floor_id": o.get("floor_id"),
            "confidence": o.get("confidence"),
            "binding_status": o.get("binding_status"),
            "floor_assignment_status": o.get("floor_assignment_status"),
            "route_availability_status": o.get("route_availability_status"),
            "warnings": o.get("warnings", []),
        }
        for o in objects
    ]


def markdown_table(rows: list[dict[str, Any]], fields: list[str], limit: int | None = None) -> str:
    if limit is not None:
        rows = rows[:limit]
    if not rows:
        return "_None._\n"
    out = ["| " + " | ".join(fields) + " |", "| " + " | ".join(["---"] * len(fields)) + " |"]
    for row in rows:
        out.append("| " + " | ".join(str(row.get(f, "")) for f in fields) + " |")
    return "\n".join(out) + "\n"
