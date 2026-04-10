from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from boxfusion.online_topology_lifecycle import RoomLifecycleState


DEFAULT_WORKING_TOPOLOGY_JSON_NAME = "working_topology_v0_1.json"
DEFAULT_COMPARISON_JSON_NAME = "working_vs_committed_topology_report_v0_1.json"

WORKING_ELIGIBLE_LIFECYCLE_STATES = {
    RoomLifecycleState.ACTIVE.value,
    RoomLifecycleState.CANDIDATE_COMPLETE.value,
    RoomLifecycleState.COMMITTED.value,
    RoomLifecycleState.REVISITABLE.value,
    RoomLifecycleState.MERGE_OR_SPLIT_PENDING.value,
}


def load_json(path: Path) -> Dict[str, Any]:
    return dict(json.loads(Path(path).read_text(encoding="utf-8")))


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def default_output_paths(topology_path: Path) -> Tuple[Path, Path]:
    base_dir = Path(topology_path).parent
    return (
        base_dir / DEFAULT_WORKING_TOPOLOGY_JSON_NAME,
        base_dir / DEFAULT_COMPARISON_JSON_NAME,
    )


def resolve_topology_and_lifecycle_artifacts(input_path: Path) -> Tuple[Path, Path]:
    path = Path(input_path)
    if path.is_dir():
        candidates = [
            (path / "topology_v0_1.json", path / "online_topology_lifecycle_v0_1.json"),
            (path / "logs" / "topology_v0_1.json", path / "logs" / "online_topology_lifecycle_v0_1.json"),
        ]
        for topology_path, lifecycle_path in candidates:
            if topology_path.exists() and lifecycle_path.exists():
                return topology_path, lifecycle_path
        raise FileNotFoundError(f"Could not resolve topology+lifecycle artifacts under: {path}")

    if path.name == "summary.json":
        summary = load_json(path)
        topology_candidates: List[Path] = []
        lifecycle_candidates: List[Path] = []
        raw_topology = str(summary.get("topology_v0_1_json") or "").strip()
        raw_lifecycle = str(summary.get("online_topology_lifecycle_json") or "").strip()
        if raw_topology:
            topology_candidates.extend([Path(raw_topology), path.parent / Path(raw_topology).name])
        if raw_lifecycle:
            lifecycle_candidates.extend([Path(raw_lifecycle), path.parent / Path(raw_lifecycle).name])
        topology_candidates.append(path.parent / "topology_v0_1.json")
        lifecycle_candidates.append(path.parent / "online_topology_lifecycle_v0_1.json")
        for topology_path in topology_candidates:
            for lifecycle_path in lifecycle_candidates:
                if topology_path.exists() and lifecycle_path.exists():
                    return topology_path, lifecycle_path
        raise FileNotFoundError(f"Could not resolve topology+lifecycle artifacts from summary: {path}")

    if path.name == "topology_v0_1.json":
        lifecycle_path = path.parent / "online_topology_lifecycle_v0_1.json"
        if lifecycle_path.exists():
            return path, lifecycle_path
        raise FileNotFoundError(f"Missing sibling lifecycle artifact for topology input: {path}")

    if path.name == "online_topology_lifecycle_v0_1.json":
        topology_path = path.parent / "topology_v0_1.json"
        if topology_path.exists():
            return topology_path, path
        raise FileNotFoundError(f"Missing sibling topology artifact for lifecycle input: {path}")

    raise FileNotFoundError(f"Unsupported input path for working-topology resolution: {path}")


def _canonical_room_ids(values: Iterable[Any]) -> List[str]:
    room_ids: List[str] = []
    seen: Set[str] = set()
    for value in values:
        canonical = _canonical_room_id(value)
        if canonical is None or canonical in seen:
            continue
        seen.add(canonical)
        room_ids.append(canonical)
    return room_ids


def _canonical_room_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return None
        if token.startswith("room_"):
            suffix = token[5:]
            if suffix.lstrip("-").isdigit() and int(suffix) < 0:
                return None
            return token
        if token.lstrip("-").isdigit():
            room_uuid = int(token)
            if room_uuid < 0:
                return None
            return f"room_{room_uuid}"
        return token
    room_uuid = int(value)
    if room_uuid < 0:
        return None
    return f"room_{room_uuid}"


def _canonical_object_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    if isinstance(value, str):
        token = value.strip()
        if not token:
            return None
        if token.startswith("obj_"):
            suffix = token[4:]
            if suffix.lstrip("-").isdigit() and int(suffix) < 0:
                return None
            return token
        if token.lstrip("-").isdigit():
            object_uuid = int(token)
            if object_uuid < 0:
                return None
            return f"obj_{object_uuid}"
        return token
    object_uuid = int(value)
    if object_uuid < 0:
        return None
    return f"obj_{object_uuid}"


def _canonical_anchor_id(value: Any) -> Optional[str]:
    if value is None:
        return None
    token = str(value).strip()
    if not token:
        return None
    if token.lstrip("-").isdigit():
        anchor_uuid = int(token)
        if anchor_uuid < 0:
            return None
        return f"anchor_{anchor_uuid}"
    return token


def _lifecycle_room_lookup(lifecycle_payload: Optional[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(dict(lifecycle_payload or {}).get("rooms") or []):
        room_id = _canonical_room_id(room.get("room_id"))
        if room_id is None:
            continue
        lookup[room_id] = dict(room)
    return lookup


def _topology_room_lookup(topology_payload: Optional[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    lookup: Dict[str, Dict[str, Any]] = {}
    for room in list(dict(topology_payload or {}).get("rooms") or []):
        room_id = _canonical_room_id(room.get("id"))
        if room_id is None:
            continue
        lookup[room_id] = dict(room)
    return lookup


def _working_room_ids(
    topology_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
) -> Tuple[List[str], List[str]]:
    topology_room_ids = set(_topology_room_lookup(topology_payload))
    lifecycle_rooms = _lifecycle_room_lookup(lifecycle_payload)
    eligible_room_ids: List[str] = []
    missing_from_topology: List[str] = []
    for room_id, room in sorted(lifecycle_rooms.items()):
        if not bool(room.get("present_in_latest_export")):
            continue
        if str(room.get("lifecycle_state")) not in WORKING_ELIGIBLE_LIFECYCLE_STATES:
            continue
        if room_id in topology_room_ids:
            eligible_room_ids.append(room_id)
        else:
            missing_from_topology.append(room_id)
    return eligible_room_ids, missing_from_topology


def _canonicalized_object_room_map(payload: Dict[str, Any]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for object_id, room_id in dict(payload.get("indices", {}).get("object_to_room", {})).items():
        canonical_object_id = _canonical_object_id(object_id)
        canonical_room_id = _canonical_room_id(room_id)
        if canonical_object_id is None or canonical_room_id is None:
            continue
        mapping[canonical_object_id] = canonical_room_id
    return mapping


def _canonicalized_anchor_room_map(payload: Dict[str, Any]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    for anchor_id, room_id in dict(payload.get("indices", {}).get("anchor_to_room", {})).items():
        canonical_anchor_id = _canonical_anchor_id(anchor_id)
        canonical_room_id = _canonical_room_id(room_id)
        if canonical_anchor_id is None or canonical_room_id is None:
            continue
        mapping[canonical_anchor_id] = canonical_room_id
    return mapping


def _filter_topology_payload(topology_payload: Dict[str, Any], allowed_room_ids: Iterable[Any]) -> Dict[str, Any]:
    allowed = set(_canonical_room_ids(allowed_room_ids))
    payload = dict(topology_payload or {})

    rooms = [
        dict(room)
        for room in list(payload.get("rooms") or [])
        if _canonical_room_id(room.get("id")) in allowed
    ]
    edges = [
        dict(edge)
        for edge in list(payload.get("edges") or [])
        if _canonical_room_id(edge.get("source")) in allowed and _canonical_room_id(edge.get("target")) in allowed
    ]

    object_to_room = _canonicalized_object_room_map(payload)
    anchor_to_room = _canonicalized_anchor_room_map(payload)

    filtered_object_to_room = {
        object_id: room_id
        for object_id, room_id in sorted(object_to_room.items())
        if room_id in allowed
    }
    filtered_anchor_to_room = {
        anchor_id: room_id
        for anchor_id, room_id in sorted(anchor_to_room.items())
        if room_id in allowed
    }

    room_to_objects = {
        room_id: [
            canonical_object_id
            for object_id in list(object_ids or [])
            for canonical_object_id in [_canonical_object_id(object_id)]
            if canonical_object_id is not None and filtered_object_to_room.get(canonical_object_id) == room_id
        ]
        for room_id, object_ids in sorted(dict(payload.get("indices", {}).get("room_to_objects", {})).items())
        for room_id in [_canonical_room_id(room_id)]
        if room_id in allowed
    }
    room_to_anchors = {
        room_id: [
            canonical_anchor_id
            for anchor_id in list(anchor_ids or [])
            for canonical_anchor_id in [_canonical_anchor_id(anchor_id)]
            if canonical_anchor_id is not None and filtered_anchor_to_room.get(canonical_anchor_id) == room_id
        ]
        for room_id, anchor_ids in sorted(dict(payload.get("indices", {}).get("room_to_anchors", {})).items())
        for room_id in [_canonical_room_id(room_id)]
        if room_id in allowed
    }

    object_ids_from_entities = {
        canonical_object_id
        for item in list(dict(payload.get("entities") or {}).get("objects") or [])
        for room_id in [_canonical_room_id(item.get("room_id"))]
        for canonical_object_id in [_canonical_object_id(item.get("id"))]
        if room_id in allowed and canonical_object_id is not None
    }
    anchor_ids_from_entities = {
        canonical_anchor_id
        for item in list(dict(payload.get("entities") or {}).get("anchors") or [])
        for room_id in [_canonical_room_id(item.get("room_id"))]
        for canonical_anchor_id in [_canonical_anchor_id(item.get("id"))]
        if room_id in allowed and canonical_anchor_id is not None
    }

    allowed_object_ids = set(filtered_object_to_room) | object_ids_from_entities
    allowed_anchor_ids = set(filtered_anchor_to_room) | anchor_ids_from_entities

    entities = dict(payload.get("entities") or {})
    filtered_payload = {
        "version": payload.get("version"),
        "sequence_id": payload.get("sequence_id"),
        "floors": list(payload.get("floors") or []),
        "rooms": rooms,
        "edges": edges,
        "indices": {
            "object_to_room": filtered_object_to_room,
            "anchor_to_room": filtered_anchor_to_room,
            "room_to_objects": {room_id: list(dict.fromkeys(object_ids)) for room_id, object_ids in room_to_objects.items()},
            "room_to_anchors": {room_id: list(dict.fromkeys(anchor_ids)) for room_id, anchor_ids in room_to_anchors.items()},
        },
        "entities": {
            "objects": [
                dict(item)
                for item in list(entities.get("objects") or [])
                if _canonical_object_id(item.get("id")) in allowed_object_ids
            ],
            "anchors": [
                dict(item)
                for item in list(entities.get("anchors") or [])
                if _canonical_anchor_id(item.get("id")) in allowed_anchor_ids
            ],
        },
        "metadata": dict(payload.get("metadata") or {}),
    }

    referenced_evidence_ids = {
        str(evidence_id)
        for edge in filtered_payload["edges"]
        for evidence_id in list(edge.get("evidence_ids") or [])
        if evidence_id is not None
    }
    filtered_payload["evidences"] = [
        dict(item)
        for item in list(payload.get("evidences") or [])
        if str(item.get("evidence_id")) in referenced_evidence_ids
    ]
    return filtered_payload


def _canonicalize_filtered_topology(payload: Dict[str, Any]) -> Dict[str, Any]:
    canonical_payload = dict(payload or {})
    canonical_payload["rooms"] = sorted(
        [dict(room) for room in list(canonical_payload.get("rooms") or [])],
        key=lambda room: str(room.get("id")),
    )
    canonical_payload["edges"] = sorted(
        [dict(edge) for edge in list(canonical_payload.get("edges") or [])],
        key=lambda edge: (
            str(edge.get("source")),
            str(edge.get("target")),
            str(edge.get("relation_type")),
        ),
    )
    canonical_payload["evidences"] = sorted(
        [dict(item) for item in list(canonical_payload.get("evidences") or [])],
        key=lambda item: str(item.get("evidence_id")),
    )
    indices = dict(canonical_payload.get("indices") or {})
    canonical_payload["indices"] = {
        "object_to_room": dict(sorted(dict(indices.get("object_to_room") or {}).items())),
        "anchor_to_room": dict(sorted(dict(indices.get("anchor_to_room") or {}).items())),
        "room_to_objects": {
            str(room_id): list(dict.fromkeys(object_ids))
            for room_id, object_ids in sorted(dict(indices.get("room_to_objects") or {}).items())
        },
        "room_to_anchors": {
            str(room_id): list(dict.fromkeys(anchor_ids))
            for room_id, anchor_ids in sorted(dict(indices.get("room_to_anchors") or {}).items())
        },
    }
    entities = dict(canonical_payload.get("entities") or {})
    canonical_payload["entities"] = {
        "objects": sorted(
            [dict(item) for item in list(entities.get("objects") or [])],
            key=lambda item: str(item.get("id")),
        ),
        "anchors": sorted(
            [dict(item) for item in list(entities.get("anchors") or [])],
            key=lambda item: str(item.get("id")),
        ),
    }
    canonical_payload.pop("query_examples", None)
    return canonical_payload


def _working_status_summary(lifecycle_payload: Dict[str, Any], room_ids: Iterable[str]) -> Dict[str, Any]:
    lifecycle_lookup = _lifecycle_room_lookup(lifecycle_payload)
    selected_room_ids = [room_id for room_id in _canonical_room_ids(room_ids) if room_id in lifecycle_lookup]
    state_counts: Dict[str, int] = {}
    blocker_counts: Dict[str, int] = {}
    candidate_complete_count = 0
    active_count = 0
    blocked_count = 0
    for room_id in selected_room_ids:
        room = lifecycle_lookup[room_id]
        state = str(room.get("lifecycle_state") or "unknown")
        state_counts[state] = int(state_counts.get(state, 0)) + 1
        if bool(room.get("candidate_complete")):
            candidate_complete_count += 1
        if state == RoomLifecycleState.ACTIVE.value:
            active_count += 1
        commit_block_reasons = [str(item) for item in list(room.get("commit_block_reasons") or []) if item not in (None, "")]
        if commit_block_reasons:
            blocked_count += 1
        for reason in commit_block_reasons:
            blocker_counts[reason] = int(blocker_counts.get(reason, 0)) + 1
    return {
        "room_count": int(len(selected_room_ids)),
        "candidate_complete_room_count": int(candidate_complete_count),
        "active_room_count": int(active_count),
        "blocked_from_commit_room_count": int(blocked_count),
        "lifecycle_state_counts": dict(sorted(state_counts.items())),
        "commit_block_reason_counts": dict(sorted(blocker_counts.items())),
    }


def _attach_room_debug_lifecycle(
    topology_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
    *,
    committed_room_ids: Optional[Sequence[Any]] = None,
) -> Dict[str, Any]:
    payload = dict(topology_payload or {})
    payload["rooms"] = [dict(room) for room in list(payload.get("rooms") or [])]
    lifecycle_lookup = _lifecycle_room_lookup(lifecycle_payload)
    committed_ids = set(_canonical_room_ids(committed_room_ids or []))
    for room in payload["rooms"]:
        room_id = _canonical_room_id(room.get("id"))
        lifecycle_room = dict(lifecycle_lookup.get(room_id) or {})
        if room_id is None:
            continue
        commit_block_reasons = [str(item) for item in list(lifecycle_room.get("commit_block_reasons") or []) if item not in (None, "")]
        lifecycle_state = str(lifecycle_room.get("lifecycle_state") or "unknown")
        room["debug_lifecycle"] = {
            "room_id": room_id,
            "lifecycle_state": lifecycle_state,
            "candidate_complete": bool(lifecycle_room.get("candidate_complete")),
            "candidate_readiness_score": lifecycle_room.get("candidate_readiness_score"),
            "candidate_block_reasons": list(lifecycle_room.get("candidate_block_reasons") or []),
            "commit_block_reasons": commit_block_reasons,
            "present_in_latest_export": bool(lifecycle_room.get("present_in_latest_export")),
            "dirty": bool(lifecycle_room.get("dirty")),
            "commit_ready": not commit_block_reasons,
            "withheld_from_committed_projection": room_id not in committed_ids,
            "provisional": lifecycle_state != RoomLifecycleState.COMMITTED.value or bool(commit_block_reasons),
        }
    return payload


def _topology_counts(payload: Dict[str, Any]) -> Dict[str, Any]:
    rooms = list(dict(payload or {}).get("rooms") or [])
    edges = list(dict(payload or {}).get("edges") or [])
    relation_counts: Dict[str, int] = {}
    gateway_pair_edge_count = 0
    gateway_count = 0
    for edge in edges:
        relation = str(edge.get("relation_type") or "unknown")
        relation_counts[relation] = int(relation_counts.get(relation, 0)) + 1
        if relation != "adjacent":
            continue
        metadata = dict(edge.get("metadata") or {})
        edge_gateway_count = metadata.get("gateway_count")
        if edge_gateway_count is None:
            continue
        edge_gateway_count = int(edge_gateway_count)
        if edge_gateway_count > 0:
            gateway_pair_edge_count += 1
            gateway_count += edge_gateway_count
    return {
        "room_count": int(len(rooms)),
        "edge_count": int(len(edges)),
        "gateway_count": int(gateway_count),
        "gateway_pair_edge_count": int(gateway_pair_edge_count),
        "relation_type_counts": dict(sorted(relation_counts.items())),
        "room_ids": [str(room.get("id")) for room in rooms if room.get("id") is not None],
    }


def build_working_topology_snapshot(
    topology_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
    *,
    committed_room_ids: Optional[Sequence[Any]] = None,
    source_artifacts: Optional[Dict[str, Any]] = None,
    lifecycle_semantics: str = "final_report_only",
) -> Dict[str, Any]:
    working_room_ids, missing_from_topology = _working_room_ids(topology_payload, lifecycle_payload)
    filtered_payload = _filter_topology_payload(topology_payload, working_room_ids)
    working_payload = _canonicalize_filtered_topology(filtered_payload)
    working_payload = _attach_room_debug_lifecycle(
        working_payload,
        lifecycle_payload,
        committed_room_ids=committed_room_ids,
    )
    working_payload["metadata"] = dict(working_payload.get("metadata") or {})
    working_payload["metadata"].update(
        {
            "artifact_kind": "working_topology_debug",
            "debug_only": True,
            "public_default": False,
            "truth_owner": "world_export",
            "derived_layer": True,
            "working_semantics": {
                "selection_rule": "present_in_latest_export && lifecycle_state in eligible_working_states",
                "eligible_lifecycle_states": sorted(WORKING_ELIGIBLE_LIFECYCLE_STATES),
                "includes_candidate_complete": True,
                "includes_active": True,
                "includes_blocked_rooms": True,
                "excludes_discovering": True,
                "non_public": True,
                "lifecycle_semantics": str(lifecycle_semantics),
            },
            "source_artifacts": dict(source_artifacts or {}),
        }
    )
    working_summary = _working_status_summary(lifecycle_payload, working_room_ids)
    working_summary["rooms_missing_from_topology_source"] = list(missing_from_topology)
    working_summary["selected_room_ids"] = list(working_room_ids)
    working_summary["committed_room_ids_for_projection"] = _canonical_room_ids(committed_room_ids or [])
    working_payload["working_summary"] = working_summary
    return working_payload


def build_committed_topology_projection(
    topology_payload: Dict[str, Any],
    committed_room_ids: Sequence[Any],
) -> Dict[str, Any]:
    filtered_payload = _filter_topology_payload(topology_payload, committed_room_ids)
    committed_payload = _canonicalize_filtered_topology(filtered_payload)
    committed_payload["metadata"] = dict(committed_payload.get("metadata") or {})
    committed_payload["metadata"].update(
        {
            "artifact_kind": "committed_topology_projection_debug",
            "debug_only": True,
            "public_default": False,
            "projection_rule": "public topology payload filtered to lifecycle committed room ids",
        }
    )
    return committed_payload


def build_working_vs_committed_report(
    working_topology_payload: Dict[str, Any],
    public_topology_payload: Dict[str, Any],
    lifecycle_payload: Dict[str, Any],
    *,
    committed_room_ids: Optional[Sequence[Any]] = None,
    source_artifacts: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    committed_ids = _canonical_room_ids(committed_room_ids or lifecycle_payload.get("committed_rooms") or [])
    committed_projection = build_committed_topology_projection(public_topology_payload, committed_ids)

    working_counts = _topology_counts(working_topology_payload)
    committed_counts = _topology_counts(committed_projection)
    working_room_ids = set(working_counts["room_ids"])
    committed_room_ids_set = set(committed_counts["room_ids"])
    working_only_room_ids = sorted(working_room_ids - committed_room_ids_set)
    committed_only_room_ids = sorted(committed_room_ids_set - working_room_ids)
    lifecycle_lookup = _lifecycle_room_lookup(lifecycle_payload)

    working_only_rooms: List[Dict[str, Any]] = []
    lifecycle_state_counts: Dict[str, int] = {}
    blocker_counts: Dict[str, int] = {}
    for room_id in working_only_room_ids:
        room = dict(lifecycle_lookup.get(room_id) or {})
        lifecycle_state = str(room.get("lifecycle_state") or "unknown")
        commit_block_reasons = [str(item) for item in list(room.get("commit_block_reasons") or []) if item not in (None, "")]
        lifecycle_state_counts[lifecycle_state] = int(lifecycle_state_counts.get(lifecycle_state, 0)) + 1
        for reason in commit_block_reasons:
            blocker_counts[reason] = int(blocker_counts.get(reason, 0)) + 1
        working_only_rooms.append(
            {
                "room_id": room_id,
                "lifecycle_state": lifecycle_state,
                "candidate_complete": bool(room.get("candidate_complete")),
                "commit_ready": not commit_block_reasons,
                "blocked_from_commit": bool(commit_block_reasons),
                "commit_block_reasons": commit_block_reasons,
                "candidate_block_reasons": list(room.get("candidate_block_reasons") or []),
                "dirty": bool(room.get("dirty")),
                "present_in_latest_export": bool(room.get("present_in_latest_export")),
                "provisional_class": (
                    "candidate_complete"
                    if bool(room.get("candidate_complete"))
                    else lifecycle_state
                ),
            }
        )

    room_count_difference = int(working_counts["room_count"] - committed_counts["room_count"])
    edge_count_difference = int(working_counts["edge_count"] - committed_counts["edge_count"])
    gateway_count_difference = int(working_counts["gateway_count"] - committed_counts["gateway_count"])
    report = {
        "version": "0.1",
        "sequence_id": working_topology_payload.get("sequence_id") or public_topology_payload.get("sequence_id"),
        "artifact_kind": "working_vs_committed_topology_report_debug",
        "debug_only": True,
        "public_default": False,
        "non_public": True,
        "truth_owner": "world_export",
        "source_artifacts": dict(source_artifacts or {}),
        "comparison_semantics": {
            "working_topology_definition": "debug-only filtered topology for present rooms in eligible working lifecycle states",
            "committed_topology_definition": "debug-only projection of the unchanged public topology payload onto lifecycle committed room ids",
            "public_query_api_unchanged": True,
        },
        "working_topology": working_counts,
        "committed_topology_projection": committed_counts,
        "difference_summary": {
            "room_count_difference": room_count_difference,
            "edge_count_difference": edge_count_difference,
            "gateway_count_difference": gateway_count_difference,
            "working_only_room_count": int(len(working_only_room_ids)),
            "working_only_rooms": list(working_only_room_ids),
            "committed_only_room_count": int(len(committed_only_room_ids)),
            "committed_only_rooms": list(committed_only_room_ids),
        },
        "working_only_rooms": working_only_rooms,
        "withheld_topology_summary": {
            "withheld_room_count": int(len(working_only_room_ids)),
            "withheld_edge_count": int(edge_count_difference),
            "withheld_gateway_count": int(gateway_count_difference),
            "lifecycle_state_counts": dict(sorted(lifecycle_state_counts.items())),
            "commit_block_reason_counts": dict(sorted(blocker_counts.items())),
        },
        "rooms_blocked_from_commit": [
            room for room in working_only_rooms if room.get("blocked_from_commit")
        ],
    }
    return report
