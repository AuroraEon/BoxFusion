from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


RUNTIME_STATE_SNAPSHOT_CONTRACT_VERSION = "0.1"
RUNTIME_SNAPSHOT_SHADOW_PARITY_SCOPE = "minimal_counts_ids_lifecycle_v0_1"
MINIMAL_PUBLIC_TOPOLOGY_SHADOW_SCOPE = "minimal_committed_public_topology_subset_v0_1"
MINIMAL_PUBLIC_TOPOLOGY_ARTIFACT_KIND = "shadow_minimal_committed_public_topology_subset"
MINIMAL_PUBLIC_TOPOLOGY_REQUIRED_KEYS = (
    "artifact_kind",
    "scope",
    "public_topology_meaning",
    "counts",
    "ids",
    "floors",
    "rooms",
    "edges",
    "object_room_memberships",
    "semantic_summary",
)


def _optional_path_text(path: Optional[Path]) -> Optional[str]:
    return None if path is None else str(Path(path).resolve())


def _as_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return int(default)
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _as_float_or_none(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int_or_none(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _canonical_prefixed_id(value: Any, prefix: str) -> Optional[str]:
    text = _clean_optional_text(value)
    if text is None:
        return None
    if text.startswith(f"{prefix}_"):
        suffix = text[len(prefix) + 1 :]
        if suffix.lstrip("-").isdigit() and int(suffix) < 0:
            return None
        return text
    if text.lstrip("-").isdigit():
        numeric_id = int(text)
        if numeric_id < 0:
            return None
        return f"{prefix}_{numeric_id}"
    return text


def _canonical_floor_id(value: Any) -> Optional[str]:
    return _clean_optional_text(value)


def _canonical_room_id(value: Any) -> Optional[str]:
    return _canonical_prefixed_id(value, "room")


def _canonical_object_id(value: Any) -> Optional[str]:
    return _canonical_prefixed_id(value, "obj")


def _canonical_anchor_id(value: Any) -> Optional[str]:
    return _canonical_prefixed_id(value, "anchor")


def _stable_sorted(values: Iterable[Optional[str]]) -> Tuple[str, ...]:
    return tuple(sorted({str(value) for value in values if value is not None}, key=str))


def _extract_ids(
    items: Sequence[Any],
    *,
    key: str,
    canonicalizer: Any,
) -> Tuple[str, ...]:
    values: List[Optional[str]] = []
    for item in list(items or []):
        if not isinstance(item, dict):
            continue
        values.append(canonicalizer(item.get(key)))
    return _stable_sorted(values)


def _extract_public_subset_from_topology(topology_payload: Dict[str, Any]) -> Dict[str, Any]:
    topology_entities = dict(topology_payload.get("entities") or {})
    floors = list(topology_payload.get("floors") or [])
    rooms = list(topology_payload.get("rooms") or [])
    edges = list(topology_payload.get("edges") or [])
    objects = list(topology_entities.get("objects") or [])
    anchors = list(topology_entities.get("anchors") or [])
    return {
        "floor_count": _as_int(topology_payload.get("floor_count"), default=len(floors)),
        "room_count": _as_int(topology_payload.get("room_count"), default=len(rooms)),
        "edge_count": _as_int(topology_payload.get("edge_count"), default=len(edges)),
        "object_count": len(objects),
        "anchor_count": len(anchors),
        "floor_ids": _extract_ids(floors, key="floor_id", canonicalizer=_canonical_floor_id),
        "room_ids": _extract_ids(rooms, key="id", canonicalizer=_canonical_room_id),
        "object_ids": _extract_ids(objects, key="id", canonicalizer=_canonical_object_id),
        "anchor_ids": _extract_ids(anchors, key="id", canonicalizer=_canonical_anchor_id),
    }


def _room_sort_key(room_id: Any) -> Tuple[int, str]:
    text = str(room_id)
    if text.startswith("room_"):
        suffix = text[len("room_") :]
        if suffix.lstrip("-").isdigit():
            return int(suffix), text
    return 10**9, text


def _object_sort_key(object_id: Any) -> Tuple[int, str]:
    text = str(object_id)
    if text.startswith("obj_"):
        suffix = text[len("obj_") :]
        if suffix.lstrip("-").isdigit():
            return int(suffix), text
    return 10**9, text


def _floor_record_from_payload(floor: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    floor_id = _canonical_floor_id(floor.get("floor_id", floor.get("id")))
    if floor_id is None:
        return None
    return {
        "floor_id": floor_id,
        "display_floor_id": _clean_optional_text(floor.get("display_floor_id")) or floor_id,
        "display_order": _as_int_or_none(floor.get("display_order")),
    }


def _room_record_from_payload(room: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    room_id = _canonical_room_id(room.get("id", room.get("room_id")))
    if room_id is None:
        return None
    floor_id = _canonical_floor_id(room.get("floor_id"))
    return {
        "id": room_id,
        "floor_id": floor_id,
        "display_floor_id": _clean_optional_text(room.get("display_floor_id")) or floor_id,
        "display_order": _as_int_or_none(room.get("display_order")),
        "room_type": _clean_optional_text(room.get("room_type")) or "unknown",
        "status": _clean_optional_text(room.get("status")) or "confirmed",
    }


def _edge_record_from_payload(edge: Dict[str, Any], public_room_ids: Iterable[str]) -> Optional[Dict[str, Any]]:
    room_ids = set(public_room_ids)
    source = _canonical_room_id(edge.get("source", edge.get("source_room_id")))
    target = _canonical_room_id(edge.get("target", edge.get("target_room_id")))
    if source is None or target is None or source == target:
        return None
    if source not in room_ids or target not in room_ids:
        return None
    ordered = sorted((source, target), key=_room_sort_key)
    relation_type = _clean_optional_text(edge.get("relation_type")) or "unknown"
    return {
        "source": ordered[0],
        "target": ordered[1],
        "relation_type": relation_type,
        "status": _clean_optional_text(edge.get("status")) or "unknown",
    }


def _resolve_runtime_object_room_id(obj: Dict[str, Any]) -> Optional[str]:
    for key in ("room_uuid", "room_id"):
        room_id = _canonical_room_id(obj.get(key))
        if room_id is not None:
            return room_id
    return None


def _membership_record(
    *,
    object_id: Optional[str],
    room_id: Optional[str],
    floor_id: Any = None,
) -> Optional[Dict[str, Any]]:
    if object_id is None or room_id is None:
        return None
    return {
        "object_id": object_id,
        "room_id": room_id,
        "floor_id": _canonical_floor_id(floor_id),
    }


def _semantic_counts(edges: Sequence[Dict[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for edge in edges:
        relation_type = str(edge.get("relation_type") or "unknown")
        counts[relation_type] = counts.get(relation_type, 0) + 1
    return {key: counts[key] for key in sorted(counts)}


def _minimal_public_topology_subset(
    *,
    floors: Sequence[Dict[str, Any]],
    rooms: Sequence[Dict[str, Any]],
    edges: Sequence[Dict[str, Any]],
    object_room_memberships: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    ordered_floors = sorted([dict(item) for item in floors], key=lambda item: str(item.get("floor_id")))
    ordered_rooms = sorted([dict(item) for item in rooms], key=lambda item: _room_sort_key(item.get("id")))
    deduped_edges = sorted(
        {
            (
                str(item.get("source")),
                str(item.get("target")),
                str(item.get("relation_type")),
                str(item.get("status")),
            ): dict(item)
            for item in edges
        }.values(),
        key=lambda item: (_room_sort_key(item.get("source")), _room_sort_key(item.get("target")), str(item.get("relation_type"))),
    )
    ordered_memberships = sorted(
        {
            (str(item.get("object_id")), str(item.get("room_id"))): dict(item)
            for item in object_room_memberships
        }.values(),
        key=lambda item: (_object_sort_key(item.get("object_id")), _room_sort_key(item.get("room_id"))),
    )
    floor_ids = [str(item["floor_id"]) for item in ordered_floors if item.get("floor_id") is not None]
    room_ids = [str(item["id"]) for item in ordered_rooms if item.get("id") is not None]
    object_ids = [str(item["object_id"]) for item in ordered_memberships if item.get("object_id") is not None]
    return {
        "artifact_kind": MINIMAL_PUBLIC_TOPOLOGY_ARTIFACT_KIND,
        "scope": MINIMAL_PUBLIC_TOPOLOGY_SHADOW_SCOPE,
        "authoritative": False,
        "public_topology_meaning": "committed_published_only",
        "counts": {
            "floor_count": int(len(ordered_floors)),
            "room_count": int(len(ordered_rooms)),
            "edge_count": int(len(deduped_edges)),
            "object_room_membership_count": int(len(ordered_memberships)),
        },
        "ids": {
            "floor_ids": floor_ids,
            "room_ids": room_ids,
            "object_ids": object_ids,
        },
        "floors": ordered_floors,
        "rooms": ordered_rooms,
        "edges": deduped_edges,
        "object_room_memberships": ordered_memberships,
        "semantic_summary": {
            "edge_relation_type_counts": _semantic_counts(deduped_edges),
            "room_membership_available": bool(ordered_memberships),
        },
        "deferred": [
            "anchors",
            "scene_graph_relationships",
            "graphml",
            "rich_diagnostics",
            "full_vector_map_details",
            "visual_debug_artifacts",
        ],
    }


def build_minimal_public_topology_subset_from_topology_payload(topology_payload: Dict[str, Any]) -> Dict[str, Any]:
    """Extract the first real-export sidecar target from topology_v0_1.json.

    The subset intentionally stays topology-shaped but small: public floors,
    rooms, undirected edge summaries, and object-to-room memberships. It omits
    anchors and all rich route/evidence/debug fields.
    """

    payload = dict(topology_payload or {})
    floors = [
        record
        for floor in list(payload.get("floors") or [])
        if isinstance(floor, dict)
        for record in [_floor_record_from_payload(floor)]
        if record is not None
    ]
    rooms = [
        record
        for room in list(payload.get("rooms") or [])
        if isinstance(room, dict)
        for record in [_room_record_from_payload(room)]
        if record is not None
    ]
    room_ids = {str(room["id"]) for room in rooms}
    edges = [
        record
        for edge in list(payload.get("edges") or [])
        if isinstance(edge, dict)
        for record in [_edge_record_from_payload(edge, room_ids)]
        if record is not None
    ]

    entities = dict(payload.get("entities") or {})
    object_records = {
        object_id: dict(obj)
        for obj in list(entities.get("objects") or [])
        if isinstance(obj, dict)
        for object_id in [_canonical_object_id(obj.get("id"))]
        if object_id is not None
    }
    memberships: List[Dict[str, Any]] = []
    indexed_object_to_room = dict(dict(payload.get("indices") or {}).get("object_to_room") or {})
    seen_memberships = set()
    for raw_object_id, raw_room_id in indexed_object_to_room.items():
        object_id = _canonical_object_id(raw_object_id)
        room_id = _canonical_room_id(raw_room_id)
        if room_id not in room_ids:
            continue
        record = _membership_record(
            object_id=object_id,
            room_id=room_id,
            floor_id=object_records.get(object_id or "", {}).get("floor_id"),
        )
        if record is None:
            continue
        key = (record["object_id"], record["room_id"])
        seen_memberships.add(key)
        memberships.append(record)
    for object_id, obj in object_records.items():
        room_id = _canonical_room_id(obj.get("room_id"))
        if room_id not in room_ids:
            continue
        key = (object_id, room_id)
        if key in seen_memberships:
            continue
        record = _membership_record(object_id=object_id, room_id=room_id, floor_id=obj.get("floor_id"))
        if record is not None:
            memberships.append(record)

    return _minimal_public_topology_subset(
        floors=floors,
        rooms=rooms,
        edges=edges,
        object_room_memberships=memberships,
    )


def build_minimal_public_topology_subset_from_vector_map(
    vector_map: Dict[str, Any],
    *,
    committed_room_ids: Sequence[str] = (),
) -> Dict[str, Any]:
    """Build the same tiny public subset from runtime vector-map-shaped state.

    When committed room IDs are available, this filters rooms, edges, and object
    memberships to that committed/published set so debug/lifecycle rooms do not
    leak into the public topology surface.
    """

    world = dict(vector_map or {})
    committed_filter = {room_id for room_id in (_canonical_room_id(item) for item in committed_room_ids) if room_id is not None}
    floors = [
        record
        for floor in list(world.get("floors") or [])
        if isinstance(floor, dict)
        for record in [_floor_record_from_payload(floor)]
        if record is not None
    ]
    all_rooms = [
        record
        for room in list(world.get("rooms") or [])
        if isinstance(room, dict)
        for record in [_room_record_from_payload(room)]
        if record is not None
    ]
    rooms = [room for room in all_rooms if not committed_filter or room.get("id") in committed_filter]
    room_ids = {str(room["id"]) for room in rooms}
    edges = [
        record
        for edge in list(world.get("edges") or [])
        if isinstance(edge, dict)
        for record in [_edge_record_from_payload(edge, room_ids)]
        if record is not None
    ]
    memberships = []
    for obj in list(world.get("objects") or []):
        if not isinstance(obj, dict):
            continue
        object_id = _canonical_object_id(obj.get("id"))
        room_id = _resolve_runtime_object_room_id(obj)
        if room_id not in room_ids:
            continue
        record = _membership_record(object_id=object_id, room_id=room_id, floor_id=obj.get("floor_id"))
        if record is not None:
            memberships.append(record)
    return _minimal_public_topology_subset(
        floors=floors,
        rooms=rooms,
        edges=edges,
        object_room_memberships=memberships,
    )


def _extract_public_subset_from_vector_map(vector_map: Dict[str, Any]) -> Dict[str, Any]:
    floors = list(vector_map.get("floors") or [])
    rooms = list(vector_map.get("rooms") or [])
    objects = list(vector_map.get("objects") or [])
    anchors = list(vector_map.get("anchors") or [])
    edges = list(vector_map.get("edges") or [])
    return {
        "floor_count": len(floors),
        "room_count": len(rooms),
        "edge_count": len(edges),
        "object_count": len(objects),
        "anchor_count": len(anchors),
        "floor_ids": _extract_ids(floors, key="floor_id", canonicalizer=_canonical_floor_id),
        "room_ids": _extract_ids(rooms, key="id", canonicalizer=_canonical_room_id),
        "object_ids": _extract_ids(objects, key="id", canonicalizer=_canonical_object_id),
        "anchor_ids": _extract_ids(anchors, key="id", canonicalizer=_canonical_anchor_id),
    }


def _extract_vector_map_from_state(*states: Any) -> Dict[str, Any]:
    for state in states:
        if state is None:
            continue
        if isinstance(state, dict) and any(key in state for key in ("rooms", "objects", "anchors", "floors")):
            return dict(state)
        for attr in ("vector_map", "latest_vector_map", "final_vector_map"):
            value = getattr(state, attr, None)
            if isinstance(value, dict) and any(key in value for key in ("rooms", "objects", "anchors", "floors")):
                return dict(value)
        snapshots = getattr(state, "snapshots", None)
        if snapshots:
            latest_snapshot = list(snapshots)[-1]
            value = getattr(latest_snapshot, "vector_map", None)
            if isinstance(value, dict):
                return dict(value)
    return {}


def _extract_latest_snapshot(*states: Any) -> Any:
    for state in states:
        if state is None:
            continue
        snapshots = getattr(state, "snapshots", None)
        if snapshots:
            return list(snapshots)[-1]
    for state in states:
        if state is not None:
            return state
    return None


def _extract_lifecycle_payload(
    *,
    lifecycle_payload: Optional[Dict[str, Any]],
    diagnostics_bundle: Any,
    stage3_state: Any,
    stage5_state: Any,
) -> Dict[str, Any]:
    if lifecycle_payload is not None:
        return dict(lifecycle_payload)
    for state in (stage5_state, stage3_state):
        payload = getattr(state, "lifecycle_payload", None)
        if isinstance(payload, dict):
            return dict(payload)
    if diagnostics_bundle is not None:
        return copy.deepcopy(diagnostics_bundle.lifecycle_payload or {})
    return {}


def _state_attr(*states: Any, attr: str, default: Any = None) -> Any:
    for state in states:
        if state is None:
            continue
        if hasattr(state, attr):
            value = getattr(state, attr)
            if value is not None:
                return value
    return default


@dataclass(frozen=True)
class RuntimeMaintainedStateSnapshot:
    sequence_id: Optional[str]
    frame_idx: Optional[int]
    timestamp: Optional[float]
    processed_frames: Optional[int]
    snapshot_count: int
    segmentation_cycle_count: int
    final_floor_count: int
    final_room_count: int
    final_object_count: int
    final_anchor_count: int
    latest_vector_map_path: Optional[str]
    final_floor_ids: Tuple[str, ...] = ()
    final_room_ids: Tuple[str, ...] = ()
    final_object_ids: Tuple[str, ...] = ()
    final_anchor_ids: Tuple[str, ...] = ()
    hot_state_notes: Tuple[str, ...] = (
        "Stage3 floor/room segmentation state stays runtime-maintained.",
        "Stage5 object association/fusion state stays runtime-maintained.",
        "Materialized vector maps, topology, anchors, scene graph, and diagnostics are export/query/artifact surfaces.",
    )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sequence_id": self.sequence_id,
            "frame_idx": self.frame_idx,
            "timestamp": self.timestamp,
            "processed_frames": self.processed_frames,
            "snapshot_count": int(self.snapshot_count),
            "segmentation_cycle_count": int(self.segmentation_cycle_count),
            "final_floor_count": int(self.final_floor_count),
            "final_room_count": int(self.final_room_count),
            "final_object_count": int(self.final_object_count),
            "final_anchor_count": int(self.final_anchor_count),
            "final_floor_ids": list(self.final_floor_ids),
            "final_room_ids": list(self.final_room_ids),
            "final_object_ids": list(self.final_object_ids),
            "final_anchor_ids": list(self.final_anchor_ids),
            "latest_vector_map_path": self.latest_vector_map_path,
            "hot_state_notes": list(self.hot_state_notes),
        }


@dataclass(frozen=True)
class CommittedPublicExportSurfaceSnapshot:
    topology_path: str
    manifest_path: Optional[str]
    summary_path: Optional[str]
    world_snapshot_path: Optional[str]
    artifact_profile: Optional[str]
    topology_surface: Optional[str]
    topology_semantics: Optional[str]
    floor_count: int
    room_count: int
    edge_count: int
    object_count: int
    anchor_count: int
    floor_ids: Tuple[str, ...] = ()
    room_ids: Tuple[str, ...] = ()
    object_ids: Tuple[str, ...] = ()
    anchor_ids: Tuple[str, ...] = ()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "surface": "public",
            "topology_path": self.topology_path,
            "manifest_path": self.manifest_path,
            "summary_path": self.summary_path,
            "world_snapshot_path": self.world_snapshot_path,
            "artifact_profile": self.artifact_profile,
            "topology_surface": self.topology_surface,
            "topology_semantics": self.topology_semantics,
            "floor_count": int(self.floor_count),
            "room_count": int(self.room_count),
            "edge_count": int(self.edge_count),
            "object_count": int(self.object_count),
            "anchor_count": int(self.anchor_count),
            "floor_ids": list(self.floor_ids),
            "room_ids": list(self.room_ids),
            "object_ids": list(self.object_ids),
            "anchor_ids": list(self.anchor_ids),
            "public_topology_meaning": "committed_published_only",
        }


@dataclass(frozen=True)
class LifecycleDebugSurfaceSnapshot:
    lifecycle_path: str
    manifest_path: Optional[str]
    summary_path: Optional[str]
    artifact_profile: Optional[str]
    lifecycle_surface: Optional[str]
    lifecycle_semantics: Optional[str]
    committed_room_count: int
    non_published_room_count: int
    publication_state_counts: Dict[str, int]
    committed_room_ids: Tuple[str, ...] = ()
    non_published_room_ids: Tuple[str, ...] = ()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "surface": "lifecycle_debug",
            "lifecycle_path": self.lifecycle_path,
            "manifest_path": self.manifest_path,
            "summary_path": self.summary_path,
            "artifact_profile": self.artifact_profile,
            "lifecycle_surface": self.lifecycle_surface,
            "lifecycle_semantics": self.lifecycle_semantics,
            "committed_room_count": int(self.committed_room_count),
            "non_published_room_count": int(self.non_published_room_count),
            "publication_state_counts": dict(self.publication_state_counts),
            "committed_room_ids": list(self.committed_room_ids),
            "non_published_room_ids": list(self.non_published_room_ids),
            "non_published_rooms_public": False,
        }


@dataclass(frozen=True)
class MinimalPublicTopologySubsetSnapshot:
    payload: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return copy.deepcopy(self.payload)


@dataclass(frozen=True)
class RuntimeSnapshotParityValidationResult:
    passed: bool
    scope: str
    compared_keys: Tuple[str, ...]
    mismatches: Tuple[Dict[str, Any], ...]
    notes: Tuple[str, ...] = (
        "Authoritative export remains the current synchronous manifest-backed path.",
        "Shadow sidecar mismatches are diagnostic and do not change consumer-facing contracts.",
    )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "passed": bool(self.passed),
            "scope": self.scope,
            "compared_keys": list(self.compared_keys),
            "mismatches": [dict(item) for item in self.mismatches],
            "notes": list(self.notes),
        }


@dataclass(frozen=True)
class RuntimeStateSnapshot:
    contract_version: str
    source_scene_root: str
    created_at_utc: str
    runtime_maintained_state: RuntimeMaintainedStateSnapshot
    committed_public_export_surface: CommittedPublicExportSurfaceSnapshot
    lifecycle_debug_surface: LifecycleDebugSurfaceSnapshot
    minimal_public_topology_subset: Optional[MinimalPublicTopologySubsetSnapshot] = None
    deferred_hooks: Tuple[str, ...] = (
        "maintained_candidate_indices",
    )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "contract_version": self.contract_version,
            "source_scene_root": self.source_scene_root,
            "created_at_utc": self.created_at_utc,
            "runtime_maintained_state": self.runtime_maintained_state.to_dict(),
            "committed_public_export_surface": self.committed_public_export_surface.to_dict(),
            "lifecycle_debug_surface": self.lifecycle_debug_surface.to_dict(),
            "minimal_public_topology_subset": None
            if self.minimal_public_topology_subset is None
            else self.minimal_public_topology_subset.to_dict(),
            "deferred_hooks": list(self.deferred_hooks),
        }


def _publication_state_counts(lifecycle_payload: Dict[str, Any]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for room in list(lifecycle_payload.get("rooms") or []):
        if not isinstance(room, dict):
            continue
        state = str(room.get("publication_state") or ("PUBLISHED" if room.get("published") else "UNKNOWN"))
        counts[state] = counts.get(state, 0) + 1
    return counts


def _lifecycle_room_id(room: Dict[str, Any]) -> Optional[str]:
    return _canonical_room_id(room.get("room_id", room.get("id")))


def _lifecycle_committed_room_ids(lifecycle_payload: Dict[str, Any]) -> Tuple[str, ...]:
    explicit = lifecycle_payload.get("committed_rooms")
    if isinstance(explicit, list):
        explicit_ids = _stable_sorted(_canonical_room_id(item) for item in explicit)
        if explicit_ids:
            return explicit_ids
    values: List[Optional[str]] = []
    for room in list(lifecycle_payload.get("rooms") or []):
        if not isinstance(room, dict):
            continue
        if str(room.get("publication_state") or "") == "PUBLISHED" or bool(room.get("published")):
            values.append(_lifecycle_room_id(room))
    return _stable_sorted(values)


def _lifecycle_non_published_room_ids(lifecycle_payload: Dict[str, Any]) -> Tuple[str, ...]:
    values: List[Optional[str]] = []
    for room in list(lifecycle_payload.get("rooms") or []):
        if not isinstance(room, dict):
            continue
        state = str(room.get("publication_state") or ("PUBLISHED" if room.get("published") else "UNKNOWN"))
        if state != "PUBLISHED":
            values.append(_lifecycle_room_id(room))
    return _stable_sorted(values)


def build_runtime_snapshot_from_bundles(
    *,
    committed_bundle: Any,
    diagnostics_bundle: Any,
    created_at_utc: str,
) -> RuntimeStateSnapshot:
    summary_payload = copy.deepcopy(committed_bundle.summary_payload or diagnostics_bundle.summary_payload or {})
    manifest_payload = copy.deepcopy(committed_bundle.manifest_payload or diagnostics_bundle.manifest_payload or {})
    topology_payload = copy.deepcopy(committed_bundle.topology_payload or {})
    lifecycle_payload = copy.deepcopy(diagnostics_bundle.lifecycle_payload or {})
    public_subset = _extract_public_subset_from_topology(topology_payload)
    minimal_topology_subset = build_minimal_public_topology_subset_from_topology_payload(topology_payload)
    lifecycle_summary = dict(lifecycle_payload.get("summary") or {})
    publication_counts = _publication_state_counts(lifecycle_payload)
    non_published_count = sum(
        count
        for state, count in publication_counts.items()
        if str(state) != "PUBLISHED"
    )

    runtime_state = RuntimeMaintainedStateSnapshot(
        sequence_id=summary_payload.get("sequence_id") or manifest_payload.get("sequence_name") or committed_bundle.sequence_name,
        frame_idx=None if lifecycle_payload.get("frame_idx") is None else _as_int(lifecycle_payload.get("frame_idx")),
        timestamp=_as_float_or_none(lifecycle_payload.get("timestamp")),
        processed_frames=None if summary_payload.get("processed_frames") is None else _as_int(summary_payload.get("processed_frames")),
        snapshot_count=_as_int(summary_payload.get("snapshot_count")),
        segmentation_cycle_count=_as_int(summary_payload.get("segmentation_cycle_count")),
        final_floor_count=public_subset["floor_count"],
        final_room_count=_as_int(summary_payload.get("final_room_count"), default=public_subset["room_count"]),
        final_object_count=_as_int(summary_payload.get("final_object_count"), default=public_subset["object_count"]),
        final_anchor_count=_as_int(summary_payload.get("final_anchor_count"), default=public_subset["anchor_count"]),
        latest_vector_map_path=summary_payload.get("final_vector_map_path"),
        final_floor_ids=public_subset["floor_ids"],
        final_room_ids=public_subset["room_ids"],
        final_object_ids=public_subset["object_ids"],
        final_anchor_ids=public_subset["anchor_ids"],
    )
    public_surface = CommittedPublicExportSurfaceSnapshot(
        topology_path=str(committed_bundle.topology_path),
        manifest_path=_optional_path_text(committed_bundle.manifest_path),
        summary_path=_optional_path_text(committed_bundle.summary_path),
        world_snapshot_path=_optional_path_text(committed_bundle.world_snapshot_path),
        artifact_profile=committed_bundle.artifact_profile,
        topology_surface=None if committed_bundle.topology_record is None else committed_bundle.topology_record.get("artifact_surface"),
        topology_semantics=None if committed_bundle.topology_record is None else committed_bundle.topology_record.get("artifact_semantics"),
        floor_count=public_subset["floor_count"],
        room_count=public_subset["room_count"],
        edge_count=public_subset["edge_count"],
        object_count=public_subset["object_count"],
        anchor_count=public_subset["anchor_count"],
        floor_ids=public_subset["floor_ids"],
        room_ids=public_subset["room_ids"],
        object_ids=public_subset["object_ids"],
        anchor_ids=public_subset["anchor_ids"],
    )
    lifecycle_surface = LifecycleDebugSurfaceSnapshot(
        lifecycle_path=str(diagnostics_bundle.lifecycle_path),
        manifest_path=_optional_path_text(diagnostics_bundle.manifest_path),
        summary_path=_optional_path_text(diagnostics_bundle.summary_path),
        artifact_profile=diagnostics_bundle.artifact_profile,
        lifecycle_surface=None if diagnostics_bundle.lifecycle_record is None else diagnostics_bundle.lifecycle_record.get("artifact_surface"),
        lifecycle_semantics=None if diagnostics_bundle.lifecycle_record is None else diagnostics_bundle.lifecycle_record.get("artifact_semantics"),
        committed_room_count=_as_int(lifecycle_summary.get("committed_room_count")),
        non_published_room_count=int(non_published_count),
        publication_state_counts=publication_counts,
        committed_room_ids=_lifecycle_committed_room_ids(lifecycle_payload),
        non_published_room_ids=_lifecycle_non_published_room_ids(lifecycle_payload),
    )
    return RuntimeStateSnapshot(
        contract_version=RUNTIME_STATE_SNAPSHOT_CONTRACT_VERSION,
        source_scene_root=str(committed_bundle.scene_root),
        created_at_utc=str(created_at_utc),
        runtime_maintained_state=runtime_state,
        committed_public_export_surface=public_surface,
        lifecycle_debug_surface=lifecycle_surface,
        minimal_public_topology_subset=MinimalPublicTopologySubsetSnapshot(minimal_topology_subset),
    )


def build_runtime_snapshot_from_live_stage_state(
    *,
    stage3_state: Any,
    stage5_state: Any,
    committed_bundle: Any,
    diagnostics_bundle: Any,
    created_at_utc: str,
    lifecycle_payload: Optional[Dict[str, Any]] = None,
) -> RuntimeStateSnapshot:
    """Build the minimal shadow-parity snapshot directly from hot runtime state.

    This intentionally reads only stable IDs, counts, frame metadata, and lifecycle
    publication states. It does not invoke vector-map/topology/anchor/scene-graph
    exporters and does not make the sidecar authoritative.
    """

    latest_snapshot = _extract_latest_snapshot(stage5_state, stage3_state)
    vector_map = _extract_vector_map_from_state(stage5_state, stage3_state)
    lifecycle = _extract_lifecycle_payload(
        lifecycle_payload=lifecycle_payload,
        diagnostics_bundle=diagnostics_bundle,
        stage3_state=stage3_state,
        stage5_state=stage5_state,
    )
    lifecycle_summary = dict(lifecycle.get("summary") or {})
    publication_counts = _publication_state_counts(lifecycle)
    non_published_ids = _lifecycle_non_published_room_ids(lifecycle)
    committed_ids = _lifecycle_committed_room_ids(lifecycle)
    runtime_subset = _extract_public_subset_from_vector_map(vector_map)
    minimal_topology_subset = build_minimal_public_topology_subset_from_vector_map(
        vector_map,
        committed_room_ids=committed_ids,
    )
    minimal_counts = dict(minimal_topology_subset.get("counts") or {})
    minimal_ids = dict(minimal_topology_subset.get("ids") or {})
    public_subset = {
        "floor_count": int(minimal_counts.get("floor_count", 0)),
        "room_count": int(minimal_counts.get("room_count", 0)),
        "edge_count": int(minimal_counts.get("edge_count", 0)),
        "object_count": int(minimal_counts.get("object_room_membership_count", 0)),
        "anchor_count": runtime_subset["anchor_count"],
        "floor_ids": tuple(minimal_ids.get("floor_ids") or ()),
        "room_ids": tuple(minimal_ids.get("room_ids") or ()),
        "object_ids": tuple(minimal_ids.get("object_ids") or ()),
        "anchor_ids": runtime_subset["anchor_ids"],
    }
    snapshots = list(getattr(stage3_state, "snapshots", []) or [])
    segmentation_cycle_count = _state_attr(stage5_state, stage3_state, attr="segmentation_cycle_count")
    if segmentation_cycle_count is None and snapshots:
        segmentation_cycle_count = max(int(getattr(snapshot, "segmentation_cycle_idx", 0)) for snapshot in snapshots)
    sequence_id = (
        _clean_optional_text(_state_attr(stage5_state, stage3_state, attr="sequence_id"))
        or _clean_optional_text(diagnostics_bundle.sequence_name)
        or _clean_optional_text(committed_bundle.sequence_name)
    )
    frame_idx = _state_attr(latest_snapshot, stage5_state, stage3_state, attr="frame_idx")
    timestamp = _state_attr(latest_snapshot, stage5_state, stage3_state, attr="timestamp")
    latest_vector_map_path = (
        _clean_optional_text(_state_attr(latest_snapshot, stage5_state, stage3_state, attr="vector_map_path"))
        or _clean_optional_text(_state_attr(stage5_state, stage3_state, attr="latest_vector_map_path"))
    )

    runtime_state = RuntimeMaintainedStateSnapshot(
        sequence_id=sequence_id,
        frame_idx=None if frame_idx is None else _as_int(frame_idx),
        timestamp=_as_float_or_none(timestamp),
        processed_frames=_as_int(_state_attr(stage5_state, stage3_state, attr="processed_frames"), default=0)
        if _state_attr(stage5_state, stage3_state, attr="processed_frames") is not None
        else None,
        snapshot_count=len(snapshots) if snapshots else _as_int(_state_attr(stage5_state, stage3_state, attr="snapshot_count")),
        segmentation_cycle_count=_as_int(segmentation_cycle_count),
        final_floor_count=runtime_subset["floor_count"],
        final_room_count=runtime_subset["room_count"],
        final_object_count=runtime_subset["object_count"],
        final_anchor_count=runtime_subset["anchor_count"],
        latest_vector_map_path=latest_vector_map_path,
        final_floor_ids=runtime_subset["floor_ids"],
        final_room_ids=runtime_subset["room_ids"],
        final_object_ids=runtime_subset["object_ids"],
        final_anchor_ids=runtime_subset["anchor_ids"],
    )
    public_surface = CommittedPublicExportSurfaceSnapshot(
        topology_path=str(committed_bundle.topology_path),
        manifest_path=_optional_path_text(committed_bundle.manifest_path),
        summary_path=_optional_path_text(committed_bundle.summary_path),
        world_snapshot_path=_optional_path_text(committed_bundle.world_snapshot_path),
        artifact_profile=committed_bundle.artifact_profile,
        topology_surface=None if committed_bundle.topology_record is None else committed_bundle.topology_record.get("artifact_surface"),
        topology_semantics=None if committed_bundle.topology_record is None else committed_bundle.topology_record.get("artifact_semantics"),
        floor_count=public_subset["floor_count"],
        room_count=public_subset["room_count"],
        edge_count=public_subset["edge_count"],
        object_count=public_subset["object_count"],
        anchor_count=public_subset["anchor_count"],
        floor_ids=public_subset["floor_ids"],
        room_ids=public_subset["room_ids"],
        object_ids=public_subset["object_ids"],
        anchor_ids=public_subset["anchor_ids"],
    )
    lifecycle_surface = LifecycleDebugSurfaceSnapshot(
        lifecycle_path=str(diagnostics_bundle.lifecycle_path),
        manifest_path=_optional_path_text(diagnostics_bundle.manifest_path),
        summary_path=_optional_path_text(diagnostics_bundle.summary_path),
        artifact_profile=diagnostics_bundle.artifact_profile,
        lifecycle_surface=None if diagnostics_bundle.lifecycle_record is None else diagnostics_bundle.lifecycle_record.get("artifact_surface"),
        lifecycle_semantics=None if diagnostics_bundle.lifecycle_record is None else diagnostics_bundle.lifecycle_record.get("artifact_semantics"),
        committed_room_count=_as_int(lifecycle_summary.get("committed_room_count"), default=len(committed_ids)),
        non_published_room_count=len(non_published_ids),
        publication_state_counts=publication_counts,
        committed_room_ids=committed_ids,
        non_published_room_ids=non_published_ids,
    )
    return RuntimeStateSnapshot(
        contract_version=RUNTIME_STATE_SNAPSHOT_CONTRACT_VERSION,
        source_scene_root=str(committed_bundle.scene_root),
        created_at_utc=str(created_at_utc),
        runtime_maintained_state=runtime_state,
        committed_public_export_surface=public_surface,
        lifecycle_debug_surface=lifecycle_surface,
        minimal_public_topology_subset=MinimalPublicTopologySubsetSnapshot(minimal_topology_subset),
    )


def _shadow_parity_subset(snapshot_payload: Dict[str, Any]) -> Dict[str, Any]:
    runtime = dict(snapshot_payload.get("runtime_maintained_state") or {})
    public = dict(snapshot_payload.get("committed_public_export_surface") or {})
    lifecycle = dict(snapshot_payload.get("lifecycle_debug_surface") or {})
    return {
        "schema.contract_version": snapshot_payload.get("contract_version"),
        "runtime.sequence_id": runtime.get("sequence_id"),
        "runtime.frame_idx": runtime.get("frame_idx"),
        "runtime.snapshot_count": runtime.get("snapshot_count"),
        "runtime.segmentation_cycle_count": runtime.get("segmentation_cycle_count"),
        "runtime.final_floor_count": runtime.get("final_floor_count"),
        "runtime.final_room_count": runtime.get("final_room_count"),
        "runtime.final_object_count": runtime.get("final_object_count"),
        "runtime.final_anchor_count": runtime.get("final_anchor_count"),
        "runtime.final_floor_ids": list(runtime.get("final_floor_ids") or []),
        "runtime.final_room_ids": list(runtime.get("final_room_ids") or []),
        "runtime.final_object_ids": list(runtime.get("final_object_ids") or []),
        "runtime.final_anchor_ids": list(runtime.get("final_anchor_ids") or []),
        "public.topology_semantics": public.get("topology_semantics"),
        "public.public_topology_meaning": public.get("public_topology_meaning"),
        "public.floor_count": public.get("floor_count"),
        "public.room_count": public.get("room_count"),
        "public.edge_count": public.get("edge_count"),
        "public.object_count": public.get("object_count"),
        "public.anchor_count": public.get("anchor_count"),
        "public.floor_ids": list(public.get("floor_ids") or []),
        "public.room_ids": list(public.get("room_ids") or []),
        "public.object_ids": list(public.get("object_ids") or []),
        "public.anchor_ids": list(public.get("anchor_ids") or []),
        "lifecycle.committed_room_count": lifecycle.get("committed_room_count"),
        "lifecycle.non_published_room_count": lifecycle.get("non_published_room_count"),
        "lifecycle.publication_state_counts": dict(lifecycle.get("publication_state_counts") or {}),
        "lifecycle.committed_room_ids": list(lifecycle.get("committed_room_ids") or []),
        "lifecycle.non_published_room_ids": list(lifecycle.get("non_published_room_ids") or []),
        "lifecycle.non_published_rooms_public": lifecycle.get("non_published_rooms_public"),
    }


def build_runtime_snapshot_shadow_parity_subset(snapshot: RuntimeStateSnapshot) -> Dict[str, Any]:
    return _shadow_parity_subset(snapshot.to_dict())


def _subset_from_sidecar_payload(sidecar_payload: Dict[str, Any]) -> Dict[str, Any]:
    materialized = dict(sidecar_payload.get("shadow_materialization") or {})
    runtime = dict(materialized.get("runtime_state") or {})
    public = dict(materialized.get("public_topology") or {})
    lifecycle = dict(materialized.get("lifecycle_debug") or {})
    return {
        "schema.contract_version": materialized.get("snapshot_contract_version"),
        "runtime.sequence_id": runtime.get("sequence_id"),
        "runtime.frame_idx": runtime.get("frame_idx"),
        "runtime.snapshot_count": runtime.get("snapshot_count"),
        "runtime.segmentation_cycle_count": runtime.get("segmentation_cycle_count"),
        "runtime.final_floor_count": runtime.get("final_floor_count"),
        "runtime.final_room_count": runtime.get("final_room_count"),
        "runtime.final_object_count": runtime.get("final_object_count"),
        "runtime.final_anchor_count": runtime.get("final_anchor_count"),
        "runtime.final_floor_ids": list(runtime.get("final_floor_ids") or []),
        "runtime.final_room_ids": list(runtime.get("final_room_ids") or []),
        "runtime.final_object_ids": list(runtime.get("final_object_ids") or []),
        "runtime.final_anchor_ids": list(runtime.get("final_anchor_ids") or []),
        "public.topology_semantics": public.get("topology_semantics"),
        "public.public_topology_meaning": public.get("public_topology_meaning"),
        "public.floor_count": public.get("floor_count"),
        "public.room_count": public.get("room_count"),
        "public.edge_count": public.get("edge_count"),
        "public.object_count": public.get("object_count"),
        "public.anchor_count": public.get("anchor_count"),
        "public.floor_ids": list(public.get("floor_ids") or []),
        "public.room_ids": list(public.get("room_ids") or []),
        "public.object_ids": list(public.get("object_ids") or []),
        "public.anchor_ids": list(public.get("anchor_ids") or []),
        "lifecycle.committed_room_count": lifecycle.get("committed_room_count"),
        "lifecycle.non_published_room_count": lifecycle.get("non_published_room_count"),
        "lifecycle.publication_state_counts": dict(lifecycle.get("publication_state_counts") or {}),
        "lifecycle.committed_room_ids": list(lifecycle.get("committed_room_ids") or []),
        "lifecycle.non_published_room_ids": list(lifecycle.get("non_published_room_ids") or []),
        "lifecycle.non_published_rooms_public": lifecycle.get("non_published_rooms_public"),
    }


def _minimal_public_topology_subset_from_snapshot_payload(snapshot_payload: Dict[str, Any]) -> Dict[str, Any]:
    subset = snapshot_payload.get("minimal_public_topology_subset")
    return copy.deepcopy(subset) if isinstance(subset, dict) else {}


def _minimal_public_topology_subset_from_sidecar_payload(sidecar_payload: Dict[str, Any]) -> Dict[str, Any]:
    materialized = dict(sidecar_payload.get("shadow_materialization") or {})
    subset = materialized.get("minimal_public_topology_subset")
    return copy.deepcopy(subset) if isinstance(subset, dict) else {}


def _flatten_minimal_public_topology_subset(subset: Dict[str, Any]) -> Dict[str, Any]:
    payload = dict(subset or {})
    counts = dict(payload.get("counts") or {})
    ids = dict(payload.get("ids") or {})
    semantic_summary = dict(payload.get("semantic_summary") or {})
    return {
        "schema.artifact_kind": payload.get("artifact_kind"),
        "schema.scope": payload.get("scope"),
        "schema.public_topology_meaning": payload.get("public_topology_meaning"),
        "counts.floor_count": counts.get("floor_count"),
        "counts.room_count": counts.get("room_count"),
        "counts.edge_count": counts.get("edge_count"),
        "counts.object_room_membership_count": counts.get("object_room_membership_count"),
        "ids.floor_ids": list(ids.get("floor_ids") or []),
        "ids.room_ids": list(ids.get("room_ids") or []),
        "ids.object_ids": list(ids.get("object_ids") or []),
        "records.floors": list(payload.get("floors") or []),
        "records.rooms": list(payload.get("rooms") or []),
        "records.edges": list(payload.get("edges") or []),
        "records.object_room_memberships": list(payload.get("object_room_memberships") or []),
        "semantic.edge_relation_type_counts": dict(semantic_summary.get("edge_relation_type_counts") or {}),
        "semantic.room_membership_available": bool(semantic_summary.get("room_membership_available")),
    }


def compare_minimal_public_topology_shadow_parity(
    *,
    sidecar_payload: Dict[str, Any],
    authoritative_topology_payload: Optional[Dict[str, Any]] = None,
    authoritative_snapshot: Optional[RuntimeStateSnapshot] = None,
    max_mismatches: int = 32,
) -> RuntimeSnapshotParityValidationResult:
    """Compare the sidecar-built minimal topology subset with the current exporter.

    This is intentionally diagnostic-only. It validates that the sidecar has the
    expected tiny schema and that stable IDs, counts, edge summaries, and
    object-room memberships agree with `topology_v0_1.json` or an authoritative
    snapshot derived from that export.
    """

    if authoritative_topology_payload is None and authoritative_snapshot is None:
        raise ValueError("Provide authoritative_topology_payload or authoritative_snapshot.")
    if authoritative_topology_payload is not None:
        authoritative_subset = build_minimal_public_topology_subset_from_topology_payload(
            dict(authoritative_topology_payload or {})
        )
    else:
        authoritative_subset = _minimal_public_topology_subset_from_snapshot_payload(
            authoritative_snapshot.to_dict() if authoritative_snapshot is not None else {}
        )
    shadow_subset = _minimal_public_topology_subset_from_sidecar_payload(dict(sidecar_payload or {}))

    compared_keys: List[str] = []
    mismatches: List[Dict[str, Any]] = []
    for required_key in MINIMAL_PUBLIC_TOPOLOGY_REQUIRED_KEYS:
        compared_key = f"schema.required_key:{required_key}"
        compared_keys.append(compared_key)
        if required_key not in shadow_subset:
            mismatches.append(
                {
                    "key": compared_key,
                    "authoritative": "present",
                    "shadow": "missing",
                    "diagnostic_only": True,
                }
            )
            if len(mismatches) >= int(max_mismatches):
                return RuntimeSnapshotParityValidationResult(
                    passed=False,
                    scope=MINIMAL_PUBLIC_TOPOLOGY_SHADOW_SCOPE,
                    compared_keys=tuple(compared_keys),
                    mismatches=tuple(mismatches),
                )

    authoritative_flat = _flatten_minimal_public_topology_subset(authoritative_subset)
    shadow_flat = _flatten_minimal_public_topology_subset(shadow_subset)
    for key in authoritative_flat:
        compared_keys.append(key)
        authoritative_value = authoritative_flat.get(key)
        shadow_value = shadow_flat.get(key)
        if authoritative_value != shadow_value:
            mismatches.append(
                {
                    "key": key,
                    "authoritative": authoritative_value,
                    "shadow": shadow_value,
                    "diagnostic_only": True,
                }
            )
            if len(mismatches) >= int(max_mismatches):
                break
    return RuntimeSnapshotParityValidationResult(
        passed=not mismatches,
        scope=MINIMAL_PUBLIC_TOPOLOGY_SHADOW_SCOPE,
        compared_keys=tuple(compared_keys),
        mismatches=tuple(mismatches),
    )


def compare_runtime_snapshot_shadow_parity(
    *,
    authoritative_snapshot: RuntimeStateSnapshot,
    shadow_snapshot: Optional[RuntimeStateSnapshot] = None,
    sidecar_payload: Optional[Dict[str, Any]] = None,
    max_mismatches: int = 32,
) -> RuntimeSnapshotParityValidationResult:
    if shadow_snapshot is None and sidecar_payload is None:
        raise ValueError("Provide either shadow_snapshot or sidecar_payload for parity comparison.")
    authoritative_subset = build_runtime_snapshot_shadow_parity_subset(authoritative_snapshot)
    shadow_subset = (
        build_runtime_snapshot_shadow_parity_subset(shadow_snapshot)
        if shadow_snapshot is not None
        else _subset_from_sidecar_payload(dict(sidecar_payload or {}))
    )
    compared_keys = tuple(key for key in authoritative_subset if key in shadow_subset)
    mismatches: List[Dict[str, Any]] = []
    for key in compared_keys:
        authoritative_value = authoritative_subset.get(key)
        shadow_value = shadow_subset.get(key)
        if authoritative_value != shadow_value:
            mismatches.append(
                {
                    "key": key,
                    "authoritative": authoritative_value,
                    "shadow": shadow_value,
                    "diagnostic_only": True,
                }
            )
            if len(mismatches) >= int(max_mismatches):
                break
    return RuntimeSnapshotParityValidationResult(
        passed=not mismatches,
        scope=RUNTIME_SNAPSHOT_SHADOW_PARITY_SCOPE,
        compared_keys=compared_keys,
        mismatches=tuple(mismatches),
    )
