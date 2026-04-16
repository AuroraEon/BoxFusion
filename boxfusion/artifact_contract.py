from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence


TIMELINE_SCHEMA_NAME = "timeline_v1"
TIMELINE_STORAGE_FORMAT = "json_array_rows"
TIMELINE_ROW_KIND_SNAPSHOT = "snapshot_frame"
TIMELINE_ROW_KIND_REPLAY = "replay_frame"
LEGACY_TIMELINE_ROW_TYPE_TO_KIND = {
    "snapshot": TIMELINE_ROW_KIND_SNAPSHOT,
    "replay_frame": TIMELINE_ROW_KIND_REPLAY,
}

ARTIFACT_PROFILE_CORE_ONLY = "core_only"
ARTIFACT_PROFILE_FULL = "full_artifact"

ARTIFACT_SURFACE_PUBLIC = "public"
ARTIFACT_SURFACE_WORKING = "working"
ARTIFACT_SURFACE_LIFECYCLE = "lifecycle"
ARTIFACT_SURFACE_DIAGNOSTIC = "diagnostic"

CAPABILITY_FINAL_STATE_QUERY = "final_state_query"
CAPABILITY_FINAL_STATE_ROUTE = "final_state_route"
CAPABILITY_FINAL_STATE_EVAL = "final_state_eval"
CAPABILITY_QUERY_BUNDLE_REBUILD = "query_bundle_rebuild"
CAPABILITY_CHECKPOINT_REPLAY = "checkpoint_replay"
CAPABILITY_DENSE_REPLAY = "dense_replay"
CAPABILITY_LIFECYCLE_HISTORY = "lifecycle_history"
CAPABILITY_WORKING_TOPOLOGY_HISTORY = "working_topology_history"


ARTIFACT_PROFILE_DECLARATIONS: Dict[str, Dict[str, Any]] = {
    ARTIFACT_PROFILE_CORE_ONLY: {
        "profile_name": ARTIFACT_PROFILE_CORE_ONLY,
        "description": "Committed final-state export profile with diagnostic lifecycle/working artifacts but no replay guarantee.",
        "timeline_policy": {
            "timeline_required": False,
            "checkpoint_replay_optional": False,
            "dense_replay_optional": False,
            "canonical_schema": TIMELINE_SCHEMA_NAME,
        },
        "guaranteed_capabilities": {
            CAPABILITY_FINAL_STATE_QUERY: True,
            CAPABILITY_FINAL_STATE_ROUTE: True,
            CAPABILITY_FINAL_STATE_EVAL: True,
            CAPABILITY_QUERY_BUNDLE_REBUILD: True,
            CAPABILITY_CHECKPOINT_REPLAY: False,
            CAPABILITY_DENSE_REPLAY: False,
            CAPABILITY_LIFECYCLE_HISTORY: True,
            CAPABILITY_WORKING_TOPOLOGY_HISTORY: True,
        },
        "notes": [
            "Core-only guarantees final committed topology and final-state query/route/eval support.",
            "Query-bundle rebuild is scoped to rebuilding final query/index artifacts from final exports.",
            "Core-only does not imply timeline replay support.",
        ],
    },
    ARTIFACT_PROFILE_FULL: {
        "profile_name": ARTIFACT_PROFILE_FULL,
        "description": "Committed final-state export profile plus a public timeline surface for checkpoint replay and downstream playback.",
        "timeline_policy": {
            "timeline_required": True,
            "checkpoint_replay_optional": True,
            "dense_replay_optional": True,
            "canonical_schema": TIMELINE_SCHEMA_NAME,
        },
        "guaranteed_capabilities": {
            CAPABILITY_FINAL_STATE_QUERY: True,
            CAPABILITY_FINAL_STATE_ROUTE: True,
            CAPABILITY_FINAL_STATE_EVAL: True,
            CAPABILITY_QUERY_BUNDLE_REBUILD: True,
            CAPABILITY_CHECKPOINT_REPLAY: True,
            CAPABILITY_DENSE_REPLAY: False,
            CAPABILITY_LIFECYCLE_HISTORY: True,
            CAPABILITY_WORKING_TOPOLOGY_HISTORY: True,
        },
        "notes": [
            "Full artifacts must export a public timeline using the canonical timeline_v1 row schema.",
            "The timeline may be snapshot-only today; dense replay remains optional and is advertised by capability flags.",
        ],
    },
}


ARTIFACT_KEY_METADATA: Dict[str, Dict[str, Any]] = {
    "manifest_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "artifact_manifest",
    },
    "scene_summary_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "scene_summary",
    },
    "topology_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "committed_topology",
    },
    "topology_query_report_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "final_state_query_report",
    },
    "vertical_transition_evidence_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "committed_transition_evidence",
    },
    "timeline_json": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "committed_timeline",
    },
    "timeline_csv": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "committed_timeline_tabular",
    },
    "topology_graphml": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "committed_topology_graphml",
    },
    "final_bev_png": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "public_render",
    },
    "final_split_png": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "public_render",
    },
    "final_demo_mp4": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "public_render",
    },
    "scene_report_md": {
        "surface": ARTIFACT_SURFACE_PUBLIC,
        "semantics": "public_report",
    },
    "online_topology_lifecycle_json": {
        "surface": ARTIFACT_SURFACE_LIFECYCLE,
        "semantics": "diagnostic_history",
    },
    "working_topology_json": {
        "surface": ARTIFACT_SURFACE_WORKING,
        "semantics": "debug_topology_snapshot",
    },
    "working_vs_committed_topology_report_json": {
        "surface": ARTIFACT_SURFACE_WORKING,
        "semantics": "debug_topology_comparison",
    },
    "working_vs_committed_topology_timeline_json": {
        "surface": ARTIFACT_SURFACE_WORKING,
        "semantics": "debug_topology_history",
    },
    "working_vs_committed_topology_timeline_md": {
        "surface": ARTIFACT_SURFACE_WORKING,
        "semantics": "debug_topology_history_report",
    },
    "floor_diagnostics_summary_json": {
        "surface": ARTIFACT_SURFACE_DIAGNOSTIC,
        "semantics": "diagnostic_summary",
    },
    "room_segmentation_diagnostics_json": {
        "surface": ARTIFACT_SURFACE_DIAGNOSTIC,
        "semantics": "diagnostic_summary",
    },
    "runtime_growth_profile_csv": {
        "surface": ARTIFACT_SURFACE_DIAGNOSTIC,
        "semantics": "runtime_profile",
    },
    "runtime_growth_profile_json": {
        "surface": ARTIFACT_SURFACE_DIAGNOSTIC,
        "semantics": "runtime_profile",
    },
}


def declared_artifact_profiles() -> Dict[str, Dict[str, Any]]:
    return copy.deepcopy(ARTIFACT_PROFILE_DECLARATIONS)


def active_artifact_profile(*, core_only: bool) -> str:
    return ARTIFACT_PROFILE_CORE_ONLY if bool(core_only) else ARTIFACT_PROFILE_FULL


def artifact_surface_for_key(key: str) -> str:
    return str(ARTIFACT_KEY_METADATA.get(str(key), {}).get("surface") or ARTIFACT_SURFACE_PUBLIC)


def artifact_semantics_for_key(key: str) -> str:
    return str(ARTIFACT_KEY_METADATA.get(str(key), {}).get("semantics") or "unspecified")


def surface_flags(surface: str) -> Dict[str, Any]:
    normalized = str(surface)
    return {
        "artifact_surface": normalized,
        "public_default": normalized == ARTIFACT_SURFACE_PUBLIC,
        "debug_only": normalized != ARTIFACT_SURFACE_PUBLIC,
        "non_public": normalized != ARTIFACT_SURFACE_PUBLIC,
    }


def timeline_row_kind_to_legacy_row_type(row_kind: str) -> str:
    if row_kind == TIMELINE_ROW_KIND_SNAPSHOT:
        return "snapshot"
    return "replay_frame"


def canonical_timeline_row_kind(row: Dict[str, Any]) -> Optional[str]:
    row_kind = str(dict(row or {}).get("row_kind") or "").strip()
    if row_kind in {TIMELINE_ROW_KIND_SNAPSHOT, TIMELINE_ROW_KIND_REPLAY}:
        return row_kind
    row_type = str(dict(row or {}).get("row_type") or "").strip()
    if row_type:
        return LEGACY_TIMELINE_ROW_TYPE_TO_KIND.get(row_type)
    if dict(row or {}).get("frame_idx") is not None:
        return TIMELINE_ROW_KIND_REPLAY
    return None


def is_timeline_frame_row(row: Dict[str, Any]) -> bool:
    return canonical_timeline_row_kind(row) is not None


def with_timeline_row_contract(
    row: Dict[str, Any],
    *,
    row_kind: str,
    replay_mode: str,
) -> Dict[str, Any]:
    payload = dict(row)
    payload["timeline_schema"] = TIMELINE_SCHEMA_NAME
    payload["timeline_storage"] = TIMELINE_STORAGE_FORMAT
    payload["row_kind"] = str(row_kind)
    payload["row_type"] = str(payload.get("row_type") or timeline_row_kind_to_legacy_row_type(row_kind))
    payload["replay_mode"] = str(replay_mode)
    payload["checkpoint_replay_compatible"] = True
    payload["dense_replay_compatible"] = row_kind == TIMELINE_ROW_KIND_REPLAY
    payload["artifact_surface"] = ARTIFACT_SURFACE_PUBLIC
    return payload


def _sorted_unique_strings(values: Iterable[Any]) -> List[str]:
    seen = set()
    result: List[str] = []
    for value in values:
        token = str(value)
        if not token or token in seen:
            continue
        seen.add(token)
        result.append(token)
    return result


def summarize_timeline_rows(
    rows: Sequence[Dict[str, Any]],
    *,
    timeline_path: Optional[Path] = None,
) -> Dict[str, Any]:
    frame_rows = [dict(row) for row in list(rows or []) if is_timeline_frame_row(dict(row))]
    row_kinds = [canonical_timeline_row_kind(row) for row in frame_rows]
    snapshot_frame_count = int(sum(1 for kind in row_kinds if kind == TIMELINE_ROW_KIND_SNAPSHOT))
    dense_replay_frame_count = int(sum(1 for kind in row_kinds if kind == TIMELINE_ROW_KIND_REPLAY))
    replay_modes = _sorted_unique_strings(
        row.get("replay_mode")
        for row in frame_rows
        if row.get("replay_mode") not in (None, "")
    )
    replay_mode = replay_modes[0] if replay_modes else ("snapshot_only" if frame_rows else None)
    return {
        "schema_name": TIMELINE_SCHEMA_NAME,
        "storage_format": TIMELINE_STORAGE_FORMAT,
        "timeline_path": None if timeline_path is None else str(timeline_path),
        "row_kind_field": "row_kind",
        "legacy_row_type_field": "row_type",
        "row_kinds_present": _sorted_unique_strings(kind for kind in row_kinds if kind is not None),
        "timeline_frame_count": int(len(frame_rows)),
        "snapshot_frame_count": snapshot_frame_count,
        "dense_replay_frame_count": dense_replay_frame_count,
        "replay_mode": replay_mode,
        "checkpoint_replay_supported": bool(frame_rows),
        "dense_replay_supported": bool(dense_replay_frame_count > 0),
    }


def load_timeline_rows(path: Optional[Path]) -> List[Dict[str, Any]]:
    if path is None or not Path(path).exists():
        return []
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if isinstance(payload, list):
        return [dict(row) for row in payload if isinstance(row, dict)]
    return []


def build_artifact_capabilities(
    *,
    topology_json_exists: bool,
    topology_query_report_exists: bool,
    timeline_summary: Dict[str, Any],
    online_topology_lifecycle_exists: bool,
    working_vs_committed_timeline_exists: bool,
) -> Dict[str, bool]:
    return {
        CAPABILITY_FINAL_STATE_QUERY: bool(topology_json_exists),
        CAPABILITY_FINAL_STATE_ROUTE: bool(topology_json_exists),
        CAPABILITY_FINAL_STATE_EVAL: bool(topology_query_report_exists),
        CAPABILITY_QUERY_BUNDLE_REBUILD: bool(topology_json_exists),
        CAPABILITY_CHECKPOINT_REPLAY: bool(timeline_summary.get("checkpoint_replay_supported")),
        CAPABILITY_DENSE_REPLAY: bool(timeline_summary.get("dense_replay_supported")),
        CAPABILITY_LIFECYCLE_HISTORY: bool(online_topology_lifecycle_exists),
        CAPABILITY_WORKING_TOPOLOGY_HISTORY: bool(working_vs_committed_timeline_exists),
    }


def summarize_artifact_surfaces(artifact_records: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    keys_by_surface: Dict[str, List[str]] = {}
    for key, record in sorted(dict(artifact_records or {}).items()):
        surface = str(record.get("artifact_surface") or artifact_surface_for_key(key))
        keys_by_surface.setdefault(surface, []).append(str(key))
    return {
        "surface_keys": {surface: list(keys) for surface, keys in sorted(keys_by_surface.items())},
        "surface_counts": {surface: int(len(keys)) for surface, keys in sorted(keys_by_surface.items())},
    }


def build_artifact_contract(
    *,
    artifact_profile: str,
    timeline_summary: Dict[str, Any],
    topology_json_exists: bool,
    topology_query_report_exists: bool,
    online_topology_lifecycle_exists: bool,
    working_vs_committed_timeline_exists: bool,
) -> Dict[str, Any]:
    capabilities = build_artifact_capabilities(
        topology_json_exists=topology_json_exists,
        topology_query_report_exists=topology_query_report_exists,
        timeline_summary=dict(timeline_summary or {}),
        online_topology_lifecycle_exists=online_topology_lifecycle_exists,
        working_vs_committed_timeline_exists=working_vs_committed_timeline_exists,
    )
    profile_declarations = declared_artifact_profiles()
    return {
        "version": "0.1",
        "artifact_profile": str(artifact_profile),
        "active_profile_declaration": dict(profile_declarations.get(str(artifact_profile)) or {}),
        "profile_declarations": profile_declarations,
        "timeline_contract": dict(timeline_summary or {}),
        "capabilities": capabilities,
    }
