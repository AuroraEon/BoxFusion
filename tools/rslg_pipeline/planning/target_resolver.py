"""QueryTask target resolution for the RSLG-SLAM static planner.

Resolves the ``target`` block of an ``rslg_query_task`` into a structured target
grounding that the route planner embeds into ``RSLGRouteResult`` provenance and
target fields. RSLG-SLAM is the project name; ``BoxFusion`` is only a historical
repository path.

This resolver is deliberately narrow. It grounds concrete object/room/floor/
connector identifiers against current 00843 canonical artifacts. It does **not**
perform arbitrary natural-language parsing — the QueryTask already carries the
structured target descriptors.

Supported resolution kinds map onto the query types:

* ``object_to_path``       -> object grounding
* ``object_in_room``       -> object + room grounding
* ``cross_floor_object``   -> object + room + floor grounding
* ``blocked_candidate_rejection`` -> object grounding + explicit blocked record
* ``cross_floor_room``     -> room + floor grounding
* ``room_gateway``         -> start/target room grounding
* ``floor_connector``      -> floor transition + connector grounding
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
    OBJECT_ID,
    OBJECT_LABEL,
    OBJECT_ROOM,
    TRUE_TRANSITION_EDGE,
)

OBJECT_QUERY_TYPES = {"object_to_path", "object_in_room", "cross_floor_object"}
STRUCTURE_QUERY_TYPES = {"cross_floor_room", "room_gateway", "floor_connector"}


def _object_record(object_arts: dict[str, Any]) -> dict[str, Any] | None:
    resolution = (
        object_arts.get("artifacts", {})
        .get("object_query_resolution", {})
        .get("data")
    )
    if isinstance(resolution, dict):
        return resolution.get("current_layer1_object_record")
    return None


def _resolve_object(
    query: dict[str, Any], object_arts: dict[str, Any]
) -> dict[str, Any]:
    """Resolve the target object id/category against object artifacts."""

    target = query.get("target") or {}
    requested_id = target.get("target_object_id")
    requested_category = target.get("target_object_category")

    resolution = (
        object_arts.get("artifacts", {})
        .get("object_query_resolution", {})
        .get("data")
    )
    candidates: list[dict[str, Any]] = []
    resolved_id: str | None = None
    resolved_label: str | None = None
    resolved_room: str | None = None
    resolved_floor: str | None = None
    reason = "unresolved"

    if isinstance(resolution, dict):
        art_id = resolution.get("object_id")
        art_label = resolution.get("object_label")
        candidates.append(
            {
                "object_id": art_id,
                "object_label": art_label,
                "source": "object_query_resolution_v0_1",
            }
        )
        # Match by explicit id first, then by category, then fall back to the
        # single resolved object in the current single-object queryset.
        if requested_id and requested_id == art_id:
            resolved_id, resolved_label = art_id, art_label
            reason = "matched_target_object_id"
        elif requested_category and requested_category == art_label:
            resolved_id, resolved_label = art_id, art_label
            reason = "matched_target_object_category"
        elif not requested_id and not requested_category:
            resolved_id, resolved_label = art_id, art_label
            reason = "single_scene_object_resolution"
        resolved_room = resolution.get("target_room")
        resolved_floor = resolution.get("target_floor")

    # Guarded fallback to project truth when artifacts are absent but the
    # QueryTask names the current curtain object.
    if resolved_id is None:
        if requested_id == OBJECT_ID or requested_category == OBJECT_LABEL:
            resolved_id, resolved_label = OBJECT_ID, OBJECT_LABEL
            resolved_room = resolved_room or OBJECT_ROOM
            reason = "project_truth_fallback"

    return {
        "target_kind": "object",
        "target_object_id": resolved_id,
        "target_object_category": resolved_label,
        "target_room_id": target.get("target_room_id") or resolved_room,
        "target_floor_id": target.get("target_floor_id") or resolved_floor,
        "candidates": candidates,
        "resolution_reason": reason,
        "resolved": resolved_id is not None,
    }


def _resolve_room(query: dict[str, Any]) -> dict[str, Any]:
    target = query.get("target") or {}
    room_id = target.get("target_room_id")
    return {
        "target_kind": "room",
        "target_room_id": room_id,
        "target_floor_id": target.get("target_floor_id"),
        "candidates": [{"room_id": room_id, "source": "query_task_target"}] if room_id else [],
        "resolution_reason": "matched_target_room_id" if room_id else "unresolved",
        "resolved": bool(room_id),
    }


def _resolve_floor_connector(
    query: dict[str, Any], connector_arts: dict[str, Any]
) -> dict[str, Any]:
    target = query.get("target") or {}
    connector_data = (
        connector_arts.get("artifacts", {})
        .get("vertical_connectors", {})
        .get("data")
    )
    connectors = []
    if isinstance(connector_data, dict):
        connectors = connector_data.get("connectors") or []

    gateway = target.get("target_gateway_id") or "vt_1"
    matched = None
    for connector in connectors:
        if connector.get("connector_id") == gateway:
            matched = connector
            break

    from_floor = None
    to_floor = None
    transition_edge = TRUE_TRANSITION_EDGE
    if matched:
        from_floor = matched.get("source_floor")
        to_floor = matched.get("target_floor")
        transition_edge = matched.get("transition_edge") or TRUE_TRANSITION_EDGE

    return {
        "target_kind": "floor_connector",
        "connector_id": gateway,
        "from_floor": from_floor or "floor_1",
        "to_floor": to_floor or target.get("target_floor_id") or "floor_2",
        "target_room_id": target.get("target_room_id"),
        "transition_edge": transition_edge,
        "non_transition_edge": NON_TRANSITION_EDGE,
        "candidates": [{"connector_id": c.get("connector_id")} for c in connectors],
        "resolution_reason": "matched_vertical_connector" if matched else "project_truth_fallback",
        "resolved": True,
    }


def resolve_target(
    query: dict[str, Any],
    *,
    object_arts: dict[str, Any] | None = None,
    connector_arts: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Resolve a QueryTask target into a normalized target grounding dict.

    The output is a normalized dict suitable for embedding in ``RSLGRouteResult``
    provenance/target. It records candidate targets, the selected target, and a
    resolution reason. It never fabricates arbitrary NL grounding.
    """

    object_arts = object_arts or {"artifacts": {}}
    connector_arts = connector_arts or {"artifacts": {}}

    query_type = query.get("query_type")
    query_id = query.get("query_id") or ""
    is_blocked_rejection = "blocked_candidate_rejection" in query_id

    target = query.get("target") or {}
    target_type = target.get("target_type")

    grounding: dict[str, Any]
    if target_type == "object" or query_type in OBJECT_QUERY_TYPES:
        grounding = _resolve_object(query, object_arts)
        if is_blocked_rejection:
            grounding["blocked_candidate_rejection"] = True
            grounding["blocked_candidate_ids"] = list(BLOCKED_LEGACY_APPROACH_IDS)
    elif target_type == "floor_connector" or query_type == "floor_connector":
        grounding = _resolve_floor_connector(query, connector_arts)
    elif target_type == "room" or query_type in {"cross_floor_room", "room_gateway"}:
        grounding = _resolve_room(query)
    else:
        grounding = {
            "target_kind": "unknown",
            "resolution_reason": f"unsupported target_type/query_type {target_type!r}/{query_type!r}",
            "resolved": False,
            "candidates": [],
        }

    grounding["query_type"] = query_type
    grounding["target_type"] = target_type
    return grounding
