"""RSLGRouteResult: central Layer 3 route-interface artifact for RSLG-SLAM.

This module owns the ``rslg_route_result`` schema constants, a construction
helper, and a self-contained semantic validator. RSLG-SLAM is the project name;
``BoxFusion`` is only a historical repository path.

``RSLGRouteResult`` is the single schema-consistent artifact that Layer 3 route
generation produces and that the Layer 4 lightweight PID ``/cmd_vel`` + ``/odom``
follower, the visualization adapters, and the future Table 2/3/4 evaluators
consume. It wraps the current room and object route artifacts without changing
canonical outputs.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
)

SCHEMA_NAME = "rslg_route_result"
SCHEMA_VERSION = "0.1"
PLANNER_VERSION = "rslg_layer3_route_result_v0_1"
CREATED_BY = "tools/rslg_pipeline/planning/route_result.py"
SCHEMA_PATH = Path(__file__).resolve().parents[1] / "schemas" / "route_result_schema.json"

QUERY_TYPES = (
    "object_to_path",
    "object_in_room",
    "room_gateway",
    "floor_connector",
    "cross_floor_room",
    "cross_floor_object",
)
VALIDATION_STATUSES = ("passed", "failed", "partial", "not_validated", "blocked")
SEGMENT_TYPES = (
    "same_floor",
    "vertical_transition",
    "object_approach",
    "same_floor_metric",
    "connector_handoff",
    "object_approach_metric",
    "semantic_only",
)
TARGET_TYPES = ("object", "room", "gateway", "floor_connector")

REQUIRED_SECTIONS = (
    "identity",
    "start",
    "target",
    "semantic_route",
    "route_segments",
    "approach",
    "metric_path",
    "validation",
    "light_geometry",
    "runtime_interface",
    "claim_boundary",
    "provenance",
)


def default_claim_boundary() -> dict[str, bool]:
    """Return the fixed RSLG-SLAM claim-boundary block.

    Every flag encodes an explicit non-claim; the schema and validator require
    each of these to stay true so a route result can never assert Nav2/AMCL,
    real-robot, collision-free, physical-stair, or Go2 control capability.
    """

    return {
        "no_nav2_dependency": True,
        "no_amcl_dependency": True,
        "no_real_robot_claim": True,
        "no_collision_free_guarantee": True,
        "no_physical_stair_climbing_claim": True,
        "no_go2_control_claim": True,
    }


def default_runtime_interface(runtime_input_ref: str | None = None) -> dict[str, Any]:
    """Return the runtime-interface block for the lightweight PID follower.

    Nav2 and AMCL are never current dependencies, so both flags are false.
    """

    return {
        "compatible_with_pid_follower": True,
        "requires_nav2": False,
        "requires_amcl": False,
        "runtime_input_ref": runtime_input_ref,
    }


def new_route_result(
    *,
    query_id: str,
    query_type: str,
    scene_id: str,
    query_text: str | None = None,
    planner_version: str = PLANNER_VERSION,
    created_by: str = CREATED_BY,
    start: dict[str, Any] | None = None,
    target: dict[str, Any] | None = None,
    semantic_route: dict[str, Any] | None = None,
    route_segments: list[dict[str, Any]] | None = None,
    approach: dict[str, Any] | None = None,
    metric_path: dict[str, Any] | None = None,
    validation: dict[str, Any] | None = None,
    light_geometry: dict[str, Any] | None = None,
    runtime_interface: dict[str, Any] | None = None,
    provenance: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Assemble a schema-consistent ``RSLGRouteResult`` dictionary."""

    return {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT_NAME,
        "identity": {
            "query_id": query_id,
            "query_text": query_text,
            "query_type": query_type,
            "scene_id": scene_id,
            "planner_version": planner_version,
            "created_by": created_by,
        },
        "start": start
        or {"start_pose": None, "start_room_id": None, "start_floor_id": None},
        "target": target
        or {
            "target_type": "room",
            "target_object_id": None,
            "target_object_category": None,
            "target_room_id": None,
            "target_floor_id": None,
            "target_gateway_id": None,
        },
        "semantic_route": semantic_route
        or {
            "room_sequence": [],
            "floor_sequence": [],
            "gateway_sequence": [],
            "vertical_connector_id": None,
            "connector_sequence": [],
        },
        "route_segments": route_segments or [],
        "approach": approach
        or {
            "approach_policy": None,
            "approach_candidates": [],
            "selected_approach": None,
            "rejected_candidates": [],
            "candidate_trial_count": None,
        },
        "metric_path": metric_path
        or {"path": None, "path_length": None, "path_found": None, "endpoint": None},
        "validation": validation
        or {
            "wall_crossing_count": None,
            "invalid_cell_ratio": None,
            "endpoint_free": None,
            "endpoint_connected": None,
            "endpoint_clearance": None,
            "endpoint_to_object_distance": None,
            "local_path_valid": None,
            "route_feasible": None,
            "route_validity": None,
            "validation_status": "not_validated",
            "failure_reason": None,
        },
        "light_geometry": light_geometry
        or {
            "stable_map_profile": None,
            "traversability_map_ref": None,
            "room_mask_ref": None,
            "gateway_geometry_ref": None,
            "vertical_connector_ref": None,
            "object_geometry_ref": None,
        },
        "runtime_interface": runtime_interface or default_runtime_interface(),
        "claim_boundary": default_claim_boundary(),
        "provenance": provenance
        or {
            "source_artifacts": [],
            "source_hashes_if_available": {},
            "canonical_root": None,
            "notes": None,
        },
    }


def _selected_candidate_ids(data: dict[str, Any]) -> list[str]:
    approach = data.get("approach") or {}
    ids: list[str] = []
    selected = approach.get("selected_approach")
    if isinstance(selected, dict):
        cid = selected.get("candidate_id")
        if cid:
            ids.append(str(cid))
    return ids


def _rejected_candidate_ids(data: dict[str, Any]) -> set[str]:
    approach = data.get("approach") or {}
    ids: set[str] = set()
    for record in approach.get("rejected_candidates") or []:
        if isinstance(record, dict) and record.get("candidate_id"):
            ids.add(str(record["candidate_id"]))
        elif isinstance(record, str):
            ids.add(record)
    return ids


def _transition_edges_used(data: dict[str, Any]) -> list[str]:
    """Collect edges that a vertical-transition segment or connector uses."""

    edges: list[str] = []
    for segment in data.get("route_segments") or []:
        if not isinstance(segment, dict):
            continue
        if segment.get("segment_type") not in {"vertical_transition", "connector_handoff"}:
            continue
        edge = segment.get("transition_edge") or segment.get("connector_edge")
        if edge:
            edges.append(str(edge))
    semantic = data.get("semantic_route") or {}
    for connector in semantic.get("connector_sequence") or []:
        if isinstance(connector, dict) and connector.get("transition_edge"):
            edges.append(str(connector["transition_edge"]))
    return edges


def validate(data: dict[str, Any]) -> tuple[bool, list[str]]:
    """Validate an ``RSLGRouteResult`` payload against the RSLG-SLAM rules."""

    errors: list[str] = []

    if data.get("schema_name") != SCHEMA_NAME:
        errors.append(f"schema_name must be {SCHEMA_NAME!r}")
    if not data.get("schema_version"):
        errors.append("schema_version is required")

    for section in REQUIRED_SECTIONS:
        if section not in data:
            errors.append(f"missing required section: {section}")

    identity = data.get("identity") or {}
    for key in ("query_id", "query_type", "scene_id", "planner_version", "created_by"):
        if not identity.get(key):
            errors.append(f"identity.{key} is required")
    if identity.get("query_type") and identity["query_type"] not in QUERY_TYPES:
        errors.append(
            f"identity.query_type {identity['query_type']!r} not in {QUERY_TYPES}"
        )

    target = data.get("target") or {}
    if target.get("target_type") and target["target_type"] not in TARGET_TYPES:
        errors.append(f"target.target_type {target['target_type']!r} not in {TARGET_TYPES}")

    for index, segment in enumerate(data.get("route_segments") or []):
        if not isinstance(segment, dict):
            errors.append(f"route_segments[{index}] must be an object")
            continue
        stype = segment.get("segment_type")
        if stype not in SEGMENT_TYPES:
            errors.append(f"route_segments[{index}].segment_type {stype!r} not in {SEGMENT_TYPES}")
        vstatus = segment.get("validation_status")
        if vstatus not in VALIDATION_STATUSES:
            errors.append(
                f"route_segments[{index}].validation_status {vstatus!r} not in {VALIDATION_STATUSES}"
            )

    validation = data.get("validation") or {}
    vstatus = validation.get("validation_status")
    if vstatus not in VALIDATION_STATUSES:
        errors.append(f"validation.validation_status {vstatus!r} not in {VALIDATION_STATUSES}")

    runtime = data.get("runtime_interface") or {}
    if runtime.get("requires_nav2") is not False:
        errors.append("runtime_interface.requires_nav2 must be false")
    if runtime.get("requires_amcl") is not False:
        errors.append("runtime_interface.requires_amcl must be false")

    claim = data.get("claim_boundary") or {}
    for key in default_claim_boundary():
        if claim.get(key) is not True:
            errors.append(f"claim_boundary.{key} must be true")

    # Blocked legacy approach ids must never be selected as the runtime goal.
    selected_ids = _selected_candidate_ids(data)
    rejected_ids = _rejected_candidate_ids(data)
    for blocked_id in BLOCKED_LEGACY_APPROACH_IDS:
        if blocked_id in selected_ids:
            errors.append(
                f"blocked legacy approach {blocked_id!r} must not be the selected runtime goal"
            )
        # If a blocked id appears at all it must be in rejected/blocked context.
        appears = blocked_id in json.dumps(data)
        if appears and blocked_id not in rejected_ids:
            errors.append(
                f"blocked legacy approach {blocked_id!r} appears but is not marked as a rejected candidate"
            )

    # A vertical-transition segment must use the true transition edge only.
    transition_edges = _transition_edges_used(data)
    for edge in transition_edges:
        if edge == NON_TRANSITION_EDGE:
            errors.append(
                f"non-transition edge {NON_TRANSITION_EDGE!r} must not be used as a transition edge"
            )
        elif edge and edge != TRUE_TRANSITION_EDGE:
            errors.append(
                f"unexpected transition edge {edge!r}; expected {TRUE_TRANSITION_EDGE!r}"
            )

    return not errors, errors


def load_schema() -> dict[str, Any]:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def dump(data: dict[str, Any]) -> str:
    return json.dumps(data, indent=2, sort_keys=True) + "\n"
