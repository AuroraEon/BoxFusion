"""Generic static route planner core for RSLG-SLAM (single-scene, 00843).

This is the formal Layer 3 planner surface. It moves route generation from a
hardcoded canonical-route wrapper to an artifact-driven flow:

    QueryTask
      -> artifact loading          (artifact_loader)
      -> target resolution          (target_resolver)
      -> semantic room/floor route  (room_route)
      -> connector resolution       (floor_connector_route)
      -> approach selection         (approach_candidate_planner)
      -> dynamic metric-path stitching (metric_path_stitcher)
      -> RSLGRouteResult            (route_result)

It never launches ROS/Gazebo/RViz/Nav2/AMCL/map_server and never writes under
canonical. The full cross-floor metric path is now **dynamically stitched** from
per-floor occupancy A* segments over the canonical stable maps plus a semantic
connector handoff; its length is computed from that geometry, not copied from a
canonical real route. Canonical real-route lengths appear only as a
comparison/provenance number (``provenance.planner_core.canonical_comparison``).

RSLG-SLAM is the project name; ``BoxFusion`` is only a historical repository path.
This is a single-scene generic static planner core for 00843, not a full
multi-scene planner and not a paper-scale evaluation.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning import artifact_loader as loader
from tools.rslg_pipeline.planning import approach_candidate_planner as approach_planner
from tools.rslg_pipeline.planning import floor_connector_route as connector_route
from tools.rslg_pipeline.planning import metric_path_stitcher
from tools.rslg_pipeline.planning import query_task as qt
from tools.rslg_pipeline.planning import room_route
from tools.rslg_pipeline.planning import target_resolver
from tools.rslg_pipeline.planning.route_result import (
    default_runtime_interface,
    new_route_result,
)
from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
)

PLANNER_VERSION = "rslg_layer3_generic_static_planner_v0_1"
CREATED_BY = "tools/rslg_pipeline/planning/route_planner.py"

OBJECT_QUERY_TYPES = {"object_to_path", "object_in_room", "cross_floor_object"}
STRUCTURE_QUERY_TYPES = {"cross_floor_room", "room_gateway", "floor_connector"}
CROSS_FLOOR_QUERY_TYPES = {"cross_floor_object", "cross_floor_room", "floor_connector"}


class PlannerError(RuntimeError):
    """Raised for unrecoverable planner failures (e.g. guard violations)."""


def _source_records(*artifact_groups: dict[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for group in artifact_groups:
        for record in group.get("artifacts", {}).values():
            records.append(
                {
                    "role": record.get("role"),
                    "path": record.get("path"),
                    "exists": record.get("exists"),
                }
            )
    return records


def plan_static_query(
    query_task: dict[str, Any],
    canonical_root: str | Path,
    *,
    no_canonical_write: bool = True,
) -> dict[str, Any]:
    """Plan a QueryTask into an ``RSLGRouteResult``-compatible dict.

    Resolution (target, room/floor route, connector, approach) flows through the
    planner-core modules, and the metric path is dynamically stitched from
    per-floor occupancy A* segments (:mod:`metric_path_stitcher`). No metric-path
    length is copied from a canonical real route.
    """

    ok, errors = qt.validate(query_task)
    if not ok:
        raise PlannerError(f"invalid QueryTask: {errors}")

    scene_id = query_task["scene_id"]
    query_id = query_task["query_id"]
    query_type = query_task["query_type"]
    start = query_task.get("start") or {}
    target = query_task.get("target") or {}
    expected = query_task.get("expected") or {}

    # --- Layer 2/3 artifact loading -------------------------------------
    context = loader.load_scene_context(canonical_root)
    object_arts = loader.load_object_artifacts(context)
    topology_arts = loader.load_topology_artifacts(context)
    connector_arts = loader.load_connector_artifacts(context)
    stable_map_arts = loader.load_stable_map_artifacts(context)
    comparison_arts = loader.load_current_route_comparison(context)

    # --- target resolution ----------------------------------------------
    grounding = target_resolver.resolve_target(
        query_task, object_arts=object_arts, connector_arts=connector_arts
    )

    is_object_query = query_type in OBJECT_QUERY_TYPES
    is_cross_floor = query_type in CROSS_FLOOR_QUERY_TYPES

    dynamic_fields: list[str] = ["target_resolution"]
    fallback_fields: list[str] = []

    # --- semantic room/floor route --------------------------------------
    start_room = start.get("start_room_id") or "room_2"
    target_room = (
        grounding.get("target_room_id")
        or target.get("target_room_id")
    )
    room_plan = room_route.plan_room_route(topology_arts, start_room, target_room)
    if room_plan["used_fallback"]:
        fallback_fields.append("room_route")
    else:
        dynamic_fields.append("room_route")

    route_crosses_floor = len({f for f in room_plan["floor_sequence"] if f}) > 1

    # --- connector resolution -------------------------------------------
    # Resolve the connector whenever the semantic route crosses floors (not only
    # for the explicit cross-floor query types), so object_in_room / object_to_path
    # that physically cross floors still record the real connector sequence.
    connector: dict[str, Any] | None = None
    connector_sequence: list[dict[str, Any]] = []
    connector_id_hint = grounding.get("connector_id") or target.get("target_gateway_id")
    if not connector_id_hint and room_plan.get("connector_nodes"):
        connector_id_hint = room_plan["connector_nodes"][0]
    if is_cross_floor or route_crosses_floor:
        connector = connector_route.resolve_connector(
            connector_arts,
            from_floor=grounding.get("from_floor") or start.get("start_floor_id") or "floor_1",
            to_floor=grounding.get("to_floor") or target.get("target_floor_id") or "floor_2",
            connector_id=connector_id_hint,
        )
        connector_sequence = [
            {
                "connector_id": connector["connector_id"],
                "connector_id_alias": connector.get("connector_id_alias"),
                "from_floor": connector["from_floor"],
                "to_floor": connector["to_floor"],
                "transition_edge": connector["transition_edge"],
                "non_transition_edge": connector["non_transition_edge"],
            }
        ]
        if connector.get("used_fallback"):
            fallback_fields.append("connector")
        else:
            dynamic_fields.append("connector")

    # --- approach selection (object queries only) -----------------------
    approach_block: dict[str, Any] = {
        "approach_policy": None,
        "approach_candidates": [],
        "selected_approach": None,
        "rejected_candidates": [],
        "candidate_trial_count": None,
    }
    approach_result: dict[str, Any] | None = None
    if is_object_query:
        approach_result = approach_planner.select_object_approach(object_arts)
        selected = approach_result["selected_approach"]
        approach_block = {
            "approach_policy": approach_result["approach_policy"],
            "approach_candidates": [selected],
            "selected_approach": {
                "candidate_id": selected.get("candidate_id"),
                "world_xy": selected.get("world_xy"),
                "yaw": selected.get("yaw"),
                "clearance_m": selected.get("clearance_m", selected.get("endpoint_clearance")),
                "is_object_centroid": False,
                "is_runtime_goal": True,
            },
            "rejected_candidates": approach_result["rejected_candidates"],
            "candidate_trial_count": approach_result["candidate_trial_count"],
        }
        if approach_result["used_fallback"]:
            fallback_fields.append("approach_selection")
        else:
            dynamic_fields.append("approach_selection")

    # --- semantic route --------------------------------------------------
    semantic_route = {
        "room_sequence": room_plan["room_sequence"],
        "floor_sequence": room_plan["floor_sequence"],
        "gateway_sequence": room_plan["gateway_sequence"],
        "vertical_connector_id": connector["connector_id"] if connector else None,
        "connector_sequence": connector_sequence,
    }

    # --- dynamic metric-path stitching ----------------------------------
    stitch = metric_path_stitcher.stitch_metric_path(
        context,
        query_task,
        grounding,
        semantic_route,
        connector,
        approach_result,
        allow_canonical_comparison=True,
        no_canonical_write=no_canonical_write,
    )
    metric_path = stitch["metric_path"]
    route_segments = stitch["route_segments"]
    stitching_report = stitch["stitching_report"]
    scope = stitch["scope"]

    fallback_segment_count = stitching_report["fallback_segment_count"]
    if fallback_segment_count == 0 and metric_path["path_found"]:
        planner_mode = "generic_static_dynamic_metric_stitching"
        dynamic_fields.append("metric_path")
    else:
        planner_mode = "generic_static_dynamic_metric_stitching_with_segment_fallbacks"
        dynamic_fields.append("metric_path")
        fallback_fields.append("metric_path_segments")

    # --- validation ------------------------------------------------------
    all_segments_passed = all(
        s.get("validation_status") == "passed"
        for s in route_segments
        if s.get("path_found")
    )
    path_found = bool(metric_path["path_found"])
    if path_found and fallback_segment_count == 0 and all_segments_passed:
        route_validity, validation_status = "valid", "passed"
    elif path_found:
        route_validity, validation_status = "partial", "partial"
    else:
        route_validity, validation_status = "invalid", "failed"

    validation: dict[str, Any] = {
        "wall_crossing_count": 0 if (path_found and all_segments_passed) else None,
        "invalid_cell_ratio": 0.0 if (path_found and all_segments_passed) else None,
        "endpoint_free": None,
        "endpoint_connected": path_found,
        "endpoint_clearance": None,
        "endpoint_to_object_distance": None,
        "local_path_valid": None,
        "route_feasible": path_found,
        "route_validity": route_validity,
        "validation_status": validation_status,
        "failure_reason": None if path_found else "; ".join(stitching_report["notes"]),
    }
    if is_object_query and approach_result is not None:
        selected = approach_result["selected_approach"]
        validation.update(
            {
                "endpoint_free": bool(selected.get("endpoint_free", True)),
                "endpoint_clearance": selected.get(
                    "clearance_m", selected.get("endpoint_clearance")
                ),
                "endpoint_to_object_distance": selected.get("endpoint_to_object_distance"),
                "local_path_valid": bool(selected.get("local_path_valid", True)),
            }
        )

    # --- guard rails -----------------------------------------------------
    selected_goal = (approach_block.get("selected_approach") or {}).get("candidate_id")
    forbidden_goals = list(expected.get("forbidden_runtime_goals") or [])
    forbidden_edges = list(expected.get("forbidden_transition_edges") or [])
    if selected_goal and selected_goal in forbidden_goals:
        raise PlannerError(
            f"{query_id}: selected runtime goal {selected_goal!r} is forbidden"
        )
    if selected_goal and selected_goal in BLOCKED_LEGACY_APPROACH_IDS:
        raise PlannerError(
            f"{query_id}: selected runtime goal {selected_goal!r} is a blocked legacy id"
        )
    transition_edge_used = None
    for seg in route_segments:
        if seg.get("segment_type") in {"vertical_transition", "connector_handoff"} and seg.get(
            "transition_edge"
        ):
            transition_edge_used = seg["transition_edge"]
            break
    if transition_edge_used is None and connector is not None:
        transition_edge_used = connector.get("transition_edge")
    if transition_edge_used == NON_TRANSITION_EDGE or transition_edge_used in forbidden_edges:
        raise PlannerError(
            f"{query_id}: transition edge {transition_edge_used!r} is forbidden / non-transition"
        )

    # --- light geometry --------------------------------------------------
    floor2_yaml = context["stable_map_yaml"].get("floor_2")
    connectors_ref = connector_arts["artifacts"]["vertical_connectors"]
    approach_ref = object_arts["artifacts"]["object_approach_selected"]
    light_geometry = {
        "stable_map_profile": "conservative_canonical",
        "traversability_map_ref": loader.rel(floor2_yaml) if floor2_yaml.is_file() else None,
        "room_mask_ref": None,
        "gateway_geometry_ref": None,
        "vertical_connector_ref": connectors_ref["path"] if connectors_ref["exists"] else None,
        "object_geometry_ref": approach_ref["path"] if (is_object_query and approach_ref["exists"]) else None,
    }

    # --- canonical comparison (provenance only, never the generated path) -
    canonical_comparison = {
        "role": "comparison_provenance_only",
        "comparison_length_m": stitching_report["canonical_comparison_length_if_available"],
        "stitched_length_m": metric_path["path_length"],
        "length_delta_vs_canonical_m": stitching_report[
            "length_delta_vs_canonical_if_available"
        ],
        "source_artifacts": [
            record.get("path")
            for record in comparison_arts.get("artifacts", {}).values()
        ],
    }

    # --- provenance ------------------------------------------------------
    provenance = {
        "source_artifacts": _source_records(
            object_arts, topology_arts, connector_arts, stable_map_arts, comparison_arts
        ),
        "source_hashes_if_available": {},
        "canonical_root": context["canonical_root_rel"],
        "notes": (
            "Generic static planner core (single-scene 00843). Target, room/floor "
            "route, connector, and approach resolved through planner-core modules; "
            "the metric path is dynamically stitched from per-floor occupancy A* "
            "segments. Canonical real routes are comparison/provenance only."
        ),
        "planner_core": {
            "planner_mode": planner_mode,
            "target_grounding": grounding,
            "room_route": {
                "node_sequence": room_plan["node_sequence"],
                "used_fallback": room_plan["used_fallback"],
                "fallback_reason": room_plan["fallback_reason"],
            },
            "connector": connector,
            "approach_selection": (
                {
                    "selection_reason": approach_result["selection_reason"],
                    "candidate_trial_count": approach_result["candidate_trial_count"],
                    "valid_candidate_count": approach_result["valid_candidate_count"],
                    "used_fallback": approach_result["used_fallback"],
                }
                if approach_result
                else None
            ),
            "metric_path_scope": scope["metric_path_scope"],
            "route_generation_scope": scope["route_generation_scope"],
            "structure_sequence_required": scope["structure_sequence_required"],
            "stitching_report": stitching_report,
            "canonical_comparison": canonical_comparison,
            "dynamic_fields": sorted(set(dynamic_fields)),
            "fallback_fields": sorted(set(fallback_fields)),
        },
        "query_task_guardrails": {
            "query_id": query_id,
            "query_type": query_type,
            "expected_object_id": expected.get("expected_object_id"),
            "expected_room_sequence": list(expected.get("expected_room_sequence") or []),
            "expected_vertical_connector": expected.get("expected_vertical_connector"),
            "expected_approach_candidate_id": expected.get("expected_approach_candidate_id"),
            "forbidden_runtime_goals": forbidden_goals,
            "forbidden_transition_edges": forbidden_edges,
            "selected_runtime_goal": selected_goal,
            "transition_edge_used": transition_edge_used,
            "structure_sequence_required": scope["structure_sequence_required"],
            "blocked_candidate_rejection": bool(grounding.get("blocked_candidate_rejection")),
        },
    }

    result = new_route_result(
        query_id=query_id,
        query_type=query_type,
        scene_id=scene_id,
        query_text=query_task.get("query_text"),
        planner_version=PLANNER_VERSION,
        created_by=CREATED_BY,
        start={
            "start_pose": start.get("start_pose"),
            "start_room_id": start.get("start_room_id"),
            "start_floor_id": start.get("start_floor_id"),
        },
        target={
            "target_type": target.get("target_type"),
            "target_object_id": target.get("target_object_id") or grounding.get("target_object_id"),
            "target_object_category": target.get("target_object_category")
            or grounding.get("target_object_category"),
            "target_room_id": target.get("target_room_id") or grounding.get("target_room_id"),
            "target_floor_id": target.get("target_floor_id") or grounding.get("target_floor_id"),
            "target_gateway_id": target.get("target_gateway_id")
            or (connector["connector_id"] if connector else None),
        },
        semantic_route=semantic_route,
        route_segments=route_segments,
        approach=approach_block,
        metric_path=metric_path,
        validation=validation,
        light_geometry=light_geometry,
        runtime_interface=default_runtime_interface(),
        provenance=provenance,
    )
    result["route_generation"] = {
        "metric_path_scope": scope["metric_path_scope"],
        "route_generation_scope": scope["route_generation_scope"],
        "structure_sequence_required": scope["structure_sequence_required"],
        "planner_mode": planner_mode,
        "metric_path_source": metric_path["path_source"],
        "stitching_report": stitching_report,
    }
    return result

