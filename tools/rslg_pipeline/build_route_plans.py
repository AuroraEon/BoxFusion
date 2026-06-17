"""Build dry-run route plan previews from candidate route contracts.

This module is intentionally preview-only. It reads Layer 3 candidate route
contracts and writes small route plan dry-run artifacts without generating real
A* samples, executable waypoints, runtime input packages, map pixels, connector
geometry, object approach geometry, or runtime validation evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .common import PROJECT_NAME, load_json, normalize_repo_relative, repo_path, resolve_repo_root, save_json


SCHEMA_NAME = "rslg_route_plan_preview"
SCHEMA_VERSION = "0.1"
ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"
ARTIFACT_ROLE = "route_plan_dry_run_preview"

SUPPORTED_ROUTE_KINDS = {"cross_floor_room", "cross_floor_object"}

PREVIEW_FILES = {
    "cross_floor_room": "cross_floor_room_route_plan_preview_v0_1.json",
    "cross_floor_object": "cross_floor_object_route_plan_preview_v0_1.json",
}

PREVIEW_CLASSIFICATIONS = {
    "cross_floor_room": "cross_floor_room_route_plan_preview_generated",
    "cross_floor_object": "cross_floor_object_route_plan_preview_generated",
}

SUMMARY_CLASSIFICATIONS = {
    "cross_floor_room": "cross_floor_room_route_plan_preview_generation_completed",
    "cross_floor_object": "cross_floor_object_route_plan_preview_generation_completed",
}

SOURCE_FLOOR = "floor_1"
TARGET_FLOOR = "floor_2"
ROOM_ROUTE_SEQUENCE = ["room_2", "room_3", "room_7", "room_13", "room_14"]
CONNECTOR_ID = "vt_1"
CONNECTOR_ALIAS = "vc_vt_1"
TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"

OBJECT_QUERY = "curtain in room_14 on floor_2"
OBJECT_ID = "obj_175"
OBJECT_LABEL = "curtain"
APPROACH_CANDIDATE_ID = "generated_ring_037"

COMMON_REQUIRED_FUTURE_INPUTS = [
    "final stable occupancy map package",
    "final vertical connector / cross-floor topology artifact",
    "route planner graph",
]

OBJECT_REQUIRED_FUTURE_INPUTS = COMMON_REQUIRED_FUTURE_INPUTS + [
    "final object interface artifact",
    "approach feasibility / clearance evidence",
]

FALSE_SAFETY_FLAGS = {
    "historical_stage_outputs_required": False,
    "stage_outputs_required": False,
    "world_model_rerun": False,
    "runtime_launched": False,
    "old_scripts_called": False,
    "business_logic_migrated": False,
    "real_astar_route_generated": False,
    "astar_waypoints_generated": False,
    "executable_route_generated": False,
    "runtime_input_package_generated": False,
    "runtime_artifacts_generated": False,
    "map_pixels_generated": False,
    "connector_geometry_regenerated": False,
    "object_approach_geometry_regenerated": False,
    "object_centroid_navigation_used": False,
    "direct_object_centroid_goal_used": False,
}


def _contract_payload(contract: Mapping[str, Any]) -> Mapping[str, Any]:
    payload = contract.get("route_contract_payload", {})
    return payload if isinstance(payload, Mapping) else {}


def _validate_source_contract(contract: Mapping[str, Any], route_kind: str, source_path: Path) -> list[str]:
    errors: list[str] = []
    if route_kind not in SUPPORTED_ROUTE_KINDS:
        errors.append(f"unsupported route_kind {route_kind!r}")
    expected_classification = {
        "cross_floor_room": "cross_floor_room_candidate_route_contract_generated",
        "cross_floor_object": "cross_floor_object_candidate_route_contract_generated",
    }.get(route_kind)
    if contract.get("classification") != expected_classification:
        errors.append(
            f"{source_path.as_posix()}: classification must be {expected_classification!r}"
        )
    for key, expected in {
        "artifact_layer": ARTIFACT_LAYER,
        "route_kind": route_kind,
        "is_candidate": True,
        "is_final_route_contract": False,
        "generated_from_layer2_candidate_artifacts": True,
        "generated_from_final_layer2_artifacts": False,
    }.items():
        if contract.get(key) != expected:
            errors.append(f"{source_path.as_posix()}: {key} must be {expected!r}")
    for flag in [
        "world_model_rerun",
        "runtime_launched",
        "old_scripts_called",
        "business_logic_migrated",
        "real_astar_route_generated",
        "executable_route_generated",
        "runtime_input_package_generated",
        "runtime_artifacts_generated",
        "map_pixels_generated",
        "connector_geometry_regenerated",
        "object_approach_geometry_regenerated",
        "object_centroid_navigation_used",
        "direct_object_centroid_goal_used",
    ]:
        if contract.get(flag) is not False:
            errors.append(f"{source_path.as_posix()}: source safety flag {flag} must be false")

    raw_text = json.dumps(contract, sort_keys=True)
    for token in [CONNECTOR_ID, TRANSITION_EDGE, NON_TRANSITION_EDGE, SOURCE_FLOOR, TARGET_FLOOR, *ROOM_ROUTE_SEQUENCE]:
        if token not in raw_text:
            errors.append(f"{source_path.as_posix()}: source contract missing {token!r}")

    if route_kind == "cross_floor_object":
        payload = _contract_payload(contract)
        expected_object = {
            "query": OBJECT_QUERY,
            "object_id": OBJECT_ID,
            "object_label": OBJECT_LABEL,
            "target_floor": TARGET_FLOOR,
            "target_room": "room_14",
            "approach_candidate_id": APPROACH_CANDIDATE_ID,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
            "approach_geometry_regenerated": False,
            "approach_feasibility_revalidated": False,
        }
        for key, expected in expected_object.items():
            if payload.get(key) != expected:
                errors.append(f"{source_path.as_posix()}: route_contract_payload.{key} must be {expected!r}")
    return errors


def _route_intent_items(route_kind: str) -> list[str]:
    items = [
        "room_2 on floor_1",
        "room_3 on floor_1",
        "vt_1 / vc_vt_1 connector",
        "room_7 on floor_2",
        "room_13 on floor_2",
        "room_14 on floor_2",
    ]
    if route_kind == "cross_floor_object":
        items.append("approach generated_ring_037 near obj_175")
    return items


def _segment_preview_list(route_kind: str) -> list[dict[str, Any]]:
    segments = [
        {
            "segment_type": "floor_1_room_segment",
            "preview": "room_2 -> room_3",
            "source_floor": SOURCE_FLOOR,
            "target_floor": SOURCE_FLOOR,
            "rooms": ["room_2", "room_3"],
            "is_real_planner_output": False,
        },
        {
            "segment_type": "vertical_transition_segment",
            "preview": "room_3 -> vt_1/vc_vt_1 -> room_7",
            "source_floor": SOURCE_FLOOR,
            "target_floor": TARGET_FLOOR,
            "connector_reference": {
                "connector_id": CONNECTOR_ID,
                "connector_alias": CONNECTOR_ALIAS,
                "transition_edge": TRANSITION_EDGE,
                "non_transition_edge": NON_TRANSITION_EDGE,
            },
            "rooms": ["room_3", "room_7"],
            "is_real_planner_output": False,
        },
        {
            "segment_type": "floor_2_room_segment",
            "preview": "room_7 -> room_13 -> room_14",
            "source_floor": TARGET_FLOOR,
            "target_floor": TARGET_FLOOR,
            "rooms": ["room_7", "room_13", "room_14"],
            "is_real_planner_output": False,
        },
    ]
    if route_kind == "cross_floor_object":
        segments.append(
            {
                "segment_type": "object_approach_segment",
                "preview": "room_14 -> generated_ring_037 near obj_175",
                "source_floor": TARGET_FLOOR,
                "target_floor": TARGET_FLOOR,
                "target_room": "room_14",
                "object_id": OBJECT_ID,
                "approach_candidate_id": APPROACH_CANDIDATE_ID,
                "object_centroid_navigation_used": False,
                "direct_object_centroid_goal_used": False,
                "approach_geometry_regenerated": False,
                "approach_feasibility_revalidated": False,
                "is_real_planner_output": False,
            }
        )
    return segments


def _blocked_real_planner_fields(route_kind: str) -> list[dict[str, Any]]:
    fields = [
        ("real A* waypoint samples", "requires a real route planner graph and final stable occupancy maps"),
        ("executable route waypoints", "requires later route generation and execution-interface export"),
        ("runtime input package", "requires a separate runtime input export task"),
        ("runtime validation evidence", "requires a separate Layer 4 validation task"),
        ("map pixels / PGM / map_server YAML", "not generated by route plan previews"),
        ("connector geometry regeneration", "requires a later Layer 2 formal artifact task"),
    ]
    if route_kind == "cross_floor_object":
        fields.extend(
            [
                ("object approach geometry regeneration", "approach candidate is referenced only"),
                ("object approach feasibility revalidation", "requires future clearance evidence"),
                ("direct object centroid goal", "not allowed for this object route"),
            ]
        )
    return [{"field": field, "blocked_reason": reason} for field, reason in fields]


def _claim_boundary(route_kind: str) -> dict[str, Any]:
    return {
        "scope": "Layer 3 route plan dry-run preview only.",
        "route_kind": route_kind,
        "is_route_plan_preview": True,
        "is_final_route_plan": False,
        "generated_from_candidate_route_contract": True,
        "generated_from_final_route_contract": False,
        "does_not_generate_final_route_plans": True,
        "does_not_generate_real_astar_routes": True,
        "does_not_generate_astar_waypoints": True,
        "does_not_generate_executable_routes": True,
        "does_not_generate_runtime_input_packages": True,
        "does_not_generate_runtime_validation_artifacts": True,
        "does_not_launch_gazebo_rviz_nav2_amcl_or_object_nav": True,
        "does_not_generate_map_pixels_pgm_or_map_server_yaml": True,
        "does_not_regenerate_connector_geometry": True,
        "does_not_regenerate_object_approach_geometry": True,
        "does_not_revalidate_approach_feasibility": True,
        "does_not_use_object_centroid_navigation_or_direct_centroid_goal": True,
        "does_not_claim_amcl_success": True,
        "does_not_claim_physical_or_real_robot_stair_climbing": True,
        "does_not_claim_full_object_navigation_benchmark": True,
    }


def _route_plan_preview_payload(route_kind: str) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "route_kind": route_kind,
        "route_intent": _route_intent_items(route_kind),
        "segment_preview_list": _segment_preview_list(route_kind),
        "segment_types": [segment["segment_type"] for segment in _segment_preview_list(route_kind)],
        "connector_reference": {
            "connector_id": CONNECTOR_ID,
            "connector_alias": CONNECTOR_ALIAS,
            "allowed_connector_ids": [CONNECTOR_ID, CONNECTOR_ALIAS],
            "source_floor": SOURCE_FLOOR,
            "target_floor": TARGET_FLOOR,
            "floor_transition": "floor_1 -> floor_2",
            "transition_edge": TRANSITION_EDGE,
            "non_transition_edge": NON_TRANSITION_EDGE,
        },
        "room_route_context": ROOM_ROUTE_SEQUENCE,
        "floor_transition": {"source_floor": SOURCE_FLOOR, "target_floor": TARGET_FLOOR},
        "required_future_planner_inputs": list(COMMON_REQUIRED_FUTURE_INPUTS),
        "object_approach_segment_included": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
    }
    if route_kind == "cross_floor_object":
        payload.update(
            {
                "query": OBJECT_QUERY,
                "object_id": OBJECT_ID,
                "object_label": OBJECT_LABEL,
                "target_floor": TARGET_FLOOR,
                "target_room": "room_14",
                "approach_candidate_id": APPROACH_CANDIDATE_ID,
                "object_approach_segment_included": True,
                "object_safety": {
                    "object_centroid_navigation_used": False,
                    "direct_object_centroid_goal_used": False,
                    "approach_geometry_regenerated": False,
                    "approach_feasibility_revalidated": False,
                },
                "object_centroid_navigation_used": False,
                "direct_object_centroid_goal_used": False,
                "approach_geometry_regenerated": False,
                "approach_feasibility_revalidated": False,
                "required_future_planner_inputs": list(OBJECT_REQUIRED_FUTURE_INPUTS),
            }
        )
    return payload


def build_route_plan_preview(
    repo_root: Path,
    scene_id: str,
    route_kind: str,
    candidate_route_contract: Path,
    output_dir: Path,
    *,
    dry_run: bool,
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    if not dry_run:
        errors.append("Pass --dry-run; route plan generation is preview-only in this task.")
    if route_kind not in SUPPORTED_ROUTE_KINDS:
        errors.append(f"unsupported route_kind {route_kind!r}")

    source_contract: Mapping[str, Any] = {}
    try:
        loaded = load_json(candidate_route_contract)
        if not isinstance(loaded, Mapping):
            raise ValueError("candidate route contract must be a JSON object")
        source_contract = loaded
        errors.extend(_validate_source_contract(source_contract, route_kind, candidate_route_contract))
    except Exception as exc:
        errors.append(f"failed to load candidate route contract {candidate_route_contract.as_posix()}: {exc}")

    preview_filename = PREVIEW_FILES.get(route_kind, "unsupported_route_plan_preview_v0_1.json")
    preview_path = output_dir / preview_filename
    preview_classification = PREVIEW_CLASSIFICATIONS.get(route_kind, "route_plan_preview_generation_failed")
    source_record = {
        "path": normalize_repo_relative(candidate_route_contract, repo_root),
        "classification": source_contract.get("classification"),
        "route_kind": source_contract.get("route_kind"),
        "is_candidate": source_contract.get("is_candidate"),
        "is_final_route_contract": source_contract.get("is_final_route_contract"),
    }

    preview = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT_NAME,
        "classification": preview_classification,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": ARTIFACT_ROLE,
        "scene_id": scene_id,
        "route_kind": route_kind,
        "is_route_plan_preview": True,
        "is_final_route_plan": False,
        "generated_from_candidate_route_contract": True,
        "generated_from_final_route_contract": False,
        "generated_from_layer2_candidate_artifacts": True,
        **FALSE_SAFETY_FLAGS,
        "source_candidate_route_contract": source_record,
        "route_plan_preview_payload": _route_plan_preview_payload(route_kind),
        "blocked_until_real_planner_inputs": _blocked_real_planner_fields(route_kind),
        "downstream_route_generation_unblock_role": {
            "status": "blocked_until_final_layer2_artifacts_and_real_route_planner_inputs_exist",
            "preview_role": "records route-plan intent and conceptual segment scope only",
            "route_generation_unblocked_now": False,
            "requires_before_real_route_generation": _route_plan_preview_payload(route_kind)[
                "required_future_planner_inputs"
            ],
        },
        "claim_boundary": _claim_boundary(route_kind),
    }

    preview_written = False
    if not errors:
        save_json(preview_path, preview)
        preview_written = True

    summary_classification = SUMMARY_CLASSIFICATIONS.get(route_kind, "route_plan_preview_generation_failed")
    if errors:
        summary_classification = f"{route_kind}_route_plan_preview_generation_failed"
    summary = {
        "project_name": PROJECT_NAME,
        "classification": summary_classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "route_kind": route_kind,
        "dry_run": True,
        "route_plan_preview_generated": preview_written,
        "route_plan_preview_path": normalize_repo_relative(preview_path, repo_root),
        "route_plan_preview_classification": preview_classification if preview_written else None,
        "generated_from_candidate_route_contract": True,
        "generated_from_final_route_contract": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "blocked_real_planner_fields": _blocked_real_planner_fields(route_kind),
        "downstream_route_generation_unblock_plan": {
            "recommended_next_task": "task32_layer3_route_plan_to_real_route_generation_readiness_review",
            "reason": "Review final Layer 2 artifact and route planner input readiness before real A* generation.",
            "real_route_generation_unblocked_now": False,
        },
        "claim_boundary": _claim_boundary(route_kind),
        "errors": errors,
        "warnings": warnings,
    }
    return summary


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build RSLG-SLAM route plan dry-run previews.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--route-kind", required=True, help="Route kind to preview.")
    parser.add_argument("--candidate-route-contract", required=True, help="Candidate route contract JSON.")
    parser.add_argument("--dry-run", action="store_true", help="Required; generate dry-run preview only.")
    parser.add_argument("--output-dir", required=True, help="Route plan preview output directory.")
    parser.add_argument("--output-json", required=True, help="Route plan preview summary JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    summary = build_route_plan_preview(
        repo_root,
        args.scene_id,
        args.route_kind,
        repo_path(repo_root, args.candidate_route_contract),
        repo_path(repo_root, args.output_dir),
        dry_run=bool(args.dry_run),
    )
    save_json(repo_path(repo_root, args.output_json), summary)
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "error_count": summary["error_count"],
                "warning_count": summary["warning_count"],
                "output_json": args.output_json,
            },
            indent=2,
        )
    )
    return 0 if summary["classification"] in set(SUMMARY_CLASSIFICATIONS.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
