"""Layer 2 object interface dry-run and candidate package builder.

This module creates metadata-only RSLG-SLAM object-interface reports. It does
not resolve objects from current World Model Layer outputs, regenerate approach
geometry, use object centroids as direct goals, generate routes, launch runtime
systems, or call historical object-navigation scripts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .artifact_registry import ArtifactRegistry
from .common import PROJECT_NAME, normalize_repo_relative, repo_path, resolve_repo_root, save_json
from .validate_artifacts import (
    check_layer_naming,
    check_no_runtime_guard,
    check_no_world_model_rerun_guard,
    check_project_truth,
    check_stage_outputs_independence,
)


ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
ARTIFACT_ROLE = "object_interface_package"
DRY_RUN_CLASSIFICATION = "layer2_object_interface_artifact_dry_run_ready"
CANDIDATE_SUMMARY_CLASSIFICATION = "layer2_object_interface_candidate_package_generated"
BLOCKED_CLASSIFICATION = "layer2_object_interface_dry_run_blocked_manifest_validation_failed"
INVALID_REQUEST_CLASSIFICATION = "layer2_object_interface_invalid_request"

TASK28_EVIDENCE_ROOT_TEMPLATE = (
    "stage_outputs/stage1_generalization/{scene_id}/tasks/"
    "task28_layer2_object_interface_artifact_dry_run_and_candidate_package"
)

QUERY_CANDIDATE_FILENAME = "object_query_resolution_candidate_v0_1.json"
APPROACH_CANDIDATE_FILENAME = "object_approach_candidate_v0_1.json"
PACKAGE_CANDIDATE_FILENAME = "object_interface_package_candidate_v0_1.json"

QUERY_CANDIDATE_CLASSIFICATION = "object_query_resolution_candidate_artifact_generated"
APPROACH_CANDIDATE_CLASSIFICATION = "object_approach_candidate_artifact_generated"
PACKAGE_CANDIDATE_CLASSIFICATION = "object_interface_package_candidate_artifact_generated"

EXPECTED_QUERY = "curtain in room_14 on floor_2"
EXPECTED_OBJECT_ID = "obj_175"
EXPECTED_OBJECT_LABEL = "curtain"
EXPECTED_TARGET_FLOOR = "floor_2"
EXPECTED_TARGET_ROOM = "room_14"
EXPECTED_APPROACH_ID = "generated_ring_037"

REQUIRED_MANIFEST_KEYS = [
    "project_truth_manifest",
    "pipeline_contract_manifest",
    "layer_artifacts_manifest",
    "workspace_policy_manifest",
    "validated_milestones_manifest",
]

FUTURE_REQUIRED_LAYER1_INPUTS = [
    "object observations / semantic object evidence",
    "object-room association evidence",
    "object-floor association evidence",
    "object bounding/anchor evidence",
    "posed RGB-D World Model Layer evidence",
]

FUTURE_REQUIRED_LAYER2_INPUTS = [
    "stable occupancy map package",
    "room/floor topology",
    "approach feasibility / clearance evidence",
    "route contract dependencies",
]

FUTURE_GENERATED_ARTIFACTS = [
    {
        "artifact_name": "object_query_resolution_v0_1.json",
        "artifact_role": "future object query resolution formal artifact",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
    {
        "artifact_name": "object_approach_candidate_v0_1.json",
        "artifact_role": "future object approach formal artifact",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
    {
        "artifact_name": "object_interface_package_v0_1.json",
        "artifact_role": "future object interface package formal artifact",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
]

BLOCKED_FIELDS = [
    "current World Model Layer object observations",
    "current object-room association evidence",
    "current object-floor association evidence",
    "current object bounding/anchor evidence",
    "current posed RGB-D world-model evidence",
    "query ambiguity recomputation",
    "approach geometry regenerated from current Layer 1 or Layer 2 evidence",
    "approach feasibility / clearance revalidation",
    "final object interface artifact path under clean_rerun",
    "cross_floor_object candidate route contract finalization",
    "target room resolution finalization",
    "object approach append / approach-stage planning later",
]


def _manifest_validation(repo_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    registry = ArtifactRegistry(repo_root)
    registry_file_validation = registry.validate_manifest_files(required=True)
    loaded = registry.load_referenced_manifests(required=True) if registry_file_validation["ok"] else {}
    missing_loaded = [key for key in REQUIRED_MANIFEST_KEYS if key not in loaded]
    validation = {
        "ok": bool(registry_file_validation["ok"]) and not missing_loaded,
        "registry_summary": registry.summary(),
        "required_manifest_keys": REQUIRED_MANIFEST_KEYS,
        "missing_required_manifest_paths": registry_file_validation["missing_required"],
        "missing_required_manifest_keys": missing_loaded,
    }
    return validation, loaded


def _contract_validation(repo_root: Path) -> dict[str, Any]:
    checks = {
        "project_truth_check": check_project_truth(repo_root),
        "layer_naming_check": check_layer_naming(repo_root),
        "stage_outputs_independence_check": check_stage_outputs_independence(repo_root),
        "no_runtime_guard": check_no_runtime_guard(),
        "no_world_model_rerun_guard": check_no_world_model_rerun_guard(),
    }
    ok = (
        checks["project_truth_check"].get("project_truth_project_name") == PROJECT_NAME
        and checks["project_truth_check"].get("ok") is True
        and checks["layer_naming_check"].get("main_chain_has_world_model_layer") is True
        and checks["layer_naming_check"].get("canonical_step_1_name") == "World Model Layer"
        and checks["stage_outputs_independence_check"].get("stage_outputs_required") is False
        and checks["no_runtime_guard"].get("runtime_launched") is False
        and checks["no_runtime_guard"].get("ok") is True
        and checks["no_world_model_rerun_guard"].get("world_model_rerun") is False
        and checks["no_world_model_rerun_guard"].get("ok") is True
    )
    return {
        "ok": ok,
        "checks": checks,
        "required_guardrails": {
            "project_name": PROJECT_NAME,
            "layer_1_name": "World Model Layer",
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "runtime_launched": False,
            "world_model_rerun": False,
        },
    }


def _milestones(manifest: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    milestones = manifest.get("milestones", [])
    return [item for item in milestones if isinstance(item, Mapping)] if isinstance(milestones, list) else []


def _find_milestone(manifest: Mapping[str, Any], milestone_id: str) -> Mapping[str, Any] | None:
    for milestone in _milestones(manifest):
        if milestone.get("id") == milestone_id:
            return milestone
    return None


def _extract_validated_object_truth(manifest: Mapping[str, Any]) -> dict[str, Any]:
    object_record = None
    for milestone in _milestones(manifest):
        if milestone.get("object_id") == EXPECTED_OBJECT_ID or milestone.get("approach_candidate") == EXPECTED_APPROACH_ID:
            object_record = milestone
            break

    present = bool(
        object_record
        and object_record.get("query") == EXPECTED_QUERY
        and object_record.get("object_id") == EXPECTED_OBJECT_ID
        and object_record.get("object_label") == EXPECTED_OBJECT_LABEL
        and object_record.get("target_floor") == EXPECTED_TARGET_FLOOR
        and object_record.get("target_room") == EXPECTED_TARGET_ROOM
        and object_record.get("approach_candidate") == EXPECTED_APPROACH_ID
        and object_record.get("object_centroid_navigation_used") is False
    )
    return {
        "present": present,
        "source_manifest": "validated_milestones_manifest_v0_1.json",
        "milestone_id": object_record.get("id") if object_record else None,
        "status": "validated" if present else "missing_or_mismatched",
        "query": object_record.get("query") if object_record else None,
        "object_id": object_record.get("object_id") if object_record else None,
        "object_label": object_record.get("object_label") if object_record else None,
        "target_floor": object_record.get("target_floor") if object_record else None,
        "target_room": object_record.get("target_room") if object_record else None,
        "approach_candidate_id": object_record.get("approach_candidate") if object_record else None,
        "object_centroid_navigation_used": object_record.get("object_centroid_navigation_used") if object_record else None,
        "direct_object_centroid_goal_used": False,
        "object_approach_candidate_is_final": False,
        "lineage_note": "Validated milestone truth is used as a candidate seed; no current Layer 1 object rerun is performed.",
    }


def _extract_route_truth(manifest: Mapping[str, Any], scene_id: str) -> dict[str, Any]:
    scene_prefix = scene_id.split("-", maxsplit=1)[0]
    route = _find_milestone(manifest, f"{scene_prefix}_cross_floor_route_truth")
    transition = _find_milestone(manifest, f"{scene_prefix}_vt_1_transition_edge_truth")
    return {
        "source_manifest": "validated_milestones_manifest_v0_1.json",
        "route_milestone_id": route.get("id") if route else None,
        "transition_milestone_id": transition.get("id") if transition else None,
        "route_sequence": route.get("route", []) if route else [],
        "connector_ids": ["vt_1", "vc_vt_1"],
        "connector_id_primary": "vt_1",
        "connector_id_alias": "vc_vt_1",
        "source_floor": "floor_1",
        "target_floor": "floor_2",
        "transition_edge": transition.get("transition_edge") if transition else "vt_1_centerline_e001",
        "non_transition_edge": transition.get("not_transition_edge") if transition else "vt_1_centerline_e003",
        "route_context_rooms": ["room_2", "room_3", "room_7", "room_13", "room_14"],
        "route_generated": False,
    }


def _validated_object_truth_lineage(object_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "source_manifest": object_truth.get("source_manifest", "validated_milestones_manifest_v0_1.json"),
        "milestone_id": object_truth.get("milestone_id"),
        "status": object_truth.get("status"),
        "generated_from_validated_milestones_or_contracts": True,
        "generated_from_current_layer1_outputs": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }


def _cross_floor_object_route_context(route_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "route_kind": "cross_floor_object",
        "start": {"room": "room_2", "floor": "floor_1"},
        "intermediate_room": {"room": "room_3", "floor": "floor_1"},
        "connector": {
            "id": route_truth.get("connector_id_primary", "vt_1"),
            "alias": route_truth.get("connector_id_alias", "vc_vt_1"),
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "transition_edge": route_truth.get("transition_edge", "vt_1_centerline_e001"),
            "non_transition_edge": route_truth.get("non_transition_edge", "vt_1_centerline_e003"),
        },
        "floor_2_rooms": ["room_7", "room_13", "room_14"],
        "target_object": {"object_id": EXPECTED_OBJECT_ID, "object_label": EXPECTED_OBJECT_LABEL},
        "approach_candidate": {"approach_candidate_id": EXPECTED_APPROACH_ID},
        "route_sequence_context": route_truth.get("route_sequence", []),
        "real_route_generated": False,
        "executable_route_waypoints_generated": False,
    }


def _downstream_layer3_unblock_plan() -> dict[str, Any]:
    return {
        "cross_floor_object_candidate_route_contract_finalization": {
            "status": "blocked_until_integrated_layer2_candidate_artifact_review",
            "object_interface_role": "provides candidate target binding and approach candidate reference for later route-contract finalization",
            "completed_in_this_task": False,
        },
        "target_room_resolution": {
            "status": "candidate_seed_available_from_validated_truth_not_finalized",
            "object_interface_role": "binds obj_175 curtain query to room_14 on floor_2 for later final resolution",
            "completed_in_this_task": False,
        },
        "object_approach_append_or_approach_stage_planning_later": {
            "status": "blocked_until_approach_geometry_and_clearance_are_revalidated",
            "object_interface_role": "carries generated_ring_037 as a non-final approach candidate identifier",
            "completed_in_this_task": False,
        },
        "route_kind_dependency_scope": {
            "cross_floor_room": [
                "stable occupancy map package",
                "vertical connector artifact",
                "cross-floor topology artifact",
            ],
            "cross_floor_object": [
                "stable occupancy map package",
                "vertical connector artifact",
                "cross-floor topology artifact",
                "object query resolution artifact",
                "object approach candidate artifact",
            ],
        },
    }


def _claim_boundary(project_truth: Mapping[str, Any] | None = None, *, scope: str | None = None) -> dict[str, Any]:
    project_truth = project_truth or {}
    return {
        "scope": scope
        or "This task creates Layer 2 object-interface dry-run reports and non-final candidate artifacts.",
        "candidate_only_until_current_layer1_or_layer2_inputs_exist": True,
        "does_not_generate_final_object_interface_artifacts": True,
        "does_not_generate_real_object_query_resolution_from_current_world_model": True,
        "does_not_regenerate_object_approach_geometry": True,
        "does_not_revalidate_approach_feasibility": True,
        "does_not_use_object_centroid_as_direct_goal": True,
        "does_not_generate_stable_maps": True,
        "does_not_generate_vertical_connectors": True,
        "does_not_generate_real_routes": True,
        "does_not_generate_executable_waypoints": True,
        "does_not_generate_runtime_input_packages": True,
        "does_not_launch_runtime_systems": True,
        "does_not_claim_object_navigation_benchmark": True,
        "does_not_claim_physical_object_approach_success": True,
        "does_not_claim_amcl_success": True,
        "dataset_side_slam_or_localization_accuracy_claimed": project_truth.get("scene_input_contract", {}).get(
            "dataset_side_slam_or_localization_accuracy_claimed",
            False,
        ),
        "not_claimed": project_truth.get(
            "not_claimed_boundaries",
            [
                "dense reconstruction",
                "physical stair climbing",
                "footstep planning",
                "gait control",
                "contact dynamics",
                "AMCL success",
                "real robot stair climbing",
                "full object-navigation benchmark",
            ],
        ),
        "forbidden_world_model_sources": project_truth.get("forbidden_world_model_sources", []),
    }


def _base_report(
    repo_root: Path,
    scene_id: str | None,
    query: str | None,
    object_id: str | None,
    approach_candidate_id: str | None,
    *,
    dry_run: bool,
) -> dict[str, Any]:
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": None,
        "error_count": 0,
        "warning_count": 0,
        "dry_run": bool(dry_run),
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": ARTIFACT_ROLE,
        "scene_id": scene_id,
        "query": query,
        "object_id": object_id,
        "approach_candidate_id": approach_candidate_id,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "object_query_resolution_generated": False,
        "object_query_resolution_generated_from_current_world_model": False,
        "object_approach_candidate_generated": False,
        "approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "stable_map_generated": False,
        "vertical_connector_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
    }


def _request_errors(scene_id: str | None, query: str | None, object_id: str | None, approach_candidate_id: str | None) -> list[str]:
    errors: list[str] = []
    if not scene_id:
        errors.append("Pass --scene-id for object interface reports.")
    if query != EXPECTED_QUERY:
        errors.append(f"query must be {EXPECTED_QUERY!r} for this validated object-interface candidate.")
    if object_id != EXPECTED_OBJECT_ID:
        errors.append(f"object-id must be {EXPECTED_OBJECT_ID!r} for this validated object-interface candidate.")
    if approach_candidate_id != EXPECTED_APPROACH_ID:
        errors.append(
            f"approach-candidate-id must be {EXPECTED_APPROACH_ID!r} for this validated object-interface candidate."
        )
    return errors


def _planned_candidate_package_preview(scene_id: str) -> dict[str, Any]:
    base = TASK28_EVIDENCE_ROOT_TEMPLATE.format(scene_id=scene_id)
    return {
        "candidate_only": True,
        "not_written_as_final_artifact": True,
        "candidate_output_dir": f"{base}/reports/candidate_artifacts",
        "candidate_artifacts": [
            QUERY_CANDIDATE_FILENAME,
            APPROACH_CANDIDATE_FILENAME,
            PACKAGE_CANDIDATE_FILENAME,
        ],
        "future_final_artifacts_expected_not_generated": [
            "object_query_resolution_v0_1.json",
            "object_approach_candidate_v0_1.json",
            "object_interface_package_v0_1.json",
        ],
        "future_final_artifact_root": f"stage_outputs/stage1_generalization/{scene_id}/clean_rerun/formal_artifacts/layer2",
    }


def _candidate_output_dir_allowed(repo_root: Path, scene_id: str, output_dir: Path) -> tuple[bool, str]:
    rel_path = normalize_repo_relative(output_dir, repo_root)
    task28_root = TASK28_EVIDENCE_ROOT_TEMPLATE.format(scene_id=scene_id)
    allowed_prefix = f"{task28_root}/reports/candidate_artifacts"
    allowed_suffixes = ("/reports/candidate_artifacts", "/reports/layer2_candidate_artifacts")
    if "clean_rerun" in Path(rel_path).parts:
        return False, "candidate output directory must not be under clean_rerun"
    if rel_path == allowed_prefix or rel_path.startswith(f"{allowed_prefix}/"):
        return True, rel_path
    generic_tasks_prefix = f"stage_outputs/stage1_generalization/{scene_id}/tasks/"
    if rel_path.startswith(generic_tasks_prefix) and rel_path.endswith(allowed_suffixes):
        return True, rel_path
    return False, (
        f"candidate output directory must be under {allowed_prefix} or another "
        "task evidence reports/candidate_artifacts or reports/layer2_candidate_artifacts directory"
    )


def build_dry_run_report(
    repo_root: Path,
    scene_id: str | None,
    query: str | None,
    object_id: str | None,
    approach_candidate_id: str | None,
    dry_run: bool,
) -> dict[str, Any]:
    report = _base_report(repo_root, scene_id, query, object_id, approach_candidate_id, dry_run=dry_run)
    warnings: list[str] = []
    errors = _request_errors(scene_id, query, object_id, approach_candidate_id)

    manifest_validation, manifests = _manifest_validation(repo_root)
    report["manifest_validation"] = manifest_validation
    if manifest_validation.get("ok") is not True:
        errors.append("Manifest validation failed.")

    contract_validation = _contract_validation(repo_root) if manifest_validation.get("ok") else {"ok": False}
    report["contract_validation"] = contract_validation
    if contract_validation.get("ok") is not True:
        errors.append("Project contract validation failed.")

    validated_manifest = manifests.get("validated_milestones_manifest", {}) if manifest_validation.get("ok") else {}
    project_truth = manifests.get("project_truth_manifest", {}) if manifest_validation.get("ok") else {}
    object_truth = _extract_validated_object_truth(validated_manifest)
    route_truth = _extract_route_truth(validated_manifest, scene_id or "unknown_scene")
    if object_truth.get("present") is not True:
        errors.append("Validated object truth for obj_175/generated_ring_037 is missing or mismatched.")

    classification = DRY_RUN_CLASSIFICATION if not errors else (
        INVALID_REQUEST_CLASSIFICATION if any("must be" in error or "Pass --scene-id" in error for error in errors) else BLOCKED_CLASSIFICATION
    )
    report.update(
        {
            "classification": classification,
            "error_count": len(errors),
            "warning_count": len(warnings),
            "validated_object_truth_used": object_truth,
            "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
            "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
            "future_generated_artifacts": FUTURE_GENERATED_ARTIFACTS,
            "planned_candidate_package_preview": _planned_candidate_package_preview(scene_id or "unknown_scene"),
            "blocked_fields": BLOCKED_FIELDS,
            "cross_floor_object_route_context": _cross_floor_object_route_context(route_truth),
            "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
            "claim_boundary": _claim_boundary(project_truth),
            "errors": errors,
            "warnings": warnings,
        }
    )
    return report


def build_query_resolution_candidate(scene_id: str, object_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_name": "rslg_object_query_resolution_candidate",
        "schema_version": "0.1",
        "classification": QUERY_CANDIDATE_CLASSIFICATION,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": "object_query_resolution_candidate",
        "scene_id": scene_id,
        "query": EXPECTED_QUERY,
        "is_candidate": True,
        "is_final_formal_artifact": False,
        "generated_from_validated_milestones_or_contracts": True,
        "generated_from_current_layer1_outputs": False,
        "historical_stage_outputs_required": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "object_query_resolution_generated_from_current_world_model": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "validated_object_truth_lineage": _validated_object_truth_lineage(object_truth),
        "candidate_payload": {
            "resolved_object_id": EXPECTED_OBJECT_ID,
            "object_label": EXPECTED_OBJECT_LABEL,
            "target_floor": EXPECTED_TARGET_FLOOR,
            "target_room": EXPECTED_TARGET_ROOM,
            "query_status": "candidate_resolution_from_validated_truth",
            "ambiguity_status": "not_recomputed_in_this_task",
            "source_status": "validated_milestone_truth_not_current_world_model_rerun",
        },
        "blocked_until_layer1_or_layer2_inputs": BLOCKED_FIELDS,
        "downstream_layer3_unblock_role": _downstream_layer3_unblock_plan(),
        "claim_boundary": _claim_boundary(scope="Non-final object query resolution candidate from validated milestone truth."),
    }


def build_approach_candidate(scene_id: str, object_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_name": "rslg_object_approach_candidate",
        "schema_version": "0.1",
        "classification": APPROACH_CANDIDATE_CLASSIFICATION,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": "object_approach_candidate",
        "scene_id": scene_id,
        "object_id": EXPECTED_OBJECT_ID,
        "approach_candidate_id": EXPECTED_APPROACH_ID,
        "is_candidate": True,
        "is_final_formal_artifact": False,
        "generated_from_validated_milestones_or_contracts": True,
        "generated_from_current_layer1_outputs": False,
        "historical_stage_outputs_required": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "validated_object_truth_lineage": _validated_object_truth_lineage(object_truth),
        "candidate_payload": {
            "object_id": EXPECTED_OBJECT_ID,
            "object_label": EXPECTED_OBJECT_LABEL,
            "target_floor": EXPECTED_TARGET_FLOOR,
            "target_room": EXPECTED_TARGET_ROOM,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
            "approach_status": "candidate_from_validated_truth_not_revalidated",
            "centroid_as_goal": False,
            "requires_future_clearance_or_runtime_probe": True,
            "source_status": "validated_milestone_truth_not_current_layer1_regeneration",
            "approach_geometry_status": "candidate_identifier_only_no_regenerated_geometry",
        },
        "blocked_until_layer1_or_layer2_inputs": BLOCKED_FIELDS,
        "downstream_layer3_unblock_role": _downstream_layer3_unblock_plan(),
        "claim_boundary": _claim_boundary(scope="Non-final object approach candidate identifier; no geometry is regenerated."),
    }


def build_interface_package_candidate(scene_id: str, route_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_name": "rslg_object_interface_package_candidate",
        "schema_version": "0.1",
        "classification": PACKAGE_CANDIDATE_CLASSIFICATION,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": "object_interface_package_candidate",
        "scene_id": scene_id,
        "query": EXPECTED_QUERY,
        "object_id": EXPECTED_OBJECT_ID,
        "approach_candidate_id": EXPECTED_APPROACH_ID,
        "is_candidate": True,
        "is_final_formal_artifact": False,
        "generated_from_validated_milestones_or_contracts": True,
        "generated_from_current_layer1_outputs": False,
        "historical_stage_outputs_required": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "object_query_resolution_generated_from_current_world_model": False,
        "approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "stable_map_generated": False,
        "vertical_connector_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "candidate_artifact_references": {
            "object_query_resolution_candidate": QUERY_CANDIDATE_FILENAME,
            "object_approach_candidate": APPROACH_CANDIDATE_FILENAME,
        },
        "cross_floor_object_route_context": _cross_floor_object_route_context(route_truth),
        "blocked_until_layer1_or_layer2_inputs": BLOCKED_FIELDS,
        "downstream_layer3_unblock_role": _downstream_layer3_unblock_plan(),
        "claim_boundary": _claim_boundary(scope="Non-final object interface package candidate for later Layer 3 review."),
    }


def generate_candidate_package(
    repo_root: Path,
    scene_id: str | None,
    query: str | None,
    object_id: str | None,
    approach_candidate_id: str | None,
    output_dir: Path,
) -> dict[str, Any]:
    summary = _base_report(repo_root, scene_id, query, object_id, approach_candidate_id, dry_run=False)
    warnings: list[str] = []
    errors = _request_errors(scene_id, query, object_id, approach_candidate_id)
    output_dir_detail = normalize_repo_relative(output_dir, repo_root)

    summary.update(
        {
            "generate_candidate": True,
            "candidate_package_generated": False,
            "candidate_artifact_paths": [],
            "candidate_artifact_classifications": [],
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
        }
    )

    if scene_id:
        output_dir_ok, output_dir_detail = _candidate_output_dir_allowed(repo_root, scene_id, output_dir)
        if not output_dir_ok:
            errors.append(output_dir_detail)

    dry_run_reference = build_dry_run_report(
        repo_root, scene_id, query, object_id, approach_candidate_id, dry_run=True
    )
    if dry_run_reference.get("classification") != DRY_RUN_CLASSIFICATION:
        errors.extend(str(error) for error in dry_run_reference.get("errors", []))
        if not errors:
            errors.append("Dry-run guardrail reference did not reach ready classification.")

    manifest_validation, manifests = _manifest_validation(repo_root)
    validated_manifest = manifests.get("validated_milestones_manifest", {}) if manifest_validation.get("ok") else {}
    project_truth = manifests.get("project_truth_manifest", {}) if manifest_validation.get("ok") else {}
    object_truth = _extract_validated_object_truth(validated_manifest)
    route_truth = _extract_route_truth(validated_manifest, scene_id or "unknown_scene")
    if manifest_validation.get("ok") is not True:
        errors.append("Manifest validation failed.")
    if object_truth.get("present") is not True:
        errors.append("Validated object truth for obj_175/generated_ring_037 is missing or mismatched.")

    if errors:
        summary.update(
            {
                "classification": BLOCKED_CLASSIFICATION,
                "error_count": len(errors),
                "warning_count": len(warnings),
                "output_dir": output_dir_detail,
                "dry_run_reference_classification": dry_run_reference.get("classification"),
                "blocked_finalization_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(project_truth),
                "errors": errors,
                "warnings": warnings,
            }
        )
        return summary

    output_dir.mkdir(parents=True, exist_ok=True)
    candidates = {
        QUERY_CANDIDATE_FILENAME: build_query_resolution_candidate(str(scene_id), object_truth),
        APPROACH_CANDIDATE_FILENAME: build_approach_candidate(str(scene_id), object_truth),
        PACKAGE_CANDIDATE_FILENAME: build_interface_package_candidate(str(scene_id), route_truth),
    }
    candidate_paths: list[str] = []
    candidate_classifications: list[str] = []
    for filename, candidate in candidates.items():
        path = output_dir / filename
        save_json(path, candidate)
        candidate_paths.append(normalize_repo_relative(path, repo_root))
        candidate_classifications.append(str(candidate["classification"]))

    summary.update(
        {
            "classification": CANDIDATE_SUMMARY_CLASSIFICATION,
            "error_count": 0,
            "warning_count": len(warnings),
            "artifact_layer": ARTIFACT_LAYER,
            "scene_id": scene_id,
            "query": query,
            "object_id": object_id,
            "approach_candidate_id": approach_candidate_id,
            "generate_candidate": True,
            "candidate_package_generated": True,
            "output_dir": output_dir_detail,
            "candidate_artifact_paths": candidate_paths,
            "candidate_artifact_classifications": candidate_classifications,
            "dry_run_reference_classification": dry_run_reference.get("classification"),
            "final_formal_artifacts_generated": False,
            "object_query_resolution_generated": False,
            "object_approach_candidate_generated": False,
            "object_centroid_navigation_used": False,
            "direct_object_centroid_goal_used": False,
            "stable_map_generated": False,
            "vertical_connector_generated": False,
            "real_route_generated": False,
            "runtime_artifacts_generated": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "blocked_finalization_fields": BLOCKED_FIELDS,
            "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
            "claim_boundary": _claim_boundary(project_truth),
            "errors": [],
            "warnings": warnings,
        }
    )
    return summary


def _placeholder_summary(repo_root: Path, dry_run: bool) -> dict[str, Any]:
    return {
        "project_name": PROJECT_NAME,
        "module": "build_object_interfaces",
        "repo_root": repo_root.as_posix(),
        "layer": ARTIFACT_LAYER,
        "current_role": "Layer 2 object-interface dry-run and non-final candidate package builder.",
        "future_role": "Build final object query and approach interfaces from current formal artifacts.",
        "business_logic_migrated": False,
        "real_logic_status": "candidate_metadata_only_no_object_navigation",
        "dry_run": bool(dry_run),
        "runtime_launched": False,
        "old_scripts_called": False,
        "large_artifacts_written": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Plan or generate non-final RSLG-SLAM Layer 2 object-interface candidates."
    )
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", default=None, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--query", default=None, help="Object query text.")
    parser.add_argument("--object-id", default=None, help="Validated object id.")
    parser.add_argument("--approach-candidate-id", default=None, help="Validated approach candidate id.")
    parser.add_argument("--dry-run", action="store_true", help="Plan only; do not generate candidate artifacts.")
    parser.add_argument(
        "--generate-candidate",
        action="store_true",
        help="Write non-final object-interface candidate artifacts under the task28 evidence directory.",
    )
    parser.add_argument("--output-dir", default=None, help="Candidate artifact output directory.")
    parser.add_argument("--output-json", default=None, help="Write the report JSON to this path.")
    parser.add_argument("--describe", action="store_true", help="Describe safe builder behavior without writing artifacts.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    if args.describe or (args.dry_run and not args.scene_id and not args.output_json and not args.generate_candidate):
        print(json.dumps(_placeholder_summary(repo_root, dry_run=bool(args.dry_run)), indent=2, sort_keys=False))
        return 0

    if args.dry_run and args.generate_candidate:
        result = _base_report(
            repo_root, args.scene_id, args.query, args.object_id, args.approach_candidate_id, dry_run=False
        )
        result.update(
            {
                "classification": INVALID_REQUEST_CLASSIFICATION,
                "error_count": 1,
                "generate_candidate": True,
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": FUTURE_GENERATED_ARTIFACTS,
                "planned_candidate_package_preview": _planned_candidate_package_preview(args.scene_id or "unknown_scene"),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": ["Use either --dry-run or --generate-candidate, not both."],
                "warnings": [],
            }
        )
    elif args.generate_candidate:
        if not args.output_dir:
            result = _base_report(
                repo_root, args.scene_id, args.query, args.object_id, args.approach_candidate_id, dry_run=False
            )
            result.update(
                {
                    "classification": INVALID_REQUEST_CLASSIFICATION,
                    "error_count": 1,
                    "generate_candidate": True,
                    "candidate_package_generated": False,
                    "candidate_artifact_paths": [],
                    "candidate_artifact_classifications": [],
                    "blocked_finalization_fields": BLOCKED_FIELDS,
                    "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                    "claim_boundary": _claim_boundary(),
                    "errors": ["Pass --output-dir with --generate-candidate."],
                    "warnings": [],
                }
            )
        else:
            result = generate_candidate_package(
                repo_root,
                args.scene_id,
                args.query,
                args.object_id,
                args.approach_candidate_id,
                repo_path(repo_root, args.output_dir),
            )
    elif args.dry_run:
        result = build_dry_run_report(
            repo_root, args.scene_id, args.query, args.object_id, args.approach_candidate_id, dry_run=True
        )
    else:
        result = _base_report(
            repo_root, args.scene_id, args.query, args.object_id, args.approach_candidate_id, dry_run=False
        )
        result.update(
            {
                "classification": INVALID_REQUEST_CLASSIFICATION,
                "error_count": 1,
                "generate_candidate": False,
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": FUTURE_GENERATED_ARTIFACTS,
                "planned_candidate_package_preview": _planned_candidate_package_preview(args.scene_id or "unknown_scene"),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": ["Pass --dry-run, --generate-candidate, or --describe."],
                "warnings": [],
            }
        )

    if args.output_json:
        save_json(repo_path(repo_root, args.output_json), result)
        output_json = args.output_json
    else:
        output_json = None
    print(
        json.dumps(
            {
                "classification": result["classification"],
                "error_count": result["error_count"],
                "warning_count": result["warning_count"],
                "output_json": output_json,
            },
            indent=2,
            sort_keys=False,
        )
    )
    return 0 if result["classification"] in {DRY_RUN_CLASSIFICATION, CANDIDATE_SUMMARY_CLASSIFICATION} else 1


if __name__ == "__main__":
    raise SystemExit(main())
