"""Manifest-aware route contract planner/stub builder for RSLG-SLAM.

This wrapper does not build real routes, call historical tools, migrate
route-contract business logic, rerun the World Model Layer, or launch runtime
systems. It plans the future route-contract shape from docs/manifests and can
write small task-evidence route contract stubs explicitly marked as non-final.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

from .artifact_registry import ArtifactRegistry
from .common import PROJECT_NAME, load_json, normalize_repo_relative, repo_path, resolve_repo_root, save_json
from .build_object_interfaces import (
    EXPECTED_APPROACH_ID,
    EXPECTED_OBJECT_ID,
    EXPECTED_OBJECT_LABEL,
    EXPECTED_QUERY,
    EXPECTED_TARGET_FLOOR,
    EXPECTED_TARGET_ROOM,
)
from .layer2_candidate_integration_review import CANDIDATE_FILENAMES, EXPECTED_CLASSIFICATIONS
from .validate_artifacts import (
    check_layer_naming,
    check_no_runtime_guard,
    check_no_world_model_rerun_guard,
    check_project_truth,
    check_stage_outputs_independence,
)


SUPPORTED_ROUTE_KINDS = {
    "cross_floor_object",
    "cross_floor_room",
    "object_approach",
    "generic",
}

STUB_SUPPORTED_ROUTE_KINDS = {
    "cross_floor_object",
    "cross_floor_room",
}

ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"
ARTIFACT_ROLE = "candidate_route_contract"
SCHEMA_NAME = "rslg_candidate_route_contract"
SCHEMA_VERSION = "0.1"

ROOM_ROUTE_SEQUENCE = ["room_2", "room_3", "room_7", "room_13", "room_14"]
SOURCE_FLOOR = "floor_1"
TARGET_FLOOR = "floor_2"
START_ROOM = "room_2"
TARGET_ROOM = "room_14"
CONNECTOR_ID = "vt_1"
CONNECTOR_ALIAS = "vc_vt_1"
TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"

ROOM_DEPENDENCY_KEYS = ["vertical_connector", "connector_graph", "cross_floor_topology", "stable_map_package"]
OBJECT_DEPENDENCY_KEYS = ROOM_DEPENDENCY_KEYS + [
    "object_query_resolution",
    "object_approach",
    "object_interface_package",
]

CANDIDATE_CONTRACT_FILES = {
    "cross_floor_room": "cross_floor_room_candidate_route_contract_v0_1.json",
    "cross_floor_object": "cross_floor_object_candidate_route_contract_v0_1.json",
}

CANDIDATE_CONTRACT_CLASSIFICATIONS = {
    "cross_floor_room": "cross_floor_room_candidate_route_contract_generated",
    "cross_floor_object": "cross_floor_object_candidate_route_contract_generated",
}

CANDIDATE_SUMMARY_CLASSIFICATIONS = {
    "cross_floor_room": "cross_floor_room_candidate_route_contract_generation_completed",
    "cross_floor_object": "cross_floor_object_candidate_route_contract_generation_completed",
}

REQUIRED_MANIFEST_KEYS = [
    "project_truth_manifest",
    "pipeline_contract_manifest",
    "layer_artifacts_manifest",
    "workspace_policy_manifest",
    "validated_milestones_manifest",
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
            "runtime_launched": False,
            "world_model_rerun": False,
        },
    }


def _milestones(manifest: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    milestones = manifest.get("milestones", [])
    if isinstance(milestones, list):
        return [item for item in milestones if isinstance(item, Mapping)]
    return []


def _find_milestone(manifest: Mapping[str, Any], milestone_id: str) -> Mapping[str, Any] | None:
    for milestone in _milestones(manifest):
        if milestone.get("id") == milestone_id:
            return milestone
    return None


def _extract_route_truth(manifest: Mapping[str, Any], scene_id: str) -> dict[str, Any]:
    route = _find_milestone(manifest, f"{scene_id.split('-')[0]}_cross_floor_route_truth")
    transition = _find_milestone(manifest, f"{scene_id.split('-')[0]}_vt_1_transition_edge_truth")
    route_sequence = route.get("route") if route else None
    has_route = isinstance(route_sequence, list) and bool(route_sequence)
    has_transition = bool(transition and transition.get("transition_edge") and transition.get("not_transition_edge"))
    return {
        "present": has_route and has_transition,
        "source_manifest": "validated_milestones_manifest_v0_1.json",
        "route_milestone_id": route.get("id") if route else None,
        "transition_milestone_id": transition.get("id") if transition else None,
        "status": "validated" if has_route and has_transition else "missing",
        "route": route_sequence if has_route else [],
        "transition_edge": transition.get("transition_edge") if transition else None,
        "non_transition_edge": transition.get("not_transition_edge") if transition else None,
        "centerline_nodes": transition.get("centerline_nodes", []) if transition else [],
    }


def _extract_object_truth(manifest: Mapping[str, Any]) -> dict[str, Any]:
    object_record = None
    for milestone in _milestones(manifest):
        if milestone.get("object_id") == "obj_175" or milestone.get("approach_candidate") == "generated_ring_037":
            object_record = milestone
            break
    present = bool(
        object_record
        and object_record.get("query")
        and object_record.get("object_id")
        and object_record.get("approach_candidate")
        and object_record.get("object_centroid_navigation_used") is False
    )
    return {
        "present": present,
        "source_manifest": "validated_milestones_manifest_v0_1.json",
        "milestone_id": object_record.get("id") if object_record else None,
        "status": "validated" if present else "missing",
        "query": object_record.get("query") if object_record else None,
        "object_id": object_record.get("object_id") if object_record else None,
        "object_label": object_record.get("object_label") if object_record else None,
        "target_room": object_record.get("target_room") if object_record else None,
        "target_floor": object_record.get("target_floor") if object_record else None,
        "approach_candidate": object_record.get("approach_candidate") if object_record else None,
        "object_centroid_navigation_used": object_record.get("object_centroid_navigation_used")
        if object_record
        else None,
    }


def _layer_by_name(layer_manifest: Mapping[str, Any], layer_name: str) -> Mapping[str, Any]:
    for layer in layer_manifest.get("layers", []):
        if isinstance(layer, Mapping) and layer.get("layer") == layer_name:
            return layer
    return {}


def _expected_inputs(layer_manifest: Mapping[str, Any]) -> tuple[list[str], list[dict[str, Any]]]:
    layer0 = _layer_by_name(layer_manifest, "Layer 0: Input Layer")
    layer1 = _layer_by_name(layer_manifest, "Layer 1: World Model Layer")
    layer2 = _layer_by_name(layer_manifest, "Layer 2: Formal Artifact Layer")
    return (
        [
            "Layer 0: Input Layer",
            "Layer 1: World Model Layer",
            "Layer 2: Formal Artifact Layer",
        ],
        [
            {
                "layer": "Layer 0: Input Layer",
                "artifact_class": "scene inputs",
                "items": layer0.get("inputs", ["RGB images", "depth images", "provided camera poses"]),
                "required_for_dry_run": False,
                "existence_checked": False,
            },
            {
                "layer": "Layer 1: World Model Layer",
                "artifact_class": "world model outputs",
                "items": layer1.get("outputs", []),
                "required_for_dry_run": False,
                "existence_checked": False,
            },
            {
                "layer": "Layer 2: Formal Artifact Layer",
                "artifact_class": "formal routing inputs",
                "items": layer2.get("outputs", []),
                "required_for_dry_run": False,
                "existence_checked": False,
            },
        ],
    )


def _future_output_artifacts(scene_id: str, route_kind: str) -> list[dict[str, Any]]:
    route_file_by_kind = {
        "cross_floor_object": "object_level_cross_floor_route_contract_v0_1.json",
        "cross_floor_room": "room_level_cross_floor_route_contract_v0_1.json",
        "object_approach": "object_approach_route_contract_v0_1.json",
        "generic": "generic_route_contract_v0_1.json",
    }
    future_path = (
        "stage_outputs/stage1_generalization/"
        f"{scene_id}/clean_rerun/routes/{route_file_by_kind[route_kind]}"
    )
    artifacts = [
        {
            "artifact": "route contract",
            "future_generated_output_path": future_path,
            "required_for_dry_run": False,
            "existence_checked": False,
            "would_be_generated_by_future_task": True,
        }
    ]
    if route_kind == "cross_floor_object":
        artifacts.append(
            {
                "artifact": "room-level route contract dependency",
                "future_generated_output_path": (
                    "stage_outputs/stage1_generalization/"
                    f"{scene_id}/clean_rerun/routes/room_level_cross_floor_route_contract_v0_1.json"
                ),
                "required_for_dry_run": False,
                "existence_checked": False,
                "would_be_generated_by_future_task": True,
            }
        )
    return artifacts


def _future_builder_steps(route_kind: str) -> list[str]:
    steps = [
        "Load project truth, pipeline contract, layer artifacts, workspace policy, and validated milestones manifests.",
        "Confirm route-contract builder remains dry-run only for task25i.",
        "Resolve expected Layer 2 formal routing inputs without checking generated output paths.",
    ]
    if route_kind in {"cross_floor_object", "cross_floor_room"}:
        steps.append("Use validated cross-floor room sequence and vt_1 transition-edge truth as future contract seed.")
    if route_kind in {"cross_floor_object", "object_approach"}:
        steps.append("Use validated obj_175 curtain approach-candidate truth as future object-interface seed.")
    steps.extend(
        [
            "Plan future Layer 3 route-contract JSON path under clean_rerun without writing it.",
            "Keep claim boundaries limited to manifest-backed route/interface planning.",
        ]
    )
    return steps


def _stub_expected_future_inputs(route_kind: str) -> list[str]:
    inputs = [
        "canonical Layer 2 route graph derived from World Model Layer outputs",
        "validated vertical connector artifact with vt_1_centerline_e001 transition semantics",
        "stable occupancy maps generated by a future Layer 2 builder",
        "route request schema for room-level cross-floor planning",
    ]
    if route_kind == "cross_floor_object":
        inputs.extend(
            [
                "canonical object candidate index derived from World Model Layer object records",
                "validated object approach candidate artifact for generated_ring_037",
                "object-level route request schema for obj_175 curtain query",
            ]
        )
    return inputs


def _stub_expected_future_outputs(route_kind: str) -> list[str]:
    outputs = [
        "final room-level route contract JSON",
        "validated graph route sequence with connector transition provenance",
        "A* route request inputs after stable occupancy maps exist",
    ]
    if route_kind == "cross_floor_object":
        outputs.extend(
            [
                "final object-level route contract JSON",
                "validated object approach interface binding for generated_ring_037",
            ]
        )
    return outputs


def _stub_expected_future_object_inputs() -> list[str]:
    return [
        "canonical object interface artifact for obj_175",
        "object approach candidate geometry generated by a future canonical builder",
        "room-level route contract dependency for room_14 on floor_2",
    ]


def _stub_expected_future_object_outputs() -> list[str]:
    return [
        "final object-level route contract with approach binding",
        "runtime input export candidate for object navigation validation",
    ]


def _missing_truth_classification(route_kind: str, route_truth: Mapping[str, Any], object_truth: Mapping[str, Any]) -> bool:
    route_required = route_kind in {"cross_floor_object", "cross_floor_room"}
    object_required = route_kind in {"cross_floor_object", "object_approach"}
    return (route_required and not route_truth.get("present")) or (object_required and not object_truth.get("present"))


def build_dry_run_plan(repo_root: Path, scene_id: str, route_kind: str, dry_run: bool) -> dict[str, Any]:
    if route_kind not in SUPPORTED_ROUTE_KINDS:
        return {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": scene_id,
            "route_kind": route_kind,
            "dry_run": bool(dry_run),
            "classification": "dry_run_unsupported_route_kind",
            "supported_route_kinds": sorted(SUPPORTED_ROUTE_KINDS),
            "business_logic_migrated": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
        }

    manifest_validation, manifests = _manifest_validation(repo_root)
    if not manifest_validation["ok"]:
        return {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": scene_id,
            "route_kind": route_kind,
            "dry_run": bool(dry_run),
            "classification": "dry_run_blocked_manifest_validation_failed",
            "manifest_validation": manifest_validation,
            "business_logic_migrated": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
        }

    contract_validation = _contract_validation(repo_root)
    if not contract_validation["ok"]:
        return {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": scene_id,
            "route_kind": route_kind,
            "dry_run": bool(dry_run),
            "classification": "dry_run_blocked_manifest_validation_failed",
            "manifest_validation": manifest_validation,
            "contract_validation": contract_validation,
            "business_logic_migrated": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
        }

    validated_milestones = manifests["validated_milestones_manifest"]
    layer_artifacts = manifests["layer_artifacts_manifest"]
    project_truth = manifests["project_truth_manifest"]
    expected_input_layers, expected_input_artifacts = _expected_inputs(layer_artifacts)
    route_truth = _extract_route_truth(validated_milestones, scene_id)
    object_truth = _extract_object_truth(validated_milestones)
    missing_milestone_truth = _missing_truth_classification(route_kind, route_truth, object_truth)
    classification = (
        "dry_run_blocked_missing_validated_milestone_truth"
        if missing_milestone_truth
        else "route_contract_dry_run_ready"
    )

    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "scene_id": scene_id,
        "route_kind": route_kind,
        "dry_run": True,
        "classification": classification,
        "business_logic_migrated": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "manifest_validation": manifest_validation,
        "contract_validation": contract_validation,
        "expected_input_layers": expected_input_layers,
        "expected_input_artifacts": expected_input_artifacts,
        "expected_output_layer": "Layer 3: Navigation Interface Layer",
        "expected_output_artifacts": _future_output_artifacts(scene_id, route_kind),
        "route_truth_from_validated_milestones": route_truth,
        "object_truth_from_validated_milestones": object_truth,
        "missing_active_artifacts": [],
        "future_builder_steps": _future_builder_steps(route_kind),
        "blocked_by_missing_generated_outputs": False,
        "blocked_by_missing_validated_milestone_truth": missing_milestone_truth,
        "claim_boundary": {
            "not_claimed": project_truth.get("not_claimed_boundaries", []),
            "dataset_side_slam_or_localization_accuracy_claimed": project_truth.get("scene_input_contract", {}).get(
                "dataset_side_slam_or_localization_accuracy_claimed"
            ),
            "world_model_source_prohibitions": project_truth.get("forbidden_world_model_sources", []),
            "route_contract_scope": "Manifest-aware dry-run planning only; no route generation or runtime validation.",
        },
    }


def build_route_contract_stub(repo_root: Path, scene_id: str, route_kind: str) -> dict[str, Any]:
    if route_kind not in STUB_SUPPORTED_ROUTE_KINDS:
        return {
            "schema_name": "rslg_route_contract_stub",
            "schema_version": "0.1",
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": scene_id,
            "route_kind": route_kind,
            "classification": "route_contract_stub_unsupported_route_kind",
            "supported_stub_route_kinds": sorted(STUB_SUPPORTED_ROUTE_KINDS),
            "is_stub": True,
            "is_final_navigation_artifact": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "business_logic_migrated": False,
            "real_astar_route_generated": False,
            "stable_map_generated": False,
            "old_scripts_called": False,
        }

    dry_run_plan = build_dry_run_plan(repo_root, scene_id, route_kind, dry_run=True)
    dry_run_classification = dry_run_plan.get("classification")
    if dry_run_classification != "route_contract_dry_run_ready":
        blocked_classification = (
            "route_contract_stub_blocked_missing_validated_milestone_truth"
            if dry_run_plan.get("blocked_by_missing_validated_milestone_truth") is True
            else "route_contract_stub_blocked_by_dry_run"
        )
        return {
            "schema_name": "rslg_route_contract_stub",
            "schema_version": "0.1",
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": scene_id,
            "route_kind": route_kind,
            "classification": blocked_classification,
            "dry_run_plan_classification": dry_run_classification,
            "is_stub": True,
            "is_final_navigation_artifact": False,
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "business_logic_migrated": False,
            "real_astar_route_generated": False,
            "stable_map_generated": False,
            "old_scripts_called": False,
        }

    route_truth = dry_run_plan["route_truth_from_validated_milestones"]
    stub = {
        "schema_name": "rslg_route_contract_stub",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": "route_contract_stub_generated",
        "stub_type": route_kind,
        "scene_id": scene_id,
        "artifact_layer": "Layer 3: Navigation Interface Layer",
        "artifact_role": "route_contract_stub",
        "generated_from": "validated_milestones_manifest_v0_1.json",
        "dry_run_plan_classification": dry_run_classification,
        "is_stub": True,
        "is_final_navigation_artifact": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "business_logic_migrated": False,
        "real_astar_route_generated": False,
        "stable_map_generated": False,
        "old_scripts_called": False,
        "route_sequence": route_truth["route"],
        "transition_edge_id": route_truth["transition_edge"],
        "not_transition_edge_id": route_truth["non_transition_edge"],
        "expected_future_inputs": _stub_expected_future_inputs(route_kind),
        "expected_future_outputs": _stub_expected_future_outputs(route_kind),
        "claim_boundary": "graph-level route contract stub only, not physical stair climbing or runtime validation",
        "next_required_builder": "task25k_route_contract_stub_schema_validator",
    }
    if route_kind == "cross_floor_object":
        object_truth = dry_run_plan["object_truth_from_validated_milestones"]
        stub.update(
            {
                "query": object_truth["query"],
                "object_id": object_truth["object_id"],
                "object_label": object_truth["object_label"],
                "target_floor": object_truth["target_floor"],
                "target_room": object_truth["target_room"],
                "approach_candidate": object_truth["approach_candidate"],
                "object_centroid_navigation_used": object_truth["object_centroid_navigation_used"],
                "object_approach_is_stub": True,
                "real_object_approach_generated": False,
                "expected_future_object_inputs": _stub_expected_future_object_inputs(),
                "expected_future_object_outputs": _stub_expected_future_object_outputs(),
                "claim_boundary": "object-level route contract stub only, not full object-navigation benchmark",
            }
        )
    return stub


def _candidate_dependency_keys(route_kind: str) -> list[str]:
    return list(OBJECT_DEPENDENCY_KEYS if route_kind == "cross_floor_object" else ROOM_DEPENDENCY_KEYS)


def _load_candidate_dependencies(
    repo_root: Path,
    candidate_root: Path,
    route_kind: str,
) -> tuple[list[dict[str, Any]], list[str], dict[str, bool]]:
    errors: list[str] = []
    dependency_records: list[dict[str, Any]] = []
    required_keys = _candidate_dependency_keys(route_kind)
    present_by_key: dict[str, bool] = {}
    for key in required_keys:
        path = candidate_root / CANDIDATE_FILENAMES[key]
        record: dict[str, Any] = {
            "dependency_key": key,
            "artifact_layer": "Layer 2: Formal Artifact Layer",
            "required": True,
            "candidate_artifact_path": normalize_repo_relative(path, repo_root),
            "expected_classification": EXPECTED_CLASSIFICATIONS[key],
        }
        if not path.exists():
            present_by_key[key] = False
            record["present"] = False
            errors.append(f"missing Layer 2 candidate dependency {key}: {normalize_repo_relative(path, repo_root)}")
            dependency_records.append(record)
            continue
        try:
            data = load_json(path)
        except Exception as exc:
            present_by_key[key] = False
            record.update({"present": False, "load_error": str(exc)})
            errors.append(f"failed to load Layer 2 candidate dependency {key}: {exc}")
            dependency_records.append(record)
            continue
        if not isinstance(data, Mapping):
            present_by_key[key] = False
            record.update({"present": False, "load_error": "candidate artifact must be a JSON object"})
            errors.append(f"Layer 2 candidate dependency {key} must be a JSON object")
            dependency_records.append(record)
            continue
        actual_classification = data.get("classification")
        classification_ok = actual_classification == EXPECTED_CLASSIFICATIONS[key]
        present_by_key[key] = bool(classification_ok)
        record.update(
            {
                "present": True,
                "classification": actual_classification,
                "classification_ok": classification_ok,
                "is_candidate": data.get("is_candidate"),
                "is_final_formal_artifact": data.get("is_final_formal_artifact"),
                "world_model_rerun": data.get("world_model_rerun", False),
                "runtime_launched": data.get("runtime_launched", False),
                "old_scripts_called": data.get("old_scripts_called", False),
            }
        )
        if not classification_ok:
            errors.append(
                f"Layer 2 candidate dependency {key} classification must be "
                f"{EXPECTED_CLASSIFICATIONS[key]!r}, got {actual_classification!r}"
            )
        if data.get("is_candidate") is not True:
            errors.append(f"Layer 2 candidate dependency {key} must have is_candidate true")
        if data.get("is_final_formal_artifact") is not False:
            errors.append(f"Layer 2 candidate dependency {key} must have is_final_formal_artifact false")
        dependency_records.append(record)
    return dependency_records, errors, present_by_key


def _safety_flags() -> dict[str, bool]:
    return {
        "historical_stage_outputs_required": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "final_route_contract_generated": False,
        "real_astar_route_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "map_pixels_generated": False,
        "connector_geometry_regenerated": False,
        "object_approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
    }


def _blocked_finalization_fields(route_kind: str) -> list[dict[str, Any]]:
    fields = [
        ("final Layer 2 formal artifact references", "requires explicit final Layer 2 promotion in a later task"),
        ("final Layer 3 route contract status", "candidate contract must be promoted separately"),
        ("real A* waypoint list", "requires route planner inputs and final stable map artifacts"),
        ("executable route waypoints", "requires a later route execution interface task"),
        ("runtime input package", "requires a separate Layer 3 export task"),
        ("runtime validation evidence", "requires a separate Layer 4 validation task"),
        ("stable map pixels / PGM / map_server YAML", "blocked outside this metadata-only candidate task"),
        ("connector geometry regeneration", "blocked outside this candidate contract task"),
    ]
    if route_kind == "cross_floor_object":
        fields.extend(
            [
                ("object approach geometry regeneration", "object approach candidate is referenced only"),
                ("object approach feasibility revalidation", "requires a later formal validation task"),
                ("direct object centroid goal", "not allowed for this object-route contract"),
            ]
        )
    return [{"field": field, "blocked_reason": reason} for field, reason in fields]


def _downstream_layer4_unblock_plan(route_kind: str) -> dict[str, Any]:
    plan = {
        "status": "blocked_until_later_layer3_route_planning_and_layer4_runtime_validation_tasks",
        "candidate_contract_role": "records route intent and Layer 2 candidate dependency references only",
        "completed_in_this_task": False,
        "next_layer3_step": "task31_layer3_candidate_route_contract_to_route_plan_dry_run_and_schema",
        "layer4_unblocked_now": False,
        "requires_before_layer4": [
            "final Layer 2 formal artifact promotion",
            "route-plan dry-run schema",
            "real route planning task if later authorized",
            "runtime input export task if later authorized",
        ],
    }
    if route_kind == "cross_floor_object":
        plan["requires_before_layer4"].append("object approach validation task if later authorized")
    return plan


def _claim_boundary(route_kind: str) -> dict[str, Any]:
    return {
        "scope": "Layer 3 candidate route contract artifact only.",
        "route_kind": route_kind,
        "candidate_only": True,
        "is_final_route_contract": False,
        "generated_from_layer2_candidate_artifacts": True,
        "generated_from_final_layer2_artifacts": False,
        "does_not_generate_final_route_contracts": True,
        "does_not_generate_real_astar_routes": True,
        "does_not_generate_executable_routes": True,
        "does_not_generate_runtime_input_packages": True,
        "does_not_generate_runtime_validation_artifacts": True,
        "does_not_launch_gazebo_rviz_nav2_amcl_or_object_nav": True,
        "does_not_regenerate_map_pixels_pgm_or_map_server_yaml": True,
        "does_not_regenerate_connector_geometry": True,
        "does_not_regenerate_object_approach_geometry": True,
        "does_not_revalidate_approach_feasibility": True,
        "does_not_use_object_centroid_navigation_or_direct_centroid_goal": True,
        "does_not_claim_amcl_success": True,
        "does_not_claim_physical_or_real_robot_stair_climbing": True,
        "does_not_claim_full_object_navigation_benchmark": True,
        "stable_map_policy": {
            "stable_occupancy_map_package_is_layer2_candidate_dependency": True,
            "not_semantic_floorplan": True,
            "not_room_mask": True,
            "not_runtime_costmap": True,
            "not_external_gt_map": True,
            "not_simulator_navmesh": True,
            "does_not_imply_amcl_success": True,
        },
    }


def _route_intent(route_kind: str) -> dict[str, Any]:
    base = {
        "start": {"room": START_ROOM, "floor": SOURCE_FLOOR},
        "through": [
            {"room": "room_3", "floor": SOURCE_FLOOR},
            {"connector_id": CONNECTOR_ID, "connector_alias": CONNECTOR_ALIAS},
            {"room": "room_7", "floor": TARGET_FLOOR},
            {"room": "room_13", "floor": TARGET_FLOOR},
        ],
        "target": {"room": TARGET_ROOM, "floor": TARGET_FLOOR},
        "route_room_sequence": ROOM_ROUTE_SEQUENCE,
        "floor_transition": {"source_floor": SOURCE_FLOOR, "target_floor": TARGET_FLOOR},
        "connector_reference": {
            "connector_id": CONNECTOR_ID,
            "connector_alias": CONNECTOR_ALIAS,
            "transition_edge": TRANSITION_EDGE,
            "non_transition_edge": NON_TRANSITION_EDGE,
        },
    }
    if route_kind == "cross_floor_object":
        base["object_target"] = {
            "query": EXPECTED_QUERY,
            "object_id": EXPECTED_OBJECT_ID,
            "object_label": EXPECTED_OBJECT_LABEL,
            "target_room": EXPECTED_TARGET_ROOM,
            "target_floor": EXPECTED_TARGET_FLOOR,
            "approach_candidate_id": EXPECTED_APPROACH_ID,
        }
    return base


def _route_contract_payload(route_kind: str, query: str | None, object_id: str | None, approach_candidate_id: str | None) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "route_kind": route_kind,
        "start_floor": SOURCE_FLOOR,
        "target_floor": TARGET_FLOOR,
        "start_room": START_ROOM,
        "target_room": TARGET_ROOM,
        "route_room_sequence": ROOM_ROUTE_SEQUENCE,
        "connector_reference": {
            "connector_id": CONNECTOR_ID,
            "connector_alias": CONNECTOR_ALIAS,
            "allowed_connector_ids": [CONNECTOR_ID, CONNECTOR_ALIAS],
            "source_floor": SOURCE_FLOOR,
            "target_floor": TARGET_FLOOR,
            "transition_edge": TRANSITION_EDGE,
            "non_transition_edge": NON_TRANSITION_EDGE,
        },
        "route_generation_status": "candidate_contract_only_no_route_generated",
        "real_astar_route_generated": False,
        "executable_route_generated": False,
    }
    if route_kind == "cross_floor_object":
        payload.update(
            {
                "query": query,
                "object_id": object_id,
                "object_label": EXPECTED_OBJECT_LABEL,
                "target_floor": EXPECTED_TARGET_FLOOR,
                "target_room": EXPECTED_TARGET_ROOM,
                "approach_candidate_id": approach_candidate_id,
                "object_centroid_navigation_used": False,
                "direct_object_centroid_goal_used": False,
                "approach_geometry_regenerated": False,
                "approach_feasibility_revalidated": False,
                "approach_binding_status": "candidate_reference_only_no_geometry_regenerated",
            }
        )
    return payload


def build_candidate_route_contract(
    repo_root: Path,
    scene_id: str,
    route_kind: str,
    candidate_root: Path,
    output_dir: Path,
    *,
    query: str | None = None,
    object_id: str | None = None,
    approach_candidate_id: str | None = None,
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    if route_kind not in CANDIDATE_CONTRACT_FILES:
        errors.append(f"unsupported candidate route kind {route_kind!r}")
    if route_kind == "cross_floor_object":
        expected_args = {
            "query": (query, EXPECTED_QUERY),
            "object_id": (object_id, EXPECTED_OBJECT_ID),
            "approach_candidate_id": (approach_candidate_id, EXPECTED_APPROACH_ID),
        }
        for name, (actual, expected) in expected_args.items():
            if actual != expected:
                errors.append(f"{name} must be {expected!r} for cross_floor_object candidate contracts")

    dependency_records, dependency_errors, present_by_key = _load_candidate_dependencies(
        repo_root,
        candidate_root,
        route_kind,
    )
    errors.extend(dependency_errors)
    dependency_check = {
        "route_kind": route_kind,
        "candidate_root": normalize_repo_relative(candidate_root, repo_root),
        "required_dependency_keys": _candidate_dependency_keys(route_kind),
        "object_dependencies_required": route_kind == "cross_floor_object",
        "object_dependencies_forbidden": route_kind == "cross_floor_room",
        "dependencies": dependency_records,
        "all_required_dependencies_present": all(present_by_key.get(key) for key in _candidate_dependency_keys(route_kind)),
        "scope_ok": route_kind == "cross_floor_object"
        or not any(key in _candidate_dependency_keys(route_kind) for key in OBJECT_DEPENDENCY_KEYS if key not in ROOM_DEPENDENCY_KEYS),
    }
    if route_kind == "cross_floor_room":
        dependency_check["forbidden_object_dependency_keys"] = [
            "object_query_resolution",
            "object_approach",
            "object_interface_package",
        ]
    if dependency_check["scope_ok"] is not True:
        errors.append(f"dependency scope failed for {route_kind}")

    contract_path = output_dir / CANDIDATE_CONTRACT_FILES.get(route_kind, "unsupported_candidate_route_contract_v0_1.json")
    contract_classification = CANDIDATE_CONTRACT_CLASSIFICATIONS.get(route_kind, "unsupported_candidate_route_contract")
    contract = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT_NAME,
        "classification": contract_classification,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": ARTIFACT_ROLE,
        "scene_id": scene_id,
        "route_kind": route_kind,
        "is_candidate": True,
        "is_final_route_contract": False,
        "generated_from_layer2_candidate_artifacts": True,
        "generated_from_final_layer2_artifacts": False,
        **_safety_flags(),
        "layer2_candidate_dependencies": dependency_records,
        "layer2_candidate_dependency_check": dependency_check,
        "route_intent": _route_intent(route_kind),
        "route_contract_payload": _route_contract_payload(route_kind, query, object_id, approach_candidate_id),
        "blocked_until_final_layer2_or_route_planner_inputs": _blocked_finalization_fields(route_kind),
        "downstream_layer4_unblock_role": _downstream_layer4_unblock_plan(route_kind),
        "claim_boundary": _claim_boundary(route_kind),
    }

    contract_written = False
    if not errors:
        save_json(contract_path, contract)
        contract_written = True

    summary_classification = CANDIDATE_SUMMARY_CLASSIFICATIONS.get(route_kind, "candidate_route_contract_generation_failed")
    if errors:
        summary_classification = f"{route_kind}_candidate_route_contract_generation_failed"
    return {
        "project_name": PROJECT_NAME,
        "classification": summary_classification,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "route_kind": route_kind,
        "generate_candidate_contract": True,
        "candidate_route_contract_generated": contract_written,
        "candidate_route_contract_path": normalize_repo_relative(contract_path, repo_root),
        "candidate_route_contract_classification": contract_classification if contract_written else None,
        "generated_from_layer2_candidate_artifacts": True,
        "generated_from_final_layer2_artifacts": False,
        "final_route_contract_generated": False,
        "real_astar_route_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "layer2_candidate_dependency_check": dependency_check,
        "blocked_finalization_fields": _blocked_finalization_fields(route_kind),
        "downstream_layer4_unblock_plan": _downstream_layer4_unblock_plan(route_kind),
        "claim_boundary": _claim_boundary(route_kind),
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plan RSLG-SLAM route contracts or generate non-final stubs.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--route-kind", required=True, help="Route kind to plan.")
    parser.add_argument("--dry-run", action="store_true", help="Plan only, do not generate route contract stubs.")
    parser.add_argument(
        "--generate-stub",
        action="store_true",
        help="Generate a small non-final route contract stub from manifest milestone truth.",
    )
    parser.add_argument(
        "--from-layer2-candidates",
        action="store_true",
        help="Read explicit Layer 2 candidate artifacts from --candidate-root.",
    )
    parser.add_argument("--candidate-root", default=None, help="Directory containing Layer 2 candidate artifacts.")
    parser.add_argument(
        "--generate-candidate-contract",
        action="store_true",
        help="Generate a non-final candidate route contract from Layer 2 candidate artifacts.",
    )
    parser.add_argument("--output-dir", default=None, help="Directory for generated candidate route contract JSON.")
    parser.add_argument("--query", default=None, help="Object route query for cross_floor_object.")
    parser.add_argument("--object-id", default=None, help="Object id for cross_floor_object.")
    parser.add_argument("--approach-candidate-id", default=None, help="Approach candidate id for cross_floor_object.")
    parser.add_argument("--output-json", required=True, help="Write the plan or stub JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    selected_modes = sum(bool(flag) for flag in [args.dry_run, args.generate_stub, args.generate_candidate_contract])
    if selected_modes > 1:
        result = {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "scene_id": args.scene_id,
            "route_kind": args.route_kind,
            "classification": "route_contract_mode_error",
            "error": "Pass only one of --dry-run, --generate-stub, or --generate-candidate-contract.",
            "stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
        }
    elif args.generate_candidate_contract:
        if not args.from_layer2_candidates or not args.candidate_root or not args.output_dir:
            result = {
                "project_name": PROJECT_NAME,
                "repo_root": repo_root.as_posix(),
                "scene_id": args.scene_id,
                "route_kind": args.route_kind,
                "classification": "candidate_route_contract_generation_failed",
                "error_count": 1,
                "warning_count": 0,
                "artifact_layer": ARTIFACT_LAYER,
                "generate_candidate_contract": True,
                "candidate_route_contract_generated": False,
                "stage_outputs_required": False,
                "historical_stage_outputs_required": False,
                "world_model_rerun": False,
                "runtime_launched": False,
                "old_scripts_called": False,
                "business_logic_migrated": False,
                "errors": [
                    "Pass --from-layer2-candidates, --candidate-root, and --output-dir with --generate-candidate-contract."
                ],
                "warnings": [],
            }
        else:
            result = build_candidate_route_contract(
                repo_root,
                args.scene_id,
                args.route_kind,
                repo_path(repo_root, args.candidate_root),
                repo_path(repo_root, args.output_dir),
                query=args.query,
                object_id=args.object_id,
                approach_candidate_id=args.approach_candidate_id,
            )
    elif args.generate_stub:
        result = build_route_contract_stub(repo_root, args.scene_id, args.route_kind)
    else:
        result = build_dry_run_plan(repo_root, args.scene_id, args.route_kind, args.dry_run)
        if not args.dry_run:
            result["classification"] = "dry_run_unsupported_route_kind"
            result["error"] = "Pass --dry-run for planning or --generate-stub for task-evidence stubs."

    save_json(repo_path(repo_root, args.output_json), result)
    print(json.dumps({"classification": result["classification"], "output_json": args.output_json}, indent=2))
    success_classifications = {
        "route_contract_dry_run_ready",
        "route_contract_stub_generated",
        *CANDIDATE_SUMMARY_CLASSIFICATIONS.values(),
    }
    return 0 if result["classification"] in success_classifications else 1


if __name__ == "__main__":
    raise SystemExit(main())
