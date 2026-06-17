"""Layer 2 planner for RSLG-SLAM vertical connector artifacts.

This module plans future vertical connector, connector graph, and cross-floor
topology formal artifacts from docs/manifests. It can also write small
candidate Layer 2 artifacts as task evidence. It does not generate final formal
artifacts, connector geometry, route waypoints, runtime inputs, or call
historical vertical connector scripts.
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

SUPPORTED_ARTIFACT_KINDS = {
    "all",
    "vertical_connectors",
    "connector_graph",
    "cross_floor_topology",
}

READY_CLASSIFICATIONS = {
    "all": "layer2_vertical_connector_topology_dry_run_ready",
    "vertical_connectors": "layer2_vertical_connector_dry_run_ready",
    "connector_graph": "layer2_connector_graph_dry_run_ready",
    "cross_floor_topology": "layer2_cross_floor_topology_dry_run_ready",
}

CANDIDATE_SUMMARY_CLASSIFICATIONS = {
    "all": "layer2_vertical_connector_topology_candidate_artifacts_generated",
    "vertical_connectors": "layer2_vertical_connector_candidate_artifact_generated",
    "connector_graph": "layer2_connector_graph_candidate_artifact_generated",
    "cross_floor_topology": "layer2_cross_floor_topology_candidate_artifact_generated",
}

CANDIDATE_ARTIFACT_CLASSIFICATIONS = {
    "vertical_connectors": "vertical_connector_candidate_artifact_generated",
    "connector_graph": "connector_graph_candidate_artifact_generated",
    "cross_floor_topology": "cross_floor_topology_candidate_artifact_generated",
}

CANDIDATE_ARTIFACT_FILES = {
    "vertical_connectors": "vertical_connectors_candidate_v0_1.json",
    "connector_graph": "stairs_or_vertical_connector_graph_candidate_v0_1.json",
    "cross_floor_topology": "cross_floor_topology_candidate_v0_1.json",
}

BLOCKED_MANIFEST_VALIDATION = "layer2_vertical_connector_dry_run_blocked_manifest_validation_failed"
BLOCKED_MISSING_TRUTH = "layer2_vertical_connector_dry_run_blocked_missing_validated_truth"
UNSUPPORTED_ARTIFACT_KIND = "layer2_vertical_connector_dry_run_unsupported_artifact_kind"

REQUIRED_MANIFEST_KEYS = [
    "project_truth_manifest",
    "pipeline_contract_manifest",
    "layer_artifacts_manifest",
    "workspace_policy_manifest",
    "validated_milestones_manifest",
]

PREVIEW_ARTIFACTS = {
    "vertical_connectors": {
        "artifact_name": "vertical_connectors_v0_1.json",
        "artifact_role": "vertical_connector_formal_artifact",
    },
    "connector_graph": {
        "artifact_name": "stairs_or_vertical_connector_graph_v0_1.json",
        "artifact_role": "vertical_connector_graph_formal_artifact",
    },
    "cross_floor_topology": {
        "artifact_name": "cross_floor_topology_v0_1.json",
        "artifact_role": "cross_floor_topology_formal_artifact",
    },
}

FUTURE_REQUIRED_LAYER1_INPUTS = [
    "posed RGB-D World Model Layer outputs",
    "floor segmentation / floor assignment evidence",
    "pose-height transition evidence",
    "semantic stair / vertical connector object evidence if available",
    "room/floor/object world model evidence if applicable",
]

FUTURE_REQUIRED_LAYER2_INPUTS = [
    "per-floor room topology",
    "gateway topology",
    "stable occupancy map package",
    "connector candidate set",
    "connector centerline candidate / graph edges",
]

BLOCKED_FIELDS = [
    "real connector geometry regenerated from current Layer 1 outputs",
    "real connector centerline fitted from current pose-height transition evidence",
    "real connector graph generated from current formal artifacts",
    "real cross-floor topology generated from current formal artifacts",
    "stable occupancy map verified locally",
    "route planner-ready topology package",
    "downstream final candidate route contract",
    "A* waypoint route",
    "executable route",
    "runtime validation",
]

TASK26_EVIDENCE_ROOT_TEMPLATE = (
    "stage_outputs/stage1_generalization/{scene_id}/tasks/"
    "task26_layer2_vertical_connector_topology_candidate_artifacts"
)


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
    if isinstance(milestones, list):
        return [item for item in milestones if isinstance(item, Mapping)]
    return []


def _find_milestone(manifest: Mapping[str, Any], milestone_id: str) -> Mapping[str, Any] | None:
    for milestone in _milestones(manifest):
        if milestone.get("id") == milestone_id:
            return milestone
    return None


def _extract_validated_truth(manifest: Mapping[str, Any], scene_id: str) -> dict[str, Any]:
    scene_prefix = scene_id.split("-", maxsplit=1)[0]
    route = _find_milestone(manifest, f"{scene_prefix}_cross_floor_route_truth")
    transition = _find_milestone(manifest, f"{scene_prefix}_vt_1_transition_edge_truth")

    route_sequence = route.get("route") if route else None
    centerline_nodes = transition.get("centerline_nodes", []) if transition else []
    has_route = isinstance(route_sequence, list) and bool(route_sequence)
    has_transition = bool(transition and transition.get("transition_edge") and transition.get("not_transition_edge"))
    has_nodes = isinstance(centerline_nodes, list) and bool(centerline_nodes)

    return {
        "present": has_route and has_transition,
        "source_manifest": "validated_milestones_manifest_v0_1.json",
        "route_milestone_id": route.get("id") if route else None,
        "transition_milestone_id": transition.get("id") if transition else None,
        "status": "validated" if has_route and has_transition else "missing",
        "connector_ids": ["vt_1", "vc_vt_1"],
        "connector_id_primary": "vt_1",
        "connector_id_alias": "vc_vt_1",
        "source_floor": "floor_1",
        "target_floor": "floor_2",
        "transition_edge": transition.get("transition_edge") if transition else None,
        "non_transition_edge": transition.get("not_transition_edge") if transition else None,
        "route_context": ["room_2", "room_3", "room_7", "room_13", "room_14"],
        "route_sequence": route_sequence if has_route else [],
        "centerline_nodes": centerline_nodes if has_nodes else "available_in_historical_truth_only_or_requires_layer1_artifact",
        "centerline_node_status": "validated_milestone_manifest" if has_nodes else "available_in_historical_truth_only_or_requires_layer1_artifact",
        "geometry_generated": False,
    }


def _artifact_kinds_for_request(artifact_kind: str) -> list[str]:
    if artifact_kind == "all":
        return ["vertical_connectors", "connector_graph", "cross_floor_topology"]
    if artifact_kind in PREVIEW_ARTIFACTS:
        return [artifact_kind]
    return []


def _future_path(scene_id: str, artifact_name: str) -> str:
    return (
        "stage_outputs/stage1_generalization/"
        f"{scene_id}/clean_rerun/formal_artifacts/layer2/{artifact_name}"
    )


def _planned_artifact_previews(scene_id: str, artifact_kind: str) -> list[dict[str, Any]]:
    previews = []
    for kind in _artifact_kinds_for_request(artifact_kind):
        metadata = PREVIEW_ARTIFACTS[kind]
        previews.append(
            {
                "artifact_kind": kind,
                "artifact_role": metadata["artifact_role"],
                "future_artifact_name": metadata["artifact_name"],
                "future_generated_output_path": _future_path(scene_id, metadata["artifact_name"]),
                "preview_only": True,
                "not_written_as_final_artifact": True,
                "requires_layer1_or_current_formal_artifacts_before_finalization": True,
                "existence_checked": False,
                "written": False,
            }
        )
    return previews


def _future_generated_artifacts(scene_id: str, artifact_kind: str) -> list[dict[str, Any]]:
    return [
        {
            "artifact_kind": preview["artifact_kind"],
            "artifact_name": preview["future_artifact_name"],
            "future_generated_output_path": preview["future_generated_output_path"],
            "would_be_generated_by_future_task": True,
            "generated_in_this_task": False,
        }
        for preview in _planned_artifact_previews(scene_id, artifact_kind)
    ]


def _downstream_layer3_unblock_plan() -> dict[str, Any]:
    return {
        "cross_floor_room_candidate_route_contract_finalization_needs": [
            "stable occupancy map package",
            "vertical connector artifact",
            "cross-floor topology artifact",
        ],
        "cross_floor_object_candidate_route_contract_finalization_additionally_needs": [
            "object query resolution artifact",
            "object approach candidate artifact",
        ],
        "unblock_sequence": [
            "Finalize Layer 2 vertical connector artifact schema and validator.",
            "Generate or validate real connector graph and cross-floor topology from current formal inputs.",
            "Finalize stable occupancy map package in Layer 2.",
            "Return to Layer 3 route contract finalization after Layer 2 artifacts exist.",
        ],
    }


def _claim_boundary(project_truth: Mapping[str, Any] | None = None, *, scope: str | None = None) -> dict[str, Any]:
    project_truth = project_truth or {}
    return {
        "scope": scope or "This task performs Layer 2 dry-run planning only.",
        "does_not_regenerate_connector_geometry": True,
        "does_not_regenerate_formal_topology": True,
        "does_not_generate_route_waypoints": True,
        "does_not_validate_physical_stair_climbing": True,
        "does_not_run_runtime_systems": True,
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


def _base_report(repo_root: Path, scene_id: str, artifact_kind: str, dry_run: bool) -> dict[str, Any]:
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": None,
        "error_count": 0,
        "warning_count": 0,
        "dry_run": bool(dry_run),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "artifact_kind": artifact_kind,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "vertical_connector_geometry_generated": False,
        "cross_floor_topology_generated": False,
        "stable_map_generated": False,
        "object_interface_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
    }


def _candidate_output_dir_allowed(repo_root: Path, scene_id: str, output_dir: Path) -> tuple[bool, str]:
    rel_path = normalize_repo_relative(output_dir, repo_root)
    task26_root = TASK26_EVIDENCE_ROOT_TEMPLATE.format(scene_id=scene_id)
    allowed_prefix = f"{task26_root}/reports/candidate_artifacts"
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


def _candidate_common_fields(
    *,
    schema_name: str,
    classification: str,
    artifact_role: str,
    scene_id: str,
    validated_truth: Mapping[str, Any],
    project_truth: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_name": schema_name,
        "schema_version": "0.1",
        "classification": classification,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": artifact_role,
        "scene_id": scene_id,
        "is_candidate": True,
        "is_final_formal_artifact": False,
        "generated_from_validated_milestones": True,
        "generated_from_current_layer1_outputs": False,
        "historical_stage_outputs_required": False,
        "stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "real_geometry_regenerated": False,
        "real_topology_regenerated": False,
        "stable_map_generated": False,
        "object_interface_generated": False,
        "real_route_generated": False,
        "runtime_artifacts_generated": False,
        "validated_truth_lineage": {
            "source_manifest": validated_truth.get("source_manifest"),
            "route_milestone_id": validated_truth.get("route_milestone_id"),
            "transition_milestone_id": validated_truth.get("transition_milestone_id"),
            "connector_id_primary": validated_truth.get("connector_id_primary"),
            "connector_id_alias": validated_truth.get("connector_id_alias"),
            "source_floor": validated_truth.get("source_floor"),
            "target_floor": validated_truth.get("target_floor"),
            "transition_edge": validated_truth.get("transition_edge"),
            "non_transition_edge": validated_truth.get("non_transition_edge"),
            "route_context": validated_truth.get("route_context"),
            "centerline_nodes": validated_truth.get("centerline_nodes"),
            "geometry_status": "candidate_topology_only_no_regenerated_geometry",
        },
        "blocked_until_layer1_or_layer2_inputs": BLOCKED_FIELDS,
        "downstream_layer3_unblock_role": _downstream_layer3_unblock_plan(),
        "claim_boundary": _claim_boundary(
            project_truth,
            scope=(
                "This task generates non-final Layer 2 candidate artifacts from validated milestone truth only. "
                "It does not regenerate real connector geometry or final cross-floor topology."
            ),
        ),
    }


def _vertical_connector_candidate(
    scene_id: str, validated_truth: Mapping[str, Any], project_truth: Mapping[str, Any]
) -> dict[str, Any]:
    artifact = _candidate_common_fields(
        schema_name="rslg_vertical_connectors_candidate",
        classification=CANDIDATE_ARTIFACT_CLASSIFICATIONS["vertical_connectors"],
        artifact_role="vertical_connector_candidate_artifact",
        scene_id=scene_id,
        validated_truth=validated_truth,
        project_truth=project_truth,
    )
    artifact["candidate_payload"] = {
        "connector_id": validated_truth.get("connector_id_primary", "vt_1"),
        "connector_id_alias": validated_truth.get("connector_id_alias", "vc_vt_1"),
        "connector_family": "vertical connector",
        "connector_type": "stair connector candidate",
        "candidate_status": "candidate topology-only connector, not final formal connector geometry",
        "source_floor": validated_truth.get("source_floor"),
        "target_floor": validated_truth.get("target_floor"),
        "transition_edge": validated_truth.get("transition_edge"),
        "non_transition_edge": validated_truth.get("non_transition_edge"),
        "centerline_node_ids": validated_truth.get("centerline_nodes"),
        "route_context_rooms": validated_truth.get("route_context"),
        "geometry_status": "candidate_topology_only_no_regenerated_geometry",
        "coordinates_available_from_current_active_manifests": False,
        "coordinates": [],
    }
    return artifact


def _connector_graph_candidate(
    scene_id: str, validated_truth: Mapping[str, Any], project_truth: Mapping[str, Any]
) -> dict[str, Any]:
    centerline_nodes = validated_truth.get("centerline_nodes")
    if not isinstance(centerline_nodes, list):
        centerline_nodes = []
    artifact = _candidate_common_fields(
        schema_name="rslg_stairs_or_vertical_connector_graph_candidate",
        classification=CANDIDATE_ARTIFACT_CLASSIFICATIONS["connector_graph"],
        artifact_role="vertical_connector_graph_candidate_artifact",
        scene_id=scene_id,
        validated_truth=validated_truth,
        project_truth=project_truth,
    )
    artifact["candidate_payload"] = {
        "graph_id": "vc_graph_candidate_v0_1",
        "connector_reference": validated_truth.get("connector_id_primary", "vt_1"),
        "graph_status": {
            "candidate_graph": True,
            "not_final_route_planner_graph": True,
            "not_generated_from_current_raw_layer1_outputs": True,
            "metric_geometry_inferred": False,
        },
        "nodes": [{"node_id": node_id, "geometry_status": "no_regenerated_coordinates"} for node_id in centerline_nodes],
        "edges": [
            {
                "edge_id": validated_truth.get("transition_edge"),
                "is_transition_edge": True,
                "source_floor": validated_truth.get("source_floor"),
                "target_floor": validated_truth.get("target_floor"),
            },
            {
                "edge_id": validated_truth.get("non_transition_edge"),
                "is_transition_edge": False,
                "not_used_as_transition": True,
            },
        ],
        "floor_transition": {
            "from": validated_truth.get("source_floor"),
            "to": validated_truth.get("target_floor"),
            "transition_edge": validated_truth.get("transition_edge"),
        },
        "geometry_status": "candidate_topology_only_no_regenerated_geometry",
    }
    return artifact


def _cross_floor_topology_candidate(
    scene_id: str, validated_truth: Mapping[str, Any], project_truth: Mapping[str, Any]
) -> dict[str, Any]:
    connector_id = validated_truth.get("connector_id_primary", "vt_1")
    artifact = _candidate_common_fields(
        schema_name="rslg_cross_floor_topology_candidate",
        classification=CANDIDATE_ARTIFACT_CLASSIFICATIONS["cross_floor_topology"],
        artifact_role="cross_floor_topology_candidate_artifact",
        scene_id=scene_id,
        validated_truth=validated_truth,
        project_truth=project_truth,
    )
    artifact["candidate_payload"] = {
        "topology_id": "cross_floor_topology_candidate_v0_1",
        "route_context": ["room_2", "room_3", connector_id, "room_7", "room_13", "room_14"],
        "source_floor": validated_truth.get("source_floor"),
        "target_floor": validated_truth.get("target_floor"),
        "connector_reference": connector_id,
        "connector_alias": validated_truth.get("connector_id_alias"),
        "transition_edge_reference": validated_truth.get("transition_edge"),
        "non_transition_edge_reference": validated_truth.get("non_transition_edge"),
        "candidate_status": {
            "candidate_topology": True,
            "not_final_cross_floor_topology": True,
            "not_route_planner_ready_final_topology": True,
            "a_star_waypoints_generated": False,
            "executable_route_generated": False,
        },
        "geometry_status": "candidate_topology_only_no_regenerated_geometry",
    }
    return artifact


def _candidate_artifact_for_kind(
    kind: str, scene_id: str, validated_truth: Mapping[str, Any], project_truth: Mapping[str, Any]
) -> dict[str, Any]:
    builders = {
        "vertical_connectors": _vertical_connector_candidate,
        "connector_graph": _connector_graph_candidate,
        "cross_floor_topology": _cross_floor_topology_candidate,
    }
    return builders[kind](scene_id, validated_truth, project_truth)


def _summary_claim_boundary(project_truth: Mapping[str, Any]) -> dict[str, Any]:
    return _claim_boundary(
        project_truth,
        scope=(
            "This summary records non-final Layer 2 candidate artifact generation. "
            "The artifacts are generated from validated milestones, not from a current Layer 1 rerun."
        ),
    )


def generate_candidate_artifacts(
    repo_root: Path,
    scene_id: str,
    artifact_kind: str,
    output_dir: Path,
) -> dict[str, Any]:
    candidate_paths: list[str] = []
    candidate_classifications: list[str] = []
    warnings: list[str] = []
    errors: list[str] = []

    base = _base_report(repo_root, scene_id, artifact_kind, dry_run=False)
    base.update(
        {
            "generate_candidate": True,
            "candidate_artifacts_generated": False,
            "candidate_artifact_paths": candidate_paths,
            "candidate_artifact_classifications": candidate_classifications,
            "final_formal_artifacts_generated": False,
            "generated_from_validated_milestones": True,
            "generated_from_current_layer1_outputs": False,
            "real_geometry_regenerated": False,
            "real_topology_regenerated": False,
            "real_route_generated": False,
            "runtime_artifacts_generated": False,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
        }
    )

    if artifact_kind not in SUPPORTED_ARTIFACT_KINDS:
        base.update(
            {
                "classification": UNSUPPORTED_ARTIFACT_KIND,
                "error_count": 1,
                "validated_truth_used": {},
                "blocked_finalization_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": [f"Unsupported artifact_kind: {artifact_kind}"],
                "warnings": warnings,
            }
        )
        return base

    output_dir_ok, output_dir_detail = _candidate_output_dir_allowed(repo_root, scene_id, output_dir)
    if not output_dir_ok:
        errors.append(output_dir_detail)

    dry_run_reference = build_dry_run_report(repo_root, scene_id, artifact_kind, dry_run=True)
    if dry_run_reference.get("classification") not in set(READY_CLASSIFICATIONS.values()):
        errors.extend(str(error) for error in dry_run_reference.get("errors", []))
        if not errors:
            errors.append("Dry-run guardrail reference did not reach ready classification.")

    validated_truth = dry_run_reference.get("validated_truth_used", {})
    if not isinstance(validated_truth, Mapping) or validated_truth.get("present") is not True:
        errors.append("Required validated truth is unavailable.")

    manifest_validation, manifests = _manifest_validation(repo_root)
    project_truth = manifests.get("project_truth_manifest", {}) if manifest_validation.get("ok") else {}
    if manifest_validation.get("ok") is not True:
        errors.append("Manifest validation failed.")

    if errors:
        base.update(
            {
                "classification": BLOCKED_MANIFEST_VALIDATION,
                "error_count": len(errors),
                "warning_count": len(warnings),
                "validated_truth_used": validated_truth if isinstance(validated_truth, Mapping) else {},
                "dry_run_reference_classification": dry_run_reference.get("classification"),
                "output_dir": output_dir_detail,
                "blocked_finalization_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _summary_claim_boundary(project_truth),
                "errors": errors,
                "warnings": warnings,
            }
        )
        return base

    output_dir.mkdir(parents=True, exist_ok=True)
    for kind in _artifact_kinds_for_request(artifact_kind):
        artifact = _candidate_artifact_for_kind(kind, scene_id, validated_truth, project_truth)
        output_path = output_dir / CANDIDATE_ARTIFACT_FILES[kind]
        save_json(output_path, artifact)
        candidate_paths.append(normalize_repo_relative(output_path, repo_root))
        candidate_classifications.append(artifact["classification"])

    base.update(
        {
            "classification": CANDIDATE_SUMMARY_CLASSIFICATIONS[artifact_kind],
            "error_count": 0,
            "warning_count": len(warnings),
            "output_dir": normalize_repo_relative(output_dir, repo_root),
            "candidate_artifacts_generated": True,
            "validated_truth_used": validated_truth,
            "dry_run_reference_classification": dry_run_reference.get("classification"),
            "blocked_finalization_fields": BLOCKED_FIELDS,
            "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
            "claim_boundary": _summary_claim_boundary(project_truth),
            "errors": [],
            "warnings": warnings,
        }
    )
    return base


def build_dry_run_report(repo_root: Path, scene_id: str, artifact_kind: str, dry_run: bool) -> dict[str, Any]:
    report = _base_report(repo_root, scene_id, artifact_kind, dry_run)
    warnings: list[str] = []

    if artifact_kind not in SUPPORTED_ARTIFACT_KINDS:
        report.update(
            {
                "classification": UNSUPPORTED_ARTIFACT_KIND,
                "error_count": 1,
                "supported_artifact_kinds": sorted(SUPPORTED_ARTIFACT_KINDS),
                "validated_truth_used": {},
                "planned_artifact_previews": [],
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": [],
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": [f"Unsupported artifact_kind: {artifact_kind}"],
                "warnings": warnings,
            }
        )
        return report

    manifest_validation, manifests = _manifest_validation(repo_root)
    report["manifest_validation"] = manifest_validation
    if not manifest_validation["ok"]:
        report.update(
            {
                "classification": BLOCKED_MANIFEST_VALIDATION,
                "error_count": 1,
                "validated_truth_used": {},
                "planned_artifact_previews": _planned_artifact_previews(scene_id, artifact_kind),
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": _future_generated_artifacts(scene_id, artifact_kind),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": ["Manifest validation failed."],
                "warnings": warnings,
            }
        )
        return report

    contract_validation = _contract_validation(repo_root)
    report["contract_validation"] = contract_validation
    if not contract_validation["ok"]:
        report.update(
            {
                "classification": BLOCKED_MANIFEST_VALIDATION,
                "error_count": 1,
                "validated_truth_used": {},
                "planned_artifact_previews": _planned_artifact_previews(scene_id, artifact_kind),
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": _future_generated_artifacts(scene_id, artifact_kind),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(manifests.get("project_truth_manifest", {})),
                "errors": ["Project contract validation failed."],
                "warnings": warnings,
            }
        )
        return report

    validated_truth = _extract_validated_truth(manifests["validated_milestones_manifest"], scene_id)
    if not validated_truth["present"]:
        report.update(
            {
                "classification": BLOCKED_MISSING_TRUTH,
                "error_count": 1,
                "validated_truth_used": validated_truth,
                "planned_artifact_previews": _planned_artifact_previews(scene_id, artifact_kind),
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": _future_generated_artifacts(scene_id, artifact_kind),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(manifests["project_truth_manifest"]),
                "errors": ["Required validated vertical connector truth is missing."],
                "warnings": warnings,
            }
        )
        return report

    if validated_truth["centerline_node_status"] != "validated_milestone_manifest":
        warnings.append("Centerline node IDs are not present in current manifests; geometry remains unavailable.")

    report.update(
        {
            "classification": READY_CLASSIFICATIONS[artifact_kind],
            "error_count": 0,
            "warning_count": len(warnings),
            "validated_truth_used": validated_truth,
            "planned_artifact_previews": _planned_artifact_previews(scene_id, artifact_kind),
            "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
            "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
            "future_generated_artifacts": _future_generated_artifacts(scene_id, artifact_kind),
            "blocked_fields": BLOCKED_FIELDS,
            "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
            "claim_boundary": _claim_boundary(manifests["project_truth_manifest"]),
            "errors": [],
            "warnings": warnings,
        }
    )

    structure_validation = validate_dry_run_report_structure(report)
    report["dry_run_structure_validation"] = structure_validation
    if not structure_validation["ok"]:
        report["classification"] = BLOCKED_MANIFEST_VALIDATION
        report["error_count"] = max(1, int(report.get("error_count", 0) or 0), structure_validation["error_count"])
        report["errors"] = list(report.get("errors", [])) + structure_validation["errors"]
    return report


def validate_dry_run_report_structure(report: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the dry-run report shape without a separate schema module."""
    errors: list[str] = []
    warnings: list[str] = []

    required_fields = [
        "classification",
        "error_count",
        "warning_count",
        "dry_run",
        "artifact_layer",
        "scene_id",
        "artifact_kind",
        "stage_outputs_required",
        "historical_stage_outputs_required",
        "world_model_rerun",
        "runtime_launched",
        "old_scripts_called",
        "business_logic_migrated",
        "final_formal_artifacts_generated",
        "vertical_connector_geometry_generated",
        "cross_floor_topology_generated",
        "stable_map_generated",
        "object_interface_generated",
        "real_route_generated",
        "runtime_artifacts_generated",
        "validated_truth_used",
        "planned_artifact_previews",
        "future_required_layer1_inputs",
        "future_required_layer2_inputs",
        "future_generated_artifacts",
        "blocked_fields",
        "downstream_layer3_unblock_plan",
        "claim_boundary",
    ]
    for field in required_fields:
        if field not in report:
            errors.append(f"missing required field: {field}")

    false_flags = [
        "stage_outputs_required",
        "historical_stage_outputs_required",
        "world_model_rerun",
        "runtime_launched",
        "old_scripts_called",
        "business_logic_migrated",
        "final_formal_artifacts_generated",
        "vertical_connector_geometry_generated",
        "cross_floor_topology_generated",
        "stable_map_generated",
        "object_interface_generated",
        "real_route_generated",
        "runtime_artifacts_generated",
    ]
    for flag in false_flags:
        if report.get(flag) is not False:
            errors.append(f"{flag} must be false")

    if report.get("dry_run") is not True:
        errors.append("dry_run must be true")
    if report.get("artifact_layer") != ARTIFACT_LAYER:
        errors.append(f"artifact_layer must be {ARTIFACT_LAYER!r}")

    classification = report.get("classification")
    allowed_classifications = set(READY_CLASSIFICATIONS.values()) | {
        BLOCKED_MANIFEST_VALIDATION,
        BLOCKED_MISSING_TRUTH,
        UNSUPPORTED_ARTIFACT_KIND,
    }
    if classification not in allowed_classifications:
        errors.append(f"unsupported classification: {classification!r}")

    previews = report.get("planned_artifact_previews", [])
    if not isinstance(previews, list):
        errors.append("planned_artifact_previews must be a list")
    else:
        for index, preview in enumerate(previews):
            if not isinstance(preview, Mapping):
                errors.append(f"planned_artifact_previews[{index}] must be an object")
                continue
            for key in [
                "preview_only",
                "not_written_as_final_artifact",
                "requires_layer1_or_current_formal_artifacts_before_finalization",
            ]:
                if preview.get(key) is not True:
                    errors.append(f"planned_artifact_previews[{index}].{key} must be true")
            if "future_generated_output_path" not in preview:
                errors.append(f"planned_artifact_previews[{index}] missing future_generated_output_path")

    truth = report.get("validated_truth_used", {})
    if classification in set(READY_CLASSIFICATIONS.values()):
        if not isinstance(truth, Mapping) or truth.get("present") is not True:
            errors.append("ready reports must include validated_truth_used.present true")
        if isinstance(truth, Mapping) and truth.get("transition_edge") != "vt_1_centerline_e001":
            errors.append("validated truth must preserve vt_1_centerline_e001 as transition edge")
        if isinstance(truth, Mapping) and truth.get("non_transition_edge") != "vt_1_centerline_e003":
            errors.append("validated truth must preserve vt_1_centerline_e003 as non-transition edge")

    if not isinstance(report.get("blocked_fields", []), list) or not report.get("blocked_fields"):
        errors.append("blocked_fields must be a non-empty list")

    return {
        "classification": "layer2_vertical_connector_dry_run_structure_validation_passed"
        if not errors
        else "layer2_vertical_connector_dry_run_structure_validation_failed",
        "ok": not errors,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Plan or generate candidate RSLG-SLAM Layer 2 vertical connector/topology artifacts."
    )
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--artifact-kind", required=True, help="Artifact kind to plan.")
    parser.add_argument("--dry-run", action="store_true", help="Plan only; do not generate final artifacts.")
    parser.add_argument(
        "--generate-candidate",
        action="store_true",
        help="Write non-final candidate Layer 2 artifacts under the task26 evidence directory.",
    )
    parser.add_argument("--output-dir", default=None, help="Candidate artifact output directory.")
    parser.add_argument("--output-json", required=True, help="Write the report JSON to this path.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    if args.dry_run and args.generate_candidate:
        result = _base_report(repo_root, args.scene_id, args.artifact_kind, dry_run=False)
        result.update(
            {
                "classification": UNSUPPORTED_ARTIFACT_KIND,
                "error_count": 1,
                "generate_candidate": bool(args.generate_candidate),
                "validated_truth_used": {},
                "planned_artifact_previews": [],
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": [],
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": ["Use either --dry-run or --generate-candidate, not both."],
                "warnings": [],
            }
        )
    elif args.generate_candidate:
        if not args.output_dir:
            result = _base_report(repo_root, args.scene_id, args.artifact_kind, dry_run=False)
            result.update(
                {
                    "classification": BLOCKED_MANIFEST_VALIDATION,
                    "error_count": 1,
                    "generate_candidate": True,
                    "candidate_artifacts_generated": False,
                    "validated_truth_used": {},
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
            result = generate_candidate_artifacts(
                repo_root,
                args.scene_id,
                args.artifact_kind,
                repo_path(repo_root, args.output_dir),
            )
    elif not args.dry_run:
        result = _base_report(repo_root, args.scene_id, args.artifact_kind, dry_run=False)
        result.update(
            {
                "classification": UNSUPPORTED_ARTIFACT_KIND,
                "error_count": 1,
                "dry_run": False,
                "generate_candidate": False,
                "validated_truth_used": {},
                "planned_artifact_previews": [],
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": [],
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": ["Pass --dry-run or --generate-candidate."],
                "warnings": [],
            }
        )
    else:
        result = build_dry_run_report(repo_root, args.scene_id, args.artifact_kind, dry_run=True)

    save_json(repo_path(repo_root, args.output_json), result)
    print(json.dumps({"classification": result["classification"], "output_json": args.output_json}, indent=2))
    success_classifications = set(READY_CLASSIFICATIONS.values()) | set(CANDIDATE_SUMMARY_CLASSIFICATIONS.values())
    return 0 if result["classification"] in success_classifications else 1


if __name__ == "__main__":
    raise SystemExit(main())
