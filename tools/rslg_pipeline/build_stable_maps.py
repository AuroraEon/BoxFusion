"""Layer 2 stable occupancy map dry-run and candidate package builder.

This module creates metadata-only RSLG-SLAM stable-map reports. It does not
generate map pixels, PGM files, map_server YAML, runtime costmaps, routes,
runtime inputs, or call historical stable-map/runtime scripts.
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
ARTIFACT_ROLE = "stable_occupancy_map_package"
SUPPORTED_MAP_KINDS = {"stable_occupancy"}

DRY_RUN_CLASSIFICATION = "layer2_stable_map_artifact_dry_run_ready"
CANDIDATE_SUMMARY_CLASSIFICATION = "layer2_stable_map_candidate_package_generated"
CANDIDATE_ARTIFACT_CLASSIFICATION = "stable_map_package_candidate_artifact_generated"
BLOCKED_CLASSIFICATION = "layer2_stable_map_dry_run_blocked_manifest_validation_failed"
UNSUPPORTED_MAP_KIND_CLASSIFICATION = "layer2_stable_map_dry_run_unsupported_map_kind"

TASK27_EVIDENCE_ROOT_TEMPLATE = (
    "stage_outputs/stage1_generalization/{scene_id}/tasks/"
    "task27_layer2_stable_map_artifact_dry_run_and_candidate_package"
)
CANDIDATE_PACKAGE_FILENAME = "stable_occupancy_map_package_candidate_v0_1.json"

REQUIRED_MANIFEST_KEYS = [
    "project_truth_manifest",
    "pipeline_contract_manifest",
    "layer_artifacts_manifest",
    "workspace_policy_manifest",
    "validated_milestones_manifest",
]

FUTURE_REQUIRED_LAYER1_INPUTS = [
    "World Model Layer BEV/free-space/wall evidence",
    "outside-boundary evidence",
    "gateway/wall evidence",
    "floor assignment evidence",
    "pose/RGB-D-derived scene evidence",
]

FUTURE_REQUIRED_LAYER2_INPUTS = [
    "vertical connector candidate/final artifact",
    "cross-floor topology candidate/final artifact",
    "map metadata contract",
    "artifact provenance",
]

FUTURE_GENERATED_ARTIFACTS = [
    {
        "artifact_name": "stable_occupancy_map_package_v0_1",
        "artifact_role": "stable_occupancy_map_package",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
    {
        "artifact_name": "stable occupancy PGM or equivalent raster",
        "artifact_role": "future stable occupancy raster",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
    {
        "artifact_name": "stable occupancy metadata YAML or JSON",
        "artifact_role": "future map metadata",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
    {
        "artifact_name": "map provenance JSON",
        "artifact_role": "future artifact provenance",
        "would_be_generated_by_future_task": True,
        "generated_in_this_task": False,
    },
]

BLOCKED_FIELDS = [
    "current World Model Layer BEV/free-space/wall evidence",
    "current outside-boundary evidence",
    "current gateway/wall evidence",
    "current floor assignment evidence",
    "map dimensions",
    "map resolution",
    "map origin",
    "stable occupancy raster pixels",
    "stable occupancy metadata YAML or JSON",
    "map provenance JSON",
    "verified Layer 3 stable map reference",
    "candidate route contract finalization",
    "A* bridge / route planning",
    "executable route generation later",
]

MAP_DISTINCTION_POLICY = {
    "not_semantic_floorplan": True,
    "not_room_mask": True,
    "not_runtime_costmap": True,
    "not_external_gt_map": True,
    "not_simulator_navmesh": True,
}


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


def _future_stable_map_path(scene_id: str, artifact_name: str) -> str:
    return (
        "stage_outputs/stage1_generalization/"
        f"{scene_id}/clean_rerun/formal_artifacts/layer2/{artifact_name}"
    )


def _planned_candidate_package_preview(scene_id: str) -> dict[str, Any]:
    return {
        "candidate_artifact_name": CANDIDATE_PACKAGE_FILENAME,
        "candidate_schema_name": "rslg_stable_map_package_candidate",
        "candidate_payload_status": "candidate_metadata_only_no_pixels",
        "candidate_only": True,
        "not_written_as_final_artifact": True,
        "future_final_artifact_name": "stable_occupancy_map_package_v0_1",
        "future_final_artifact_path": _future_stable_map_path(scene_id, "stable_occupancy_map_package_v0_1"),
        "future_files_are_expected_not_generated": True,
        "map_pixels_included": False,
        "pgm_included": False,
        "map_server_yaml_included": False,
        "runtime_costmap_included": False,
    }


def _downstream_layer3_unblock_plan() -> dict[str, Any]:
    return {
        "candidate_route_contract_finalization": {
            "status": "blocked_until_stable_map_package_and_other_layer2_artifacts_exist",
            "stable_map_role": "provides the formal stable occupancy map reference required by route contracts",
            "completed_in_this_task": False,
        },
        "a_star_bridge_route_planning": {
            "status": "blocked_until_final_stable_map_pixels_and_metadata_exist",
            "stable_map_role": "will provide the raster/metadata contract for later A* bridge planning",
            "completed_in_this_task": False,
        },
        "executable_route_generation_later": {
            "status": "blocked_until_layer3_route_generation_uses_final_layer2_artifacts",
            "stable_map_role": "will support executable route generation after formal map finalization",
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
        or "This task creates a Layer 2 stable-map dry-run or metadata-only candidate package.",
        "stable_map_is_layer2_formal_artifact": True,
        "candidate_only_until_current_layer1_or_layer2_inputs_exist": True,
        "does_not_generate_final_stable_map": True,
        "does_not_generate_map_pixels": True,
        "does_not_generate_pgm": True,
        "does_not_generate_map_server_yaml": True,
        "does_not_generate_runtime_costmap": True,
        "does_not_generate_routes": True,
        "does_not_generate_executable_route": True,
        "does_not_launch_runtime_systems": True,
        "does_not_claim_amcl_success": True,
        "does_not_use_external_gt_map": True,
        "does_not_use_simulator_navmesh": True,
        "not_semantic_floorplan": True,
        "not_room_mask": True,
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


def _base_report(repo_root: Path, scene_id: str | None, map_kind: str, dry_run: bool) -> dict[str, Any]:
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
        "map_kind": map_kind,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_formal_artifacts_generated": False,
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "runtime_costmap_generated": False,
        "external_gt_map_used": False,
        "simulator_navmesh_used": False,
        "semantic_floorplan_claimed_as_stable_map": False,
        "room_mask_claimed_as_stable_map": False,
        "real_route_generated": False,
        "executable_route_generated": False,
        "runtime_artifacts_generated": False,
    }


def _candidate_output_dir_allowed(repo_root: Path, scene_id: str, output_dir: Path) -> tuple[bool, str]:
    rel_path = normalize_repo_relative(output_dir, repo_root)
    task27_root = TASK27_EVIDENCE_ROOT_TEMPLATE.format(scene_id=scene_id)
    allowed_prefix = f"{task27_root}/reports/candidate_artifacts"
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


def _candidate_payload() -> dict[str, Any]:
    return {
        "expected_future_map_artifact_name": "stable_occupancy_map_package_v0_1",
        "expected_future_files": [
            {
                "file_role": "stable occupancy PGM or equivalent raster",
                "status": "future_expected_not_generated",
                "generated_in_this_task": False,
            },
            {
                "file_role": "stable occupancy metadata YAML or JSON",
                "status": "future_expected_not_generated",
                "generated_in_this_task": False,
            },
            {
                "file_role": "map provenance JSON",
                "status": "future_expected_not_generated",
                "generated_in_this_task": False,
            },
        ],
        "expected_future_consumers": [
            "Layer 3 route planner / A* bridge",
            "candidate route contract finalization",
            "executable route generation later",
        ],
        "map_distinction_policy": MAP_DISTINCTION_POLICY,
        "map_dimensions": "requires_current_layer1_or_layer2_artifact",
        "map_resolution": "requires_current_layer1_or_layer2_artifact",
        "map_origin": "requires_current_layer1_or_layer2_artifact",
        "pixel_arrays": [],
        "pixel_arrays_included": False,
        "actual_pgm_path": None,
        "actual_map_server_yaml_path": None,
        "actual_runtime_costmap_path": None,
    }


def build_candidate_package(scene_id: str, map_kind: str, project_truth: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_name": "rslg_stable_map_package_candidate",
        "schema_version": "0.1",
        "classification": CANDIDATE_ARTIFACT_CLASSIFICATION,
        "artifact_layer": ARTIFACT_LAYER,
        "artifact_role": "stable_occupancy_map_package_candidate",
        "scene_id": scene_id,
        "map_kind": map_kind,
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
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "runtime_costmap_generated": False,
        "external_gt_map_used": False,
        "simulator_navmesh_used": False,
        "semantic_floorplan_claimed_as_stable_map": False,
        "room_mask_claimed_as_stable_map": False,
        "real_route_generated": False,
        "executable_route_generated": False,
        "runtime_artifacts_generated": False,
        "map_payload_status": "candidate_metadata_only_no_pixels",
        "candidate_payload": _candidate_payload(),
        "blocked_until_layer1_or_layer2_inputs": BLOCKED_FIELDS,
        "downstream_layer3_unblock_role": _downstream_layer3_unblock_plan(),
        "claim_boundary": _claim_boundary(
            project_truth,
            scope=(
                "This package is a non-final Layer 2 candidate stable-map package. "
                "It contains metadata only and no map pixels, PGM/YAML, costmap, route, or runtime validation."
            ),
        ),
    }


def build_dry_run_report(repo_root: Path, scene_id: str | None, map_kind: str, dry_run: bool) -> dict[str, Any]:
    report = _base_report(repo_root, scene_id, map_kind, dry_run)
    warnings: list[str] = []
    errors: list[str] = []

    if map_kind not in SUPPORTED_MAP_KINDS:
        errors.append(f"Unsupported map_kind: {map_kind}")
        report.update(
            {
                "classification": UNSUPPORTED_MAP_KIND_CLASSIFICATION,
                "error_count": len(errors),
                "warning_count": len(warnings),
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": [],
                "planned_candidate_package_preview": {},
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(),
                "errors": errors,
                "warnings": warnings,
            }
        )
        return report

    if not scene_id:
        errors.append("Pass --scene-id for stable map dry-run reports.")

    manifest_validation, manifests = _manifest_validation(repo_root)
    report["manifest_validation"] = manifest_validation
    if manifest_validation.get("ok") is not True:
        errors.append("Manifest validation failed.")

    contract_validation = _contract_validation(repo_root) if manifest_validation.get("ok") else {"ok": False}
    report["contract_validation"] = contract_validation
    if contract_validation.get("ok") is not True:
        errors.append("Project contract validation failed.")

    project_truth = manifests.get("project_truth_manifest", {}) if manifest_validation.get("ok") else {}
    if errors:
        report.update(
            {
                "classification": BLOCKED_CLASSIFICATION,
                "error_count": len(errors),
                "warning_count": len(warnings),
                "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
                "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
                "future_generated_artifacts": FUTURE_GENERATED_ARTIFACTS,
                "planned_candidate_package_preview": _planned_candidate_package_preview(scene_id or "unknown_scene"),
                "blocked_fields": BLOCKED_FIELDS,
                "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
                "claim_boundary": _claim_boundary(project_truth),
                "errors": errors,
                "warnings": warnings,
            }
        )
        return report

    report.update(
        {
            "classification": DRY_RUN_CLASSIFICATION,
            "error_count": 0,
            "warning_count": len(warnings),
            "future_required_layer1_inputs": FUTURE_REQUIRED_LAYER1_INPUTS,
            "future_required_layer2_inputs": FUTURE_REQUIRED_LAYER2_INPUTS,
            "future_generated_artifacts": FUTURE_GENERATED_ARTIFACTS,
            "planned_candidate_package_preview": _planned_candidate_package_preview(str(scene_id)),
            "blocked_fields": BLOCKED_FIELDS,
            "downstream_layer3_unblock_plan": _downstream_layer3_unblock_plan(),
            "claim_boundary": _claim_boundary(project_truth),
            "errors": [],
            "warnings": warnings,
        }
    )
    return report


def generate_candidate_package(repo_root: Path, scene_id: str | None, map_kind: str, output_dir: Path) -> dict[str, Any]:
    summary = _base_report(repo_root, scene_id, map_kind, dry_run=False)
    warnings: list[str] = []
    errors: list[str] = []
    candidate_path: Path | None = None

    summary.update(
        {
            "generate_candidate": True,
            "candidate_package_generated": False,
            "candidate_package_path": None,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
        }
    )

    if map_kind not in SUPPORTED_MAP_KINDS:
        errors.append(f"Unsupported map_kind: {map_kind}")
    if not scene_id:
        errors.append("Pass --scene-id with --generate-candidate.")
    if scene_id:
        output_dir_ok, output_dir_detail = _candidate_output_dir_allowed(repo_root, scene_id, output_dir)
        if not output_dir_ok:
            errors.append(output_dir_detail)
    else:
        output_dir_detail = normalize_repo_relative(output_dir, repo_root)

    dry_run_reference = build_dry_run_report(repo_root, scene_id, map_kind, dry_run=True)
    if dry_run_reference.get("classification") != DRY_RUN_CLASSIFICATION:
        errors.extend(str(error) for error in dry_run_reference.get("errors", []))
        if not errors:
            errors.append("Dry-run guardrail reference did not reach ready classification.")

    manifest_validation, manifests = _manifest_validation(repo_root)
    project_truth = manifests.get("project_truth_manifest", {}) if manifest_validation.get("ok") else {}
    if manifest_validation.get("ok") is not True:
        errors.append("Manifest validation failed.")

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
    candidate = build_candidate_package(str(scene_id), map_kind, project_truth)
    candidate_path = output_dir / CANDIDATE_PACKAGE_FILENAME
    save_json(candidate_path, candidate)

    summary.update(
        {
            "classification": CANDIDATE_SUMMARY_CLASSIFICATION,
            "error_count": 0,
            "warning_count": len(warnings),
            "output_dir": normalize_repo_relative(output_dir, repo_root),
            "candidate_package_generated": True,
            "candidate_package_path": normalize_repo_relative(candidate_path, repo_root),
            "dry_run_reference_classification": dry_run_reference.get("classification"),
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
        "module": "build_stable_maps",
        "repo_root": repo_root.as_posix(),
        "layer": "Layer 2: Formal Artifact Layer / Layer 3: Navigation Interface Layer",
        "current_role": "Layer 2 stable map dry-run and metadata-only candidate package builder.",
        "future_role": "Build final stable occupancy maps from World Model Layer BEV/free-space/wall/gateway evidence.",
        "business_logic_migrated": False,
        "real_logic_status": "candidate_metadata_only_no_pixel_generation",
        "dry_run": bool(dry_run),
        "runtime_launched": False,
        "old_scripts_called": False,
        "large_artifacts_written": False,
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "runtime_costmap_generated": False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Plan or generate metadata-only RSLG-SLAM Layer 2 stable occupancy map candidates."
    )
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", default=None, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--map-kind", default="stable_occupancy", help="Map kind to plan.")
    parser.add_argument("--dry-run", action="store_true", help="Plan only; do not generate final artifacts.")
    parser.add_argument(
        "--generate-candidate",
        action="store_true",
        help="Write a non-final metadata-only stable-map candidate package under the task27 evidence directory.",
    )
    parser.add_argument("--output-dir", default=None, help="Candidate package output directory.")
    parser.add_argument("--output-json", default=None, help="Write the report JSON to this path.")
    parser.add_argument("--describe", action="store_true", help="Describe safe builder behavior without writing artifacts.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    if args.describe or (args.dry_run and not args.scene_id and not args.output_json and not args.generate_candidate):
        print(json.dumps(_placeholder_summary(repo_root, dry_run=bool(args.dry_run)), indent=2, sort_keys=False))
        return 0

    if args.dry_run and args.generate_candidate:
        result = _base_report(repo_root, args.scene_id, args.map_kind, dry_run=False)
        result.update(
            {
                "classification": BLOCKED_CLASSIFICATION,
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
            result = _base_report(repo_root, args.scene_id, args.map_kind, dry_run=False)
            result.update(
                {
                    "classification": BLOCKED_CLASSIFICATION,
                    "error_count": 1,
                    "generate_candidate": True,
                    "candidate_package_generated": False,
                    "candidate_package_path": None,
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
                args.map_kind,
                repo_path(repo_root, args.output_dir),
            )
    elif args.dry_run:
        result = build_dry_run_report(repo_root, args.scene_id, args.map_kind, dry_run=True)
    else:
        result = _base_report(repo_root, args.scene_id, args.map_kind, dry_run=False)
        result.update(
            {
                "classification": BLOCKED_CLASSIFICATION,
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
