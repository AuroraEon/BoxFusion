"""Review the boundary before final RSLG-SLAM Layer 2 artifact generation.

This module produces a planning/report artifact only. It does not generate
final Layer 2 artifacts, final Layer 3 route contracts, real A* routes,
waypoints, executable routes, runtime input packages, map pixels, connector
geometry, object approach geometry, or runtime validation evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

from .common import PROJECT_NAME, normalize_repo_relative, repo_path, resolve_repo_root, save_json


COMPLETED_CLASSIFICATION = "final_artifact_boundary_review_completed"
FAILED_CLASSIFICATION = "final_artifact_boundary_review_failed"

ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
REAL_ROUTE_BLOCKED = "real_route_generation_blocked_until_final_layer2_artifacts_exist"
RECOMMENDED_NEXT_PHASE = "return_to_layer2_final_artifact_generation_planning"
RECOMMENDED_NEXT_TASK = "task34_layer2_final_artifact_minimal_generation_plan"

DOCS_AND_MANIFESTS_READ = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/manifest_retention_policy.md",
    "docs/rslg_slam/rslg_pipeline_skeleton.md",
    "docs/rslg_slam/rslg_pipeline_test_plan.md",
    "docs/rslg_slam/ai_handoff_context.md",
    "docs/rslg_slam/ai_handoff_file_list.md",
    "docs/rslg_slam/tool_entrypoint_mapping.md",
    "docs/rslg_slam/manifests/rslg_slam_manifest_index_v0_1.json",
    "docs/rslg_slam/manifests/project_truth_manifest_v0_1.json",
    "docs/rslg_slam/manifests/pipeline_contract_manifest_v0_1.json",
    "docs/rslg_slam/manifests/layer_artifacts_manifest_v0_1.json",
    "docs/rslg_slam/manifests/workspace_policy_manifest_v0_1.json",
    "docs/rslg_slam/manifests/protected_assets_manifest_v0_1.json",
    "docs/rslg_slam/manifests/validated_milestones_manifest_v0_1.json",
    "docs/rslg_slam/manifests/manifest_retention_plan_v0_1.json",
    "docs/rslg_slam/manifests/rslg_pipeline_test_plan_manifest_v0_1.json",
]

TOOLS_INSPECTED = [
    "tools/rslg_pipeline/common.py",
    "tools/rslg_pipeline/artifact_registry.py",
    "tools/rslg_pipeline/validate_artifacts.py",
    "tools/rslg_pipeline/build_vertical_connectors.py",
    "tools/rslg_pipeline/vertical_connector_schema.py",
    "tools/rslg_pipeline/build_stable_maps.py",
    "tools/rslg_pipeline/stable_map_schema.py",
    "tools/rslg_pipeline/build_object_interfaces.py",
    "tools/rslg_pipeline/object_interface_schema.py",
    "tools/rslg_pipeline/layer2_candidate_integration_review.py",
    "tools/rslg_pipeline/build_route_contracts.py",
    "tools/rslg_pipeline/candidate_route_contract_schema.py",
    "tools/rslg_pipeline/build_route_plans.py",
    "tools/rslg_pipeline/route_plan_schema.py",
    "tools/rslg_pipeline/route_generation_readiness_review.py",
    "tools/rslg_pipeline/layer3_boundary_review.py",
]

COMMON_FINAL_LAYER2_FOR_ROOM = [
    "final stable occupancy map package",
    "final stable map metadata",
    "final vertical connector artifact",
    "final connector graph artifact",
    "final cross-floor topology artifact",
    "final route planner graph or planner-ready topology package",
]

OBJECT_FINAL_LAYER2_ADDITIONS = [
    "final object query resolution artifact",
    "final object approach artifact",
    "final object interface package",
    "approach feasibility / clearance validation evidence",
]


def _path_records(repo_root: Path, paths: Iterable[str]) -> list[dict[str, Any]]:
    return [
        {
            "path": path,
            "exists": repo_path(repo_root, path).is_file(),
        }
        for path in paths
    ]


def _canonical_output_root(scene_id: str) -> str:
    return f"stage_outputs/rslg_slam/{scene_id}/canonical/"


def _artifact_category_policy(scene_id: str) -> dict[str, Any]:
    task_evidence_pattern = f"stage_outputs/stage1_generalization/{scene_id}/tasks/<task_id>/"
    return {
        "task_evidence_artifacts": {
            "path_pattern": task_evidence_pattern,
            "description": "Task-local audit evidence produced by individual review, dry-run, or validation tasks.",
            "reproducible": True,
            "deletable": True,
            "permanent_project_truth": False,
            "canonical_final_output": False,
            "safe_for_task_validation_and_audit_only": True,
        },
        "candidate_artifacts": {
            "description": "Non-final generated artifacts used to validate interfaces and schemas before finalization.",
            "examples": [
                "Layer 2 vertical connector candidate artifacts",
                "Layer 2 stable map package candidate artifacts",
                "Layer 2 object interface candidate artifacts",
                "Layer 3 candidate route contracts",
                "Layer 3 route plan previews",
            ],
            "useful_for_interface_validation": True,
            "useful_for_schema_validation": True,
            "final_route_planner_inputs": False,
            "runtime_inputs": False,
            "real_astar_consumption_allowed": False,
            "promotion_or_regeneration_required_before_real_astar": True,
        },
        "final_canonical_artifacts": {
            "description": "Future final artifacts written only after sufficient current Layer 1 / Layer 2 evidence exists.",
            "generated_in_task33": False,
            "proposed_root": _canonical_output_root(scene_id),
            "consumed_by_real_route_generation_after_finalization": True,
        },
        "runtime_artifacts": {
            "description": "Future Layer 4 evidence and runtime-facing packages produced only after executable routes exist.",
            "examples": [
                "executable route package",
                "runtime input package",
                "RViz overlay inputs",
                "validation logs",
                "runtime reports",
            ],
            "generated_in_task33": False,
            "must_not_be_project_truth": True,
        },
    }


def canonical_output_plan(scene_id: str) -> dict[str, Any]:
    root = _canonical_output_root(scene_id)
    subdirectories = [
        "layer0_input/",
        "layer1_world_model/",
        "layer2_formal_artifacts/",
        "layer3_navigation_interface/",
        "layer4_runtime_validation/",
        "manifests/",
        "logs/",
    ]
    return {
        "plan_only": True,
        "proposed_canonical_output_root": root,
        "root_policy": "generated output separate from task evidence and not permanent docs/manifests truth",
        "subdirectories": [
            {
                "path": f"{root}{subdir}",
                "created_in_task33": False,
                "intended_role": role,
            }
            for subdir, role in [
                ("layer0_input/", "future input references and provenance"),
                ("layer1_world_model/", "future World Model Layer outputs and provenance"),
                ("layer2_formal_artifacts/", "future final formal artifacts consumed by Layer 3"),
                ("layer3_navigation_interface/", "future route contracts, planner requests, and route outputs"),
                ("layer4_runtime_validation/", "future runtime packages, logs, overlays, and validation reports"),
                ("manifests/", "future generated-output manifests for this canonical run"),
                ("logs/", "future command and validation logs for this canonical run"),
            ]
        ],
        "subdirectory_names": subdirectories,
        "directories_created": False,
        "final_artifacts_generated": False,
        "runtime_artifacts_generated": False,
        "task_evidence_consumption_policy": "future launch scripts should consume canonical outputs, not task evidence directories",
    }


def candidate_to_final_decision_matrix() -> list[dict[str, Any]]:
    return [
        {
            "candidate_artifact_family": "vertical connector candidate",
            "decision": "must_regenerate_from_current_layer1_outputs",
            "reason": "The candidate encodes validated milestone truth, but real geometry/centerline must be regenerated or verified from current World Model Layer/formal evidence before finalization.",
        },
        {
            "candidate_artifact_family": "connector graph candidate",
            "decision": "must_regenerate_from_current_layer2_inputs",
            "reason": "The final graph depends on the final vertical connector artifact and final topology or planner graph inputs.",
        },
        {
            "candidate_artifact_family": "cross-floor topology candidate",
            "decision": "must_regenerate_from_current_layer2_inputs",
            "reason": "The final topology depends on final room topology, connector graph, and stable map/planner compatibility.",
        },
        {
            "candidate_artifact_family": "stable occupancy map package candidate",
            "decision": "must_regenerate_from_current_layer1_outputs",
            "reason": "The current candidate is metadata-only and includes no pixels, PGM, YAML, or final map metadata.",
        },
        {
            "candidate_artifact_family": "object query resolution candidate",
            "decision": "must_regenerate_from_current_layer1_outputs",
            "reason": "Current evidence is validated milestone truth rather than a regenerated current object-resolution pass from World Model Layer outputs.",
        },
        {
            "candidate_artifact_family": "object approach candidate",
            "decision": "requires_runtime_or_planner_validation_before_final",
            "reason": "generated_ring_037 is useful as a candidate, but approach geometry and feasibility were not regenerated or revalidated.",
        },
        {
            "candidate_artifact_family": "object interface package candidate",
            "decision": "must_regenerate_from_current_layer2_inputs",
            "reason": "The package depends on final object query resolution and final object approach artifacts.",
        },
        {
            "candidate_artifact_family": "candidate route contracts",
            "decision": "must_regenerate_from_final_layer2_inputs",
            "reason": "The current candidate route contracts depend on candidate Layer 2 artifacts, not final formal artifacts.",
        },
        {
            "candidate_artifact_family": "route plan previews",
            "decision": "must_regenerate_from_final_route_contracts_and_planner_inputs",
            "reason": "The current previews are dry-run plans, not real planner output.",
        },
    ]


def _minimal_final_layer2_artifact_set() -> dict[str, Any]:
    return {
        "cross_floor_room": COMMON_FINAL_LAYER2_FOR_ROOM,
        "cross_floor_object": COMMON_FINAL_LAYER2_FOR_ROOM + OBJECT_FINAL_LAYER2_ADDITIONS,
        "readiness": REAL_ROUTE_BLOCKED,
    }


def _minimal_layer3_artifact_set() -> list[str]:
    return [
        "final route contract for cross_floor_room",
        "final route contract for cross_floor_object",
        "final route plan or planner request artifact",
        "real A* route output",
        "executable route candidate package",
    ]


def _minimal_layer4_artifact_set() -> list[str]:
    return [
        "runtime input package",
        "RViz overlay package",
        "launch/config package",
        "execution log",
        "validation report",
    ]


def _blocked_generation_steps() -> list[dict[str, Any]]:
    return [
        {
            "step": "final Layer 2 artifact generation",
            "status": "blocked",
            "blocked_reason": "task33 is boundary planning only; task34 should define the minimal final generation plan",
        },
        {
            "step": "final route contract generation",
            "status": "blocked",
            "blocked_reason": "requires final Layer 2 formal artifacts",
        },
        {
            "step": "real A* route generation",
            "status": "blocked",
            "blocked_reason": "requires final Layer 2 artifacts and real planner inputs",
        },
        {
            "step": "executable route generation",
            "status": "blocked",
            "blocked_reason": "requires a real route output",
        },
        {
            "step": "runtime validation",
            "status": "blocked",
            "blocked_reason": "requires executable route and runtime inputs",
        },
        {
            "step": "one-command full pipeline launch",
            "status": "blocked",
            "blocked_reason": "should not be implemented until final artifacts and real route generation are ready",
        },
    ]


def _one_command_launch_implications(scene_id: str) -> dict[str, Any]:
    return {
        "future_phase_scripts": [
            "run_layer0_input_check.sh",
            "run_layer1_world_model.sh",
            "run_layer2_formal_artifacts.sh",
            "run_layer3_navigation_interface.sh",
            "run_layer4_runtime_validation.sh",
        ],
        "future_full_wrapper": "run_rslg_pipeline_full.sh",
        "implemented_in_task33": False,
        "should_consume": _canonical_output_root(scene_id),
        "should_not_consume": f"stage_outputs/stage1_generalization/{scene_id}/tasks/<task_id>/",
        "blocked_until": [
            "final Layer 2 artifact generation exists",
            "real route generation exists",
            "runtime input package contract exists",
        ],
    }


def _claim_boundary_checks() -> dict[str, Any]:
    return {
        "ok": True,
        "review_only": True,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "final_route_plans_generated": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "stable_map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "connector_geometry_regenerated": False,
        "object_approach_geometry_regenerated": False,
        "approach_feasibility_revalidated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "external_gt_map_used": False,
        "simulator_navmesh_used": False,
        "dataset_side_slam_or_localization_accuracy_claimed": False,
        "amcl_success_claimed": False,
    }


def build_final_artifact_boundary_review(repo_root: Path, scene_id: str) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []

    report = {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": COMPLETED_CLASSIFICATION if not errors else FAILED_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "review_scope": {
            "scope": "boundary between task evidence, candidate artifacts, final canonical artifacts, and runtime artifacts",
            "review_only": True,
            "generation_task": False,
            "real_route_generation_readiness": REAL_ROUTE_BLOCKED,
        },
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_layer2_artifacts_generated": False,
        "final_layer3_route_contracts_generated": False,
        "final_route_plans_generated": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "artifact_category_policy": _artifact_category_policy(scene_id),
        "canonical_output_plan": canonical_output_plan(scene_id),
        "candidate_to_final_decision_matrix": candidate_to_final_decision_matrix(),
        "minimal_final_layer2_artifact_set_for_real_route_generation": _minimal_final_layer2_artifact_set(),
        "minimal_layer3_artifact_set_after_final_layer2": _minimal_layer3_artifact_set(),
        "minimal_layer4_artifact_set_after_executable_route": _minimal_layer4_artifact_set(),
        "blocked_generation_steps": _blocked_generation_steps(),
        "one_command_launch_implications": _one_command_launch_implications(scene_id),
        "real_route_generation_readiness": REAL_ROUTE_BLOCKED,
        "recommended_next_phase": RECOMMENDED_NEXT_PHASE,
        "recommended_next_task": RECOMMENDED_NEXT_TASK,
        "claim_boundary_checks": _claim_boundary_checks(),
        "docs_and_manifests_read": _path_records(repo_root, DOCS_AND_MANIFESTS_READ),
        "tools_inspected": _path_records(repo_root, TOOLS_INSPECTED),
        "errors": errors,
        "warnings": warnings,
    }
    return report


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Review RSLG-SLAM final artifact boundary and canonical output plan.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--output-json", required=True, help="Write the boundary review report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    try:
        report = build_final_artifact_boundary_review(repo_root, args.scene_id)
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        report = {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "classification": FAILED_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "scene_id": args.scene_id,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "final_layer2_artifacts_generated": False,
            "final_layer3_route_contracts_generated": False,
            "final_route_plans_generated": False,
            "real_astar_route_generated": False,
            "astar_waypoints_generated": False,
            "executable_route_generated": False,
            "runtime_input_package_generated": False,
            "runtime_artifacts_generated": False,
            "errors": [str(exc)],
            "warnings": [],
        }

    output_path = repo_path(repo_root, args.output_json)
    save_json(output_path, report)
    print(
        json.dumps(
            {
                "classification": report["classification"],
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
                "real_route_generation_readiness": report.get("real_route_generation_readiness"),
                "output_json": normalize_repo_relative(output_path, repo_root),
            },
            indent=2,
            sort_keys=False,
        )
    )
    return 0 if report["classification"] == COMPLETED_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
