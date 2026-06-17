"""Plan the minimal final Layer 2 generation chain for RSLG-SLAM.

This module is a planning/preflight utility only. It does not generate final
Layer 1 outputs, final Layer 2 formal artifacts, final Layer 3 route
contracts, route plans, real A* routes, waypoints, runtime input packages, map
pixels, connector geometry, object approach geometry, or runtime evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

from .common import PROJECT_NAME, normalize_repo_relative, repo_path, resolve_repo_root, save_json


COMPLETED_CLASSIFICATION = "final_layer2_minimal_generation_plan_completed"
FAILED_CLASSIFICATION = "final_layer2_minimal_generation_plan_failed"

ARTIFACT_LAYER = "Layer 2: Formal Artifact Layer"
REAL_ROUTE_READINESS = "real_route_generation_blocked_until_minimal_final_layer2_chain_exists"
RECOMMENDED_NEXT_PHASE = "layer1_world_model_canonical_rerun_preflight"
RECOMMENDED_NEXT_TASK = "task35_layer1_world_model_canonical_rerun_preflight_and_input_contract"

ROUTE_CONTEXTS = {
    "cross_floor_room": [
        "room_2 on floor_1",
        "room_3 on floor_1",
        "vt_1 / vc_vt_1 connector",
        "room_7 on floor_2",
        "room_13",
        "room_14",
    ],
    "cross_floor_object": [
        "room_2 on floor_1",
        "room_3 on floor_1",
        "vt_1 / vc_vt_1 connector",
        "room_7 on floor_2",
        "room_13",
        "room_14",
        "approach generated_ring_037 near obj_175",
    ],
}

DOCS_AND_MANIFESTS_READ = [
    "docs/rslg_slam/project_contract.md",
    "docs/rslg_slam/pipeline_architecture.md",
    "docs/rslg_slam/workspace_contract.md",
    "docs/rslg_slam/manifest_retention_policy.md",
    "docs/rslg_slam/canonical_output_plan.md",
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
    "tools/rslg_pipeline/build_world_model.py",
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
    "tools/rslg_pipeline/final_artifact_boundary_review.py",
    "tools/rslg_pipeline/layer3_boundary_review.py",
]


def _path_records(repo_root: Path, paths: Iterable[str]) -> list[dict[str, Any]]:
    return [{"path": path, "exists": repo_path(repo_root, path).is_file()} for path in paths]


def _canonical_path(canonical_root: str, relative_path: str) -> str:
    return f"{canonical_root.rstrip('/')}/{relative_path.lstrip('/')}"


def _minimal_generation_dag() -> dict[str, Any]:
    nodes = [
        {
            "id": "layer1_world_model_canonical_outputs",
            "layer": "Layer 1: World Model Layer",
            "depends_on": [],
            "purpose": "Produce or locate current canonical World Model Layer evidence used by final Layer 2 builders.",
            "status_in_task34": "not_generated_required_before_final_layer2_generation",
            "must_eventually_provide": [
                "stable map source evidence",
                "object interface regeneration evidence",
                "connector/topology validation evidence",
                "artifact provenance roots",
            ],
        },
        {
            "id": "final_stable_occupancy_map_package",
            "layer": ARTIFACT_LAYER,
            "depends_on": ["layer1_world_model_canonical_outputs"],
            "purpose": "Create the final planner-compatible stable occupancy map package.",
            "must_eventually_produce": [
                "final stable occupancy map raster or equivalent planner map",
                "final stable map metadata",
                "final map provenance",
                "explicit distinction from semantic floorplan, room mask, runtime costmap, external GT map, and simulator navmesh",
            ],
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_vertical_connector_artifact",
            "layer": ARTIFACT_LAYER,
            "depends_on": ["layer1_world_model_canonical_outputs"],
            "purpose": "Finalize or verify the cross-floor connector artifact from current Layer 1 evidence.",
            "must_eventually_produce_or_verify": [
                "connector id vt_1 / vc_vt_1",
                "floor_1 -> floor_2",
                "transition edge vt_1_centerline_e001",
                "non-transition edge vt_1_centerline_e003",
                "connector centerline / topology evidence if available",
                "geometry/provenance status",
            ],
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_connector_graph_artifact",
            "layer": ARTIFACT_LAYER,
            "depends_on": ["final_vertical_connector_artifact", "final_room_floor_topology"],
            "purpose": "Build the final connector graph from final connector and topology inputs.",
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_cross_floor_topology_artifact",
            "layer": ARTIFACT_LAYER,
            "depends_on": [
                "final_vertical_connector_artifact",
                "final_connector_graph_artifact",
                "final_room_floor_topology",
                "final_stable_occupancy_map_package_if_required_for_planner_compatibility",
            ],
            "purpose": "Bind room/floor topology to the final vertical connector graph for cross-floor route contexts.",
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_route_planner_graph_or_topology_package",
            "layer": ARTIFACT_LAYER,
            "depends_on": [
                "final_stable_occupancy_map_package",
                "final_connector_graph_artifact",
                "final_cross_floor_topology_artifact",
            ],
            "purpose": "Provide the final planner-facing graph/topology package required before Layer 3 real route generation.",
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_object_query_resolution_artifact",
            "layer": ARTIFACT_LAYER,
            "depends_on": ["layer1_world_model_canonical_outputs"],
            "purpose": "Resolve the target object query from current canonical World Model Layer evidence.",
            "must_eventually_resolve": {
                "query": "curtain in room_14 on floor_2",
                "object_id": "obj_175",
                "object_label": "curtain",
                "target_room": "room_14",
                "target_floor": "floor_2",
            },
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_object_approach_artifact",
            "layer": ARTIFACT_LAYER,
            "depends_on": [
                "final_object_query_resolution_artifact",
                "final_stable_occupancy_map_package",
                "final_room_floor_topology",
                "approach_feasibility_clearance_validation",
            ],
            "purpose": "Finalize an approach candidate for the object route without using the object centroid as the navigation goal.",
            "must_preserve": {
                "object_centroid_navigation_used": False,
                "direct_object_centroid_goal_used": False,
                "candidate_seed_allowed": "generated_ring_037",
                "final_requires_regenerated_or_validated_approach_evidence": True,
            },
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_object_interface_package",
            "layer": ARTIFACT_LAYER,
            "depends_on": ["final_object_query_resolution_artifact", "final_object_approach_artifact"],
            "purpose": "Package final object query and approach artifacts for Layer 3 object-route contracts.",
            "status_in_task34": "not_generated",
        },
        {
            "id": "final_layer3_route_contract_generation_ready",
            "layer": "Layer 3: Navigation Interface Layer",
            "depends_on_by_route_kind": {
                "cross_floor_room": [
                    "final_stable_occupancy_map_package",
                    "final_vertical_connector_artifact",
                    "final_connector_graph_artifact",
                    "final_cross_floor_topology_artifact",
                    "final_route_planner_graph_or_topology_package",
                ],
                "cross_floor_object": [
                    "final_stable_occupancy_map_package",
                    "final_vertical_connector_artifact",
                    "final_connector_graph_artifact",
                    "final_cross_floor_topology_artifact",
                    "final_route_planner_graph_or_topology_package",
                    "final_object_query_resolution_artifact",
                    "final_object_approach_artifact",
                    "final_object_interface_package",
                ],
            },
            "purpose": "Mark the exact point at which final Layer 3 route contracts may be regenerated from final Layer 2 inputs.",
            "status_in_task34": "not_ready_not_generated",
        },
    ]
    return {
        "graph_type": "minimal_dependency_dag",
        "plan_only": True,
        "nodes": nodes,
        "edges": [
            {"from": dependency, "to": node["id"]}
            for node in nodes
            for dependency in node.get("depends_on", [])
        ]
        + [
            {"from": dependency, "to": "final_layer3_route_contract_generation_ready", "route_kind": route_kind}
            for route_kind, dependencies in nodes[-1]["depends_on_by_route_kind"].items()
            for dependency in dependencies
        ],
    }


def _layer1_source_requirements() -> dict[str, Any]:
    return {
        "task34_verifies_existence": False,
        "task34_reruns_layer1": False,
        "minimal_required_outputs": [
            "posed RGB-D derived world model evidence",
            "floor assignment / floor height evidence",
            "free-space / wall / outside-boundary evidence",
            "gateway / wall evidence",
            "room/floor topology evidence",
            "object observation / semantic object evidence",
            "object-room and object-floor association evidence",
            "provenance records for each derived formal artifact",
        ],
        "source_boundary": {
            "allowed_source": "RGB-D plus provided pose through the World Model Layer",
            "external_gt_floorplan_used": False,
            "external_gt_occupancy_map_used": False,
            "manual_stair_centerline_used": False,
            "manual_object_target_pose_used": False,
            "simulator_navmesh_used": False,
        },
    }


def _artifact_specs(canonical_root: str) -> list[dict[str, Any]]:
    specs = [
        (
            "stable_occupancy_map_package_v0_1",
            "layer2_formal_artifacts/stable_maps/stable_occupancy_map_package_v0_1.json",
            ["layer1_world_model_canonical_outputs"],
            ["schema validation", "planner compatibility", "map provenance", "map distinction policy"],
            "must_regenerate_from_current_layer1_outputs",
            True,
            True,
            True,
        ),
        (
            "vertical_connectors_v0_1",
            "layer2_formal_artifacts/vertical_connectors/vertical_connectors_v0_1.json",
            ["layer1_world_model_canonical_outputs"],
            ["connector ids", "floor_1_to_floor_2", "transition edge truth", "geometry/provenance status"],
            "must_regenerate_or_verify_from_current_layer1_outputs",
            True,
            True,
            True,
        ),
        (
            "stairs_or_vertical_connector_graph_v0_1",
            "layer2_formal_artifacts/vertical_connectors/stairs_or_vertical_connector_graph_v0_1.json",
            ["vertical_connectors_v0_1", "final room/floor topology"],
            ["graph connectivity", "connector edge binding", "non-transition edge not used as transition"],
            "must_regenerate_from_final_layer2_inputs",
            True,
            True,
            True,
        ),
        (
            "cross_floor_topology_v0_1",
            "layer2_formal_artifacts/topology/cross_floor_topology_v0_1.json",
            ["vertical_connectors_v0_1", "stairs_or_vertical_connector_graph_v0_1", "final room/floor topology"],
            ["route context rooms", "floor transition binding", "planner compatibility if required"],
            "must_regenerate_from_final_layer2_inputs",
            True,
            True,
            True,
        ),
        (
            "route_planner_graph_v0_1",
            "layer2_formal_artifacts/planner_graph/route_planner_graph_v0_1.json",
            ["stable_occupancy_map_package_v0_1", "stairs_or_vertical_connector_graph_v0_1", "cross_floor_topology_v0_1"],
            ["coordinate consistency", "resolution/origin compatibility", "cross-floor edge availability"],
            "must_generate_from_final_layer2_inputs",
            True,
            True,
            True,
        ),
        (
            "object_query_resolution_v0_1",
            "layer2_formal_artifacts/object_interfaces/object_query_resolution_v0_1.json",
            ["layer1_world_model_canonical_outputs"],
            ["query/object id", "label", "room/floor association", "provenance"],
            "must_regenerate_or_verify_from_current_layer1_outputs",
            True,
            False,
            True,
        ),
        (
            "object_approach_v0_1",
            "layer2_formal_artifacts/object_interfaces/object_approach_v0_1.json",
            ["object_query_resolution_v0_1", "stable_occupancy_map_package_v0_1", "final room/floor topology"],
            ["approach feasibility", "clearance validation", "centroid-goal prohibition"],
            "candidate_seed_allowed_but_final_requires_regenerated_or_validated_approach_evidence",
            True,
            False,
            True,
        ),
        (
            "object_interface_package_v0_1",
            "layer2_formal_artifacts/object_interfaces/object_interface_package_v0_1.json",
            ["object_query_resolution_v0_1", "object_approach_v0_1"],
            ["package references", "object truth", "approach safety flags"],
            "must_regenerate_from_final_object_query_and_approach_artifacts",
            True,
            False,
            True,
        ),
    ]
    return [
        {
            "artifact_name": name,
            "intended_canonical_path": _canonical_path(canonical_root, path),
            "required_inputs": inputs,
            "required_validation_checks": checks,
            "promotion_or_regeneration_decision": decision,
            "missing_current_layer1_outputs_block_it": layer1_blocked,
            "needed_for_cross_floor_room": room_needed,
            "needed_for_cross_floor_object": object_needed,
            "generated_in_task34": False,
        }
        for name, path, inputs, checks, decision, layer1_blocked, room_needed, object_needed in specs
    ]


def _promotion_vs_regeneration_policy() -> list[dict[str, Any]]:
    return [
        {"artifact_family": "stable map candidate", "decision": "must_regenerate_from_current_layer1_outputs"},
        {"artifact_family": "vertical connector candidate", "decision": "must_regenerate_or_verify_from_current_layer1_outputs"},
        {"artifact_family": "connector graph candidate", "decision": "must_regenerate_from_final_layer2_inputs"},
        {"artifact_family": "cross-floor topology candidate", "decision": "must_regenerate_from_final_layer2_inputs"},
        {"artifact_family": "object query resolution candidate", "decision": "must_regenerate_or_verify_from_current_layer1_outputs"},
        {"artifact_family": "object approach candidate", "decision": "requires_approach_feasibility_clearance_validation_before_final"},
        {"artifact_family": "object interface package candidate", "decision": "must_regenerate_from_final_object_query_and_approach_artifacts"},
        {"artifact_family": "route contract candidates", "decision": "must_regenerate_from_final_layer2_inputs"},
        {"artifact_family": "route plan previews", "decision": "must_regenerate_from_final_route_contracts_and_planner_inputs"},
    ]


def _canonical_output_write_plan(canonical_root: str) -> dict[str, Any]:
    subdirs = [
        "layer1_world_model/",
        "layer2_formal_artifacts/stable_maps/",
        "layer2_formal_artifacts/vertical_connectors/",
        "layer2_formal_artifacts/topology/",
        "layer2_formal_artifacts/object_interfaces/",
        "layer2_formal_artifacts/planner_graph/",
        "layer3_navigation_interface/route_contracts/",
        "layer3_navigation_interface/route_plans/",
        "layer3_navigation_interface/real_routes/",
        "layer4_runtime_validation/runtime_inputs/",
        "layer4_runtime_validation/rviz/",
        "layer4_runtime_validation/logs/",
        "manifests/",
    ]
    return {
        "plan_only": True,
        "canonical_root": canonical_root,
        "directories_created_in_task34": False,
        "task_evidence_directories_are_canonical_final_output_directories": False,
        "future_one_command_launch_scripts_must_consume_canonical_outputs": True,
        "future_one_command_launch_scripts_must_not_consume_task_evidence_outputs": True,
        "proposed_paths": [{"path": _canonical_path(canonical_root, subdir), "created_in_task34": False} for subdir in subdirs],
    }


def _final_artifact_validation_plan() -> list[dict[str, Any]]:
    return [
        {"sequence": 1, "validator": "stable map schema validation", "current_status": "exists_for_candidate_artifacts", "final_mode_extension_needed": True},
        {"sequence": 2, "validator": "vertical connector schema validation", "current_status": "exists_for_dry_run_and_candidate_artifacts", "final_mode_extension_needed": True},
        {"sequence": 3, "validator": "object interface schema validation", "current_status": "exists_for_dry_run_and_candidate_artifacts", "final_mode_extension_needed": True},
        {"sequence": 4, "validator": "final Layer 2 integration validation", "current_status": "candidate_integration_review_exists", "final_mode_extension_needed": True},
        {"sequence": 5, "validator": "final route contract schema validation", "current_status": "candidate_route_contract_validation_exists", "final_mode_extension_needed": True},
        {"sequence": 6, "validator": "route plan schema validation", "current_status": "preview_validation_exists", "final_mode_extension_needed": True},
        {"sequence": 7, "validator": "route generation readiness validation", "current_status": "readiness_review_exists", "final_mode_extension_needed": True},
        {"sequence": 8, "validator": "JSON/static validation", "current_status": "static validator exists", "final_mode_extension_needed": False},
    ]


def _real_route_generation_unblock_conditions() -> dict[str, Any]:
    return {
        "cross_floor_room": [
            "final stable map package exists and is planner-compatible",
            "final vertical connector/topology artifacts exist",
            "final route planner graph exists",
            "final route contract can be regenerated from final Layer 2 inputs",
            "route plan can be regenerated from final route contract and planner graph",
        ],
        "cross_floor_object": [
            "all cross_floor_room unblock conditions",
            "final object interface package exists",
            "final object approach artifact exists",
            "approach feasibility / clearance validation evidence exists",
        ],
        "current_task34_status": REAL_ROUTE_READINESS,
    }


def _risk_and_blocker_analysis() -> list[dict[str, Any]]:
    return [
        {"risk": "final stable map may require current Layer 1 / World Model Layer rerun", "severity": "high", "mitigation": "run a Layer 1 canonical rerun preflight before final Layer 2 generation"},
        {"risk": "connector geometry cannot be finalized from candidate-only evidence", "severity": "high", "mitigation": "regenerate or verify connector geometry from current Layer 1 evidence"},
        {"risk": "object approach candidate may not be planner-feasible", "severity": "high", "mitigation": "require approach feasibility / clearance validation before final object interface packaging"},
        {"risk": "route planner graph may need stable map resolution/origin/coordinate consistency", "severity": "medium", "mitigation": "validate stable map metadata against topology and graph coordinates"},
        {"risk": "canonical output paths may be confused with task evidence", "severity": "medium", "mitigation": "write final artifacts only under the canonical root in future generation tasks"},
        {"risk": "final route generation may be mistaken for runtime success", "severity": "medium", "mitigation": "keep Layer 3 route generation claims separate from Layer 4 runtime validation claims"},
    ]


def _claim_boundary_checks() -> dict[str, Any]:
    return {
        "ok": True,
        "plan_only": True,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "canonical_directories_created": False,
        "final_layer1_outputs_generated": False,
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
        "external_gt_floorplan_used": False,
        "external_gt_occupancy_map_used": False,
        "manual_stair_centerline_used": False,
        "manual_object_target_pose_used": False,
        "simulator_navmesh_used": False,
        "dataset_side_slam_or_localization_accuracy_claimed": False,
        "amcl_success_claimed": False,
    }


def build_final_layer2_generation_plan(repo_root: Path, scene_id: str, canonical_root: str) -> dict[str, Any]:
    errors: list[str] = []
    warnings = [
        "Task34 does not verify that current canonical Layer 1 outputs exist.",
        "Existing Layer 2 and Layer 3 validators are candidate/preview-oriented and need final-mode extensions.",
    ]
    return {
        "project_name": PROJECT_NAME,
        "repo_root": repo_root.as_posix(),
        "classification": COMPLETED_CLASSIFICATION if not errors else FAILED_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "scene_id": scene_id,
        "canonical_root": canonical_root,
        "plan_scope": {
            "scope": "minimal final Layer 2 artifact generation chain required before real A* route generation can begin",
            "planning_preflight_only": True,
            "final_artifact_generation_task": False,
            "route_contexts": ROUTE_CONTEXTS,
        },
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "canonical_directories_created": False,
        "final_layer1_outputs_generated": False,
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
        "minimal_generation_dag": _minimal_generation_dag(),
        "layer1_source_requirements": _layer1_source_requirements(),
        "final_layer2_artifact_specs": _artifact_specs(canonical_root),
        "promotion_vs_regeneration_policy": _promotion_vs_regeneration_policy(),
        "canonical_output_write_plan": _canonical_output_write_plan(canonical_root),
        "final_artifact_validation_plan": _final_artifact_validation_plan(),
        "real_route_generation_unblock_conditions": _real_route_generation_unblock_conditions(),
        "risk_and_blocker_analysis": _risk_and_blocker_analysis(),
        "recommended_execution_order": [
            "layer1_world_model_canonical_rerun_preflight",
            "layer1_world_model_canonical_outputs",
            "final_stable_occupancy_map_package",
            "final_vertical_connector_artifact",
            "final_connector_graph_artifact",
            "final_cross_floor_topology_artifact",
            "final_route_planner_graph_or_topology_package",
            "final_object_query_resolution_artifact",
            "final_object_approach_artifact",
            "final_object_interface_package",
            "final_layer3_route_contract_generation_ready",
        ],
        "recommended_next_phase": RECOMMENDED_NEXT_PHASE,
        "recommended_next_task": RECOMMENDED_NEXT_TASK,
        "real_route_generation_readiness": REAL_ROUTE_READINESS,
        "claim_boundary_checks": _claim_boundary_checks(),
        "docs_and_manifests_read": _path_records(repo_root, DOCS_AND_MANIFESTS_READ),
        "tools_inspected": _path_records(repo_root, TOOLS_INSPECTED),
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Create the RSLG-SLAM final Layer 2 minimal generation plan.")
    parser.add_argument("--repo-root", default=None, help="Repository root path.")
    parser.add_argument("--scene-id", required=True, help="Scene identifier, for example 00843-DYehNKdT76V.")
    parser.add_argument("--canonical-root", required=True, help="Future canonical generated-output root. Not created.")
    parser.add_argument("--output-json", required=True, help="Write the planning report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    repo_root = resolve_repo_root(args.repo_root)
    try:
        report = build_final_layer2_generation_plan(repo_root, args.scene_id, args.canonical_root)
    except Exception as exc:  # pragma: no cover - defensive CLI boundary
        report = {
            "project_name": PROJECT_NAME,
            "repo_root": repo_root.as_posix(),
            "classification": FAILED_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "scene_id": args.scene_id,
            "canonical_root": args.canonical_root,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "canonical_directories_created": False,
            "final_layer1_outputs_generated": False,
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
                "recommended_next_phase": report.get("recommended_next_phase"),
                "recommended_next_task": report.get("recommended_next_task"),
                "output_json": normalize_repo_relative(output_path, repo_root),
            },
            indent=2,
            sort_keys=False,
        )
    )
    return 0 if report["classification"] == COMPLETED_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
