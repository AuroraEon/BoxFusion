#!/usr/bin/env python3
"""Generate the task25a reintegration preflight report bundle.

This script is intentionally read-only with respect to Stage-A and task24
artifacts. It only writes the task25a output directory.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict


REPO = Path("/home/ws/workspace/BoxFusion")
SCENE = "00843-DYehNKdT76V"
TASK_ROOT = (
    REPO
    / "stage_outputs/stage1_generalization"
    / SCENE
    / "tasks/task25a_stage_a_vertical_connector_and_object_interface_reintegration_preflight"
)
CLEAN_ROOT = REPO / "stage_outputs/stage1_generalization" / SCENE / "clean_rerun"
RAW_ROOT = CLEAN_ROOT / "canonical_stage1/raw_outputs" / SCENE
COMMITTED_PUBLIC = CLEAN_ROOT / "committed_public"
TASKS_ROOT = REPO / "stage_outputs/stage1_generalization" / SCENE / "tasks"


def read_json(path: Path, default: Any = None) -> Any:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return default


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
        f.write("\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def exists(path: Path) -> bool:
    return path.exists()


def rel(path: Path) -> str:
    return str(path)


def build_inventory() -> Dict[str, Any]:
    artifact_paths = {
        "stage_a_vertical_transition_evidence_committed": COMMITTED_PUBLIC
        / "vertical_transition_evidence.json",
        "stage_a_topology_committed": COMMITTED_PUBLIC / "topology_v0_1.json",
        "stage_a_vector_map_committed": COMMITTED_PUBLIC / "final_vector_map_snapshot.json",
        "stage_a_room_world_model_committed": COMMITTED_PUBLIC
        / "committed_room_world_model_v0_1.json",
        "floor_1_stable_map_yaml": CLEAN_ROOT
        / "maps/floor_1/stage1_floor_1_stable_occupancy_map.yaml",
        "floor_1_stable_map_npz": CLEAN_ROOT
        / "maps/floor_1/stage1_floor_1_stable_occupancy_map.npz",
        "floor_2_stable_map_yaml": CLEAN_ROOT
        / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml",
        "floor_2_stable_map_npz": CLEAN_ROOT
        / "maps/floor_2/stage1_floor_2_stable_occupancy_map.npz",
        "task24c_vertical_connectors": TASKS_ROOT
        / "task24c_rslg_cross_floor_connector_router_and_overlay/vertical_connectors_v0_1.json",
        "task24c_cross_floor_topology": TASKS_ROOT
        / "task24c_rslg_cross_floor_connector_router_and_overlay/cross_floor_topology_v0_1.json",
        "task24d_corrected_cross_floor_topology": TASKS_ROOT
        / "task24d_cross_floor_rviz_overlay_adapter_with_transition_semantics_fix/corrected_cross_floor_topology_v0_2.json",
        "task24g2_occupancy_route": TASKS_ROOT
        / "task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/planned_occupancy_aware_3d_route_v0_1.json",
        "task24h2_tracking_summary": TASKS_ROOT
        / "task24h2_cross_floor_visual_proxy_tracking_evidence_hardening/task24h2_summary.json",
        "task24i_object_contract": TASKS_ROOT
        / "task24i_cross_floor_object_level_tracking_smoke/object_level_cross_floor_route_contract_v0_1.json",
        "task24i_object_query_resolution": TASKS_ROOT
        / "task24i_cross_floor_object_level_tracking_smoke/object_query_resolution_v0_1.json",
        "task24i_approach_candidate": TASKS_ROOT
        / "task24i_cross_floor_object_level_tracking_smoke/object_approach_candidate_v0_1.json",
    }
    evidence = read_json(artifact_paths["stage_a_vertical_transition_evidence_committed"], {})
    transitions = evidence.get("summary", {}).get("transitions", [])
    vt_1 = next((t for t in transitions if t.get("transition_id") == "vt_1"), None)
    return {
        "classification": "task25a_reintegration_preflight_completed",
        "scene_id": SCENE,
        "clean_rerun_root": rel(CLEAN_ROOT),
        "stage_a_raw_output": rel(RAW_ROOT),
        "artifact_presence": {
            name: {"path": rel(path), "exists": exists(path)}
            for name, path in artifact_paths.items()
        },
        "current_stage_a_validated_vertical_transition": vt_1,
        "validated_facts_to_preserve": {
            "vertical_connector": "vt_1",
            "route": [
                "floor_1 room_2",
                "floor_1 room_3",
                "vt_1 connector",
                "floor_2 room_7",
                "floor_2 room_13",
                "floor_2 room_14",
            ],
            "floor_transition_edge": "vt_1_centerline_e001",
            "not_floor_transition_edge": "vt_1_centerline_e003",
            "object_query": "curtain in room_14 on floor_2",
            "object_id": "obj_175",
            "approach_candidate": "generated_ring_037",
        },
        "protected_artifacts": {
            "modified_by_task25a": False,
            "paths": [
                rel(REPO / "stage_outputs/stage1_00824_step30p1"),
                "00824 reference baseline",
                rel(CLEAN_ROOT / "committed_public"),
                rel(CLEAN_ROOT / "maps/floor_1"),
                rel(CLEAN_ROOT / "maps/floor_2"),
                "task23b validated quadruped proxy demo",
                "task24h/task24h2/task24i evidence outputs",
                rel(RAW_ROOT),
            ],
        },
    }


STAGE_A_PRODUCER_AUDIT = {
    "classification": "stage_a_producer_locations_identified",
    "producers": [
        {
            "artifact_or_behavior": "Stage-A CLI and output root semantics",
            "file": "stage_a_demo.py",
            "functions_or_lines": [
                "argument parser around lines 468-545",
                "_run_single_sequence around lines 184-279",
            ],
            "current_behavior": "Loads dataset/config/model, creates ClosedLoopDemoRecorder, and runs one sequence into the requested output root.",
            "task25b_modification": "Use a staging output root for the full rerun. Do not point directly at clean_rerun until validation has passed.",
        },
        {
            "artifact_or_behavior": "vertical_transition_evidence.json, topology_v0_1.json, final vector map, committed room world files",
            "file": "boxfusion/stage_a_demo.py",
            "functions_or_lines": [
                "ClosedLoopDemoRecorder finalization around lines 1293-1410",
                "manifest/copy/export surfaces around lines 1515+",
            ],
            "current_behavior": "Writes final_vector_map_snapshot.json, topology_v0_1.json, vertical_transition_evidence.json, committed_room_world_model_v0_1.json, and snapshot outputs.",
            "task25b_modification": "Add formal connector/object-route artifact registration and export after vector map, topology, and maps are available.",
        },
        {
            "artifact_or_behavior": "floor-aware transition records, from_position_xy, to_position_xy, room/floor/object bindings",
            "file": "boxfusion/floor_aware_room_segmenter.py",
            "functions_or_lines": [
                "observe_frame around lines 247-288",
                "export vector map around lines 544-575",
                "_build_vertical_transitions around lines 2766-2860",
                "_build_object_export_record around lines 2271-2383",
            ],
            "current_behavior": "Builds vt_1 from floor assignment intervals, stable room entry/exit evidence, per-frame xy/z pose samples, and object room/floor binding exports.",
            "task25b_modification": "Preserve evidence fields and add or feed a formal centerline connector exporter; do not replace room/object binding semantics.",
        },
        {
            "artifact_or_behavior": "floor canonicalization and vertical transition public normalization",
            "file": "boxfusion/floor_artifacts.py",
            "functions_or_lines": [
                "canonicalize_floors around lines 67-91",
                "attach_floor_metadata around lines 102-127",
                "canonicalize_vertical_transition_record around lines 149-213",
                "build_vertical_transition_summary around lines 237-267",
            ],
            "current_behavior": "Normalizes Stage-A floor ids, floor display metadata, and vertical transition evidence summary.",
            "task25b_modification": "Add canonical helpers for vertical_connectors_v0_1.json and claim-boundary fields.",
        },
        {
            "artifact_or_behavior": "committed topology vertical transition room edges",
            "file": "boxfusion/room_topology.py",
            "functions_or_lines": [
                "RoomTopologyBuilder.build around lines 1225-1247",
                "_sync_containment_indices around lines 1342-1410",
                "_collect_edge_support around lines 1435-1446",
                "_collect_vertical_transition_support around lines 1627-1715",
            ],
            "current_behavior": "Creates a room_3 to room_7 vertical_transition edge supported by vt_1 evidence; current committed topology is room-level and does not contain centerline nodes.",
            "task25b_modification": "Attach formal connector ids and transition edge ids when the connector graph is produced; keep topology room-level.",
        },
        {
            "artifact_or_behavior": "floor-level stable occupancy map validation surface",
            "file": "tools/stage1_runtime/build_scene_floor_occupancy_map.py",
            "functions_or_lines": ["CLI validator/export helper"],
            "current_behavior": "Validates an existing generalized floor stable occupancy map package; it is not itself the original Stage-A map producer.",
            "task25b_modification": "Use for regression validation. If maps must regenerate, formalize the generation hook before relying on this validator.",
        },
        {
            "artifact_or_behavior": "ROS/public vertical transition marker bridge",
            "file": "boxfusion/ros_artifact_bridge.py",
            "functions_or_lines": [
                "vertical transition xy validation around lines 374-389",
                "endpoint/connector marker generation around lines 699-756",
            ],
            "current_behavior": "Consumes vertical transition evidence for ROS marker surfaces.",
            "task25b_modification": "Teach it to prefer formal vertical_connectors_v0_1.json when present.",
        },
    ],
    "not_currently_stage_a_outputs": [
        "vt_1 centerline node/edge graph",
        "transition_edge_id vt_1_centerline_e001",
        "cross-floor route query contract",
        "occupancy-aware room_2 to room_14 visual route",
        "object query resolution for curtain in room_14 on floor_2",
        "generated_ring_037 object approach candidate",
        "object-level route contract to obj_175 approach",
    ],
}


TASK24_LINEAGE = {
    "classification": "task24_lineage_reconstructed",
    "downstream_artifacts": [
        {
            "task": "task24b/task24b2/task24b3/task24b4",
            "role": "post-Stage-A vertical connector audit, pose trace extraction, sparse stair centerline fitting, floor_1 stable map correction/backfill",
            "formalization_recommendation": "Promote the validated connector centerline and floor map package interfaces into task25b formal export/validation surfaces; keep exploratory audit logs task-only.",
            "important_outputs": [
                "candidate_vertical_connectors_v0_1.json",
                "semantic_vertical_connector_candidates_v0_1.json",
                "vertical_transition_paths_v0_1.json",
                "stairs_graph_v0_3.json",
                "sparse_stair_connector_graph_vt_1_v0_3.json",
                "raw_transition_pose_trace_vt_1.json",
            ],
        },
        {
            "task": "task24c",
            "role": "built a connector router and cross-floor topology/query overlay from Stage-A evidence plus downstream centerline/map artifacts",
            "formalization_recommendation": "Promote vertical_connectors_v0_1.json and cross_floor_topology_v0_1.json to committed_public/formal route inputs after regeneration from formal producers.",
            "important_outputs": [
                "vertical_connectors_v0_1.json",
                "cross_floor_topology_v0_1.json",
                "cross_floor_route_query_report.json",
                "cross_floor_route_waypoints_v0_1.json",
            ],
        },
        {
            "task": "task24d",
            "role": "corrected per-edge transition semantics and RViz marker spec",
            "formalization_recommendation": "Preserve the invariant that vt_1_centerline_e001 is the floor transition edge and vt_1_centerline_e003 is not.",
            "important_outputs": [
                "corrected_cross_floor_topology_v0_2.json",
                "corrected_cross_floor_route_query_report_v0_2.json",
                "cross_floor_rviz_markers_v0_1.json",
            ],
        },
        {
            "task": "task24f/task24g/task24g2",
            "role": "visual-kinematic route contract and occupancy-aware same-floor A* plus stair connector centerline playback input",
            "formalization_recommendation": "Promote route-contract builders to consume formal connector and stable map packages; keep visual playback evidence task-only.",
            "important_outputs": [
                "cross_floor_3d_route_contract_v0_1.json",
                "planned_occupancy_aware_3d_route_v0_1.json",
            ],
        },
        {
            "task": "task24h/task24h2",
            "role": "room-level tracking evidence using integrated actual pose and SetEntityState as visual display only",
            "formalization_recommendation": "Do not make Gazebo/RViz evidence public Stage-A output; use it as downstream regression evidence after task25b.",
            "validated_facts": {
                "direct_playback": False,
                "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
                "z_start": 2.456,
                "z_end": 4.056,
                "z_delta": 1.6,
                "transition_edge": "vt_1_centerline_e001",
            },
        },
        {
            "task": "task24i",
            "role": "object-level smoke test to obj_175/generated_ring_037",
            "formalization_recommendation": "Promote query resolution, approach candidates, and object-level route contract schemas; keep visual tracking execution logs task-only.",
            "validated_facts": {
                "query": "curtain in room_14 on floor_2",
                "object_id": "obj_175",
                "approach_candidate": "generated_ring_037",
                "object_centroid_navigation_used": False,
                "object_append_source": "astar_over_floor_2_stable_occupancy_map",
                "final_approach_distance_m": 0.179242,
                "final_yaw_error_rad": 0.288844,
                "direct_playback": False,
                "set_entity_state_success_count": 2730,
                "transition_edge": "vt_1_centerline_e001",
            },
        },
    ],
}


VERTICAL_SCHEMA = {
    "schema_name": "vertical_connectors_v0_1",
    "artifact_type": "formal_stage_a_vertical_connector_graph",
    "required_top_level_fields": [
        "schema_version",
        "artifact_type",
        "scene_id",
        "sequence_id",
        "created_utc",
        "generator",
        "source_stage_a_run_id",
        "claim_boundary",
        "connectors",
    ],
    "claim_boundary": {
        "topological_vertical_transition_only": True,
        "physical_stair_climbing_supported": False,
        "gait_supported": False,
        "footstep_planning_supported": False,
        "contact_based_locomotion_supported": False,
        "slam_or_localization_accuracy_claimed": False,
        "nav2_or_amcl_execution_claimed": False,
    },
    "connector_record": {
        "connector_id": "vc_vt_1",
        "transition_id": "vt_1",
        "connector_type": "stairs_or_vertical_connector",
        "from_floor_id": "floor_1",
        "to_floor_id": "floor_2",
        "from_room_id": "room_3",
        "to_room_id": "room_7",
        "from_position_xy": [-5.262, 1.254],
        "to_position_xy": [-5.223, 5.437],
        "endpoint_from": {
            "node_id": "vt_1_centerline_n000",
            "floor_id": "floor_1",
            "room_id": "room_3",
            "position_xyz": ["x", "y", "z_start"],
        },
        "endpoint_to": {
            "node_id": "vt_1_centerline_n004",
            "floor_id": "floor_2",
            "room_id": "room_7",
            "position_xyz": ["x", "y", "z_end"],
        },
        "centerline_nodes": [
            {
                "node_id": "vt_1_centerline_n001",
                "floor_id": "floor_1",
                "position_xyz": ["x", "y", "z"],
                "source": "fitted_from_transition_pose_trace",
                "pose_trace_provenance": "raw_transition_pose_trace_vt_1.json",
            }
        ],
        "centerline_edges": [
            {
                "edge_id": "vt_1_centerline_e001",
                "source_node_id": "vt_1_centerline_n001",
                "target_node_id": "vt_1_centerline_n002",
                "source_floor_id": "floor_1",
                "target_floor_id": "floor_2",
                "distance_3d_m": "float",
                "delta_z_m": "float",
                "topological_transition_edge": True,
            }
        ],
        "transition_edge_id": "vt_1_centerline_e001",
        "transition_edge": {
            "source_node_id": "vt_1_centerline_n001",
            "target_node_id": "vt_1_centerline_n002",
            "source_floor_id": "floor_1",
            "target_floor_id": "floor_2",
        },
        "z_start": 2.456,
        "z_end": 4.056,
        "z_delta": 1.6,
        "evidence": {
            "stage_a_transition_record": "vertical_transition_evidence.json summary transition vt_1",
            "frame_start": 1516,
            "frame_end": 1538,
            "entry_stable_frame": 1515,
            "exit_stable_frame": 1539,
            "pose_trace_source": "Stage-A provided-pose RGB-D dataset side backend",
            "evidence_frames": "list or compact range of source frame ids",
        },
        "confidence": 0.95,
        "validation": {
            "source_target_node_floor_ids_present": True,
            "transition_edge_is_vt_1_centerline_e001": True,
            "vt_1_centerline_e003_is_not_floor_transition_edge": True,
        },
    },
}


OBJECT_SCHEMA = {
    "schema_name": "object_level_route_contract_v0_1",
    "artifact_type": "formal_object_level_cross_floor_route_contract",
    "required_top_level_fields": [
        "schema_version",
        "artifact_type",
        "scene_id",
        "query_text",
        "query_resolution",
        "approach",
        "route_contract",
        "validation_requirements",
        "claim_boundary",
    ],
    "query_resolution": {
        "query_text": "curtain in room_14 on floor_2",
        "object_id": "obj_175",
        "object_label": "curtain",
        "target_floor_id": "floor_2",
        "target_room_id": "room_14",
        "object_room_binding_source": "committed_room_world_model_v0_1 object room_assignment plus topology containment index",
        "object_centroid_navigation_used": False,
    },
    "approach": {
        "approach_candidate_id": "generated_ring_037",
        "approach_pose": {
            "x": -7.442624,
            "y": 2.055545,
            "z": 4.056,
            "yaw": -1.989675,
        },
        "facing_yaw": -1.989675,
        "generated_approach_candidate_used": True,
        "approach_candidate_source": "object_approach_candidates_v0_1 generated around object using floor_2 stable occupancy map",
        "approach_validation": {
            "final_distance_to_approach_pose_m_max": 0.25,
            "final_yaw_error_rad_max": 0.35,
            "wall_crossing_forbidden": True,
        },
    },
    "route_contract": {
        "start_floor_id": "floor_1",
        "start_room_id": "room_2",
        "room_level_route": [
            {"floor_id": "floor_1", "room_id": "room_2"},
            {"floor_id": "floor_1", "room_id": "room_3"},
            {"connector_id": "vc_vt_1", "transition_id": "vt_1"},
            {"floor_id": "floor_2", "room_id": "room_7"},
            {"floor_id": "floor_2", "room_id": "room_13"},
            {"floor_id": "floor_2", "room_id": "room_14"},
        ],
        "cross_floor_connector_segment": {
            "connector_id": "vc_vt_1",
            "transition_edge_id": "vt_1_centerline_e001",
            "segment_source": "stair_connector_2p5d_centerline",
        },
        "object_approach_segment": {
            "target": "approach_pose_not_object_centroid",
            "planner": "same_floor_astar_over_floor_2_stable_occupancy_map",
            "map_package": "clean_rerun/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml",
        },
        "route_sources": [
            "same-floor A* over stable occupancy map",
            "stair connector 2.5D centerline",
        ],
    },
    "validation_requirements": [
        "object query resolves to obj_175",
        "approach candidate exists and is reachable without using object centroid as navigation goal",
        "transition edge is vt_1_centerline_e001 and not vt_1_centerline_e003",
        "task24h-style room-level tracking input can be regenerated",
        "task24i-style object-level tracking input can be regenerated",
    ],
    "claim_boundary": {
        "topological_vertical_transition_only": True,
        "physical_stair_climbing_supported": False,
        "gait_supported": False,
        "footstep_planning_supported": False,
        "nav2_execution_supported": False,
        "amcl_execution_supported": False,
        "gazebo_set_entity_state_visual_display_only": True,
    },
}


INTEGRATION_PLAN = {
    "classification": "task25b_integration_targets_identified",
    "formal_outputs": [
        {
            "artifact": "vertical_connectors_v0_1.json",
            "recommended_location": "clean_rerun/committed_public",
            "raw_location": "canonical_stage1/raw_outputs/00843-DYehNKdT76V/logs",
            "producer_to_modify": "boxfusion/stage_a_demo.py finalization plus boxfusion/floor_aware_room_segmenter.py/floor_artifacts.py support",
            "notes": "Must include vt_1 centerline nodes and transition_edge_id vt_1_centerline_e001.",
        },
        {
            "artifact": "stairs_or_vertical_connector_graph_v0_1.json",
            "recommended_location": "clean_rerun/committed_public",
            "raw_location": "canonical_stage1/raw_outputs/00843-DYehNKdT76V/logs",
            "producer_to_modify": "new formal connector graph exporter, reusing task24b4 centerline logic",
            "notes": "Use claim-boundary guarded 2.5D/topological graph; no physical stair climbing claim.",
        },
        {
            "artifact": "cross_floor_topology_v0_1.json",
            "recommended_location": "clean_rerun/committed_public",
            "raw_location": "canonical_stage1/raw_outputs/00843-DYehNKdT76V/logs or routes/cross_floor",
            "producer_to_modify": "tools/vertical_connectors/build_cross_floor_connector_route.py and boxfusion/room_topology.py metadata",
            "notes": "Committed room topology can stay room-level; this artifact binds rooms to formal connectors.",
        },
        {
            "artifact": "cross_floor_route_query_report_v0_1.json",
            "recommended_location": "clean_rerun/routes/cross_floor/room_2_to_room_14",
            "raw_location": "not required",
            "producer_to_modify": "tools/vertical_connectors/build_cross_floor_connector_route.py",
            "notes": "Canonical route smoke query report, not raw perception output.",
        },
        {
            "artifact": "floor_1 stable occupancy map package",
            "recommended_location": "clean_rerun/maps/floor_1",
            "raw_location": "canonical_stage1/raw_outputs/00843-DYehNKdT76V/maps or logs if retained",
            "producer_to_modify": "formal map export/generation hook; validate with tools/stage1_runtime/build_scene_floor_occupancy_map.py",
            "notes": "Current floor_1 package was backfilled/corrected downstream and is a major regeneration risk.",
        },
        {
            "artifact": "floor_2 stable occupancy map package",
            "recommended_location": "clean_rerun/maps/floor_2",
            "raw_location": "canonical_stage1/raw_outputs/00843-DYehNKdT76V/maps or logs if retained",
            "producer_to_modify": "existing stable map export/generation hook; validate with stage1 runtime validator",
            "notes": "Must remain parseable and not regress.",
        },
        {
            "artifact": "object_query_resolution_v0_1.json",
            "recommended_location": "clean_rerun/routes/object_queries/curtain_room14_floor2",
            "raw_location": "not required",
            "producer_to_modify": "tools/object_nav/query_object_candidates.py or new formal object query resolver wrapper",
            "notes": "Should fill object_id, label, floor, room, query_text, and binding provenance.",
        },
        {
            "artifact": "object_approach_candidates_v0_1.json",
            "recommended_location": "clean_rerun/routes/object_approach_candidates/obj_175",
            "raw_location": "not required",
            "producer_to_modify": "tools/object_nav/audit_object_anchor_approach.py logic promoted to formal candidate generator",
            "notes": "generated_ring_037 or equivalent candidate must be map-validated and not use centroid navigation.",
        },
        {
            "artifact": "object_level_route_contract_v0_1.json",
            "recommended_location": "clean_rerun/routes/object_routes/room_2_to_obj_175_generated_ring_037",
            "raw_location": "not required",
            "producer_to_modify": "tools/vertical_connectors/build_cross_floor_object_approach_route.py generalized to formal artifacts",
            "notes": "Route source must say floor A* plus stair connector 2.5D centerline.",
        },
    ],
    "keep_task_only": [
        "Gazebo SetEntityState logs",
        "RViz marker screenshots/evidence packs",
        "task24h/task24i visual tracking execution traces",
        "exploratory audit reports and dry-run overlays",
    ],
    "code_locations_for_task25b": [
        "boxfusion/stage_a_demo.py",
        "boxfusion/floor_aware_room_segmenter.py",
        "boxfusion/floor_artifacts.py",
        "boxfusion/room_topology.py",
        "boxfusion/artifact_contract.py",
        "boxfusion/ros_artifact_bridge.py",
        "boxfusion/runtime_snapshot.py",
        "boxfusion/sidecar_exporter.py",
        "boxfusion/runtime_export_coordinator.py",
        "tools/stage1_runtime/build_scene_floor_occupancy_map.py",
        "tools/vertical_connectors/build_cross_floor_connector_route.py",
        "tools/vertical_connectors/build_occupancy_aware_cross_floor_visual_route.py",
        "tools/vertical_connectors/build_cross_floor_object_approach_route.py",
        "tools/object_nav/query_object_candidates.py",
        "tools/object_nav/audit_object_anchor_approach.py",
    ],
}


RISK_REGISTER = {
    "classification": "task25a_risks_identified",
    "risks": [
        {
            "risk": "floor_1 stable map package was downstream-backfilled and may not regenerate identically",
            "severity": "high",
            "mitigation": "Run task25b into a staging root; validate floor_1 YAML/PGM/NPZ/provenance before any public promotion.",
        },
        {
            "risk": "Stage-A output-root semantics may differ from task24 post-processing expectations",
            "severity": "high",
            "mitigation": "Keep canonical raw output, clean rerun candidate, maps, routes, and committed_public surfaces explicitly separated.",
        },
        {
            "risk": "vt_1 centerline currently exists as fitted downstream artifact, not formal Stage-A output",
            "severity": "high",
            "mitigation": "Promote centerline fitting/export logic and add regression checks for nodes and transition_edge_id.",
        },
        {
            "risk": "generated_ring_037 may only exist in task14/task24 outputs",
            "severity": "medium",
            "mitigation": "Formalize object approach candidate generation or allow an equivalent candidate with same validation thresholds.",
        },
        {
            "risk": "object centroid accidentally used as navigation goal",
            "severity": "high",
            "mitigation": "Schema and validation must require object_centroid_navigation_used=false and target=approach_pose_not_object_centroid.",
        },
        {
            "risk": "stable occupancy map, semantic floorplan, and gateway map could be confused",
            "severity": "medium",
            "mitigation": "Every route segment should name the map package source and planner type.",
        },
        {
            "risk": "00824 reference baseline could be overwritten by a broad output command",
            "severity": "high",
            "mitigation": "Use scene-scoped task25b staging paths and validate git status/protected mtimes before promotion.",
        },
        {
            "risk": "ROS2 tracking helpers fail under conda Python or Python 3.12 ABI",
            "severity": "medium",
            "mitigation": "Use conda Python only for Stage-A; run ROS2 tools in the ROS runtime interpreter/environment.",
        },
        {
            "risk": "Current task24 tools hard-code task paths and cannot consume formal artifacts directly",
            "severity": "medium",
            "mitigation": "Refactor route builders to accept formal clean_rerun/committed_public and maps paths.",
        },
        {
            "risk": "Claim boundary drift into physical stair-climbing or SLAM/localization accuracy claims",
            "severity": "high",
            "mitigation": "Require claim boundary files and schemas to state graph-level visual-kinematic traversal only.",
        },
    ],
}


CLAIM_BOUNDARY = {
    "classification": "task25_claim_boundary",
    "applies_to": ["task25a", "task25b", "task24 downstream reintegration"],
    "positive_scope": [
        "rich-semantic, light-geometry world-modeling backend",
        "dataset-side backend using RGB-D plus provided pose",
        "graph-level cross-floor traversal contracts",
        "topological vertical connectors and 2.5D centerline route segments",
        "visual-kinematic proxy tracking evidence only",
    ],
    "explicit_non_claims": {
        "slam_or_localization_accuracy_claimed": False,
        "real_deployment_without_localization_frontend_claimed": False,
        "physical_stair_climbing_supported": False,
        "gait_supported": False,
        "footstep_planning_supported": False,
        "contact_based_locomotion_supported": False,
        "nav2_execution_supported": False,
        "amcl_execution_supported": False,
        "gazebo_set_entity_state_is_control_execution": False,
    },
    "required_text_for_public_claims": "Stairs are represented as topological vertical connectors / 2.5D centerlines. The task does not support or claim physical stair climbing, gait, footstep planning, contact-based locomotion, Nav2, AMCL, or SLAM/localization accuracy.",
}


VALIDATION_CHECKLIST_MD = """
**Task25b Validation Checklist**

- Confirm the 00824 reference baseline and `stage_outputs/stage1_00824_step30p1` are untouched.
- Run Stage-A into a task25b staging output root, not directly into `clean_rerun`.
- Validate every generated JSON with `jq`.
- Confirm `floor_1` stable occupancy map package exists and parses: YAML, PGM, NPZ, provenance, preview when available.
- Confirm `floor_2` stable occupancy map package exists, parses, and is not broken.
- Confirm formal `vertical_connectors_v0_1.json` contains `vc_vt_1` / `vt_1`.
- Confirm vt_1 centerline nodes exist and carry floor ids.
- Confirm the floor transition edge is `vt_1_centerline_e001`, not `vt_1_centerline_e003`.
- Generate the formal room route `room_2 -> room_3 -> vt_1 -> room_7 -> room_13 -> room_14`.
- Resolve `curtain in room_14 on floor_2` to `obj_175`.
- Confirm `generated_ring_037` or an equivalent validated formal approach candidate exists.
- Build the object-level route to the approach pose, not the object centroid.
- Regenerate task24h-style room-level tracking input from formal artifacts.
- Regenerate task24i-style object-level tracking input from formal artifacts.
- Verify route segment sources state same-floor A* over stable occupancy maps and stair connector 2.5D centerline.
- Verify claim-boundary files state no physical stair climbing, gait, footstep planning, contact locomotion, Nav2, AMCL, or SLAM/localization accuracy claim.
"""


COMMANDS_SH = """#!/usr/bin/env bash
set -euo pipefail

REPO=/home/ws/workspace/BoxFusion
SCENE_ID=00843-DYehNKdT76V
RERUN_ROOT="$REPO/stage_outputs/stage1_generalization/$SCENE_ID/task25b_stage_a_rerun_candidate"
RAW_ROOT="$RERUN_ROOT/canonical_stage1/raw_outputs/$SCENE_ID"
FORMAL_CLEAN="$RERUN_ROOT/clean_rerun_candidate"
VALID="$RERUN_ROOT/validation"
mkdir -p "$VALID"

cd "$REPO"

# Stage-A must use the BoxFusion/RSLG-SLAM conda environment, not /usr/bin/python3.
/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \\
  --model-path models/cutr_rgbd.pth \\
  --config config/hm3d.yaml \\
  --seq "$SCENE_ID" \\
  --output-root "$RERUN_ROOT/canonical_stage1/raw_outputs" \\
  --capture-stride 25 \\
  --room-seg-interval 100 \\
  --runtime-profile-interval 25 \\
  --device cpu \\
  --quiet

jq -e '.sequence_id == "00843-DYehNKdT76V"' "$RAW_ROOT/logs/vertical_transition_evidence.json"
jq -e '[.summary.transitions[] | select(.transition_id=="vt_1" and .from_floor_id=="floor_1" and .to_floor_id=="floor_2")] | length == 1' "$RAW_ROOT/logs/vertical_transition_evidence.json"
jq -e '[.edges[] | select(.relation_type=="vertical_transition" and (.metadata.transition_ids | index("vt_1")))] | length >= 1' "$RAW_ROOT/logs/topology_v0_1.json"

# The following commands are task25b validation targets after the formal exporter
# creates $FORMAL_CLEAN. Some current task24 tools must first be generalized to
# consume formal artifacts instead of task-only paths.
/usr/bin/python3 tools/stage1_runtime/build_scene_floor_occupancy_map.py \\
  --scene-id "$SCENE_ID" \\
  --floor-id floor_1 \\
  --stage-output-dir "$FORMAL_CLEAN" \\
  --stage-a-output-dir "$RAW_ROOT" \\
  --output-json "$VALID/floor_1_map_validation.json" \\
  --output-md "$VALID/floor_1_map_validation.md"

/usr/bin/python3 tools/stage1_runtime/build_scene_floor_occupancy_map.py \\
  --scene-id "$SCENE_ID" \\
  --floor-id floor_2 \\
  --stage-output-dir "$FORMAL_CLEAN" \\
  --stage-a-output-dir "$RAW_ROOT" \\
  --output-json "$VALID/floor_2_map_validation.json" \\
  --output-md "$VALID/floor_2_map_validation.md"

jq -e '.connectors[] | select(.connector_id=="vc_vt_1" and .transition_id=="vt_1" and .transition_edge_id=="vt_1_centerline_e001")' \\
  "$FORMAL_CLEAN/committed_public/vertical_connectors_v0_1.json"
jq -e '[.connectors[].centerline_edges[] | select(.edge_id=="vt_1_centerline_e003" and (.source_floor_id != .target_floor_id))] | length == 0' \\
  "$FORMAL_CLEAN/committed_public/vertical_connectors_v0_1.json"

/usr/bin/python3 tools/vertical_connectors/build_cross_floor_connector_route.py \\
  --scene-root "$RERUN_ROOT" \\
  --start-room room_2 \\
  --goal-room room_14 \\
  --connector-id vc_vt_1 \\
  --output-dir "$VALID/cross_floor_room2_to_room14"

/usr/bin/python3 tools/vertical_connectors/build_occupancy_aware_cross_floor_visual_route.py \\
  --route-contract "$VALID/cross_floor_room2_to_room14/cross_floor_3d_route_contract_v0_1.json" \\
  --floor-1-map-yaml "$FORMAL_CLEAN/maps/floor_1/stage1_floor_1_stable_occupancy_map.yaml" \\
  --floor-2-map-yaml "$FORMAL_CLEAN/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml" \\
  --output-dir "$VALID/room2_to_room14_tracking_input"

/usr/bin/python3 tools/object_nav/query_object_candidates.py \\
  --index "$FORMAL_CLEAN/committed_public/object_candidate_index_v0_1.json" \\
  --query "curtain in room_14 on floor_2" \\
  --preferred-room-id room_14 \\
  --preferred-floor-id floor_2 \\
  --top-k 5 \\
  --output "$VALID/object_query_resolution_probe.json"

/usr/bin/python3 tools/vertical_connectors/build_cross_floor_object_approach_route.py \\
  --base-route-json "$VALID/room2_to_room14_tracking_input/planned_occupancy_aware_3d_route_v0_1.json" \\
  --floor-2-map-yaml "$FORMAL_CLEAN/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml" \\
  --output-dir "$VALID/room2_to_obj175_tracking_input"
"""


RERUN_PLAN_MD = """
**Task25b Full Rerun Plan**

1. Modify the Stage-A finalization/export path to produce formal connector and object route interface artifacts without changing the validated task24 evidence directories.
2. Run Stage-A into `stage_outputs/stage1_generalization/00843-DYehNKdT76V/task25b_stage_a_rerun_candidate`, not directly into the current `clean_rerun`.
3. Generate a clean-rerun candidate public surface from the staged raw outputs.
4. Validate floor maps, vt_1 connector semantics, cross-floor route regeneration, object query resolution, object approach route regeneration, and claim-boundary files.
5. Only after all checks pass, compare against current 00843 clean rerun and decide whether to promote selected formal artifacts.

Recommended Stage-A command:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \\
  --model-path models/cutr_rgbd.pth \\
  --config config/hm3d.yaml \\
  --seq 00843-DYehNKdT76V \\
  --output-root stage_outputs/stage1_generalization/00843-DYehNKdT76V/task25b_stage_a_rerun_candidate/canonical_stage1/raw_outputs \\
  --capture-stride 25 \\
  --room-seg-interval 100 \\
  --runtime-profile-interval 25 \\
  --device cpu \\
  --quiet
```

Use `/home/ws/miniconda3/envs/boxfusion/bin/python` for Stage-A. Use the ROS2 runtime interpreter/environment for ROS2 tracking players; probing those scripts under the conda Python hits an `rclpy` ABI mismatch.
"""


REPORT_MD = """
**Task25a Report**

Classification: `task25a_reintegration_preflight_completed`

Protected artifacts modified: `false`

Task25a was a read-only reintegration preflight. It did not rerun Stage-A, did not launch Gazebo/RViz, and did not modify the 00824 reference, 00843 committed_public artifacts, stable map packages, Stage-A raw outputs, or task24 evidence.

**Current Stage-A Producers**

- `stage_a_demo.py` provides the top-level Stage-A CLI and output-root wiring.
- `boxfusion/stage_a_demo.py` finalization writes `vertical_transition_evidence.json`, `topology_v0_1.json`, `final_vector_map_snapshot.json`, committed room-world files, and public/log copies.
- `boxfusion/floor_aware_room_segmenter.py` builds floor-aware transition records, `from_position_xy`, `to_position_xy`, frame/floor/room evidence, and object floor/room bindings.
- `boxfusion/floor_artifacts.py` canonicalizes floor and vertical transition evidence records.
- `boxfusion/room_topology.py` turns vt_1 into the committed room-level `vertical_transition` topology edge between room_3 and room_7.
- `tools/stage1_runtime/build_scene_floor_occupancy_map.py` validates existing stable occupancy map packages; it is not the original Stage-A map producer.

**Downstream Task24 Artifacts To Promote**

- Promote `vertical_connectors_v0_1.json` and a compact `stairs_or_vertical_connector_graph_v0_1.json` into formal committed/public connector artifacts.
- Promote `cross_floor_topology_v0_1.json` into a formal connector-aware topology artifact.
- Generate canonical route reports under `clean_rerun/routes`.
- Keep floor stable occupancy map packages under `clean_rerun/maps/floor_1` and `clean_rerun/maps/floor_2`, with formal provenance and validation.
- Promote object query resolution, object approach candidates, and object-level route contracts into formal route/object interfaces.
- Keep Gazebo/RViz execution evidence and SetEntityState logs task-only.

**Validated Facts To Preserve**

- Relevant vertical connector: `vt_1`.
- Route: `floor_1 room_2 -> room_3 -> vt_1 -> floor_2 room_7 -> room_13 -> room_14`.
- Real floor transition edge: `vt_1_centerline_e001`.
- `vt_1_centerline_e003` is not the floor transition edge.
- Object smoke route: `curtain in room_14 on floor_2 -> obj_175 -> generated_ring_037`.
- Object centroid navigation must remain `false`.

**Task25b Can Proceed**

Task25b can proceed if it uses a staging rerun root, formalizes the downstream connector/object interfaces before promotion, and runs the validation checklist in this bundle.
"""


FINAL_ANSWER_MD = """
classification: `task25a_reintegration_preflight_completed`

Protected artifacts modified: `false`.

Stage-A producer trace was identified in `stage_a_demo.py`, `boxfusion/stage_a_demo.py`, `boxfusion/floor_aware_room_segmenter.py`, `boxfusion/floor_artifacts.py`, and `boxfusion/room_topology.py`. The important gap is that Stage-A currently produces vt_1 evidence and room-level topology, but not the formal centerline connector graph, transition-edge id, or object-level route contract.

Task24 artifacts recommended for formalization are `vertical_connectors_v0_1.json`, a compact vertical connector graph, connector-aware cross-floor topology, canonical route query reports, stable floor map packages, object query resolution, object approach candidates, and object-level route contracts. Gazebo/RViz tracking evidence should stay task-only.

Task25b can proceed using the staged rerun command in `task25b_candidate_commands.sh`, with the claim boundary preserved: topological / visual-kinematic only, no physical stair climbing, no gait, no footstep planning, no Nav2, no AMCL, and no SLAM/localization accuracy claim.
"""


def main() -> None:
    TASK_ROOT.mkdir(parents=True, exist_ok=True)

    inventory = build_inventory()
    report_json = {
        "classification": "task25a_reintegration_preflight_completed",
        "protected_artifacts_modified": False,
        "task25b_can_proceed": True,
        "task25b_can_proceed_conditions": [
            "Run Stage-A into a staging output root first.",
            "Formalize connector centerline and object approach artifacts before public promotion.",
            "Run the validation checklist and keep claim boundaries explicit.",
        ],
        "stage_a_producer_summary": [
            "stage_a_demo.py top-level CLI",
            "boxfusion/stage_a_demo.py finalization/export",
            "boxfusion/floor_aware_room_segmenter.py floor-aware transitions and bindings",
            "boxfusion/floor_artifacts.py canonicalization",
            "boxfusion/room_topology.py committed room-level topology",
        ],
        "formal_artifacts_recommended": [
            item["artifact"] for item in INTEGRATION_PLAN["formal_outputs"]
        ],
        "main_risks": [risk["risk"] for risk in RISK_REGISTER["risks"]],
        "required_outputs_written": [
            "task25a_report.md",
            "task25a_report.json",
            "existing_artifact_inventory.json",
            "stage_a_producer_audit.json",
            "task24_downstream_artifact_lineage.json",
            "formal_vertical_connector_schema_proposal.json",
            "formal_object_route_schema_proposal.json",
            "integration_target_file_plan.json",
            "task25b_full_rerun_plan.md",
            "task25b_validation_checklist.md",
            "task25b_candidate_commands.sh",
            "risk_register.json",
            "claim_boundary_for_task25.json",
            "final_answer_for_user.md",
        ],
    }

    write_text(TASK_ROOT / "task25a_report.md", REPORT_MD)
    write_json(TASK_ROOT / "task25a_report.json", report_json)
    write_json(TASK_ROOT / "existing_artifact_inventory.json", inventory)
    write_json(TASK_ROOT / "stage_a_producer_audit.json", STAGE_A_PRODUCER_AUDIT)
    write_json(TASK_ROOT / "task24_downstream_artifact_lineage.json", TASK24_LINEAGE)
    write_json(TASK_ROOT / "formal_vertical_connector_schema_proposal.json", VERTICAL_SCHEMA)
    write_json(TASK_ROOT / "formal_object_route_schema_proposal.json", OBJECT_SCHEMA)
    write_json(TASK_ROOT / "integration_target_file_plan.json", INTEGRATION_PLAN)
    write_text(TASK_ROOT / "task25b_full_rerun_plan.md", RERUN_PLAN_MD)
    write_text(TASK_ROOT / "task25b_validation_checklist.md", VALIDATION_CHECKLIST_MD)
    commands_path = TASK_ROOT / "task25b_candidate_commands.sh"
    write_text(commands_path, COMMANDS_SH)
    os.chmod(commands_path, 0o755)
    write_json(TASK_ROOT / "risk_register.json", RISK_REGISTER)
    write_json(TASK_ROOT / "claim_boundary_for_task25.json", CLAIM_BOUNDARY)
    write_text(TASK_ROOT / "final_answer_for_user.md", FINAL_ANSWER_MD)

    print(f"Wrote task25a bundle to {TASK_ROOT}")


if __name__ == "__main__":
    main()
