# Step30R3 Repo Cleanup Report

## 1. Artifact Move

`baselines/stage1_00824_step30p1/` moved to `stage_outputs/stage1_00824_step30p1/`. The path rewrite record is `stage_outputs/stage1_00824_step30p1/manifest/path_relocation_manifest_v0_1.json`.

## 2. Reserved Baselines Directory

`baselines/` is now reserved for future comparison baselines from other methods or workflows. It contains only `baselines/README.md` and no Stage1 Step30P1 milestone artifact.

## 3. Referenced Artifacts

Yes. `referenced_artifacts/` was removed from the active artifact tree. Its files were classified in `manifest/provenance_manifest_v0_2.json` and `manifest/deletion_manifest_step30r3_v0_1.json`; old StepXX names remain there as provenance/deletion metadata only.

## 4. Preserved Semantic Folders

Preserved artifacts now live under `stage1_committed_public/`, `route/`, `gateway/`, `maps/`, `overlay/`, `nav2/`, `execution/`, `validation/`, `trajectories/`, `regression/`, `manifest/`, and `reports/`.

## 5. Deleted Top-Level Python Scripts

Deleted top-level `stage_a_*.py` scripts except `stage_a_demo.py`:

- `stage_a_end_to_end_vln_demo.py`
- `stage_a_minimal_vln_closed_loop.py`
- `stage_a_minimal_vln_demo.py`
- `stage_a_multifloor_query_acceptance.py`
- `stage_a_online_topology_blocker_diag.py`
- `stage_a_online_topology_timeline_eval.py`
- `stage_a_online_topology_working_snapshot.py`
- `stage_a_publication_candidate_survivability.py`
- `stage_a_publication_policy_simulation.py`
- `stage_a_room_commit_diagnosis.py`
- `stage_a_room_graph_vln_demo.py`
- `stage_a_room_graph_vln_demo_bundle.py`
- `stage_a_ros2_sim_ingress_validation.py`
- `stage_a_ros_publication_diagnostics_validation.py`
- `stage_a_ros_query_server_validation.py`
- `stage_a_runtime_export_coordinator_validation.py`
- `stage_a_template_grounding.py`
- `stage_a_template_grounding_acceptance.py`
- `stage_a_topology_acceptance.py`
- `stage_a_topology_export.py`
- `stage_a_topology_query.py`
- `stage_a_topology_route.py`
- `stage_a_vln_tool_use_demo.py`
- `stage_a_working_vs_committed_topology_timeline_eval.py`
- `stage_a_world_model_backend_eval.py`

Deleted other non-pipeline top-level scripts: `map_builder.py`, `gen_features.py`, `visualize_check.py`.

## 6. Remaining Top-Level Python Scripts

- `stage_a_demo.py`: active Stage-A / Stage1 pipeline entry point.
- `demo.py`: imported by `stage_a_demo.py` as the runtime execution backend.
- `setup.py`: package setup metadata.

## 7. Docs Merged/Deleted

Merged useful project-state content from the previous historical summary, baseline index, runtime reproduction notes, and README into `docs/current_project_state.md`. Deleted stale docs under `docs/` and removed root reproduction guides.

Deleted docs count: 81 under `docs/`; root docs deleted: `runtime_reproduction_guide.md`, `runtime_reproduction_quickstart.md`.

Additional stale script/tool folders pruned: `scripts`, `stage_a_eval`, `demo`, `tools/__pycache__`, `tools/analyze_same_frame_object_deltas.py`, `tools/audit_committed_topology.py`, `tools/build_step16_navigation_projection_audit.py`, `tools/build_step17_gazebo_nav2_asset_layer.py`, `tools/build_step6_manual_topology_triage.py`, `tools/generate_step12_ros_rviz_smoke_evidence.py`, `tools/step29a3r_export_gateway_wall_evidence.py`, `tools/step29b1r_audit_structural_wall_generation.py`, `tools/step29b2r2_build_local_gateway_inspection_pack.py`, `tools/step29b3_build_gateway_hypotheses.py`, `tools/step29b3r_audit_gateway_hypotheses.py`, `tools/step30a_full_stage_a_dual_wall_gateway_rerun.py`, `tools/step30b2_00824_gateway_truth_and_auto_selection.py`, `tools/step30b_00824_gateway_benchmark_expansion.py`, `tools/step30c_00824_gateway_augmented_topology_candidate.py`, `tools/step30d_00824_topology_route_projection_overlay_review.py`, `tools/step30e_00824_nav_projection_map_and_planner_audit.py`, `tools/step30f_00824_prepare_gui_overlay_payload.py`, `tools/step30g_00824_route_command_interface.py`, `tools/step30h_00824_live_gui_nav2_cleanup_planning_smoke.py`, `tools/trace_route_visible_topology_edges.py`, `tools/validate_ros_bridge_artifact_contract.py`, `tools/stage1_step30p1/consolidate_stage1_step30p1_baseline.py`, `tools/stage1_step30p1/legacy_moved_scripts`.

## 8. New Docs Entry Point

`docs/current_project_state.md` is the single docs entry point.

## 9. Verification

Verification status: passed. Report files:

- `stage_outputs/stage1_00824_step30p1/reports/step30r3_cleanup_verification_report_v0_1.json`
- `stage_outputs/stage1_00824_step30p1/reports/step30r3_cleanup_verification_report_v0_1.md`

## 10. Pipeline Continuity

The project can continue from `stage_a_demo.py` and the cleaned Stage1 Step30P1 artifact. Syntax and compile checks passed for `stage_a_demo.py`, `demo.py`, `boxfusion`, and `tools/stage1_step30p1`. A direct `stage_a_demo.py --help` preflight was blocked in this shell because `yaml`/PyYAML is not installed, so no full Stage-A, Gazebo, Nav2, gateway regeneration, or object-level navigation run was attempted.
