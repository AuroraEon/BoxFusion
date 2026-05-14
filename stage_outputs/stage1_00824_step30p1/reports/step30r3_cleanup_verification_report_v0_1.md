# Step30R3 Cleanup Verification Report

- Status: PASSED
- Stage output: `stage_outputs/stage1_00824_step30p1`
- Failure count: 0

## Failures
- None

## Passed Checks
- Stage output exists: stage_outputs/stage1_00824_step30p1
- baselines/ is reserved and has README.md
- baselines/ contains no milestone artifacts
- No active artifact path contains referenced_artifacts
- No active artifact directory name starts with step
- Readable required file: stage_outputs/stage1_00824_step30p1/manifest/artifact_manifest_v0_2.json
- Readable required file: stage_outputs/stage1_00824_step30p1/manifest/provenance_manifest_v0_2.json
- Readable required file: stage_outputs/stage1_00824_step30p1/manifest/deletion_manifest_step30r3_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/manifest/path_relocation_manifest_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/stage1_committed_public/committed_room_world_model_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/stage1_committed_public/committed_room_world_snapshot_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/stage1_committed_public/topology_query_report.json
- Readable required file: stage_outputs/stage1_00824_step30p1/stage1_committed_public/topology_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/stage1_committed_public/step30p1_committed_public_artifact_index_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/execution/step30p1_execute_room_chain_result_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/execution/latest_planned_path.json
- Readable required file: stage_outputs/stage1_00824_step30p1/execution/latest_follow_path_slice.json
- Readable required file: stage_outputs/stage1_00824_step30p1/validation/step30p1_fallback_resume_report_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/validation/step30p1_trajectory_quality_validation_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/validation/step30p1_through_room_physical_visit_validation_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/maps/map_manifest_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/overlay/rviz_overlay_payload_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/overlay/overlay_manifest_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/route/room_chain_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/route/gateway_sequence_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/route/route_spec_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/route/resolved_route_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/gateway/selected_gateway_summary_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/gateway/gateway_candidates_or_hypotheses_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/gateway/gateway_truth_blind_selection_v0_1.json
- Readable required file: stage_outputs/stage1_00824_step30p1/README.md
- Readable required file: stage_outputs/stage1_00824_step30p1/manifest/artifact_manifest_v0_2.md
- Readable required file: stage_outputs/stage1_00824_step30p1/execution/step30p1_runbook_v0_1.md
- Readable required file: stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml
- Readable required file: stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.pgm
- Readable required file: stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_masks.npz
- Readable required file: stage_outputs/stage1_00824_step30p1/nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py
- Readable required file: stage_outputs/stage1_00824_step30p1/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml
- Readable required file: stage_outputs/stage1_00824_step30p1/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf
- Readable required file: stage_a_demo.py
- Readable required file: docs/current_project_state.md
- Nav2 ROS package templates exist
- Gazebo overlay assets exist
- Trajectory files exist
- Bad fallback regression evidence exists
- No top-level stage_a_*.py remains except stage_a_demo.py
- Old StepXX provenance is preserved in provenance manifest
- No active artifact file outside provenance/deletion manifests refers to old StepXX paths
- No active Python script refers to deleted runtime_stage1_frozen_evidence/stepXX paths
