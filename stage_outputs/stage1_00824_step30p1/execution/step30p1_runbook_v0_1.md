# Step30P1 00824 Runbook

Step30P1 repairs the frozen Step30P route demo with progress-aware sparse fallback resume and trajectory-based through-room physical visit validation. Step30O and Step30P artifacts remain frozen; Step30P1 writes only under `stage_outputs/stage1_00824_step30p1/manifest/provenance_manifest_v0_2.json

## Preserved Stack

- Gazebo uses the copied large continuous collision floor world.
- Localization is static `map -> odom`; AMCL is not launched.
- Nav2 execution uses `FollowPath`; the harness does not manually publish `/cmd_vel`.
- Sparse `NavigateToPose` fallback is forward-only and may not restart at already-completed semantic waypoints.
- Bridge and smooth bridge waypoints are path samples only, not NavigateToPose goals.
- RViz displays `/map` plus `/step30p1_h8r2_route_markers`.
- Forbidden shortcut edges are rejected.

## Semantic Additions

- `outputs/validation/step30p1_fallback_resume_report_v0_1.json` records projected FollowPath progress and the selected sparse fallback resume waypoint.
- `outputs/validation/step30p1_trajectory_quality_validation_v0_1.json` compares executed trajectory length and progress monotonicity against the planned waypoint path.
- `outputs/validation/step30p1_through_room_physical_visit_validation_v0_1.json` separates topology satisfaction from trajectory evidence for requested through-rooms.
- `outputs/validation/step30p1_bad_run_regression_report_v0_1.json` proves the preserved Step30P restart-at-waypoint-2 behavior would be flagged as backtracking.
- `outputs/semantic_targets/step30p1_room_point_registry_v0_1.json` records `safe_terminal`, `gateway_approach_terminal`, `interior_terminal`, `via_room_terminal`, `search_vantage_points`, validation, sources, quality, and rejected candidates.
- `outputs/query/step30p1_committed_public_artifact_index_v0_1.json` indexes committed/public 00824 topology and room-world artifacts.
- Object queries use committed/public object records only. `online_object_detection_claimed=false`.
- RViz overlay payload includes route, gateway labels, room chain, target room/object, target pose, via/search terminal, trajectory, latest FollowPath slice, and query-result text.
- Gazebo floorplan visual overlay is intentionally not added; physics stability is preserved and recorded in `outputs/overlay/step30p1_gazebo_overlay_validation_v0_1.json`.

## Key Commands

Cleanup:

```bash
tools/stage1_step30p1/legacy_moved_scripts/step30p1/cleanup_step30p1_processes.sh --reset-ros2-daemon
```

Auto topology:

```bash
tools/stage1_step30p1/legacy_moved_scripts/step30p1/run_step30p1_end_to_end.sh \
  --start-room room_1 \
  --goal-room room_15 \
  --mode full \
  --from-start \
  --terminal-room room_15 \
  --gui
```

Strengthened through-room route:

```bash
tools/stage1_step30p1/legacy_moved_scripts/step30p1/run_step30p1_end_to_end.sh \
  --start-room room_1 \
  --goal-room room_16 \
  --through-rooms room_8 \
  --mode full \
  --from-start \
  --terminal-room room_16 \
  --gui
```

Object with target room:

```bash
tools/stage1_step30p1/legacy_moved_scripts/step30p1/run_step30p1_end_to_end.sh \
  --start-room room_1 \
  --target-object chair \
  --target-room room_15 \
  --mode full \
  --from-start \
  --gui
```

Object without target room:

```bash
tools/stage1_step30p1/legacy_moved_scripts/step30p1/run_step30p1_end_to_end.sh \
  --start-room room_1 \
  --target-object chair \
  --mode full \
  --from-start \
  --gui
```

Template query examples:

```bash
--query "find chair in room_15"
--query "go to room_15"
--query "find chair"
```

## Validated Result

The final integrated run executed:

`room_1 -> room_3 -> room_8 -> room_11 -> room_7 -> room_14 -> room_16`

Evidence:

- `route_source=start_goal_through_rooms`
- `follow_path_used=true`
- `sparse_fallback_used=true`
- FollowPath aborted at projected slice `82`; fallback resumed at selected list index `5`, waypoint `83`
- `fallback_backtracking_detected=false`
- `sparse_fallback_clean_resume_passed=true`
- `floor_audit_passed=true`
- `fall_detected=false`
- `robot_motion_observed=true`
- final arrival success at room16: `true`
- trajectory distance: `25.630699 m`
- trajectory/planned distance ratio: `0.988642`
- trajectory backtracking detected: `false`
- room8 physical visit passed: `true` using public topology room polygon mask evidence
- overall clean demo passed: `true`

Room8 through-room semantics are recorded separately from topology: the route contains room8, and the trajectory entered the public topology room polygon for 33 samples / 6.399886 seconds while also crossing the expected entry and exit gateways. The selected room8 via terminal itself was not closely approached (`3.986014 m` minimum), so this result depends on available polygon-mask evidence rather than terminal-proximity evidence.
