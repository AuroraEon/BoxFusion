# Current Project State

## Current Goal

BoxFusion's current working path is the Stage-A / Stage1 pipeline plus the cleaned 00824 Step30P1 milestone artifact. The repository should remain small enough to support future work, especially object-level navigation, without carrying old StepXX runtime folders or one-off migration scripts.

## Current Entry Points

- Active pipeline entry point: `stage_a_demo.py`
- Supporting runtime implementation: `boxfusion/` package code and `demo.py`
- Current milestone artifact: `stage_outputs/stage1_00824_step30p1/`
- Reserved comparison-baseline area: `baselines/`

`baselines/` is reserved for future comparison baselines from other methods or workflows. The Stage1 Step30P1 artifact is not a comparison baseline and must stay under `stage_outputs/`.

## Step30P1 Status

Accepted wording: “Step30P1 repaired execution succeeded with clean forward-only fallback.”

Preserved Step30P1 material includes the committed/public Stage1 world and topology JSON, route and gateway records, H8R2 Nav2 map assets, RViz/Gazebo overlay payload/assets, Nav2 launch/config/world/template assets, execution outputs, trajectory files, validation reports, and minimal bad-fallback regression evidence.

Step30S2 restored the compact Stage1 process artifacts and rebuilt the post-restructure Gazebo/TurtleBot3/Nav2 runtime wrapper. The live post-restructure validation ran in simulation with static `map -> odom` localization, TurtleBot3 burger, the H8R2 map, the large continuous collision floor world, and the existing Nav2 staticloc stack. This validation does not replace the accepted Step30P1 milestone wording above.

Step30S7 is the current GUI-quality request-aware projection and room15-through validation path. The one-command runner treats `--gui` as Gazebo GUI plus RViz2, using `stage_outputs/stage1_00824_step30p1/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`. RViz separates the Nav2 occupancy/free-space map from semantic room-mask markers, room fill/outline, selected interior targets, gateways, planned route, executed trajectory, and invalid/suspicious wall-crossing markers.

Step30S6 root-cause audit found the room15 free-space loss was introduced by H8R2 map projection, not by Stage-A room segmentation, wall processing, gateway-wall-preclose, coordinate/y-flip mismatch, Step30R3 restoration, or real source geometry. Stage30A marks all 1324 room15 mask cells as `free_space`, but H8R2 projected only `route_room_ids = [1, 3, 8, 11, 7, 14, 16]`. Since room15 was not in that whitelist, its room interior was left mostly unknown and only the r7-r15 gateway carve contributed 89 free cells.

Step30S7 builds `maps/step30s7_request_aware_nav_map.yaml` from the current request and resolved topology route. It includes start, goal, terminal, through, and intermediate route rooms. For `room_1 -> room_16` through `room_15`, the resolved topology route is `room_1 -> room_3 -> room_7 -> room_15 -> room_7 -> room_14 -> room_16`; revisiting room7 is expected. Room15 is included because it is requested, not because projection code hard-codes room15.

Step30S6 also audited gateway extraction generalization. The Step30B2 artifacts support a truth-blind geometry/semantic selection pass for 00824, with truth labels marked validation-only. However, the packaged pipeline is not yet generalizable: current support and projection tools contain 00824 route/gateway/forbidden constants, Step30A role annotations include positive-control labels, the public `gateway_truth_blind_selection_v0_1.json` is actually a truth benchmark artifact, and no held-out scene gateway extraction CLI/assets were available for a non-destructive cross-scene run.

Current active map profiles are only `h8r2`, `step30s7_request_aware`, and `auto`. The old Step30S5 room15-only map was removed from active use; current runtime tools reject the old profile names instead of selecting them. `--map-profile auto` may choose H8R2 for already-supported routes or Step30S7 for requests that need additional route/through room interiors, but never selects the removed map.

Step30S7 strengthens through-room validation for `--through-rooms room_15`. The planned route is topology-valid and may revisit room7: `room_1 -> room_3 -> room_7 -> room_15 -> room_7 -> room_14 -> room_16`. The validator uses trajectory samples and the restored room segmentation mask, not topology alone, and requires by default at least 8 room15 samples and 3.0 seconds of room15 dwell. The route runner splits at the explicit room15 interior waypoint, records a dwell, then continues to room16. A trajectory that only stops near or grazes the room15 gateway fails.

Step30S7 separately validates trajectory wall crossing, local gateway looping/spinning, and room16 terminal quality. Wall checking tests trajectory points and dense Bresenham segments against original H8R2, the Step30S7 map, and structural wall evidence when available. Local looping checks angular travel and repeated revisits near the through-room doorway. Room16 terminal quality distinguishes Nav2 action success from visual terminal quality and reports whether the final pose is inside the room16 mask, too close to the r14-r16 gateway, inside a conservative room16 interior region, and whether `terminal_visual_quality_passed` is true.

## Boundaries

- no AMCL
- static map -> odom localization
- no manual cmd_vel
- no real physical robot deployment claim
- no full collision-free guarantee
- no online object detection
- do not claim pure FollowPath-only uninterrupted success
- do not claim deep room8 interior terminal visit
- do not treat Gazebo-only as GUI success for Step30S7
- do not treat route topology alone as physical through-room success

## Historical Summary

- Step17: created the early Gazebo/Nav2 asset layer, scene maps, ROS package scaffolding, launch templates, and environment feasibility inventory.
- Step30A: reran Stage-A for scene 00824 with dual-wall and gateway-wall-preclose artifacts, refreshing public topology outputs.
- Step30B/B2: expanded gateway extraction, benchmarked the eight-gateway set, and selected candidate-level gateway truth for later routing.
- Step30E/H8R2: repaired Nav2 map projection into the gateway-preserving H8R2 map used by the final milestone.
- Step30F: bridged semantic route/topology information into RViz/Gazebo overlay payloads; Step30P1 keeps RViz as the full semantic overlay carrier.
- Step30K/L/N/O/P: iterated route execution, static localization, smooth room traversal, semantic target routing, and fallback behavior.
- Step30P: preserved a bad regression example where fallback resumed too early and caused large backtracking.
- Step30P1: final repaired forward-only fallback milestone. It resumed from `selected_list_index=5` / waypoint 83, had no backward progress jumps, and validated room8 route topology without claiming a deep room8 terminal visit.

## Current Directory Layout

```text
stage_outputs/stage1_00824_step30p1/
  manifest/
  stage1_process/
    room_segmentation/
    gateway_extraction/
    provenance/
  stage1_committed_public/
  route/
  gateway/
  maps/
  overlay/
  nav2/
  execution/
  validation/
  trajectories/
  regression/
  post_restructure_validation/
  reports/
baselines/
  README.md
tools/stage1_step30p1/
  prepare_stage1_step30p1_room15_route.py
  publish_stage1_step30p1_rviz_overlay.py
  publish_stage1_step30p1_route_markers.py
  publish_stage1_step30p1_trajectory_markers.py
  audit_stage1_step30p1_room15_map_coverage.py
  audit_stage1_step30p1_room15_free_space_root_cause.py
  audit_stage1_gateway_extraction_generalization.py
  publish_stage1_step30p1_room_mask_overlay.py
  validate_stage1_step30p1_physical.py
  restore_stage1_process_artifacts.py
  launch_stage1_step30p1_gazebo_nav2.sh
  probe_stage1_step30p1_dataplane.py
  run_stage1_step30p1_smoke_nav.py
  run_stage1_step30p1_route.py
  verify_stage1_step30p1_artifact.py
docs/
  current_project_state.md
```

Old StepXX names are retained as provenance metadata in `stage_outputs/stage1_00824_step30p1/manifest/provenance_manifest_v0_2.json` and deletion metadata in `stage_outputs/stage1_00824_step30p1/manifest/deletion_manifest_step30r3_v0_1.json`, not as active artifact directories.

## Stage1 Process Artifacts

Restored process artifacts live under `stage_outputs/stage1_00824_step30p1/stage1_process/`.

- Room segmentation: `stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/`
- Gateway extraction: `stage_outputs/stage1_00824_step30p1/stage1_process/gateway_extraction/`
- Restore provenance: `stage_outputs/stage1_00824_step30p1/stage1_process/provenance/`

The restore copied 58 compact files from `/home/ws/workspace/runtime_stage1_frozen_evidence`: 19 room-segmentation files and 39 gateway-extraction files. It intentionally did not restore full RGB frame dumps or old StepXX folders as active directories.

Restore command:

```bash
python3 tools/stage1_step30p1/restore_stage1_process_artifacts.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --backup-root /home/ws/workspace/runtime_stage1_frozen_evidence
```

## Post-Restructure Runtime

Bringup command:

```bash
tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --headless
```

Dataplane probe:

```bash
/usr/bin/python3 tools/stage1_step30p1/probe_stage1_step30p1_dataplane.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --timeout-sec 15
```

Navigation smoke test:

```bash
/usr/bin/python3 tools/stage1_step30p1/run_stage1_step30p1_smoke_nav.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --distance-m 0.65 \
  --goal-timeout-sec 90 \
  --min-movement-m 0.05
```

Optional full route rerun after smoke passes:

```bash
/usr/bin/python3 tools/stage1_step30p1/run_stage1_step30p1_route.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --follow-path-timeout-sec 240 \
  --goal-timeout-sec 90
```

Step30S7 full GUI BEV semantic-overlay room15-through demo:

```bash
tools/stage1_step30p1/run_stage1_step30p1_end_to_end.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 \
  --goal-room room_16 \
  --through-rooms room_15 \
  --mode full \
  --from-start \
  --terminal-room room_16 \
  --gui \
  --map-profile step30s7_request_aware \
  --through-room-dwell-sec 3.0 \
  --room15-min-inside-samples 8 \
  --keep-gui-open-sec 20
```

Expected Step30S7 evidence directory:

```text
stage_outputs/stage1_00824_step30p1/post_restructure_validation/step30s7_request_aware_projection/
```

Step30S6 root-cause and gateway-generalization audit:

```bash
python3 tools/stage1_step30p1/audit_stage1_step30p1_room15_free_space_root_cause.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1

python3 tools/stage1_step30p1/audit_stage1_gateway_extraction_generalization.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

Step30S6 evidence is written under:

```text
stage_outputs/stage1_00824_step30p1/post_restructure_validation/step30s6_root_cause_and_gateway_generalization/
```

Step30S7 cleanup command:

```bash
tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --stop
```

Step30S2 live evidence is saved in `stage_outputs/stage1_00824_step30p1/post_restructure_validation/`.

Observed validation:

- Dataplane ready: `/clock`, `/odom`, `/scan`, `/tf`, `/tf_static`, `/map`, `/cmd_vel` subscribers, and Nav2 actions were present.
- `/odom` published at about 29.6 Hz and `/scan` at about 5.1 Hz during the probe.
- Static `map -> odom`, `odom -> base_footprint`, `base_footprint -> base_link`, and `base_link -> base_scan` were present.
- The 0.65 m smoke NavigateToPose moved the robot about 0.409 m and produced `/cmd_vel`.
- The post-restructure full route rerun completed in simulation with FollowPath accepted and final arrival success; no sparse fallback was needed in that rerun.

Current limitations:

- No AMCL; localization remains static `map -> odom`.
- No real physical robot deployment claim.
- No manual `cmd_vel` publication by the harness.
- No full collision-free guarantee.
- No online object detection or object-level navigation.
- The post-restructure route rerun is simulator runtime validation and must not be used to reword the original Step30P1 milestone as pure FollowPath-only uninterrupted success.
- Step30S7 still does not implement object-level navigation or online object detection.
- Step30S7 derives through-room and terminal interior targets from existing room/mask/map artifacts and does not rerun Stage-A.
- Step30S7 does not claim full multi-scene gateway extraction generalization.
- Step30S6 did not rerun Stage1; it recommends a smallest-scope future H8R2 projection successor after changing the general room-inclusion policy.

## Future Object-Level Navigation Chain

committed/public object query -> object candidate / target room selection -> object approach or room search target -> topology route -> overlay -> Nav2 execution -> trajectory/object-navigation validation
