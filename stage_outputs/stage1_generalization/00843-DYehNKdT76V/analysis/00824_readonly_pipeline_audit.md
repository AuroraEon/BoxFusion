# 00824 Read-only Pipeline Audit

## Executive Summary

This audit traced the accepted 00824 Stage1-to-Gazebo/Nav2/RViz pipeline statically. The user-facing entrypoint is `tools/stage1_nav/run_gui_demo.sh`; it wraps the internal Step30P1 orchestration and defaults `stable`/`auto` to the stable full-scene occupancy map.

The accepted runtime bridge is not the committed topology JSON alone. The bridge is the scene-specific chain of dual-wall raster/process artifacts, selected gateway config/registry, stable occupancy map YAML/PGM/NPZ, generated semantic route waypoints, 00824 Gazebo world, Nav2 static-localization launch/config, RViz BEV overlay config, and validation outputs.

Truth boundaries preserved: no AMCL is launched by the active runtime, no manual `cmd_vel` is claimed, no physical robot deployment is claimed, no full collision-free guarantee is claimed, and 00843 GUI/Nav2 success is not claimed. Accepted wording remains: `Step30P1 repaired execution succeeded with clean forward-only fallback.`

## Read-only Scope and Safety Constraints

Audit root: `/home/ws/workspace/BoxFusion`.

Accepted 00824 baseline: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1`. The audit used only static inspection commands and a report-generation script that wrote into `/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/analysis`. No ROS launch, Gazebo, Nav2, RViz, FollowPath, NavigateToPose, or route execution was attempted.

Forbidden-area git status at audit end: `['M tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py']`. The dirty `tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py` entry was detected but not edited by this audit.

## 00824 Accepted Runtime Entry Points

Primary user-facing command:

```bash
tools/stage1_nav/run_gui_demo.sh   --stage-output-dir stage_outputs/stage1_00824_step30p1   --start-room room_1 --goal-room room_16   --through-rooms room_8   --terminal-room room_16   --from-start --gui   --map-profile stable   --keep-gui-open-sec 20
```

`--stage-output-dir` is required. Defaults are `start=room_1`, `goal=room_16`, `terminal=room_16`, `map-profile=stable`, `gui=true`, `keep-gui-open-sec=20`, through-room minimum samples `8`, and dwell `3.0s`. `stable`, `auto`, `full_scene`, and `stage1_full_scene` normalize to `stage1_full_scene_occupancy`. Deprecated Step30S5 map profiles are rejected.

## Static Call Graph

- `tools/stage1_nav/run_gui_demo.sh` -> `tools/stage1_step30p1/run_stage1_step30p1_end_to_end.sh`: exec wrapper with normalized CLI and map-profile aliases
- `run_stage1_step30p1_end_to_end.sh` -> `build_stage1_full_scene_occupancy_map.py`: stable/full_scene/auto map profile
- `run_stage1_step30p1_end_to_end.sh` -> `build_stage1_step30p1_request_aware_nav_map.py`: request_aware debug/reference profile
- `run_stage1_step30p1_end_to_end.sh` -> `validate_stage1_step30p1_request_map.py`: request-aware map validation only
- `run_stage1_step30p1_end_to_end.sh` -> `audit_stage1_step30p1_bev_visual_regression.py`: request-aware visual diagnostics
- `run_stage1_step30p1_end_to_end.sh` -> `write_stage1_step30p1_gateway_readiness_report.py`: gateway readiness/provenance check
- `run_stage1_step30p1_end_to_end.sh` -> `prepare_stage1_step30p1_semantic_route.py`: route query and waypoint generation
- `run_stage1_step30p1_end_to_end.sh` -> `launch_stage1_step30p1_gazebo_nav2.sh --stop`: cleanup old runtime processes
- `run_stage1_step30p1_end_to_end.sh` -> `launch_stage1_step30p1_gazebo_nav2.sh`: Gazebo, TurtleBot3, static TF, Nav2 bringup
- `launch_stage1_step30p1_gazebo_nav2.sh` -> `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py`: Nav2 static localization launch
- `launch_stage1_step30p1_gazebo_nav2.sh` -> `wait_stage1_step30p1_nav2_readiness.py`: lifecycle/map/FollowPath readiness
- `run_stage1_step30p1_end_to_end.sh` -> `publish_stage1_step30p1_rviz_overlay.py`: RViz semantic overlay before/after route/validation
- `run_stage1_step30p1_end_to_end.sh` -> `rviz2 -d 00824_stage1_step30p1_bev_semantic_route_demo.rviz`: RViz GUI
- `run_stage1_step30p1_end_to_end.sh` -> `probe_stage1_step30p1_dataplane.py`: topics, TF, actions check
- `run_stage1_step30p1_end_to_end.sh` -> `run_stage1_step30p1_route.py`: FollowPath and repaired fallback execution
- `run_stage1_step30p1_route.py` -> `/follow_path action`: primary route execution action
- `run_stage1_step30p1_route.py` -> `/navigate_to_pose action`: forward-only sparse fallback when FollowPath does not finish
- `run_stage1_step30p1_route.py` -> `/compute_path_to_pose action`: optional plan-before-goal diagnostic/fallback planning
- `run_stage1_step30p1_route.py` -> `/set_entity_state or /set_model_state`: route-start reset when --from-start/--reset-to-route-start is requested
- `run_stage1_step30p1_end_to_end.sh` -> `validate_stage1_step30p1_physical.py`: through-room, terminal, wall, spin validation
- `validate_stage1_step30p1_physical.py` -> `maps/stage1_full_scene_occupancy_map.yaml`: active-map wall/occupied validation
- `validate_stage1_step30p1_physical.py` -> `maps/h8r2_gateway_preserving_nav_map.yaml`: reference wall validation
- `validate_stage1_step30p1_physical.py` -> `stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`: physical room visit mask
- `publish_stage1_step30p1_rviz_overlay.py` -> `stage1_committed_public/topology_v0_1.json`: room polygons, room labels, topology overlay
- `publish_stage1_step30p1_rviz_overlay.py` -> `stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`: semantic fill overlay
- `publish_stage1_step30p1_rviz_overlay.py` -> `route_query_result_v0_1.json and semantic_route_waypoints_v0_1.json`: route/gateway/interior marker overlay

## File Consumption Graph

- `stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz`: produced by Stage-A/Stage1 canonical rerun or restored process artifacts; consumed by build_stage1_full_scene_occupancy_map.py, build_stage1_step30p1_request_aware_nav_map.py, prepare_stage1_step30p1_semantic_route.py, validate_stage1_step30p1_physical.py; role: contains free_space, gateway_wall_preclose, segmentation_wall_processed, structural_wall, outside_boundary layers.
- `stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`: produced by Stage-A/Stage1 room segmentation; consumed by build map scripts, prepare route target selection, RViz overlay publisher, physical validators; role: room masks for map projection, target choice, overlays, and physical visit checks.
- `config/step30s7_gateway_projection_config_v0_1.json`: produced by gateway extraction / config externalization; consumed by step30s7_common.load_config, map builders, route preparer, gateway readiness report; role: selected gateway pairs, forbidden pairs, input paths.
- `maps/stage1_full_scene_occupancy_map.yaml/.pgm/.npz`: produced by build_stage1_full_scene_occupancy_map.py; consumed by launch_stage1_step30p1_gazebo_nav2.sh, Nav2 map_server via launch override, RViz /map, prepare route planner grid, validators; role: stable full-scene static occupancy map.
- `current_validation/<run_id>/route_query_result_v0_1.json`: produced by prepare_stage1_step30p1_semantic_route.py; consumed by run_stage1_step30p1_route.py, publish_stage1_step30p1_rviz_overlay.py, validate_stage1_step30p1_physical.py, stable_map_gui_report writer; role: room sequence, gateway sequence, map profile/path, interior targets.
- `current_validation/<run_id>/semantic_route_waypoints_v0_1.json`: produced by prepare_stage1_step30p1_semantic_route.py; consumed by route runner, overlay publisher, physical validator; role: continuous path waypoints, semantic gateway points, bridge points, interior targets.
- `Nav2 nodes + /map + static map->odom TF`: produced by launch_stage1_step30p1_gazebo_nav2.sh; consumed by dataplane probe, route runner, RViz, validators through recorded trajectory; role: runtime ROS graph.
- `route_execution_result_v0_1.json and trajectory_sample_result_v0_1.json`: produced by run_stage1_step30p1_route.py; consumed by physical validator, overlay publisher, final report; role: execution outcome and sampled trajectory.

## Required Runtime Inputs vs Diagnostic/Provenance Files

Required runtime inputs include: `stage1_committed_public/topology_v0_1.json`, `stage1_committed_public/topology_query_report.json`, `stage1_committed_public/committed_room_world_model_v0_1.json`, `stage1_committed_public/committed_room_world_snapshot_v0_1.json`, `config/step30s7_gateway_projection_config_v0_1.json`, `gateway/gateway_registry_v0_1.json`, `stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json`, `stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz`, `stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.json`, `stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`, `maps/h8r2_gateway_preserving_nav_map.yaml`, `maps/h8r2_gateway_preserving_nav_map.pgm`, `maps/h8r2_gateway_preserving_masks.npz`, `maps/stage1_full_scene_occupancy_map.yaml`, `maps/stage1_full_scene_occupancy_map.pgm`, `maps/stage1_full_scene_occupancy_map.npz`, `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf`, `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py`, `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml`, `rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`, `current_validation/<run_id>/route_query_result_v0_1.json`, `current_validation/<run_id>/semantic_route_waypoints_v0_1.json`.

Diagnostic/provenance files include: `stage1_process/room_segmentation/visualizations/*.png`, `stage1_process/gateway_extraction/visualizations/pair_local/**/*.png`, `stage1_committed_public/artifact_source_manifest_v0_1.json`, `manifest/provenance_manifest_v0_2.json`, `current_validation/*/stable_map_gui_report_v0_1.{json,md}`, `current_validation/*/dataplane_probe_result_v0_1.{json,md}`, `current_validation/*/rviz_process_check_v0_1.json`, `current_validation/*/gazebo_process_check_v0_1.json`, `current_validation/*/visual_diagnostics/*.png`.

The committed/public files (`topology_v0_1.json`, `topology_query_report.json`, `committed_room_world_model_v0_1.json`, `committed_room_world_snapshot_v0_1.json`) document public semantic topology and query state. They do not directly prove Nav2 execution; route execution depends on generated waypoints, the stable map, Nav2/RViz/Gazebo runtime assets, and validation outputs.

## Dual-Wall Raster Contract

`segmentation_wall_processed.png` is the room-segmentation wall layer. It may include blur, thresholding, and morphological close; it can close doorways and must not be treated as the gateway/navigation wall source.

`gateway_wall_preclose.png` is the gateway-preserving wall layer for gateway extraction, gateway validation, and navigation-map carving. Its thresholded export `final_gateway_wall_preclose_thr_0p25.png` plus metadata/overlays are important diagnostics and stable-map provenance references. The active map builder consumes the corresponding layers from `00824_step30a_layered_bev_v0_1.npz`, combines wall evidence, then restores selected gateway carve cells as free.

The key 00824 visualization files are `free_space.png`, `unknown_layer.png`, `outside_boundary.png`, `room_mask_global_id.png`, `segmentation_wall_processed.png`, `gateway_wall_preclose.png`, `final_gateway_wall_preclose_thr_0p25.png`, `gateway_wall_preclose_threshold_metadata.png`, `gateway_wall_preclose_over_free_space.png`, `gateway_wall_preclose_over_room_mask.png`, `segmentation_wall_vs_gateway_wall_preclose.png`, and `final_walls_skeleton.png`.

## Gateway and Forbidden-Shortcut Contract

Allowed gateway pairs are `r1-r3`, `r3-r7`, `r3-r8`, `r7-r11`, `r7-r14`, `r14-r16`, `r7-r15`, and `r11-r8`. Forbidden shortcuts are `r3-r11`, `r8-r14`, `r14-r15`, `r15-r16`, `r3-r15`, and `r7-r16`.

Gateway registry: `stage_outputs/stage1_00824_step30p1/gateway/gateway_registry_v0_1.json`. Runtime gateway config: `stage_outputs/stage1_00824_step30p1/config/step30s7_gateway_projection_config_v0_1.json`. The config is consumed by map builders and route preparation; readiness reports explicitly state that full multi-scene gateway extraction generalization is not claimed.

## Stable Occupancy Map Contract

Stable map files are `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`, `.pgm`, and `.npz`. They are created by `tools/stage1_step30p1/build_stage1_full_scene_occupancy_map.py`.

The builder reads H8R2 map metadata/baseline, H8R2 selected gateway carve masks, Stage1 layered BEV layers, the Stage1 room mask, and gateway config/registry. It is request-independent: all Stage1 room-mask cells that are Stage1 free-space and not wall evidence become free; wall evidence remains occupied; selected gateway carve cells are restored as free. Nav2 loads the YAML via launch override and RViz displays `/map`.

## Route and Waypoint Contract

Route query/waypoints are created by `prepare_stage1_step30p1_semantic_route.py`.

Room8 canonical route: `room_1 -> room_3 -> room_8 -> room_11 -> room_7 -> room_14 -> room_16`. Gateways: `gw_00824_r1_r3_01, gw_00824_r3_r8_01, gw_00824_r8_r11_01, gw_00824_r7_r11_02, gw_00824_r7_r14_01, gw_00824_r14_r16_01`.

Room15 through-room route: `room_1 -> room_3 -> room_7 -> room_15 -> room_7 -> room_14 -> room_16`. Gateways: `gw_00824_r1_r3_01, gw_00824_r3_r7_01, gw_00824_r7_r15_01, gw_00824_r7_r15_01, gw_00824_r7_r14_01, gw_00824_r14_r16_01`.

Waypoints are JSON records with `route_id`, `waypoint_index`, `x`, `y`, `yaw`, `source`, `from_room`, `to_room`, `gateway_id`, `pair_key`, and `reason`. Sources include semantic segment points, gateway crossings, bridge path points, and explicit through-room/terminal interior targets. Bridge points shape FollowPath but are forbidden as sparse fallback semantic goals. A physical room15 visit must be proven by trajectory/mask/dwell validation, not by route inclusion.

## Gazebo Runtime Contract

World file: `stage_outputs/stage1_00824_step30p1/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf`.

The world name is `boxfusion_step30p1_00824_large_continuous_floor`. It includes a large continuous 70m x 70m collision/visual floor, static gateway carve/center markers, room anchors, and route polyline markers. The launch script spawns a TurtleBot3 burger and can reset it to route waypoint 0. From static file inspection, the continuous floor provides one stable Gazebo support plane under the 2D Nav2 map/overlay rather than scene-accurate collision walls.

## Nav2 Runtime Contract

Launch file: `stage_outputs/stage1_00824_step30p1/nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py`. Params: `stage_outputs/stage1_00824_step30p1/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml`.

The launch file starts map_server, controller_server, planner_server, recoveries_server, bt_navigator, waypoint_follower, and lifecycle manager. It deliberately does not launch AMCL; the shell script starts a static `map -> odom` transform. The params file still contains an `amcl` section as inert config baggage, but no AMCL node is in the active launch graph.

Visible settings include `global_frame=map`, local costmap frame `odom`, `robot_base_frame=base_link`, Navfn `GridBased` planner with `allow_unknown=false`, and DWB `FollowPath` controller with `max_vel_x=0.18`, `robot_radius=0.12`, and inflation radius `0.22`.

## RViz / GUI Overlay Contract

RViz config: `stage_outputs/stage1_00824_step30p1/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`.

The fixed frame is `map`. Displays subscribe to `/map`, `/tf`, `/tf_static`, `/robot_description`, `/scan`, `/stage1_nav/semantic_overlay_markers`, `/plan`, and `/local_plan`. The overlay publisher builds marker namespaces for stable semantic room fills/outlines, requested through-room highlights, planned topology route, gateway crossings, interior targets, room chain labels, request summary, executed trajectory, and wall diagnostics. GUI success requires Gazebo plus RViz BEV semantic overlay; Gazebo alone is not enough.

## Validation Contract

Validation spans lifecycle readiness, dataplane topics/TF/actions, route execution, physical visit, terminal quality, wall crossing, spin/looping, GUI process/overlay evidence, floorplan consistency, and forbidden shortcuts.

`validate_stage1_step30p1_physical.py` checks through-room samples/dwell, room15 gateway-only guard, terminal room16 interior quality, wall point/segment hits against active and reference maps, structural wall overlap, and local spin/looping. `write_current_status_report.py` aggregates room8 and room15 evidence and preserves the accepted wording: `Step30P1 repaired execution succeeded with clean forward-only fallback.`

## Scene-Mismatch Failure Modes

- `map path`: Nav2 /map and RViz floorplan display the wrong scene or wrong floor; route may hit occupied/unknown cells.
- `RViz config`: Overlay topic or fixed frame may be wrong; GUI evidence can show 00824 labels/assets.
- `Gazebo world`: Robot moves over 00824 static route/gateway/floor visual markers even if Nav2 map is 00843.
- `marker namespace/topic`: RViz may subscribe to a stale compatibility topic or hide active floor-specific markers.
- `route waypoints`: FollowPath attempts 00824 coordinates in an 00843 map, causing aborts or occupied-cell hits.
- `stage-output-dir`: overlay publisher and validators load 00824 topology/masks despite 00843 maps.
- `Nav2 params`: frames/tolerances/controller behavior may be okay but embedded stale map defaults/comments can mask wrong launch overrides.
- `floor map`: floor_1/floor_2 mismatch causes topology route to be visually plausible but physically invalid.
- `gateway wall layer`: using segmentation wall closes real doorways or creates false forbidden/allowed gateway connectivity.
- `static TF/frame assumptions`: map->odom identity works only if world, map origin, robot spawn, and route coordinates share the same scene frame.

## 00843 Gap Analysis Against 00824 Contract

00843 has useful committed/public Stage1 JSON artifacts and later reconstructed dual-wall floor_1/floor_2 raster diagnostics. It also has a candidate floor_2 map and route artifacts.

The gaps are runtime isolation and proof: 00843 still has copied 00824 Nav2 launch/config/RViz/world files, 00824 marker/world names, and rejected run evidence showing FollowPath abort, no room_14 visit, wall/occupied-cell failures, and GUI scene leakage. 00843 needs floor-specific gateway registry/validation, world, RViz config, overlay namespaces, Nav2 launch/config defaults, route waypoints, and validation outputs before another Nav2/Gazebo/RViz run can be trusted.

## 00843 00824-Leakage Scan Summary

Leakage hits found: `88` across `25` files; filename hits: `5`.

Classification counts: `{'suspicious but not active': 4, 'active runtime leakage': 76, 'harmless reference/template': 5, 'unknown needs manual inspection': 3}`.

Highest-risk active runtime leakage files: `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml`, `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py`, `nav2/launch/__pycache__/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.cpython-38.pyc`, `nav2/ros_asset_manifest_v0_1.json`, `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf`, `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log`, `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_logs/bringup_result.json`, `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt`, `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/rviz_overlay_process_check.json`, `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/rviz_overlay_publisher.log`, `rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`, `rviz/rviz_manifest_v0_1.json`.

Full leakage scan details are in `00843_00824_string_leakage_scan.md`.

## Which 00843 Evidence Is Rejected vs Potentially Reusable

Rejected: the first 00843 generalization attempt, because it did not rerun canonical Stage1, used the wrong 2D floorplan, used pose-cluster-like placeholder polygons, and produced invalid route/topology-floorplan/GUI evidence.

Rejected: the later floor_2-only Nav2 attempt as success evidence, because FollowPath aborted, room_14 was not visited, wall validation failed, route/trajectory hit occupied cells, and GUI scene isolation failed with 00824 runtime asset leakage. Do not state that 00843 GUI/Nav2 succeeded.

Potentially reusable: canonical Stage1 rerun committed/public artifacts, floor_1/floor_2 dual-wall reconstruction diagnostics, candidate floor_2 route `room_11 -> room_7 -> room_13 -> room_14`, and the floor_2 occupancy map as inputs for a future isolated runtime.

## Proposed Future 00843 Scene-Specific Runtime Contract

Planning target only; no files were moved:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/
  input/
  canonical_stage1/
  committed_public/
  process/
    room_segmentation/
      assets/floor_1/
      assets/floor_2/
      visualizations/floor_1/
      visualizations/floor_2/
    gateway/
  maps/floor_1/
  maps/floor_2/
  routes/floor_2_room11_to_room14/
  runtime/gazebo/
  runtime/nav2/
  runtime/rviz/
  runtime/overlay/
  runs/
  validation/
  current_validation/
  diagnostics/
```

Every runtime path in that future structure should be 00843- and floor-specific, including world model names, launch defaults, RViz config name/title, marker topics/namespaces, route files, map YAML/PGM/NPZ, and validation outputs.

## Final Checklist Before Any Future 00843 Nav2/Gazebo/RViz Run

- No 00824 filenames in active 00843 runtime/nav2/rviz/gazebo paths.
- No 00824 path defaults in launch/config manifests unless inert template comments are explicitly marked.
- Gazebo world contains 00843 floor geometry/markers only.
- RViz config uses 00843/floor-specific title, topic names, and marker namespaces.
- Nav2 map YAML, route waypoints, spawn pose, static TF, world coordinates, and room mask share one floor frame.
- Gateway registry and forbidden-shortcut validation are regenerated/validated for 00843 floor_2.
- Validation must pass FollowPath/fallback outcome, room_14 physical visit, wall/occupied hits, terminal quality, and Gazebo+RViz GUI evidence.

Self-check for this audit:

- No files under the accepted 00824 baseline were modified by this audit.
- No files under `tools/stage1_nav` or `tools/stage1_step30p1` were intentionally modified by this audit; git status already reports a dirty internal overlay publisher file that was preserved untouched.
- No live ROS/Gazebo/Nav2/RViz execution was attempted.
- Audit outputs were written only under the 00843 analysis directory.
- The report distinguishes provenance from runtime file consumption.
- The report does not claim 00843 GUI/Nav2 success.
- The report does not claim AMCL usage.
- The report preserves the accepted Step30P1 wording.
