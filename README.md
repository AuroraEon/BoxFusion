# BoxFusion

## 1. Project Overview

BoxFusion demonstrates the current converged Stage-A / Stage1 indoor topology-route navigation runtime for scene `00824`.

The current active milestone is:

`stage_outputs/stage1_00824_step30p1/`

The current validation root is:

`stage_outputs/stage1_00824_step30p1/current_validation/`

The demo uses committed Stage-A topology artifacts, Stage1 global room/free/wall geometry, a stable full-scene occupancy map, Gazebo TurtleBot3 simulation, Nav2 route execution, and an RViz BEV semantic overlay.

Accepted historical wording remains:

`Step30P1 repaired execution succeeded with clean forward-only fallback.`

Do not rewrite this milestone as uninterrupted `FollowPath` success.

## 2. Current Active Runtime

User-facing commands live under:

`tools/stage1_nav/`

Normal use should go through those wrappers. Internal implementation may still live under:

`tools/stage1_step30p1/`

Treat `tools/stage1_step30p1/` as implementation detail, not the normal user-facing interface.

Historical evidence is archived under:

`stage_outputs/stage1_00824_step30p1/archive/`

Do not add active code or active validation output under `runtime_stage1_frozen_evidence`.

## 3. Pipeline

### Stage-A committed/public artifacts

Stage-A provides the committed room, gateway, topology, and query artifacts used by the Stage1 demo. These define the semantic room graph and the allowed gateway connectivity consumed by route generation.

### Stage1 global geometry and stable occupancy/floorplan map

Stage1 uses global room/free/wall geometry and gateway carve metadata to build the stable full-scene occupancy map. The stable map is not request-specific and is not a room15 patch.

The current primary map is:

`stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`

### Topology query and route generation

The topology query produces room sequences, gateway sequences, and route waypoints. Allowed gateway connections are:

- `r1-r3`
- `r3-r7`
- `r3-r8`
- `r7-r11`
- `r7-r14`
- `r14-r16`
- `r7-r15`
- `r11-r8`

Forbidden shortcuts include:

- `r3-r11`
- `r8-r14`
- `r14-r15`
- `r15-r16`
- `r3-r15`
- `r7-r16`

### Gazebo/Nav2 execution

Gazebo provides the simulated TurtleBot3 runtime. Nav2 uses static `map -> odom` localization, the stable full-scene `/map`, and route execution without AMCL or manual `cmd_vel`.

The repaired runtime now gates route execution on Nav2 lifecycle readiness: `/map_server`, `/controller_server`, `/planner_server`, and `/bt_navigator` must be active, `/map` must publish a real `OccupancyGrid` sample with `TRANSIENT_LOCAL` QoS, and `/follow_path` must be ready.

`/compute_path_to_pose` and `/navigate_to_pose` are recorded as diagnostics for the current FollowPath runtime. They are not hard blockers unless a future planning gate explicitly requires them.

### RViz BEV semantic overlay

RViz displays the stable `/map` floorplan plus a separate semantic overlay for room masks, labels, gateways, planned route, executed trajectory, and route highlights. Gazebo-only operation is not full GUI success; GUI success requires Gazebo plus RViz BEV semantic overlay.

### Validation

Validation checks route execution, trajectory samples, through-room physical visit evidence, room16 terminal quality, wall consistency, local spinning/looping, marker lifecycle, GUI process behavior, route/topology integrity, and cross-request floorplan consistency.

## 4. Current Map and Floorplan Architecture

The default primary Nav2/RViz floorplan is:

`stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`

Associated map files:

- `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.pgm`
- `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.npz`

The runtime has three separate visual/physical layers:

- Gazebo physical floor: a large continuous simulation floor for TurtleBot3.
- Nav2/RViz `/map`: the stable full-scene occupancy/floorplan map.
- RViz semantic overlay: room masks, labels, gateways, route, trajectory, and diagnostics.

Request-aware maps may remain as explicit debug/reference profiles only. They are not the default primary RViz/Nav2 floorplan.

## 5. Current Validated Routes

Room8 route:

`room_1 -> room_3 -> room_8 -> room_11 -> room_7 -> room_14 -> room_16`

Room15 route:

`room_1 -> room_3 -> room_7 -> room_15 -> room_7 -> room_14 -> room_16`

Room15 physical visit is validated by trajectory and dwell, not by topology inclusion alone. Room8 behavior is validated by wall and spin checks.

## 6. How to Run

Stop any active Stage1 runtime:

```bash
tools/stage1_nav/stop.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

Run the lifecycle-ready room8 GUI validation:

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_8 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --keep-gui-open-sec 60 \
  --run-id room8_lifecycle_ready_gui_check
```

Run the lifecycle-ready room15 GUI validation:

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_15 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --through-room-dwell-sec 3.0 \
  --through-room-min-inside-samples 8 \
  --keep-gui-open-sec 60 \
  --run-id room15_lifecycle_ready_gui_check
```

Verify the current artifact:

```bash
/usr/bin/python3 tools/stage1_nav/verify_current_artifact.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

Build the stable full-scene map:

```bash
/usr/bin/python3 tools/stage1_nav/build_stable_map.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

## 7. Current Evidence and Validation Results

Latest manual-runtime evidence:

- `stage_outputs/stage1_00824_step30p1/current_validation/room8_lifecycle_ready_gui_check/`
- `stage_outputs/stage1_00824_step30p1/current_validation/room15_lifecycle_ready_gui_check/`
- `stage_outputs/stage1_00824_step30p1/current_validation/current_status_report.json`
- `stage_outputs/stage1_00824_step30p1/current_validation/current_status_report.md`

Current status:

- stable occupancy map: passed
- Nav2 lifecycle readiness: passed
- RViz floorplan consistency: passed
- room8 behavior: passed
- room15 physical visit: passed
- room15 cross-request display: passed
- marker lifecycle: passed
- GUI performance: passed
- route/topology integrity: passed
- overall: passed

Latest lifecycle-ready evidence:

- room8 reached room16 using the stable full-scene map.
- room8 wall/spin validation passed with zero wall point violations, zero wall segment violations, zero structural wall overlaps, and `spinning_detected=false`.
- room15 physically entered room15 with 53 inside samples and 23.457 seconds dwell.
- room15 gateway-only failure guard passed.
- room16 terminal quality passed.
- sparse fallback was not used in the lifecycle-ready room8 or room15 runs.
- bridge waypoints were not used as `NavigateToPose` goals.
- `/map_server`, `/controller_server`, `/planner_server`, and `/bt_navigator` reached `active`.
- `/map` produced a `TRANSIENT_LOCAL` `OccupancyGrid` sample.
- `/follow_path` was ready before route execution.
- room15 cross-request floorplan consistency remains passed.

## 8. Important Boundaries / Non-Claims

Current project boundaries:

- no AMCL
- static `map -> odom` localization
- no manual `cmd_vel`
- no real physical robot deployment claim
- no full collision-free guarantee
- no online object detection
- object-level navigation is future work, not complete
- no full multi-scene gateway extraction generalization claim
- topology route inclusion is not physical room visit
- Gazebo-only is not full GUI success
- request-aware maps are debug/reference only, not the default primary floorplan

## 9. Directory Guide

- `stage_a_demo.py`: Stage-A pipeline entry point.
- `tools/stage1_nav/`: current public Stage1 navigation commands.
- `tools/stage1_step30p1/`: internal implementation scripts used by the wrappers.
- `stage_outputs/stage1_00824_step30p1/`: active Stage1 milestone artifact.
- `stage_outputs/stage1_00824_step30p1/maps/`: active and reference occupancy map artifacts.
- `stage_outputs/stage1_00824_step30p1/current_validation/`: current validation reports and run evidence.
- `stage_outputs/stage1_00824_step30p1/archive/`: historical evidence archive.
- `stage_outputs/stage1_00824_step30p1/gateway/`: gateway registry and topology artifacts.
- `stage_outputs/stage1_00824_step30p1/stage1_process/`: Stage1 geometry and BEV artifacts.
- `baselines/`: reserved for future comparison baselines only.

## 10. Notes for Future Work

- Object-level navigation remains future work.
- Future planning gates may make `/compute_path_to_pose` or `/navigate_to_pose` hard blockers, but the current FollowPath runtime does not.
- Keep request-aware maps as debug/reference outputs only.
- Keep `tools/stage1_nav/` as the normal user-facing runtime surface.
- Keep historical evidence archived rather than restoring old StepXX folders as active project directories.
