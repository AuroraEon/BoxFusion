# Current Project State

## Active Runtime

Stage1 navigation for scene `00824-Dd4bFSTQ8gi` is converged on:

- User-facing commands: `tools/stage1_nav/`
- Internal implementation: `tools/stage1_step30p1/`
- Current validation: `stage_outputs/stage1_00824_step30p1/current_validation/`
- Historical evidence archive: `stage_outputs/stage1_00824_step30p1/archive/`

Accepted Step30P1 wording remains: "Step30P1 repaired execution succeeded with clean forward-only fallback." This is not an uninterrupted FollowPath claim.

## Stable Map

The primary Nav2/RViz floorplan is now the stable full-scene occupancy map:

- Map YAML: `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`
- Map image: `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.pgm`
- Map NPZ: `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.npz`
- Provenance: `stage_outputs/stage1_00824_step30p1/current_validation/stable_full_scene_occupancy_map_provenance.json`

The map is generated from Stage1 global geometry artifacts:

- `stage1_process/room_segmentation/visualizations/final_gateway_wall_preclose_thr_0p25.png`
- `stage1_process/room_segmentation/visualizations/outside_boundary.png`
- `stage1_process/room_segmentation/visualizations/free_space.png`
- `stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz`
- `stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`
- `gateway/gateway_registry_v0_1.json`

The default runtime no longer uses the request-aware projection as the primary `/map` floorplan. Request-aware maps may remain as debug/reference artifacts, but the active `stable` and `auto` profiles use the stable full-scene map.

## Current Validation

Latest integrated validation report:

- `stage_outputs/stage1_00824_step30p1/current_validation/current_status_report.json`
- `stage_outputs/stage1_00824_step30p1/current_validation/current_status_report.md`

Statuses from the current report:

| Area | Status |
|---|---|
| Stable occupancy map | passed |
| RViz floorplan consistency | passed |
| Room8 behavior | passed |
| Room15 physical visit | passed |
| Room15 cross-request display | passed |
| Marker lifecycle | passed |
| GUI performance | passed |
| Route/topology integrity | passed |
| Overall | passed |

Room8 evidence is under `current_validation/room8_stable_map_gui_check/`. Room15 evidence is under `current_validation/room15_stable_map_gui_check/`.

Key validation facts:

- Both room8 and room15 GUI runs loaded `maps/stage1_full_scene_occupancy_map.yaml` through Nav2 map_server.
- Room15 base floorplan values were identical across the room8 and room15 requests.
- Room15 stable map coverage is 1,295 free cells, 29 occupied cells, and 0 unknown cells inside the 1,324-cell room15 mask.
- Room8 passed wall/spin validation with zero wall point violations, zero wall segment violations, and zero structural wall overlaps.
- Room15 physical visit passed with 52 inside samples and 23.151 seconds of room15 dwell.
- Both routes used the expected gateway sequences and did not use bridge waypoints as NavigateToPose goals.
- Marker lifecycle validation found the overlay clear path and current run ids; Gazebo + RViz process checks passed.

## Runtime Notes

- No AMCL; localization is static `map -> odom`.
- No manual `cmd_vel`.
- No real physical robot deployment claim.
- No object-level navigation claim.
- No online object detection claim.
- No full collision-free guarantee.
- Do not treat topology route inclusion as physical room visit; physical room visits are validated from trajectory samples and dwell time.
- Gazebo-only GUI is not counted as full GUI success; current GUI success requires Gazebo plus RViz BEV semantic overlay.
- Step30S7 is not documented as a full final success milestone.

## Commands

```bash
tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_8 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --run-id room8_stable_map_gui_check

tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_15 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --through-room-dwell-sec 3.0 --through-room-min-inside-samples 8 \
  --run-id room15_stable_map_gui_check

tools/stage1_nav/stop.sh --stage-output-dir stage_outputs/stage1_00824_step30p1
```
