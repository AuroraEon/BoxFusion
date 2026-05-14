# Stage1 Navigation Tools

Consolidated entry points for the Stage1 00824 navigation runtime.

## Active Commands

### Run GUI Demo (room8 through-room)
```bash
tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_8 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --keep-gui-open-sec 20 \
  --run-id room8_canonical
```

### Run GUI Demo (room15 through-room)
```bash
tools/stage1_nav/run_gui_demo.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --start-room room_1 --goal-room room_16 \
  --through-rooms room_15 --terminal-room room_16 \
  --from-start --gui --map-profile stable \
  --through-room-dwell-sec 3.0 --through-room-min-inside-samples 8 \
  --keep-gui-open-sec 20 \
  --run-id room15_through
```

### Stop
```bash
tools/stage1_nav/stop.sh --stage-output-dir stage_outputs/stage1_00824_step30p1
```

### Verify Artifact Integrity
```bash
python3 tools/stage1_nav/verify_current_artifact.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

### Build Stable Full-Scene Map Only
```bash
python3 tools/stage1_nav/build_stable_map.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1
```

### Validate a Completed Run
```bash
python3 tools/stage1_nav/validate_current_run.py \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --route-query-json <path> --waypoints-json <path> \
  --route-execution-json <path> --trajectory-json <path> \
  --through-output-json <path> --through-output-md <path> \
  --terminal-output-json <path> --terminal-output-md <path>
```

## Map Profiles

| Profile | Description | Status |
|---------|-------------|--------|
| `stable` | Stable full-scene Stage1 occupancy/floorplan map (default) | Active |
| `reference_h8r2` | H8R2 reference/historical map | Reference only |
| `auto` | Alias for the stable full-scene map | Active |
| `request_aware` | Route-request projection for debugging only | Reference/debug only |

### Removed Profiles
- `step30s5_room15_diagnostic_patch` - Deprecated, not usable
- `step30s5_room15_interior` - Deprecated, not usable

## Architecture

- **Gateway registry**: `stage_outputs/stage1_00824_step30p1/gateway/gateway_registry_v0_1.json`
- **Config**: `stage_outputs/stage1_00824_step30p1/config/step30s7_gateway_projection_config_v0_1.json`
- **Stable map**: `stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`
- **Current validation**: `stage_outputs/stage1_00824_step30p1/current_validation/`
- **Archive**: `stage_outputs/stage1_00824_step30p1/archive/`

## Constraints

- No AMCL. Static map → odom localization.
- No manual cmd_vel.
- No full collision-free guarantee.
- No online object detection.
- No real physical robot deployment.
- Gazebo + RViz GUI demonstration only.
- Step30P1 accepted wording: "Step30P1 repaired execution succeeded with clean forward-only fallback."

## Internal Implementation

Implementation scripts are under `tools/stage1_step30p1/`. Do not invoke them directly for normal use.
