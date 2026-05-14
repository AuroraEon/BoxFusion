# Stage1 Step30P1 Tools (Internal Implementation)

**For user-facing commands, see `tools/stage1_nav/README.md`.**

This folder contains the internal implementation scripts for Stage1 navigation.
Do not invoke these directly for normal use - use the consolidated entry points in `tools/stage1_nav/`.

Active map profiles:

- `h8r2`: canonical H8R2 map.
- `step30s7_request_aware`: generated request-aware successor.
- `auto`: uses H8R2 for requests already covered by H8R2, and Step30S7 when through/terminal route rooms need additional interiors.

The old Step30S5 room15-only map is removed from active runtime use. Current tools reject the old profile names instead of selecting them.

One-command Step30S7 GUI route validation:

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

Main Step30S7 tools:

- `build_stage1_step30p1_request_aware_nav_map.py`: builds `maps/step30s7_request_aware_nav_map.*`.
- `validate_stage1_step30p1_request_map.py`: validates per-room coverage and gateway-to-interior connectivity.
- `audit_stage1_step30p1_bev_visual_regression.py`: compares H8R2 to Step30S7 and writes visual diagnostics without using the old map.
- `prepare_stage1_step30p1_semantic_route.py`: resolves arbitrary semantic route requests and derives through/terminal room interior targets.
- `validate_stage1_step30p1_physical.py`: validates through-room visit, terminal quality, spin/looping, and wall crossing against H8R2, Step30S7, and structural wall evidence.

Step30S7 evidence is written under:

```text
stage_outputs/stage1_00824_step30p1/post_restructure_validation/step30s7_request_aware_projection/
```

RViz uses `stage_outputs/stage1_00824_step30p1/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`. The Nav2 map and semantic room mask are separate layers; route, gateway, room, and trajectory overlays are marker topics.

Cleanup:

```bash
tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh \
  --stage-output-dir stage_outputs/stage1_00824_step30p1 \
  --stop
```

Current limitations remain: no AMCL, static `map -> odom`, no manual `cmd_vel`, no real robot deployment claim, no full collision-free guarantee, no object-level navigation, and no full multi-scene gateway generalization claim.
