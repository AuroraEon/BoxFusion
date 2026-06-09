# RSLG-SLAM task24g2 Occupancy-Aware Cross-Floor Visual Proxy Demo

This demo is the opt-in task24g2 successor to task24g. It keeps the same route:

`room_2 -> room_3 -> vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14`

The difference is that same-floor portions are generated with A* over the existing stable occupancy maps:

- `floor_1`: `room_2 -> room_3 -> vt_1_centerline_n000`
- stair connector: `vt_1_centerline_n000 -> ... -> vt_1_centerline_n004`
- `floor_2`: `vt_1_centerline_n004 -> room_7 -> room_13 -> room_14`

The stair connector still uses the graph-level 3D centerline with its z values. It is not physical stair geometry traversal.

## Run

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_astar_demo.sh
```

Use `--rebuild-route` to regenerate `planned_occupancy_aware_3d_route_v0_1.json`, `--keep-open` to leave Gazebo/RViz open, or `--no-rviz` when running headless.

## Expected Visual Result

Gazebo spawns or reuses `rslg_cross_floor_quadruped_proxy`. The visual proxy is moved directly with Gazebo `SetEntityState`.

The proxy should not simply travel in straight lines from topology point to topology point. It should follow denser same-floor A* paths on `floor_1`, ascend the `vt_1` graph-level connector in z, then follow denser same-floor A* paths on `floor_2` until it reaches `room_14`.

RViz uses `tools/vertical_connectors/rviz_room2_to_room14_cross_floor_visual_proxy_astar.rviz` and should show:

- planned occupancy-aware 3D route on `/rslg/cross_floor_planned_route`
- executed trajectory on `/rslg/cross_floor_executed_trajectory`
- semantic checkpoints, stair connector, proxy marker, and claim boundary text
- TF/odom for `rslg_cross_floor_base_link`

## Claim Boundary

This is `visual_kinematic_proxy_only`.

`topological_vertical_transition_only`

`occupancy_aware_same_floor_visual_playback=true`

`physical_stair_climbing_supported=false`

No gait, no footstep planning, no contact-based stair climbing, no real quadruped stair locomotion, no Nav2 execution, and no AMCL/localization are claimed.

## Troubleshooting A* Failures

If route generation fails, inspect:

```bash
cat stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/route_backfill_validation.json
cat stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/astar_segment_validation.json
```

The backfill tool does not silently fall back to straight-line playback. A failed same-floor segment is classified as `task24g2_blocked_by_same_floor_astar_segment`.

If Gazebo state control is unavailable, check:

```bash
ros2 service list | grep -E 'set_entity_state|get_entity_state'
```

The demo does not launch Nav2, does not run AMCL, and does not use a physical controller.

## Outputs

Artifacts are written under:

`stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/`

Important files include `planned_occupancy_aware_3d_route_v0_1.json`, `route_backfill_validation.json`, `astar_segment_validation.json`, `same_floor_path_validation.json`, `stair_connector_segment_validation.json`, `task24g2_report.md`, `task24g2_report.json`, `executed_3d_trajectory.csv`, `executed_3d_trajectory.json`, `checkpoint_validation.json`, `z_transition_validation.json`, `gazebo_entity_state_validation.json`, `rviz_marker_topic_validation.txt`, `claim_boundary.json`, and `manual_gui_validation_instructions.md`.
