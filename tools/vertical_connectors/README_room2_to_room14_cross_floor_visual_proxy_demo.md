# RSLG-SLAM task24g Room 2 to Room 14 Cross-Floor Visual Proxy Demo

This demo replays the existing task24f 3D route contract with a Gazebo visual quadruped proxy. The proxy is moved directly with Gazebo `SetEntityState` along:

`room_2 -> room_3 -> vc_vt_1_from_binding -> vt_1_centerline_n000 -> vt_1_centerline_n001 -> vt_1_centerline_n002 -> vt_1_centerline_n003 -> vt_1_centerline_n004 -> vc_vt_1_to_binding -> room_7 -> room_13 -> room_14`

The real floor-transition edge is `vt_1_centerline_e001`, from `vt_1_centerline_n001` on `floor_1` to `vt_1_centerline_n002` on `floor_2`.

## Run

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_demo.sh
```

Use `--keep-open` to leave Gazebo/RViz open after the traversal, or `--no-rviz` when running headless.

## Expected Behavior

Gazebo spawns or reuses `rslg_cross_floor_quadruped_proxy` and the task24g player updates its `x/y/z/yaw` pose through `SetEntityState`. The proxy starts near `room_2`, moves through `room_3`, climbs the `vt_1` connector in z, reaches `room_7`, continues through `room_13`, and ends near `room_14`.

RViz uses `tools/vertical_connectors/rviz_room2_to_room14_cross_floor_visual_proxy.rviz` and should show:

- planned 3D route on `/rslg/cross_floor_planned_route`
- executed 3D trajectory on `/rslg/cross_floor_executed_trajectory`
- proxy/checkpoint/claim markers on `/rslg/cross_floor_visual_proxy_markers`
- TF/odom for `rslg_cross_floor_base_link`
- transition marker for `vt_1_centerline_e001`

## Claim Boundary

This is `visual_kinematic_proxy_only` and `topological_vertical_transition_only`.

`physical_stair_climbing_supported=false`

No gait, no footstep planning, no contact-based stair climbing, and no real quadruped stair locomotion are claimed. The script does not launch Nav2 and does not make an AMCL/localization claim.

## What This Does Not Prove

This demo does not prove physical stair traversal, legged locomotion, controller stability, contact dynamics, foot placement, traction, collision feasibility, or real robot deployment. Same-floor room segments use the task24f route contract as visual playback input; this is downstream visualization and kinematic playback, not a Stage-A rerun.

## Outputs

The player writes task24g artifacts under:

`stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g_room2_to_room14_cross_floor_visual_proxy_traversal_demo/`

Important files include `task24g_report.md`, `task24g_report.json`, `planned_3d_route_v0_1.json`, `executed_3d_trajectory.csv`, `checkpoint_validation.json`, `z_transition_validation.json`, `gazebo_entity_state_validation.json`, and `manual_gui_validation_instructions.md`.

## Troubleshooting

If the Gazebo state service is not found, check:

```bash
ros2 service list | grep -E 'set_entity_state|get_entity_state'
```

The player detects `/set_entity_state`, `/demo/set_entity_state`, and `/gazebo/set_entity_state`. If none exists, the report classifies the run as `task24g_blocked_by_gazebo_entity_or_set_entity_state`.

If the entity is missing, inspect:

```bash
cat stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g_room2_to_room14_cross_floor_visual_proxy_traversal_demo/launch_logs/spawn_entity.log
```

If RViz does not show the route, set Fixed Frame to `world`, then add `MarkerArray` displays for:

- `/rslg/cross_floor_planned_route`
- `/rslg/cross_floor_executed_trajectory`
- `/rslg/cross_floor_visual_proxy_markers`

If `world` and `map` frames are disconnected, the launcher starts a static `world -> map` TF publisher. You can also run:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
```
