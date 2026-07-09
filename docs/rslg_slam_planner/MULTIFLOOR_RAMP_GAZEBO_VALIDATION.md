# RSLG-SLAM Multi-Floor Ramp Gazebo Validation

This note documents the task59 Gazebo/RViz validation path for the already
planned RSLG-SLAM cross-floor route in scene `00843-DYehNKdT76V`.

Task59 differs from the task58 same-floor Gazebo validation by executing the
complete cross-floor route:

1. floor_1 planned route
2. `vt_1_centerline_e001` represented as a Gazebo ramp surrogate
3. floor_2 planned route
4. room_14 / `obj_175` curtain approach at `generated_ring_002`

The ramp is a simulation surrogate. It preserves the semantic transition
identity for runtime validation, but it is not physical stair climbing, not a
real robot deployment, not Unitree gait control, and not a global
collision-free guarantee.

## Generated Runtime Input

Build the task-local ramp runtime input and generated world:

```bash
tools/rslg_pipeline/gazebo/run_multifloor_ramp_gazebo_validation.sh --dry-run
```

The default query is `00843_cross_floor_object_curtain_room14`. The fallback
query, used only if the preferred route fails audit, is
`00843_cross_floor_room_room2_to_room14`.

The runtime adapter preserves the original floor_1 waypoints, inserts
switchback ramp waypoints for `vt_1_centerline_e001`, and then resumes the
original floor_2 waypoint order. `vt_1_centerline_e003` remains
forbidden/non-transition evidence only. `generated_ring_037` remains
blocked/rejected evidence only. `generated_ring_002` remains the valid object
approach for the preferred object query.

## Manual Gazebo and RViz

Start Gazebo GUI:

```bash
stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task59_multifloor_ramp_gazebo_validation/multifloor_ramp_pack/run_gazebo_gui.sh
```

Start the RViz publisher node:

```bash
stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task59_multifloor_ramp_gazebo_validation/multifloor_ramp_pack/run_rviz_node.sh
```

Start RViz GUI:

```bash
stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task59_multifloor_ramp_gazebo_validation/multifloor_ramp_pack/run_rviz_gui.sh
```

Start the multifloor ramp follower:

```bash
stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task59_multifloor_ramp_gazebo_validation/multifloor_ramp_pack/run_multifloor_follower.sh
```

The RViz fixed frame is `odom`. The primary topics are:

- planned route: `/rslg/multifloor_planned_path_odom`
- executed trajectory: `/rslg/multifloor_gazebo_executed_path`
- robot pose: `/rslg/multifloor_gazebo_robot_pose`
- semantic/ramp markers: `/rslg/multifloor_ramp_marker_array`

Replay topics are not the primary evidence for this view.

## Interpreting Results

The follower anchors the first planned waypoint to the first observed `/odom`
pose when `--anchor-first-waypoint-to-odom-start` is used. That aligns the
planned path and actual Gazebo trajectory in the `odom` frame without requiring
AMCL or map server localization.

Inspect:

- `09_headless_gazebo_ramp_execution_summary.json`
- `11_offline_plan_vs_executed_ramp_comparison_summary.json`
- the run-local follower summary and trajectory CSV under
  `multifloor_ramp_pack/runs/`

If `/odom` z remains planar, that is reported but is not a standalone failure.
The TurtleBot3/Gazebo odom source may not expose full ramp height even when
collision geometry is present.

## Out Of Scope

Task59 does not use or claim:

- Nav2
- AMCL
- map_server / nav2_map_server
- planner_server / controller_server / bt_navigator
- NavigateToPose
- FollowPath
- physical stair climbing
- Unitree stair gait control
- real robot deployment
- global collision-free navigation
