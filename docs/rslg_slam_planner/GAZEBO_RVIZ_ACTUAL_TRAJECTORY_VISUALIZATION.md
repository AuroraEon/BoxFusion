# Gazebo RViz Actual Trajectory Visualization

This document describes the RSLG-SLAM Gazebo RViz view for comparing the planned route with the actual Gazebo odom trajectory.

## Why This Replaces Replay As The Main View

The earlier RViz showcase emphasized:

- `/rslg/pid_replay_path`
- `/rslg/robot_pose`

Those topics are useful for replay review, but Gazebo simulation validation should inspect what the simulator actually produced from `/odom`. The Gazebo-specific view therefore uses:

- `/rslg/planned_path_odom`
- `/rslg/gazebo_executed_path`
- `/rslg/gazebo_robot_pose`
- `/rslg/gazebo_actual_marker_array`

Fixed frame: `odom`.

## Start Gazebo GUI

Source ROS 2 first:

```bash
source /opt/ros/foxy/setup.bash
```

Then start the same-floor Gazebo world. For TurtleBot3 Burger:

```bash
export GAZEBO_MODEL_PATH="$PWD/tools/rslg_pipeline/gazebo/models:${GAZEBO_MODEL_PATH:-}"
gazebo --verbose tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty_turtlebot3_burger.world
```

Repo-local fallback world:

```bash
export GAZEBO_MODEL_PATH="$PWD/tools/rslg_pipeline/gazebo/models:${GAZEBO_MODEL_PATH:-}"
gazebo --verbose tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty.world
```

## Start The Actual Trajectory RViz Node

```bash
tools/rslg_pipeline/gazebo/run_gazebo_rviz_actual_trajectory_showcase.sh --no-rviz-gui
```

The default query is:

`00843_object_in_room_curtain_room14`

The default profile is:

`practical_zero_collision`

The default floor filter is:

`floor_2`

## Start RViz GUI

```bash
rviz2 -d tools/rslg_pipeline/rviz/config/rslg_gazebo_actual_trajectory_showcase.rviz
```

You can also start the node and RViz together:

```bash
tools/rslg_pipeline/gazebo/run_gazebo_rviz_actual_trajectory_showcase.sh --with-rviz-gui
```

This script does not start Gazebo or the PID follower.

## Start The Gazebo PID Follower

With Gazebo already running:

```bash
tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh --no-gazebo --duration-sec 240
```

The PID follower subscribes to `/odom`, publishes `/cmd_vel`, and uses `anchor_first_waypoint_to_odom_start`.

## Odom Anchoring

The Gazebo follower anchors the first selected route waypoint to the first odom pose. The RViz actual trajectory node mirrors that policy:

- first selected route waypoint: route anchor
- first received `/odom` pose: odom anchor
- planned path output: `/rslg/planned_path_odom`
- output frame: `odom`

The XY/yaw transform follows the PID follower. Z is anchored for visualization so floor_2 waypoints render at the Gazebo odom height instead of semantic floor height.

## Verify The Overlay

In RViz:

- Blue path: planned route in odom frame.
- Yellow path: executed Gazebo odom trajectory.
- Pose arrow: current Gazebo robot pose.
- Markers: target, selected approach, and guard evidence.

The planned path appears after the first `/odom` pose arrives.

## Inspect Final Error And Deviation

Use:

```bash
tools/rslg_pipeline/gazebo/compare_plan_vs_gazebo_trajectory.py \
  --pid-input-json stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56c_pid_profile_promotion_and_regression_validation/regression_pack/runtime_adapter_inputs/pid_follower_inputs/00843_object_in_room_curtain_room14_pid_runtime_input.json \
  --trajectory-csv stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task58a_gazebo_pid_same_floor_simulation_scaffold/gazebo_sim_pack/runs/00843_object_in_room_curtain_room14_20260707T082155Z/00843_object_in_room_curtain_room14_gazebo_pid_trajectory.csv \
  --profile-json configs/rslg_runtime_profiles/pid_profiles_v0_1.json \
  --profile-id practical_zero_collision \
  --query-id 00843_object_in_room_curtain_room14 \
  --floor-id floor_2 \
  --anchor-first-waypoint-to-odom-start \
  --output-json /tmp/rslg_plan_vs_gazebo.json \
  --output-md /tmp/rslg_plan_vs_gazebo.md
```

Task58b measured the latest successful task58a primary run as:

- final error: `0.013358 m`
- mean nearest path deviation: `0.006885 m`
- max nearest path deviation: `0.064507 m`

## Out Of Scope

This Gazebo/RViz view does not claim:

- Stage-A or raw RGB-D inference
- Nav2
- AMCL
- map_server
- physical robot deployment
- physical stair climbing
- Unitree gait control
- cross-floor physical traversal
- global collision-free guarantee
