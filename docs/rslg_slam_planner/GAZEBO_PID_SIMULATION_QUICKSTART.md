# RSLG-SLAM Gazebo PID Simulation Quickstart

This quickstart runs a bounded same-floor Gazebo scaffold for the current
RSLG-SLAM runtime validation chain:

`QueryTask -> RSLGRouteResult -> RouteResult-derived PID runtime input -> practical_zero_collision -> Gazebo /odom feedback and /cmd_vel control`

It does not run Stage-A, raw RGB-D inference, Nav2, AMCL, `map_server`,
`nav2_map_server`, `planner_server`, `controller_server`, `bt_navigator`,
`NavigateToPose`, `FollowPath`, physical robot code, Unitree gait control, or
stair traversal.

## Source ROS 2

Use the ROS 2 environment available on the machine:

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/foxy/setup.bash
```

`BoxFusion` is only the historical repository path. The project is RSLG-SLAM.

## Dry-Run

Dry-run checks that the PID runtime input, promoted profile, floor filter, and
waypoint extraction load correctly. It does not start Gazebo.

```bash
tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh --dry-run
```

The default query is `00843_object_in_room_curtain_room14`, filtered to
`floor_2` for same-floor simulation. To run the shorter object approach:

```bash
RSLG_QUERY_ID=00843_object_to_path_curtain \
tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh --dry-run
```

## Headless Gazebo Smoke

The headless smoke starts `gzserver` with
`tools/rslg_pipeline/gazebo/worlds/rslg_flat_empty.world`, then runs the
lightweight PID follower against `/odom` and `/cmd_vel`.

```bash
RSLG_QUERY_ID=00843_object_to_path_curtain \
tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh --headless --duration-sec 60
```

Outputs are written under:

```text
stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task58a_gazebo_pid_same_floor_simulation_scaffold/gazebo_sim_pack/runs/
```

Use `--no-gazebo` only when another same-floor simulator is already publishing
`/odom` and accepting `/cmd_vel`.

## Gazebo + RViz

Gazebo shows the simulated differential-drive robot moving on the flat floor.
RViz can show the RSLG-SLAM marker showcase and replay path alongside the live
Gazebo odometry path.

```bash
RSLG_QUERY_ID=00843_object_in_room_curtain_room14 \
tools/rslg_pipeline/gazebo/run_gazebo_pid_smoke.sh --headless --with-rviz --duration-sec 60
```

In RViz, keep the display names clear:

- `Gazebo odom`: `/odom`
- `Gazebo robot pose`: TF from `odom` to `base_footprint` when available
- `RSLG planned path`: `/rslg/planned_path`
- `RSLG practical replay path`: `/rslg/pid_replay_path`
- `RSLG semantic markers`: `/rslg/marker_array`

The RSLG replay marker is visualization of the prior lightweight replay; it is
not the simulated robot. The simulated robot is the Gazebo model driven through
`/cmd_vel` with `/odom` feedback.

## Query Selection

Supported task58a smoke queries are:

- `00843_object_in_room_curtain_room14`: primary route, filtered to `floor_2`
  for same-floor Gazebo simulation of the room_13 to room_14/object approach
  portion.
- `00843_object_to_path_curtain`: secondary quick route, a shorter floor_2
  object approach through `generated_ring_002`.

The selected profile is `practical_zero_collision` from
`configs/rslg_runtime_profiles/pid_profiles_v0_1.json`.

## Why Same-Floor First

The current Gazebo scaffold validates control plumbing: `/cmd_vel` publication,
`/odom` feedback, endpoint tracking, and trajectory logging. A flat same-floor
world is the smallest useful runtime step after the deterministic lightweight
PID replay.

Cross-floor semantics remain represented by RSLG-SLAM route artifacts and RViz
handoff markers. Physical stair traversal, Unitree gait control, and real robot
deployment are future integration tasks, not task58a results.

## Claim Boundary

Gazebo smoke success means the same-floor PID control scaffold moved a simulated
differential-drive robot against the selected waypoints. It is not a global
collision-free guarantee, not Nav2 success, not AMCL success, not `map_server`
runtime success, not physical stair climbing, and not real robot deployment.

