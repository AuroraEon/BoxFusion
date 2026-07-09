# RSLG-SLAM RViz Showcase Quickstart

This quickstart runs a visualization-only RViz display for the current practical
PID replay chain:

`QueryTask -> RSLGRouteResult -> RouteResult-derived RViz marker input -> practical_zero_collision PID replay trajectory -> z-aware floor overlay -> RViz MarkerArray / Path / moving robot display`

It does not run Stage-A, raw RGB-D inference, Gazebo, Nav2, AMCL, map_server,
FollowPath, NavigateToPose, or physical robot code.

## Primary Demo

The primary query is `00843_cross_floor_object_curtain_room14`. It shows a
cross-floor route to `obj_175` / `curtain` in `room_14` on `floor_2`, the
selected `generated_ring_002` approach, the blocked `generated_ring_037`
evidence marker, and the `vt_1_centerline_e001` connector handoff.

## Source ROS 2

Use the ROS 2 environment available on the machine, for example:

```bash
source /opt/ros/<distro>/setup.bash
```

Then use the ROS-enabled Python. The helper defaults to `/usr/bin/python3`; set
`RSLG_ROS_PYTHON` if your ROS Python is elsewhere.

## Run Only the Publisher

```bash
cd /home/ws/workspace/BoxFusion

RSLG_ROS_PYTHON=/usr/bin/python3 \
tools/rslg_pipeline/rviz/run_rviz_showcase.sh --publisher-only
```

The publisher topics are:

- `/rslg/marker_array`
- `/rslg/planned_path`
- `/rslg/pid_replay_path`
- `/rslg/robot_pose`

The fixed frame is `map`.

## Run RViz Manually

In another sourced ROS 2 shell:

```bash
cd /home/ws/workspace/BoxFusion
rviz2 -d tools/rslg_pipeline/rviz/config/rslg_practical_profile_showcase.rviz
```

If the config fails to load, add these displays manually with fixed frame `map`:

- MarkerArray: `/rslg/marker_array`
- Path: `/rslg/planned_path`
- Path: `/rslg/pid_replay_path`
- Pose: `/rslg/robot_pose`

## Run Publisher and RViz Together

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/<distro>/setup.bash
tools/rslg_pipeline/rviz/run_rviz_showcase.sh
```

The script starts only the RViz showcase publisher and `rviz2` if available.

## Switch Query

Use one of the task56c practical replay query ids:

```bash
RSLG_QUERY_ID=00843_object_in_room_curtain_room14 \
tools/rslg_pipeline/rviz/run_rviz_showcase.sh --publisher-only
```

Available showcase query ids:

- `00843_cross_floor_object_curtain_room14`
- `00843_object_in_room_curtain_room14`
- `00843_cross_floor_room_room2_to_room14`
- `00843_object_to_path_curtain`
- `00843_floor_connector_floor1_to_floor2`
- `00843_blocked_candidate_rejection_curtain`

## Interpret the View

`floor_1` is rendered at `z=0.0`; `floor_2` is rendered at `z=1.6`. The z gap is
a visualization convention for separating semantic floors, not a physical climb
execution.

The blue planned path comes from RouteResult/z-aware overlay waypoints. The
yellow replay path and moving robot marker come from the
`practical_zero_collision` trajectory CSV. The moving marker is replay
visualization, not a live robot.

`generated_ring_002` is the selected valid approach for the curtain target.
`generated_ring_037` is blocked/rejected evidence only and is not selected as a
runtime goal.

`vt_1_centerline_e001` is the connector handoff edge. `vt_1_centerline_e003` is
shown only as forbidden/non-transition evidence when available.

## Claim Boundary

This RViz display visualizes RSLG-SLAM RouteResult-derived artifacts and the
validated lightweight PID replay trajectory. It is not Nav2 runtime, AMCL
localization, map_server navigation, Gazebo simulation, real robot deployment,
physical stair climbing, or a global collision-free guarantee.

