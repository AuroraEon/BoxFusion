# RSLG-SLAM Planner Project Truth

This planner-facing note locks the same project truth used by
`docs/rslg_slam_truth/PROJECT_TRUTH.md` and `tools/rslg_pipeline/project_truth.py`.

## Positioning

RSLG-SLAM is a route-oriented light-geometry semantic world model, also described
as a semantic-topological world model interface for language-to-route navigation.

It is not a dense reconstruction system, neural implicit SLAM system, full
embodied navigation benchmark, full BEV planner, AMCL success report, Nav2
success report, real robot deployment, collision-free guarantee, physical stair
climbing claim, quadruped gait/control stack, LLM runtime, or osmAG-Nav system.

## Formal Layers

- Layer 0: Input Layer
- Layer 1: World Model Layer
- Layer 2: Formal Artifact Layer
- Layer 3: Navigation Interface Layer
- Layer 4: Runtime Validation Layer

The current formal route surface is:

`QueryTask -> plan_query_static / batch_plan_query_static -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

## Runtime Policy

- No Nav2.
- No AMCL.
- No map_server or nav2_map_server active runtime dependency.
- No ROS lifecycle runtime dependency.
- No planner_server, controller_server, or bt_navigator.
- No NavigateToPose or FollowPath action surface.
- The current Layer 4 runtime adapter is the lightweight PID/proportional
  waypoint follower through `/cmd_vel` and `/odom`.
- RViz and Gazebo are optional Layer 4 adapters only.
- z-aware vertical transition output is visualization-only.

## Scene And Guard Truth

- Scene id: `00843-DYehNKdT76V`
- Canonical root:
  `/home/ws/workspace/BoxFusion/stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- Task root:
  `/home/ws/workspace/BoxFusion/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`
- Current valid object approach: `generated_ring_002` at
  `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20`.
- `generated_ring_037` is blocked evidence only and must not be selected as a
  runtime goal.
- True transition edge: `vt_1_centerline_e001`.
- `vt_1_centerline_e003` is a forbidden non-transition edge and must never be
  used as the transition.
- Floor z values are visualization-only: `floor_1` z=`0.0`, `floor_2` z=`1.6`.
