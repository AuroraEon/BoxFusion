# RSLG-SLAM Planner Project Truth

This planner-facing note is part of the current RSLG-SLAM truth surface under
`docs/rslg_slam_planner/`, aligned with `tools/rslg_pipeline/project_truth.py`.
`docs/rslg_slam/` was migrated into this planner truth surface and deleted in
task53b; it must not be restored as a competing source of project truth.

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

`Frozen canonical Layer 1/2 artifacts + Layer 3 QueryTask -> plan_query_static / batch_plan_query_static -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

Layer 0 is the raw input/provenance layer: RGB-D frames, depth, provided poses,
sequence identity, dataset/config paths, semantic class text, model/checkpoint
provenance, text-feature provenance, and an input manifest. QueryTask is not
Layer 0. QueryTask is the input unit for Layer 3: Navigation Interface Layer.

The current task49-task52 static demo path runs in frozen canonical mode. It
consumes existing canonical Layer 1/2 artifacts and does not rerun Stage-A or raw
RGB-D inference. A full raw RGB-D to Layer 1 rerun remains legacy-backed through
Stage-A lineage until a clean current Layer 1 builder exists.

The current authoritative documentation surface is `docs/rslg_slam_planner/`.
Old `00824`, `Step30P1`, `Stage1`, Nav2, AMCL, map_server, task39, task41, and
task42 material is historical evidence only. It is not the current formal
planner/runtime path.

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

## Workspace Policy

Task evidence belongs under task directories in
`stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`. Old generated outputs are
historical evidence, not current truth sources, and should not be restored as
the active source for current docs or validators.
