# RSLG-SLAM Project Truth

This document is the permanent truth record for the RSLG-SLAM pipeline convergence refactor.

## Official Project Name

The project name is RSLG-SLAM.

The repository path `/home/ws/workspace/BoxFusion` is historical path context only and is not the project name.

## Official Five-Layer Pipeline

- Layer 0: Input Layer
- Layer 1: World Model Layer
- Layer 2: Formal Artifact Layer
- Layer 3: Navigation Interface Layer
- Layer 4: Runtime Validation Layer

`tools/rslg_pipeline/` is the formal command surface for these layers.

## Positioning

RSLG-SLAM is positioned as Rich Semantic + Light Geometry:

- posed RGB-D semantic-topological world-modeling backend
- navigation-interface-oriented light-geometry semantic topology
- structured semantic-topological navigation interface
- formal artifacts for route and validation

## Forbidden Claims

The project must not claim:

- dense reconstruction
- neural implicit SLAM
- full embodied navigation benchmark
- full BEV planner
- AMCL success
- LLM runtime system
- real robot deployment
- collision-free guarantee
- osmAG-Nav

## Main Scene

The main scene is `00843-DYehNKdT76V`.

The canonical output root is:

`/home/ws/workspace/BoxFusion/stage_outputs/rslg_slam/00843-DYehNKdT76V/`

The canonical directories are:

- `/home/ws/workspace/BoxFusion/stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- `/home/ws/workspace/BoxFusion/stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`

## Canonical Cross-Floor Room Route

The canonical cross-floor room route is:

1. `room_2` on `floor_1`
2. `room_3` on `floor_1`
3. `vt_1` / `vc_vt_1`
4. `room_7` on `floor_2`
5. `room_13` on `floor_2`
6. `room_14` on `floor_2`

Do not reverse `floor_1` and `floor_2`.

## Vertical Connector Truth

- True transition edge: `vt_1_centerline_e001`
- Non-transition edge: `vt_1_centerline_e003`

Do not use `vt_1_centerline_e003` as the transition edge.

## Object Navigation Truth

- Query: `curtain in room_14 on floor_2`
- Object id: `obj_175`
- Label: `curtain`
- Floor: `floor_2`
- Room: `room_14`

The current valid object approach goal is:

- Candidate id: `generated_ring_002`
- Position: `[-7.020484, 1.558795]`
- Yaw: `-2.09057`
- Clearance: `0.20 m`

`generated_ring_002` is an object approach goal, not the object centroid.

`generated_ring_037` is blocked / occupied / legacy evidence only and must not be used as a runtime goal.

## Environment Rules

Offline Python / artifact / Layer 1-3 tools should use:

`/home/ws/miniconda3/envs/boxfusion/bin/python`

ROS2 / Nav2 / RViz / Gazebo / rclpy tools should use:

`/usr/bin/python3`

Do not run long GPU jobs for static convergence checks.

Do not run live ROS, Gazebo, or RViz demos in convergence refactor tasks.
