# RSLG-SLAM

## What This Project Is

RSLG-SLAM is a route-oriented light-geometry semantic world model: a
semantic-topological world model interface for language-to-route navigation. It
produces queryable, interpretable, routable, and executable-validation-ready
artifacts from rich semantics and light geometry. `BoxFusion` is only the
historical repository path; the project name is RSLG-SLAM.

## What This Project Is Not

RSLG-SLAM is not dense reconstruction, neural implicit SLAM, a full embodied
navigation benchmark, a full BEV planner, Nav2 success, AMCL success, real robot
deployment, physical stair climbing, Unitree Go2 control, quadruped gait
control, a collision-free guarantee, an LLM runtime system, or osmAG-Nav.

The current formal runtime policy has no active Nav2, AMCL, `map_server`,
`nav2_map_server`, ROS lifecycle, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, or `FollowPath` dependency. RViz and Gazebo
are optional visualization/runtime adapters only, and z-aware vertical
transition output is visualization-only.

## Current Static Main Chain

The current formal runnable static chain is frozen canonical mode:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

Current static entrypoints:

- `tools/rslg_pipeline/batch_plan_query_static.py`
- `tools/rslg_pipeline/export_route_result_runtime_inputs.py`

This path consumes frozen canonical Layer 1/2 artifacts. It does not rerun
Stage-A, raw RGB-D inference, ROS nodes, Gazebo, RViz, Habitat, GPU jobs, Nav2,
AMCL, `map_server`, `planner_server`, `controller_server`, `bt_navigator`,
`NavigateToPose`, or `FollowPath`.

## Official Layers

- Layer 0: Input Layer. Raw/provenance input: RGB-D frames, depth, provided
  poses, scene id, sequence id, dataset/config paths, model/checkpoint
  provenance, CLIP checkpoint provenance, semantic class text, text-feature
  provenance, and a Layer 0 manifest.
- Layer 1: World Model Layer. Current static mode consumes frozen canonical
  world-model evidence; full raw RGB-D to Layer 1 rerun remains legacy
  Stage-A-backed.
- Layer 2: Formal Artifact Layer. Frozen canonical stable maps, object
  interfaces, topology, vertical connectors, object approach candidates, and
  planner graph.
- Layer 3: Navigation Interface Layer. QueryTask input plus Layer 2 artifacts,
  producing `RSLGRouteResult`. QueryTask is Layer 3 input, not Layer 0.
- Layer 4: Runtime Validation Layer. RouteResult-derived adapter inputs for the
  lightweight PID follower, RViz marker records, and z-aware overlay records.

## Quickstart

Start with `docs/rslg_slam_planner/STATIC_PIPELINE_QUICKSTART.md`.
For the Layer 4 visualization surface, use
`docs/rslg_slam_planner/VISUALIZATION_QUICKSTART.md`.
For lightweight PID executable replay over RouteResult-derived runtime inputs,
use `docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`.
For stable-map footprint collision diagnosis and bounded PID replay mitigation,
use `docs/rslg_slam_planner/PID_REPLAY_COLLISION_MITIGATION.md`.

Use the current static entrypoints above with:

- Queryset manifest: `configs/rslg_queryset_v0/queryset_manifest.json`
- Canonical root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- Task output root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`

## Authoritative Documentation

The current authoritative truth source is `docs/rslg_slam_planner/`. Key entry
points:

- `docs/rslg_slam_planner/HANDOFF_README.md`
- `docs/rslg_slam_planner/LAYER_ENTRYPOINTS.md`
- `docs/rslg_slam_planner/FORMAL_ARTIFACT_INDEX.md`
- `docs/rslg_slam_planner/PAPER_NARRATIVE_ENGINEERING_MAP.md`
- `docs/rslg_slam_planner/VISUALIZATION_QUICKSTART.md`
- `docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`
- `docs/rslg_slam_planner/PID_REPLAY_COLLISION_MITIGATION.md`
- `docs/rslg_slam_planner/LEGACY_BOUNDARY.md`

The old `docs/rslg_slam/` tree was migrated and deleted. Do not recreate it.

## Current Main Scene

- Scene id: `00843-DYehNKdT76V`
- Canonical root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- Task root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`
- Object target: `obj_175` / `curtain` / `room_14` / `floor_2`
- Valid approach: `generated_ring_002`, position `[-7.020484, 1.558795]`, yaw
  `-2.09057`, clearance `0.20`
- Blocked evidence candidate: `generated_ring_037`; never select it as a
  runtime goal
- True transition edge: `vt_1_centerline_e001`
- Forbidden non-transition edge: `vt_1_centerline_e003`; never use it as a
  transition edge
- Visualization-only floor z: `floor_1 = 0.0`, `floor_2 = 1.6`

## Legacy Boundary

Old `00824`, `Step30P1`, `Stage1`, `tools/stage1_nav`, Nav2, AMCL,
`map_server`, ROS lifecycle, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, `FollowPath`, RViz/Gazebo GUI showcase, and
task39/task41/task42/task48 material is historical only. It is not the current
formal static path, and this README intentionally does not provide historical
active runtime commands.
