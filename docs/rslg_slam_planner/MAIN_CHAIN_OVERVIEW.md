# RSLG-SLAM Main Chain Overview

RSLG-SLAM is a route-oriented light-geometry semantic world model, also usable as a semantic-topological world model interface for language-to-route navigation.

`BoxFusion` is only the historical repository path.

`docs/rslg_slam_planner/` is the current RSLG-SLAM truth surface. The old
`docs/rslg_slam/` tree was migrated and deleted in task53b; it is no longer a
valid competing truth source.

## Current Formal Chain

`Frozen canonical Layer 1/2 artifacts + QueryTask -> plan_query_static / batch_plan_query_static -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

This is the current static demo chain. It consumes frozen canonical world-model
and formal artifacts; it does not rerun raw RGB-D inference or Stage-A.

## Layers

- Layer 0: Input Layer. Conceptual input is posed RGB-D, depth, sequence id,
  dataset/config paths, semantic class text, model/checkpoint provenance,
  text-feature provenance, and a Layer 0 input manifest. `tools/rslg_pipeline/build_input_manifest.py`
  records this provenance only; it does not run inference or write canonical
  artifacts. QueryTask is not Layer 0.
- Layer 1: World Model Layer. Frozen canonical 00843 evidence lives under
  `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer1_world_model/`.
  `tools/rslg_pipeline/build_world_model.py` is a legacy-backed wrapper around
  Stage-A and is not part of the current static demo path.
- Layer 2: Formal Artifact Layer. Canonical stable maps, object interfaces,
  topology, planner graph, and vertical connectors live under
  `canonical/layer2_formal_artifacts/`. These artifacts are consumed by the
  planner in frozen canonical mode. Stable maps are RSLG-SLAM formal artifacts
  derived from world-model free-space, wall, outside-boundary, gateway, and
  topology evidence; they are not external GT maps, semantic floorplans, room
  masks, runtime costmaps, or active map_server products.
- Layer 3: Navigation Interface Layer. QueryTask belongs here. `tools/rslg_pipeline/plan_query_static.py`
  and `batch_plan_query_static.py` call `planning/route_planner.py` and produce
  `RSLGRouteResult` JSON.
- Layer 4: Runtime Validation Layer. `tools/rslg_pipeline/export_route_result_runtime_inputs.py` exports PID follower, RViz marker, and z-aware overlay inputs from RouteResults.

## Current Runtime Policy

- No Nav2, AMCL, map_server, nav2_map_server, ROS lifecycle runtime dependency, planner_server, controller_server, bt_navigator, NavigateToPose, or FollowPath in the current formal runtime dependency chain.
- Current Layer 4 adapter is a lightweight PID/proportional waypoint follower through `/cmd_vel` and `/odom`.
- RViz and Gazebo are optional Layer 4 adapters only.
- z-aware vertical transition is visualization-only.

Old `00824`, `Step30P1`, `Stage1`, task39, task41, and task42 runtime material
is historical evidence only. It does not define the current formal path.

## 00843 Scene Truth

- Object target: `obj_175` / `curtain` / `room_14` / `floor_2`.
- Valid approach: `generated_ring_002`, position `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20`.
- Blocked evidence candidate: `generated_ring_037`, never selected as runtime goal.
- True transition edge: `vt_1_centerline_e001`.
- Forbidden non-transition edge: `vt_1_centerline_e003`, never used as transition edge.
- Visualization-only floor z: `floor_1 = 0.0`, `floor_2 = 1.6`.

## Static Planner Commands

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static   --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json   --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/route_results   --no-canonical-write
```

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs   --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/route_results   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/runtime_adapter_inputs   --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}'   --no-canonical-write
```

## What Not To Claim

RSLG-SLAM does not claim dense reconstruction, neural implicit SLAM, full embodied navigation benchmark, full BEV planner, Nav2 success, AMCL success, real robot deployment, physical stair climbing, Unitree Go2 control, quadruped gait control, collision-free guarantee, LLM runtime, or osmAG-Nav.
