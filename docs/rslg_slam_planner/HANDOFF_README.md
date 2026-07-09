# RSLG-SLAM Main-Chain Handoff

`docs/rslg_slam_planner/` is the current authoritative RSLG-SLAM truth source.
The old `docs/rslg_slam/` tree was migrated and deleted in task53b and must not
be recreated.

RSLG-SLAM is a route-oriented light-geometry semantic world model. It can also
be described as a semantic-topological world model interface for
language-to-route navigation. Its current engineering surface is designed to
produce queryable, interpretable, routable, and executable-validation-ready
semantic-topological artifacts.

`BoxFusion` is only the historical repository path.

## What It Is Not

RSLG-SLAM is not dense reconstruction, neural implicit SLAM, a full embodied
navigation benchmark, a full BEV planner, Nav2 success, AMCL success, a real
robot deployment, physical stair climbing, Unitree Go2 control, quadruped gait
control, a collision-free guarantee, an LLM runtime system, or osmAG-Nav.

## Official Layers

| Layer | Name | Current role |
| --- | --- | --- |
| Layer 0 | Input Layer | Raw/provenance input: RGB-D frames, depth, provided poses, scene id, sequence id, dataset/config paths, model/checkpoint provenance, CLIP checkpoint provenance, semantic class text, text-feature provenance, and a Layer 0 manifest. |
| Layer 1 | World Model Layer | Frozen canonical world-model evidence for 00843. Current rebuild remains legacy Stage-A-backed. |
| Layer 2 | Formal Artifact Layer | Frozen canonical stable maps, topology, object interfaces, object approach candidates, vertical connectors, and planner graph. |
| Layer 3 | Navigation Interface Layer | QueryTask input and `RSLGRouteResult` output. |
| Layer 4 | Runtime Validation Layer | RouteResult-derived PID follower, RViz marker, and z-aware overlay adapter inputs. |

QueryTask is Layer 3 input, not Layer 0. Layer 0 is the raw/provenance input
surface.

## Conceptual Chain

`Layer 0 raw/provenance input -> Layer 1 world model -> Layer 2 formal artifacts -> QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

The full raw RGB-D to Layer 1 rerun remains legacy Stage-A-backed until a clean
current Layer 1 builder exists. `stage_a_demo.py` is legacy provenance/exporter
material, not the current formal project entrypoint.

## Current Runnable Static Chain

The current static handoff demo path is frozen canonical mode:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

This mode consumes canonical artifacts under:

`stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`

It writes task-local outputs under:

`stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/`

It does not rerun Stage-A, raw RGB-D inference, Gazebo, RViz, ROS nodes,
Habitat, GPU jobs, Nav2, AMCL, `map_server`, ROS lifecycle nodes,
`planner_server`, `controller_server`, `bt_navigator`, `NavigateToPose`, or
`FollowPath`.

## Main Scene Truth

- Scene id: `00843-DYehNKdT76V`
- Object target: `obj_175` / `curtain` / `room_14` / `floor_2`
- Valid approach: `generated_ring_002`, position `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20`
- Blocked evidence candidate: `generated_ring_037`; it must never be selected as a runtime goal
- True transition edge: `vt_1_centerline_e001`
- Forbidden non-transition edge: `vt_1_centerline_e003`; it must never be used as the transition edge
- Visualization-only floor z: `floor_1 = 0.0`, `floor_2 = 1.6`

## Current Commands

Layer 3 batch static planning:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results \
  --no-canonical-write
```

Layer 4 adapter export:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/runtime_adapter_inputs \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

## Validation Evidence

The current validation surface is:

- `tools/rslg_pipeline/audits/validate_project_truth.py`
- `tools/rslg_pipeline/audits/validate_query_task.py`
- `tools/rslg_pipeline/audits/validate_route_result.py`
- `tools/rslg_pipeline/audits/validate_planner_smoke.py`
- `tools/rslg_pipeline/audits/validate_route_result_runtime_adapters.py`
- `tools/rslg_pipeline/runtime/replay_pid_runtime_input.py`
- `python -m compileall tools/rslg_pipeline`

Task-local demo packs should also run a `build_input_manifest.py` smoke only if
the helper exists. That smoke records Layer 0 provenance and does not run
inference or modify canonical artifacts.

## Legacy Boundary

Old `00824`, `Step30P1`, `Stage1`, task39, task41, task42, task48
Go2/Gazebo/RViz showcase, Nav2, AMCL, and map-server material is historical
evidence only. It is not the current formal path.

The current formal path is:

`QueryTask -> RSLGRouteResult -> RouteResult-derived adapter inputs`

For lightweight executable validation of the PID adapter inputs, see
`docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`. This replay validates
same-floor waypoint tracking and connector handoff events without Nav2, AMCL,
`map_server`, Gazebo, RViz live processes, or physical robot execution.

For stable-map footprint collision diagnosis and bounded task-local replay
mitigation sweeps, see
`docs/rslg_slam_planner/PID_REPLAY_COLLISION_MITIGATION.md`.
