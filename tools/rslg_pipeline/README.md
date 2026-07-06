# RSLG-SLAM Pipeline

`tools/rslg_pipeline/` is the formal RSLG-SLAM command surface.

RSLG-SLAM is Rich Semantic + Light Geometry: a posed RGB-D semantic-topological world-modeling backend that produces structured semantic-topological navigation interfaces and formal route/validation artifacts.

## Official Layers

- Layer 0: Input Layer
- Layer 1: World Model Layer
- Layer 2: Formal Artifact Layer
- Layer 3: Navigation Interface Layer
- Layer 4: Runtime Validation Layer

## Formal Entrypoints

- `build_world_model.py` and `run_layer1_world_model.sh`: Layer 1 world-model entrypoints.
- `build_layer2_formal_artifacts.py` and `run_layer2_formal_artifacts.sh`: Layer 2 stable maps, vertical connector, object interface, and formal artifact generation.
- `build_layer3_navigation_interface.py` and `run_layer3_navigation_interface.sh`: Layer 3 navigation-interface generation.
- `plan_query_static.py`: formal static Layer 3 planner. It reads an `rslg_query_task` JSON (`schemas/query_task_schema.json`) and runs the generic static planner core (`planning/route_planner.py`) to produce an `RSLGRouteResult` (`schemas/route_result_schema.json`). Target resolution, room/floor route, connector resolution, and approach selection are performed from canonical Layer 2/3 artifacts; only the full metric-path length may be canonical-fallback derived. See `docs/rslg_slam_planner/GENERIC_STATIC_PLANNER_CORE.md`.
- `batch_plan_query_static.py`: initial batch surface that runs the planner core over a queryset manifest (`configs/rslg_queryset_v0/queryset_manifest.json`) and writes batch summaries. It does not compute final paper tables.
- `export_route_result_runtime_inputs.py`: Layer 4 adapter-input exporter for RouteResult-derived PID runtime, RViz marker, and z-aware visualization payloads.
- `audits/validate_project_truth.py`: guardrail check for project truth, object goal truth, transition-edge truth, layer naming, and claim boundaries.
- `audits/validate_query_task.py`: schema and semantic validator for `RSLGQueryTask` specs.
- `audits/validate_route_result.py`: schema and semantic validator for `RSLGRouteResult` artifacts.
- `audits/validate_planner_smoke.py`: lightweight planner smoke validation over a directory of generated route results (no final paper tables).

The formal current surface is: QueryTask JSON -> `plan_query_static.py` / `batch_plan_query_static.py` -> `planning/route_planner.py` -> `RSLGRouteResult` JSON -> RouteResult-derived Layer 4 adapter inputs. The preview-only `build_route_contracts.py`, `build_route_plans.py`, `planning/route_contracts.py`, and `planning/route_plans.py` dry-run wrappers were retired in task49c; the `planning/wrap_current_routes_as_route_results.py` seed wrapper was retired in task49d. See `docs/rslg_slam_planner/PREVIEW_WRAPPERS_RETIRED.md`.

## Safe Static Regeneration

Formal Layer 3 and RouteResult-derived Layer 4 adapter generation support task-local safe mode:

```bash
--scene-id 00843-DYehNKdT76V \
--canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
--output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/<safe-output> \
--dry-run \
--no-canonical-write
```

`--canonical-root` is read-only input context. `--output-dir` redirects generated validation or replay outputs. `--dry-run` keeps execution offline/static, and `--no-canonical-write` fails fast if an output path would land under canonical. These static modes do not launch live ROS, Gazebo, RViz, Nav2, AMCL, map_server, route executors, rclpy nodes, Habitat rendering, or world-model inference.

## Ownership Boundaries

Layer 3 planner ownership lives under `planning/`.

- `planning/occupancy_planner.py` owns occupancy-grid A* and polyline validation.
- `planning/object_approach.py` owns the current object approach goal ownership/validation helpers: `generated_ring_002` for `obj_175`.
- `planning/query_task.py` owns the `RSLGQueryTask` schema constants, construction, loading, and semantic validation.
- `planning/route_result.py` owns the `RSLGRouteResult` schema constants, construction, and semantic validation.
- `planning/artifact_loader.py` loads canonical Layer 2/3 artifacts (structured missing-artifact status; read-only).
- `planning/target_resolver.py` resolves a QueryTask target into a grounded object/room/floor/connector target.
- `planning/room_route.py` builds a room adjacency graph from `route_planner_graph_v0_1.json` and plans a BFS room/floor route.
- `planning/floor_connector_route.py` resolves the vertical connector and enforces the transition-edge guard.
- `planning/approach_candidate_planner.py` selects the object approach via an explicit policy over canonical candidate records.
- `planning/route_planner.py` is the generic static planner core: `plan_static_query(query_task, canonical_root)` orchestrates the above into an `RSLGRouteResult`. See `docs/rslg_slam_planner/GENERIC_STATIC_PLANNER_CORE.md`.
- The preview-only `planning/route_contracts.py` and `planning/route_plans.py` dry-run generators (and their `build_route_contracts.py` / `build_route_plans.py` compatibility wrappers) were removed in task49c. The `planning/wrap_current_routes_as_route_results.py` seed wrapper was removed in task49d after the planner core superseded it. See `docs/rslg_slam_planner/PREVIEW_WRAPPERS_RETIRED.md`.

Layer 4 runtime validation lives under `runtime/`.

- `export_route_result_runtime_inputs.py` exports all current RouteResult-derived Layer 4 adapter inputs without launching ROS, Gazebo, RViz, Nav2, or AMCL.
- `runtime/route_result_runtime_adapter.py` builds the lightweight PID/proportional `/cmd_vel` + `/odom` waypoint-follower input.
- `runtime/route_result_marker_adapter.py` builds RViz marker input records; RViz itself is optional and not launched by the exporter.
- `runtime/route_result_z_aware_adapter.py` builds visualization-only z-aware vertical transition overlays.
- `runtime/base_level_route_follower.py` is the current lightweight PID/proportional `/cmd_vel` + `/odom` waypoint follower. It consumes RouteResult-derived PID runtime input or an `RSLGRouteResult` converted through the adapter.
- The legacy `runtime/cross_floor_runtime.py` and `runtime/object_runtime.py` Nav2/`map_server` executor modules were removed in task49b. See `docs/rslg_slam_planner/LEGACY_RUNTIME_REMOVED.md`.
- The pre-RouteResult `export_runtime_inputs.py`, `run_layer4_runtime_validation_static.sh`, and `run_rslg_pipeline_static.sh` wrappers were retired in task50 because the formal Layer 4 surface is now RouteResult-derived adapter input generation and validation.

Replay and visualization live under `viz/`.

- `viz/live_marker_publisher.py` publishes RouteResult-derived marker payloads when a ROS environment is intentionally used as an optional Layer 4 visualization adapter.
- `viz/z_aware_trajectory_markers.py` contains visualization-only marker helpers for z-aware overlays.

Static audits live under `audits/`.

## Current Truth

- Main scene: `00843-DYehNKdT76V`.
- Route: `room_2 floor_1 -> room_3 floor_1 -> vt_1 / vc_vt_1 -> room_7 floor_2 -> room_13 floor_2 -> room_14 floor_2`.
- True transition edge: `vt_1_centerline_e001`.
- Non-transition edge: `vt_1_centerline_e003`.
- Object query: `curtain in room_14 on floor_2`.
- Object id: `obj_175`.
- Current object approach goal: `generated_ring_002` at `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20 m`.
- `generated_ring_037` is blocked legacy evidence only and is not a runtime goal.

## Historical Directories

Historical directories such as `tools/object_nav/`, `tools/stage1_nav/`, `tools/stage1_runtime/`, `tools/stage1_step30p1/`, and `tools/vertical_connectors/` are not formal RSLG-SLAM entrypoints. Essential planner, schema, runtime validation, and replay logic has been migrated or wrapped under `tools/rslg_pipeline/`.

## Claim Boundaries

RSLG-SLAM does not claim dense reconstruction, neural implicit SLAM, a full embodied navigation benchmark, a full BEV planner, AMCL success, an LLM runtime system, real robot deployment, a collision-free guarantee, or osmAG-Nav.
