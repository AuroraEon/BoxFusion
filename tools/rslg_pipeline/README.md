# RSLG-SLAM Pipeline

`tools/rslg_pipeline/` is the formal RSLG-SLAM command surface. RSLG-SLAM is a route-oriented light-geometry semantic world model and a semantic-topological world model interface for language-to-route navigation.

`BoxFusion` is only the historical repository path.

`docs/rslg_slam_planner/` is the current RSLG-SLAM truth surface. The old
`docs/rslg_slam/` documentation tree was migrated and deleted in task53b and
must not be restored as a competing truth source.

## Official Layers

- Layer 0: Input Layer
- Layer 1: World Model Layer
- Layer 2: Formal Artifact Layer
- Layer 3: Navigation Interface Layer
- Layer 4: Runtime Validation Layer

## Current Formal Chain

`Frozen canonical Layer 1/2 artifacts + QueryTask -> plan_query_static / batch_plan_query_static -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

This is frozen canonical mode. QueryTask is the Layer 3 request object, not the
raw Layer 0 input of the project.

## Formal Entrypoints

- `plan_query_static.py`: single QueryTask static planner. Reads an `rslg_query_task` JSON and writes an `RSLGRouteResult` JSON.
- `batch_plan_query_static.py`: runs the current six-query manifest at `configs/rslg_queryset_v0/queryset_manifest.json`.
- `export_route_result_runtime_inputs.py`: exports RouteResult-derived PID follower, RViz marker, and z-aware overlay inputs.
- `runtime/replay_pid_runtime_input.py`: replays RouteResult-derived PID inputs with a lightweight 2D waypoint follower and task-local reports.
- `viz/export_static_visualization_pack.py`: exports the static Layer 4 HTML/Markdown/SVG visualization pack from RouteResults and adapter inputs.
- `audits/validate_project_truth.py`: project truth, layer naming, object goal, transition edge, and claim-boundary guard.
- `audits/validate_query_task.py`: QueryTask schema/semantic validation.
- `audits/validate_route_result.py`: RouteResult schema/semantic validation.
- `audits/validate_planner_smoke.py`: directory-level planner smoke validation.
- `audits/validate_route_result_runtime_adapters.py`: adapter-input validation.

Layer 0-2 provenance and builders:

- `build_input_manifest.py`: Layer 0 provenance manifest writer. It records
  RGB-D/depth/pose/config/model/class/text-feature paths and claim boundaries
  only; it does not run inference or modify canonical artifacts.
- `build_world_model.py` and `run_layer1_world_model.sh`: Layer 1 canonical wrapper around the legacy Stage-A implementation. Do not treat `stage_a_demo.py` as the current planner entrypoint or as part of frozen static demo mode.
- `build_layer2_formal_artifacts.py`: current final Layer 2 canonical artifact builder.
- `build_stable_maps.py` and `build_object_interfaces.py`: legacy/candidate provenance surfaces, not the current QueryTask input path.

## Planner Ownership

- `planning/query_task.py`: QueryTask constants, loading, and semantic validation.
- `planning/artifact_loader.py`: read-only canonical artifact loading.
- `planning/target_resolver.py`: target grounding.
- `planning/room_route.py`: room/floor route planning over the route planner graph.
- `planning/floor_connector_route.py`: connector sequence and transition-edge
  handling.
- `planning/metric_path_stitcher.py`: dynamic metric path stitching.
- `planning/route_result.py`: `RSLGRouteResult` validation and schema-facing
  contract.

## Current Static Demo

Use the frozen canonical path for current handoff demos:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results \
  --no-canonical-write
```

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/runtime_adapter_inputs \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.viz.export_static_visualization_pack \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results \
  --adapter-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/runtime_adapter_inputs \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/static_visualization \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title 'RSLG-SLAM Static Visualization Pack'
```

For the fuller handoff map, see
`docs/rslg_slam_planner/HANDOFF_README.md`,
`docs/rslg_slam_planner/LAYER_ENTRYPOINTS.md`, and
`docs/rslg_slam_planner/STATIC_PIPELINE_QUICKSTART.md`.
- `planning/floor_connector_route.py`: connector resolution and transition-edge guard.
- `planning/approach_candidate_planner.py`: selected object approach policy.
- `planning/metric_path_stitcher.py`: dynamic per-floor metric-path stitching.
- `planning/route_result.py`: RouteResult construction and validation.
- `planning/route_planner.py`: generic static planner orchestration.

## Layer 4 Adapter Ownership

- `runtime/route_result_runtime_adapter.py`: lightweight PID/proportional `/cmd_vel` + `/odom` input.
- `runtime/replay_pid_runtime_input.py`: offline PID runtime replay over RouteResult-derived inputs; validates waypoint tracking, stable-map footprint checks when available, and semantic connector handoffs.
- `runtime/diagnose_pid_replay_collisions.py`: offline segment-focused collision diagnostics for existing PID replay reports and trajectories.
- `runtime/sweep_pid_runtime_replay.py`: bounded task-local PID replay parameter and waypoint-shaping sweep runner.
- `runtime/route_result_marker_adapter.py`: optional RViz marker input records.
- `runtime/route_result_z_aware_adapter.py`: visualization-only z-aware overlay records.
- `runtime/base_level_route_follower.py`: optional lightweight waypoint follower; not Nav2.
- `viz/export_static_visualization_pack.py`: static HTML/Markdown/SVG visualization pack exporter.
- `viz/live_marker_publisher.py`: optional bounded marker publisher for RouteResult-derived RViz marker inputs.
- `viz/z_aware_trajectory_markers.py`: optional z-aware marker helpers for visualization overlays.

## Static Regeneration

Layer 0 manifest example:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.build_input_manifest   --scene-id 00843-DYehNKdT76V   --dataset-name hm3d   --sequence-id 00843-DYehNKdT76V   --rgb-root /home/ws/data/00843-DYehNKdT76V   --depth-root /home/ws/data/00843-DYehNKdT76V   --pose-root /home/ws/data/00843-DYehNKdT76V   --config-path config/hm3d.yaml   --model-checkpoint-path models/cutr_rgbd.pth   --clip-checkpoint-path models/ViT-B-32/open_clip_pytorch_model.bin   --class-text-path data/panoptic_categories_nomerge.txt   --text-features-path data/class_features_small.pt   --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical   --output-json stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/layer0_input_manifest.json   --no-canonical-write
```

Layer 3 RouteResult generation:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static   --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json   --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/route_results   --no-canonical-write
```

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs   --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/route_results   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/runtime_adapter_inputs   --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}'   --no-canonical-write
```

## Current 00843 Truth

- Scene: `00843-DYehNKdT76V`.
- Object target: `obj_175` / `curtain` / `room_14` / `floor_2`.
- Selected runtime approach: `generated_ring_002` at `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20`.
- Blocked legacy evidence: `generated_ring_037`; never a runtime goal.
- True transition edge: `vt_1_centerline_e001`.
- Forbidden non-transition edge: `vt_1_centerline_e003`; never a transition edge.
- Visualization-only floor z: `floor_1 = 0.0`, `floor_2 = 1.6`.

## Runtime Policy

No Nav2, AMCL, map_server, nav2_map_server, ROS lifecycle runtime dependency, planner_server, controller_server, bt_navigator, NavigateToPose, or FollowPath is required by the current formal chain. RViz and Gazebo are optional Layer 4 adapters only and are not launched by the static exporter.

## Documentation

- `docs/rslg_slam_planner/TRUTH_SOURCE_INDEX.md`
- `docs/rslg_slam_planner/PROJECT_TRUTH.md`
- `docs/rslg_slam_planner/MAIN_CHAIN_OVERVIEW.md`
- `docs/rslg_slam_planner/LAYER0_2_PROVENANCE.md`
- `docs/rslg_slam_planner/LAYER_ENTRYPOINTS.md`
- `docs/rslg_slam_planner/FROZEN_CANONICAL_MODE.md`
- `docs/rslg_slam_planner/STAGE_A_LEGACY_BOUNDARY.md`
- `docs/rslg_slam_planner/FORMAL_ARTIFACT_INDEX.md`
- `docs/rslg_slam_planner/DEMO_PACK_README.md`
- `docs/rslg_slam_planner/VISUALIZATION_QUICKSTART.md`
- `docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`
- `docs/rslg_slam_planner/PID_REPLAY_COLLISION_MITIGATION.md`
- `docs/rslg_slam_planner/LEGACY_BOUNDARY.md`
- Existing policy/schema docs under `docs/rslg_slam_planner/`

Old `00824`, `Step30P1`, `Stage1`, task39, task41, and task42 runtime/showcase
material is historical evidence only. The current formal path is QueryTask to
`RSLGRouteResult` to RouteResult-derived Layer 4 adapter inputs.

## Claim Boundaries

RSLG-SLAM does not claim dense reconstruction, neural implicit SLAM, a full embodied navigation benchmark, a full BEV planner, Nav2 success, AMCL success, real robot deployment, physical stair climbing, Unitree Go2 control, quadruped gait control, collision-free guarantee, LLM runtime, or osmAG-Nav.
