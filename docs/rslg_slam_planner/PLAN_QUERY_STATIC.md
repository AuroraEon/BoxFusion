# plan_query_static / batch_plan_query_static (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

`plan_query_static.py` is the **formal static Layer 3 planner** command surface.
It reads an `rslg_query_task` JSON (the input unit), runs the generic static
planner core (`planning/route_planner.py`), and writes a schema-consistent
`RSLGRouteResult` JSON (the output unit).

Every query flows through `route_planner.plan_static_query`. Target resolution,
room/floor route (BFS over the canonical planner graph), connector resolution,
and object approach selection are performed by the planner-core modules from
canonical Layer 2/3 artifacts. The full cross-floor metric path is dynamically
stitched from per-floor occupancy A* segments (`metric_path_stitcher`); no
metric-path length is copied from a canonical real route. Canonical real routes
appear only as comparison/provenance. It is a single-scene generic static
planner core for 00843, not yet a full multi-scene planner or paper-scale
evaluation.

Formal Layer 3 flow:

```
QueryTask JSON  ->  plan_query_static  ->  route_planner  ->  RSLGRouteResult JSON  ->  validate_route_result.py
```

## Single query

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.plan_query_static \
  --query-json configs/rslg_queryset_v0/00843_cross_floor_object_curtain_room14.json \
  --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/route_results \
  --no-canonical-write
```

Arguments:

- `--query-json`: path to an `rslg_query_task` JSON.
- `--canonical-root`: read-only canonical root.
- `--output-dir`: task-local output directory (`<query_id>_route_result.json`).
- `--no-canonical-write`: fail fast if `--output-dir` resolves under canonical.

## Planner core flow (per query_type)

All query types are handled by `route_planner.plan_static_query`:

- `object_to_path`: resolve object -> select approach -> build path to endpoint
  (metric path derived from the approach A* waypoints when available) -> RouteResult.
- `object_in_room`: resolve object + room -> room route + approach -> RouteResult.
- `cross_floor_object`: resolve object -> room/floor route -> connector -> approach -> RouteResult.
- `cross_floor_room`: resolve target room -> room/floor route -> connector -> RouteResult.
- `room_gateway`: resolve start/target room -> room route -> RouteResult.
- `floor_connector`: resolve floor transition -> connector route -> RouteResult.
- `blocked_candidate_rejection` (a `cross_floor_object` query): resolve object,
  explicitly record the blocked `generated_ring_037` rejection, select a valid
  candidate -> RouteResult.

`object_to_path` is treated as an object view: room/gateway sequence success is
not required, only the object endpoint/path.

## Provenance: dynamic vs fallback fields

`provenance.planner_core` records `planner_mode` plus `dynamic_fields` and
`fallback_fields`. Dynamic fields (target resolution, room route, connector,
approach selection) are derived from canonical artifacts; the full cross-floor
`metric_path` is **dynamically stitched** from per-floor occupancy A* segments
(`generic_static_dynamic_metric_stitching`). Canonical real routes are
comparison/provenance only; the `generic_static_with_canonical_metric_fallback`
mode is retired for the current seed cross-floor queries.

## Guardrails enforced

- `identity` (`query_id`, `query_text`, `query_type`, `scene_id`) comes from the
  QueryTask, not from hardcoded constants.
- The selected runtime goal must never be a blocked legacy approach id
  (`generated_ring_037`).
- No transition edge may be a forbidden transition edge
  (`vt_1_centerline_e003`); only `vt_1_centerline_e001` is used.
- `requires_nav2 = false` and `requires_amcl = false` in every result.
- Guardrails are recorded under `provenance.query_task_guardrails`, and the
  planner-core resolution under `provenance.planner_core`.

## Batch

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/batch_route_results \
  --no-canonical-write
```

`batch_plan_query_static.py` loads each query in the manifest, validates it,
plans it, validates each generated route result, and writes `batch_summary.json`,
`batch_summary.md`, and `per_query_index.json`. Object-centric and
structure-aware queries are separated in the summary. It does **not** compute
final paper tables.

## Consumers

RViz / Gazebo / the lightweight PID `/cmd_vel` + `/odom` follower consume
`RSLGRouteResult` later through Layer 4 adapters. They are not planner
prerequisites, and no Nav2 / AMCL / map_server dependency is implied.
