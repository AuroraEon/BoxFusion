# Generic Static Planner Core (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

`tools/rslg_pipeline/planning/route_planner.py` is the **generic static planner
core** for the current 00843 scene. It replaces the retired seed-wrapping
surface (`wrap_current_routes_as_route_results.py`, deleted in task49d).

## Scope and honesty boundary

- This is a **single-scene generic static planner core for 00843**, not a full
  multi-scene planner and not a paper-scale evaluation.
- It uses `RSLGQueryTask` as input and `RSLGRouteResult` as output.
- Target resolution, room/floor route, connector resolution, and object approach
  selection are **derived** from canonical Layer 2/3 artifacts.
- The full cross-floor **metric path** is now **dynamically stitched** from
  per-floor occupancy A* segments over the canonical stable maps plus a semantic
  connector handoff (`metric_path_stitcher`). Its `path_length` is computed from
  that geometry, **not** copied from a canonical real route. See
  `DYNAMIC_METRIC_PATH_STITCHING.md`.
- Canonical real routes are used **only** as comparison/provenance
  (`provenance.planner_core.canonical_comparison`), never as the generated metric
  path. The retired `generic_static_with_canonical_metric_fallback` mode is no
  longer used for the current seed cross-floor queries.
- It does **not** require Nav2 / AMCL / map_server.
- It does **not** start ROS / Gazebo / RViz.
- The lightweight PID follower and RViz/Gazebo are later Layer 4 consumers, not
  planner prerequisites.

## Flow

```
QueryTask
  -> artifact_loader        (load canonical Layer 2/3 artifacts, read-only)
  -> target_resolver        (ground object/room/floor/connector)
  -> room_route             (BFS over route_planner_graph_v0_1 -> room/floor route)
  -> floor_connector_route  (resolve vt_1, enforce transition-edge guard)
  -> approach_candidate_planner (policy over 51 approach candidate records)
  -> metric_path_stitcher   (per-floor occupancy A* + connector handoff -> stitched metric path)
  -> RSLGRouteResult
```

`planner_mode` is `generic_static_dynamic_metric_stitching` when every metric
segment is planned dynamically, or
`generic_static_dynamic_metric_stitching_with_segment_fallbacks` when a small
same-floor segment degrades to a `semantic_only` segment. This is still not
formal paper evaluation.

Entry point:

```python
from tools.rslg_pipeline.planning import route_planner
result = route_planner.plan_static_query(query_task, canonical_root, no_canonical_write=True)
```

## Planner-core modules

| module | responsibility |
|---|---|
| `planning/artifact_loader.py` | read-only loaders with structured missing-artifact status |
| `planning/target_resolver.py` | ground the QueryTask target (`obj_175`/`curtain`, `room_14`, `floor_2`, `vt_1`) |
| `planning/room_route.py` | build room adjacency graph, BFS room/floor route (fallback: centralized `CROSS_FLOOR_ROOM_ROUTE`) |
| `planning/floor_connector_route.py` | resolve vertical connector, guarantee `vt_1_centerline_e001` transition / `vt_1_centerline_e003` never transition |
| `planning/approach_candidate_planner.py` | hard-constraint filter + clearance/object-distance ranking over candidate records |
| `planning/route_planner.py` | orchestrate all of the above into an `RSLGRouteResult` |

## Dynamic vs fallback (current 00843 seed queries)

- **Dynamic (current formal path)**: target resolution, room route (BFS
  `room_2 -> room_3 -> vt_1 -> room_7 -> room_13 -> room_14`), connector
  resolution, approach selection (`generated_ring_002` chosen as the only
  candidate of 51 passing the hard constraints; `generated_ring_037` recorded as
  blocked/rejected evidence), and the full cross-floor metric path, which is now
  **dynamically stitched** from per-floor occupancy A* segments
  (`generic_static_dynamic_metric_stitching`). `object_to_path` builds its
  approach metric path from the candidate's own A* waypoints.
- **Comparison/provenance only**: canonical real routes may appear only as
  comparison/provenance and never define the generated metric path. The
  `generic_static_with_canonical_metric_fallback` mode is **retired** for the
  current seed cross-floor queries.

## Guardrails

- `generated_ring_037` is never a selected runtime goal (only rejected evidence).
- `vt_1_centerline_e003` is never used as a transition edge.
- `requires_nav2 = false`, `requires_amcl = false`, all `claim_boundary` flags true.
- No physical stair climbing: vertical movement is a **semantic handoff**.

## Validation

- `audits/validate_route_result.py` — schema + semantic rules.
- `audits/validate_planner_smoke.py` — lightweight smoke checks over generated
  route results (no final paper tables).
