# Dynamic Metric Path Stitching (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

Module: `tools/rslg_pipeline/planning/metric_path_stitcher.py`
Consumed by: `tools/rslg_pipeline/planning/route_planner.py`

## What it does

The static planner now **dynamically stitches** the cross-floor metric path for
the current 00843 seed queries instead of copying a canonical real-route length.
It converts a semantic room/floor/connector route into a stitched metric path:

```
QueryTask
  -> dynamic target resolution
  -> dynamic room/floor/connector route
  -> per-floor occupancy A* segments        (same_floor_metric)
  -> connector handoff segment              (connector_handoff)
  -> object approach segment                (object_approach_metric)
  -> stitched metric path
  -> RSLGRouteResult
```

## Geometry sources (canonical Layer 2, read-only)

- Room representative points: `route_planner_graph_v0_1.json` node
  `planner_anchor_xy` (room polygon centroid).
- Per-floor stable occupancy maps:
  `stable_maps/floor_{1,2}/floor_{1,2}_stable_occupancy_map_v0_1.yaml`.
- Connector entry/exit: `vertical_connectors_v0_1.json` `vt_1`
  `layer1_endpoint_geometry.source_position_xy` / `target_position_xy`.
- Object approach: the selected candidate's reachable A* waypoints.

A* mirrors the canonical conservative real-route planner: `inflation_radius_m=0.2`,
`waypoint_spacing_m=0.2`, 8-connectivity, `maximum_anchor_snap_m=1.0`
(`occupancy_planner.OccupancyPlanner`).

## Segment types

| segment_type | meaning |
|---|---|
| `same_floor_metric` | A* on a floor stable occupancy map between two anchors |
| `connector_handoff` | semantic vertical transition (edge `vt_1_centerline_e001`) |
| `object_approach_metric` | final local object approach (`dynamic_local_approach` when the candidate's own A* waypoints are used) |
| `semantic_only` | a segment that could not be dynamically planned; explicit `fallback_reason` |

The stitched `metric_path.path` concatenates all segment waypoints (adjacent
duplicates removed). `metric_path.path_length` is the sum of segment geometry
lengths. `metric_path.path_source` is `dynamic_stitched` (full routes) or
`dynamic_local_approach` (`object_to_path`).

## Per-query-type scope

| query_type | metric_path_scope | structure_sequence_required |
|---|---|---|
| object_to_path | local_object_approach | false |
| object_in_room | full_route_to_object_approach | true |
| cross_floor_object | full_cross_floor_route_to_object_approach | true |
| cross_floor_object (blocked-rejection) | full_route_to_valid_approach_with_blocked_candidate_rejection | true |
| cross_floor_room | full_cross_floor_room_route | true |
| floor_connector | floor_transition_handoff_route | true |

## planner_mode

- `generic_static_dynamic_metric_stitching` — every metric segment planned
  dynamically.
- `generic_static_dynamic_metric_stitching_with_segment_fallbacks` — a small
  same-floor segment degraded to `semantic_only`.
- `generic_static_with_canonical_metric_fallback` — **retired** for the current
  seed cross-floor queries.

## Canonical comparison (provenance only)

`provenance.planner_core.canonical_comparison` records the canonical real-route
length and the delta against the stitched length, marked
`role = comparison_provenance_only`. It is **never** the generated metric path.

## Honesty boundary

- Connector handoff is a **semantic** vertical transition, never physical stair
  climbing, gait, or footstep planning.
- No Nav2 / AMCL / map_server; no ROS / Gazebo / RViz required for
  `plan_query_static`.
- Single-scene generic static planner core for 00843; **not** formal paper
  evaluation and not a multi-scene generalization claim.
