# RSLGRouteResult Schema (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

Schema file: `tools/rslg_pipeline/schemas/route_result_schema.json`
Constructor / validator: `tools/rslg_pipeline/planning/route_result.py`
CLI validator: `tools/rslg_pipeline/audits/validate_route_result.py`

`RSLGRouteResult` (`schema_name = rslg_route_result`) is the single Layer 3
route-interface artifact and the **output unit** of the static planner. Its
metric path is dynamically stitched from per-floor occupancy A* segments; it does
not wrap or copy a canonical real route.

The **input unit** is an `RSLGQueryTask` (`schema_name = rslg_query_task`, see
`QUERY_TASK_SCHEMA.md`). The formal Layer 3 flow is:

```
QueryTask JSON  ->  plan_query_static  ->  RSLGRouteResult JSON
```

## Sections

- `identity`: `query_id`, `query_text`, `query_type`, `scene_id`,
  `planner_version`, `created_by`.
- `start`: `start_pose`, `start_room_id`, `start_floor_id`.
- `target`: `target_type`, `target_object_id`, `target_object_category`,
  `target_room_id`, `target_floor_id`, `target_gateway_id`.
- `semantic_route`: `room_sequence`, `floor_sequence`, `gateway_sequence`,
  `vertical_connector_id`, `connector_sequence`.
- `route_segments`: list of `{segment_id, segment_type, floor_id, from_room_id,
  to_room_id, connector_id, transition_edge, start, end, waypoints,
  segment_length, path_source, path_found, validation_status}`. `segment_type`
  is one of `same_floor_metric`, `connector_handoff`, `object_approach_metric`,
  `semantic_only` (legacy `same_floor` / `vertical_transition` / `object_approach`
  remain accepted).
- `approach`: `approach_policy`, `approach_candidates`, `selected_approach`,
  `rejected_candidates`, `candidate_trial_count`.
- `metric_path`: `path` (stitched `[{x, y}, ...]` polyline), `path_length`
  (computed from stitched A* geometry, never copied from a canonical route),
  `path_found`, `endpoint`, `path_source` (`dynamic_stitched` /
  `dynamic_local_approach`), `metric_path_scope`.
- `route_generation` (task49e): `metric_path_scope`, `route_generation_scope`,
  `structure_sequence_required`, `planner_mode`, `metric_path_source`,
  `stitching_report`.
- `validation`: `wall_crossing_count`, `invalid_cell_ratio`, `endpoint_free`,
  `endpoint_connected`, `endpoint_clearance`, `endpoint_to_object_distance`,
  `local_path_valid`, `route_feasible`, `route_validity`, `validation_status`,
  `failure_reason`.
- `light_geometry`: `stable_map_profile`, `traversability_map_ref`,
  `room_mask_ref`, `gateway_geometry_ref`, `vertical_connector_ref`,
  `object_geometry_ref`.
- `runtime_interface`: `compatible_with_pid_follower`, `requires_nav2`,
  `requires_amcl`, `runtime_input_ref`.
- `claim_boundary`: `no_nav2_dependency`, `no_amcl_dependency`,
  `no_real_robot_claim`, `no_collision_free_guarantee`,
  `no_physical_stair_climbing_claim`, `no_go2_control_claim`.
- `provenance`: `source_artifacts`, `source_hashes_if_available`,
  `canonical_root`, `notes`.

## Controlled vocabularies

- `query_type`: `object_to_path`, `object_in_room`, `room_gateway`,
  `floor_connector`, `cross_floor_room`, `cross_floor_object`.
- `segment_type`: `same_floor`, `vertical_transition`, `object_approach`.
- `target_type`: `object`, `room`, `gateway`, `floor_connector`.
- `validation_status`: `passed`, `failed`, `partial`, `not_validated`,
  `blocked`.

## Semantic validation rules

- `runtime_interface.requires_nav2` must be false.
- `runtime_interface.requires_amcl` must be false.
- All `claim_boundary` flags must be true.
- The blocked legacy approach candidate must never be the selected runtime
  goal; if it appears at all it must be a rejected / blocked / evidence
  candidate.
- If a cross-floor route uses a connector, only the true transition edge may be
  used; the non-transition edge must never be used as a transition edge.
- `validation_status` and each segment `validation_status` must be in the
  controlled set.

## Producer

`tools/rslg_pipeline/planning/route_planner.py` (`plan_static_query`) produces
`RSLGRouteResult` structures from an `rslg_query_task` and the canonical Layer 2/3
artifacts. `plan_query_static.py` runs the planner core, records guardrails under
`provenance.query_task_guardrails` and planner-core resolution under
`provenance.planner_core`, and writes `<query_id>_route_result.json`.

The retired `planning/wrap_current_routes_as_route_results.py` seed wrapper
(task49d) has been superseded by the planner core. Canonical artifacts are read
only and never modified.
