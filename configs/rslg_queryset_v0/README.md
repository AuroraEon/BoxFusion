# RSLG-SLAM QuerySet v0

`configs/rslg_queryset_v0/` holds the initial RSLG-SLAM `QueryTask` seed set for
the current `00843-DYehNKdT76V` canonical routes.

RSLG-SLAM is the project name (Rich Semantic + Light Geometry). `BoxFusion` is
only a historical repository path.

## What a QueryTask is

A `QueryTask` (`schema_name: rslg_query_task`) is the **input unit** to the
static Layer 3 planner. It carries:

- the language/task query (`query_id`, `query_text`, `query_type`, `scene_id`),
- `start` and `target` descriptors,
- `expected` structured route truth and guardrails
  (`expected_room_sequence`, `expected_approach_candidate_id`,
  `forbidden_runtime_goals`, `forbidden_transition_edges`),
- `support` flags including `dualmap_native_support`, and
- `evaluation_modes` (planner_static / runtime_static / rviz_demo / gazebo_smoke).

The planner consumes a `QueryTask` and produces an `RSLGRouteResult`
(`schema_name: rslg_route_result`). QueryTask JSON is the input, RSLGRouteResult
JSON is the output.

## Supported query types

- `object_to_path`
- `object_in_room`
- `room_gateway`
- `floor_connector`
- `cross_floor_room`
- `cross_floor_object`

## Seed queries

| query_id | query_type | dualmap_native_support | file |
|---|---|---|---|
| `00843_cross_floor_object_curtain_room14` | `cross_floor_object` | `object_proxy` | `00843_cross_floor_object_curtain_room14.json` |
| `00843_cross_floor_room_room2_to_room14` | `cross_floor_room` | `native` | `00843_cross_floor_room_room2_to_room14.json` |
| `00843_object_to_path_curtain` | `object_to_path` | `object_proxy` | `00843_object_to_path_curtain.json` |
| `00843_object_in_room_curtain_room14` | `object_in_room` | `object_proxy` | `00843_object_in_room_curtain_room14.json` |
| `00843_floor_connector_floor1_to_floor2` | `floor_connector` | `unsupported` | `00843_floor_connector_floor1_to_floor2.json` |
| `00843_blocked_candidate_rejection_curtain` | `cross_floor_object` | `object_proxy` | `00843_blocked_candidate_rejection_curtain.json` |

`queryset_manifest.json` lists each query's `query_id`, `query_type`, `file`,
`dualmap_native_support`, `expected_rslg_output`, and `evaluation_modes`.

## Current scene truth

- Route: `room_2 floor_1 -> room_3 floor_1 -> vt_1 / vc_vt_1 -> room_7 floor_2 -> room_13 floor_2 -> room_14 floor_2`.
- Object target: `obj_175` (`curtain`) in `room_14` on `floor_2`.
- Selected approach: `generated_ring_002` at `[-7.020484, 1.558795]`, yaw `-2.09057`, clearance `0.20 m`.
- Blocked candidate `generated_ring_037` is evidence-only and must never be a runtime goal.
- Transition edge is `vt_1_centerline_e001`; `vt_1_centerline_e003` must never be a transition edge.

## Guardrail rules enforced by the validator

- Object-centric queries (`object_to_path`, `object_in_room`, `cross_floor_object`)
  must list `generated_ring_037` in `forbidden_runtime_goals`.
- Cross-floor queries (`floor_connector`, `cross_floor_room`, `cross_floor_object`)
  must list `vt_1_centerline_e003` in `forbidden_transition_edges`.
- `dualmap_native_support` must be one of `native`, `object_proxy`, `unsupported`,
  or `not_evaluated`.

## Validate a QueryTask

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python \
  tools/rslg_pipeline/audits/validate_query_task.py \
  --query-json configs/rslg_queryset_v0/00843_cross_floor_object_curtain_room14.json
```

## Scope note

These seeds map onto the current canonical seed routes for a single scene. The
static planner dispatches them to current-scene route wrappers; this is a
query-task-driven static wrapper/planner for the current canonical seed tasks,
not a fully general planner benchmark yet. No Nav2, AMCL, or map_server runtime
dependency is implied.
