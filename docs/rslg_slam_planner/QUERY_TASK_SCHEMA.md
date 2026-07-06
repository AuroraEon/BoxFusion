# RSLGQueryTask Schema (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

Schema file: `tools/rslg_pipeline/schemas/query_task_schema.json`
Constructor / validator: `tools/rslg_pipeline/planning/query_task.py`
CLI validator: `tools/rslg_pipeline/audits/validate_query_task.py`
Seed queryset: `configs/rslg_queryset_v0/`

`RSLGQueryTask` (`schema_name = rslg_query_task`) is the **input unit** for the
static Layer 3 planner. A QueryTask is consumed by `plan_query_static.py` /
`batch_plan_query_static.py` to produce an `RSLGRouteResult`.

Formal Layer 3 flow:

```
QueryTask JSON  ->  plan_query_static  ->  RSLGRouteResult JSON
```

## Sections

- top level: `schema_name`, `schema_version`, `project_name`, `query_id`,
  `query_text`, `query_type`, `scene_id`.
- `start`: `start_pose`, `start_room_id`, `start_floor_id`.
- `target`: `target_type`, `target_object_id`, `target_object_category`,
  `target_room_id`, `target_floor_id`, `target_gateway_id`.
- `expected`: `expected_object_id`, `expected_room_id`, `expected_floor_id`,
  `expected_room_sequence`, `expected_gateway_sequence`,
  `expected_vertical_connector`, `expected_approach_candidate_id`,
  `forbidden_runtime_goals`, `forbidden_transition_edges`.
- `support`: `requires_object`, `requires_room`, `requires_gateway`,
  `requires_floor`, `requires_route_contract`, `dualmap_native_support`.
- `evaluation_modes`: `planner_static`, `runtime_static`, `rviz_demo`,
  `gazebo_smoke`.

## Controlled vocabularies

- `query_type`: `object_to_path`, `object_in_room`, `room_gateway`,
  `floor_connector`, `cross_floor_room`, `cross_floor_object`.
- `target_type`: `object`, `room`, `gateway`, `floor_connector`.
- `dualmap_native_support`: `native`, `object_proxy`, `unsupported`,
  `not_evaluated`.

## Required fields

`query_id`, `query_text`, `query_type`, and `scene_id` are required.

## Semantic validation rules

- Object-centric queries (`object_to_path`, `object_in_room`,
  `cross_floor_object`) must include every blocked legacy approach id
  (`generated_ring_037`) in `expected.forbidden_runtime_goals`, and
  `expected_approach_candidate_id` must not be a blocked legacy id.
- Cross-floor queries (`floor_connector`, `cross_floor_room`,
  `cross_floor_object`) must include the non-transition edge
  (`vt_1_centerline_e003`) in `expected.forbidden_transition_edges`.
- `support.dualmap_native_support` must be one of the allowed values.

## Validate

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python \
  tools/rslg_pipeline/audits/validate_query_task.py \
  --query-json configs/rslg_queryset_v0/00843_cross_floor_object_curtain_room14.json
```
