# RSLG-SLAM Paper Narrative Engineering Map

This map aligns the current engineering assets with the paper-facing narrative.
It is scoped to the current frozen static 00843 handoff path.

## World Model

The world-model claim maps to the frozen canonical Layer 1/2 artifact chain:

- Layer 1 world-model evidence under `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer1_world_model/`
- Layer 2 formal artifacts under `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer2_formal_artifacts/`

The current runnable static demo consumes these artifacts. It does not rerun raw
RGB-D inference.

## Rich Semantics, Light Geometry

Engineering assets:

- Objects and object interfaces
- Rooms and floor identifiers
- Gateways and topology
- Vertical connectors
- Stable maps
- Route planner graph
- Object approach candidates and selected runtime-safe approach evidence

These assets are semantic-topological and light-geometry artifacts, not dense
reconstruction or neural implicit geometry.

## LLM-Oriented / Query-Oriented Interface

Engineering assets:

- QueryTask schema: `tools/rslg_pipeline/schemas/query_task_schema.json`
- QueryTask configs: `configs/rslg_queryset_v0/*.json`
- RouteResult schema: `tools/rslg_pipeline/schemas/route_result_schema.json`
- Planner entrypoints: `tools/rslg_pipeline/plan_query_static.py` and `tools/rslg_pipeline/batch_plan_query_static.py`

QueryTask is the Layer 3 request object that represents language-query-oriented
navigation intent. It is not Layer 0 raw input.

## Queryable

Engineering support:

- Target resolution from object, room, floor, and connector query fields
- Current six-query set covering object, room, path, connector, and blocked
  candidate rejection cases
- Grounded target fields in `RSLGRouteResult`

## Interpretable

Engineering support:

- Semantic route fields for room sequence, floor sequence, and connector sequence
- Target object, room, floor, and connector fields
- Approach candidate policy and selected approach fields
- Rejected candidate evidence, including `generated_ring_037` as blocked
  evidence only
- Transition-edge guard evidence that uses `vt_1_centerline_e001` and excludes
  `vt_1_centerline_e003`

## Routable

Engineering support:

- Room/floor/connector route planning over the planner graph
- Dynamic metric path stitching
- Route segments with path source and validation status
- Selected approach `generated_ring_002` for the 00843 curtain target

## Executable-Validation-Ready

Engineering support:

- PID follower runtime input
- RViz marker input
- z-aware overlay input
- Static validators for QueryTasks, RouteResults, planner smoke, adapter inputs,
  and project truth

Layer 4 adapter inputs are generated from RouteResults. They are ready for
runtime/visualization adapter validation without claiming Nav2, AMCL, real robot
deployment, physical stair climbing, or collision-free execution.

## Claim Boundaries

RSLG-SLAM does not claim dense reconstruction, neural implicit SLAM, full
embodied navigation benchmark coverage, full BEV planning, active Nav2/AMCL
runtime dependency, real robot deployment, collision-free guarantee, physical
stair climbing, Unitree Go2 control, quadruped gait control, LLM runtime
navigation, or osmAG-Nav.
