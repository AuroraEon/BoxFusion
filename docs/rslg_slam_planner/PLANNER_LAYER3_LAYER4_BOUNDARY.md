# Planner Layer 3 / Layer 4 Boundary (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

## Layer 3: Navigation Interface Layer

Layer 3 generates the route interface. Its central artifact is `RSLGRouteResult`
(`tools/rslg_pipeline/schemas/route_result_schema.json`,
`tools/rslg_pipeline/planning/route_result.py`).

`RSLGRouteResult` wraps:

- the semantic route (room / floor / gateway / vertical-connector sequences),
- metric route segments and the combined metric path,
- object approach candidates and the selected approach,
- light-geometry references (stable map profile, traversability, connector,
  object geometry),
- validation status and claim boundary.

Layer 3 does **not** launch runtime. The static entrypoint
`tools/rslg_pipeline/plan_query_static.py` wraps the current canonical seed
routes into `RSLGRouteResult` without touching canonical outputs.

## Layer 4: Runtime Validation Layer

Layer 4 consumes route results. The current adapter is the lightweight PID /
proportional `/cmd_vel` + `/odom` waypoint follower
(`tools/rslg_pipeline/runtime/base_level_route_follower.py`).

- RViz and Gazebo are Layer 4 **adapters**, not planner prerequisites.
- Nav2 and AMCL are **not** current dependencies.
- Bounded Gazebo smoke is retained as a Layer 4 adapter only.
- The z-aware vertical transition is a visualization overlay only.

## Boundary rules

- Layer 3 owns route generation and route-interface schema.
- Layer 4 owns runtime execution/validation and visualization adapters.
- A route result must be consumable by the PID follower with
  `requires_nav2 = false` and `requires_amcl = false`.
- The old `map_server` / Nav2 lifecycle runtime paths were removed
  (`LEGACY_RUNTIME_REMOVED.md`).
- `task48h/i/j` outputs are evidence snapshots, not schema definitions.

## Table alignment

Engineering serves four tables. Their evaluators will consume `RSLGRouteResult`:

- Table 1: World Model Capability Comparison.
- Table 2: Common Object-to-Path Navigation.
- Table 3: Structured Route Artifact Generation.
- Table 4: Light-Geometry Executable Interface.
