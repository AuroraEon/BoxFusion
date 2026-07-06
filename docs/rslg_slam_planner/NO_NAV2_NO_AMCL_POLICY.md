# NO Nav2 / NO AMCL Policy (RSLG-SLAM)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

## Current runtime policy

- Nav2 is **not** a current formal dependency.
- AMCL is **not** a current formal dependency.
- `map_server` / `nav2_map_server` is **not** a current formal runtime dependency.
- The current Layer 4 runtime adapter is the lightweight PID / proportional
  `/cmd_vel` + `/odom` waypoint follower
  (`tools/rslg_pipeline/runtime/base_level_route_follower.py`).
- Bounded Gazebo smoke is retained only as a Layer 4 adapter, not as a planner
  prerequisite.
- The z-aware vertical-transition layer is a **visualization overlay only**.

## Route interface

The `RSLGRouteResult` metric path is dynamically stitched from per-floor
occupancy A* segments over the canonical stable maps plus a semantic connector
handoff. This is offline light-geometry planning: it needs no ROS, Gazebo, RViz,
Nav2, AMCL, or `map_server` — only the canonical Layer 2/3 artifacts. The
connector handoff is a semantic vertical transition, never physical stair
climbing. See `DYNAMIC_METRIC_PATH_STITCHING.md`.

Every `RSLGRouteResult` sets:

- `runtime_interface.requires_nav2 = false`
- `runtime_interface.requires_amcl = false`
- `runtime_interface.compatible_with_pid_follower = true`
- `claim_boundary.no_nav2_dependency = true`
- `claim_boundary.no_amcl_dependency = true`

The `RSLGRouteResult` validator (`tools/rslg_pipeline/audits/validate_route_result.py`)
rejects any route result that requires Nav2 or AMCL.

## Explicit non-claims

The pipeline does not claim: real robot execution, physical stair climbing,
Unitree Go2 control, quadruped gait control, a collision-free guarantee, Nav2
success, AMCL success, or LLM runtime navigation.

## Removed legacy

The historical `map_server` / Nav2 lifecycle executor modules
(`runtime/cross_floor_runtime.py`, `runtime/object_runtime.py`) were removed in
task49b. See `LEGACY_RUNTIME_REMOVED.md`.

In task49c, the misleadingly named Layer 4 `map_server_inputs` packaging output
was renamed to `pid_follower_map_inputs` (schema
`rslg_layer4_pid_follower_map_inputs`, file `pid_follower_map_inputs_v0_1.json`)
to make clear that the lightweight PID `/cmd_vel` + `/odom` follower reads the
occupancy grid directly and that `map_server` / `nav2_map_server` is not an
active runtime dependency. The preview-only route contract / route plan wrappers
were also retired; see `PREVIEW_WRAPPERS_RETIRED.md`.
