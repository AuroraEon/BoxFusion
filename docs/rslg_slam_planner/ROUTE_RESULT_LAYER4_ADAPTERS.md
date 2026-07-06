# RouteResult -> Layer 4 Runtime / Visualization Adapters

Project: **RSLG-SLAM** (the repository path `BoxFusion` is historical only).

## Where this sits

```
Layer 0: Input Layer
Layer 1: World Model Layer
Layer 2: Formal Artifact Layer
Layer 3: Navigation Interface Layer     <- owns RSLGRouteResult
Layer 4: Runtime Validation Layer       <- consumes RouteResult-derived adapter inputs
```

Layer 3 owns route generation and the route-interface schema. The single central
Layer 3 output is the formal artifact:

```
RSLGRouteResult   (schema_name: rslg_route_result)
```

Layer 4 owns runtime execution/validation and visualization adapters. It does not
re-plan and does not own a planner schema. It consumes **RouteResult-derived
adapter inputs** only.

## The three RouteResult-derived adapter inputs

`RSLGRouteResult` is converted, statically, into three Layer 4 adapter inputs by
`tools/rslg_pipeline/export_route_result_runtime_inputs.py`:

| Adapter input schema | Builder module | Consumer |
|---|---|---|
| `rslg_route_result_pid_runtime_input` | `runtime/route_result_runtime_adapter.py` | `runtime/base_level_route_follower.py` |
| `rslg_route_result_rviz_marker_input` | `runtime/route_result_marker_adapter.py` | `viz/live_marker_publisher.py` |
| `rslg_route_result_z_aware_overlay_input` | `runtime/route_result_z_aware_adapter.py` | `runtime/z_aware_route_projection.py` |

Shared, ROS-free extraction/guard helpers live in
`runtime/route_result_adapter_common.py`.

Important truths:

- The PID follower input is **not** a planner schema.
- The RViz marker input is **not** a planner schema.
- The z-aware overlay is **visualization-only**.
- task48h / task48i / task48j runtime/viz outputs are **evidence snapshots**, not
  formal schema definitions. They must not be treated as formal interfaces.

## PID follower input (`rslg_route_result_pid_runtime_input`)

Derived from the executable route segments (`same_floor_metric`,
`object_approach_metric`). Key fields: `runtime_policy` (lightweight PID via
`/cmd_vel` + `/odom`), `requires_nav2=false`, `requires_amcl=false`,
`compatible_with_pid_follower=true`, `floor_z_map`, `runtime_segments`,
`connector_handoffs`, `runtime_waypoints`, `selected_goal`/`endpoint`,
`selected_approach`, `rejected_runtime_candidates`, `claim_boundary`, `provenance`.

Design rule for cross-floor routes: same-floor and object-approach segments are
represented as executable waypoint segments; the vertical connector is represented
**separately** as a semantic handoff (visualization-only). No physical stair-climb
trajectory or gait command is synthesized.

Connector handoffs are derived only from explicit connector route segments
(`segment_type=connector_handoff` or equivalent). `semantic_route.connector_sequence`
may annotate a matching connector segment, but it must not create a Layer 4
handoff by itself. Therefore `object_to_path` / `local_object_approach` adapter
outputs stay on the target floor and have no connector handoff.

## RViz marker input (`rslg_route_result_rviz_marker_input`)

Plain marker records (`ns`/`id`/`type`/`points`/`pose`/`scale`/`color_rgba`/
`text`/`yaw`/`metadata`) that `live_marker_publisher.py` republishes. Includes
semantic route markers, metric path markers, per-segment markers, the connector
handoff marker, the selected approach marker, and the rejected
`generated_ring_037` marker when present, plus floor-z reference labels.

## Z-aware overlay input (`rslg_route_result_z_aware_overlay_input`)

Derived from executable `route_segments`, explicit connector-handoff
`route_segments`, and `metric_path`. Projects 2D route points into
visualization-only 3D using `floor_1 z=0.0` and `floor_2 z=1.6`. The connector
handoff is drawn as a connector line only when an explicit connector segment
exists, and is marked `semantic_handoff`, `visualization_only`, and
`not_physical_stair_climbing`.

## Runtime policy (Layer 4)

- No Nav2, no AMCL.
- No `map_server` / `nav2_map_server` active runtime dependency.
- Lightweight PID / proportional waypoint follower via `/cmd_vel` + `/odom`.
- Bounded Gazebo smoke is an optional Layer 4 adapter only.
- RViz is an optional Layer 4 visualization adapter only.
- z-aware vertical transition is a visualization overlay only.
- Floor z values are visualization-only.
- The vertical connector / stair is a semantic handoff and visualization
  connector line only. **No physical stair climbing** is claimed.

Never reintroduce ROS lifecycle, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, `FollowPath`, `map_server`, `nav2_map_server`,
or AMCL as runtime dependencies.

## Truth guards enforced by the adapters and validator

- `generated_ring_037` is never a selected/used runtime goal; it may appear only
  as a rejected / blocked / evidence marker.
- `vt_1_centerline_e003` is never used as a transition edge.
- `vt_1_centerline_e001` is the only transition edge when one is needed.
- `requires_nav2=false`, `requires_amcl=false`; no `map_server`/`nav2_map_server`.
- `floor_1 z=0.0`, `floor_2 z=1.6` are visualization-only.
- Connector handoff is `semantic_handoff` / `visualization_only` /
  `not_physical_stair_climbing`.

## Commands

Generate adapter inputs from a directory of RouteResults:

```
python -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir <task_dir>/route_results \
  --output-dir <task_dir>/runtime_adapter_inputs \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

Validate the generated adapter inputs:

```
python tools/rslg_pipeline/audits/validate_route_result_runtime_adapters.py \
  --adapter-root <task_dir>/runtime_adapter_inputs
```

The follower and marker publisher read the RouteResult-derived inputs directly:

```
python -m tools.rslg_pipeline.runtime.base_level_route_follower \
  --runtime-input-json <..._pid_runtime_input.json> --output-dir <out>

python -m tools.rslg_pipeline.viz.live_marker_publisher \
  --marker-input-json <..._rviz_marker_input.json> --output-dir <out>
```

Both also accept `--route-result-json <..._route_result.json>`, converting it
through the corresponding adapter internally. Launching the follower / publisher /
RViz / Gazebo is out of scope for the static adapter flow.

## Current formal metric path

The current formal cross-floor metric path is **dynamically stitched** from
per-floor occupancy A* segments (`generic_static_dynamic_metric_stitching`).
Canonical real routes are comparison/provenance only. The retired
`generic_static_with_canonical_metric_fallback` mode does not define the current
formal cross-floor metric path.
