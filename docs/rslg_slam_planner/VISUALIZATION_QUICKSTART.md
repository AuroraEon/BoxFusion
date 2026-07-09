# RSLG-SLAM Visualization Quickstart

This quickstart builds a static Layer 4 visualization pack for the current
RSLG-SLAM main chain:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs -> static visualization pack`

It does not rerun Stage-A, raw RGB-D inference, ROS, RViz, Gazebo, Nav2, AMCL,
`map_server`, `nav2_map_server`, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, `FollowPath`, or physical robot code.

## Generate RouteResults

```bash
mkdir -p stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/route_results

/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/route_results \
  --no-canonical-write
```

## Generate Adapter Inputs

```bash
mkdir -p stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/runtime_adapter_inputs

/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/route_results \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/runtime_adapter_inputs \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

This writes:

- PID follower inputs under `runtime_adapter_inputs/pid_follower_inputs/`
- RViz MarkerArray-style inputs under `runtime_adapter_inputs/rviz_marker_inputs/`
- z-aware overlay inputs under `runtime_adapter_inputs/z_aware_overlay_inputs/`

These are RouteResult-derived Layer 4 adapter inputs. They are not Nav2 runtime
commands and do not imply AMCL or map-server localization.

## Generate Static Visualization Pack

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.viz.export_static_visualization_pack \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/route_results \
  --adapter-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/runtime_adapter_inputs \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/static \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title 'RSLG-SLAM Static Visualization Pack'
```

The static pack writes:

- `visualization_pack/static/index.html`
- `visualization_pack/static/index.md`
- `visualization_pack/static/summary.json`
- `visualization_pack/static/routes/<query_id>.svg`
- `visualization_pack/static/routes/<query_id>.md`

Open `visualization_pack/static/index.html` in a browser to view the full pack.
Inspect per-query SVG and Markdown files for room/floor route sequences,
connector handoffs, object target records, selected approach candidates, blocked
candidate evidence, and z-aware floor transition annotations.

## Interpret the Static View

- `generated_ring_002` is the selected valid object approach when the query
  targets `obj_175` / `curtain` in `room_14` on `floor_2`.
- `generated_ring_037` is blocked/rejected evidence only. It must not be
  selected or treated as a runtime goal.
- `vt_1_centerline_e001` is the true transition edge for the vertical connector.
- `vt_1_centerline_e003` is the forbidden/non-transition edge. It must not be
  used as a transition edge.
- Floor z values are visualization-only: `floor_1=0.0`, `floor_2=1.6`.
- The connector handoff is a semantic/topological visualization handoff, not a
  physical stair-climbing or quadruped gait claim.

## Interpret RViz Marker Inputs

RViz marker input JSON files are passive MarkerArray-style records derived from
`RSLGRouteResult`. They can be inspected directly as JSON. Marker roles include
semantic room sequence labels, metric route lines, selected object approach
markers, rejected blocked evidence markers when present, and connector handoff
markers.

The marker input surface is a visualization adapter. It does not run Nav2,
AMCL, `map_server`, or a navigation action.

## Interpret Z-Aware Overlays

Z-aware overlay JSON files project route waypoints into visualization-only 3D
coordinates. For the current scene, `floor_1` is rendered at `z=0.0` and
`floor_2` at `z=1.6`. The z-aware transition highlights the connector handoff
through `vt_1_centerline_e001` and records `vt_1_centerline_e003` only as the
non-transition edge.

This is only a visual presentation surface. It does not claim physical stair
climbing, collision-free navigation, or real robot deployment.

## Related PID Runtime Replay

For executable validation of the RouteResult-derived PID inputs, use
`docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`. The replay writes
per-query reports plus trajectory CSV/SVG/HTML outputs. It validates same-floor
waypoint tracking and semantic connector handoff events without Nav2, AMCL,
`map_server`, Gazebo, RViz live processes, or physical robot code.

## Optional RViz Live Marker Display

If a ROS 2 environment with the current RSLG-SLAM visualization helper is
available, the existing bounded marker publisher can publish one
RouteResult-derived marker input manually:

```bash
/usr/bin/python3 tools/rslg_pipeline/viz/live_marker_publisher.py \
  --marker-input-json stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/runtime_adapter_inputs/rviz_marker_inputs/00843_cross_floor_object_curtain_room14_rviz_marker_input.json \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/visualization_pack/live_marker_manual \
  --duration-sec 30 \
  --publish-rate-hz 2 \
  --no-canonical-write
```

This optional command publishes marker records only. It is not part of static
validation, it does not launch RViz, and it does not start Nav2, AMCL,
`map_server`, or any navigation action.

## What This Visualization Does Not Claim

The visualization pack does not claim dense reconstruction, neural implicit
SLAM, a full embodied navigation benchmark, a full BEV planner, Nav2 success,
AMCL success, real robot deployment, physical stair climbing, Unitree Go2
control, quadruped gait control, collision-free navigation, LLM runtime
operation, or osmAG-Nav.
