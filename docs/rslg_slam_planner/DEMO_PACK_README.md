# RSLG-SLAM Demo Pack README

Task52 generated a static, task-local handoff/demo pack for scene `00843-DYehNKdT76V`.

The current authoritative docs for this pack live under
`docs/rslg_slam_planner/`. The old `docs/rslg_slam/` tree was migrated and
deleted in task53b.

The demo pack is frozen canonical mode: existing canonical Layer 1/2 artifacts
plus Layer 3 QueryTasks produce fresh `RSLGRouteResult` files and
RouteResult-derived Layer 4 adapter inputs. It does not regenerate Layer 0, run
Stage-A, or run raw RGB-D inference.

## Location

`stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task52_runtime_interface_consolidation_and_demo_pack/demo_pack/`

## Contents

- `route_results/`: six fresh `RSLGRouteResult` files from the six current QueryTasks.
- `runtime_adapter_inputs/pid_follower_inputs/`: lightweight PID/proportional `/cmd_vel` + `/odom` waypoint follower inputs.
- `runtime_adapter_inputs/rviz_marker_inputs/`: optional RViz marker payloads. RViz is not launched by export.
- `runtime_adapter_inputs/z_aware_overlay_inputs/`: visualization-only z-aware overlays using `floor_1 = 0.0`, `floor_2 = 1.6`.
- `planner_smoke/`: static planner smoke validation summary.

## Regeneration Commands

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.batch_plan_query_static   --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json   --canonical-root stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results   --no-canonical-write
```

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.export_route_result_runtime_inputs   --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/route_results   --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/demo_pack/runtime_adapter_inputs   --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}'   --no-canonical-write
```

## Claim Boundary

The demo pack is static artifact generation and validation. QueryTask is Layer 3
input, not raw Layer 0. The pack does not run Gazebo, RViz, ROS nodes, Habitat,
GPU jobs, Stage-A, raw RGB-D inference, Nav2, AMCL, map_server, planner_server,
controller_server, bt_navigator, NavigateToPose, FollowPath, real robot
deployment, or physical stair climbing.

Old `00824`, `Step30P1`, `Stage1`, task39, task41, and task42 runtime-success
material is historical evidence only and is not the current formal demo path.
