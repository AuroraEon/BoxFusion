# RSLG-SLAM PID Runtime Replay Quickstart

This quickstart validates RouteResult-derived PID runtime inputs with a lightweight deterministic replay. It uses a 2D unicycle/proportional waypoint follower for same-floor segments and treats cross-floor connector traversal as a semantic handoff event.

It does not run Stage-A, raw RGB-D inference, Nav2, AMCL, `map_server`, `nav2_map_server`, `planner_server`, `controller_server`, `bt_navigator`, `NavigateToPose`, `FollowPath`, Gazebo, RViz live processes, or physical robot code.

## Setup

```bash
cd /home/ws/workspace/BoxFusion
PY=/home/ws/miniconda3/envs/boxfusion/bin/python
TASK_DIR=stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>
CANONICAL_ROOT=stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical
```

`BoxFusion` is only the historical repository path. The project is RSLG-SLAM.

## 1. Generate RouteResults

```bash
mkdir -p "$TASK_DIR/execution_pack/route_results"
"$PY" -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root "$CANONICAL_ROOT" \
  --output-dir "$TASK_DIR/execution_pack/route_results" \
  --no-canonical-write
```

This consumes frozen canonical Layer 1/2 artifacts and QueryTask configs. It does not rerun Stage-A or raw RGB-D inference.

## 2. Export PID Runtime Inputs

```bash
mkdir -p "$TASK_DIR/execution_pack/runtime_adapter_inputs"
"$PY" -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir "$TASK_DIR/execution_pack/route_results" \
  --output-dir "$TASK_DIR/execution_pack/runtime_adapter_inputs" \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

This writes RouteResult-derived PID follower inputs under:

`$TASK_DIR/execution_pack/runtime_adapter_inputs/pid_follower_inputs/`

The same export also writes RViz marker and z-aware overlay inputs for static inspection.

## 3. Run PID Replay

Named profiles are documented in
`docs/rslg_slam_planner/PID_RUNTIME_PROFILE_GUIDE.md`. To run the current
profile config instead of passing each PID parameter manually:

```bash
mkdir -p "$TASK_DIR/execution_pack/pid_replay"
"$PY" -m tools.rslg_pipeline.runtime.run_pid_profile_replay \
  --profile-json configs/rslg_runtime_profiles/pid_profiles_v0_1.json \
  --profile-id practical_zero_collision \
  --pid-inputs-dir "$TASK_DIR/execution_pack/runtime_adapter_inputs/pid_follower_inputs" \
  --route-results-dir "$TASK_DIR/execution_pack/route_results" \
  --z-aware-inputs-dir "$TASK_DIR/execution_pack/runtime_adapter_inputs/z_aware_overlay_inputs" \
  --output-dir "$TASK_DIR/execution_pack/pid_replay" \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title "RSLG-SLAM practical PID profile replay"
```

The direct parameter CLI remains available:

```bash
mkdir -p "$TASK_DIR/execution_pack/pid_replay"
"$PY" -m tools.rslg_pipeline.runtime.replay_pid_runtime_input \
  --pid-inputs-dir "$TASK_DIR/execution_pack/runtime_adapter_inputs/pid_follower_inputs" \
  --route-results-dir "$TASK_DIR/execution_pack/route_results" \
  --z-aware-inputs-dir "$TASK_DIR/execution_pack/runtime_adapter_inputs/z_aware_overlay_inputs" \
  --output-dir "$TASK_DIR/execution_pack/pid_replay" \
  --dt 0.1 \
  --max-linear-velocity 0.25 \
  --max-angular-velocity 0.8 \
  --linear-gain 0.8 \
  --angular-gain 1.5 \
  --waypoint-tolerance 0.12 \
  --yaw-tolerance 0.25 \
  --timeout-sec 240 \
  --stuck-window-sec 8 \
  --stuck-progress-epsilon 0.02 \
  --robot-radius 0.18 \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title "RSLG-SLAM PID Runtime Replay"
```

The replay auto-discovers canonical Layer 2 stable occupancy maps from RouteResult provenance when available. These maps are used for offline footprint checks only; they are not runtime costmaps and do not require `map_server`.

## 4. Inspect Reports

Open:

- `$TASK_DIR/execution_pack/pid_replay/index.html`
- `$TASK_DIR/execution_pack/pid_replay/index.md`
- `$TASK_DIR/execution_pack/pid_replay/summary.json`

Per-query reports are written under:

- `$TASK_DIR/execution_pack/pid_replay/reports/`

Trajectory CSV and SVG files are written under:

- `$TASK_DIR/execution_pack/pid_replay/trajectories/`

## 5. Interpret Results

`success` means all waypoints were reached, final error was within tolerance, no stuck/timeout occurred, project truth guards passed, and map-backed collision/invalid-cell checks were clean when available.

`partial` means waypoint tracking completed, but at least one guard such as map-backed collision, invalid cell, or final yaw tolerance failed.

`failed` means waypoint tracking did not complete, timed out, became stuck, or violated a hard project truth guard.

Use `collision-free in lightweight replay` only when a route reports `collision_count == 0`. This is not a global collision-free guarantee.

## Cross-Floor Connector Handoff

For cross-floor routes, the replay validates same-floor metric segments with the PID model. The vertical connector is represented as a semantic handoff event with before/after states, connector id, floor ids, visualization z values, and transition edge.

The current true transition edge is `vt_1_centerline_e001`. `vt_1_centerline_e003` remains forbidden/non-transition only.

The handoff is not physical stair climbing, 3D dynamics, stair gait, or real robot traversal.

## PID Tuning Path

PID tuning is the expected execution-validation path. If a parameter set fails, keep the failure in the reports, then run a bounded documented sweep rather than silently overfitting.

For collision-specific diagnosis and task-local mitigation sweeps, see
`docs/rslg_slam_planner/PID_REPLAY_COLLISION_MITIGATION.md`.

Suggested bounded sweep dimensions:

- `max_linear_velocity`: `0.10`, `0.15`, `0.20`, `0.25`
- `max_angular_velocity`: `0.6`, `0.8`, `1.0`
- `linear_gain`: `0.5`, `0.8`
- `angular_gain`: `1.5`, `2.0`, `2.5`
- `waypoint_tolerance`: `0.06`, `0.08`, `0.10`, `0.12`
- `yaw_tolerance`: `0.20`, `0.25`, `0.30`

## What This Does Not Claim

This replay does not claim Nav2 success, AMCL success, real robot deployment, physical stair climbing, collision-free navigation, Gazebo execution, RViz live execution, Unitree Go2 control, quadruped gait control, dense reconstruction, neural implicit SLAM, a full embodied navigation benchmark, a full BEV planner, LLM runtime operation, or osmAG-Nav.
