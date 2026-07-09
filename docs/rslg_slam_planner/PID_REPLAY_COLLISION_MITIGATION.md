# RSLG-SLAM PID Replay Collision Mitigation

This note covers task-local diagnosis and mitigation of stable-map footprint
samples found by lightweight PID replay. It stays inside the current formal
static chain:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs -> lightweight PID replay`

It does not run Stage-A, raw RGB-D inference, Nav2, AMCL, `map_server`,
`nav2_map_server`, `planner_server`, `controller_server`, `bt_navigator`,
`NavigateToPose`, `FollowPath`, Gazebo, RViz live processes, or physical robot
code.

## Task56 Baseline

Task56 replayed six RouteResult-derived PID inputs:

- success: `2`
- partial: `4`
- failed: `0`
- average final error: about `0.098865 m`
- average waypoint reached ratio: `1.0`
- invalid cells: `0`
- stuck/timeout: none
- collision check mode: `stable_map_footprint`
- `generated_ring_037` selected count: `0`
- `vt_1_centerline_e003` transition count: `0`

The four partial routes all reached their waypoints. They were partial because
stable-map footprint checking reported eight collision samples each on:

`seg_05_same_floor_metric_room_13_to_room_14`

That distinction matters: `partial` does not mean unreachable. It means the
waypoint follower completed, but at least one replay guard failed.

## Diagnose A Segment

Use the diagnostic tool against an existing PID replay pack:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/runtime/diagnose_pid_replay_collisions.py \
  --baseline-replay-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/pid_replay \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/route_results \
  --pid-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/runtime_adapter_inputs/pid_follower_inputs \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/diagnostic_pack/collision_diagnostics \
  --target-segment-id seg_05_same_floor_metric_room_13_to_room_14 \
  --robot-radius-list 0.10,0.12,0.15,0.18 \
  --floor-z-map '{"floor_1":0.0,"floor_2":1.6}'
```

Inspect:

- `summary.json`
- `seg_05_collision_samples.csv`
- `seg_05_collision_samples.json`
- `seg_05_diagnostic.md`
- `seg_05_collision_overlay.svg`

The diagnostic checks the planned centerline footprint and the executed PID
trajectory footprint when the stable-map package is available. If the checker
is not available, the report says so explicitly and does not fake map-backed
results.

## Interpret Root Cause

Use these rules:

- If the planned centerline footprint collides at `robot_radius=0.18`, the
  issue is likely route clearance or stable-map boundary sensitivity.
- If the centerline is clear but PID samples collide near turns, the issue is
  likely PID corner-cutting or tracking tolerance sensitivity.
- If smaller radii pass but `0.18` fails, the issue is footprint-clearance
  sensitivity.
- If reasonable PID tuning and task-local waypoint shaping still collide,
  recommend a future local clearance-aware replan rather than claiming success.

For task56b, the planned centerline was clear at `0.18 m`; the PID trajectory
collided at `0.18 m`; the affected samples were near `x=-7.77..-7.69`,
`y=4.96..4.97` on `floor_2`; and the maximum collision-sample distance to the
planned centerline was about `0.042182 m`. The classification is PID
corner-cutting / tracking tolerance sensitivity, with radius sensitivity also
visible.

## Run A Bounded Sweep

Use the sweep wrapper to preserve one replay pack per candidate:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/runtime/sweep_pid_runtime_replay.py \
  --pid-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/runtime_adapter_inputs/pid_follower_inputs \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/route_results \
  --z-aware-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task56_pid_runtime_executable_replay_validation/execution_pack/runtime_adapter_inputs/z_aware_overlay_inputs \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/diagnostic_pack/pid_parameter_sweep \
  --floor-z-map '{"floor_1":0.0,"floor_2":1.6}' \
  --target-segment-id seg_05_same_floor_metric_room_13_to_room_14 \
  --max-candidates 40 \
  --title 'RSLG-SLAM PID replay mitigation sweep'
```

The sweep covers bounded combinations of:

- `max_linear_velocity`
- `max_angular_velocity`
- `linear_gain`
- `angular_gain`
- `waypoint_tolerance`
- `yaw_tolerance`

It also includes task-local waypoint densification at `0.10 m` and `0.05 m`,
plus corner-guard densification variants. These transformed PID inputs are
written under the sweep output directory only. They do not modify canonical
RouteResults, QueryTasks, or planner behavior.

## Choose A Candidate

Prefer a candidate that satisfies all of the following:

- `generated_ring_037_selected_count == 0`
- `vt_1_centerline_e003_transition_count == 0`
- `failure_count == 0`
- no stuck/timeout
- average waypoint reached ratio `1.0`
- no invalid-cell increase
- stable-map footprint collision count minimized, ideally `0`
- average final error `<= 0.15 m`

For task56b, multiple candidates removed all collision samples. The best-ranked
candidate was parameter-only:

```json
{
  "dt": 0.1,
  "max_linear_velocity": 0.1,
  "max_angular_velocity": 0.8,
  "linear_gain": 0.8,
  "angular_gain": 2.5,
  "waypoint_tolerance": 0.06,
  "yaw_tolerance": 0.2,
  "timeout_sec": 240.0,
  "stuck_window_sec": 8.0,
  "stuck_progress_epsilon": 0.02,
  "robot_radius": 0.18
}
```

It produced six successes, zero partials, zero failures, zero collision samples,
zero invalid cells, average final error `0.043215 m`, waypoint reach ratio
`1.0`, and preserved both project guards.

Task56c promotes that conservative candidate and the practical-speed
`candidate_001_lower_v_stricter_tol` into named lightweight replay profiles in
`configs/rslg_runtime_profiles/pid_profiles_v0_1.json`. See
`docs/rslg_slam_planner/PID_RUNTIME_PROFILE_GUIDE.md` for profile selection and
regression commands.

## Remaining Claims

These reports are lightweight executable replay evidence only. They do not
claim Nav2 success, AMCL success, real robot deployment, physical stair
climbing, or a global collision-free guarantee.
