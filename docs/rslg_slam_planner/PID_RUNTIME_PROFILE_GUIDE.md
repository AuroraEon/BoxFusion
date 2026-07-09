# RSLG-SLAM PID Runtime Profile Guide

This guide documents lightweight PID replay profiles derived from task56b. The
profiles are for RouteResult-derived replay validation only:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> Layer 4 adapter inputs -> lightweight PID replay`

They do not change planner behavior, QueryTasks, or canonical artifacts. They
do not run Stage-A, raw RGB-D inference, Nav2, AMCL, `map_server`, Gazebo,
RViz live processes, or physical robot code. A zero-collision replay result is
not a global collision-free guarantee.

## Profiles

Profile config:

`configs/rslg_runtime_profiles/pid_profiles_v0_1.json`

### baseline_task56

The task56 baseline is retained for comparison only.

- source: `task56`
- success/partial/failed: `2/4/0`
- stable-map footprint collision samples: `32`
- average final error: about `0.098865 m`
- waypoint reached ratio: `1.0`
- status: baseline-only, not recommended default

Parameters:

```json
{
  "dt": 0.1,
  "max_linear_velocity": 0.25,
  "max_angular_velocity": 0.8,
  "linear_gain": 0.8,
  "angular_gain": 1.5,
  "waypoint_tolerance": 0.12,
  "yaw_tolerance": 0.25,
  "timeout_sec": 240.0,
  "stuck_window_sec": 8.0,
  "stuck_progress_epsilon": 0.02,
  "robot_radius": 0.18
}
```

### conservative_zero_collision

The conservative profile is the safest task56b parameter-only candidate.

- source candidate: `candidate_011_v0p10_w0p8_lg0p8_ag2p5_tol0p06_yaw0p20`
- success/partial/failed in task56b: `6/0/0`
- stable-map footprint collision samples: `0`
- average final error: about `0.043215 m`
- waypoint reached ratio: `1.0`
- intended use: fallback / safety profile for lightweight replay

Parameters:

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

### practical_zero_collision

The practical profile keeps more speed while preserving the task56b zero
collision result.

- source candidate: `candidate_001_lower_v_stricter_tol`
- success/partial/failed in task56b: `6/0/0`
- stable-map footprint collision samples: `0`
- average final error: about `0.062766 m`
- waypoint reached ratio: `1.0`
- intended use: recommended practical lightweight replay default when
  regression passes

Parameters:

```json
{
  "dt": 0.1,
  "max_linear_velocity": 0.15,
  "max_angular_velocity": 0.8,
  "linear_gain": 0.8,
  "angular_gain": 2.0,
  "waypoint_tolerance": 0.08,
  "yaw_tolerance": 0.25,
  "timeout_sec": 240.0,
  "stuck_window_sec": 8.0,
  "stuck_progress_epsilon": 0.02,
  "robot_radius": 0.18
}
```

### baseline_plus_densify_010

Task56b also showed that replay-time densification at `0.10 m` and `0.05 m`
removed collision samples with baseline PID parameters. That is documented as
diagnostic evidence for corner-cutting / tracking tolerance sensitivity. It is
not promoted as a default profile because it transforms replay inputs and is
not a planner or canonical artifact change.

## Run A Profile

Generate task-local RouteResults and adapter inputs as in
`docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md`, then run:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python -m tools.rslg_pipeline.runtime.run_pid_profile_replay \
  --profile-json configs/rslg_runtime_profiles/pid_profiles_v0_1.json \
  --profile-id practical_zero_collision \
  --pid-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/regression_pack/runtime_adapter_inputs/pid_follower_inputs \
  --route-results-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/regression_pack/route_results \
  --z-aware-inputs-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/regression_pack/runtime_adapter_inputs/z_aware_overlay_inputs \
  --output-dir stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/<task>/regression_pack/practical_zero_collision_pid_replay \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title "RSLG-SLAM practical PID profile replay"
```

Change `--profile-id` to `conservative_zero_collision` to run the fallback
profile, or `baseline_task56` to reproduce the comparison baseline.

## Inspect Reports

Each replay output directory contains:

- `summary.json`
- `index.md`
- `index.html`
- `profile_run_metadata.json`
- per-query reports under `reports/`
- trajectory CSV/SVG files under `trajectories/`

Use a profile only when the summary preserves all project guards:

- `generated_ring_037_selected_count == 0`
- `vt_1_centerline_e003_transition_count == 0`
- `success_count == 6`
- `partial_count == 0`
- `failure_count == 0`
- `collision_check_mode == stable_map_footprint`
- total collision count across reports is `0`
- invalid-cell count is `0`
- no stuck/timeout
- average final error is within the task threshold

## Selection Guidance

Use `practical_zero_collision` as the current default lightweight replay
profile only after regression confirms the guard set above. Keep
`conservative_zero_collision` as the fallback / safety profile. Keep
`baseline_task56` as a comparison profile only.

These profiles are replay evidence for the current static chain. They are not
Nav2, AMCL, `map_server`, physical stair climbing, real robot deployment, or a
global collision-free guarantee.
