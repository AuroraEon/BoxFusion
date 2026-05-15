# Step30S2 Bringup Dataplane Report

Created: `2026-05-14T13:29:24.966039+00:00`

Dataplane ready: `True`

## Topics

- `/clock`: publishers=1 subscribers=21 messages=249 rate_hz=9.952
- `/odom`: publishers=1 subscribers=2 messages=736 rate_hz=29.397
- `/scan`: publishers=1 subscribers=4 messages=125 rate_hz=4.993
- `/tf`: publishers=2 subscribers=7 messages=1221 rate_hz=48.87
- `/tf_static`: publishers=2 subscribers=7 messages=503 rate_hz=20.088
- `/map`: publishers=1 subscribers=3 messages=1 rate_hz=0.0
- `/cmd_vel`: publishers=4 subscribers=2 messages=0 rate_hz=0.0

## TF And Actions

- `map_to_odom`: `True`
- `odom_to_base_footprint`: `True`
- `odom_to_base_link`: `True`
- `base_footprint_to_base_link`: `True`
- `base_link_to_base_scan`: `True`
- `map_to_base_footprint`: `True`
- `map_to_base_link`: `True`
- action `/compute_path_to_pose`: `True` role=`non_blocking_diagnostic_unless_planning_gate_requested`
- action `/navigate_to_pose`: `True` role=`non_blocking_diagnostic_for_follow_path_runtime`
- action `/follow_path`: `True` role=`hard_blocker_for_current_route_execution`
