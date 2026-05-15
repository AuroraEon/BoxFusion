# Step30S2 Bringup Dataplane Report

Created: `2026-05-14T13:04:17.016453+00:00`

Dataplane ready: `False`

## Topics

- `/clock`: publishers=1 subscribers=20 messages=250 rate_hz=9.963
- `/odom`: publishers=1 subscribers=2 messages=735 rate_hz=29.39
- `/scan`: publishers=1 subscribers=2 messages=125 rate_hz=4.998
- `/tf`: publishers=2 subscribers=4 messages=1221 rate_hz=48.851
- `/tf_static`: publishers=2 subscribers=4 messages=503 rate_hz=20.081
- `/map`: publishers=1 subscribers=2 messages=0 rate_hz=0.0
- `/cmd_vel`: publishers=1 subscribers=2 messages=0 rate_hz=0.0

## TF And Actions

- `map_to_odom`: `True`
- `odom_to_base_footprint`: `True`
- `odom_to_base_link`: `True`
- `base_footprint_to_base_link`: `True`
- `base_link_to_base_scan`: `True`
- `map_to_base_footprint`: `True`
- `map_to_base_link`: `True`
- action `/compute_path_to_pose`: `False`
- action `/navigate_to_pose`: `False`
- action `/follow_path`: `True`

## Missing Conditions

- `map_received`
- `navigate_to_pose_action_server_ready`
- `compute_path_to_pose_action_server_ready`
