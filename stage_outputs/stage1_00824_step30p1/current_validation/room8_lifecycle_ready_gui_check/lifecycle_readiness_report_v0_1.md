# Stage1 Nav2 Lifecycle Readiness Report

Created: `2026-05-14T13:28:55.871209+00:00`
Succeeded: `True`
Failure reason: `None`
FollowPath-only runtime: `True`

## Lifecycle

- `/map_server`: state=`active` available=`True` timed_out=`False`
- `/controller_server`: state=`active` available=`True` timed_out=`False`
- `/planner_server`: state=`active` available=`True` timed_out=`False`
- `/bt_navigator`: state=`active` available=`True` timed_out=`False`
- `/recoveries_server`: state=`active` available=`True` timed_out=`False`
- `/waypoint_follower`: state=`active` available=`True` timed_out=`False`

## Map And Actions

- `/map` OccupancyGrid received with TRANSIENT_LOCAL QoS: `True`
- `/map` sample: `1227x1167` at `0.05000000074505806` m/cell
- `/follow_path`: ready=`True` role=`hard_blocker_for_current_route_execution`
- `/compute_path_to_pose`: ready=`True` role=`non_blocking_diagnostic_unless_planning_gate_requested`
- `/navigate_to_pose`: ready=`True` role=`non_blocking_diagnostic_for_follow_path_runtime`
