# Stage1 Nav2 Lifecycle Readiness Report

Created: `2026-05-14T13:26:54.670073+00:00`
Succeeded: `False`
Failure reason: `Nav2 lifecycle bringup failed or required dataplane readiness was not reached`
FollowPath-only runtime: `True`

## Lifecycle

- `/map_server`: state=`None` available=`False` timed_out=`True`
- `/controller_server`: state=`None` available=`False` timed_out=`True`
- `/planner_server`: state=`None` available=`False` timed_out=`True`
- `/bt_navigator`: state=`None` available=`False` timed_out=`True`
- `/recoveries_server`: state=`None` available=`False` timed_out=`True`
- `/waypoint_follower`: state=`None` available=`False` timed_out=`False`

## Map And Actions

- `/map` OccupancyGrid received with TRANSIENT_LOCAL QoS: `True`
- `/map` sample: `1227x1167` at `0.05000000074505806` m/cell
- `/follow_path`: ready=`True` role=`hard_blocker_for_current_route_execution`
- `/compute_path_to_pose`: ready=`True` role=`non_blocking_diagnostic_unless_planning_gate_requested`
- `/navigate_to_pose`: ready=`True` role=`non_blocking_diagnostic_for_follow_path_runtime`

## Missing Conditions

- `/map_server not active (query_timeout)`
- `/controller_server not active (query_timeout)`
- `/planner_server not active (query_timeout)`
- `/bt_navigator not active (query_timeout)`

## Stuck Nodes

- `/map_server: query_timeout`
- `/controller_server: query_timeout`
- `/planner_server: query_timeout`
- `/bt_navigator: query_timeout`
