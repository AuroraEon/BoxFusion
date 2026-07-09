# Multi-Room / Multi-Object Goal Generalization (task63)

Project: **RSLG-SLAM** (Rich Semantic + Light Geometry semantic-topological navigation).
Scene: `00843-DYehNKdT76V`. `BoxFusion` is only a historical repository path.

This document describes the task63 object-goal generalization benchmark: taking **different
object goals in different rooms** and producing valid
`query -> target -> route -> runtime-adapter -> dry-run / Gazebo execution` outputs, while staying
inside the validated RSLG-SLAM runtime boundary.

## Motivation

The task62 demo package validated the existing QuerySet v0 routes, but all executable routes
target `obj_175` / `curtain` / `room_14`. task63 expands to a diverse, canonically-grounded set
of object goals to demonstrate true object-goal generalization rather than demo-package reuse.

## Pipeline

```
canonical world model snapshot
  -> audit_object_goal_candidates.py        (object inventory)
  -> select_multi_room_object_goals.py      (diverse goal selection)
  -> build_task63_object_goal_query_tasks.py(task-local QueryTasks)
  -> build_task63_object_goal_routes.py     (RouteResults via plan_query_static + adapters)
  -> run_task63_object_goal_generalization.py(dry-run classification)
  -> run_task63_gazebo_pid_smoke.sh         (same-floor Gazebo PID execution)
```

All tools live under `tools/rslg_pipeline/demo/`. Route generation reuses the existing generic
static planner `tools/rslg_pipeline/plan_query_static.py` and the runtime adapter exporter
`tools/rslg_pipeline/export_route_result_runtime_inputs.py`; no routing logic is duplicated.

## Object-goal realization (honest reuse)

The canonical object-approach resolver is single-object: it resolves `obj_175` to the validated
approach `generated_ring_002`. To avoid mis-attributing that approach to other objects,
non-reference object goals are routed to the **room that contains the object** (`room_gateway`
for same-floor, `cross_floor_room` for cross-floor); the object centroid is the in-room semantic
target. This is an honest object goal at **room-approach granularity**. The `obj_175` reference
goal reuses its previously validated fine-grained cross-floor route read-only.

## Results

- Inventory: 116 objects, 86 room/floor-assigned, 55 goal candidates, 33 categories, 10 rooms,
  2 floors.
- Selected: 7 goals (5 categories, 7 rooms, 2 floors; 5 same-floor + 2 cross-floor).
- Generated: 7 object-goal QueryTasks, 6 route-gen QueryTasks, 6 RouteResults (+1 reused),
  6 runtime adapters (+1 reused).
- Dry-run: 7/7 passed.
- Execution: 3/3 same-floor Gazebo PID runs passed (bed/room_11, toilet/room_8, couch/room_3),
  all reaching 100% of filtered-floor waypoints with final error ≈ 0.075 m.

## Runtime boundary (unchanged)

- Same-floor: Gazebo TurtleBot3 burger + lightweight PID follower (`practical_zero_collision`).
- Cross-floor: floor_1 PID + scripted `vt_1_centerline_e001` transition animation + floor_2 PID.
- The scripted stair transition is a visual/runtime-interface demo, **not** physical stair
  climbing (`physical_stair_climbing_claim` stays false).
- No Nav2, no AMCL, no `map_server`/`nav2_map_server`, no `planner_server`/`controller_server`/
  `bt_navigator`, no `NavigateToPose`/`FollowPath`, no real-robot claim, no global collision-free
  guarantee.
- `generated_ring_037` is never a runtime goal; `vt_1_centerline_e003` is never a transition edge.
- Generated QueryTasks are task-local; the official QuerySet v0 is not modified.

## Limitations and next step

Fine-grained per-object approach poses exist only for `obj_175`. The recommended next task is to
recover and validate object-approach candidates for the additional task63 objects so future
object-goal routes can terminate at a validated object-approach pose, and to execute a cross-floor
scripted-transition demo for one non-reference object.
