# RSLG-SLAM Runtime Demo Package

Unified, reproducible demo surface for RSLG-SLAM runtime validation. (Repository
path `BoxFusion` is historical only; the project is **RSLG-SLAM**.)

The package hides task-specific path complexity behind a **route registry** and a
**single launcher**, and reuses the stable task58 (same-floor Gazebo PID) and
task60 (scripted stair transition) tools.

## Layer context
- Layer 3 (Navigation Interface) → Layer 4 (Runtime Validation) tooling only.
- Consumes task56c RouteResults + PID runtime inputs. Does not run Stage-A, raw
  RGB-D inference, or Layer 0/1/2 generation.

## Files
- Registry: `tools/rslg_pipeline/demo/rslg_demo_route_registry_v0_1.json`
- Launcher (shell): `tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh`
- Launcher helper (python): `tools/rslg_pipeline/demo/run_rslg_runtime_demo.py`
- Route classifier: `tools/rslg_pipeline/demo/classify_demo_route.py`

## Modes
| mode | meaning | world | rviz |
|---|---|---|---|
| same_floor_pid | single-floor Gazebo PID over floor-filtered waypoints | rslg_flat_empty_turtlebot3_burger.world | rslg_gazebo_actual_trajectory_showcase.rviz |
| scripted_stair_transition | floor_1 PID + scripted vt_1_centerline_e001 transition + floor_2 PID | rslg_scripted_stair_transition_turtlebot3_burger.world | rslg_scripted_stair_transition_showcase.rviz |
| connector_scripted_transition | vertical connector demo (no object goal) | scripted stair world | scripted stair rviz |
| dry_run_only | guard/availability validation only | — | — |

## Common commands
```bash
cd /home/ws/workspace/BoxFusion
# list routes
tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --list
# dry-run one route / all routes
tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --query-id 00843_object_to_path_curtain --dry-run
tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --dry-run
# print resolved paths
tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --print-paths --query-id 00843_cross_floor_object_curtain_room14
# headless execution (outputs isolated under task62)
tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --query-id 00843_object_to_path_curtain --headless --no-rviz-gui --duration-sec 60
# GUI: run gazebo + rviz in separate terminals, then the route with --with-gazebo-gui --with-rviz-gui
```

## Supported routes (QuerySet v0)
| query_id | mode | notes |
|---|---|---|
| 00843_object_to_path_curtain | same_floor_pid (floor_2) | 8 waypoints |
| 00843_object_in_room_curtain_room14 | same_floor_pid (floor_2) | 53 floor_2 waypoints |
| 00843_cross_floor_object_curtain_room14 | scripted_stair_transition | floor_1 + stair + floor_2 → generated_ring_002 |
| 00843_cross_floor_room_room2_to_room14 | scripted_stair_transition | dry-run first |
| 00843_floor_connector_floor1_to_floor2 | connector_scripted_transition | dry-run only |
| 00843_blocked_candidate_rejection_curtain | dry_run_only | generated_ring_037 rejection guard |

## Claim boundaries
- No Nav2, AMCL, map_server / nav2_map_server, planner_server, controller_server,
  bt_navigator, NavigateToPose, or FollowPath.
- No physical stair climbing; the scripted stair transition is an animation only.
- No real robot deployment; no global collision-free guarantee.
- `generated_ring_037` is never the runtime goal (blocked/rejected/evidence only).
- `vt_1_centerline_e003` is the non-transition edge and is never used as a transition edge.
- `vt_1_centerline_e001` is the only scripted transition edge for cross-floor demos.

## Reproducibility notes
- The launcher uses robust upward repo-root discovery (no fixed-depth paths).
- Live outputs default to `stage_outputs/.../task62_.../demo_package_pack/runs/<qid>_<UTC>/`.
- Scripted-stair runs set `RSLG_SCRIPTED_STAIR_TASK_DIR` so task60 outputs are never overwritten.
