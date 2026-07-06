# Legacy Runtime Removed (task49b)

RSLG-SLAM is the project name. `BoxFusion` is only a historical repository path.

task49b performed a schema-first aggressive refactor. The following active
source files were **deleted** from the formal runtime surface because they
depended on `nav2_map_server` / `map_server` lifecycle management and rclpy live
launch, which conflicts with the current no-Nav2, no-AMCL, lightweight PID
policy.

## Deleted source files

| Deleted file | Reason |
| --- | --- |
| `tools/rslg_pipeline/runtime/cross_floor_runtime.py` | task39 executor that launched `nav2_map_server` / `map_server` lifecycle nodes and rclpy runtime; superseded by the lightweight PID follower and `RSLGRouteResult`. |
| `tools/rslg_pipeline/runtime/object_runtime.py` | task41 object executor with the same `nav2_map_server` / `map_server` lifecycle dependency. |
| `tools/rslg_pipeline/run_task39_cross_floor_runtime.py` | compatibility wrapper importing the deleted `cross_floor_runtime`. |
| `tools/rslg_pipeline/run_task41_cross_floor_object_runtime.py` | compatibility wrapper importing the deleted `object_runtime`. |
| `tools/rslg_pipeline/runtime/legacy_live_runtime_launcher_removed.sh` | orphaned launcher stub referencing Nav2/ROS launch; only referenced by the deleted executors. |

## What replaces them

- Layer 3 now produces `RSLGRouteResult` (`tools/rslg_pipeline/planning/route_result.py`,
  `tools/rslg_pipeline/schemas/route_result_schema.json`).
- Layer 4 runtime is the lightweight PID `/cmd_vel` + `/odom` follower
  (`tools/rslg_pipeline/runtime/base_level_route_follower.py`), which consumes
  route interfaces rather than Nav2/`map_server`.

## Historical evidence retained

The task39 / task41 / task48 output directories under
`stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/` remain as evidence snapshots.
They are not schema definitions and are not part of the current formal runtime
source surface. `task48h/i/j` outputs are evidence snapshots only.

## Verification

`tools/rslg_pipeline/runtime/cross_floor_runtime.py` and
`tools/rslg_pipeline/runtime/object_runtime.py` are no longer importable. Any
remaining `nav2` / `map_server` / `AMCL` mentions in the formal source are
forbidden-claim guards (all false), process-cleanup patterns, or documentation
of this removal.
