# RSLG-SLAM Runtime Validation Runbook

Task38 packaged the selected `cross_floor_room` route under `conservative_canonical`. Task39 validated that route in authorized controlled simulation. Task40 recovered the object approach with `generated_ring_002`, and task41 validated the selected `cross_floor_object` route in authorized controlled simulation.

## Runtime Authorization

Actual ROS2, Gazebo, RViz, map_server, or route-executor runtime requires explicit authorization:

```bash
export RSLG_TASK42_ALLOW_RUNTIME=1
export RSLG_TASK38_ALLOW_RUNTIME=1
export RSLG_TASK39_ALLOW_RUNTIME=1
export RSLG_TASK41_ALLOW_RUNTIME=1
export ROS_DOMAIN_ID=84
export TURTLEBOT3_MODEL=burger
```

Runtime Python must be `/usr/bin/python3`. Offline/static artifact checks use `/home/ws/miniconda3/envs/boxfusion/bin/python`.

## Validated Runtime Evidence

Task39 completed:

1. `room_2 -> room_3 -> vt_1` on `floor_1`
2. explicit map/pose handoff over `vt_1_centerline_e001`
3. `vt_1 -> room_7 -> room_13 -> room_14` on `floor_2`

Task41 completed the same room route and the object approach segment to `generated_ring_002`, with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`.

`vt_1_centerline_e001` is the true transition edge. `vt_1_centerline_e003` is not the transition edge.

## Task42 Replay Showcase

Task42 generated replay assets under `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/`.

Use:

```bash
tools/rslg_pipeline/show_task42_rviz_replay.sh
tools/rslg_pipeline/show_task42_gazebo_replay.sh
```

The RViz replay uses MarkerArray overlays and intentionally does not depend on RViz Map display.

## Claim Boundary

This is controlled simulation evidence only. Do not claim real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, gait planning, footstep planning, contact planning, object-centroid navigation, manual target pose, generated_ring_037 as runtime goal, dense reconstruction, neural implicit SLAM, or LLM runtime navigation.
