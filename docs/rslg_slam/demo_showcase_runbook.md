# RSLG-SLAM Demo Showcase Runbook

Scene: `00843-DYehNKdT76V`

The task42 showcase is replay-oriented. It visualizes the already validated room-level task39 and object-level task41 controlled-simulation evidence. It does not require rerunning the route executor.

## Replay Exports

```bash
cd /home/ws/workspace/BoxFusion
/home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/export_task42_marker_overlays.py
/home/ws/miniconda3/envs/boxfusion/bin/python tools/rslg_pipeline/export_task42_demo_figures.py
```

Outputs are written to `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/`.

## RViz Replay

```bash
cd /home/ws/workspace/BoxFusion
RSLG_TASK42_ALLOW_RUNTIME=1 tools/rslg_pipeline/show_task42_rviz_replay.sh
```

The RViz package prefers MarkerArray overlays on `/task42_demo_markers`; the RViz Map display is intentionally omitted because the historical Map display path was avoided. The replay inputs include room and object route traces, executed trajectories, the `vt_1_centerline_e001` handoff marker, `generated_ring_002`, blocked `generated_ring_037`, and `obj_175`.

## Task42b 3D Dynamic RViz Replay

Task42 was the replay evidence package. Task42b adds a presentation-oriented 3D dynamic RViz replay without changing the validated Layer 1, Layer 2, Layer 3, or Layer 4 evidence.

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/foxy/setup.bash
export RSLG_TASK42B_ALLOW_RVIZ=1
tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh
```

Static-only view:

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/foxy/setup.bash
export RSLG_TASK42B_ALLOW_RVIZ=1
tools/rslg_pipeline/show_task42b_rviz_3d_static.sh
```

The task42b display publishes MarkerArray messages on `/task42b_3d_demo_markers`. It separates `floor_1` at z=`0.0 m` and `floor_2` at z=`1.6 m`, draws `vt_1_centerline_e001` as the 3D transition connector, and shows planned route, display-processed executed trajectory, current replay pose, `generated_ring_002`, blocked `generated_ring_037`, and `obj_175`.

RViz Map display is not required. Gazebo is not required. Nav2, map_server, route executors, object recovery, and route planning are not launched by the task42b wrappers.

Task42b smoothing and downsampling are visualization-only. They do not create new validation metrics and do not replace task39/task41 runtime evidence.

## Gazebo Replay Context

```bash
cd /home/ws/workspace/BoxFusion
RSLG_TASK42_ALLOW_RUNTIME=1 tools/rslg_pipeline/show_task42_gazebo_replay.sh
```

This command is a guarded showcase wrapper for reviewing the existing Gazebo evidence and logs. Task42 does not rerun live Gazebo by default.

## Claim Boundary

This demo is controlled simulation evidence only. It does not claim real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, or object-centroid navigation.
