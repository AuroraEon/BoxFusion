# RSLG-SLAM RViz 3D Showcase Notes

Scene: `00843-DYehNKdT76V`

Task42b is a showcase-polish layer for existing RSLG-SLAM evidence. It does not rerun object recovery, route planning, Gazebo, Nav2, map_server, or route executors.

## Display Convention

The 3D RViz replay uses MarkerArray overlays as the primary display:

- topic: `/task42b_3d_demo_markers`
- fixed frame: `map`
- `floor_1`: z=`0.0 m`
- `floor_2`: z=`1.6 m`
- connector: `vt_1_centerline_e001`

`vt_1_centerline_e003` is not the floor-transition edge.

## Demo Commands

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/foxy/setup.bash
export RSLG_TASK42B_ALLOW_RVIZ=1
tools/rslg_pipeline/show_task42b_rviz_3d_replay.sh
```

Static-only:

```bash
cd /home/ws/workspace/BoxFusion
source /opt/ros/foxy/setup.bash
export RSLG_TASK42B_ALLOW_RVIZ=1
tools/rslg_pipeline/show_task42b_rviz_3d_static.sh
```

## Evidence Boundary

The task42b trajectory lines are downsampled and lightly smoothed for display only. The smoothing is not used for validation metrics and does not modify task39 or task41 artifacts.

The object target remains `generated_ring_002`; `generated_ring_037` remains blocked and is not used. The object centroid is shown only as context for `obj_175` and is not a navigation goal.

No real robot execution, physical stair climbing, gait planning, footstep planning, contact planning, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, dense reconstruction, neural implicit SLAM, or LLM runtime navigation is claimed.
