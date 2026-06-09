# RSLG-SLAM Cross-Floor 3D RViz MarkerArray Demo

This task24e/task24e2 demo publishes a graph/world-model overlay for scene `00843-DYehNKdT76V`.
It visualizes the already-built route from `floor_1 room_3` through `vc_vt_1` to
`floor_2 room_7 -> room_13 -> room_14`.

Claim boundary:

```text
RSLG-SLAM cross-floor topology overlay
topological_vertical_transition_only
physical_execution_supported=false
no Gazebo/Nav2/Habitat/robot execution in task24e2
```

Run:

```bash
tools/vertical_connectors/run_00843_cross_floor_3d_rviz_overlay_demo.sh
```

Useful options:

```bash
tools/vertical_connectors/run_00843_cross_floor_3d_rviz_overlay_demo.sh --no-rviz
tools/vertical_connectors/run_00843_cross_floor_3d_rviz_overlay_demo.sh --no-static-tf
tools/vertical_connectors/run_00843_cross_floor_3d_rviz_overlay_demo.sh --z-visual-scale 1.5
tools/vertical_connectors/run_00843_cross_floor_3d_rviz_overlay_demo.sh --topic /rslg/cross_floor_markers
```

The publisher uses `/usr/bin/python3`, converts the task24d JSON marker contract into
`visualization_msgs/msg/MarkerArray`, and publishes to `/rslg/cross_floor_markers`.
The QoS profile uses transient-local durability and also republishes periodically at
a low rate so RViz can see the markers after opening.

The task24e2 publisher adds two lightweight synthetic `floor_context` slab markers
at the existing `floor_1` and `floor_2` z references. These are visual context only;
they do not modify the task24d marker JSON or any validated runtime maps.

RViz manual setup:

1. Set `Fixed Frame` to `world`.
2. Add a `MarkerArray` display.
3. Select `/rslg/cross_floor_markers`.
4. Use an oblique camera view to see `floor_1`, the vertical connector centerline,
   and `floor_2` as a 3D stacked topology.

The standalone script starts a static transform publisher for `world -> map` by
default so RViz can display markers whose `frame_id` is `map` without Gazebo, Nav2,
or map server.

The floor transition marker is derived from `source_node.floor_id != target_node.floor_id`.
For this scene, the corrected transition edge is `vt_1_centerline_e001`
from `vt_1_centerline_n001` on `floor_1` to `vt_1_centerline_n002` on `floor_2`.
The old `vt_1_centerline_e003` edge is same-floor `floor_2 -> floor_2` and is not
treated as the transition.
