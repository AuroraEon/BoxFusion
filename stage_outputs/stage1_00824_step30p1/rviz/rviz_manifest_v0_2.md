# Stage1 Step30P1 RViz Manifest v0.2

Primary Step30S5 config: `00824_stage1_step30p1_bev_semantic_route_demo.rviz`

Primary marker topic: `/step30s5_rviz_overlay_markers`

The config is a BEV/floorplan view in the `map` frame. It displays the Nav2 `/map` occupancy/free-space layer separately from semantic room segmentation markers, room15 fill/outline, selected room15 interior target, r7-r15 gateway, planned route, executed trajectory, and invalid/suspicious wall-crossing segments.

## Expected Topics

- `/map`
- `/tf`
- `/tf_static`
- `/scan`
- `/robot_description`
- `/plan`
- `/local_plan`
- `/step30s5_rviz_overlay_markers`
