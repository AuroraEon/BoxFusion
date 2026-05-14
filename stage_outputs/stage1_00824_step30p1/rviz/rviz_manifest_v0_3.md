# Stage1 Step30P1 RViz Manifest v0.3

Primary Stage1 config: `00824_stage1_step30p1_bev_semantic_route_demo.rviz`

Primary marker topic: `/step30s7_rviz_overlay_markers`

The config is a BEV/floorplan view in the `map` frame. It displays the Nav2 `/map` occupancy/free-space layer separately from semantic room segmentation markers, translucent room fill/outline, selected interior targets, gateways, planned route, executed trajectory, and invalid/suspicious wall-crossing segments.

The stable full-scene Nav2 `/map` floorplan and semantic room mask are visually distinct layers. The old room15-only map is not used for RViz display, and the request-aware map is no longer the default primary floorplan.

## Expected Topics

- `/map`
- `/tf`
- `/tf_static`
- `/scan`
- `/robot_description`
- `/plan`
- `/local_plan`
- `/step30s7_rviz_overlay_markers`
