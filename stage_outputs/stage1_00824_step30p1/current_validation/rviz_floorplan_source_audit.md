# RViz Floorplan Source Audit

- RViz config: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`
- Primary Map display subscribes to `/map`.
- `/map` is loaded from `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml` in both validated runs.
- Request-aware maps are not the primary RViz floorplan in the repaired default profile.
- Semantic room fill, route highlighting, labels, gateways, path, and trajectory are marker overlays separate from the base floorplan.
