# Occupancy Map Generation Audit

The previous active path generated `step30s7_request_aware_nav_map.yaml` from the route request and freed only requested/resolved route rooms.

The repaired active path builds `stage1_full_scene_occupancy_map.yaml` from Stage1 global geometry artifacts and does not read start, goal, terminal, through rooms, or route-room ids.

## Source Artifacts

- `final_gateway_wall_preclose_thr_0p25_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/final_gateway_wall_preclose_thr_0p25.png`
- `free_space_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/free_space.png`
- `gateway_projection_config`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/config/step30s7_gateway_projection_config_v0_1.json`
- `gateway_registry`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/gateway/gateway_registry_v0_1.json`
- `h8r2_masks_npz_for_gateway_carves_only`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_masks.npz`
- `h8r2_reference_map_yaml_for_metadata_and_baseline`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml`
- `outside_boundary_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/outside_boundary.png`
- `stage1_layered_bev_npz`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz`
- `stage1_room_mask`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy`

## Active Output

- map yaml: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`
- request dependent: `False`
- through rooms used: `False`
- Step30S5 patch used: `False`
