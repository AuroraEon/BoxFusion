# Stable Full-Scene Occupancy Map Provenance

Map: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml`
Policy: stable full-scene Stage1 global geometry, request-independent.

## Inputs

- `h8r2_reference_map_yaml_for_metadata_and_baseline`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml` exists `True`
- `h8r2_masks_npz_for_gateway_carves_only`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_masks.npz` exists `True`
- `stage1_layered_bev_npz`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz` exists `True`
- `stage1_room_mask`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy` exists `True`
- `final_gateway_wall_preclose_thr_0p25_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/final_gateway_wall_preclose_thr_0p25.png` exists `True`
- `outside_boundary_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/outside_boundary.png` exists `True`
- `free_space_png`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/free_space.png` exists `True`
- `gateway_registry`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/gateway/gateway_registry_v0_1.json` exists `True`
- `gateway_projection_config`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/config/step30s7_gateway_projection_config_v0_1.json` exists `True`

## Checks

- room15 free cells: `1295`
- through rooms used: `False`
- route room whitelist used: `False`
- Step30S5 room15 patch used: `False`
- validation passed: `True`
