# Step30S7 Request-Aware Projection GUI Command Transcript

Started: `2026-05-14T07:12:19Z`
Run id: `room8_manual_check`
Stage output: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1`
Evidence: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check`

## build_request_aware_map

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/build_stage1_step30p1_request_aware_nav_map.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/build_request_aware_map.log`

<details><summary>tail</summary>

```text
        "intermediate_route"
      ],
      "largest_free_component_area_cells": 4687,
      "occupied_to_free_cells": 0,
      "semantic_mask_area_cells": 5015,
      "step30s7_free_cells": 5005,
      "step30s7_free_fraction": 0.998006,
      "unknown_to_free_cells": 0
    },
    "room_8": {
      "changed_cells": 0,
      "connected_component_count": 3,
      "free_connected_component_sizes_desc": [
        5583,
        48,
        24
      ],
      "h8r2_free_cells": 5655,
      "h8r2_free_fraction": 0.989328,
      "included_because": [
        "resolved_route",
        "through"
      ],
      "largest_free_component_area_cells": 5583,
      "occupied_to_free_cells": 0,
      "semantic_mask_area_cells": 5716,
      "step30s7_free_cells": 5655,
      "step30s7_free_fraction": 0.989328,
      "unknown_to_free_cells": 0
    }
  },
  "gateway_to_interior_connectivity_by_requested_room": {},
  "general_request_aware_successor": true,
  "included_room_ids": [
    1,
    3,
    7,
    8,
    11,
    14,
    16
  ],
  "included_room_names": [
    "room_1",
    "room_3",
    "room_7",
    "room_8",
    "room_11",
    "room_14",
    "room_16"
  ],
  "included_room_reasons": {
    "room_1": [
      "resolved_route",
      "start"
    ],
    "room_11": [
      "intermediate_route"
    ],
    "room_14": [
      "intermediate_route"
    ],
    "room_16": [
      "goal",
      "resolved_route",
      "terminal"
    ],
    "room_3": [
      "intermediate_route"
    ],
    "room_7": [
      "intermediate_route"
    ],
    "room_8": [
      "resolved_route",
      "through"
    ]
  },
  "not_room15_only_patched_map": true,
  "outputs": {
    "map_npz": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.npz",
    "map_pgm": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.pgm",
    "map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml"
  },
  "request_rooms": {
    "goal_room": "room_16",
    "start_room": "room_1",
    "terminal_room": "room_16",
    "through_rooms": [
      "room_8"
    ]
  },
  "resolved_route": [
    "room_1",
    "room_3",
    "room_8",
    "room_11",
    "room_7",
    "room_14",
    "room_16"
  ],
  "selected_gateway_ids": [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01"
  ],
  "source_inputs": {
    "gateway_hypotheses": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json",
    "h8r2_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
    "h8r2_masks_npz": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_masks.npz",
    "stage1_layered_bev": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz",
    "stage1_room_mask": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy"
  },
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "validation_passed": null,
  "version": "v0_1"
}
```
</details>

## request_map_validation

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/validate_stage1_step30p1_request_map.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 --map-yaml /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/request_map_validation_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/request_map_validation_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/request_map_validation.log`

<details><summary>tail</summary>

```text
      "room": "room_16",
      "semantic_mask_area_cells": 1584,
      "step30s7_free_cells": 1556,
      "step30s7_free_space_overlap_fraction": 0.982323,
      "target_distance_from_nearest_gateway_m": 0.813083,
      "target_wall_clearance_m": 0.45,
      "unknown_or_non_free_overlap_cells": 28,
      "validation_passed": true,
      "wall_clearance_conflict_cells": 0
    },
    "room_3": {
      "connected_component_count": 4,
      "gateway_to_interior_connectivity": true,
      "included_because": [
        "intermediate_route"
      ],
      "interior_target": {
        "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
        "distance_to_nearest_gateway_m": 2.421304,
        "distance_to_occupied_or_unknown_m": 0.55,
        "meets_preferred_gateway_distance": true,
        "meets_wall_distance_threshold": true,
        "room_id": "room_3",
        "x": 3.8,
        "y": -0.6,
        "yaw": 0.0
      },
      "interior_target_feasible": true,
      "largest_free_component_area_cells": 19876,
      "original_h8r2_free_cells": 20343,
      "original_h8r2_free_space_overlap_fraction": 0.995157,
      "preexisting_wall_label_free_overlap_cells": 17,
      "room": "room_3",
      "semantic_mask_area_cells": 20442,
      "step30s7_free_cells": 20343,
      "step30s7_free_space_overlap_fraction": 0.995157,
      "target_distance_from_nearest_gateway_m": 2.421304,
      "target_wall_clearance_m": 0.55,
      "unknown_or_non_free_overlap_cells": 99,
      "validation_passed": true,
      "wall_clearance_conflict_cells": 0
    },
    "room_7": {
      "connected_component_count": 2,
      "gateway_to_interior_connectivity": true,
      "included_because": [
        "intermediate_route"
      ],
      "interior_target": {
        "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
        "distance_to_nearest_gateway_m": 2.149758,
        "distance_to_occupied_or_unknown_m": 0.85,
        "meets_preferred_gateway_distance": true,
        "meets_wall_distance_threshold": true,
        "room_id": "room_7",
        "x": -0.1,
        "y": -5.65,
        "yaw": 0.0
      },
      "interior_target_feasible": true,
      "largest_free_component_area_cells": 4687,
      "original_h8r2_free_cells": 5005,
      "original_h8r2_free_space_overlap_fraction": 0.998006,
      "preexisting_wall_label_free_overlap_cells": 6,
      "room": "room_7",
      "semantic_mask_area_cells": 5015,
      "step30s7_free_cells": 5005,
      "step30s7_free_space_overlap_fraction": 0.998006,
      "target_distance_from_nearest_gateway_m": 2.149758,
      "target_wall_clearance_m": 0.85,
      "unknown_or_non_free_overlap_cells": 10,
      "validation_passed": true,
      "wall_clearance_conflict_cells": 0
    },
    "room_8": {
      "connected_component_count": 3,
      "gateway_to_interior_connectivity": true,
      "included_because": [
        "resolved_route",
        "through"
      ],
      "interior_target": {
        "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
        "distance_to_nearest_gateway_m": 1.793485,
        "distance_to_occupied_or_unknown_m": 1.33938,
        "meets_preferred_gateway_distance": true,
        "meets_wall_distance_threshold": true,
        "room_id": "room_8",
        "x": 7.25,
        "y": -5.8,
        "yaw": 0.0
      },
      "interior_target_feasible": true,
      "largest_free_component_area_cells": 5583,
      "original_h8r2_free_cells": 5655,
      "original_h8r2_free_space_overlap_fraction": 0.989328,
      "preexisting_wall_label_free_overlap_cells": 24,
      "room": "room_8",
      "semantic_mask_area_cells": 5716,
      "step30s7_free_cells": 5655,
      "step30s7_free_space_overlap_fraction": 0.989328,
      "target_distance_from_nearest_gateway_m": 1.793485,
      "target_wall_clearance_m": 1.33938,
      "unknown_or_non_free_overlap_cells": 61,
      "validation_passed": true,
      "wall_clearance_conflict_cells": 0
    }
  },
  "selected_gateway_ids": [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01"
  ],
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "validation_passed": true,
  "version": "v0_1"
}
```
</details>

## bev_visual_regression

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/audit_stage1_step30p1_bev_visual_regression.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 --map-yaml /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bev_visual_regression_report_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bev_visual_regression_report_v0_1.md --visual-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/visual_diagnostics 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bev_visual_regression.log`

<details><summary>tail</summary>

```text
{
  "artifact_type": "step30s7_bev_visual_regression_report",
  "broad_uncontrolled_cleanup_occurred": false,
  "changed_cells_inside_requested_route_rooms": 0,
  "changed_cells_outside_requested_route_rooms": 0,
  "changed_cells_vs_h8r2": 0,
  "created_utc": "2026-05-14T07:12:21.627760+00:00",
  "non_route_rooms_touched": [],
  "not_compared_against_step30s5_patched_map": true,
  "original_h8r2_map": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
  "resolved_route": [
    "room_1",
    "room_3",
    "room_8",
    "room_11",
    "room_7",
    "room_14",
    "room_16"
  ],
  "room_floorplan_boundaries_visually_distorted": false,
  "rviz_bev_display_remains_clean_and_interpretable": true,
  "semantic_mask_and_nav2_free_space_are_separate_layers": true,
  "step30s7_map": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml",
  "unknown_cells_changed_to_free": 0,
  "validation_passed": true,
  "version": "v0_1",
  "visual_diagnostics": [
    "full_map_difference_visualization.png",
    "original_h8r2_map_clipped_around_route.png",
    "requested_room_coverage_visualization.png",
    "room15_semantic_mask_vs_h8r2_free.png",
    "room15_semantic_mask_vs_step30s7_free.png",
    "rviz_expected_layer_preview.png",
    "step30s7_map_clipped_around_route.png"
  ],
  "visual_diagnostics_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/visual_diagnostics",
  "wall_or_occupied_evidence_cells_changed_to_free": 0
}
```
</details>

## gateway_generalization_readiness

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/write_stage1_step30p1_gateway_readiness_report.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/gateway_generalization_readiness_report_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/gateway_generalization_readiness_report_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/gateway_generalization_readiness.log`

<details><summary>tail</summary>

```text
{
  "active_route_projection_code_hard_codes_room15": false,
  "active_route_projection_code_hard_codes_route_room_ids": false,
  "artifact_type": "step30s7_gateway_generalization_readiness_report",
  "created_utc": "2026-05-14T07:12:21.853296+00:00",
  "externalized": {
    "forbidden_gateway_pairs": [
      "r14_r15",
      "r15_r16",
      "r3_r11",
      "r3_r15",
      "r7_r16"
    ],
    "gateway_hypotheses_path": "stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json",
    "projection_inputs": [
      "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy",
      "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz",
      "maps/h8r2_gateway_preserving_masks.npz"
    ],
    "selected_gateway_pairs": {
      "r14_r16": "gw_00824_r14_r16_01",
      "r1_r3": "gw_00824_r1_r3_01",
      "r3_r7": "gw_00824_r3_r7_01",
      "r3_r8": "gw_00824_r3_r8_01",
      "r7_r11": "gw_00824_r7_r11_02",
      "r7_r14": "gw_00824_r7_r14_01",
      "r7_r15": "gw_00824_r7_r15_01",
      "r8_r11": "gw_00824_r8_r11_01"
    }
  },
  "full_multi_scene_gateway_generalization_claimed": false,
  "readiness_summary": "Step30S7 externalizes obvious route/projection constants and uses request-aware inclusion, but does not claim full multi-scene gateway extraction generalization.",
  "scene_specific_remaining": [
    "The config is still for scene 00824 and depends on scene-local gateway artifacts.",
    "Gateway extraction itself is not re-run as a multi-scene truth-blind package in Step30S7.",
    "Topology quality still depends on the selected gateway artifact produced for this scene."
  ],
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "version": "v0_1"
}
```
</details>

## route_query

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/prepare_stage1_step30p1_semantic_route.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 --map-profile step30s7_request_aware --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_query_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_query_result_v0_1.md --waypoints-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/semantic_route_waypoints_v0_1.json --target-selection-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/room15_target_selection_v0_1.json --target-selection-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/room15_target_selection_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_query.log`

<details><summary>tail</summary>

```text
        "x": -4.15,
        "y": -3.3
      },
      "from_room": "room_14",
      "gateway_id": "gw_00824_r14_r16_01",
      "method": "nearby_free_cell_maximizing_nav_and_structural_clearance",
      "move_distance_m": 0.010424,
      "nav_map_clearance_m": 0.309845,
      "original_xy": {
        "x": -4.1396,
        "y": -3.3007
      },
      "room_constraint": null,
      "structural_wall_clearance_m": 0.35,
      "to_room": "room_16",
      "waypoint_source": "segment_crossing"
    },
    {
      "adjusted": true,
      "adjusted_xy": {
        "x": -3.95,
        "y": -2.45
      },
      "from_room": "room_14",
      "gateway_id": "gw_00824_r14_r16_01",
      "method": "nearby_free_cell_maximizing_nav_and_structural_clearance",
      "move_distance_m": 0.538558,
      "nav_map_clearance_m": 0.529843,
      "original_xy": {
        "x": -4.1481,
        "y": -2.9508
      },
      "room_constraint": 16,
      "structural_wall_clearance_m": 0.529843,
      "to_room": "room_16",
      "waypoint_source": "segment_goal"
    }
  ],
  "goal_room": "room_16",
  "included_room_reasons": {
    "room_1": [
      "resolved_route",
      "start"
    ],
    "room_11": [
      "intermediate_route"
    ],
    "room_14": [
      "intermediate_route"
    ],
    "room_16": [
      "goal",
      "resolved_route",
      "terminal"
    ],
    "room_3": [
      "intermediate_route"
    ],
    "room_7": [
      "intermediate_route"
    ],
    "room_8": [
      "resolved_route",
      "through"
    ]
  },
  "interior_targets": {
    "room_16": {
      "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
      "distance_to_nearest_gateway_m": 0.807775,
      "distance_to_occupied_or_unknown_m": 0.36969,
      "meets_preferred_gateway_distance": true,
      "meets_wall_distance_threshold": true,
      "room_id": "room_16",
      "x": -3.2,
      "y": -2.75,
      "yaw": 0.0
    },
    "room_8": {
      "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
      "distance_to_nearest_gateway_m": 1.432655,
      "distance_to_occupied_or_unknown_m": 1.33938,
      "meets_preferred_gateway_distance": true,
      "meets_wall_distance_threshold": true,
      "room_id": "room_8",
      "x": 7.25,
      "y": -5.8,
      "yaw": 0.0
    }
  },
  "map_profile": "step30s7_request_aware",
  "map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml",
  "not_room15_specific": true,
  "revisited_rooms": [
    "room_3",
    "room_7",
    "room_8",
    "room_11",
    "room_14"
  ],
  "room_sequence": [
    "room_1",
    "room_3",
    "room_8",
    "room_11",
    "room_7",
    "room_14",
    "room_16"
  ],
  "route_id": "room_chain_r1_r3_r8_r11_r7_r14_r16",
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "start_room": "room_1",
  "terminal_room": "room_16",
  "through_rooms": [
    "room_8"
  ],
  "version": "v0_1",
  "waypoint_count": 119,
  "waypoints_path": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/semantic_route_waypoints_v0_1.json"
}
```
</details>

## clean_old_processes

```bash
/home/ws/workspace/BoxFusion/tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --log-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bringup_logs --stop 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/clean_old_processes.log`

<details><summary>tail</summary>

```text
gazebo: not running
robot_state_publisher: not running
static_tf: not running
nav2: not running
```
</details>

## bringup

```bash
/home/ws/workspace/BoxFusion/tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --ros-domain-id 84 --gui --map-profile step30s7_request_aware --log-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bringup_logs --spawn-x 5.85 --spawn-y 2.1 --spawn-yaw 3.141593 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bringup.log`

<details><summary>tail</summary>

```text
[stage1_step30p1] starting Gazebo world: /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf
[stage1_step30p1] starting robot_state_publisher
[stage1_step30p1] spawning TurtleBot3 burger
[stage1_step30p1] starting static map->odom TF
[stage1_step30p1] starting Nav2 staticloc stack
[stage1_step30p1] launched. Logs: /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/bringup_logs
gazebo: running pid=20484
robot_state_publisher: running pid=20753
static_tf: running pid=20802
nav2: running pid=20804
```
</details>

## dataplane_probe

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/probe_stage1_step30p1_dataplane.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --timeout-sec 25 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/dataplane_probe_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/dataplane_probe_result_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/dataplane_probe.log`

<details><summary>tail</summary>

```text
    "odom_nonzero_rate": true,
    "scan_nonzero_rate": true,
    "staticloc_dataplane_ready": true,
    "tf_base_footprint_to_base_link_exists_or_recoverable": true,
    "tf_base_link_to_base_scan_exists": true,
    "tf_has_frames": true,
    "tf_map_to_odom_exists": true,
    "tf_odom_to_robot_exists": true,
    "tf_static_has_frames": true
  },
  "samples": {
    "map": {
      "frame_id": "map",
      "height": 1167,
      "origin": {
        "x": -50.0,
        "y": -50.0
      },
      "resolution": 0.05000000074505806,
      "width": 1227
    },
    "odom": {
      "child_frame_id": "base_footprint",
      "frame_id": "odom",
      "x": 5.849963,
      "y": 2.099998,
      "yaw": -3.141586
    },
    "scan": {
      "finite_count": 0,
      "finite_max": null,
      "finite_min": null,
      "frame_id": "base_scan",
      "range_count": 360
    }
  },
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "tf_edges_observed": {
    "/tf": [
      "base_link->wheel_left_link",
      "base_link->wheel_right_link",
      "odom->base_footprint"
    ],
    "/tf_static": [
      "base_footprint->base_link",
      "base_link->base_scan",
      "base_link->caster_back_link",
      "base_link->imu_link",
      "map->odom"
    ]
  },
  "tf_frames": {
    "base_footprint_to_base_link": true,
    "base_link_to_base_scan": true,
    "map_to_base_footprint": true,
    "map_to_base_link": true,
    "map_to_odom": true,
    "odom_to_base_footprint": true,
    "odom_to_base_link": true
  },
  "topics": {
    "/clock": {
      "message_count_observed": 250,
      "message_received": true,
      "observed_rate_hz": 9.953,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 21
    },
    "/cmd_vel": {
      "message_count_observed": 0,
      "message_received": false,
      "observed_rate_hz": 0.0,
      "probe_window_sec": 25.0,
      "publisher_count": 4,
      "subscription_count": 2
    },
    "/map": {
      "message_count_observed": 1,
      "message_received": true,
      "observed_rate_hz": 0.0,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 3
    },
    "/odom": {
      "message_count_observed": 736,
      "message_received": true,
      "observed_rate_hz": 29.407,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 2
    },
    "/scan": {
      "message_count_observed": 125,
      "message_received": true,
      "observed_rate_hz": 4.994,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 4
    },
    "/tf": {
      "message_count_observed": 1223,
      "message_received": true,
      "observed_rate_hz": 48.89,
      "probe_window_sec": 25.0,
      "publisher_count": 2,
      "subscription_count": 7
    },
    "/tf_static": {
      "message_count_observed": 503,
      "message_received": true,
      "observed_rate_hz": 20.097,
      "probe_window_sec": 25.0,
      "publisher_count": 2,
      "subscription_count": 7
    }
  },
  "version": "v0_1"
}
```
</details>

## route_execution

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/run_stage1_step30p1_route.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --waypoints-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/semantic_route_waypoints_v0_1.json --expected-room-chain room_1\,room_3\,room_8\,room_11\,room_7\,room_14\,room_16 --expected-gateway-sequence gw_00824_r1_r3_01\,gw_00824_r3_r8_01\,gw_00824_r8_r11_01\,gw_00824_r7_r11_02\,gw_00824_r7_r14_01\,gw_00824_r14_r16_01 --allow-non-step30p1-truth --from-start --reset-to-route-start --follow-path-timeout-sec 480 --goal-timeout-sec 180 --split-dwell-source room8_interior_terminal --split-dwell-sec 3.0 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_execution_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_execution_result_v0_1.md --trajectory-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/trajectory_sample_result_v0_1.json --latest-slice-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/latest_follow_path_slice_v0_1.json 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_execution.log`

<details><summary>tail</summary>

```text
      "x": -4.041445772787033,
      "y": -3.1221334488888894,
      "yaw": 1.3110303552244744,
      "z": 0.008730914216925838
    },
    {
      "frame_id": "map",
      "sample_index": 348,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742966.0059006,
      "x": -4.020710053781379,
      "y": -3.0354066301971807,
      "yaw": 1.3493663097506057,
      "z": 0.008731051119092328
    },
    {
      "frame_id": "map",
      "sample_index": 349,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742966.5342414,
      "x": -3.994222135457203,
      "y": -2.9430810123074957,
      "yaw": 1.2285527792744846,
      "z": 0.008730727762110096
    },
    {
      "frame_id": "map",
      "sample_index": 350,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742967.0458775,
      "x": -3.955874092475779,
      "y": -2.8709551807707814,
      "yaw": 0.9216983042640409,
      "z": 0.00873082652516551
    },
    {
      "frame_id": "map",
      "sample_index": 351,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742967.5514245,
      "x": -3.893998130819696,
      "y": -2.8127453030401415,
      "yaw": 0.6069602016150574,
      "z": 0.008716364896216714
    },
    {
      "frame_id": "map",
      "sample_index": 352,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742968.0668213,
      "x": -3.8183490281841412,
      "y": -2.7721842401758776,
      "yaw": 0.3857885246556989,
      "z": 0.008730989335080175
    },
    {
      "frame_id": "map",
      "sample_index": 353,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742968.576591,
      "x": -3.735934681237878,
      "y": -2.746839349409051,
      "yaw": 0.22737242963404108,
      "z": 0.008730967988971886
    },
    {
      "frame_id": "map",
      "sample_index": 354,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742969.0875597,
      "x": -3.6499003870369178,
      "y": -2.7346450927987007,
      "yaw": 0.0598645286659881,
      "z": 0.008710140246781914
    },
    {
      "frame_id": "map",
      "sample_index": 355,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742969.598612,
      "x": -3.570088310156693,
      "y": -2.733381996580249,
      "yaw": -0.0223493408764772,
      "z": 0.008731023612058237
    },
    {
      "frame_id": "map",
      "sample_index": 356,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742970.1011798,
      "x": -3.4854147765360017,
      "y": -2.738188050260186,
      "yaw": -0.08023173943142953,
      "z": 0.008730861848927515
    },
    {
      "frame_id": "map",
      "sample_index": 357,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742970.6049175,
      "x": -3.412239995640968,
      "y": -2.744494707990025,
      "yaw": -0.0922011081103311,
      "z": 0.008712221940405995
    },
    {
      "frame_id": "map",
      "sample_index": 358,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778742971.1055,
      "x": -3.391005811817172,
      "y": -2.74675751131378,
      "yaw": -0.2067184029703343,
      "z": 0.008723942641086207
    }
  ],
  "version": "v0_1",
  "waypoint_results": [],
  "waypoints_source": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/semantic_route_waypoints_v0_1.json"
}
```
</details>

## physical_validation

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/validate_stage1_step30p1_physical.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --route-query-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_query_result_v0_1.json --waypoints-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/semantic_route_waypoints_v0_1.json --route-execution-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/route_execution_result_v0_1.json --trajectory-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/trajectory_sample_result_v0_1.json --through-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/through_room15_physical_visit_validation_v0_1.json --through-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/through_room15_physical_visit_validation_v0_1.md --terminal-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/room16_terminal_quality_validation_v0_1.json --terminal-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/room16_terminal_quality_validation_v0_1.md --wall-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/trajectory_wall_crossing_validation_v0_2.json --wall-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/trajectory_wall_crossing_validation_v0_2.md --spin-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/local_looping_spin_validation_v0_2.json --spin-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/local_looping_spin_validation_v0_2.md --room15-min-inside-samples 8 --through-room-dwell-sec 3.0 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_check/physical_validation.log`

<details><summary>tail</summary>

```text
    "local_loop_count_around_gateway": 0,
    "local_looping_validation_passed": true,
    "near_gateway_interval_count": 13,
    "near_stationary_high_yaw_change_interval_count": 0,
    "repeated_revisit_cell_count_near_gateway": 0,
    "spinning_detected": false,
    "total_angular_travel_near_gateway_rad": 1.311986,
    "version": "v0_1"
  },
  "terminal": {
    "artifact_type": "stage1_room16_terminal_quality_validation",
    "created_utc": "2026-05-14T07:16:11.873029+00:00",
    "final_arrival_action_success": true,
    "final_pose": {
      "frame_id": "map",
      "source": "/tf map->base_footprint",
      "x": -3.3911875890120786,
      "y": -2.7466995995274837,
      "yaw": -0.32803826120417995,
      "z": 0.008728533813973672
    },
    "final_pose_distance_to_r14_r16_gateway_m": 0.939115,
    "final_pose_distance_to_room16_interior_target_m": 0.191216,
    "final_pose_inside_room16_interior_region": true,
    "final_pose_inside_room16_mask": true,
    "final_pose_near_r14_r16_gateway": false,
    "min_distance_to_room16_interior_target": 0.191033,
    "room16_interior_target": {
      "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
      "distance_to_nearest_gateway_m": 0.807775,
      "distance_to_occupied_or_unknown_m": 0.36969,
      "meets_preferred_gateway_distance": true,
      "meets_wall_distance_threshold": true,
      "room_id": "room_16",
      "x": -3.2,
      "y": -2.75,
      "yaw": 0.0
    },
    "route_execution_success": true,
    "terminal_visual_quality_passed": true,
    "version": "v0_1"
  },
  "through": {
    "all_through_rooms_success": true,
    "artifact_type": "through_room_physical_visit_validation",
    "created_utc": "2026-05-14T07:16:11.872994+00:00",
    "gateway_only_failure_guard_passed": true,
    "room15_inside_dwell_sec": 0.0,
    "room15_inside_sample_count": 0,
    "route_topology_includes_room15": false,
    "through_room_results": {
      "room_8": {
        "gateway_crossed": true,
        "gateway_only_failure_guard_passed": true,
        "inside_dwell_sec": 33.674,
        "inside_sample_count": 73,
        "interior_target": {
          "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
          "distance_to_nearest_gateway_m": 1.432655,
          "distance_to_occupied_or_unknown_m": 1.33938,
          "meets_preferred_gateway_distance": true,
          "meets_wall_distance_threshold": true,
          "room_id": "room_8",
          "x": 7.25,
          "y": -5.8,
          "yaw": 0.0
        },
        "max_distance_from_gateway_while_inside_m": 2.794095,
        "min_distance_to_gateway_m": 0.0985,
        "min_distance_to_interior_target_m": 0.153069,
        "required_dwell_sec": 3.0,
        "required_inside_sample_count": 8,
        "room_id": "room_8",
        "route_topology_includes_room": true,
        "trajectory_entered_room_mask": true,
        "visual_through_room_success": true
      }
    },
    "through_rooms_requested": [
      "room_8"
    ],
    "trajectory_entered_room15_mask": false,
    "trajectory_sample_count": 359,
    "version": "v0_2",
    "visual_through_room15_success": true
  },
  "wall": {
    "active_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml",
    "artifact_type": "step30s7_trajectory_wall_crossing_validation",
    "boundary_tolerance_not_used_for_pass": true,
    "boundary_tolerance_point_violations_h8r2": 0,
    "boundary_tolerance_point_violations_step30s7": 0,
    "boundary_tolerance_segment_violations_h8r2": 0,
    "boundary_tolerance_segment_violations_step30s7": 0,
    "created_utc": "2026-05-14T07:16:12.031596+00:00",
    "final_wall_validation_passed": true,
    "h8r2_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
    "max_wall_penetration_or_occupied_crossing": 0,
    "point_violations": [],
    "real_point_violations_h8r2": 0,
    "real_point_violations_step30s7": 0,
    "real_segment_violations_h8r2": 0,
    "real_segment_violations_step30s7": 0,
    "step30s7_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/step30s7_request_aware_nav_map.yaml",
    "structural_wall_boundary_count": 0,
    "structural_wall_overlap_count": 0,
    "suspicious_wall_crossing_segments": [],
    "trajectory_sample_count": 359,
    "trajectory_wall_point_violation_count": 0,
    "trajectory_wall_segment_violation_count": 0,
    "version": "v0_2",
    "visualization_artifact_possible": false,
    "visualization_interpolation_artifact_possible": false,
    "wall_crossing_validation_passed": true,
    "wall_point_violations_h8r2": 0,
    "wall_point_violations_step30s7": 0,
    "wall_segment_violations_h8r2": 0,
    "wall_segment_violations_step30s7": 0
  }
}
```
</details>

Keeping Gazebo/RViz runtime open for 60s before this command exits.
