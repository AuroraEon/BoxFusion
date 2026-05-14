# Stage1 Stable Full-Scene Map GUI Command Transcript

Started: `2026-05-14T11:42:30Z`
Run id: `room8_manual_stable_check`
Stage output: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1`
Evidence: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check`

## build_stable_full_scene_map

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/build_stage1_full_scene_occupancy_map.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/stable_full_scene_occupancy_map_provenance.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/stable_full_scene_occupancy_map_provenance.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/build_stable_full_scene_map.log`

<details><summary>tail</summary>

```text
      "semantic_mask_area_cells": 4723,
      "stable_free_cells": 4711,
      "stable_free_fraction": 0.997459,
      "stable_occupied_cells": 12,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 4723,
      "wall_evidence_cells": 12
    },
    "room_15": {
      "component_sizes_desc": [
        1295
      ],
      "connected_component_count": 1,
      "gateway_carve_cells": 89,
      "largest_component_cells": 1295,
      "semantic_mask_area_cells": 1324,
      "stable_free_cells": 1295,
      "stable_free_fraction": 0.978097,
      "stable_occupied_cells": 29,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 1324,
      "wall_evidence_cells": 37
    },
    "room_16": {
      "component_sizes_desc": [
        1325,
        231
      ],
      "connected_component_count": 2,
      "gateway_carve_cells": 101,
      "largest_component_cells": 1325,
      "semantic_mask_area_cells": 1584,
      "stable_free_cells": 1556,
      "stable_free_fraction": 0.982323,
      "stable_occupied_cells": 28,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 1584,
      "wall_evidence_cells": 28
    },
    "room_3": {
      "component_sizes_desc": [
        19873,
        224,
        222,
        7
      ],
      "connected_component_count": 4,
      "gateway_carve_cells": 247,
      "largest_component_cells": 19873,
      "semantic_mask_area_cells": 20442,
      "stable_free_cells": 20326,
      "stable_free_fraction": 0.994325,
      "stable_occupied_cells": 116,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 20442,
      "wall_evidence_cells": 116
    },
    "room_7": {
      "component_sizes_desc": [
        4684,
        316
      ],
      "connected_component_count": 2,
      "gateway_carve_cells": 378,
      "largest_component_cells": 4684,
      "semantic_mask_area_cells": 5015,
      "stable_free_cells": 5000,
      "stable_free_fraction": 0.997009,
      "stable_occupied_cells": 15,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 5015,
      "wall_evidence_cells": 16
    },
    "room_8": {
      "component_sizes_desc": [
        5574,
        38,
        24
      ],
      "connected_component_count": 3,
      "gateway_carve_cells": 203,
      "largest_component_cells": 5574,
      "semantic_mask_area_cells": 5716,
      "stable_free_cells": 5636,
      "stable_free_fraction": 0.986004,
      "stable_occupied_cells": 80,
      "stable_unknown_cells": 0,
      "stage1_free_space_cells": 5716,
      "wall_evidence_cells": 85
    }
  },
  "route_room_ids_whitelist_used": false,
  "source_inputs": {
    "final_gateway_wall_preclose_thr_0p25_png": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/final_gateway_wall_preclose_thr_0p25.png",
    "free_space_png": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/free_space.png",
    "gateway_projection_config": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/config/step30s7_gateway_projection_config_v0_1.json",
    "gateway_registry": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/gateway/gateway_registry_v0_1.json",
    "h8r2_masks_npz_for_gateway_carves_only": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_masks.npz",
    "h8r2_reference_map_yaml_for_metadata_and_baseline": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
    "outside_boundary_png": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/visualizations/outside_boundary.png",
    "stage1_layered_bev_npz": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz",
    "stage1_room_mask": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy"
  },
  "source_inputs_exist": {
    "final_gateway_wall_preclose_thr_0p25_png": true,
    "free_space_png": true,
    "gateway_projection_config": true,
    "gateway_registry": true,
    "h8r2_masks_npz_for_gateway_carves_only": true,
    "h8r2_reference_map_yaml_for_metadata_and_baseline": true,
    "outside_boundary_png": true,
    "stage1_layered_bev_npz": true,
    "stage1_room_mask": true
  },
  "stage_output_dir": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1",
  "step30s5_room15_patch_used": false,
  "through_rooms_used": false,
  "validation_passed": true,
  "version": "v0_1"
}
```
</details>

## gateway_generalization_readiness

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/write_stage1_step30p1_gateway_readiness_report.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/gateway_generalization_readiness_report_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/gateway_generalization_readiness_report_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/gateway_generalization_readiness.log`

<details><summary>tail</summary>

```text
{
  "active_route_projection_code_hard_codes_room15": false,
  "active_route_projection_code_hard_codes_route_room_ids": false,
  "artifact_type": "step30s7_gateway_generalization_readiness_report",
  "created_utc": "2026-05-14T11:42:31.710689+00:00",
  "externalized": {
    "forbidden_gateway_pairs": [
      "r14_r15",
      "r15_r16",
      "r3_r11",
      "r3_r15",
      "r8_r14",
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
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/prepare_stage1_step30p1_semantic_route.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 --map-profile stage1_full_scene_occupancy --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_query_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_query_result_v0_1.md --waypoints-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/semantic_route_waypoints_v0_1.json --target-selection-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/room15_target_selection_v0_1.json --target-selection-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/room15_target_selection_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_query.log`

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
  "map_profile": "stage1_full_scene_occupancy",
  "map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml",
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
  "waypoints_path": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/semantic_route_waypoints_v0_1.json"
}
```
</details>

## clean_old_processes

```bash
/home/ws/workspace/BoxFusion/tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --log-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/bringup_logs --stop 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/clean_old_processes.log`

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
/home/ws/workspace/BoxFusion/tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --ros-domain-id 84 --gui --map-profile stage1_full_scene_occupancy --log-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/bringup_logs --spawn-x 5.85 --spawn-y 2.1 --spawn-yaw 3.141593 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/bringup.log`

<details><summary>tail</summary>

```text
[stage1_step30p1] starting Gazebo world: /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf
[stage1_step30p1] starting robot_state_publisher
[stage1_step30p1] spawning TurtleBot3 burger
[stage1_step30p1] starting static map->odom TF
[stage1_step30p1] starting Nav2 staticloc stack
[stage1_step30p1] launched. Logs: /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/bringup_logs
gazebo: running pid=10447
robot_state_publisher: running pid=10695
static_tf: running pid=10744
nav2: running pid=10746
```
</details>

## dataplane_probe

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/probe_stage1_step30p1_dataplane.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --timeout-sec 25 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/dataplane_probe_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/dataplane_probe_result_v0_1.md 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/dataplane_probe.log`

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
      "observed_rate_hz": 9.97,
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
      "message_count_observed": 735,
      "message_received": true,
      "observed_rate_hz": 29.379,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 2
    },
    "/scan": {
      "message_count_observed": 125,
      "message_received": true,
      "observed_rate_hz": 4.993,
      "probe_window_sec": 25.0,
      "publisher_count": 1,
      "subscription_count": 4
    },
    "/tf": {
      "message_count_observed": 1221,
      "message_received": true,
      "observed_rate_hz": 48.833,
      "probe_window_sec": 25.0,
      "publisher_count": 2,
      "subscription_count": 7
    },
    "/tf_static": {
      "message_count_observed": 503,
      "message_received": true,
      "observed_rate_hz": 20.075,
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
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/run_stage1_step30p1_route.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --waypoints-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/semantic_route_waypoints_v0_1.json --expected-room-chain room_1\,room_3\,room_8\,room_11\,room_7\,room_14\,room_16 --expected-gateway-sequence gw_00824_r1_r3_01\,gw_00824_r3_r8_01\,gw_00824_r8_r11_01\,gw_00824_r7_r11_02\,gw_00824_r7_r14_01\,gw_00824_r14_r16_01 --allow-non-step30p1-truth --from-start --reset-to-route-start --follow-path-timeout-sec 480 --goal-timeout-sec 180 --split-dwell-source room8_interior_terminal --split-dwell-sec 3.0 --output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_execution_result_v0_1.json --output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_execution_result_v0_1.md --trajectory-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/trajectory_sample_result_v0_1.json --latest-slice-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/latest_follow_path_slice_v0_1.json 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_execution.log`

<details><summary>tail</summary>

```text
      "x": -4.05007923422477,
      "y": -3.1172399140626053,
      "yaw": 1.3262450123647689,
      "z": 0.008731069906242698
    },
    {
      "frame_id": "map",
      "sample_index": 347,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759176.3609564,
      "x": -4.030331805378739,
      "y": -3.036123460184378,
      "yaw": 1.3531416479833278,
      "z": 0.008730186151798785
    },
    {
      "frame_id": "map",
      "sample_index": 348,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759176.8811066,
      "x": -4.0066887187762585,
      "y": -2.945943508966543,
      "yaw": 1.2930380790284792,
      "z": 0.008730994066348675
    },
    {
      "frame_id": "map",
      "sample_index": 349,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759177.384319,
      "x": -3.973017011902213,
      "y": -2.8694810788760066,
      "yaw": 0.9954921279864242,
      "z": 0.008731092999967398
    },
    {
      "frame_id": "map",
      "sample_index": 350,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759177.9012024,
      "x": -3.914535931403454,
      "y": -2.8065228852750854,
      "yaw": 0.6517002375747938,
      "z": 0.008722524221752641
    },
    {
      "frame_id": "map",
      "sample_index": 351,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759178.4108293,
      "x": -3.8437442296050173,
      "y": -2.7672906366529326,
      "yaw": 0.38732577766516246,
      "z": 0.008715376737913064
    },
    {
      "frame_id": "map",
      "sample_index": 352,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759178.9221437,
      "x": -3.7603887907480265,
      "y": -2.742867397098598,
      "yaw": 0.20178943752179426,
      "z": 0.008729130372559409
    },
    {
      "frame_id": "map",
      "sample_index": 353,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759179.433632,
      "x": -3.6722079962529888,
      "y": -2.733039425851144,
      "yaw": 0.030648994684203006,
      "z": 0.008725889836935992
    },
    {
      "frame_id": "map",
      "sample_index": 354,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759179.943909,
      "x": -3.5897495290175314,
      "y": -2.7354342408960677,
      "yaw": -0.07154635298157347,
      "z": 0.008729664308928742
    },
    {
      "frame_id": "map",
      "sample_index": 355,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759180.4542475,
      "x": -3.5015649018653128,
      "y": -2.7431712889323823,
      "yaw": -0.10141512917930112,
      "z": 0.008711329991192772
    },
    {
      "frame_id": "map",
      "sample_index": 356,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759180.961276,
      "x": -3.4244277247951684,
      "y": -2.7518680417420556,
      "yaw": -0.11954433446292546,
      "z": 0.008730945852025414
    },
    {
      "frame_id": "map",
      "sample_index": 357,
      "source": "/tf map->base_footprint",
      "time_wall_sec": 1778759181.4768677,
      "x": -3.4065374610600547,
      "y": -2.754015511180483,
      "yaw": -0.21463127814197644,
      "z": 0.00873106672696086
    }
  ],
  "version": "v0_1",
  "waypoint_results": [],
  "waypoints_source": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/semantic_route_waypoints_v0_1.json"
}
```
</details>

## physical_validation

```bash
/usr/bin/python3 /home/ws/workspace/BoxFusion/tools/stage1_step30p1/validate_stage1_step30p1_physical.py --stage-output-dir /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1 --route-query-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_query_result_v0_1.json --waypoints-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/semantic_route_waypoints_v0_1.json --route-execution-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/route_execution_result_v0_1.json --trajectory-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/trajectory_sample_result_v0_1.json --through-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/through_room15_physical_visit_validation_v0_1.json --through-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/through_room15_physical_visit_validation_v0_1.md --terminal-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/room16_terminal_quality_validation_v0_1.json --terminal-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/room16_terminal_quality_validation_v0_1.md --wall-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/trajectory_wall_crossing_validation_v0_2.json --wall-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/trajectory_wall_crossing_validation_v0_2.md --spin-output-json /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/local_looping_spin_validation_v0_2.json --spin-output-md /home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/local_looping_spin_validation_v0_2.md --room15-min-inside-samples 8 --through-room-dwell-sec 3.0 
```

Exit code: `0`
Log: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_stable_check/physical_validation.log`

<details><summary>tail</summary>

```text
    "local_loop_count_around_gateway": 0,
    "local_looping_validation_passed": true,
    "near_gateway_interval_count": 13,
    "near_stationary_high_yaw_change_interval_count": 0,
    "repeated_revisit_cell_count_near_gateway": 0,
    "spinning_detected": false,
    "total_angular_travel_near_gateway_rad": 1.373219,
    "version": "v0_1"
  },
  "terminal": {
    "artifact_type": "stage1_room16_terminal_quality_validation",
    "created_utc": "2026-05-14T11:46:22.162542+00:00",
    "final_arrival_action_success": true,
    "final_pose": {
      "frame_id": "map",
      "source": "/tf map->base_footprint",
      "x": -3.4066468053905017,
      "y": -2.753936191768144,
      "yaw": -0.32168374503482994,
      "z": 0.00870573295861611
    },
    "final_pose_distance_to_r14_r16_gateway_m": 0.922366,
    "final_pose_distance_to_room16_interior_target_m": 0.206684,
    "final_pose_inside_room16_interior_region": true,
    "final_pose_inside_room16_mask": true,
    "final_pose_near_r14_r16_gateway": false,
    "min_distance_to_room16_interior_target": 0.206576,
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
    "created_utc": "2026-05-14T11:46:22.162501+00:00",
    "gateway_only_failure_guard_passed": true,
    "room15_inside_dwell_sec": 0.0,
    "room15_inside_sample_count": 0,
    "route_topology_includes_room15": false,
    "through_room_results": {
      "room_8": {
        "gateway_crossed": true,
        "gateway_only_failure_guard_passed": true,
        "inside_dwell_sec": 33.738,
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
        "max_distance_from_gateway_while_inside_m": 2.764827,
        "min_distance_to_gateway_m": 0.104633,
        "min_distance_to_interior_target_m": 0.182207,
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
    "trajectory_sample_count": 358,
    "version": "v0_2",
    "visual_through_room15_success": true
  },
  "wall": {
    "active_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml",
    "artifact_type": "step30s7_trajectory_wall_crossing_validation",
    "boundary_tolerance_not_used_for_pass": true,
    "boundary_tolerance_point_violations_h8r2": 0,
    "boundary_tolerance_point_violations_step30s7": 0,
    "boundary_tolerance_segment_violations_h8r2": 0,
    "boundary_tolerance_segment_violations_step30s7": 0,
    "created_utc": "2026-05-14T11:46:22.310656+00:00",
    "final_wall_validation_passed": true,
    "h8r2_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
    "max_wall_penetration_or_occupied_crossing": 0,
    "point_violations": [],
    "real_point_violations_h8r2": 0,
    "real_point_violations_step30s7": 0,
    "real_segment_violations_h8r2": 0,
    "real_segment_violations_step30s7": 0,
    "step30s7_map_yaml": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/stage1_full_scene_occupancy_map.yaml",
    "structural_wall_boundary_count": 0,
    "structural_wall_overlap_count": 0,
    "suspicious_wall_crossing_segments": [],
    "trajectory_sample_count": 358,
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

Keeping Gazebo/RViz runtime open for 90s before this command exits.
