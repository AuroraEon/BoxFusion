# 00843 / 00824 String Leakage Scan

Scanned root: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V`

Total hits: `88`
Files with hits: `25`
Filename hits: `5`

## Classification Counts

- `active runtime leakage`: `76`
- `harmless reference/template`: `5`
- `suspicious but not active`: `4`
- `unknown needs manual inspection`: `3`

## Filename Hits

- `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml`
- `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py`
- `nav2/launch/__pycache__/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.cpython-38.pyc`
- `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf`
- `rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz`

## Hits

- `suspicious but not active` `README.md:3` match=`00824`: This lane repairs 00843 Stage1-to-Nav2 evidence without creating a new StepXX lane and without writing under `runtime_stage1_frozen_evidence`. The accepted 00824 lane was used only as a read-only process reference.
- `suspicious but not active` `README.md:66` match=`/stage1_nav/semantic_overlay_markers`: Bringup succeeded with static `map -> odom`, no AMCL, no manual `cmd_vel`, and the floor_2 occupancy map as `/map`. `map_server`, `controller_server`, `planner_server`, and `bt_navigator` were active; `/map`, `/follow_path`, `/compute_path_to_pose`, and `/navigate_to_pose` were available. Gazebo GUI, RViz, and `/stage1_nav/semantic_overlay_markers` were launched.
- `suspicious but not active` `README.md:93` match=`00824`: - The copied 00824 Nav2 world/config/RViz files are runtime scaffolding only; the active `/map`, route, and validation artifacts are 00843 floor_2 artifacts.
- `active runtime leakage` `rviz/rviz_manifest_v0_1.md:3` match=`00824`: Primary Step30S4 config: `00824_stage1_step30p1_bev_semantic_route_demo.rviz`
- `active runtime leakage` `rviz/rviz_manifest_v0_1.json:2` match=`step30p1`: "artifact_type": "stage1_step30p1_rviz_manifest",
- `active runtime leakage` `rviz/rviz_manifest_v0_1.json:5` match=`00824`: "00824_stage1_step30p1_bev_semantic_route_demo.rviz"
- `active runtime leakage` `rviz/rviz_manifest_v0_1.json:7` match=`00824`: "primary_config": "00824_stage1_step30p1_bev_semantic_route_demo.rviz",
- `active runtime leakage` `rviz/rviz_manifest_v0_3.md:3` match=`00824`: Primary Stage1 config: `00824_stage1_step30p1_bev_semantic_route_demo.rviz`
- `active runtime leakage` `rviz/rviz_manifest_v0_3.md:5` match=`/step30s7_rviz_overlay_markers`: Primary marker topic: `/step30s7_rviz_overlay_markers`
- `active runtime leakage` `rviz/rviz_manifest_v0_3.md:20` match=`/step30s7_rviz_overlay_markers`: - `/step30s7_rviz_overlay_markers`
- `active runtime leakage` `rviz/rviz_manifest_v0_2.json:2` match=`step30p1`: "artifact_type": "stage1_step30p1_rviz_manifest",
- `active runtime leakage` `rviz/rviz_manifest_v0_2.json:15` match=`00824`: "primary_config": "00824_stage1_step30p1_bev_semantic_route_demo.rviz",
- `active runtime leakage` `rviz/rviz_manifest_v0_2.json:18` match=`00824`: "00824_stage1_step30p1_bev_semantic_route_demo.rviz"
- `active runtime leakage` `rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz` match=`filename`: rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz
- `active runtime leakage` `rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz:57` match=`/stage1_nav/semantic_overlay_markers`: Value: /stage1_nav/semantic_overlay_markers
- `active runtime leakage` `rviz/rviz_manifest_v0_3.json:2` match=`step30p1`: "artifact_type": "stage1_step30p1_rviz_manifest",
- `active runtime leakage` `rviz/rviz_manifest_v0_3.json:4` match=`00824`: "primary_config": "00824_stage1_step30p1_bev_semantic_route_demo.rviz",
- `active runtime leakage` `rviz/rviz_manifest_v0_3.json:5` match=`/step30s7_rviz_overlay_markers`: "primary_marker_topic": "/step30s7_rviz_overlay_markers",
- `active runtime leakage` `rviz/rviz_manifest_v0_2.md:3` match=`00824`: Primary Step30S5 config: `00824_stage1_step30p1_bev_semantic_route_demo.rviz`
- `suspicious but not active` `routes/route_candidate_report.json:432` match=`00824`: "uses_00824_truth": false,
- `harmless reference/template` `validation/map_projection_failure_report.json:5` match=`00824`: "no_00824_geometry": true,
- `harmless reference/template` `validation/map_projection_failure_report.md:11` match=`00824`: No AMCL, manual cmd_vel, object-level navigation claim, previous invalid 00843 route, or 00824 geometry is used.
- `harmless reference/template` `validation/failure_report.json:5` match=`00824`: "no_00824_geometry": true,
- `active runtime leakage` `nav2/ros_asset_manifest_v0_1.json:2` match=`step30p1`: "artifact_type": "stage1_step30p1_ros_asset_manifest",
- `active runtime leakage` `nav2/ros_asset_manifest_v0_1.json:3` match=`00824`: "config": "nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml",
- `active runtime leakage` `nav2/ros_asset_manifest_v0_1.json:5` match=`00824`: "launch": "nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py",
- `active runtime leakage` `nav2/ros_asset_manifest_v0_1.json:7` match=`h8r2_gateway_preserving`: "map": "maps/h8r2_gateway_preserving_nav_map.yaml",
- `active runtime leakage` `nav2/ros_asset_manifest_v0_1.json:9` match=`00824`: "world": "nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf"
- `harmless reference/template` `current_validation/map_projection_failure_report.json:5` match=`00824`: "no_00824_geometry": true,
- `harmless reference/template` `current_validation/map_projection_failure_report.md:11` match=`00824`: No AMCL, manual cmd_vel, object-level navigation claim, previous invalid 00843 route, or 00824 geometry is used.
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf` match=`filename`: nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:3` match=`00824`: <world name="boxfusion_step30p1_00824_large_continuous_floor">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:7` match=`step30p1`: <model name="step30p1_large_continuous_floor">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:45` match=`00824`: <model name="carve_capsule_line_gw_00824_r1_r3_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:62` match=`00824`: <model name="carve_capsule_line_gw_00824_r1_r3_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:79` match=`00824`: <model name="carve_capsule_line_gw_00824_r3_r7_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:96` match=`00824`: <model name="carve_capsule_line_gw_00824_r3_r7_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:113` match=`00824`: <model name="carve_capsule_line_gw_00824_r3_r8_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:130` match=`00824`: <model name="carve_capsule_line_gw_00824_r3_r8_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:147` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r11_02_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:164` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r11_02_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:181` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r14_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:198` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r14_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:215` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r15_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:232` match=`00824`: <model name="carve_capsule_line_gw_00824_r7_r15_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:249` match=`00824`: <model name="carve_capsule_line_gw_00824_r8_r11_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:266` match=`00824`: <model name="carve_capsule_line_gw_00824_r8_r11_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:283` match=`00824`: <model name="carve_capsule_line_gw_00824_r14_r16_01_segment_0">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:300` match=`00824`: <model name="carve_capsule_line_gw_00824_r14_r16_01_segment_1">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:317` match=`00824`: <model name="gateway_center_gw_00824_r14_r16_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:334` match=`00824`: <model name="gateway_center_gw_00824_r1_r3_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:351` match=`00824`: <model name="gateway_center_gw_00824_r3_r7_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:368` match=`00824`: <model name="gateway_center_gw_00824_r3_r8_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:385` match=`00824`: <model name="gateway_center_gw_00824_r7_r11_02">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:402` match=`00824`: <model name="gateway_center_gw_00824_r7_r14_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:419` match=`00824`: <model name="gateway_center_gw_00824_r7_r15_01">
- `active runtime leakage` `nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf:436` match=`00824`: <model name="gateway_center_gw_00824_r8_r11_01">
- `active runtime leakage` `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml` match=`filename`: nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml
- `active runtime leakage` `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml:1` match=`00824`: # BoxFusion Step23J downstream Gazebo/Nav2 projection params for 00824-Dd4bFSTQ8gi.
- `active runtime leakage` `nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml:243` match=`00824`: yaml_filename: "stage_outputs/stage1_00824_step30p1/manifest/provenance_manifest_v0_2.json"
- `active runtime leakage` `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py` match=`filename`: nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py
- `active runtime leakage` `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py:4` match=`step30p1`: scripts/launch_step30p1_staticloc_tf.sh. It deliberately does not launch AMCL.
- `active runtime leakage` `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py:47` match=`00824`: default_value="/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
- `active runtime leakage` `nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py:52` match=`00824`: default_value="/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml",
- `active runtime leakage` `nav2/launch/__pycache__/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.cpython-38.pyc` match=`filename`: nav2/launch/__pycache__/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.cpython-38.pyc
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/rviz_overlay_publisher.log:1` match=`/stage1_nav/semantic_overlay_markers`: [INFO] [1778940217.249507953] [boxfusion_stage1_nav_overlay_publisher]: Stage1 overlay run_id=00843_floor2_nav2_room11_to_room14_20260516_215657 marker_count=35 topic=/stage1_nav/semantic_overlay_markers room_stride=8
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:1` match=`00824`: [stage1_step30p1] starting Gazebo world: /home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:2` match=`step30p1`: [stage1_step30p1] starting robot_state_publisher
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:3` match=`step30p1`: [stage1_step30p1] spawning TurtleBot3 burger
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:4` match=`step30p1`: [stage1_step30p1] starting static map->odom TF
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:5` match=`step30p1`: [stage1_step30p1] starting Nav2 staticloc stack
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:6` match=`step30p1`: [stage1_step30p1] waiting for Nav2 lifecycle/map/FollowPath readiness
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_command.log:7` match=`step30p1`: [stage1_step30p1] launched. Logs: /home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_logs
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/rviz_overlay_process_check.json:4` match=`/stage1_nav/semantic_overlay_markers`: "marker_topic": "/stage1_nav/semantic_overlay_markers",
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/rviz_overlay_process_check.json:7` match=`00824`: "rviz_config": "stage_outputs/stage1_generalization/00843-DYehNKdT76V/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz",
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:66` match=`00824`: 26508    16 Ssl  ros2            /usr/bin/python3 /opt/ros/foxy/bin/ros2 launch gazebo_ros gazebo.launch.py world:=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf gui:=true
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:67` match=`00824`: 26534 26508 S    sh              /bin/sh -c gzserver /home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf                                                                      -s libgazebo_ros_init.so   -s libgazebo_ros_factory.so   -s libgazebo_ros_force_system.so
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:70` match=`00824`: 26539 26534 SLl  gzserver        gzserver /home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:74` match=`00824`: 26829    16 Ssl  ros2            /usr/bin/python3 /opt/ros/foxy/bin/ros2 launch /home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py map:=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/maps/floor_2/stage1_floor_2_occupancy_map.yaml params_file:=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/config/00824_nav2_turtlebot3_step
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:83` match=`00824`: 30133  3046 Ss   bash            /bin/bash -lc set -euo pipefail RUN_ID="$(cat stage_outputs/stage1_generalization/00843-DYehNKdT76V/current_validation/latest_floor2_run_id.txt)" RUN_DIR="$(cat stage_outputs/stage1_generalization/00843-DYehNKdT76V/current_validation/latest_floor2_run_dir.txt)" STAGE="stage_outputs/stage1_generalization/00843-DYehNKdT76V" set +u source /opt/ros/foxy/setup.bash set -u setsid /usr/bin/python3 tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py \   --stage
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:84` match=`step30p1`: 30148 30133 Ssl  python3         /usr/bin/python3 tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py --stage-output-dir stage_outputs/stage1_generalization/00843-DYehNKdT76V --route-query-json stage_outputs/stage1_generalization/00843-DYehNKdT76V/runs/00843_floor2_nav2_room11_to_room14_20260516_215657/floor2_rviz_route_query.json --waypoints-json stage_outputs/stage1_generalization/00843-DYehNKdT76V/runs/00843_floor2_nav2_room11_to_room14_20260516_215657/floor2_rviz_overlay_waypoints.
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/process_snapshot_after_rviz_launch.txt:85` match=`00824`: 30149 30133 Ssl  rviz2           rviz2 -d stage_outputs/stage1_generalization/00843-DYehNKdT76V/rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_logs/bringup_result.json:4` match=`00824`: "world": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/worlds/00824_step30p1_large_continuous_floor_world.sdf",
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_logs/bringup_result.json:7` match=`00824`: "nav2_params": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml",
- `active runtime leakage` `runs/00843_floor2_nav2_room11_to_room14_20260516_215657/bringup_logs/bringup_result.json:8` match=`00824`: "nav2_launch": "/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V/nav2/launch/00824_nav2_turtlebot3_h8r2_staticloc_step30p1.launch.py",
- `unknown needs manual inspection` `stage1_canonical_rerun/scenes/00843-DYehNKdT76V/logs/runtime_instrumentation/per_profiled_frame.csv:6` match=`00824`: 00843-DYehNKdT76V,100,keyframe_segmentation_refresh,True,True,True,False,floor_1,stable,3,True,False,False,scheduled_segmentation_refresh_plus_full_export_rebuild,current_room_state|room_exit_or_completion_signal|incremental_room_buffer|topology_delta_update_queue,0.04121136665344238,0.05583691596984863,0.003992147001554258,0.014974164994782768,0.0005687679949915037,,,0.005850094996276312,0.030431737992330454,,,,,,,0.055816912979935296,2.0002989913336933e-05,0.6129157543182373,0.2282311916351318
- `unknown needs manual inspection` `stage1_canonical_rerun/scenes/00843-DYehNKdT76V/logs/runtime_instrumentation/per_profiled_frame.csv:28` match=`00824`: 00843-DYehNKdT76V,650,keyframe,True,True,False,False,floor_1,stable,3,True,False,False,scheduled_segmentation_refresh_plus_full_export_rebuild,current_room_state|room_exit_or_completion_signal|incremental_room_buffer|topology_delta_update_queue,0.038268327713012695,0.05157113075256348,0.0028658219962380826,0.015184278003289364,0.0006502070027636364,,,0.005995989995426498,0.026857147997361608,,,,,,,0.05155344499507919,1.7685757484287024e-05,0.649850606918335,7.152557373046875e-07,0.00030512499506
- `unknown needs manual inspection` `stage1_canonical_rerun/scenes/00843-DYehNKdT76V/logs/runtime_instrumentation/per_profiled_frame.csv:37` match=`00824`: 00843-DYehNKdT76V,875,keyframe,True,True,False,False,floor_1,stable,2,True,False,False,scheduled_segmentation_refresh_plus_full_export_rebuild,current_room_state|room_exit_or_completion_signal|incremental_room_buffer|topology_delta_update_queue,0.0388944149017334,0.05290412902832031,0.002398365002591163,0.015716811001766473,0.0008240220049628988,,,0.005937511014053598,0.0280075779883191,,,,,,,0.05288428701169323,1.984201662708074e-05,0.6554019451141357,7.152557373046875e-07,0.0003427670017117634

## Interpretation

The copied 00824 RViz, Nav2 launch/config, and Gazebo world files are active runtime leakage for the rejected 00843 run evidence. References in failure reports or rejection notes are harmless when they explicitly document rejected evidence or truth boundaries.
