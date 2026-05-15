# Stage1 Stable Map GUI Report

Succeeded: `True`
Failure reason: `None`

Accepted historical wording remains: `Step30P1 repaired execution succeeded with clean forward-only fallback.`

## Required Answers

- Why did room15 appear only partially white/free in RViz? `The old request-aware/H8R2-derived primary /map could omit room15 free space when room15 was not in the request. The repaired primary /map is the stable full-scene occupancy map.`
- Was this a Nav2 map/free-space issue, semantic display issue, or both? `primary Nav2/RViz base map source issue; semantic overlay remains a separate marker layer`
- What is the room15 semantic mask area? `None`
- What fraction of room15 was free in original H8R2 Nav2 map? `None`
- Was the stable full-scene map used as the primary map? `True`
- Was the RViz semantic room mask overlay displayed separately from Nav2 free space? `True`
- What room15 interior target was selected? `None`
- How far is the room15 interior target from r7-r15 gateway? `None`
- How far is it from walls/occupied cells? `None`
- Did Gazebo GUI launch? `True`
- Did RViz launch? `True`
- Did Nav2 lifecycle readiness pass? `True`
- What lifecycle states were observed? `{'/map_server': 'active', '/controller_server': 'active', '/planner_server': 'active', '/bt_navigator': 'active'}`
- Was a TRANSIENT_LOCAL /map OccupancyGrid received? `True`
- Was /follow_path ready? `True`
- Is /compute_path_to_pose a hard blocker? `False`
- Is /navigate_to_pose a hard blocker? `False`
- Which RViz config was used? `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/00824_stage1_step30p1_bev_semantic_route_demo.rviz`
- Was a BEV/floorplan visible in RViz? `True`
- Were route/gateway/room/topology overlay markers published? `True`
- Were robot TF, map, route and trajectory visible or available in RViz topics? `True`
- Did the robot start from room_1 / route start? `True`
- What route was planned? `['room_1', 'room_3', 'room_8', 'room_11', 'room_7', 'room_14', 'room_16']`
- Did the route include room_15 topologically? `False`
- Did the trajectory physically enter room_15 mask? `False`
- How many trajectory samples were inside room_15? `0`
- What was room15 dwell time? `0.0`
- Did it only pass near the room15 gateway? `entered_room15_interior`
- Was spinning/looping near room15 detected? `False`
- Was spinning/looping reduced or eliminated? `True`
- Trajectory wall point violation count? `0`
- Trajectory wall segment violation count? `0`
- Was the previous wall crossing appearance artifact or real? `no_wall_crossing_detected`
- Did the robot reach room_16? `True`
- Was the final pose inside room16 mask? `True`
- Was the final pose too close to r14-r16 gateway? `False`
- Did room16 terminal visual quality pass? `True`
- Did sparse fallback occur? `False`
- Were bridge/smooth bridge waypoints incorrectly used as NavigateToPose goals? `False`

## Evidence

- `report_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/stable_map_gui_report_v0_1.json`
- `report_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/stable_map_gui_report_v0_1.md`
- `command_transcript`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/command_transcript_v0_1.md`
- `lifecycle_readiness_report_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/lifecycle_readiness_report_v0_1.json`
- `lifecycle_readiness_report_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/lifecycle_readiness_report_v0_1.md`
- `dataplane_probe_result_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/dataplane_probe_result_v0_1.json`
- `dataplane_probe_result_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/dataplane_probe_result_v0_1.md`
- `route_query_result_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/route_query_result_v0_1.json`
- `route_query_result_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/route_query_result_v0_1.md`
- `route_execution_result_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/route_execution_result_v0_1.json`
- `route_execution_result_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/route_execution_result_v0_1.md`
- `through_room15_validation_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/through_room15_physical_visit_validation_v0_1.json`
- `through_room15_validation_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/through_room15_physical_visit_validation_v0_1.md`
- `room16_terminal_validation_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/room16_terminal_quality_validation_v0_1.json`
- `room16_terminal_validation_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/room16_terminal_quality_validation_v0_1.md`
- `trajectory_wall_crossing_validation_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/trajectory_wall_crossing_validation_v0_2.json`
- `trajectory_wall_crossing_validation_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/trajectory_wall_crossing_validation_v0_2.md`
- `local_looping_spin_validation_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/local_looping_spin_validation_v0_2.json`
- `local_looping_spin_validation_md`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/local_looping_spin_validation_v0_2.md`
- `room15_map_coverage_audit_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/bev_visual_regression_report_v0_1.json`
- `room15_target_selection_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/room15_target_selection_v0_1.json`
- `rviz_overlay_manifest_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/rviz_overlay_manifest_v0_3.json`
- `rviz_process_check_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/rviz_process_check_v0_1.json`
- `gazebo_process_check_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/gazebo_process_check_v0_1.json`
- `trajectory_sample_result_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/trajectory_sample_result_v0_1.json`
- `latest_follow_path_slice_json`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/latest_follow_path_slice_v0_1.json`
- `process_list_after_bringup`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/process_list_after_bringup_v0_1.txt`
- `process_list_after_route`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/process_list_after_route_v0_1.txt`
- `rviz_config_used`: `/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/current_validation/room8_manual_watch_20260515_093213/00824_stage1_step30p1_bev_semantic_route_demo.rviz`

## Reproduce

`tools/stage1_nav/run_gui_demo.sh --stage-output-dir stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_15 --terminal-room room_16 --from-start --gui --map-profile stable --through-room-dwell-sec 3.0 --through-room-min-inside-samples 8 --keep-gui-open-sec 20`
