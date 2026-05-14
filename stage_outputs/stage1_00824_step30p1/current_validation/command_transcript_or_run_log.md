# Stable Map Repair Command Transcript

Exact commands run manually for the integrated repair and validation:

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/stop.sh --stage-output-dir stage_outputs/stage1_00824_step30p1
```

```bash
ps -eo pid,ppid,stat,comm,args | grep -E 'gzclient|gzserver|gazebo|rviz2|nav2_|bt_navigator|controller_server|planner_server|map_server|publish_stage1_step30p1_rviz_overlay|robot_state_publisher|static_transform_publisher|turtlebot3' | grep -v grep || true
```

```bash
/usr/bin/python3 tools/stage1_nav/build_stable_map.py --stage-output-dir stage_outputs/stage1_00824_step30p1 --output-json stage_outputs/stage1_00824_step30p1/current_validation/stable_full_scene_occupancy_map_provenance.json --output-md stage_outputs/stage1_00824_step30p1/current_validation/stable_full_scene_occupancy_map_provenance.md
```

```bash
/usr/bin/python3 -m py_compile tools/stage1_step30p1/build_stage1_full_scene_occupancy_map.py tools/stage1_step30p1/audit_stage1_stable_floorplan_consistency.py tools/stage1_nav/write_current_status_report.py tools/stage1_step30p1/prepare_stage1_step30p1_semantic_route.py tools/stage1_step30p1/validate_stage1_step30p1_physical.py tools/stage1_nav/build_stable_map.py tools/stage1_nav/verify_current_artifact.py
```

```bash
bash -n tools/stage1_nav/run_gui_demo.sh tools/stage1_step30p1/run_stage1_step30p1_end_to_end.sh tools/stage1_step30p1/launch_stage1_step30p1_gazebo_nav2.sh tools/stage1_nav/stop.sh
```

```bash
/usr/bin/python3 tools/stage1_nav/verify_current_artifact.py --stage-output-dir stage_outputs/stage1_00824_step30p1 --output-json stage_outputs/stage1_00824_step30p1/current_validation/artifact_verification_after_stable_map.json
```

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/run_gui_demo.sh --stage-output-dir stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_8 --terminal-room room_16 --from-start --gui --map-profile stable --keep-gui-open-sec 5 --run-id room8_stable_map_gui_check
```

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/stop.sh --stage-output-dir stage_outputs/stage1_00824_step30p1
```

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/run_gui_demo.sh --stage-output-dir stage_outputs/stage1_00824_step30p1 --start-room room_1 --goal-room room_16 --through-rooms room_15 --terminal-room room_16 --from-start --gui --map-profile stable --through-room-dwell-sec 3.0 --through-room-min-inside-samples 8 --keep-gui-open-sec 5 --run-id room15_stable_map_gui_check
```

```bash
ROS_DOMAIN_ID=84 tools/stage1_nav/stop.sh --stage-output-dir stage_outputs/stage1_00824_step30p1
```

```bash
/usr/bin/python3 tools/stage1_step30p1/audit_stage1_stable_floorplan_consistency.py --stage-output-dir stage_outputs/stage1_00824_step30p1 --room8-run-id room8_stable_map_gui_check --room15-run-id room15_stable_map_gui_check
```

```bash
/usr/bin/python3 tools/stage1_nav/write_current_status_report.py --stage-output-dir stage_outputs/stage1_00824_step30p1 --room8-run-id room8_stable_map_gui_check --room15-run-id room15_stable_map_gui_check
```

```bash
rg -n 'Loading yaml file|Loading image_file|stage1_full_scene|step30s7_request' stage_outputs/stage1_00824_step30p1/current_validation/room8_stable_map_gui_check/bringup_logs/nav2.log stage_outputs/stage1_00824_step30p1/current_validation/room15_stable_map_gui_check/bringup_logs/nav2.log
```

Per-run transcripts are also stored at:

- `stage_outputs/stage1_00824_step30p1/current_validation/room8_stable_map_gui_check/command_transcript_v0_1.md`
- `stage_outputs/stage1_00824_step30p1/current_validation/room15_stable_map_gui_check/command_transcript_v0_1.md`
