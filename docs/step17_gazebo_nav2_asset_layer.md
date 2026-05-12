# Step 17 Gazebo/Nav2 Asset Layer

Step 17 prepares the first concrete Gazebo/Nav2 execution path from BoxFusion committed/public artifacts. It creates maps, waypoints, marker worlds, launch templates, an optional dependency installer, and a runbook. It does not execute Gazebo or Nav2 in this environment.

## Environment Feasibility

Safe audit results:

- `ros2`: `/opt/ros/foxy/bin/ros2`
- `rviz2`: `/opt/ros/foxy/bin/rviz2`
- `gazebo`: `None`
- `gzserver`: `None`
- `gzclient`: `None`
- `ROS_DISTRO`: `foxy`
- Matching ROS package probe: `dummy_map_server, geometry_msgs, nav_msgs, robot_state_publisher, tf2_geometry_msgs, tf2_ros, visualization_msgs`

Gazebo executables are not available on PATH, and Nav2/Gazebo packages are not installed locally. Apt candidates exist for ROS Foxy packages in the configured ROS repository. Step 17 did not run `sudo apt install`.

## Generated Scenes

| scene_id | role | floors | candidate edges | robot-enabled edges | marker world |
| --- | --- | ---: | ---: | ---: | --- |
| `00829-QaLdnwvtxbs` | primary_sanity_scene | 1 | 2 | 0 | `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/worlds/00829-QaLdnwvtxbs_marker_world.sdf` |
| `00843-DYehNKdT76V` | primary_paper_scene | 2 | 18 | 0 | `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/worlds/00843-DYehNKdT76V_marker_world.sdf` |

## Approximate Maps

These maps are rasterized from committed room polygons. They are not validated occupancy maps.

| scene_id | floor_id | size | map YAML | status |
| --- | --- | ---: | --- | --- |
| `00829-QaLdnwvtxbs` | `floor_1` | 315x238 | `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/maps/floor_1_approx_nav_map.yaml` | approximate_unvalidated_from_room_polygons |
| `00843-DYehNKdT76V` | `floor_1` | 373x219 | `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_1_approx_nav_map.yaml` | approximate_unvalidated_from_room_polygons |
| `00843-DYehNKdT76V` | `floor_2` | 261x182 | `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_2_approx_nav_map.yaml` | approximate_unvalidated_from_room_polygons |

## Asset Contract

- `robot_enabled_edges` remains empty.
- Same-floor candidate edges remain `robot_candidate_needs_validation`.
- Weak, possible, and unsupported multi-floor edges are excluded from robot execution.
- The SDF world is marker-only and not a validated collision world.
- The Nav2 parameter and launch files are templates for later execution.
- Generated artifacts are downstream-only and leave Stage-A behavior, topology, route planning, and committed exports untouched.

## Output Root

`runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer`

## Key Files

- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/tables/environment_feasibility_audit.json`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/tables/environment_feasibility_audit.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/maps/floor_1_approx_nav_map.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/maps/floor_1_approx_nav_map.pgm`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/worlds/00829-QaLdnwvtxbs_marker_world.sdf`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/waypoints/artifact_route_waypoints.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/waypoints/artifact_route_waypoints.json`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/waypoints/candidate_edges_for_validation.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/nav2/nav2_params_template.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00829-QaLdnwvtxbs/scene_asset_manifest.json`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_1_approx_nav_map.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_2_approx_nav_map.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_1_approx_nav_map.pgm`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/maps/floor_2_approx_nav_map.pgm`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/worlds/00843-DYehNKdT76V_marker_world.sdf`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/waypoints/artifact_route_waypoints.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/waypoints/artifact_route_waypoints.json`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/waypoints/candidate_edges_for_validation.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/nav2/nav2_params_template.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scenes/00843-DYehNKdT76V/scene_asset_manifest.json`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/tables/generated_map_inventory.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/tables/candidate_edge_waypoint_inventory.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/tables/scene_asset_summary.csv`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/package.xml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/setup.py`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/resource/boxfusion_gazebo_nav2_demo`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/boxfusion_gazebo_nav2_demo/__init__.py`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/launch/gazebo_marker_world.launch.py`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/launch/nav2_map_only.launch.py`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/launch/nav2_bringup_template.launch.py`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws/src/boxfusion_gazebo_nav2_demo/params/nav2_params_template.yaml`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scripts/install_foxy_gazebo_nav2_deps.sh`
- `runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/runbook.md`

## Reproduction

```bash
python3 tools/build_step17_gazebo_nav2_asset_layer.py
```

## Claim Boundary

Supported after Step 17: a concrete, reproducible Gazebo/Nav2 preparation layer exists for the 00829 sanity scene and 00843 paper scene.

Not supported after Step 17: Gazebo execution, Nav2 execution, collision-free local planning, local obstacle avoidance, full robot navigation, physical traversability, or promotion of any edge to `robot_enabled`.
