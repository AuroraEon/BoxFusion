# Scripted Stair Transition Gazebo/RViz Demo

Task60 adds a stable RSLG-SLAM cross-floor demo for `00843_cross_floor_object_curtain_room14`.

Task59 used a ramp surrogate for `vt_1_centerline_e001`, but the visual world was too abstract for the intended stair-transition demo and the headless Gazebo run timed out on the ramp. Task60 changes the executor boundary: the semantic connector is represented as scripted stair-transition animation while floor_1 and floor_2 remain flat Gazebo PID segments.

Pipeline:

1. Build the scripted stair runtime input:
   `tools/rslg_pipeline/gazebo/build_scripted_stair_transition_runtime_input.py`
2. Start the scripted stair Gazebo world:
   `tools/rslg_pipeline/gazebo/worlds/rslg_scripted_stair_transition_turtlebot3_burger.world`
3. Run the segmented executor:
   `tools/rslg_pipeline/gazebo/rslg_gazebo_scripted_stair_demo_orchestrator.py`
4. View composite planned/executed paths in RViz:
   `tools/rslg_pipeline/rviz/config/rslg_scripted_stair_transition_showcase.rviz`

Important distinction:

- Same-floor Gazebo PID: the TurtleBot3 follows XY/yaw waypoints by publishing `/cmd_vel` and reading `/odom`.
- Scripted stair transition animation: the executor publishes RViz path/pose evidence and, when available, uses `/set_entity_state` to move the Gazebo model through stair keyframes.
- Physical stair climbing: out of scope for this demo.

Transition identity:

- `vt_1_centerline_e001` is the scripted transition edge.
- `vt_1_centerline_e003` is forbidden/non-transition evidence only.
- `generated_ring_002` is the valid final curtain approach.
- `generated_ring_037` remains blocked/rejected evidence only.

Run dry-run:

```bash
tools/rslg_pipeline/gazebo/run_scripted_stair_transition_demo.sh --dry-run --query-id 00843_cross_floor_object_curtain_room14 --rviz-only-stair-fallback
```

Run headless/service demo:

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11361 ROS_DOMAIN_ID=61 \
tools/rslg_pipeline/gazebo/run_scripted_stair_transition_demo.sh \
  --headless \
  --no-rviz-gui \
  --duration-sec 360 \
  --query-id 00843_cross_floor_object_curtain_room14 \
  --rviz-only-stair-fallback
```

RViz topics:

- `/rslg/scripted_stair/composite_planned_path_odom`
- `/rslg/scripted_stair/composite_executed_path`
- `/rslg/scripted_stair/robot_pose`
- `/rslg/scripted_stair/stair_transition_path`
- `/rslg/scripted_stair/marker_array`
- fixed frame: `odom`

Odom anchoring:

The executor can anchor the first route waypoint to the first received odom pose and applies that same transform to the full composite path. This makes the planned path, executed trace, stair keyframes, and robot pose agree in RViz even when Gazebo odom origin differs from route coordinates.

Segment status:

The executor writes a JSON summary with `floor_1_status`, `stair_transition_status`, `floor_2_status`, `final_target_status`, `odom_received`, `cmd_vel_published`, service availability, timeout, and final error.

Out of scope:

- physical stair climbing
- real robot deployment
- Unitree stair gait control
- Nav2
- AMCL
- map_server
- global collision-free guarantee
