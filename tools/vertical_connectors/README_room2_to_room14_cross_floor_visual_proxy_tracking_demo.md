# Room 2 To Room 14 Cross-Floor Visual Proxy Tracking Demo

This is the task24h opt-in tracking-mode demo for the RSLG-SLAM downstream visual traversal path.

It loads the task24g2 planned occupancy-aware 3D route:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24g2_occupancy_aware_cross_floor_visual_proxy_traversal/planned_occupancy_aware_3d_route_v0_1.json
```

The route remains:

```text
room_2 -> room_3 -> stair connector vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14
```

Same-floor portions track A* paths over stable occupancy maps. The stair portion tracks the graph-level `vt_1` 3D centerline with 2.5D z integration. The transition edge remains `vt_1_centerline_e001` from `vt_1_centerline_n001` on floor_1 to `vt_1_centerline_n002` on floor_2.

Default run:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh
```

Recommended human-viewing command:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --keep-open --realtime-scale 2.0
```

The wrapper also accepts the equivalent manual preset:

```bash
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --manual-demo
```

Fast validation mode:

```bash
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --no-rviz --hold-final 0 --realtime-scale 0.0
```

Fast validation is intended for automated evidence refreshes and JSON checks, not for human viewing. It runs as quickly as the player can publish and does not leave time to inspect the GUI.

Useful options:

```bash
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --no-rviz
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --no-gazebo
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --keep-open
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --demo-speed slow
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --demo-speed normal
tools/vertical_connectors/run_00843_room2_to_room14_cross_floor_visual_proxy_tracking_demo.sh --demo-speed fast
```

Demo lineage and tracking behavior:

- task24g2 is the occupancy-aware A* cross-floor visual proxy traversal that directly displays planned route poses with SetEntityState.
- task24h reuses the task24g2 A* route, computes a cmd_vel-like internal tracking command, integrates an actual pose from those commands, and uses Gazebo SetEntityState only as a visual display of that integrated actual pose.
- Planned route samples are therefore not directly replayed into Gazebo by task24h.
- RViz publishes planned and executed trajectories separately.

The planned and executed trajectories should be close because the controller is intentionally tracking the task24g2 route and the route is collision-aware on same-floor segments. They should not be identical because the executed trajectory comes from integrated velocity commands with finite controller gains, speed limits, lookahead behavior, final pose tolerance, and the 2.5D stair z tracking law.

This is not physical stair climbing. The stair connector is represented as a topological vertical transition over the `vt_1` 3D centerline, and task24h visualizes a kinematic proxy following that connector. There is no gait generation, no footstep planning, no contact dynamics, and no validation of real quadruped stair locomotion.

RViz topics:

- `/rslg/cross_floor_tracking_planned_route`
- `/rslg/cross_floor_tracking_executed_trajectory`
- `/rslg/cross_floor_tracking_error_markers`
- `/rslg/cross_floor_tracking_checkpoints`
- `/rslg/cross_floor_tracking_claim_boundary`

Outputs are written to:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24h_cross_floor_visual_proxy_tracking_mode/
```

Claim boundary:

- `visual_kinematic_proxy_only`
- `occupancy_aware_same_floor_tracking=true`
- `stair_connector_2p5d_tracking=true`
- `topological_vertical_transition_only`
- `physical_stair_climbing_supported=false`
- no gait
- no footstep planning
- no contact-based stair climbing
- not real quadruped stair locomotion
- not Nav2 execution
- not AMCL/localization

This task does not modify task24g2 direct playback artifacts or stable maps.
