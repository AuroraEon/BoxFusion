# RSLG-SLAM Room 2 To Obj175 Cross-Floor Object Tracking Demo

This task24i smoke test extends the task24h visual-kinematic cross-floor chain to an object-level target approach:

`room_2 -> room_3 -> stair connector vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14 -> obj_175 approach generated_ring_037`

The object query is `curtain in room_14 on floor_2`. It resolves to `obj_175` / `curtain` in `room_14` on `floor_2`, and uses the previously audited standoff approach candidate `generated_ring_037`. The object centroid is not used as the navigation target.

## Recommended Manual Demo

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh --manual-demo
```

## Fast Validation Mode

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh --no-rviz --hold-final 0 --realtime-scale 0.0
```

For deterministic dry-run validation without Gazebo:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh --no-gazebo --no-rviz --hold-final 0 --realtime-scale 0.0
```

Pass `--rebuild-route` to regenerate only the task24i object-level route outputs.

## What It Shows

- planned object-level route on `/rslg/cross_floor_object_tracking_planned_route`
- executed integrated tracking trajectory on `/rslg/cross_floor_object_tracking_executed_trajectory`
- semantic checkpoints on `/rslg/cross_floor_object_tracking_checkpoints`
- object target and approach pose on `/rslg/cross_floor_object_tracking_target`
- claim boundary on `/rslg/cross_floor_object_tracking_claim_boundary`

Gazebo uses the distinct entity name `rslg_cross_floor_object_quadruped_proxy_tracking`.

## Boundary

This is `visual_kinematic_proxy_only` and `object_level_smoke_test_only`.

- `occupancy_aware_same_floor_tracking=true`
- `stair_connector_2p5d_tracking=true`
- `topological_vertical_transition_only`
- `physical_stair_climbing_supported=false`
- `object_centroid_navigation_used=false`
- no gait
- no footstep planning
- no contact-based stair climbing
- not real quadruped stair locomotion
- not Nav2 execution
- not AMCL/localization
- not a full object-navigation benchmark

Outputs are written only under:

`stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24i_cross_floor_object_level_tracking_smoke/`
