# RSLG-SLAM Current Project Status

Scene: `00843-DYehNKdT76V`

| Layer | Status |
| --- | --- |
| Layer 0: Input Layer | RGB-D images and provided camera poses are the dataset inputs. |
| Layer 1: World Model Layer | Completed. |
| Layer 2: Formal Artifact Layer | Completed; conservative stable maps accepted; object approach recovered with `generated_ring_002`. |
| Layer 3: Navigation Interface Layer | Completed for selected `cross_floor_room` and `cross_floor_object` routes under `conservative_canonical`. |
| Layer 4: Runtime Validation Layer | Room-level and object-level controlled simulation runtime validation completed. |

Selected room route:

`room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14`

Selected object route:

`room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14 -> generated_ring_002 approach candidate for obj_175`

`vt_1_centerline_e001` is the true floor-transition edge. `vt_1_centerline_e003` is not the floor-transition edge.

Task41 reached `generated_ring_002` with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`. `generated_ring_037` remains occupied with zero clearance under `conservative_canonical`; it was not used. Object centroid navigation and manual target pose were not used.

Task42 completed the final audit and generated the Gazebo/RViz replay showcase package at `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/`.

This remains controlled simulation evidence only: no real robot, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, dense reconstruction, neural implicit SLAM, gait/footstep/contact planning, or LLM runtime navigation is claimed.
