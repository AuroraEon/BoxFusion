# RSLG-SLAM Final Project Status

Scene: `00843-DYehNKdT76V`

RSLG-SLAM is Rich Semantic, Light Geometry SLAM. For this dataset-side scene, the inputs are RGB-D images and provided camera poses; dataset-side SLAM/localization accuracy is not claimed.

| Layer | Final status |
| --- | --- |
| Layer 0: Input Layer | RGB-D images and provided camera poses are the dataset inputs. |
| Layer 1: World Model Layer | Completed by task35. |
| Layer 2: Formal Artifact Layer | Completed by task36/task36b/task36c; conservative stable maps accepted; task40 added the recovered `generated_ring_002` object approach artifacts. |
| Layer 3: Navigation Interface Layer | Completed by task37/task40 for `cross_floor_room` and `cross_floor_object` under `conservative_canonical`. |
| Layer 4: Runtime Validation Layer | Room-level controlled simulation validation completed by task39; object-level controlled simulation validation completed by task41. |

The selected room route is `room_2 on floor_1 -> room_3 on floor_1 -> vt_1 / vc_vt_1 -> room_7 on floor_2 -> room_13 -> room_14`.

The selected object route extends that room route to `generated_ring_002` for `obj_175` (`curtain`) in `room_14` on `floor_2`. The selected candidate is at `[-7.020484, 1.558795]` with yaw `-2.09057` rad, free stable-map state, `0.20 m` clearance, and a `1.278742 m` local A* connection from the room_14 route context. `generated_ring_037` remains occupied with zero clearance under `conservative_canonical` and is not used as the runtime goal.

Task42 generated the Gazebo/RViz replay showcase package under `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/demo_evidence_pack/`. The package uses MarkerArray overlays, trajectory replay data, PNG figures, and run instructions. RViz Map display is not required.

Live task42 runtime was not rerun; the final package consolidates the already validated task39/task41 controlled-simulation evidence.

Claim boundaries: no real robot execution, physical stair climbing, AMCL success, visual object confirmation, full robot-footprint collision-free guarantee, full object-navigation benchmark, dense reconstruction, neural implicit SLAM, gait planning, footstep planning, contact planning, object-centroid goal, manual target pose, generated_ring_037 runtime goal, LLM runtime navigation, external GT floorplan, external GT occupancy map, or simulator navmesh as a world-model source is claimed.
