# RSLG-SLAM Object Navigation Status

Scene: `00843-DYehNKdT76V`

Target query: `curtain in room_14 on floor_2`

The query resolves to `obj_175` (`curtain`) in `room_14` on `floor_2`.

The selected object approach candidate is `generated_ring_002`:

- world position: `[-7.020484, 1.558795]`
- yaw: `-2.09057` rad
- stable-map state: free
- clearance: `0.20 m`
- local A* length from the room_14 route context to the exact candidate endpoint: `1.278742 m`

`generated_ring_037` remains occupied with zero clearance under `conservative_canonical` and is not the object runtime goal. Its status in non-selected profile analysis does not promote `navigation_thr0p25_candidate`.

Task41 executed the selected `cross_floor_object` route in authorized controlled simulation and reached `generated_ring_002` with endpoint distance `0.236665 m` and final yaw error `-0.081595 rad`. The route completed the floor_1 segment, the explicit `vt_1_centerline_e001` handoff, the floor_2 room route through `room_14`, and the final approach/yaw alignment.

The navigation goal is `generated_ring_002`, not the object centroid. No manual object pose, hand-moved approach pose, external GT floorplan, external GT occupancy map, simulator navmesh, or manual geometry was used.

Task42 generated replay evidence, MarkerArray inputs, PNG overlays, and final demo instructions. It did not perform visual object confirmation and does not claim a full object-navigation benchmark, real robot execution, physical stair climbing, AMCL success, or a full robot-footprint collision-free guarantee.
