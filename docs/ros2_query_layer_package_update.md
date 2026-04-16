# ROS2 Query Layer Package Update

## Chosen package layout

This step lands the prototype as two ROS2 packages inside the repo:

- `ros_interfaces`
  - ROS2 interface package that owns the `.srv` definitions
  - built with `ament_cmake` and `rosidl_default_generators`
- `boxfusion_ros_query_server`
  - ROS2 Python node package that exposes the query services
  - built with `ament_python`
  - installs the existing `boxfusion` Python package as its runtime payload so the thin backend logic is not duplicated

The packages are split because ROS2 interface generation and the Python node have different build systems, and splitting them keeps the query contract stable while letting the node depend on generated service types in the normal ROS2 way.

## Service contract adjustments

The `.srv` contracts were kept transport-thin and materially unchanged:

- request/response fields still use primitive strings plus a JSON payload string
- no typed-message expansion was added
- no new services were added

The only edits to the existing `.srv` files were clarifying comments so the wrapper-level semantics are more obvious in ROS2 tooling:

- `success=false` means malformed request or invalid node configuration
- expected query misses still come back inside `json_response`
- selector requirements are called out inline for object queries

## Node bring-up shape

The ROS2 node remains a thin adapter over committed/public artifacts only:

- default generated service module: `ros_interfaces.srv`
- default service prefix: `/boxfusion/query`
- explicit artifact-path configuration supported via ROS parameters and CLI defaults
- accepted artifact inputs stay the same:
  - scene root
  - `manifest.json`
  - `logs/summary.json`
  - `logs/topology_v0_1.json`
- `working_topology_v0_1.json` remains rejected

`ros2 run` and `ros2 launch` can now pass:

- `artifact_path`
- `service_prefix`

`artifact_search_roots` is also still supported by the Python node entrypoint for latest-bundle discovery.

## Minimal workspace bring-up

Assume the repo is available at `~/ws/src/BoxFusion`.

Build:

```bash
cd ~/ws
colcon build --packages-select ros_interfaces boxfusion_ros_query_server
source install/setup.bash
```

Launch with an explicit committed/public artifact path:

```bash
ros2 launch boxfusion_ros_query_server query_server.launch.py \
  artifact_path:=/absolute/path/to/scene_or_manifest
```

Or run directly:

```bash
ros2 run boxfusion_ros_query_server boxfusion_query_server_node --ros-args \
  -p artifact_path:=/absolute/path/to/scene_or_manifest
```

Discover services:

```bash
ros2 service list | grep /boxfusion/query
```

Example service calls from ROS2 tooling:

```bash
ros2 service call /boxfusion/query/get_topology ros_interfaces/srv/GetTopology "{}"
```

```bash
ros2 service call /boxfusion/query/resolve_object_room ros_interfaces/srv/ResolveObjectRoom \
  "{object_id: '', object_label: 'couch'}"
```

```bash
ros2 service call /boxfusion/query/route_to_room ros_interfaces/srv/RouteToRoom \
  "{start_room_id: 'room_0', goal_room_id: 'room_1', route_policy: 'balanced'}"
```

## Intentionally deferred

This package landing still intentionally does not do the following:

- working-topology publication or dependence
- replay or timeline stepping
- room-name grounding
- VLN/action bridge work
- broader typed-message redesign
- topology/backend feature expansion beyond the existing thin query surface

## Validation status

What was verified in this shell:

- the existing backend tests still run against committed/public artifact inputs
- the validation script still exercises the concrete query path without ROS
- the node argument parser now defaults to the local generated interface module name
- ROS2 package metadata and launch files were added for a real workspace bring-up path

What could not be verified in this shell:

- `colcon build`
- generated ROS2 interfaces
- `ros2 launch`
- service discovery through `ros2 service list`
- live `ros2 service call`

Those runtime checks were not possible here because the shell does not currently provide the ROS2 Python/runtime stack (`rclpy`, `launch`, `launch_ros`) or normal ROS2 CLI tooling.

## Overall status

The result is now a real ROS2-oriented package arrangement rather than only a repo-local prototype, but final end-to-end ROS2 usability still needs one pass in an actual ROS2 workspace to confirm interface generation, node startup, and service calls.
