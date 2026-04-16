# ROS Query Server First Step

## Node responsibility

`boxfusion_query_server_node` is a thin ROS-facing adapter over the already-committed/public artifact bundle.

Its job is to:

- load one committed/public export bundle
- build `RoomTopologyQueryAPI` from `logs/topology_v0_1.json`
- expose narrow ROS services that forward existing query semantics
- keep service responses strictly backed by committed/public topology only

Its job is not to:

- rebuild topology from runtime state
- consume working/debug topology
- depend on replay or timeline stepping
- publish provisional topology as default truth
- add planner or action semantics

## Artifact loading path

The server resolves a bundle in this order:

1. explicit input path if provided
2. otherwise latest discoverable `manifest.json` under configured search roots

Accepted explicit inputs:

- scene root
- `manifest.json`
- `logs/summary.json`
- `logs/topology_v0_1.json`

Preferred path is `manifest.json`, because it lets the server verify:

- `topology_json` exists
- `artifact_surface == public`
- `artifact_semantics == committed_topology`

Fallback loading from `summary.json` or `topology_v0_1.json` is allowed for minimal bring-up, but still maps only to the committed topology path and rejects `working_topology_v0_1.json`.

`GetWorldSnapshot` is intentionally best-effort:

- if `summary.json` exposes `final_vector_map_path`, the server returns that committed snapshot
- otherwise the service still returns bundle metadata and summary state, but marks the snapshot as unavailable

This keeps `core_only` compatible without inventing replay or requiring timeline reconstruction.

## Service list

Service paths default to `/boxfusion/query/...`:

- `/boxfusion/query/get_world_snapshot`
- `/boxfusion/query/get_topology`
- `/boxfusion/query/resolve_object_room`
- `/boxfusion/query/route_to_room`
- `/boxfusion/query/route_to_object`
- `/boxfusion/query/explain_connection`

Minimal `.srv` files live under `ros_interfaces/srv/` and intentionally use primitive fields plus a JSON string response:

- `bool success`
- `string error_code`
- `string json_response`

That keeps the first landing transport-thin and avoids overcommitting to typed ROS messages before the public query surface is fully exercised.

## Request and response semantics

`GetWorldSnapshot`

- request: empty
- response payload: bundle metadata, world-model summary, optional committed final vector-map snapshot

`GetTopology`

- request: empty
- response payload: full committed `topology_v0_1.json` content

`ResolveObjectRoom`

- request: exactly one of `object_id` or `object_label`
- response payload: direct `RoomTopologyQueryAPI.resolve_object_room(...)` result
- invalid selector combinations are rejected at the service wrapper level

`RouteToRoom`

- request: `start_room_id`, `goal_room_id`, optional `route_policy`
- response payload: direct `RoomTopologyQueryAPI.query_route(...)` result

`RouteToObject`

- request: `start_room_id`, exactly one of `object_id` or `object_label`, optional `route_policy`
- response payload: direct `RoomTopologyQueryAPI.query_route_to_object(...)` result

`ExplainConnection`

- request: `room_a`, `room_b`
- response payload: direct `RoomTopology.explain_connection(...)` result

Response convention:

- service-level `success=false` means malformed request or invalid bundle/service configuration
- query-level failures such as unresolved object ids or no route found are returned inside `json_response` from the existing query API semantics

## Intentionally out of scope

- online publication redesign
- working/debug topology publication
- room-name grounding or natural-language target selection
- anchor-specific ROS services
- replay control or ROS actions
- typed ROS message refinement beyond the minimal JSON-string wrapper
- ROS package/build-system wiring for generated interfaces

The included node expects a generated service module at runtime, but this task stops short of the full ROS workspace/package landing. The point of this step is to prove the committed export already works as a ROS-consumable backend.
