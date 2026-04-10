# ROS/Gazebo Landing Contract v0.1

## Purpose

This note defines the smallest practical ROS 2 / Gazebo-facing landing path for the current backend.

It is intentionally conservative:

- backend/world-model first
- no Query API semantic changes
- no public/default working-topology exposure
- no topology truth-ownership change
- no planner-system redesign
- no reopening of runtime optimization

Gazebo is treated as an embodiment and simulation environment around the backend, not as a reason to turn the system into a planner or full embodied agent stack.

## Role of the Backend in a ROS 2 / Gazebo System

The backend node is the world-model producer.

Its job is to:

- ingest posed RGB-D and related backend inputs
- maintain the retained world export / world graph line
- derive committed room-centric topology from that world export
- answer structured topology queries through the existing Query API semantics
- optionally emit debug/private topology diagnostics

Its job is not to:

- own navigation planning
- own task planning
- expose provisional topology as the default public map
- replace Nav2 or another downstream navigation system

## Core Semantic Contract

### Truth owner

Truth ownership remains with world export / world graph.

In ROS-facing terms:

- the backend may publish a committed world export representation
- all public/default topology outputs are derived from that committed world export
- downstream consumers must not treat working topology as authoritative world truth

### Derived committed topology

Committed topology remains the public/default topology layer.

In ROS-facing terms:

- committed topology may be published as a stable derived signal
- the query server reads committed topology only by default
- route and target-resolution services must keep their current meaning

### Working topology

Working topology remains private/debug-only.

In ROS-facing terms:

- it may be published only on debug/private interfaces
- it must be disabled by default
- it must not silently back the public query service

## Artifact Classification

The following classification should remain explicit in the ROS/Gazebo landing.

| Artifact / layer | Meaning | Exposure class |
| --- | --- | --- |
| truth-owner world export / world graph | retained backend truth for rooms, objects, anchors, gateways, transitions, relationships | public/default |
| committed topology | derived stable topology for default query and downstream integration | public/default |
| working/debug topology | provisional derived topology for inspection only | debug-only |
| publication-policy simulation outputs | additive analysis of hypothetical non-default publication rules | offline-analysis-only |
| survivability analysis outputs | additive analysis of whether early-publication candidates actually survive | offline-analysis-only |

Additional note:

- `working_vs_committed` comparison artifacts and withheld-topology timelines are debug/offline analysis layers, not public/default interfaces

## Recommended Minimal ROS-Facing Architecture

The smallest practical architecture is:

### 1. Backend node

Suggested name:

- `boxfusion_backend_node`

Responsibilities:

- consume sensor and pose inputs
- maintain retained backend state
- publish committed world export snapshots
- publish committed topology snapshots
- persist the same committed artifacts already used by the current backend line

### 2. Query server node

Suggested name:

- `boxfusion_query_server_node`

Responsibilities:

- load or subscribe to committed topology only
- expose the existing Query API semantics over ROS services
- keep route policies and failure semantics unchanged

This can live in the same process as the backend node for the first landing if that is simpler. The logical separation matters more than process separation.

### 3. Debug topology publisher

Suggested name:

- `boxfusion_debug_topology_node`

Responsibilities:

- publish working-topology snapshots
- publish working-versus-committed summaries
- publish blocker/timeline diagnostics if explicitly enabled

This should be off by default.

### 4. Optional playback / replay bridge

Suggested name:

- `boxfusion_playback_bridge_node`

Responsibilities:

- feed recorded data or retained Stage-A outputs into the backend for debugging, rosbag replay, or deterministic integration testing

This is optional but useful for early implementation and demo bring-up.

### 5. Optional navigation adapter layer

Suggested name:

- `boxfusion_navigation_adapter_node`

Responsibilities:

- translate committed Query API route outputs into a downstream consumer format
- stay strictly downstream of the backend and query server

This is optional and should not be merged into the backend contract itself.

## ROS Interface Recommendations

### Public/default topics

Recommended public/default topics:

- `/boxfusion/world_export/committed`
- `/boxfusion/topology/committed`
- `/boxfusion/backend/status`

Recommended behavior:

- reliable delivery
- transient-local / latched semantics for snapshot-style committed outputs
- version field in payload
- explicit `truth_owner` and `public_default` flags

### Public/default services

Recommended public/default services:

- `/boxfusion/query/route`
- `/boxfusion/query/explain`

Smallest practical first landing:

- one generic route/query service that transports the existing Query API methods with explicit `query_type`
- one explanation service or an explanation block in the main response

Allowed `query_type` values should map directly to the current API:

- `route_to_room`
- `route_to_anchor`
- `route_to_object`

Optional later split, if stronger typing is desired:

- `/boxfusion/query/route_to_room`
- `/boxfusion/query/route_to_anchor`
- `/boxfusion/query/route_to_object`

That split would still be transport-level only and must not change semantics.

### Actions

Recommended first-pass action policy:

- no core backend action interface is required for the first landing
- query execution should remain a service, not an action
- committed topology publication should remain topic-based

The only reasonable early action candidate is:

- optional replay or playback control for the `boxfusion_playback_bridge_node`

That keeps long-running playback orchestration separate from backend semantics.

### Debug/private topics

Recommended debug/private topics:

- `/boxfusion/debug/topology/working`
- `/boxfusion/debug/topology/working_vs_committed`
- `/boxfusion/debug/topology/withheld_timeline`
- `/boxfusion/debug/topology/blockers`

These should:

- be disabled by default
- be clearly namespaced as debug/private
- carry `debug_only = true` and `public_default = false`
- never be used implicitly by public/default query services

### Offline-analysis-only artifacts

Recommended offline-only artifacts:

- publication-policy simulation JSON / markdown outputs
- publication-candidate survivability JSON / markdown outputs

These should not be first-pass live ROS topics.

If the team later wants replay visibility, the correct path is:

- optional replay publication from saved artifacts under `/boxfusion/replay/...`

not:

- folding these analyses into live committed/public semantics

## Query API Mapping Contract

The Query API public meaning must remain unchanged.

Current semantic anchors in the repo:

- `resolve_room_target(...)`
- `resolve_anchor_room(...)`
- `resolve_object_room(...)`
- `query_route(...)`
- `query_route_to_anchor(...)`
- `query_route_to_object(...)`

ROS mapping rules:

- `start_room_id`, target selectors, and `route_policy` keep the same meaning
- route policies remain `strict`, `balanced`, and `exploratory`
- the server resolves targets against committed topology only
- failures remain structured rather than silently guessed around
- explanation text remains an organization layer over the same structured result

This means ROS is only a transport and integration layer here. It is not permission to reinterpret the backend.

## Gazebo Contract

Gazebo should be used for:

- sensor generation
- embodiment and motion playback
- deterministic scenario bring-up
- integration testing of backend inputs and outputs

Gazebo should not be used here as justification for:

- planner-centric architecture changes
- topology truth-ownership changes
- public/default working-topology publication
- replacing the backend query contract with task-planning APIs

The intended Gazebo relationship is:

- Gazebo provides the world and the robot embodiment
- ROS transports sensor, pose, and backend outputs
- the backend produces committed world-model and topology products
- downstream navigation or control systems may consume backend query outputs later

## Public vs Debug vs Offline Rules

These rules should be treated as contract-level, not implementation suggestions.

### Public/default

Allowed:

- committed world export
- committed topology
- backend health/status
- query services over committed topology

Not allowed:

- working topology as default map
- publication-policy simulation outputs
- survivability outputs

### Debug-only

Allowed:

- working topology snapshots
- working-versus-committed summaries
- blocker summaries
- withheld-topology timelines

Requirements:

- explicit debug namespace
- explicit `debug_only = true`
- disabled by default

### Offline-analysis-only

Allowed:

- publication-policy simulation
- survivability analysis
- replay inspection of saved analysis artifacts

Not allowed:

- direct use as live public backend state
- use as a substitute for committed topology

## Tiny Interface Stubs

These are intentionally transport-level examples, not final ROS message definitions.

### Example committed topology topic payload

```json
{
  "version": "0.1",
  "artifact_kind": "committed_topology",
  "truth_owner": "world_export",
  "public_default": true,
  "debug_only": false,
  "sequence_id": "00843-DYehNKdT76V",
  "frame_idx": 999,
  "timestamp": 999.0,
  "source_world_export_id": "world_export_999",
  "rooms": [
    {
      "id": "room_3",
      "floor_id": "floor_0"
    }
  ],
  "edges": []
}
```

Semantics:

- this is the stable derived topology currently safe for public/default consumers
- it is derived from committed world export
- it is the topology the query server should answer against

### Example debug working-topology topic payload

```json
{
  "version": "0.1",
  "artifact_kind": "working_topology_debug",
  "truth_owner": "world_export",
  "public_default": false,
  "debug_only": true,
  "sequence_id": "00843-DYehNKdT76V",
  "frame_idx": 999,
  "timestamp": 999.0,
  "working_semantics": {
    "selection_rule": "present_in_latest_export && lifecycle_state in eligible_working_states"
  },
  "working_summary": {
    "selected_room_ids": ["room_2", "room_3", "room_4", "room_5", "room_6"],
    "committed_room_ids_for_projection": ["room_3"]
  },
  "rooms": [
    {
      "id": "room_2",
      "lifecycle_state": "active",
      "candidate_complete": false,
      "commit_block_reasons": [
        "containment_not_stable",
        "gateway_structure_not_stable",
        "room_currently_active",
        "room_signature_not_stable"
      ]
    }
  ]
}
```

Semantics:

- useful for visualization and debugging
- not stable enough for default public routing semantics
- must never silently replace committed topology in the query server

### Example query service request

```json
{
  "query_type": "route_to_object",
  "start_room_id": "room_3",
  "target": {
    "object_id": null,
    "object_label": "chair",
    "anchor_id": null,
    "goal_room_id": null
  },
  "route_policy": "balanced"
}
```

### Example query service response

```json
{
  "ok": true,
  "query_type": "route_to_object",
  "route_policy": "balanced",
  "resolved_goal_room_id": "room_5",
  "failure_reason": null,
  "route": {
    "attempted": true,
    "found": true,
    "room_ids": ["room_3", "room_4", "room_5"],
    "used_relation_types": ["transition", "adjacent"]
  },
  "explanation": {
    "summary": "Resolved object target and found a route on committed topology."
  }
}
```

Service semantics:

- a transport wrapper over the existing committed-topology Query API
- no access to working topology by default
- no hidden planner behavior

## Recommended First ROS/Gazebo Implementation Step

The first implementation step after this document should be:

- create a minimal ROS 2 backend bridge that publishes committed topology snapshots and exposes one generic committed-topology query service

That step is the highest-value landing because it:

- exercises the existing backend without changing semantics
- keeps truth ownership and publication rules intact
- gives Gazebo or rosbag-based integration something concrete to consume
- leaves working-topology and analysis interfaces safely out of the default public path

## Summary Contract

If there is one rule to preserve during implementation, it is this:

- committed world export and committed derived topology are the public/default line
- working topology is private/debug-only
- publication-policy simulation and survivability remain offline-analysis-only

If that rule stays intact, the project can land in ROS 2 / Gazebo without accidentally changing backend semantics.
