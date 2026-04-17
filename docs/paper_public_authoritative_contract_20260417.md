# Paper-Facing Authoritative Public-Consumption Contract

Date: 2026-04-17

## Reusable Statement

BoxFusion downstream consumers read a manifest-backed committed/public bundle.
The authoritative public routing input is the scene `manifest.json`, which resolves to `logs/topology_v0_1.json` as the public committed-topology surface and may also expose `logs/summary.json` plus the committed world snapshot for convenience.

Working and lifecycle artifacts are not the default downstream routing surface.
In particular, `logs/working_topology_v0_1.json` and `logs/online_topology_lifecycle_v0_1.json` remain debug/diagnostic only and are kept separate from public query consumption.

The lightweight room-graph VLN demo routes only on committed/public outputs.
Explicit room-goal routing uses the public topology, and semantic room-goal routing ranks committed room summaries only among rooms that are also present in the public topology export.

The ROS public query entry uses the same authoritative public bundle.
The preferred input is the manifest-backed path; the runtime export coordinator preserves this contract by exposing stable latest pointers such as `<coordination_root>/latest` and `<coordination_root>/latest_manifest.json` without changing committed/public semantics.

## Code/Artifact Grounding

- `boxfusion/ros_query_server.py`
  - accepted public inputs: scene root, `manifest.json`, `logs/summary.json`, or `logs/topology_v0_1.json`
  - rejected as public input: `logs/working_topology_v0_1.json`
- `boxfusion/runtime_export_coordinator.py`
  - stable latest pointers preserve the synchronous manifest-backed export as the source of truth
- room-graph VLN demo JSON contract
  - `public_topology_source: topology_v0_1.json`
  - `semantic_summary_source: committed_room_world_model_v0_1.json`
  - `working_state_used_for_routing: false`

## Debug-Only Examples

- `logs/working_topology_v0_1.json`
- `logs/online_topology_lifecycle_v0_1.json`
- publication-diagnostics surfaces under the lifecycle/debug path
- shadow sidecar parity outputs under `runtime_export_validation/.../sidecar_shadow/`
