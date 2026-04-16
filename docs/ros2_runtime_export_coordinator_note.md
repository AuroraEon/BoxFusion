# ROS2 Runtime Export Coordinator Note

## Chosen Shape

This step adds a narrow producer-side coordinator that sits above the existing Stage A scene-root export path.

- it does not redefine the Stage A artifact layout
- it does not add incremental publication semantics
- it does not widen the public topology contract

Instead, it treats the existing Stage A scene root as the authoritative bundle and refreshes a stable latest-pointer location that the existing ROS2 consumers can follow.

Implementation entry point:

- `boxfusion/runtime_export_coordinator.py`

Validation helper:

- `stage_a_runtime_export_coordinator_validation.py`
- it enables the shadow sidecar and records the diagnostic-only parity check in `shadow_sidecar_parity_check`
- it also records `shadow_minimal_public_topology_parity_check` for the first
  real-export shadow subset: committed/public floors, rooms, edge summaries, and
  object-to-room memberships

## Cadence / Trigger Model

The coordinator supports snapshot-style refreshes.

Per refresh cycle it:

1. optionally runs a configured producer command
2. refreshes `manifest.json` for the current Stage A scene root
3. validates that the committed/public query bundle resolves
4. validates that the lifecycle/debug diagnostics bundle resolves
5. builds the immutable runtime snapshot from the authoritative manifest-backed bundles
6. optionally writes a shadow-only minimal sidecar subset
7. updates stable latest pointers
8. writes refresh metadata

This keeps the model intentionally periodic and coarse-grained.
There is no replay requirement, no fine-grained online room publication stream, and no attempt to push per-room incremental updates into ROS.

## Output Strategy

Coordinator-owned output root:

- `<coordination_root>/latest`
  - symlink to the current Stage A scene root
- `<coordination_root>/latest_manifest.json`
  - symlink to the current `manifest.json`
- `<coordination_root>/latest_export.json`
  - metadata describing the latest refresh
- `<coordination_root>/refresh_history/<timestamp>.json`
  - immutable per-refresh metadata snapshot
- `<coordination_root>/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`
  - optional shadow-only count/ID/lifecycle subset plus the minimal
    committed/public topology subset when sidecar shadow export is enabled

This means the existing ROS2 consumers can simply point at:

- query server:
  - `<coordination_root>/latest`
  - or `<coordination_root>/latest_manifest.json`
- publication diagnostics server:
  - the same stable path

No consumer API changes are required.

The sidecar file is explicitly non-authoritative. It is useful for shadow parity validation only; the synchronous manifest-backed export remains the source of truth.

## Public vs Lifecycle Separation

The coordinator does not merge surfaces.

- committed/public stays backed by `topology_v0_1.json`
- lifecycle/debug stays backed by `online_topology_lifecycle_v0_1.json`
- both are discovered through the same manifest, but reported separately in `latest_export.json`

The metadata explicitly keeps:

- `committed_public`
- `lifecycle_debug`

as separate sections.

That preserves the current rule:

- public topology means committed/published only
- non-`PUBLISHED` rooms remain lifecycle/debug only

## ROS2 Bring-Up

The existing ROS2 consumer nodes can now use the stable latest pointer rather than a one-off scene path.

Example launch inputs:

- query server `artifact_path:=/abs/path/to/<coordination_root>/latest`
- publication diagnostics `artifact_path:=/abs/path/to/<coordination_root>/latest`

An additional launch file is included for the coordinator process itself:

- `boxfusion_ros_query_server/launch/runtime_export_coordinator.launch.py`

## Intentionally Deferred

This step still does not add:

- a mature live incremental public topology service
- new public topology semantics
- publication of working topology as public truth
- sidecar replacement of the synchronous exporter
- rich sidecar materialization of anchors, scene graphs, GraphML, vector maps, or diagnostics
- replay orchestration
- runtime optimization work
- broader ROS API expansion

The coordinator is only a stable producer-side bridge from the current Stage A export path to the already-landed ROS2 consumers.
