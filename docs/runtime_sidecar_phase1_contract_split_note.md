# Runtime Sidecar Phase 1 / 1.5 Contract Split

This phase is intentionally a skeleton. The current synchronous Stage A export remains authoritative for ROS/VLN/query consumers.

## Runtime Snapshot Contract

The new immutable snapshot schema lives in `boxfusion/runtime_snapshot.py`.

`RuntimeStateSnapshot` has three explicit surfaces:

- `runtime_maintained_state`
  - sequence/frame/timestamp counters
  - snapshot/segmentation counts
  - final room/object/anchor counts
  - latest vector-map path when available
  - notes that Stage3 segmentation state and Stage5 association/fusion stay hot
- `committed_public_export_surface`
  - manifest, summary, topology, and final vector-map paths
  - topology surface/semantics
  - public topology meaning fixed as committed/published only
  - floor/room/edge/object/anchor counts
- `lifecycle_debug_surface`
  - lifecycle artifact path and semantics
  - committed-room and non-published-room counts
  - publication-state counts
  - explicit `non_published_rooms_public: false`

The coordinator currently populates this snapshot from the existing manifest-backed committed/public and lifecycle/debug bundles. A future runtime-sidecar handoff can populate the same schema directly from in-memory Stage3/Stage5 state.

Phase 1.5 adds a tiny direct runtime population path for shadow parity checks. It reads only stable, low-risk fields from live Stage3/Stage5-style state:

- sequence/frame/timestamp metadata
- snapshot and segmentation-cycle counts
- final floor/room/object/anchor counts
- stable floor/room/object/anchor identifiers
- lifecycle publication-state counts and committed/non-published room identifiers when lifecycle payloads are available

This path intentionally does not call exporters to rebuild vector maps, topology, anchors, scene graphs, GraphML, diagnostics, or query indices. The manifest-backed synchronous export remains the source of truth.

## Sidecar Exporter Skeleton

The sidecar skeleton lives in `boxfusion/sidecar_exporter.py`.

`SnapshotMetadataSidecarExporter` accepts a `RuntimeStateSnapshot` and writes only a small shadow subset file:

- `runtime_snapshot_sidecar_shadow_subset_v0_1.json`
- `authoritative: false`
- `materialized_keys: ["minimal_runtime_state_summary", "minimal_public_topology_summary", "minimal_committed_public_topology_subset", "minimal_lifecycle_debug_linkage"]`

The shadow materialization contains only:

- runtime counts and stable IDs
- committed/public topology counts and stable IDs
- lifecycle/debug committed and non-published room summary linkage

It does not build vector maps, public topology, anchors, scene graphs, GraphML, rich diagnostics, or query indices. It proves the snapshot is consumable without moving authority away from the existing exporter.

## Shadow Parity Validation

`boxfusion/runtime_snapshot.py` now includes comparison tooling for the minimal phase-1.5 scope:

- `build_runtime_snapshot_from_live_stage_state(...)` builds the direct runtime-side snapshot from live Stage3/Stage5-style objects.
- `build_runtime_snapshot_shadow_parity_subset(...)` extracts only the count/ID/lifecycle subset.
- `compare_runtime_snapshot_shadow_parity(...)` compares either a direct runtime snapshot or a sidecar materialization against the authoritative manifest-backed snapshot.

The comparison checks schema presence and key-field agreement for the small subset only. Any diff is diagnostic-only; it is not a public contract change and is not exposed as a consumer-facing source of truth.

## First Real-Export Shadow Target

The next narrow target is documented in
`docs/runtime_sidecar_minimal_public_topology_shadow_note.md`.

The sidecar now also materializes a tiny committed/public topology subset:

- floor records
- room records
- undirected edge summaries
- object-to-room memberships
- related counts, stable IDs, and edge relation-type counts

This subset is still shadow-only and non-authoritative. It intentionally omits
anchors, scene graph relationships, GraphML, rich diagnostics, full vector-map
details, and visual/debug artifacts.

`compare_minimal_public_topology_shadow_parity(...)` compares this sidecar-built
subset against the current authoritative `topology_v0_1.json` export. Mismatches
remain diagnostic-only.

## Service/Debug Artifact Switch

The explicit policy lives in `boxfusion/runtime_artifact_policy.py`.

Modes:

- `benchmark` keeps the existing requested defaults.
- `debug` is available as a named non-service/debug-full mode.
- `service` coerces core-only behavior and disables/defer obvious artifact-only work by default:
  - GT visualization pointcloud
  - scene graph PNGs
  - topology GraphML
  - full RGB replay
  - readonly-tail reference audit
  - debug room artifacts
  - global point-cloud PLY

`stage_a_demo.py` now resolves one policy object before constructing the recorder or calling `demo.run`. The ROS export coordinator records the same policy in `latest_export.json`.

## Intentionally Deferred

This phase does not:

- replace the current synchronous exporter
- migrate vector-map/topology/anchor/scene-graph materialization to the sidecar
- migrate authoritative export to direct runtime snapshots
- add public incremental topology
- widen query/debug guarantees
- expose non-`PUBLISHED` rooms through public topology
- implement maintained candidate indices
- change detector/CLIP/multi-GPU behavior

Maintained candidate indices remain only a future insertion point behind the runtime snapshot contract.
