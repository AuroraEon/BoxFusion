# Unified Artifact Timeline Contract v0.1

## Scope

This note closes the current artifact-contract gap without changing the runtime model or requiring dense replay.

Goals:

- make current snapshot-based timeline exports consumable by replay/downstream loaders
- define one canonical timeline schema that can represent both snapshot-only and future dense replay
- make `core_only` and `full_artifact` explicit artifact profiles
- separate public, working, lifecycle, and diagnostic surfaces in manifests

## Canonical Timeline

`logs/timeline.json` remains a JSON array for backward compatibility.

The canonical schema is row-level `timeline_v1`:

- `timeline_schema = "timeline_v1"`
- `timeline_storage = "json_array_rows"`
- `row_kind = "snapshot_frame"` or `row_kind = "replay_frame"`

Compatibility rules:

- legacy `row_type = "snapshot"` maps to `row_kind = "snapshot_frame"`
- legacy `row_type = "replay_frame"` maps to `row_kind = "replay_frame"`
- snapshot-only timelines are valid `timeline_v1` artifacts
- dense replay is optional and advertised by capability flags, not implied by profile name

Practical interpretation:

- `snapshot_frame` rows represent committed/public checkpoint observations
- `replay_frame` rows represent future dense replay observations
- both row kinds are loadable by replay/downstream consumers when they expose `frame_idx`, `timestamp`, `current_room_id`, and optional `vector_map_path`

## Profiles

### `core_only`

Guarantees:

- final committed topology export
- final-state query support
- final-state route support
- final-state eval support
- query-bundle rebuild from final exports

Does not guarantee:

- checkpoint replay
- dense replay

Still allowed:

- lifecycle diagnostics
- working/committed comparison diagnostics

### `full_artifact`

Guarantees:

- everything in `core_only`
- a public `timeline_v1` artifact
- checkpoint replay from the exported timeline

Does not imply:

- dense replay

`full_artifact` may still be `snapshot_only`; dense replay is signaled separately through capability flags.

## Surfaces

Manifest artifact records now declare `artifact_surface` and `artifact_semantics`.

Surface rules:

- `public`: committed/published topology, timeline, reports, and renders
- `working`: debug/internal working-topology projections and working-vs-committed comparisons
- `lifecycle`: lifecycle-history diagnostics only, never treated as topology
- `diagnostic`: runtime/floor/segmentation diagnostics

## Capability Flags

Artifacts now declare:

- `final_state_query`
- `final_state_route`
- `final_state_eval`
- `query_bundle_rebuild`
- `checkpoint_replay`
- `dense_replay`
- `lifecycle_history`
- `working_topology_history`

These are computed from the actual artifact set, so snapshot-only `full_artifact` exports report:

- `checkpoint_replay = true`
- `dense_replay = false`

## Compatibility Strategy

- exporter writes canonical `row_kind` fields while preserving legacy `row_type`
- replay loader accepts both snapshot and replay row kinds immediately
- summary/manifest keep legacy fields such as `replay_mode` and `replay_frame_count`
- `replay_frame_count` remains dense-replay-only for compatibility with its name
- consumers that need the total exported timeline size should use `timeline_frame_count`
