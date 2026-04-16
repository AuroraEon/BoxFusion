# Runtime Sidecar Topology-Variety Acceptance Report

## Bundles tested

- `codex_perf_probe/server/benchmark_cold25/output/00843-DYehNKdT76V`
  - source profile: `benchmark_cold25`
  - manifest artifact profile: `core_only`
  - coordinator report: `runtime_export_validation/topology_variety_benchmark_cold25_report.json`
  - sidecar shadow output: `runtime_export_validation/topology_variety_benchmark_cold25/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`
- `codex_perf_probe/server/benchmark_full300/output/00843-DYehNKdT76V`
  - source profile: `benchmark_full300`
  - manifest artifact profile: `core_only`
  - coordinator report: `runtime_export_validation/topology_variety_benchmark_full300_report.json`
  - sidecar shadow output: `runtime_export_validation/topology_variety_benchmark_full300/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`

## Why these bundles were chosen

No complete coordinator-ready manifest/topology/lifecycle bundle for a different scene id was present in this checkout. These two additional real committed bundles were chosen because they are physically present in the repo workspace, run through the existing coordinator path without producer changes, and give meaningful committed/public topology-shape contrast against the previously validated `codex_acceptance_audit/{full_artifact,core_only}/00843-DYehNKdT76V` case.

The validation set remains intentionally small and does not broaden sidecar materialization scope.

## Topology-shape contrast vs the previous validated case

Previous validated case, `codex_acceptance_audit/*/00843-DYehNKdT76V`:

- public subset counts: 1 floor, 2 rooms, 1 edge, 3 object-room memberships
- public rooms: `room_2`, `room_3`
- edge relation summary: `adjacent: 1`
- membership distribution: `room_3: 3`

Additional `benchmark_cold25` bundle:

- public subset counts: 1 floor, 1 room, 0 edges, 5 object-room memberships
- public rooms: `room_2`
- edge relation summary: none
- membership distribution: `room_2: 5`
- contrast: smaller public room set, no committed/public edge or gateway surface, and all memberships concentrated in one public room.

Additional `benchmark_full300` bundle:

- public subset counts: 1 floor, 4 rooms, 5 edges, 25 object-room memberships
- public rooms: `room_2`, `room_3`, `room_4`, `room_5`
- edge relation summary: `adjacent: 4`, `possible_connection: 1`
- membership distribution: `room_3: 25`
- contrast: larger public room set, mixed edge relation types, and a denser object-room membership set than the prior acceptance case.

## Shadow subset parity results

Both additional real-bundle runs passed the same existing coordinator validation path with sidecar shadow export enabled.

| Bundle | `shadow_sidecar_parity_check` | `shadow_minimal_public_topology_parity_check` |
| --- | --- | --- |
| `benchmark_cold25` | passed, 0 mismatches | passed, 0 mismatches |
| `benchmark_full300` | passed, 0 mismatches | passed, 0 mismatches |

The sidecar remained shadow-only in both runs:

- sidecar mode: `shadow_minimal_subset`
- sidecar authoritative flag: `false`
- `latest_export.json` keeps `sidecar_replaces_authoritative_export: false`
- authoritative export mode remains `synchronous_current_path`

## Exact matches

The runtime sidecar parity check matched all compared fields for both additional bundles:

- schema contract version
- runtime sequence, frame, snapshot, and segmentation counters
- runtime final floor, room, object, and anchor counts
- runtime final floor, room, object, and anchor ids
- public topology semantics and public topology meaning
- public floor, room, edge, object, and anchor counts
- public floor, room, object, and anchor ids
- lifecycle committed and non-published room counts
- lifecycle publication state counts
- lifecycle committed and non-published room ids
- `non_published_rooms_public`

The minimal public topology shadow parity check matched all compared fields for both additional bundles:

- required schema keys
- artifact kind, scope, and public topology meaning
- floor, room, edge, and object-room membership counts
- floor, room, and object ids
- floor records
- room records
- edge records
- object-room membership records
- edge relation type counts
- room membership availability

## Diagnostic-only diffs

No bounded diagnostic-only parity diffs were reported. Both parity result objects contain empty `mismatches` arrays for both bundles.

The coordinator contrast check still shows the expected separation between committed/public topology and lifecycle/debug diagnostics:

- `benchmark_cold25`: query/public topology reports 1 public room; lifecycle/debug diagnostics report 2 lifecycle rooms and 0 published rooms.
- `benchmark_full300`: query/public topology reports 4 public rooms; lifecycle/debug diagnostics report 5 lifecycle rooms and 1 published room.
- `non_published_rooms_public` stayed `false` in both runs.

This preserves the rule that public topology means committed/published-only topology, while non-PUBLISHED rooms remain lifecycle/debug-only.

## Fixes applied, if any

No code fixes were required.

Generated acceptance artifacts were written under:

- `runtime_export_validation/topology_variety_benchmark_cold25/`
- `runtime_export_validation/topology_variety_benchmark_full300/`
- `runtime_export_validation/topology_variety_benchmark_cold25_report.json`
- `runtime_export_validation/topology_variety_benchmark_full300_report.json`

## Remaining gaps

- This pass adds topology-shape variety, but not scene-id variety. The checkout only contained complete coordinator-ready real bundles for `00843-DYehNKdT76V`.
- The sidecar still intentionally excludes anchors, scene graph relationships, GraphML, rich diagnostics, full vector-map details, visual debug artifacts, maintained candidate indices, and broader public topology expansion.
- The synchronous manifest-backed exporter remains authoritative.

## Exact commands used

```bash
python3 stage_a_runtime_export_coordinator_validation.py --artifact-path codex_perf_probe/server/benchmark_cold25/output/00843-DYehNKdT76V --coordination-root runtime_export_validation/topology_variety_benchmark_cold25 --json-out runtime_export_validation/topology_variety_benchmark_cold25_report.json
```

```bash
python3 stage_a_runtime_export_coordinator_validation.py --artifact-path codex_perf_probe/server/benchmark_full300/output/00843-DYehNKdT76V --coordination-root runtime_export_validation/topology_variety_benchmark_full300 --json-out runtime_export_validation/topology_variety_benchmark_full300_report.json
```

## Final recommendation

`minimal_public_topology_subset` now has enough topology-shape coverage to be considered stable for this phase within the available real-bundle evidence: the validated set covers single-room/no-edge, two-room/single-edge, and four-room/mixed-edge committed/public shapes, all with exact shadow parity.

The narrowest next step is to freeze this sidecar target as phase-stable and add a similarly tiny acceptance gate when a non-`00843-DYehNKdT76V` committed bundle is available. Do not expand sidecar coverage or migrate richer topology surfaces until that next bundle exists or the project explicitly chooses the next narrow target.

## Phase-stable freeze

As of this phase closeout, `minimal_public_topology_subset` is frozen as a phase-stable shadow-only target.

The frozen subset remains limited to committed/public topology parity fields, and the synchronous manifest-backed Stage A export remains authoritative for query, diagnostics, and downstream consumers. Non-PUBLISHED rooms remain lifecycle/debug-only and must not be introduced into public query semantics through the sidecar.
