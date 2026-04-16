# Runtime Sidecar Real-Bundle Acceptance Report

## Bundles tested

- `codex_acceptance_audit/full_artifact/00843-DYehNKdT76V`
  - profile: `full_artifact`
  - coordinator report: `runtime_export_validation/full_artifact_acceptance_report.json`
  - sidecar shadow output: `runtime_export_validation/full_artifact_acceptance/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`
- `codex_acceptance_audit/core_only/00843-DYehNKdT76V`
  - profile: `core_only`
  - coordinator report: `runtime_export_validation/core_only_acceptance_report.json`
  - sidecar shadow output: `runtime_export_validation/core_only_acceptance/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`

## Shadow subset parity results

Both real-bundle runs passed the existing coordinator validation path with sidecar shadow export enabled.

| Bundle profile | `shadow_sidecar_parity_check` | `shadow_minimal_public_topology_parity_check` |
| --- | --- | --- |
| `full_artifact` | passed, 0 mismatches | passed, 0 mismatches |
| `core_only` | passed, 0 mismatches | passed, 0 mismatches |

Both bundles produced the same minimal committed/public topology subset:

- floors: `floor_1`
- rooms: `room_2`, `room_3`
- objects with room membership: `obj_2`, `obj_3`, `obj_4`
- counts: 1 floor, 2 rooms, 1 edge, 3 object-room memberships
- edge relation summary: `adjacent: 1`

The sidecar remained shadow-only in both runs:

- sidecar mode: `shadow_minimal_subset`
- sidecar authoritative flag: `false`
- refresh metadata keeps `sidecar_replaces_authoritative_export: false`

## Exact matches

The runtime sidecar parity check matched all compared fields for both profiles:

- schema contract version
- runtime sequence/frame/snapshot/segmentation counters
- runtime final floor, room, object, and anchor counts
- runtime final floor, room, object, and anchor ids
- public topology semantics and public topology meaning
- public floor, room, edge, object, and anchor counts
- public floor, room, object, and anchor ids
- lifecycle committed/non-published room counts
- lifecycle publication state counts
- lifecycle committed/non-published room ids
- `non_published_rooms_public`

The minimal public topology shadow parity check matched all compared fields for both profiles:

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

No bounded diagnostic-only parity diffs were reported. Both parity result objects contain empty `mismatches` arrays.

The coordinator contrast check still shows the expected surface separation:

- query/public topology path reported 2 public rooms
- publication diagnostics reported 3 lifecycle/debug rooms and 1 published room
- `non_published_rooms_public` stayed `false`

This means the shadow subset stayed aligned with the authoritative committed/public export and did not pull lifecycle/debug-only room material into the public sidecar subset.

## Fixes applied, if any

No code fixes were required for this acceptance pass.

Generated acceptance artifacts were written under `runtime_export_validation/`.

## Remaining gaps

- This validates one scene id across two artifact profiles, not a broad corpus.
- The sidecar still intentionally excludes anchors, scene graph relationships, GraphML, rich diagnostics, full vector-map details, and visual debug artifacts.
- The synchronous manifest-backed exporter remains the authoritative export path.

## Exact commands used

```bash
python3 stage_a_runtime_export_coordinator_validation.py --artifact-path codex_acceptance_audit/full_artifact/00843-DYehNKdT76V --coordination-root runtime_export_validation/full_artifact_acceptance --json-out runtime_export_validation/full_artifact_acceptance_report.json
```

```bash
python3 stage_a_runtime_export_coordinator_validation.py --artifact-path codex_acceptance_audit/core_only/00843-DYehNKdT76V --coordination-root runtime_export_validation/core_only_acceptance --json-out runtime_export_validation/core_only_acceptance_report.json
```

An initial `python ...` invocation was attempted before this and failed because `python` is not installed on this machine; the successful runs used `python3`.

## Final recommendation

The first real-export shadow parity target, `minimal_public_topology_subset`, is stable enough to consider validated on these real committed-bundle paths.

The narrowest logical next target is another tiny real-bundle acceptance pass over one or two additional scenes/profiles with different committed/public topology shape, while keeping the same subset and keeping the synchronous exporter authoritative. Do not expand sidecar coverage until this same subset has seen a little more real topology variety.
