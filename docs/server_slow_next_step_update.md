# Server Slow Next Step Update

Date: 2026-04-21

## Scope

This change takes the smallest safe next step on the Stage-A backend path toward separating benchmark/paper-critical synchronous work from richer non-essential service/debug materialization.

It does **not** reopen broad performance hunting, does **not** change the detector/CLIP/model stack, does **not** change CUDA topology, does **not** broaden sidecar scope, and does **not** make sidecar outputs authoritative.

The current narrow future-facing target remains `minimal_public_topology_subset`.

## What Changed

Added one explicit Stage-A switch:

- `--suppress-service-debug-artifacts`

When the flag is enabled, Stage-A keeps the authoritative committed/public export path and the current lifecycle export, but suppresses the richer non-authoritative service/debug artifacts that were still being materialized synchronously in `ClosedLoopDemoRecorder.finalize()`.

The switch is wired through the runtime policy as:

- `RuntimeArtifactPolicy.materialize_rich_service_debug_artifacts`

Default behavior is unchanged. The richer artifacts are still materialized unless the new flag is explicitly enabled.

## Exact Synchronous Work Now Suppressible

With `--suppress-service-debug-artifacts`, the following synchronous finalize-time work is skipped and any stale copies are removed from the scene root:

- `logs/room_scoped_runtime_state_v0_1.json`
- `logs/final_vector_map_snapshot.json`
- `logs/working_topology_v0_1.json`
- `logs/working_vs_committed_topology_report_v0_1.json`
- `logs/working_vs_committed_topology_timeline_v0_1.json`
- `logs/working_vs_committed_topology_timeline_v0_1.md`
- `logs/room_commit_diagnosis_v0_1.json`
- `logs/room_commit_diagnosis_v0_1.md`

The switch also skips the extra pre-finalize lifecycle materialization step that existed only to feed the working/debug comparison exports:

- `OnlineTopologyLifecycleManager.finalize_report(..., public_topology_export_succeeded=False)`

## What Remains Mandatory In The Synchronous Path

This change intentionally keeps the benchmark/paper-safe committed/public contract unchanged:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/committed_room_world_snapshot_v0_1.json`
- `logs/vertical_transition_evidence.json`
- `logs/floor_diagnostics_summary.json`
- `logs/online_topology_lifecycle_v0_1.json`
- `logs/summary.json`
- `manifest.json`

Optional demo/full-artifact behavior is unchanged and remains controlled by the existing `core_only` / service-mode policy.

## Exact Files Changed

- `boxfusion/runtime_artifact_policy.py`
- `stage_a_demo.py`
- `boxfusion/stage_a_demo.py`
- `boxfusion/room_scoped_runtime.py`
- `boxfusion/test_runtime_artifact_policy.py`
- `boxfusion/test_room_scoped_runtime.py`
- `docs/server_slow_next_step_update.md`
- `docs/server_slow_next_step_update.csv`

## Exact Flags Added

- `--suppress-service-debug-artifacts`

## Timing Comparison

Measurement path:

- synthetic Stage-A finalize smoke path
- `ClosedLoopDemoRecorder.finalize()`
- `core_only=True`
- 3 snapshots, 2-room committed/public smoke scene
- 10 runs averaged for each condition

Relevant host-side timing buckets from `logs/summary.json -> finalize_export_timing_sec`:

| bucket | baseline mean sec | suppressed mean sec | delta sec |
| --- | ---: | ---: | ---: |
| `public_authoritative_export_sec` | 0.005010 | 0.004696 | -0.000314 |
| `mandatory_supporting_export_sec` | 0.002994 | 0.002953 | -0.000041 |
| `rich_service_debug_artifact_materialization_sec` | 0.008652 | 0.000000 | -0.008652 |
| `manifest_refresh_sec` | 0.230921 | 0.008147 | -0.222774 |
| `total_finalize_export_sec` | 0.248758 | 0.017427 | -0.231331 |

Interpretation:

- The new switch cleanly zeros the targeted rich-debug materialization bucket.
- The public authoritative export bucket stays effectively unchanged.
- The largest observed follow-on reduction in this smoke path appears in manifest refresh, because the manifest no longer has to inspect the heavier working/debug artifacts that were previously written synchronously.

This is still a narrow export/materialization smoke measurement, not a reopened end-to-end runtime hunt.

## Validation

Focused validation run:

- `python3 -m pytest -q boxfusion/test_runtime_artifact_policy.py boxfusion/test_room_scoped_runtime.py`
- result: `7 passed`

The recorder smoke test now explicitly checks that:

- committed/public topology, query report, world model, world snapshot, and lifecycle export remain available
- the suppressed rich service/debug artifacts are absent on disk
- the new timing bucket reports `0.0` for suppressed rich-debug materialization

## What Was Intentionally Not Changed

- no detector / CLIP / model-stack changes
- no multi-GPU or CUDA-topology changes
- no candidate-2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue`) expansion
- no broad contract redesign
- no sidecar authority change
- no manifest-backed authoritative-path removal
- no broad lifecycle/publication redesign
- no query/VLN public-contract change

## Does This Strengthen The Later Narrow Sidecar Case?

Yes, modestly.

This change makes the synchronous boundary cleaner without promoting sidecar authority:

- the authoritative committed/public bundle remains the same
- lifecycle export still stays in the current path
- the richer debug/service artifacts that are clearly broader than `minimal_public_topology_subset` now have an explicit opt-out switch

That improves the next-sidecar story because the remaining synchronous work is more clearly the committed/public core, while the newly suppressible artifacts are now an explicitly separable non-authoritative tier.
