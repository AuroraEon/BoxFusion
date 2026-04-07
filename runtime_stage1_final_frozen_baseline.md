# Runtime Stage-1 Final Frozen Baseline

Date: 2026-04-07
Status: frozen from the current cleaned repo state

## Current status

- The stage-1 runtime line is now treated as closed and stabilized for this repo state.
- The accepted runtime work is the 10-item baseline below. No new runtime optimization line was opened in this task.
- The freeze package now relies on compact evidence under `runtime_stage1_frozen_evidence/` plus two fresh representative reruns performed in this task. It does not require deleted bulky raw roots to remain useful.

## Accepted frozen baseline stack

1. Floor-scoped pruning.
2. Within-floor retained-history filter: keep retained same-floor history only when `recent<=25` profiled steps or `XY<=4m`, with conservative fallback handling for missing recency/XY metadata.
3. Export-side structure reuse.
4. Changed-room rebuild reduction.
5. Spatial-association AABB coarse reject before OBB / ConvexHull.
6. Object-export eager-skip for unchanged cached objects in unchanged rooms.
7. Readonly-tail shadow-reference audit gated OFF by default unless explicitly enabled.
8. Skip readonly reference-mask build when the audit is off.
9. Export-side per-floor wall-label vote cache.
10. Rebuilt-anchor scoring-clearance reuse through unchanged-room anchor reuse.

## Strongest accepted effects still supported by current evidence

### Historical pre-freeze growth evidence that still survives

These numbers come from surviving historical retained evidence in `runtime_stage1_frozen_evidence/historical_retained/` and explain why the runtime line was worth freezing:

- `00829-QaLdnwvtxbs`: total-step growth `2.898x`, topology growth `4.198x`, feature/boxfusion growth `4.809x`.
- `00843-DYehNKdT76V`: total-step growth `2.270x`, topology growth `3.992x`, feature/boxfusion growth `3.454x`.
- `00862-LT9Jq6dN3Ea`: total-step growth `8.682x`, topology growth `5.025x`, feature/boxfusion growth `11.204x`.

These historical numbers are carried forward, not regenerated in this task.

### Surviving compact post-baseline evidence

The compact representative rerun bundle in `runtime_stage1_frozen_evidence/current_partial_rerun/` shows the accepted baseline behaviors directly:

- History scopes are no longer `global_retained_history` / `near_global_retained_history`; they are now `same_floor_recent_or_near_retained_history` and `same_floor_recent_or_near_cached_room_tail_pruned_retained_history`.
- Duplicate export frames are `0` for all three compact representative scenes.
- Readonly-tail audit remains off by default:
  - `tail_audit_disabled_rows`: `11 / 9 / 10`
  - `assoc_reference_mask_prep_zero_rows`: `12 / 12 / 12`
- Changed-room local rebuild is active:
  - `delta_export_call_count`: `4 / 4 / 2`
  - `changed_room_local_rebuild_used_count`: `4 / 4 / 2`
- Object export reuse is active:
  - `object_export_reused_total`: `103 / 83 / 105`
  - `object_export_rebuilt_total`: `85 / 62 / 86`
- Anchor reuse is active on unchanged-room cases:
  - `anchor_reuse_room_total`: `4 / 4 / 0`
  - `anchor_rebuild_room_total`: `20 / 17 / 10`

Scene order above is `00829 / 00843 / 00862`.

### Fresh representative reruns performed in this task

On 2026-04-07 I reran two accepted compact representative checks into `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/`:

- `00843-DYehNKdT76V`: `260` frames, `34.400 s`, `7.558 FPS`
- `00862-LT9Jq6dN3Ea`: `260` frames, `35.645 s`, `7.294 FPS`

The fresh reruns reproduced the frozen compact behavior:

- duplicate export frames: `0 / 0`
- history scopes:
  - `00843`: `current_frame_only=1`, `same_floor_recent_or_near_retained_history=6`, `same_floor_recent_or_near_cached_room_tail_pruned_retained_history=21`
  - `00862`: `current_frame_only=1`, `same_floor_recent_or_near_retained_history=18`, `same_floor_recent_or_near_cached_room_tail_pruned_retained_history=12`
- readonly-tail audit remained off by default, with `audit_disabled` counts `9 / 10`

## Rejected or non-accepted lines

- The older global / near-global retained-history runtime line is not accepted anymore. It survives only as historical evidence explaining the original bottleneck.
- Treating stage-5 cost as "CLIP / BoxFusion only" remains rejected. Surviving instrumentation and code both show export work inside the same bucket.
- Readonly-tail shadow audit is not part of the normal frozen runtime path. It remains an explicit opt-in audit path only.
- Rebuilding unchanged rooms, unchanged object exports, and unchanged-room anchors on every export remains rejected; the frozen baseline keeps reuse and changed-room-local rebuild behavior instead.
- Broad topology redesign, Query API redesign, planner redesign, or new optimization-line work is intentionally not part of this freeze.

## Closure rationale

- The accepted 10-item stack is still reflected in the current codebase.
- The cleaned repo still contains enough compact evidence to support the freeze without depending on deleted bulky roots.
- Two representative present-state reruns were completed successfully in this task.
- The core mock validations still pass on the live repo.

Taken together, that is enough to freeze the runtime baseline conservatively from the current repo state.

## Evidence still available now

- Surviving historical retained evidence:
  - `runtime_stage1_frozen_evidence/historical_retained/`
  - `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_readout.md`
  - `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/aggregate_summary.json`
- Surviving compact representative evidence:
  - `runtime_stage1_frozen_evidence/current_partial_rerun/`
  - `runtime_stage1_frozen_evidence/current_partial_rerun/current_partial_runtime_report.md`
  - `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/aggregate_summary.json`
  - `runtime_stage1_frozen_evidence/freeze_support_summary.json`
- New verification evidence created in this task:
  - `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/`
  - `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V/`
  - `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00862-LT9Jq6dN3Ea/`
  - `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/`

## What was reconstructed or rerun in this task

- Reanalyzed the surviving `current_partial_rerun` bundle into `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/`.
- Fresh rerun: `00843-DYehNKdT76V` compact accepted verification path.
- Fresh rerun: `00862-LT9Jq6dN3Ea` compact accepted verification path.
- Reanalyzed those fresh reruns into `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/`.
- Verified live code with `py_compile`.
- Verified live mock test suites by directly running:
  - `boxfusion/test_floor_aware_world_graph.py`
  - `boxfusion/test_scene_graph.py`

## Trust caveats

- The long full-run historical bundles were not rerun in this task. Their metrics are carried forward from surviving retained evidence.
- The fresh reruns in this task are compact `260`-frame representative reruns, not exact replacements for deleted full-scene raw runtime forests.
- Some short-window growth ratios are noisy when early-frame denominators are very small. For example, short-window topology ratios should be read together with absolute times, not alone.
- Optional showcase/demo artifacts were intentionally not regenerated; the freeze package is backend/runtime focused.
