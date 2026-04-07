# Runtime Stage-1 Repo State Audit

Date: 2026-04-07
Scope: current cleaned worktree only

## 1. Present runtime docs and bundles

### Present compact runtime evidence roots

- `runtime_stage1_frozen_evidence/current_partial_rerun/`
- `runtime_stage1_frozen_evidence/historical_retained/`
- `runtime_stage1_frozen_evidence/freeze_support_summary.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/` created in this task

### Present heavyweight runtime root

- `world_model_backend_outputs_v0_2_final/` still exists in the worktree and is about `1.8G`.
- This root still contains runtime-growth CSV/JSON files and runtime-instrumentation outputs, but it is no longer required for the compact freeze package.

### Present runtime markdown/docs

- `runtime_stage1_frozen_evidence/current_partial_rerun/current_partial_runtime_report.md`
- `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_readout.md`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/README.md`

## 2. Missing or cleaned historical artifacts

### Tracked runtime reports deleted from the current worktree

These were visible through git but are absent from the cleaned worktree:

- `runtime_instrumentation_consistency_fix_report.md`
- `runtime_topology_formal_rerun_report.md`
- `runtime_topology_instrumented_report.md`
- `runtime_topology_root_cause_report.md`

Their contents were used only as historical reference while preparing this freeze; they are not required for the new compact evidence chain.

### Runtime docs referenced in IDE context but not present on disk

These filenames were not found anywhere in the current worktree:

- `runtime_artifact_cleanup_plan.md`
- `runtime_artifact_retention_manifest.md`
- `runtime_stage1_evidence_repack_audit.md`

I treated them as unavailable in the current cleaned state.

### Intentionally absent optional scene artifacts

The current compact scene manifests intentionally do not carry optional demo/showcase outputs such as:

- `timeline.json`
- `timeline.csv`
- final PNG/MP4 outputs
- `topology_v0_1.graphml`
- optional scene reports

That absence is consistent with `--core-only` and with the backend-first freeze goal.

## 3. Does the live code still reflect the accepted 10-item baseline?

Yes. The current codebase still reflects the accepted baseline.

### Baseline-to-code mapping

1. Floor-scoped pruning
- `demo.py`
- `_build_floor_scoped_candidate_mask(...)` assigns each retained object to a stable floor and prunes cross-floor retained history.

2. Within-floor retained-history filter (`recent<=25 OR XY<=4m`)
- `demo.py`
- `_build_floor_scoped_candidate_mask(...)` keeps same-floor retained history only when the profiled-step recency window or XY-distance threshold passes.

3. Export-side structure reuse
- `boxfusion/floor_aware_room_segmenter.py`
- `get_vector_map_data(...)` builds `export_structure_cache_token` and reuses cached export structure when compatible.

4. Changed-room rebuild reduction
- `boxfusion/floor_aware_room_segmenter.py`
- `_build_room_local_delta_object_exports(...)` rebuilds only impacted room buckets and reports changed-room locality metrics.

5. AABB coarse reject before OBB / ConvexHull
- `boxfusion/instances.py`
- `calculate_obb_iou(...)` performs an AABB overlap reject before calling `Instances3D.obb_iou(...)`.

6. Object-export eager-skip for unchanged cached objects in unchanged rooms
- `boxfusion/floor_aware_room_segmenter.py`
- `_build_object_exports(...)` and `_build_room_local_delta_object_exports(...)` reuse unchanged cached object-export records when signatures match.

7. Readonly-tail shadow-reference audit OFF by default
- `stage_a_demo.py`
- `demo.py`
- `--enable-readonly-tail-reference-audit` is opt-in only, and the runtime path defaults to disabled metrics.

8. Skip readonly reference-mask build when audit is off
- `demo.py`
- `stage5_assoc_reference_mask_prep_sec` is explicitly set to `0.0` and no reference mask is prepared when the audit is disabled.

9. Export-side per-floor wall-label vote cache
- `boxfusion/floor_aware_room_segmenter.py`
- `_build_object_export_floor_vote_cache(...)` caches per-floor wall labels and feeds object export.

10. Rebuilt-anchor scoring-clearance reuse
- `boxfusion/floor_aware_room_segmenter.py`
- `boxfusion/scene_graph_builder.py`
- `_build_anchor_layer_with_cached_room_reuse(...)` reuses unchanged-room anchor payloads so unchanged rooms skip re-running anchor generation and its clearance/scoring work.

## 4. Surviving evidence versus rebuilt support

### Surviving evidence used directly

- `runtime_stage1_frozen_evidence/historical_retained/*`
- `runtime_stage1_frozen_evidence/current_partial_rerun/*`
- `runtime_stage1_frozen_evidence/freeze_support_summary.json`

### Rebuilt or regenerated in this task

- Reanalysis of current partial bundle:
  - `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/*`
- Fresh compact rerun:
  - `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V/*`
- Fresh compact rerun:
  - `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00862-LT9Jq6dN3Ea/*`
- Aggregate analysis of those fresh reruns:
  - `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/*`

### Verification performed without new disk artifacts

- `py_compile` of the main runtime files passed.
- Direct execution of `boxfusion/test_floor_aware_world_graph.py` passed.
- Direct execution of `boxfusion/test_scene_graph.py` passed.
- `pytest` was not available in the active environment, so the direct script path was used instead.

## 5. Audit conclusion

- The cleaned repo still retains enough compact evidence to freeze the runtime baseline responsibly.
- The accepted 10-item baseline is still present in code.
- Several earlier top-level runtime reports are gone from the worktree, but their role is now replaced by the new freeze docs plus compact retained evidence.
- Exact reconstruction of every deleted bulky historical artifact was not necessary for this freeze.
