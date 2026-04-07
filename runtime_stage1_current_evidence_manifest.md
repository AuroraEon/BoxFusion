# Runtime Stage-1 Current Evidence Manifest

Date: 2026-04-07
Goal: list the exact current on-disk evidence chain sufficient to support the frozen runtime baseline.

## Required evidence chain

### 1. Historical retained problem statement

These surviving compact historical files preserve the pre-freeze growth evidence and the reason the runtime line existed:

- `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_readout.md`
- `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_figure_data.csv`
- `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_highlight_table.csv`
- `runtime_stage1_frozen_evidence/historical_retained/docs/runtime_growth_paper_table.csv`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/aggregate_summary.json`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/blocker_summary.json`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/combined_per_profiled_frame.csv`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/combined_history_scope.csv`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/combined_segmentation_runs.csv`
- `runtime_stage1_frozen_evidence/historical_retained/eval/runtime_instrumentation/combined_vector_map_export_calls.csv`

Per-scene historical-retained representatives:

- `runtime_stage1_frozen_evidence/historical_retained/scenes/00829-QaLdnwvtxbs/manifest.json`
- `runtime_stage1_frozen_evidence/historical_retained/scenes/00843-DYehNKdT76V/manifest.json`
- `runtime_stage1_frozen_evidence/historical_retained/scenes/00862-LT9Jq6dN3Ea/manifest.json`
- Each of those scene roots also retains:
  - `logs/summary.json`
  - `logs/runtime_growth_profile.csv`
  - `logs/runtime_growth_profile.json`
  - `logs/runtime_instrumentation/per_profiled_frame.csv`
  - `logs/runtime_instrumentation/history_scope.csv`
  - `logs/runtime_instrumentation/segmentation_runs.csv`
  - `logs/runtime_instrumentation/vector_map_export_calls.csv`
  - `logs/runtime_instrumentation/summary.json`

### 2. Surviving compact post-baseline bundle

These files support the cleaned-repo compact frozen behavior:

- `runtime_stage1_frozen_evidence/freeze_support_summary.json`
- `runtime_stage1_frozen_evidence/current_partial_rerun/current_partial_runtime_report.md`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/aggregate_summary.json`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/blocker_summary.json`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/combined_per_profiled_frame.csv`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/combined_history_scope.csv`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/combined_segmentation_runs.csv`
- `runtime_stage1_frozen_evidence/current_partial_rerun/eval/runtime_instrumentation/combined_vector_map_export_calls.csv`

Per-scene compact representatives:

- `runtime_stage1_frozen_evidence/current_partial_rerun/scenes/00829-QaLdnwvtxbs/manifest.json`
- `runtime_stage1_frozen_evidence/current_partial_rerun/scenes/00843-DYehNKdT76V/manifest.json`
- `runtime_stage1_frozen_evidence/current_partial_rerun/scenes/00862-LT9Jq6dN3Ea/manifest.json`
- Each of those scene roots also retains:
  - `logs/summary.json`
  - `logs/topology_v0_1.json`
  - `logs/topology_query_report.json`
  - `logs/vertical_transition_evidence.json`
  - `logs/floor_diagnostics_summary.json`
  - `logs/runtime_growth_profile.csv`
  - `logs/runtime_growth_profile.json`
  - `logs/runtime_instrumentation/per_profiled_frame.csv`
  - `logs/runtime_instrumentation/history_scope.csv`
  - `logs/runtime_instrumentation/segmentation_runs.csv`
  - `logs/runtime_instrumentation/vector_map_export_calls.csv`
  - `logs/runtime_instrumentation/summary.json`

### 3. Reanalysis performed in this task

The existing compact bundle was reanalyzed into a fresh freeze-verification root:

- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/aggregate_summary.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/blocker_summary.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/report.md`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/combined_per_profiled_frame.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/combined_history_scope.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/combined_segmentation_runs.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/combined_vector_map_export_calls.csv`

### 4. Fresh representative reruns performed in this task

New compact reruns:

- `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V/manifest.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00862-LT9Jq6dN3Ea/manifest.json`

Each fresh scene rerun currently includes:

- `logs/summary.json`
- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/vertical_transition_evidence.json`
- `logs/floor_diagnostics_summary.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`
- `logs/runtime_instrumentation/per_profiled_frame.csv`
- `logs/runtime_instrumentation/history_scope.csv`
- `logs/runtime_instrumentation/segmentation_runs.csv`
- `logs/runtime_instrumentation/vector_map_export_calls.csv`
- `logs/runtime_instrumentation/summary.json`

Fresh aggregate over those reruns:

- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/aggregate_summary.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/blocker_summary.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/report.md`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/combined_per_profiled_frame.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/combined_history_scope.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/combined_segmentation_runs.csv`
- `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/combined_vector_map_export_calls.csv`

## Optional surviving evidence that is no longer required

The heavyweight root `world_model_backend_outputs_v0_2_final/` still exists and still contains runtime material, but the frozen baseline package above does not depend on it. It is now supplementary rather than required.

## Why this chain is sufficient

- Historical-retained evidence preserves the strongest long-run pre-freeze runtime-growth measurements.
- The compact current-partial bundle preserves post-baseline representative behavior in a cleaned-repo-safe form.
- The fresh 2026-04-07 reruns prove the current cleaned repo can still regenerate accepted representative evidence for `00843` and `00862`.
- The aggregate reanalysis outputs prove that the current evidence roots are internally readable and reproducible by the shipped analysis script.
