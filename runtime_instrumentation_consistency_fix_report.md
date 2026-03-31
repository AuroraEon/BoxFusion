# Runtime Instrumentation Consistency Fix Report

## 1. Executive summary

Two different consistency problems were present.

- `segmentation_runs.csv` / `combined_segmentation_runs.csv` was empty because of a writer-side bug: per-run segmentation diagnostics were being computed inside `FloorAwareRoomSegmenter`, but they were never bridged into `RuntimeInstrumentation`, so the CSV writer had no rows to flush.
- `blocker_summary.json` was stale because of an aggregation/stale-artifact bug: the aggregate analyzer did not write or refresh that file at all on successful runs, so an older blocked artifact from 2026-03-30 remained in place.

What was fixed:

- Added an explicit segmentation-run sync path from `FloorAwareRoomSegmenter` to `RuntimeInstrumentation`.
- Added a deterministic scene-repair utility for previously successful runs whose segmentation CSVs were left header-only.
- Updated aggregate analysis so `blocker_summary.json` is always rewritten with current status on both blocked and success paths.

Result:

- Per-scene segmentation CSVs are now populated and summary counts are refreshed.
- `combined_segmentation_runs.csv` is populated.
- `blocker_summary.json` now reports `cleared_no_active_blocker` after a successful aggregate refresh.

## 2. Root-cause analysis

### A. Empty segmentation runs

Exact root cause:

- `FloorAwareRoomSegmenter._record_segmentation_run(...)` was already creating authoritative run records and storing them in `FloorState.segmentation_reports`.
- `RuntimeInstrumentation.log_segmentation_run(...)` existed, but nothing in `demo.py` ever called it.
- Because `RuntimeInstrumentation.finalize()` writes `segmentation_runs.csv` only from its own in-memory `self.segmentation_runs`, the scene CSV stayed header-only.
- The aggregate analyzer was not dropping rows; it simply concatenated already-empty per-scene segmentation CSVs, so `combined_segmentation_runs.csv` also ended up empty.

Affected files/functions:

- `boxfusion/floor_aware_room_segmenter.py`
  - `_record_segmentation_run(...)`
  - `FloorState.segmentation_reports`
- `boxfusion/runtime_instrumentation.py`
  - `log_segmentation_run(...)`
  - `finalize(...)`
- `demo.py`
  - stage-3 segmentation refresh path
  - finalization path
- `stage_a_eval/analyze_runtime_instrumentation.py`
  - aggregate combiner was only a downstream symptom here

Classification:

- Primary bug type: logging/writer bug
- Secondary symptom: aggregate output looked empty because upstream scene CSVs were empty

Impact on interpretation:

- Stage-3 refresh work existed in `per_profiled_frame.csv`, but the dedicated per-run segmentation artifact was missing, so per-refresh evidence was incomplete and the aggregate segmentation file could not be trusted as source-of-truth.

### B. Stale blocker summary

Exact root cause:

- There was no current code path refreshing `world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation/blocker_summary.json`.
- The file on disk was an older blocked artifact dated 2026-03-30 and remained untouched even after successful reruns and aggregate refreshes.

Affected files/functions:

- `stage_a_eval/analyze_runtime_instrumentation.py`
  - `main()`
  - success path had no blocker-summary write
  - blocked path had no current refresh logic either

Classification:

- Stale-artifact bug
- Aggregate-policy bug

Impact on interpretation:

- Advisor-facing aggregate outputs simultaneously showed successful aggregate evidence in `aggregate_summary.json` and a contradictory blocked state in `blocker_summary.json`.

## 3. Fixes implemented

### Code files changed

- `boxfusion/runtime_instrumentation.py`
- `boxfusion/floor_aware_room_segmenter.py`
- `demo.py`
- `stage_a_eval/analyze_runtime_instrumentation.py`
- `stage_a_eval/repair_runtime_instrumentation_scene_outputs.py`

### What each change does

`boxfusion/runtime_instrumentation.py`

- Added segmentation-run synchronization support.
- Added dedupe/replace behavior keyed by `run_id`, so repeated syncs update the same logical segmentation run instead of duplicating rows.

`boxfusion/floor_aware_room_segmenter.py`

- Added `export_segmentation_runs()` to expose the authoritative stored segmentation diagnostics with floor metadata attached.

`demo.py`

- Added `sync_segmentation_run_logs()` and invoked it after stage-3 segmentation refresh handling and again after finalization export handling.
- This means the per-scene runtime instrumentation output now receives the same segmentation run records already stored by the mapper-side segmentation code.

`stage_a_eval/analyze_runtime_instrumentation.py`

- Added explicit blocker-summary refresh policy.
- On missing inputs, the analyzer now writes `blocked_missing_runtime_instrumentation_artifacts`.
- On success, it now writes `cleared_no_active_blocker` with current timestamp, scene status, and aggregate artifact counts.

`stage_a_eval/repair_runtime_instrumentation_scene_outputs.py`

- New repair utility that reconstructs `logs/runtime_instrumentation/segmentation_runs.csv` and `logs/runtime_instrumentation/summary.json` from saved `logs/floor_diagnostics_summary.json`.
- This is for previously successful full-scene runs that already persisted authoritative segmentation-run diagnostics but were affected by the old writer bug.

### Behavior vs instrumentation

- Mapper semantics were not changed.
- The fixes are instrumentation/aggregation correctness fixes plus scene-artifact repair for already completed runs.

## 4. Validation plan

### Phase 1: light validation rerun

Executed:

- Scene: `00829-QaLdnwvtxbs`
- Command used: canonical runtime command with `--max-frames 130`

Observed results:

- `world_model_backend_outputs_v0_2_final/scenes/00829-QaLdnwvtxbs/logs/runtime_instrumentation/segmentation_runs.csv` became non-empty.
- The light rerun produced 3 segmentation runs:
  - frame `0`
  - frame `100`
  - final flush at frame `129`
- Single-scene aggregate refresh produced non-empty `combined_segmentation_runs.csv`.
- `blocker_summary.json` was rewritten from the stale blocked state to `cleared_no_active_blocker` at `2026-03-31T09:17:38+08:00`.

### Phase 2: representative-set refresh

Executed in two parts:

1. Full rerun executed for `00829-QaLdnwvtxbs`
2. Existing successful full-scene outputs for `00843-DYehNKdT76V` and `00862-LT9Jq6dN3Ea` were repaired from persisted `floor_diagnostics_summary.json` segmentation diagnostics using the new repair utility

Why this is acceptable:

- The issue was instrumentation serialization, not mapper semantics.
- `00843` and `00862` already had successful full representative runs on disk.
- Their saved floor diagnostics retained the authoritative per-run segmentation records, so deterministic scene-artifact repair was sufficient and avoided unnecessary recomputation.

Final representative-set results after refresh:

- `00829-QaLdnwvtxbs`: `segmentation_run_count = 20`
- `00843-DYehNKdT76V`: `segmentation_run_count = 29`
- `00862-LT9Jq6dN3Ea`: `segmentation_run_count = 76`
- Aggregate combined segmentation rows: `125`
- Aggregate blocker status: `cleared_no_active_blocker`

## 5. Expected outputs after the fix

Correct per-scene output state:

- `logs/runtime_instrumentation/per_profiled_frame.csv` exists and is non-empty
- `logs/runtime_instrumentation/segmentation_runs.csv` exists and is non-empty
- `logs/runtime_instrumentation/history_scope.csv` exists and is non-empty
- `logs/runtime_instrumentation/vector_map_export_calls.csv` exists and is non-empty
- `logs/runtime_instrumentation/summary.json` exists and reports a non-zero `segmentation_run_count` when segmentation actually occurred

Correct aggregate output state:

- `combined_per_profiled_frame.csv` exists and is non-empty
- `combined_segmentation_runs.csv` exists and is non-empty
- `combined_history_scope.csv` exists and is non-empty
- `combined_vector_map_export_calls.csv` exists and is non-empty
- `aggregate_summary.json` exists and includes refreshed per-scene `segmentation_run_count` values
- `blocker_summary.json` exists and says `cleared_no_active_blocker` after a successful refresh

Current verified counts:

- `00829` scene segmentation CSV lines: `21` total (`20` data rows + header)
- `00843` scene segmentation CSV lines: `30` total (`29` data rows + header)
- `00862` scene segmentation CSV lines: `77` total (`76` data rows + header)
- aggregate segmentation CSV lines: `126` total (`125` data rows + header)

## 6. Reproducible commands

### Light validation rerun

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00829-QaLdnwvtxbs \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --max-frames 130
```

### Aggregate analysis refresh after light validation

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --output-root ./world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation \
  --report-out ./runtime_topology_instrumented_report.md \
  --sequence-ids 00829-QaLdnwvtxbs
```

### Scene-artifact repair for already successful full runs

```bash
PYTHONPATH=. python3 stage_a_eval/repair_runtime_instrumentation_scene_outputs.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --sequence-ids 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

### Full representative rerun commands

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00829-QaLdnwvtxbs \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

### Final aggregate refresh

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --output-root ./world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation \
  --report-out ./runtime_topology_instrumented_report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```
