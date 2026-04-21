# Paper Experiment Enablement Update

Date: 2026-04-20

## Summary

This update keeps the project on the paper-safe Stage-A backend scope and makes the smallest high-impact changes needed to unblock:

- rerunnable segmentation-cadence ablations
- rerunnable BoxFusion on/off ablations
- an optional broader-history fusion-scope baseline
- direct compact evaluation on the frozen HM3D bundles that are actually present in this workspace
- paper-facing CSV/JSON aggregation for frozen-scene runtime, storage, public topology, working-vs-committed counts, and evaluator success

The default committed/public semantics were not changed.

## What Changed

### 1. Minimal ablation controls

Added two small Stage-A CLI ablation switches:

- `stage_a_demo.py --box-fusion-mode {config,on,off}`
- `stage_a_demo.py --history-scope-mode {selective_floor_aware,broad_history}`

Implementation details:

- `--box-fusion-mode off` forces the existing `cfg["box_fusion"]["use"]` path off without changing the default YAML.
- `--history-scope-mode broad_history` disables the current selective floor-aware candidate mask and uses the full retained history as a controlled broader-history baseline.
- default behavior remains `selective_floor_aware` plus the existing committed/public export semantics.

Paper experiments unblocked:

- segmentation cadence sweep
- BoxFusion / semantic association on-off reruns
- optional selective-history vs broader-history comparison

### 2. Compact evaluator now accepts current frozen roots directly

Extended `stage_a_eval/run_backend_eval.py` so it can discover scenes directly from:

- repeated `--scene-root`
- recursive `--manifest-glob`

It now automatically filters the task sheet to the discovered sequences, so the current split frozen layout under `runtime_stage1_frozen_evidence/...` runs cleanly without relying on the stale canonical root in `stage_a_eval/scene_registry.json`.

While doing this work, a real evaluator blocker surfaced and was fixed:

- `boxfusion/vln_closed_loop.py` referenced `_canonical_room_id` and `display_floor_label` without importing them.
- That bug caused execute-style compact-eval tasks to crash even after the root mismatch was bypassed.

Paper experiments unblocked:

- compact route/query evaluation on the four frozen HM3D bundles in this workspace
- scene-level query/route success tables for the frozen paper subset

### 3. Paper-facing aggregation scripts

Added:

- `stage_a_eval/aggregate_paper_scene_results.py`
- `stage_a_eval/run_paper_ablation_harness.py`

`aggregate_paper_scene_results.py` writes compact per-scene CSV/JSON summaries covering:

- runtime
- artifact bytes
- public room count
- public edge count
- working room count
- working edge count
- withheld room count
- withheld edge count
- object label count
- optional merged compact-eval success rates

`run_paper_ablation_harness.py` builds and optionally executes a small condition matrix over:

- `--room-seg-intervals`
- `--box-fusion-modes`
- `--history-scope-modes`

and writes command matrices plus compact per-condition CSV/JSON summaries.

Paper experiments unblocked:

- paper-friendly frozen-bundle summary tables
- ablation command generation and aggregation

## Files Modified

Source changes:

- `stage_a_demo.py`
- `demo.py`
- `boxfusion/backend_eval_scaffold.py`
- `stage_a_eval/run_backend_eval.py`
- `boxfusion/vln_closed_loop.py`
- `stage_a_eval/aggregate_paper_scene_results.py`
- `stage_a_eval/run_paper_ablation_harness.py`

Deliverables added:

- `docs/paper_experiment_enablement_update.md`
- `docs/paper_experiment_enablement_update.csv`

## Exact Commands To Run

### A. Compact evaluation on the current frozen HM3D bundles

```bash
python3 stage_a_eval/run_backend_eval.py \
  --manifest-glob 'runtime_stage1_frozen_evidence/**/manifest.json' \
  --tasks stage_a_eval/backend_tasks_v0_1.jsonl \
  --report-root stage_a_eval/output/backend_eval_frozen_current
```

Key outputs:

- `stage_a_eval/output/backend_eval_frozen_current/backend_eval_results.json`
- `stage_a_eval/output/backend_eval_frozen_current/aggregate_summary.md`
- `stage_a_eval/output/backend_eval_frozen_current/scene_runtime_summary.csv`
- `stage_a_eval/output/backend_eval_frozen_current/scene_task_breakdown.csv`

Observed smoke result on the current four frozen scenes:

- 44 tasks over `00824`, `00829`, `00843`, `00862`
- task success `43/44`
- route found `35/36`

### B. Frozen-scene paper summary table generation

```bash
python3 stage_a_eval/aggregate_paper_scene_results.py \
  --manifest-glob 'runtime_stage1_frozen_evidence/**/manifest.json' \
  --backend-eval-results stage_a_eval/output/backend_eval_frozen_current/backend_eval_results.json \
  --report-root stage_a_eval/output/paper_scene_results_frozen_with_eval
```

Key outputs:

- `stage_a_eval/output/paper_scene_results_frozen_with_eval/paper_scene_summary.json`
- `stage_a_eval/output/paper_scene_results_frozen_with_eval/paper_scene_summary.csv`

### C. Dry-run a paper ablation matrix

```bash
python3 stage_a_eval/run_paper_ablation_harness.py hm3d \
  --model-path <checkpoint.pt> \
  --config config/hm3d.yaml \
  --seqs 00843-DYehNKdT76V \
  --room-seg-intervals 50 100 \
  --box-fusion-modes on off \
  --history-scope-modes selective_floor_aware broad_history \
  --output-root stage_a_eval/output \
  --experiment-name paper_ablation_00843 \
  --dry-run
```

Key outputs:

- `stage_a_eval/output/paper_ablation_00843/commands.json`
- `stage_a_eval/output/paper_ablation_00843/ablation_summary.json`
- `stage_a_eval/output/paper_ablation_00843/ablation_summary.csv`

### D. Execute a real segmentation-cadence rerun

```bash
python3 stage_a_eval/run_paper_ablation_harness.py hm3d \
  --model-path <checkpoint.pt> \
  --config config/hm3d.yaml \
  --seqs 00843-DYehNKdT76V \
  --room-seg-intervals 50 100 150 \
  --box-fusion-modes on \
  --history-scope-modes selective_floor_aware \
  --output-root stage_a_eval/output \
  --experiment-name seg_interval_00843 \
  --core-only \
  --quiet
```

### E. Execute a BoxFusion on/off rerun pair

```bash
python3 stage_a_eval/run_paper_ablation_harness.py hm3d \
  --model-path <checkpoint.pt> \
  --config config/hm3d.yaml \
  --seqs 00843-DYehNKdT76V \
  --room-seg-intervals 100 \
  --box-fusion-modes on off \
  --history-scope-modes selective_floor_aware \
  --output-root stage_a_eval/output \
  --experiment-name boxfusion_toggle_00843 \
  --core-only \
  --quiet
```

### F. Execute the optional broader-history baseline

```bash
python3 stage_a_eval/run_paper_ablation_harness.py hm3d \
  --model-path <checkpoint.pt> \
  --config config/hm3d.yaml \
  --seqs 00843-DYehNKdT76V \
  --room-seg-intervals 100 \
  --box-fusion-modes on \
  --history-scope-modes selective_floor_aware broad_history \
  --output-root stage_a_eval/output \
  --experiment-name history_scope_00843 \
  --core-only \
  --quiet
```

## What Was Intentionally Left Unchanged

- default committed/public export semantics
- overall Stage-A backend architecture
- room finalization theory and publication policy
- any move toward full RAG-SLAM, LLM reasoning, BEV planning, control, or embodied navigation
- any new external GT alignment or annotation tooling
- world-model replay evaluation logic itself

## What Still Remains Blocked

- Actual rerun ablation numbers still require access to the real checkpoint, dataset root, and compute budget.
- Replay-backed `world_model_eval.py` for a new paper rerun still needs at least one fresh non-core-only HM3D rerun that emits `logs/timeline.json`.
- No new external-GT room/topology accuracy tooling was added, so paper claims should stay on internal artifact/query/routing/runtime/storage evidence.
- One frozen-scene compact-eval miss remains in the current subset: `00829` is weaker than the other three scenes and should still be treated as a sanity scene rather than the main result.

## Verification

Verified in this workspace:

- `python3 -m py_compile stage_a_demo.py demo.py stage_a_eval/run_backend_eval.py stage_a_eval/aggregate_paper_scene_results.py stage_a_eval/run_paper_ablation_harness.py boxfusion/backend_eval_scaffold.py boxfusion/vln_closed_loop.py`
- `python3 stage_a_eval/run_backend_eval.py --manifest-glob 'runtime_stage1_frozen_evidence/**/manifest.json' --tasks stage_a_eval/backend_tasks_v0_1.jsonl --report-root stage_a_eval/output/backend_eval_frozen_current`
- `python3 stage_a_eval/aggregate_paper_scene_results.py --manifest-glob 'runtime_stage1_frozen_evidence/**/manifest.json' --backend-eval-results stage_a_eval/output/backend_eval_frozen_current/backend_eval_results.json --report-root stage_a_eval/output/paper_scene_results_frozen_with_eval`
- `python3 stage_a_eval/run_paper_ablation_harness.py hm3d --model-path ./missing_model.ckpt --config config/hm3d.yaml --seqs 00843-DYehNKdT76V --room-seg-intervals 50 100 --box-fusion-modes on off --history-scope-modes selective_floor_aware broad_history --dry-run --output-root stage_a_eval/output --experiment-name smoke_ablation`
- `python3 boxfusion/test_artifact_contract.py`
- `python3 boxfusion/test_world_model_eval.py`
