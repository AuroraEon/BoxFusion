# Runtime Reproduction Guide

Date: 2026-04-07

This guide reproduces the current accepted runtime baseline from the current repo state.

Interpreter used below:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python
```

Important baseline note:

- The frozen baseline keeps readonly-tail shadow-reference audit OFF by default.
- Do not add `--enable-readonly-tail-reference-audit` unless you are intentionally running the audit path.

## Tier 1: fast sanity checks

### 1. Compile the live runtime files

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python -m py_compile \
  demo.py \
  stage_a_demo.py \
  boxfusion/floor_aware_room_segmenter.py \
  boxfusion/instances.py \
  boxfusion/box_fusion.py \
  boxfusion/runtime_instrumentation.py \
  tools/analyze_same_frame_object_deltas.py
```

- Rough cost: very low, usually a few seconds.
- Output path: none.
- Inspect afterward: command should exit cleanly with no traceback.
- Meaning: present-state sanity check only.

### 2. Run the floor-aware world-graph mock validation

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python boxfusion/test_floor_aware_world_graph.py
```

- Rough cost: low, usually a few seconds.
- Output path: no dedicated runtime evidence root; stdout should end with `Floor-aware world graph validated.`
- Inspect afterward: confirm the validation completes without assertion failures.
- Meaning: present-state sanity check for export reuse, changed-room rebuild logic, and anchor/object export regressions.

### 3. Run the scene-graph mock validation

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python boxfusion/test_scene_graph.py
```

- Rough cost: low, usually a few seconds.
- Output path: `./debug_mock/mock_scene_graph_bev.png`
- Inspect afterward: stdout should list anchors and finish without assertion failures.
- Meaning: present-state sanity check for scene-graph / anchor behavior.

### 4. Reanalyze the surviving compact representative bundle

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./runtime_stage1_frozen_evidence/current_partial_rerun/scenes \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial \
  --report-out ./runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

- Rough cost: low, usually under a few seconds.
- Output path: `runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/`
- Inspect afterward:
  - `aggregate_summary.json`
  - `blocker_summary.json`
  - `combined_history_scope.csv`
  - `combined_vector_map_export_calls.csv`
- Meaning: direct reproduction of a frozen accepted compact analysis, using surviving on-disk scene outputs.

## Tier 2: medium-cost accepted verification commands

These are the most important fresh representative reruns for the frozen baseline. They reproduce the compact accepted verification path from the current cleaned repo state.

### 1. Rerun `00843-DYehNKdT76V`

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --max-frames 260
```

- Rough cost: medium, about 30-40 seconds on the machine used for this freeze.
- Output path: `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V/`
- Inspect afterward:
  - `logs/summary.json`
  - `logs/runtime_instrumentation/summary.json`
  - `logs/runtime_instrumentation/history_scope.csv`
  - `logs/runtime_instrumentation/vector_map_export_calls.csv`
- What should be true:
  - `processed_frames = 260`
  - `duplicate_export_frame_count = 0`
  - history-scope labels are `current_frame_only`, `same_floor_recent_or_near_retained_history`, and `same_floor_recent_or_near_cached_room_tail_pruned_retained_history`
  - readonly-tail audit remains disabled by default
- Meaning: direct reproduction of a frozen accepted result.

### 2. Rerun `00862-LT9Jq6dN3Ea`

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --max-frames 260
```

- Rough cost: medium, about 30-40 seconds on the machine used for this freeze.
- Output path: `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00862-LT9Jq6dN3Ea/`
- Inspect afterward:
  - `logs/summary.json`
  - `logs/runtime_instrumentation/summary.json`
  - `logs/runtime_instrumentation/history_scope.csv`
  - `logs/runtime_instrumentation/vector_map_export_calls.csv`
- What should be true:
  - `processed_frames = 260`
  - `duplicate_export_frame_count = 0`
  - history-scope labels are same-floor recent-or-near labels, not `global_retained_history`
  - readonly-tail audit remains disabled by default
- Meaning: direct reproduction of a frozen accepted result.

### 3. Aggregate the two fresh reruns

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/scenes \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407 \
  --report-out ./runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/report.md \
  --sequence-ids 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

- Rough cost: low after the scene reruns finish.
- Output path: `runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/`
- Inspect afterward:
  - `aggregate_summary.json`
  - `blocker_summary.json`
  - `combined_history_scope.csv`
  - `combined_vector_map_export_calls.csv`
- What should be true:
  - `status = cleared_no_active_blocker`
  - `duplicate_export_frame_count = 0`
  - combined history-scope counter contains only same-floor recent-or-near labels plus `current_frame_only`
- Meaning: direct reproduction of the fresh accepted aggregate verification produced in this freeze task.

## Tier 3: full expensive reproduction commands

These are heavier confirmations. They are stronger than the compact freeze reruns, but they are not necessary for the compact freeze package itself.

### 1. Full representative rerun: `00829-QaLdnwvtxbs`

Command:

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

- Rough cost: expensive, historically about 6 minutes.
- Output path: `world_model_backend_outputs_v0_2_final/scenes/00829-QaLdnwvtxbs/`
- Inspect afterward:
  - `logs/summary.json`
  - `logs/runtime_instrumentation/summary.json`
  - `logs/runtime_growth_profile.csv`
- Meaning: stronger historical-style confirmation, not required for compact freeze.

### 2. Full representative rerun: `00843-DYehNKdT76V`

Command:

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

- Rough cost: expensive, historically about 8 minutes.
- Output path: `world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/`
- Inspect afterward:
  - `logs/summary.json`
  - `logs/runtime_instrumentation/summary.json`
  - `logs/runtime_growth_profile.csv`
- Meaning: stronger historical-style confirmation, not required for compact freeze.

### 3. Full representative rerun: `00862-LT9Jq6dN3Ea`

Command:

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

- Rough cost: very expensive, historically about 80 minutes.
- Output path: `world_model_backend_outputs_v0_2_final/scenes/00862-LT9Jq6dN3Ea/`
- Inspect afterward:
  - `logs/summary.json`
  - `logs/runtime_instrumentation/summary.json`
  - `logs/runtime_growth_profile.csv`
- Meaning: strongest single-scene confirmation of long-run behavior; not required for compact freeze.

### 4. Full aggregate refresh

Command:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --output-root ./world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation \
  --report-out ./runtime_topology_instrumented_report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

- Rough cost: low after the full reruns finish.
- Output path: `world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation/`
- Inspect afterward:
  - `aggregate_summary.json`
  - `blocker_summary.json`
  - `combined_history_scope.csv`
  - `combined_vector_map_export_calls.csv`
- Meaning: stronger historical-style confirmation, not required for the compact freeze package.

## Interpretation notes

- Tier 1 checks prove the current code and mock regression paths are healthy.
- Tier 2 commands are the main accepted reproduction path for this freeze.
- Tier 3 commands provide stronger confirmation but are costlier and no longer necessary to keep the frozen baseline trustworthy after cleanup.
