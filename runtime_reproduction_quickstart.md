# Runtime Reproduction Quickstart

Run these first if you only want the highest-signal checks.

## 1. Reanalyze the surviving compact bundle

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./runtime_stage1_frozen_evidence/current_partial_rerun/scenes \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial \
  --report-out ./runtime_stage1_frozen_evidence/final_freeze_verification/reanalyzed_current_partial/report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

What it proves:
- The surviving compact evidence bundle is still readable and sufficient for aggregate analysis.

## 2. Fresh rerun: `00843-DYehNKdT76V`

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

What it proves:
- The current cleaned repo can still regenerate accepted compact baseline evidence on a representative scene.

## 3. Fresh rerun: `00862-LT9Jq6dN3Ea`

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

What it proves:
- The accepted same-floor recent-or-near baseline still reproduces on the heavier representative compact case.

## 4. Aggregate the two fresh reruns

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/scenes \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407 \
  --report-out ./runtime_stage1_frozen_evidence/final_freeze_verification/rerun_20260407/report.md \
  --sequence-ids 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

What it proves:
- The fresh reruns aggregate cleanly, with no active blocker and no duplicate export frames in the compact accepted verification path.
