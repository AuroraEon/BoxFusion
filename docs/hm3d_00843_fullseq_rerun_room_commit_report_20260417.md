# HM3D `00843-DYehNKdT76V` Full-Sequence Rerun And Room-Commit Diagnosis

Date: 2026-04-17

## Scope

This note records the real HM3D stage-A rerun performed in the current BoxFusion environment for `00843-DYehNKdT76V`, using the existing runtime and diagnosis tooling without changing room/publication semantics.

Stable semantics kept unchanged during this work:

- public/default topology remains committed-only
- working/lifecycle rooms are not intentionally exposed through public/default outputs
- sidecar remains diagnostic/shadow-only

## Environment Used

```bash
CUDA_HOME=/usr/local/cuda
PATH=/usr/local/cuda/bin:$PATH
PYTHON=/home/ami/miniconda3/envs/boxfusion/bin/python
```

## Actual Commands Executed

### 1. Fresh full-sequence rerun executed on 2026-04-17

Only the frame cap was removed from the requested baseline. A fresh output root was used to avoid overwriting archived evidence.

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

### 2. Diagnosis executed on the new rerun

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_commit_diagnosis.py \
  ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

### 3. Comparison diagnosis executed on the pre-existing 300-frame baseline artifact

This was not rerun today. It was used only as the closest local baseline with matching `capture_stride=25`.

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_commit_diagnosis.py \
  ./codex_perf_probe/server/benchmark_full300/output/00843-DYehNKdT76V
```

## What Changed Relative To The 300-Frame-Capped Run

- Primary change: removed `--max-frames 300`
- Kept unchanged: `--room-seg-interval 100`, `--capture-stride 25`, `--video-fps 12`, `--core-only`, `--runtime-profile-interval 25`
- Operational change only: wrote the new run into `./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes`

The HM3D scene directory contains 2710 RGB/depth/pose triplets, and the current HM3D loader iterates the full directory. In this repo state, omitting `--max-frames` is supported and resulted in `processed_frames=2710`.

## Per-Run Summary

### A. Pre-existing comparable 300-frame baseline

Artifact root:

```text
./codex_perf_probe/server/benchmark_full300/output/00843-DYehNKdT76V
```

Observed summary:

- processed frames: 300
- duration: 71.637 s
- average fps: 4.188
- snapshots: 13
- parameters: `room_seg_interval=100`, `capture_stride=25`, `runtime_profile_interval=25`, `max_frames=300`

Diagnosis highlights:

- any room reached `candidate_complete`: yes, `candidate_complete_ever_count=2`
- any room committed internally: yes, `runtime_internal_committed_room_count=1`
- any room survived the committed/public filter: no, `committed_rooms_surviving_public_filter_count=0`
- public topology count reported by diagnosis JSON: `public_topology_room_count=4`
- transition evidence: `room_transition_event_count=0`

Main limiting pattern:

- the 300-frame run did produce some candidate/commit progress
- it did not carry any committed room through the committed/public survivor count
- it also showed no room-transition evidence, which is materially weaker than the full-sequence rerun

### B. Fresh full-sequence rerun executed today

Artifact root:

```text
./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

Observed summary:

- processed frames: 2710
- duration: 783.198 s
- average fps: 3.46
- snapshots: 110
- parameters: `room_seg_interval=100`, `capture_stride=25`, `runtime_profile_interval=25`, `max_frames=None`

Diagnosis highlights:

- any room reached `candidate_complete`: yes, `candidate_complete_ever_count=14`
- any room committed internally: yes, `runtime_internal_committed_room_count=14`
- any room survived the committed/public filter: yes, `committed_rooms_surviving_public_filter_count=11`
- public topology count reported by diagnosis JSON: `public_topology_room_count=11`
- transition evidence: `room_transition_event_count=18`

Most common remaining blockers on some refreshes:

- `room_currently_active`
- `room_signature_not_stable`
- `containment_not_stable`
- `gateway_structure_not_stable`
- `no_leave_like_signal`

These blockers did not prevent successful commit/public output overall.

## Comparison And Conclusion

For this sequence, removing the `300`-frame cap was enough to produce a clearly better practical outcome without changing any of the cadence parameters:

- the 300-frame baseline reached only limited completion/commit progress
- the uncapped rerun produced strong late-run transition evidence
- the uncapped rerun committed 14 rooms internally and preserved 11 through the committed/public survivor count

Conclusion:

- the original "only about 300 frames" issue was fully explained by the explicit `--max-frames 300` stop condition
- the weak room/public outcome of the comparable 300-frame run was not caused by `capture_stride`, `room_seg_interval`, or `runtime_profile_interval`, because the full-sequence rerun kept those unchanged and succeeded
- no additional parameter sweep was necessary for this task

## Best Current Configuration

Best current base for ROS demo and paper experiments on this sequence:

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

Why this is the best current choice:

- it is the smallest change from the known baseline
- it already achieves committed/public room output
- it avoids speculative tuning
- it preserves the current committed-only public/default semantics

## Demo-Ready Standardization Recommendation

Yes: there is an obvious next command to standardize on for this sequence.

Recommendation:

- standardize on the same command as above, with `--max-frames` omitted

If a future benchmark script needs a hard runtime bound for reproducibility, the safer explicit cap would be the current full sequence length rather than `300`; however, that was not required for this successful rerun.

## Files Added Or Produced

New report added:

- `docs/hm3d_00843_fullseq_rerun_room_commit_report_20260417.md`

New runtime output produced:

- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/...`
