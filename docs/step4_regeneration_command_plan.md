# Step 4 Regeneration Command Plan

The verified paper-safe regeneration command is the current Stage-A path:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq <scene_id> \
  --output-root ./runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --suppress-service-debug-artifacts \
  --runtime-profile-interval 25 \
  --quiet
```

Environment used:

```bash
PATH=/usr/local/cuda-12.4/bin:$PATH
CUDA_HOME=/usr/local/cuda-12.4
CUDA_VISIBLE_DEVICES=0
BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1
```

The plan intentionally did not use legacy `demo.py` as the entrypoint, did not enable Tier-2 repair beyond existing config semantics, did not use SegFormer room segmentation, did not promote sidecar/debug outputs, and did not change topology construction or artifact export semantics.

| scene_id | regeneration needed | output scene root | risk level | status |
|---|---:|---|---|---|
| `00824-Dd4bFSTQ8gi` | true | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi` | `safe_current_main_path` | completed |
| `00862-LT9Jq6dN3Ea` | true | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea` | `safe_current_main_path` | completed |
| `00829-QaLdnwvtxbs` | true | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs` | `safe_current_main_path` | completed |

Run logs and wall-time files were captured under `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/run_logs/`.
