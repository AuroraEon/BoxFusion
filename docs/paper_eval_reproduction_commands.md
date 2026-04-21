# Paper Eval Reproduction Commands

Date: 2026-04-18

This note lists the minimal commands needed to refresh or revalidate the frozen paper package.
Use them only if a table entry or demo asset must be regenerated.

## 1. Primary public-query validation

```bash
python3 stage_a_ros_query_server_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json \
  --json-out runtime_export_validation/paper_ros_query_validation_00843.json
```

## 2. Primary coordinator/latest validation

```bash
python3 stage_a_runtime_export_coordinator_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --coordination-root runtime_export_validation/paper_eval_validation \
  --json-out runtime_export_validation/paper_eval_validation_report.json
```

## 3. Regenerate a room-goal demo page from committed/public artifacts

Primary scene:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --goal-room room_13 \
  --html-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.html \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.json
```

Supporting scene:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi \
  --start-room room_8 \
  --goal-room room_16 \
  --html-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_room_16.html \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_room_16.json
```

## 4. Regenerate a semantic room-summary demo page from committed/public artifacts

Primary scene:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --semantic-target couch \
  --html-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.html \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json
```

Supporting scene:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi \
  --start-room room_8 \
  --semantic-target bathtub \
  --html-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_bathtub.html \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_bathtub.json
```

## 5. Rebuild the advisor-facing bundle

```bash
python3 stage_a_room_graph_vln_demo_bundle.py \
  --spec demo/room_graph_vln_advisor_bundle_20260417.json \
  --output-root runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417
```

## 6. Smallest-cost sequence reruns if a frozen scene must be refreshed

Supporting scene:

```bash
python3 stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00824-Dd4bFSTQ8gi \
  --output-root ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

Primary scene:

```bash
python3 stage_a_demo.py hm3d \
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

Reserve scene:

```bash
python3 stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

Sanity scene:

```bash
python3 stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00829-QaLdnwvtxbs \
  --output-root ./runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

## Notes

- Main paper/default downstream consumption stays on committed/public artifacts only.
- The current freeze does not require new reruns unless a frozen artifact is missing or a table cell must be refreshed.
- `00824` is the preferred smallest-cost rerun if only one supporting sequence needs to be regenerated.
