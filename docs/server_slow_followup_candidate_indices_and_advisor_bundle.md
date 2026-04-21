# Server Slow Followup: Candidate Indices And Advisor Bundle

This follow-up stayed narrow on the current adopted Stage-A backend path:

- single GPU
- `CUDA_VISIBLE_DEVICES=0`
- `--capture-stride 100`
- `--core-only`
- fast GT RGB resize left as-is (`BOXFUSION_FAST_GT_RGB_RESIZE=1`, default)
- `--room-seg-interval 100`
- `--suppress-service-debug-artifacts`
- no detector, CLIP, model-stack, or CUDA-topology changes

## What Was Measured

I added a narrow Stage-5 validation around repeated candidate-mask construction in `demo.py::_build_floor_scoped_candidate_mask(...)`.

New narrow runtime-instrumentation buckets:

- `stage5_candidate_floor_assignment_eval_sec`
- `stage5_candidate_readonly_room_lookup_sec`
- `stage5_candidate_floor_assignment_cache_hit_count`
- `stage5_candidate_floor_assignment_cache_miss_count`

Measurement scene and output roots:

- full baseline scene: `00843-DYehNKdT76V`
- baseline output root: `stage_a_eval/output/server_slow_candidate_indices_before_full/00843-DYehNKdT76V`
- optimized output root: `stage_a_eval/output/server_slow_candidate_indices_after_full/00843-DYehNKdT76V`
- smoke sanity roots used before the full runs:
  - `stage_a_eval/output/server_slow_candidate_indices_smoke_before/00843-DYehNKdT76V`
  - `stage_a_eval/output/server_slow_candidate_indices_smoke_after/00843-DYehNKdT76V`

Full-run commands used for the A/B comparison:

```bash
export PATH=/usr/local/cuda-12.4/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.4

CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=0 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00843-DYehNKdT76V \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --output-root stage_a_eval/output/server_slow_candidate_indices_before_full \
  --quiet

CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00843-DYehNKdT76V \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --output-root stage_a_eval/output/server_slow_candidate_indices_after_full \
  --quiet
```

## Was Repeated Candidate-Mask Construction Meaningful?

Yes. On the adopted full-sequence `00843` path, repeated floor-assignment work inside Stage-5 candidate-mask construction was a real remaining host-side cost.

Full-run timing comparison from `logs/runtime_instrumentation/summary.json`:

| narrow bucket | before | after | delta | relative |
| --- | ---: | ---: | ---: | ---: |
| `stage5_candidate_floor_assignment_eval_sec` mean | 0.112976 s | 0.062542 s | -0.050434 s | -44.64% |
| `stage5_candidate_floor_assignment_eval_sec` p95 | 0.302613 s | 0.154266 s | -0.148347 s | -49.02% |
| `stage5_candidate_readonly_room_lookup_sec` mean | 0.000358 s | 0.000355 s | -0.000003 s | negligible |
| `stage5_candidate_mask_prep_sec` mean | 0.118338 s | 0.070183 s | -0.048155 s | -40.69% |
| `stage5_total_sec` mean | 0.280521 s | 0.247164 s | -0.033357 s | -11.89% |
| full run duration | 773.297 s | 453.755 s | -319.542 s | -41.32% |

Additional full-run cache evidence:

- optimized run floor-assignment cache hits: `3877`
- optimized run floor-assignment cache misses: `4341`
- mean cache hits across mask-bearing profiled frames: `53.847222`
- mean cache misses across mask-bearing profiled frames: `60.291667`
- `65` profiled frames had non-zero cache hits

Conclusion:

- repeated per-object floor assignment was confirmed as the meaningful narrow bucket
- repeated readonly-room lookup was not the main lever

## What Changed

Exact files changed for this task:

- `demo.py`
- `boxfusion/runtime_instrumentation.py`
- `boxfusion/test_stage5_candidate_indices.py`
- `demo/room_graph_vln_advisor_bundle_20260417.json`
- `docs/server_slow_followup_candidate_indices_and_advisor_bundle.md`
- `docs/server_slow_followup_candidate_indices_and_advisor_bundle.csv`

Stage-5 code change:

- `demo.py` now maintains a tiny per-process Stage-5 candidate-index cache keyed by object `init_id`.
- Cached floor assignments are invalidated conservatively when the floor-state token changes.
- Reuse is local to the existing mask-builder path and does not change mask semantics.
- The readonly-room tail path was only measured more narrowly; it was not turned into a broader new authority path.

Instrumentation change:

- `boxfusion/runtime_instrumentation.py` now records and summarizes the narrow candidate-mask floor-assignment bucket separately from the broader `stage5_candidate_mask_prep_sec`.

Verification:

- direct cache regression: `boxfusion/test_stage5_candidate_indices.py`
- direct advisor-bundle smoke reuse of the existing bundle generator path
- policy smoke for the already-existing service/debug suppression switch
- `pytest` was not installed in the runtime env, so verification was executed through direct Python assertions in the BoxFusion env instead

## Flags And Switches Added

One narrow env switch was added for controlled A/B validation:

- `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE`
  - default: enabled (`1`)
  - `0` disables the maintained candidate-index reuse path
  - used only to compare the same adopted runtime path before vs after the optimization

No new CLI flags were added.

## Is The Result Strong Enough To Keep?

Yes.

Reasoning:

- the measured target was non-trivial on the adopted full path
- the optimization reduced the exact target bucket by about `44.6%` at mean and `49.0%` at p95
- the broader Stage-5 candidate-mask prep bucket also dropped materially
- the readonly-room lookup stayed negligible, so the change stayed focused on the right lever
- the change stayed local and semantics-preserving rather than broadening into a larger cache or lifecycle redesign

## Advisor Bundle Production

Existing advisor-bundle generator path inspected:

- script: `stage_a_room_graph_vln_demo_bundle.py`
- bundle builder: `boxfusion/room_graph_vln_demo_bundle.py`
- updated spec: `demo/room_graph_vln_advisor_bundle_20260417.json`

Generation command used:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo_bundle.py \
  --spec demo/room_graph_vln_advisor_bundle_20260417.json \
  --output-root runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417
```

Exact advisor output root:

- bundle root: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417`
- bundle index: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/index.html`
- bundle manifest: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/demo_bundle_manifest.json`

Scene bundle outputs generated:

- `00843-DYehNKdT76V`: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00843-dyehnkdt76v`
- `00824-Dd4bFSTQ8gi`: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00824-dd4bfstq8gi`
- `00862-LT9Jq6dN3Ea`: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00862-lt9jq6dn3ea`
- `00829-QaLdnwvtxbs`: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00829-qaldnwvtxbs`

Minimal spec update made:

- kept the existing dated spec path
- preserved the existing public query/routing consumption surface
- added only two new scenes and two queries per new scene

Four-scene advisor bundle contents:

- `00843-DYehNKdT76V`
  - explicit: `room_11 -> room_13`
  - semantic: `couch` from `room_11`
- `00824-Dd4bFSTQ8gi`
  - explicit: `room_8 -> room_16`
  - semantic: `bathtub` from `room_8`
- `00862-LT9Jq6dN3Ea`
  - explicit: `room_1 -> room_40`
  - semantic: `refrigerator` from `room_1`
- `00829-QaLdnwvtxbs`
  - explicit: `room_3 -> room_7`
  - semantic: `bathtub` from `room_3`

## What Was Intentionally Not Changed

- no broad performance hunting beyond this narrow Stage-5 validation and fix
- no detector change
- no CLIP change
- no model-stack change
- no multi-GPU change
- no CUDA-topology change
- no broad lifecycle redesign
- no new sidecar authority
- no sidecar/public contract change
- no change to Candidate 2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue` remains opt-in and untouched)
- no change to `minimal_public_topology_subset` authority or scope
