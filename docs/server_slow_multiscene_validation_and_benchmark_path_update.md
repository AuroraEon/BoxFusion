# Server Slow Multi-Scene Validation And Benchmark Path Update

Date: 2026-04-21

## Scope

This change takes the next safe but somewhat larger step on the current Stage-A backend path:

- single GPU
- `CUDA_VISIBLE_DEVICES=0`
- `--capture-stride 100`
- `--core-only`
- fast GT RGB resize left as-is
- `--room-seg-interval 100`
- `--suppress-service-debug-artifacts`

It does **not** reopen broad performance hunting, does **not** change the detector / CLIP / model stack, does **not** change CUDA topology, does **not** change the committed/public contract, does **not** make sidecars authoritative, and does **not** touch Candidate 2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue`) beyond keeping it opt-in only.

## Scenes Run

Exactly what was run in this update:

- `00824-Dd4bFSTQ8gi`: full A/B validation
- `00862-LT9Jq6dN3Ea`: narrower but meaningful A/B validation on a `3000`-frame prefix

Existing evidence reused for the recommendation:

- `00843-DYehNKdT76V`: prior full A/B validation from `docs/server_slow_followup_candidate_indices_and_advisor_bundle.md`

Why `00862` was narrowed:

- the frozen reference scene is `7498` frames and `2260.149s`
- a full A/B rerun would have been a much larger detour than the rest of this narrow task
- the `3000`-frame prefix still covers `30` room-seg intervals and produced a large enough Stage-5 sample to test whether the candidate-index result generalizes

## Exact Commands Used

Shared preamble:

```bash
export PATH=/usr/local/cuda-12.4/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.4
```

`00824-Dd4bFSTQ8gi` cache OFF:

```bash
/usr/bin/time -p -o stage_a_eval/output/server_slow_multiscene_candidate_indices_before_full/00824-Dd4bFSTQ8gi_wall_time.txt \
  env CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=0 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00824-Dd4bFSTQ8gi \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --output-root stage_a_eval/output/server_slow_multiscene_candidate_indices_before_full \
  --quiet
```

`00824-Dd4bFSTQ8gi` cache ON:

```bash
/usr/bin/time -p -o stage_a_eval/output/server_slow_multiscene_candidate_indices_after_full/00824-Dd4bFSTQ8gi_wall_time.txt \
  env CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00824-Dd4bFSTQ8gi \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --output-root stage_a_eval/output/server_slow_multiscene_candidate_indices_after_full \
  --quiet
```

`00862-LT9Jq6dN3Ea` cache OFF, narrowed `3000`-frame prefix:

```bash
/usr/bin/time -p -o stage_a_eval/output/server_slow_multiscene_candidate_indices_00862_before_3000/00862-LT9Jq6dN3Ea_wall_time.txt \
  env CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=0 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00862-LT9Jq6dN3Ea \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --max-frames 3000 \
  --output-root stage_a_eval/output/server_slow_multiscene_candidate_indices_00862_before_3000 \
  --quiet
```

`00862-LT9Jq6dN3Ea` cache ON, narrowed `3000`-frame prefix:

```bash
/usr/bin/time -p -o stage_a_eval/output/server_slow_multiscene_candidate_indices_00862_after_3000/00862-LT9Jq6dN3Ea_wall_time.txt \
  env CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00862-LT9Jq6dN3Ea \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --max-frames 3000 \
  --output-root stage_a_eval/output/server_slow_multiscene_candidate_indices_00862_after_3000 \
  --quiet
```

## Result Summary

The benchmark tables below use `logs/runtime_instrumentation/summary.json` means and `logs/summary.json -> duration_sec`.

| scene | scope | floor eval OFF | floor eval ON | delta | mask prep OFF | mask prep ON | delta | stage5 total OFF | stage5 total ON | delta | duration OFF | duration ON | delta |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `00843-DYehNKdT76V` | full | `0.112976` | `0.062542` | `-44.64%` | `0.118338` | `0.070183` | `-40.69%` | `0.280521` | `0.247164` | `-11.89%` | `773.297s` | `453.755s` | `-41.32%` |
| `00824-Dd4bFSTQ8gi` | full | `0.195208` | `0.101582` | `-47.96%` | `0.201926` | `0.110726` | `-45.17%` | `0.540034` | `0.467085` | `-13.51%` | `628.945s` | `410.565s` | `-34.72%` |
| `00862-LT9Jq6dN3Ea` | `3000`-frame prefix | `0.291410` | `0.153516` | `-47.32%` | `0.298499` | `0.162948` | `-45.41%` | `0.573963` | `0.469600` | `-18.18%` | `792.746s` | `542.866s` | `-31.52%` |

Additional wall-clock reference from `/usr/bin/time -p` for the new runs:

- `00824`: `655.17s` OFF, `437.05s` ON
- `00862` `3000`-frame prefix: `820.69s` OFF, `570.80s` ON

## Do `00824` And `00862` Confirm The `00843` Result?

Yes, with one important scope note.

- `00824` is a full-scene confirmation. It reproduces the same pattern as `00843`: roughly halved `stage5_candidate_floor_assignment_eval_sec`, materially lower `stage5_candidate_mask_prep_sec`, lower `stage5_total_sec`, and a substantially shorter end-to-end run.
- `00862` confirms the same direction on a larger scene prefix. The `3000`-frame validation shows the same Stage-5 reduction pattern and unchanged scene-level public counts across the A/B pair, but it is not a full-scene advisor/query acceptance sweep.

That is strong enough to treat the maintained candidate-index path as the recommended benchmark/paper setting on the adopted runtime path.

## Committed/Public Stability And Parity

### `00843-DYehNKdT76V`

- public counts stayed stable across OFF/ON: `11` rooms, `24` edges, `44` object labels
- explicit query stayed accepted across OFF/ON:
  - `room_11 -> room_13`
  - next hop: `room_13`
- semantic query stayed accepted across OFF/ON:
  - `couch` resolves to `room_3`
  - route: `room_11 -> room_7 -> room_3`
- public topology edge signatures matched exactly across OFF/ON
- committed room-world-model room/object/adjacency summaries matched exactly across OFF/ON
- raw `committed_room_world_model_v0_1.json` bytes differed, but there was no observed behavioral mismatch in the authoritative committed/public surface

### `00824-Dd4bFSTQ8gi`

- public counts stayed stable across OFF/ON: `7` rooms, `19` edges, `51` object labels
- explicit query stayed accepted across OFF/ON:
  - route: `room_8 -> room_3 -> room_16`
  - next hop: `room_3`
- semantic `bathtub` query failed in both OFF and ON runs
- public topology room sets and edge signatures matched across OFF/ON
- committed room-world-model room/object/adjacency summaries matched across OFF/ON
- raw topology JSON bytes and raw committed-room-world-model bytes differed across OFF/ON, but the committed/public structure and query outcomes stayed the same

Important note:

- this adopted benchmark path is **not** byte-identical to the older frozen advisor-bundle scene for `00824`
- the frozen advisor scene exposes `8` rooms / `28` edges / `52` labels from a different frozen scene root / recipe
- the A/B parity reference for this task was therefore the paired adopted-path reruns, not the older frozen demo bundle

### `00862-LT9Jq6dN3Ea`

- this was intentionally a `3000`-frame prefix, not a full-scene rerun
- public counts stayed stable across OFF/ON on that prefix: `15` rooms, `60` edges, `56` object labels
- the bundle-target explicit `room_1 -> room_40` query failed in both OFF and ON prefix runs
- the bundle-target semantic `refrigerator` query failed in both OFF and ON prefix runs
- the non-acceptance is expected on this capped prefix and does **not** indicate a cache-specific regression
- public topology bytes matched exactly across OFF/ON
- committed room-world-model room/object/adjacency summaries matched across OFF/ON
- raw committed-room-world-model bytes differed across OFF/ON, but there was no observed committed/public behavioral mismatch on the sampled prefix

## Benchmark/Paper Helper Default Update

The suppressor default is now applied conservatively in benchmark/paper-scoped paths only.

Exactly where the default was applied:

- `stage_a_eval/run_paper_ablation_harness.py`
  - now appends `--suppress-service-debug-artifacts` by default
  - new opt-out: `--materialize-service-debug-artifacts`
- `docs/paper_eval_reproduction_commands.md`
  - recommended benchmark/paper rerun commands now include `--suppress-service-debug-artifacts`

What was **not** changed:

- `stage_a_demo.py` general default semantics remain unchanged
- the suppressor is still opt-in outside the benchmark/paper helper path

Helper verification:

- dry-run harness command includes `--suppress-service-debug-artifacts` by default
- dry-run harness command omits it when `--materialize-service-debug-artifacts` is supplied

## Four-Scene Advisor Bundle State

Inspected bundle:

- spec: `demo/room_graph_vln_advisor_bundle_20260417.json`
- bundle root: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417`
- bundle manifest: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/demo_bundle_manifest.json`
- bundle index: `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/index.html`

Validated and reused as-is:

- spec scene count = manifest scene count = `4`
- scenes present and consistent:
  - `00843-DYehNKdT76V`
  - `00824-Dd4bFSTQ8gi`
  - `00862-LT9Jq6dN3Ea`
  - `00829-QaLdnwvtxbs`
- each scene bundle has:
  - `index.html`
  - `scene_bundle_summary.json`
  - both HTML/JSON query outputs declared in the manifest
- missing bundle outputs found: `0`

Result:

- no regeneration was necessary
- no public query/routing contract change was made

## What Was Intentionally Not Changed

- no detector / CLIP / model-stack change
- no multi-GPU or CUDA-topology change
- no broad lifecycle redesign
- no sidecar authority change
- no sidecar/public export contract change
- no Candidate 2 sweep or promotion
- no benchmark redesign beyond this narrow helper-path standardization
- no unnecessary advisor-bundle regeneration

## Final Recommended Benchmark/Paper Runtime Recipe

Recommended setting:

- keep `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
- keep `--suppress-service-debug-artifacts` on the benchmark/paper path
- keep `CUDA_VISIBLE_DEVICES=0`
- keep `--capture-stride 100`
- keep `--core-only`
- keep `--room-seg-interval 100`
- keep fast GT RGB resize as already adopted
- keep Candidate 2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue`) opt-in only

Recommended command pattern:

```bash
CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --device cuda \
  --capture-stride 100 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --quiet
```

Freeze recommendation:

- yes, the benchmark/paper runtime path is now stable enough to freeze on this adopted recipe
- the recommendation is supported by full-scene `00843` and `00824` validation plus a meaningful larger-scene `00862` prefix validation
- the only caveat is that `00862` was not rerun end-to-end in this turn, so this update does **not** claim full-scene advisor-query acceptance for `00862` on the adopted benchmark path
