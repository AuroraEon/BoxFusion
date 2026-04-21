# 00862 Parity Triage: Capture-Stride Follow-Up

Date: 2026-04-21

## Scope

This follow-up stayed narrow by design:

- no broad performance hunting
- no detector / CLIP / model-stack changes
- no multi-GPU or CUDA-topology changes
- no public-contract changes
- no sidecar promotion
- no Candidate 2 work; `BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue` remains opt-in only

The only required runtime experiment was a full-scene `00862-LT9Jq6dN3Ea` rerun with the adopted recipe held fixed except `--capture-stride 25` instead of `100`.

## Exact Roots Compared

- Frozen reference root:
  `/home/ami/zn_ws/BoxFusion/runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`
- Already-known adopted-path full-scene root (`capture-stride 100`):
  `/home/ami/zn_ws/BoxFusion/stage_a_eval/output/performance_freeze_closure_00862_full/00862-LT9Jq6dN3Ea`
- New follow-up full-scene root (`capture-stride 25`):
  `/home/ami/zn_ws/BoxFusion/stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea`

## Exact Commands Used

Shared preamble:

```bash
export PATH=/usr/local/cuda-12.4/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.4
```

Full-scene parity-triage rerun:

```bash
/usr/bin/time -p -o stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea_wall_time.txt \
  env CUDA_VISIBLE_DEVICES=0 BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1 \
  /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config config/hm3d.yaml \
  --seq 00862-LT9Jq6dN3Ea \
  --device cuda \
  --capture-stride 25 \
  --core-only \
  --suppress-service-debug-artifacts \
  --room-seg-interval 100 \
  --runtime-profile-interval 25 \
  --output-root stage_a_eval/output/00862_parity_triage_capture_stride25 \
  --quiet
```

Explicit frozen reserve-scene query replay on the new root:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea \
  --start-room room_40 \
  --goal-room room_3 \
  --json-out stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_room_3.json
```

Semantic frozen reserve-scene query replay on the new root:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea \
  --start-room room_40 \
  --semantic-target bathtub \
  --json-out stage_a_eval/output/00862_parity_triage_capture_stride25/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_bathtub.json
```

## Artifact Comparison Summary

| variant | capture stride | suppress debug artifacts | candidate index cache | duration sec | wall time real sec | public rooms | public edges | object labels | `room_40` in `topology_v0_1.json` | `room_40` in `committed_room_world_model_v0_1.json` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| frozen reference | historical reference | reference bundle | reference bundle | `2260.149` | n/a | `30` | `107` | `102` | yes | yes |
| adopted full scene | `100` | yes | yes | `2175.213` | `2209.49` | `29` | `104` | `101` | no | no |
| follow-up full scene | `25` | yes | yes | `2056.453` | `2093.08` | `30` | `107` | `102` | yes | yes |

## What Changed At The Artifact Level

### Frozen reference vs adopted `capture-stride 100`

- `room_40` was the only public room present in frozen but missing from the adopted `capture-stride 100` exports.
- That removal broke the exact committed/public links that frozen reserve-scene routing depends on:
  - `room_33 -> room_40` (`adjacent`, `transition`) missing
  - `room_34 -> room_40` (`adjacent`, `possible_connection`) missing
- The nearby route rooms `room_33`, `room_34`, `room_23`, `room_12`, `room_15`, `room_21`, and `room_3` all remained present.
- The lifecycle artifact from the adopted `capture-stride 100` run still contained `room_40`, so the break was not “scene never observed”; it was a final committed/public export parity break.

### Adopted `capture-stride 25` follow-up

- Changing only `--capture-stride 100 -> 25` restored `room_40` in both:
  - `logs/topology_v0_1.json`
  - `logs/committed_room_world_model_v0_1.json`
- The restored `room_40` neighbor structure matched the frozen reference:
  - `room_40` neighbors: `room_33`, `room_34`
  - `room_33` neighbors include `room_40`
  - `room_34` neighbors include `room_40`
- The public topology room set and public topology edge set from the new `capture-stride 25` run matched the frozen reference exactly.
- The committed room-world room set and committed adjacency set from the new `capture-stride 25` run matched the frozen reference exactly.

## Query Behavior

Frozen reserve-scene queries on the new `capture-stride 25` root became valid again.

Explicit query:

- query: `room_40 -> room_3`
- status on adopted `capture-stride 100`: `start_room_not_found`
- status on follow-up `capture-stride 25`: success
- route on follow-up: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3`
- next hop on follow-up: `room_33`

Semantic query:

- query: `room_40 -> bathtub`
- status on adopted `capture-stride 100`: `start_room_not_found`
- status on follow-up `capture-stride 25`: success
- resolved goal room on follow-up: `room_21`
- route on follow-up: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21`
- next hop on follow-up: `room_33`

## Smallest Concrete Explanation

Yes: the `00862` mismatch is explained primarily by `capture-stride`.

The narrowest evidence-backed explanation is:

- with the adopted recipe at `--capture-stride 100`, the final committed/public export drops `room_40`
- dropping `room_40` removes the committed/public `room_33` / `room_34` links that the frozen reserve-scene routes start from
- holding every other adopted recipe factor fixed and changing only to `--capture-stride 25` restores `room_40`, restores those exact public links, and restores both frozen reserve-scene queries

Because `--suppress-service-debug-artifacts` stayed enabled for both adopted runs while parity changed only with `capture-stride`, this follow-up found no evidence that `--suppress-service-debug-artifacts` is the cause.

## Residual Non-Route Delta

One smaller committed/public delta remains even after the successful `capture-stride 25` rerun:

- frozen `object_count`: `469`
- follow-up `capture-stride 25` `object_count`: `468`

The missing public object was `obj_191` (`light`) in `room_15`. This did not change:

- public room count
- public edge count
- object label count
- `room_40` presence
- explicit `room_40 -> room_3`
- semantic `room_40 -> bathtub`

So this object-level delta does not explain the route break that motivated this triage.

## Whether Further Work Is Justified

No additional parity hunting is justified from this follow-up.

The requested highest-value single-factor check already explains the route break, and the stop condition was met:

- `room_40` restored
- both frozen reserve-scene queries valid again
- committed/public room and edge structure restored to frozen parity

The only remaining decision is policy, not investigation:

- either continue using the already-frozen `00862` reserve bundle as-is
- or encode a scene-specific `00862` runtime exception that keeps `--capture-stride 25`

## Final Recommendation

use scene-specific recipe for 00862
