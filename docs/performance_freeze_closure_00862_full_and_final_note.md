# Performance Freeze Closure: 00862 Full-Scene Adopted-Path Validation And Final Note

Date: 2026-04-21

## Scope

This was the final narrow performance-closure step requested for the current Stage-A backend:

- single GPU
- `CUDA_VISIBLE_DEVICES=0`
- `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
- `--capture-stride 100`
- `--core-only`
- `--suppress-service-debug-artifacts`
- `--room-seg-interval 100`
- `--runtime-profile-interval 25`

No new optimization direction was opened. No detector / CLIP / model-stack changes were made. No multi-GPU, CUDA-topology, export-contract, sidecar-authority, Candidate-2, caching-project, or broad refactor work was done.

## Exact Command Used

Shared preamble:

```bash
export PATH=/usr/local/cuda-12.4/bin:$PATH
export CUDA_HOME=/usr/local/cuda-12.4
```

Full-scene adopted-path validation:

```bash
/usr/bin/time -p -o stage_a_eval/output/performance_freeze_closure_00862_full/00862-LT9Jq6dN3Ea_wall_time.txt \
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
  --output-root stage_a_eval/output/performance_freeze_closure_00862_full \
  --quiet
```

## Exact Scene Root And Output Root

- Dataset scene root: `/home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea`
- Output root: `/home/ami/zn_ws/BoxFusion/stage_a_eval/output/performance_freeze_closure_00862_full`
- Output scene root: `/home/ami/zn_ws/BoxFusion/stage_a_eval/output/performance_freeze_closure_00862_full/00862-LT9Jq6dN3Ea`
- Current reference scene root used for comparison: `/home/ami/zn_ws/BoxFusion/runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`

## Measured Timing Numbers

For the new full-scene adopted-path run, Stage-5 means were read from:

- `logs/runtime_instrumentation/summary.json`

Full-run duration was read from:

- `logs/summary.json -> duration_sec`
- `/usr/bin/time -p`

### New adopted-path full-scene result

| metric | value |
| --- | ---: |
| `stage5_candidate_floor_assignment_eval_sec` mean | `0.763562` |
| `stage5_candidate_mask_prep_sec` mean | `0.781667` |
| `stage5_total_sec` mean | `1.381908` |
| `duration_sec` | `2175.213` |
| `/usr/bin/time -p real` | `2209.49` |
| processed frames | `7498` |
| average FPS | `3.447` |

## Committed/Public Counts And Artifact Footprint

For this adopted path, the public room / edge / label counts were taken from `logs/topology_query_report.json`, because `--suppress-service-debug-artifacts` intentionally omits the working-vs-committed debug reports.

### New adopted-path full-scene counts

| metric | value |
| --- | ---: |
| floor count | `3` |
| public room count | `29` |
| public edge count | `104` |
| anchor count | `65` |
| object count | `457` |
| object label count | `101` |
| vertical transitions | `2` |

Mandatory committed/public artifacts present:

- `manifest.json`
- `logs/summary.json`
- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/committed_room_world_snapshot_v0_1.json`
- `logs/vertical_transition_evidence.json`
- `logs/floor_diagnostics_summary.json`
- `logs/online_topology_lifecycle_v0_1.json`

Expected because of `--suppress-service-debug-artifacts`, and therefore not a blocker by themselves:

- `logs/room_scoped_runtime_state_v0_1.json` absent
- `logs/final_vector_map_snapshot.json` absent
- `logs/working_topology_v0_1.json` absent
- `logs/working_vs_committed_topology_report_v0_1.json` absent
- `logs/working_vs_committed_topology_timeline_v0_1.json` absent
- `logs/working_vs_committed_topology_timeline_v0_1.md` absent
- `logs/room_commit_diagnosis_v0_1.json` absent
- `logs/room_commit_diagnosis_v0_1.md` absent

## Route / Query Parity Notes

Frozen reserve-scene reference behavior:

- explicit query `room_40 -> room_3`: accepted
- route: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3`
- next hop: `room_33`
- semantic query `room_40 -> bathtub`: accepted
- resolved semantic goal room: `room_21`
- route: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21`
- next hop: `room_33`

New adopted-path full-scene behavior:

- explicit query `room_40 -> room_3`: failed
- failure mode: `start_room_not_found`
- semantic query `room_40 -> bathtub`: failed
- failure mode: `start_room_not_found`
- direct cause: `room_40` is absent from the new `logs/topology_v0_1.json`
- the same issue is present in the new `logs/committed_room_world_model_v0_1.json`; this is not only a query-wrapper issue
- `bathtub` semantics still exist and still map to `room_21`; the parity break is the missing start room, not a missing bathtub semantic target

Generated parity-check outputs for the new scene root:

- `stage_a_eval/output/performance_freeze_closure_00862_full/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_room_3.json`
- `stage_a_eval/output/performance_freeze_closure_00862_full/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_bathtub.json`

## Comparison Against The Previous 3000-Frame Prefix Result

The earlier `00862` adopted-path result was intentionally only a `3000`-frame prefix:

| metric | previous prefix `3000` | new full scene |
| --- | ---: | ---: |
| `stage5_candidate_floor_assignment_eval_sec` mean | `0.153516` | `0.763562` |
| `stage5_candidate_mask_prep_sec` mean | `0.162948` | `0.781667` |
| `stage5_total_sec` mean | `0.469600` | `1.381908` |
| `duration_sec` | `542.866` | `2175.213` |
| processed frames | `3000` | `7498` |
| public room count | `15` | `29` |
| public edge count | `60` | `104` |
| object label count | `56` | `101` |
| query status | prefix-only shared failures; not final-state meaningful | final-state failure on frozen start room `room_40` |

Interpretation:

- the prefix validation was still useful for confirming the Stage-5 performance direction
- it was not sufficient to prove final-state public/query parity on `00862`
- the new full-scene rerun closes that uncertainty, but the answer is negative for parity against the current frozen `00862` reference

## Comparison Against The Current 00862 Reference

Current frozen reference summary:

| metric | frozen reference | new adopted full scene | delta |
| --- | ---: | ---: | ---: |
| `duration_sec` | `2260.149` | `2175.213` | `-84.936` |
| `/usr/bin/time -p real` | not recorded in frozen note | `2209.49` | n/a |
| public room count | `30` | `29` | `-1` |
| public edge count | `107` | `104` | `-3` |
| anchor count | `67` | `65` | `-2` |
| object count | `469` | `457` | `-12` |
| object label count | `102` | `101` | `-1` |
| explicit `room_40 -> room_3` | accepted | failed | parity fail |
| semantic `room_40 -> bathtub` | accepted | failed | parity fail |

Stage-5 timing comparison to the frozen reference is only partially apples-to-apples:

- the frozen reference uses an older instrumentation surface and does not expose `stage5_candidate_floor_assignment_eval_sec`
- it does expose:
  - `stage5_candidate_mask_prep_sec` mean = `1.550026`
  - `stage5_total_sec` mean = `3.288631`
- the new adopted full-scene run is lower on those two buckets:
  - `stage5_candidate_mask_prep_sec`: `1.550026 -> 0.781667`
  - `stage5_total_sec`: `3.288631 -> 1.381908`

Important conclusion:

- the remaining problem on `00862` is not a performance blocker
- it is a public/committed final-state parity blocker versus the current frozen reserve-scene reference

## Should The Benchmark/Paper Runtime Recipe Now Be Frozen?

Conservative answer: not yet as a fully closed `00862`-validated freeze.

What the evidence now supports:

- the adopted runtime assumptions are performance-strong enough
- no further broad performance hunting is justified before returning to the paper experiment track
- `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1` remains the recommended adopted-path setting
- Candidate 2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue`) remains opt-in only and should stay out of this freeze path

What the evidence does **not** support:

- claiming that the adopted full-scene `00862` run is parity-equivalent to the current frozen `00862` public bundle
- using this new `00862` root as a drop-in replacement for the frozen reserve-scene query artifacts

## What Performance Work Should Be Explicitly Stopped Here

Stop here:

- no new optimization direction
- no sidecar promotion
- no new data-path caching project
- no Candidate-2 sweep
- no detector / CLIP / model experiments
- no advisor-bundle regeneration
- no broad benchmark redesign
- no C++ / native-port work

These are not justified by the evidence collected here.

## What Should Be Explicitly Deferred

If the team wants a completely frozen benchmark/paper recipe that includes `00862` as full adopted-path confirmation, the remaining work is a narrow non-performance follow-up:

- either accept that `00862` stays as the older frozen reserve-scene reference and do not claim the new adopted-path rerun as parity-equivalent
- or do one small targeted `00862` public/committed parity follow-up focused on the missing `room_40`, without reopening performance hunting

That follow-up is about artifact parity, not speed.

## Final Recommendation

Performance recommendation:

- freeze the current performance work now

Overall freeze recommendation for the benchmark/paper runtime recipe:

- **one more small step remains**

Reason:

- the full-scene adopted-path `00862` rerun is fast enough and completes cleanly
- but it is not public-parity equivalent to the current frozen `00862` reference, because `room_40` disappears and the frozen reserve-scene explicit/semantic routes no longer resolve

