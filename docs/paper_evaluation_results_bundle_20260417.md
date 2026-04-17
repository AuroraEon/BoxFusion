# Paper-Oriented Evaluation And Results Bundle

Date: 2026-04-17

## Scope

This package summarizes the current BoxFusion indoor RGB-D RSLG-SLAM backend using artifacts that already exist in the repo workspace.
It does not change runtime logic, committed/public semantics, ROS consumer APIs, or the lightweight room-graph VLN pipeline.

Validated in this turn:

- cross-sequence metrics were collected from the frozen bundles for `00843`, `00824`, `00862`, and `00829`
- the public ROS query backend was revalidated on the primary manifest-backed bundle and saved to `runtime_export_validation/paper_ros_query_validation_00843.json`
- the runtime export coordinator latest-pointer path was revalidated and saved to `runtime_export_validation/paper_eval_validation_report.json`

Reused existing frozen demo evidence:

- room-graph VLN HTML/JSON outputs already present under each scene `final/` directory

## Experimental Setup

Artifact sources:

- primary scene:
  `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V`
- supporting scene:
  `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi`
- reserve high-complexity scene:
  `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`
- sanity cross-check scene:
  `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs`

Metrics reported here come from:

- `logs/summary.json`
- `logs/room_commit_diagnosis_v0_1.json`
- `logs/topology_query_report.json`
- `logs/vertical_transition_evidence.json`
- saved room-graph VLN JSONs under `final/`

Public-consumption contract used for all downstream claims:

- authoritative public input: manifest-backed committed/public bundle
- public topology source: `logs/topology_v0_1.json`
- semantic room summary source: `logs/committed_room_world_model_v0_1.json`
- working/lifecycle artifacts remain debug-only and non-default

## Sequence Comparison

CSV asset:

- `docs/paper_sequence_comparison_20260417.csv`

| Sequence | Role | Frames | Runtime (s) | Avg FPS | Diagnosis | Public rooms | Public edges | Room transitions | Explicit route | Semantic route |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | --- |
| `00843-DYehNKdT76V` | primary paper/demo | 2710 | 783.198 | 3.46 | `public_committed_rooms_available` | 11 | 25 | 18 | yes, `room_11 -> room_7 -> room_13` | yes, `room_11 -> room_7 -> room_3` |
| `00824-Dd4bFSTQ8gi` | supporting paper/demo | 2253 | 450.378 | 5.002 | `public_committed_rooms_available` | 8 | 28 | 12 | yes, `room_8 -> room_11 -> room_7 -> room_14 -> room_16` | yes, `room_8 -> room_11 -> room_7 -> room_15` |
| `00862-LT9Jq6dN3Ea` | reserve high-complexity | 7498 | 2260.149 | 3.317 | `public_committed_rooms_available` | 30 | 107 | 63 | yes, `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3` | yes, `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21` |
| `00829-QaLdnwvtxbs` | sanity cross-check | 1804 | 552.318 | 3.266 | `public_committed_rooms_available` | 2 | 2 | 11 | yes, `room_3 -> room_7` | yes, `room_3 -> room_7` |

Role assignment used in this package:

- primary paper/demo sequence: `00843-DYehNKdT76V`
- supporting paper/demo sequence: `00824-Dd4bFSTQ8gi`
- reserve high-complexity sequence: `00862-LT9Jq6dN3Ea`
- sanity cross-check sequence: `00829-QaLdnwvtxbs`

## Downstream Demo Success

CSV asset:

- `docs/paper_demo_success_matrix_20260417.csv`

| Sequence | Public ROS bundle status | Explicit room-goal demo | Semantic room-summary demo | Next hop | Cross-floor evidence |
| --- | --- | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | revalidated this turn | success | success | `Go next to room_7.` | yes |
| `00824-Dd4bFSTQ8gi` | frozen bundle available | success | success | `Go next to room_11.` | no |
| `00862-LT9Jq6dN3Ea` | frozen bundle available | success | success | `Go next to room_33.` | yes |
| `00829-QaLdnwvtxbs` | frozen bundle available | success | success | `Go next to room_7.` | no |

Primary downstream validation commands executed in this workspace:

```bash
python3 stage_a_ros_query_server_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json \
  --json-out runtime_export_validation/paper_ros_query_validation_00843.json
```

```bash
python3 stage_a_runtime_export_coordinator_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --coordination-root runtime_export_validation/paper_eval_validation \
  --json-out runtime_export_validation/paper_eval_validation_report.json
```

Observed primary ROS-facing examples:

- route-to-room success:
  `room_11 -> room_7 -> room_13`
- route-to-object success in saved validation:
  `room_11 -> room_7 -> room_3 -> room_2`
- coordinator latest-pointer query path:
  `runtime_export_validation/paper_eval_validation/latest` and `latest_manifest.json`

## Observations

- The current public/committed surface is already strong enough for a paper mainline: all four checked sequences expose non-empty public topologies and both explicit and semantic room-level routing succeed on the saved bundles.
- `00843` remains the clearest paper/demo mainline because it combines a non-trivial 11-room public graph with validated ROS public-query entry, successful explicit and semantic VLN demos, and supported cross-floor evidence.
- `00824` is the best supporting sequence because it is cheaper to regenerate than `00843` or `00862` while still preserving an 8-room, 28-edge public graph with successful explicit and semantic routing.
- `00862` is the reserve high-complexity scene because it shows the richest public graph and strongest cross-floor complexity, but it is much heavier to rerun and less lightweight for a compact advisor-style walkthrough.
- `00829` is a useful sanity cross-check because it shows that the downstream demos still work even when only two public rooms survive the committed/public filter.

## Minimal Ablation Framing

Small paper-facing comparisons already supported by the current state:

- committed/public routing surface vs lifecycle/debug surface:
  use the public contract note to emphasize separation rather than unsafe mixing
- explicit room-goal routing vs semantic room-summary routing:
  both succeed on the same authoritative public graph, with semantic selection restricted to public rooms
- smaller/simple vs richer multi-room sequence:
  `00829` provides the minimal cross-check, while `00843` and `00824` provide clearer paper/demo graphs and `00862` provides the high-complexity reserve case

## Limitations

- This package reports only what is already grounded in frozen artifacts and the small validation passes above; it does not claim online navigation control, BEV planning, or controller-level execution.
- ROS public-query validation was rerun on the primary scene in this turn; the other three sequences are summarized from frozen bundle artifacts rather than a fresh ROS launch in this turn.
- `00862` demonstrates valuable high-complexity behavior, but its cost and graph density make it better suited as a reserve evaluation scene than as the default supporting walkthrough.
- The current paper evidence is room-graph-centric. It does not attempt broader detector changes, backend refactors, or new downstream surfaces.

## Related Notes

- public contract note:
  `docs/paper_public_authoritative_contract_20260417.md`
- downstream utility summary:
  `docs/paper_downstream_utility_summary_20260417.md`
