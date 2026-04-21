# HM3D Scene Readiness Matrix

Date: 2026-04-20

## Assumption

This matrix covers the HM3D Stage-A bundles that are actually present in this workspace under `runtime_stage1_frozen_evidence/...`.

It does not treat the 8-scene plan in `stage_a_eval/scene_registry.json` as materialized evidence, because `world_model_backend_outputs_v0_2_final/scenes/` is not present in this workspace.

| scene id | sequence name | processed frames | committed/public rooms available | topology available | query usable | routing usable | semantic room-summary usable | recommended role in paper | known blocker |
| --- | --- | ---: | --- | --- | --- | --- | --- | --- | --- |
| `00843` | `00843-DYehNKdT76V` | 2710 | yes, 11 public rooms | yes, `logs/topology_v0_1.json` and `logs/topology_query_report.json` present | yes; `stage_a_topology_acceptance.py --acceptance` passes for anchor, object-id, and object-label queries | yes; explicit route `room_11 -> room_7 -> room_13` and semantic route `room_11 -> room_7 -> room_3` are frozen in `final/` | yes; `logs/committed_room_world_model_v0_1.json` present and semantic demo frozen | main result | no blocker for final-state backend claims; blocked only for replay-backed execution metrics because the frozen bundle is `core_only` and omits `logs/timeline.json` |
| `00824` | `00824-Dd4bFSTQ8gi` | 2253 | yes, 8 public rooms | yes, `logs/topology_v0_1.json` and `logs/topology_query_report.json` present | yes; topology acceptance passes for anchor, object-id, and object-label queries | yes; explicit route `room_8 -> room_11 -> room_7 -> room_14 -> room_16` and semantic route `room_8 -> room_11 -> room_7 -> room_15` are frozen in `final/` | yes; committed room world model present | secondary | no blocker for final-state backend claims; same replay-timeline omission as `00843`; use as supporting scene, not the main figure |
| `00862` | `00862-LT9Jq6dN3Ea` | 7498 | yes, 30 public rooms | yes, `logs/topology_v0_1.json` and `logs/topology_query_report.json` present | yes; topology acceptance passes for anchor, object-id, and object-label queries | yes; explicit route `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3` and semantic route `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21` are frozen in `final/` | yes; committed room world model present | secondary | no blocker for final-state backend claims; very high rerun/runtime/storage cost and dense 30-room graph make it better as a reserve or appendix scene |
| `00829` | `00829-QaLdnwvtxbs` | 1804 | yes, 2 public rooms | yes, `logs/topology_v0_1.json` and `logs/topology_query_report.json` present | yes; topology acceptance passes for anchor, object-id, and object-label queries | yes; explicit and semantic routes both succeed on `room_3 -> room_7` | yes; committed room world model present | secondary | no blocker for final-state backend claims; the public topology is very small, so this scene is best used as a sanity cross-check rather than a main result |
