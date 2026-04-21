# Paper Eval Freeze v1

Date: 2026-04-18

## Purpose

This freeze defines the minimal paper-ready evaluation package for the current BoxFusion indoor RGB-D world-model / semantic SLAM backend.

This is a paper-first backend package, not a navigation-stack package.
It does not add BEV planning, controller execution, ROS expansion, sidecar expansion, or new runtime features.

No new sequence reruns were executed for this freeze pass.
The package below is assembled from frozen artifacts already present in the repo workspace.

## Frozen Protocol

### Sequence roles

- Primary paper/demo sequence: `00843-DYehNKdT76V`
- Supporting paper/demo sequence: `00824-Dd4bFSTQ8gi`
- Reserve high-complexity sequence: `00862-LT9Jq6dN3Ea`
- Sanity cross-check sequence: `00829-QaLdnwvtxbs`

### Exact downstream evaluation scope

Only the following downstream checks are in scope:

1. Public query bundle validation
2. Explicit room-goal routing
3. Semantic room-summary routing

### Authoritative downstream contract

All downstream evaluation must read committed/public artifacts only.

Authoritative inputs:

- `manifest.json`
- `logs/topology_v0_1.json`
- `logs/committed_room_world_model_v0_1.json`

Non-authoritative by default:

- `logs/working_topology_v0_1.json`
- `logs/online_topology_lifecycle_v0_1.json`
- `logs/working_vs_committed_topology_*`
- other working / lifecycle / debug surfaces

## Package Outputs

Report:

- `docs/paper_eval_freeze_v1.md`

Table-ready files:

- `sequence_summary.csv`
- `downstream_public_query_and_vln.csv`
- `ablation_minimal.csv`
- `qualitative_assets_manifest.json`

Command manifest:

- `docs/paper_eval_reproduction_commands.md`

## Table 1: Sequence / World-Model Summary

Source file: `sequence_summary.csv`

| Sequence | Role | Frames | Public rooms | Public edges | Room transitions | Non-empty committed/public world model | Explicit route | Semantic route | Vertical-transition evidence |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | primary | 2710 | 11 | 25 | 18 | yes | yes | yes | yes, 1 transition |
| `00824-Dd4bFSTQ8gi` | supporting | 2253 | 8 | 28 | 12 | yes | yes | yes | no |
| `00862-LT9Jq6dN3Ea` | reserve | 7498 | 30 | 107 | 63 | yes | yes | yes | yes, 2 transitions |
| `00829-QaLdnwvtxbs` | sanity | 1804 | 2 | 2 | 11 | yes | yes | yes | no |

Authoritative scene roots:

- `00843`: `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V`
- `00824`: `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi`
- `00862`: `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`
- `00829`: `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs`

## Table 2: Downstream Public-Query / Room-Graph Routing

Source file: `downstream_public_query_and_vln.csv`

| Sequence | Public query bundle status | Explicit room-goal routing | Semantic room-summary routing | Example next hop | Notes |
| --- | --- | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | revalidated from `runtime_export_validation/paper_ros_query_validation_00843.json` | `room_11 -> room_7 -> room_13` | `room_11 -> room_7 -> room_3` for `couch` | `Go next to room_7.` | strongest main-paper sequence; advisor bundle present |
| `00824-Dd4bFSTQ8gi` | frozen `manifest.json` plus `logs/topology_query_report.json` present | `room_8 -> room_11 -> room_7 -> room_14 -> room_16` | `room_8 -> room_11 -> room_7 -> room_15` for `bathtub` | `Go next to room_11.` | strongest supporting page; advisor bundle present |
| `00862-LT9Jq6dN3Ea` | frozen `manifest.json` plus `logs/topology_query_report.json` present | `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3` | `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21` for `bathtub` | `Go next to room_33.` | high-complexity reserve; cross-floor evidence |
| `00829-QaLdnwvtxbs` | frozen `manifest.json` plus `logs/topology_query_report.json` present | `room_3 -> room_7` | `room_3 -> room_7` for `bed` | `Go next to room_7.` | sanity cross-check |

Primary fresh validation artifacts:

- ROS public-query validation: `runtime_export_validation/paper_ros_query_validation_00843.json`
- Coordinator/latest validation: `runtime_export_validation/paper_eval_validation_report.json`

## Table 3: Minimal Ablation

Source file: `ablation_minimal.csv`

This freeze uses the allowed fallback ablation choice:

- supporting systems ablation only
- not the main method ablation
- compares authoritative committed/public downstream consumption against a weaker non-authoritative projection already present in `logs/working_vs_committed_topology_report_v0_1.json`

Why this choice:

- A true priority-1 runtime/retrieval ablation is not already frozen as a paper-ready artifact here.
- Producing a clean priority-1 ablation would require either a controlled weaker runtime variant or a new rerun.
- This freeze is intentionally avoiding new feature work and non-minimal reruns.

| Sequence | Authoritative public rooms / edges | Weaker projection rooms / edges | Explicit route preserved on weaker surface | Semantic route preserved on weaker surface | Interpretation |
| --- | --- | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | 11 / 25 | 10 / 15 | no | no | both saved routes require `room_7`, which disappears on the weaker surface |
| `00824-Dd4bFSTQ8gi` | 8 / 28 | 3 / 2 | no | no | the weaker surface drops the route backbone `room_8 -> room_11 -> room_7` |
| `00862-LT9Jq6dN3Ea` | 30 / 107 | 23 / 73 | no | no | the weaker surface drops key rooms `room_40`, `room_33`, `room_34`, `room_23` |
| `00829-QaLdnwvtxbs` | 2 / 2 | 0 / 0 | no | no | the weaker surface removes the entire saved route |

Interpretation:

- The main paper story should stay on committed/public downstream consumption.
- Replacing that surface with the weaker non-authoritative projection breaks all saved downstream routes in this freeze set.
- This is useful as a systems sanity check, but it should be labeled as supporting evidence, not as the main method ablation.

## What Is Already Sufficient

Sufficient now for the minimal paper package:

- exact four-sequence protocol frozen
- main summary table frozen
- downstream routing table frozen
- one minimal ablation table frozen
- authoritative artifact paths frozen
- advisor-facing HTML/JSON pages frozen for `00843` and `00824`
- scene-local HTML/JSON demo pages frozen for all four sequences

Recommended main-paper placement:

- main paper: `00843`, `00824`, Table 1, Table 2
- appendix / supplement: `00862`, `00829`, `qualitative_assets_manifest.json`, scene-local demo pages, coordinator/query validation JSONs
- supporting systems ablation: Table 3

## What Is Still Missing Or Optional

No table cells are currently missing for the minimal package defined here.

Still not present in this freeze, by design:

- no priority-1 architecture-level retrieval/runtime ablation
- no fresh ROS query-server rerun for `00824`, `00862`, or `00829`
- no advisor-bundle page copies for `00862` or `00829` under `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/`

Interpretation of those gaps:

- They do not block this minimal paper package.
- The only meaningful upgrade still missing is a true method ablation that directly weakens the room-scoped selective-retrieval / bounded-local-current + committed-global design under controlled rerun conditions.

## Artifact Map

Detailed asset inventory lives in `qualitative_assets_manifest.json`.

Most important paths for manuscript drafting:

- shared advisor bundle index:
  `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/index.html`
- advisor bundle manifest:
  `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/demo_bundle_manifest.json`
- advisor bundle spec:
  `demo/room_graph_vln_advisor_bundle_20260417.json`
- primary advisor scene bundle:
  `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00843-dyehnkdt76v/`
- supporting advisor scene bundle:
  `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00824-dd4bfstq8gi/`

Scene-local HTML/JSON bundles for reserve and sanity:

- `00862`: `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea/final/`
- `00829`: `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/`

## Freeze Conclusion

This repo already contains enough frozen evidence to support a minimal paper-ready backend package centered on committed/public world-model consumption.

For the current paper-first scope, the package is ready to draft from:

- `sequence_summary.csv`
- `downstream_public_query_and_vln.csv`
- `ablation_minimal.csv`
- `qualitative_assets_manifest.json`
- `docs/paper_eval_reproduction_commands.md`
