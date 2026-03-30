# Paper Asset Inventory for Backend v0.2

## 1. Executive Summary

### What paper can already be written from the current package

The current package is already strong enough for a backend-first paper whose center of gravity is:

- A. backend / world-model correctness
- B. retained-scene runtime / practicality
- C. scene case studies
- D. a brief downstream compatibility note

In reviewer-facing terms, this is already a credible floor-aware, room-centric world-model / semantic-SLAM backend paper. It is not yet a paper about online query serving, broad ambiguity robustness, object identity quality, or full embodied navigation.

### What the current evidence most strongly supports

- The retained canonical package is internally coherent: all 8 retained HM3D scenes are `ready`, registered under the canonical root, and evaluated through the same Query API and symbolic executor interfaces.
- Backend correctness is very strong on the seeded benchmark: `94/94` task success, `94/94` exact room hit, `94/94` exact floor hit, and `78/78` route-found.
- Floor-aware routing/query behavior is genuinely exercised, not only same-floor lookup: `30` floor-sensitive query tasks and `5` cross-floor execute tasks succeed.
- Query API and symbolic executor compatibility are already well evidenced: query latency is sub-millisecond on average and execute tasks are `13/13`.
- Retained-scene practicality is supportable in a bounded way: Tier 1 backend artifacts total about `3.38 MB` across 8 scenes, while query-time behavior is cheap.
- A bounded ambiguity/near-miss claim is now defensible: the augmented suite reports `126/126` success-matched tasks, including correct `ambiguous` abstentions and `not_found` rejections.
- The four-scene story is unusually clean: `00843` backend anchor, `00873` showcase, `00829` clean single-floor control, `00862` difficult/runtime-risk case.

### What the current evidence does not support

- A fully online queryable semantic backend.
- Broad ambiguity robustness or open-vocabulary grounding.
- Strong object identity / merge quality.
- Broad room-semantic competence.
- Broad scalability claims.
- Open-ended VLN, planner, or full embodied navigation competence.

### Practical caveat

Some per-scene `logs/summary.json` fields still embed legacy `world_model_backend_outputs_v0_1` path strings. For paper writing, the safer source of truth is:

- `manifest.json` for actual retained artifact existence and sizes
- the files physically present under `world_model_backend_outputs_v0_2_final/`

## 2. Artifact Inventory

| File / path | What it contains | Supports | Quality |
| --- | --- | --- | --- |
| `stage_a_eval/scene_registry.json` | Canonical scene registry, root policy, `ready` status, artifact-source provenance for all 8 scenes. | A, B, C | Main-paper quality |
| `stage_a_eval/scene_retention_plan.json` | Final role/tier policy for the 8 retained scenes; defines full vs core-only retention. | B, C | Appendix quality |
| `stage_a_eval/backend_tasks_v0_1.jsonl` | Seeded benchmark provenance for the 94-task canonical suite. | A | Appendix quality |
| `stage_a_eval/backend_tasks_v0_2_augmented.jsonl` | Augmented SUP-01 task pack with `positive_seeded`, `ambiguity`, and `hard_negative` slices. | A, D | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/aggregate_summary.md` | Top-line seeded benchmark results and compact narrative summary. | A | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/aggregate_summary.json` | Machine-readable seeded aggregate metrics. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/task_results.csv` | Per-task seeded outcomes, room/floor hits, route hops, cost, latency, status. | A, D | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/task_type_breakdown.csv` | Seeded breakdown by query/resolve/execute task types. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/policy_breakdown.csv` | Seeded policy comparison (`balanced`, `strict`, `exploratory`). | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/scene_runtime_summary.csv` | Per-scene floors, rooms, nodes, VT count, bytes, compactness. | B, C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/docs/backend_eval_hierarchy_summary.csv` | Hierarchy-level summary for `query`, `resolve`, `execute`. | A, D | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/backend_eval_task_family_summary.csv` | Family-level summary for room/anchor/object/resolve slices. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/backend_eval_scene_role_summary.csv` | Scene-role readout combining coverage, runtime, task counts, and recommended usage. | B, C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/docs/backend_eval_runtime_readout.csv` | Runtime-growth and practicality summary across all 8 scenes. | B | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented/aggregate_summary.md` | Top-line augmented results with expected-status matching. | A | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented/probe_slice_breakdown.csv` | Compact table for `ambiguity`, `hard_negative`, and `positive_seeded` slices. | A | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented/slice_ambiguity_summary.csv` | Single-slice summary for ambiguity abstention behavior. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented/slice_hard_negative_summary.csv` | Single-slice summary for near-miss rejection behavior. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented/task_results.csv` | Per-task augmented outcomes and actual backend statuses. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/augmented_probe_readout.md` | Conservative narrative interpretation of the augmented slice. | A | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/runtime_growth_paper_table.csv` | Full 8-scene runtime-growth table with early/mid/late windows and ratios. | B | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/runtime_growth_highlight_table.csv` | Four-scene runtime/practicality comparison for `00843`, `00873`, `00829`, `00862`. | B, C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/docs/runtime_growth_figure_data.csv` | Plot-ready long-form data for a 3-panel runtime-growth figure. | B | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/docs/runtime_growth_readout.md` | Short reviewer-facing runtime interpretation. | B | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/scene_case_study_packet.md` | Four-scene role summary, example query traces, caption-ready notes. | C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/docs/scene_case_study_assets.csv` | Exact figure asset inventory for the four named scenes. | C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/manifest.json` | Per-scene source of truth for actual retained artifacts, sizes, floors, rooms, and runtime summary. | A, B, C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/summary.json` | Per-scene runtime, growth summary, replay counts, and packaging metadata. Useful, but some embedded paths are stale. | B, C | Appendix quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/topology_v0_1.json` | Room-centric query/routing graph: floors, rooms, edges, entities, indices, evidence. | A, C, D | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/topology_query_report.json` | Query-layer report: floor order, sample routes, first-room summary, lookup capability flags. | A, C | Appendix quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/vertical_transition_evidence.json` | Floor-transition evidence with room association and edge-eligibility status. | A, C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/floor_diagnostics_summary.json` | Detailed floor hypothesis / support diagnostics. | A, B | Appendix quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/runtime_growth_profile.csv` | Per-scene runtime-growth samples for plotting and table generation. | B | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/logs/runtime_growth_profile.json` | Machine-readable runtime-growth summary and stage windows. | B | Appendix quality |
| `world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/final/*` | Full Tier 2 BEV / split / demo assets for the backend anchor scene. | C, D | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/00873-bxsVRursffK/final/*` | Full Tier 2 BEV / split / demo assets for the showcase scene. | C, D | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/00829-QaLdnwvtxbs/final/*` | Full Tier 2 BEV / split / demo assets for the clean single-floor control. | C | Main-paper quality |
| `world_model_backend_outputs_v0_2_final/scenes/{00843,00873,00829}/report.md` | Teacher-facing qualitative notes, revisit summaries, and presentation cues. | C | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/online_offline_behavior_report.md` | Honest system audit: partially online internals, retained-scene/post-hoc query layer in practice. | A, B, D | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/online_offline_summary.json` | Compact machine-readable scope boundary for online vs retained behavior. | A, B, D | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/claim_boundary_note.md` | Supported, weakly supported, and deferred claims; highly useful for writing scope-limited discussion. | A, B, D | Appendix quality |
| `world_model_backend_outputs_v0_2_final/docs/paper_table_figure_plan.md` | Already-reasonable table/figure plan for the current package. | A, B, C | Internal-only |
| `world_model_backend_outputs_v0_2_final/docs/paper_experiment_*.md` and `world_model_backend_outputs_v0_2_final/docs/paper_limitations_claim_boundary_draft.md` | Writing scaffolds, bridge paragraphs, and draft experiment text. Valuable for drafting, not evidence themselves. | A, B, C, D | Internal-only |

## 3. Scene / Task Coverage

### Retained HM3D scenes currently covered

| Scene | Floors | Rooms | Retention tier | Role | Seeded tasks | Augmented extras | Best use |
| --- | ---: | ---: | --- | --- | ---: | ---: | --- |
| `00824-Dd4bFSTQ8gi` | 1 | 8 | `core_only` | backup | 8 | 0 | Appendix single-floor coverage |
| `00829-QaLdnwvtxbs` | 1 | 2 | `full` | single_floor_clean | 8 | 8 | Clean single-floor control |
| `00843-DYehNKdT76V` | 2 | 11 | `full` | backend_anchor | 14 | 8 | Main backend anchor |
| `00861-GLAQ4DNUx5U` | 2 | 15 | `core_only` | coverage | 14 | 0 | Appendix 2-floor coverage |
| `00862-LT9Jq6dN3Ea` | 3 | 30 | `difficult_core_only` | difficult_case | 14 | 8 | Runtime-risk hard case |
| `00873-bxsVRursffK` | 2 | 11 | `full` | showcase | 14 | 8 | Rich qualitative showcase |
| `00877-4ok3usBNeis` | 1 | 3 | `core_only` | backup | 8 | 0 | Minimal single-floor backup |
| `00890-6s7QHgap2fW` | 2 | 12 | `core_only` | coverage | 14 | 0 | Route-complexity coverage |

### Seeded suite coverage

- Total seeded tasks: `94`
- Task mix: `65` query, `16` resolve, `13` execute
- Route-bearing tasks: `78`
- Floor-sensitive tasks: `30`
- Per-scene split:
  - `00824`, `00829`, `00877`: `8` tasks each
  - `00843`, `00861`, `00862`, `00873`, `00890`: `14` tasks each
- Floor-sensitive coverage lives in the 5 multi-floor scenes:
  - `00843`, `00861`, `00862`, `00873`, `00890`

### Augmented suite coverage

- Total augmented tasks: `126`
- Preserved seeded slice: `94` `positive_seeded`
- Added slices:
  - `24` `ambiguity`
  - `8` `hard_negative`
- Augmented scenes:
  - `00843`, `00873`, `00862`: each adds `6` ambiguity + `2` hard-negative probes
  - `00829`: optional clean single-floor slice, also adds `6` ambiguity + `2` hard-negative probes
- Observed augmented status histogram:
  - `ambiguity`: `14` `ambiguous`, `10` `success`
  - `hard_negative`: `8` `not_found`
  - `positive_seeded`: `94` `success`

### Best current scenes by paper role

- Backend anchor: `00843-DYehNKdT76V`
  - Best balanced multi-floor scene, `6` floor-sensitive tasks, full Tier 2 assets, best multi-floor FPS (`5.618`), and mildest highlighted runtime-growth ratios.
- Showcase: `00873-bxsVRursffK`
  - Strongest teacher-facing scene, denser objects (`319`), full Tier 2 assets, but much heavier runtime than `00843`.
- Clean single-floor control: `00829-QaLdnwvtxbs`
  - Only `2` rooms, all routed tasks are `1` hop, easy to explain, low discrimination but excellent clarity.
- Difficult / runtime-risk case: `00862-LT9Jq6dN3Ea`
  - Only 3-floor scene, largest graph (`675` nodes), longest runtime (`4949.727 s`), worst FPS (`1.515`), strongest growth ratios.

### Useful secondary scenes

- `00890-6s7QHgap2fW`: strongest 2-floor route-complexity support; highest average route hops in the eval.
- `00861-GLAQ4DNUx5U`: useful heavier 2-floor coverage, but less presentation-friendly than `00862`.
- `00824-Dd4bFSTQ8gi` and `00877-4ok3usBNeis`: clean backups, mainly appendix value.

## 4. Metrics And Tables Already Available

### Metrics already available

#### Correctness

- Task success rate
- Resolve success rate
- Exact room hit rate
- Exact floor hit rate
- Route-found rate
- Per-task-family success and latency
- Per-hierarchy-level success and latency
- Per-policy success and latency
- Per-task route hop count and total route cost

#### Runtime / practicality

- Query latency mean / p50 / p90
- Symbolic plan latency mean / p50 / p90
- Scene duration
- Average FPS
- Tier 1 bytes
- Tier 2 optional bytes
- Bytes per room
- Early / mid / late average total-step runtime
- Early / mid / late feature-fusion runtime
- Early / mid / late topology-segmentation runtime
- Runtime-growth ratios for total step, feature fusion, and topology segmentation
- Runtime-risk flag and reasons

#### Ambiguity / near-miss rejection

- Success under expected-status matching
- Expected-failure task count
- Actual status histogram
- Exact room / floor hit on the concrete-resolution subset
- Route-found rate on the route-requiring subset
- Slice-level latency mean
- Case-kind mix via task JSONL (`alias_probe`, `duplicate_label_cross_room_abstain`, `duplicate_label_same_room_success`, `near_miss_not_found`)

#### Downstream compatibility

- Execute-task success (`13/13`)
- Cross-floor execute count (`5`)
- Symbolic plan latency
- Closed-loop demo MP4 availability on showcase scenes
- Replay-backed executor interface availability (`VLNClosedLoopExecutor.from_paths(...)`)

### Concrete table / figure mapping

| Item | Purpose | Immediate from current outputs? | Recommended sources |
| --- | --- | --- | --- |
| Table 1. Seeded backend correctness summary | Core A-block quantitative table. | Yes | `eval/backend_eval_v0_1/aggregate_summary.md`, `eval/backend_eval_v0_1/task_type_breakdown.csv`, `eval/backend_eval_v0_1/policy_breakdown.csv` |
| Table 2. Bounded ambiguity / near-miss slice | Compact evidence for abstention and rejection under expected-status matching. | Yes | `eval/backend_eval_v0_2_augmented/aggregate_summary.md`, `eval/backend_eval_v0_2_augmented/probe_slice_breakdown.csv`, `docs/augmented_probe_readout.md` |
| Table 3. Four-scene runtime / practicality comparison | Main B-block table contrasting `00843`, `00873`, `00829`, `00862`. | Yes | `docs/runtime_growth_highlight_table.csv`, `docs/backend_eval_scene_role_summary.csv` |
| Appendix Table A1. Full 8-scene role/runtime table | Full retained-scene coverage without crowding the main text. | Yes | `eval/backend_eval_v0_1/scene_runtime_summary.csv`, `docs/runtime_growth_paper_table.csv` |
| Appendix Table A2. Family / hierarchy / policy breakdown | Shows saturation and benchmark composition clearly. | Yes | `docs/backend_eval_task_family_summary.csv`, `docs/backend_eval_hierarchy_summary.csv`, `eval/backend_eval_v0_1/policy_breakdown.csv` |
| Figure 1. `00843` backend anchor | Clean multi-floor visual anchor for the paper. | Yes | `scenes/00843-DYehNKdT76V/final/*`, `scenes/00843-DYehNKdT76V/logs/vertical_transition_evidence.json` |
| Figure 2. Four-scene runtime-growth figure | Honest B-block runtime figure with early/mid/late growth. | Yes | `docs/runtime_growth_figure_data.csv` |
| Figure 3. `00873` showcase figure | Rich qualitative backend showcase without changing paper identity. | Yes | `scenes/00873-bxsVRursffK/final/*` |
| Figure 4. `00829` clean single-floor control | Optional clean-control figure for same-floor readability. | Yes | `scenes/00829-QaLdnwvtxbs/final/*`, `scenes/00829-QaLdnwvtxbs/logs/topology_query_report.json` |
| Figure 5. `00862` difficult-case inset / limitation figure | Runtime-risk and hardest-case honesty figure. | Yes, with simple plotting/composition | `scenes/00862-LT9Jq6dN3Ea/logs/runtime_growth_profile.csv`, `scenes/00862-LT9Jq6dN3Ea/logs/vertical_transition_evidence.json`, `scenes/00862-LT9Jq6dN3Ea/logs/topology_v0_1.json` |

### Immediate main-paper set

If space is limited, the best immediate main-paper bundle is:

- Table 1
- Table 2
- Table 3
- Figure 1 (`00843`)
- Figure 2 (runtime growth)

Everything else can safely move to appendix first.

## 5. Claim-To-Evidence Mapping

| Paper-safe claim | Supporting artifacts | Confidence | Gaps / caveats |
| --- | --- | --- | --- |
| Retained backend correctness | `eval/backend_eval_v0_1/aggregate_summary.md`, `eval/backend_eval_v0_1/task_results.csv`, `stage_a_eval/scene_registry.json`, `stage_a_eval/backend_tasks_v0_1.jsonl` | Strong | Acceptance-style benchmark, not external human-annotated stress test. |
| Floor-aware route/query correctness | `eval/backend_eval_v0_1/task_results.csv`, `docs/backend_eval_hierarchy_summary.csv`, per-scene `logs/topology_query_report.json`, per-scene `logs/vertical_transition_evidence.json` | Strong | Strong on retained scenes; not a broad online routing claim. |
| Query API + symbolic executor compatibility | `docs/backend_eval_hierarchy_summary.csv`, `eval/backend_eval_v0_1/task_results.csv`, `eval/backend_eval_v0_1/task_type_breakdown.csv`, `docs/online_offline_behavior_report.md` | Strong | Compatibility is shown on retained exported topology, not on live mapping-time topology. |
| Retained-scene practicality | `eval/backend_eval_v0_1/scene_runtime_summary.csv`, `docs/backend_eval_runtime_readout.csv`, `docs/runtime_growth_highlight_table.csv`, per-scene manifests | Medium to strong | Query-time practicality is strong; construction-time growth is still substantial, especially `00862`. |
| Bounded ambiguity abstention / near-miss rejection | `eval/backend_eval_v0_2_augmented/aggregate_summary.md`, `eval/backend_eval_v0_2_augmented/probe_slice_breakdown.csv`, `eval/backend_eval_v0_2_augmented/task_results.csv`, `docs/augmented_probe_readout.md` | Medium | The claim must stay explicitly bounded, label-based, and room-resolution-centric. |
| Backend-first, downstream-second framing | `docs/canonical_dataset.md`, `docs/claim_boundary_note.md`, `docs/online_offline_behavior_report.md`, `docs/paper_experiment_matrix.md` | Strong as a framing claim | This is a scope boundary, not a performance result; the writing must stay disciplined. |

## 6. Missing Experiments For A Stronger Submission

Prioritized for high ROI, backend-first scope, and minimal system change:

### 1. Identity / merge-sensitive appendix probe suite

- Why: the biggest remaining reviewer-facing weakness is not seeded correctness, but whether object-level duplication/merge issues are being hidden by room-level success metrics.
- Minimal version: add a targeted appendix-only slice on `00843`, `00873`, and `00862` using the existing evaluator scaffold and expected-status logic.
- Safe outcome: even a negative result is valuable if written honestly.

### 2. Slightly harder multi-floor route slice on `00862` and `00890`

- Why: the current floor-aware result is strong but somewhat saturated.
- Minimal version: add a small set of longer-hop room/anchor/object route probes stressing double floor switches and higher hop count.
- Safe outcome: strengthens floor-aware routing without drifting into planner or control claims.

### 3. Snapshot-time queryability timeline on `00843`

- Why: the package already contains strong evidence that the backend is partially online internally but late/stabilized as a query layer.
- Minimal version: turn the existing retained-snapshot analysis into one compact figure/table showing first same-floor provisional queryability, first multi-floor catalog, and first reliable cross-floor queryability.
- Safe outcome: sharpens the honesty boundary around “not fully online.”

### 4. Constrained NL-to-query-to-executor trace pack

- Why: this is the cleanest way to keep D present but small.
- Minimal version: 3-5 traces on `00843` and `00873`, limited to intent/slot extraction, tool selection, query grounding, and executor handoff.
- Safe outcome: enough to motivate downstream relevance without turning the paper into an agent benchmark.

### What should not be added now

- Broad embodied-navigation benchmarking
- Open-ended planner evaluation
- Large rerun-heavy scale studies
- Major mapper redesign for object identity
- Broad room-semantic or open-vocabulary benchmarking

## 7. Reproducibility Commands

All commands below assume the existing BoxFusion env:

```bash
ENV=/home/aurora/miniconda3/envs/boxfusion/bin/python
```

### A. Backend / world-model correctness

Heavy raw regeneration of canonical scene outputs:

```bash
$ENV stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seqs 00824-Dd4bFSTQ8gi 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00861-GLAQ4DNUx5U 00862-LT9Jq6dN3Ea 00873-bxsVRursffK 00877-4ok3usBNeis 00890-6s7QHgap2fW \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --aggregate-name stage_a_multi_sequence
```

```bash
$ENV stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seqs 00873-bxsVRursffK 00843-DYehNKdT76V 00829-QaLdnwvtxbs \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --full-rgb-replay \
  --aggregate-name stage_a_multi_sequence
```

Lightweight seeded benchmark regeneration:

```bash
$ENV stage_a_eval/build_scene_registry.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --registry-out ./stage_a_eval/scene_registry.json \
  --write-manifests
```

```bash
$ENV stage_a_eval/build_tasks_hierarchical_overlap.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --tasks-out ./stage_a_eval/backend_tasks_v0_1.jsonl
```

```bash
$ENV stage_a_eval/run_backend_eval.py \
  --registry ./stage_a_eval/scene_registry.json \
  --tasks ./stage_a_eval/backend_tasks_v0_1.jsonl \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --report-root ./world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1
```

Augmented ambiguity / hard-negative regeneration:

```bash
$ENV stage_a_eval/build_augmented_backend_tasks.py \
  --base-tasks ./stage_a_eval/backend_tasks_v0_1.jsonl \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --tasks-out ./stage_a_eval/backend_tasks_v0_2_augmented.jsonl \
  --include-optional-single-floor
```

```bash
$ENV stage_a_eval/run_backend_eval.py \
  --registry ./stage_a_eval/scene_registry.json \
  --tasks ./stage_a_eval/backend_tasks_v0_2_augmented.jsonl \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --report-root ./world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented
```

### B. Runtime / practicality

Regenerate the paper-facing runtime / supplement tables from existing retained artifacts:

```bash
$ENV stage_a_eval/package_paper_supplements.py \
  --scene-root ./world_model_backend_outputs_v0_2_final/scenes \
  --docs-root ./world_model_backend_outputs_v0_2_final/docs \
  --eval-root ./world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_2_augmented \
  --scene-role-summary ./world_model_backend_outputs_v0_2_final/docs/backend_eval_scene_role_summary.csv \
  --runtime-readout ./world_model_backend_outputs_v0_2_final/docs/backend_eval_runtime_readout.csv
```

### C. Scene case studies

Reproduce the exact example case-study queries used in the current packet:

```bash
$ENV stage_a_topology_query.py \
  --topology-json ./world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/logs/topology_v0_1.json \
  --start room_2 \
  --goal-room room_11 \
  --route-policy balanced
```

```bash
$ENV stage_a_topology_query.py \
  --topology-json ./world_model_backend_outputs_v0_2_final/scenes/00873-bxsVRursffK/logs/topology_v0_1.json \
  --start room_14 \
  --object-label tree \
  --route-policy balanced
```

```bash
$ENV stage_a_topology_query.py \
  --topology-json ./world_model_backend_outputs_v0_2_final/scenes/00829-QaLdnwvtxbs/logs/topology_v0_1.json \
  --start room_3 \
  --goal-room room_7 \
  --route-policy balanced
```

```bash
$ENV stage_a_topology_query.py \
  --topology-json ./world_model_backend_outputs_v0_2_final/scenes/00862-LT9Jq6dN3Ea/logs/topology_v0_1.json \
  --start room_40 \
  --goal-room room_3 \
  --route-policy balanced
```

### D. Narrow downstream validation

Current packaged D evidence is mostly:

- the `13/13` execute rows already present in `backend_eval_v0_1/task_results.csv`
- the retained closed-loop demo MP4s for `00843`, `00873`, and `00829`

If a dedicated replay-backed downstream acceptance report is desired, the closest existing harness is:

```bash
$ENV stage_a_minimal_vln_closed_loop.py \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --sequences 00843-DYehNKdT76V 00873-bxsVRursffK 00829-QaLdnwvtxbs \
  --route-policy balanced \
  --report-dir ./world_model_backend_outputs_v0_2_final/eval/minimal_vln_closed_loop_v0_1
```

Note:

- This harness requires both `logs/topology_v0_1.json` and `logs/timeline.json`, so it is naturally limited to the showcase scenes that retain Tier 2 timeline artifacts.

Low-level executor unit validation:

```bash
$ENV boxfusion/test_vln_closed_loop.py
```

## 8. Recommended Next Steps

The smallest high-impact path forward is:

1. Write Blocks A, B, and C now.
   - The current package is already sufficient for a solid backend paper if the claims stay conservative.
2. Use the immediate main-paper bundle.
   - Table 1, Table 2, Table 3, Figure 1 (`00843`), Figure 2 (runtime growth).
3. Keep D brief unless a trace pack is added.
   - Without a constrained NL trace pack, treat D as one short compatibility paragraph grounded in `13/13` execute success and existing demo assets.
4. If only one extra experiment is added, make it the identity/merge-sensitive appendix probe suite.
   - That is the highest-ROI strengthening step that stays backend-first.
5. Clean up reviewer-facing packaging before submission.
   - In particular, avoid citing stale embedded `v0_1` paths from `summary.json`; use manifests and actual retained files as the paper source of truth.

Bottom line: the backend paper is already writeable now. The safest version is a strong A/B/C paper with a very short D block and explicit honesty about retained-scene scope, bounded ambiguity evidence, and remaining runtime / identity limitations.
