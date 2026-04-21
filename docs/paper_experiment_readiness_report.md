# Paper Experiment Readiness Report

Date: 2026-04-20

## Assumptions

- "Available HM3D sequences" means the full frozen HM3D bundles that are actually present in this workspace under `runtime_stage1_frozen_evidence/...`.
- When prose notes conflict with code or on-disk artifacts, this report treats the current code and frozen artifacts as authoritative.
- Paper scope is the current pose-driven Stage-A backend: RGB-D plus provided pose, room/floor-aware world modeling, committed/public artifact export, artifact-backed query, and room-graph routing.

## Current Paper-Safe System Scope

Paper-safe scope, grounded in code:

- Runtime ingestion and mapping loop:
  - `demo.py:run(...)`
  - `boxfusion/floor_aware_room_segmenter.py`
  - `boxfusion/room_scoped_runtime.py`
  - `boxfusion/stage_a_demo.py`
- Public/default committed export surface:
  - `logs/topology_v0_1.json`
  - `logs/topology_query_report.json`
  - `logs/committed_room_world_model_v0_1.json`
  - `logs/committed_room_world_snapshot_v0_1.json`
- Structured query and routing:
  - `boxfusion/room_topology.py`
  - `boxfusion/query_api.py`
  - `boxfusion/ros_query_server.py`
  - `stage_a_topology_query.py`
  - `stage_a_room_graph_vln_demo.py`
- Debug and observability layers that are useful for analysis but are not public/default claims:
  - `logs/online_topology_lifecycle_v0_1.json`
  - `logs/working_topology_v0_1.json`
  - `logs/working_vs_committed_topology_report_v0_1.json`
  - `logs/working_vs_committed_topology_timeline_v0_1.json`
  - `logs/room_commit_diagnosis_v0_1.json`

Not paper-safe to claim from the current repo state:

- full RAG-SLAM
- live mutable-state query over runtime memory
- BEV planning
- controller execution
- full VLN benchmark coverage
- online localization / loop-closure accuracy

## Already Validated Capabilities

Validated from frozen artifacts already in the repo:

- Four full HM3D frozen bundles are present and internally consistent:
  - `00843-DYehNKdT76V`
  - `00824-Dd4bFSTQ8gi`
  - `00862-LT9Jq6dN3Ea`
  - `00829-QaLdnwvtxbs`
- All four bundles contain the full final-state paper-relevant artifact set:
  - `manifest.json`
  - `logs/summary.json`
  - `logs/topology_v0_1.json`
  - `logs/topology_query_report.json`
  - `logs/committed_room_world_model_v0_1.json`
  - `logs/committed_room_world_snapshot_v0_1.json`
  - `logs/room_scoped_runtime_state_v0_1.json`
  - `logs/room_commit_diagnosis_v0_1.json`
  - `logs/runtime_growth_profile.csv`
  - `logs/runtime_growth_profile.json`
  - `logs/vertical_transition_evidence.json`
- `logs/summary.json` on the frozen bundles declares the `core_only` contract:
  - `final_state_query=true`
  - `final_state_route=true`
  - `final_state_eval=true`
  - `checkpoint_replay=false`
  - `dense_replay=false`
- Public query / routing demos are already frozen:
  - `00843`: `final/00843-DYehNKdT76V_room_graph_vln_room_13.json`, `final/00843-DYehNKdT76V_room_graph_vln_couch.json`
  - `00824`: `final/00824-Dd4bFSTQ8gi_room_graph_vln_room_16.json`, `final/00824-Dd4bFSTQ8gi_room_graph_vln_bathtub.json`
  - `00862`: `final/00862-LT9Jq6dN3Ea_room_graph_vln_room_3.json`, `final/00862-LT9Jq6dN3Ea_room_graph_vln_bathtub.json`
  - `00829`: `final/00829-QaLdnwvtxbs_room_graph_vln_room_7.json`, `final/00829-QaLdnwvtxbs_room_graph_vln_bed.json`
- Fresh topology acceptance checks were reproducible from the frozen exports with:
  - `python3 stage_a_topology_acceptance.py --topology-json <scene>/logs/topology_v0_1.json --acceptance`
- Those acceptance checks passed on all four frozen scenes for:
  - route-to-anchor
  - route-to-object-id
  - route-to-object-label
- ROS-facing public bundle validation is already present for the primary scene:
  - `stage_a_ros_query_server_validation.py`
  - `runtime_export_validation/paper_ros_query_validation_00843.json`
- Coordinator/latest-pointer validation is already present for the primary scene:
  - `stage_a_runtime_export_coordinator_validation.py`
  - `runtime_export_validation/paper_eval_validation_report.json`

## Feasible Experiment Categories Now

| Experiment category | Status now | What is paper-safe to report | Required artifacts | Scripts / commands |
| --- | --- | --- | --- | --- |
| Final-state committed/public world-model summary | implemented and artifact-backed | room count, floor count, object count, anchor count, edge count, vertical-transition count, runtime, artifact size | `manifest.json`, `logs/summary.json`, `logs/room_commit_diagnosis_v0_1.json`, `logs/runtime_growth_profile.*` | inspect frozen bundles; optional refresh via `python3 stage_a_demo.py hm3d ... --core-only --room-seg-interval 100 --runtime-profile-interval 25` |
| Public query bundle validation | implemented | committed/public topology loads and serves room/object/anchor queries | `manifest.json`, `logs/topology_v0_1.json`, `logs/committed_room_world_model_v0_1.json` | `python3 stage_a_ros_query_server_validation.py --artifact-path <scene>/manifest.json` |
| Structured query / route success from exported topology | implemented | room route success, object-label route success, anchor route success, failure reasons, route length/cost/confidence | `logs/topology_v0_1.json` | `python3 stage_a_topology_acceptance.py --topology-json <scene>/logs/topology_v0_1.json --acceptance`; `python3 stage_a_topology_query.py ...` |
| Artifact-backed room-graph demo evaluation | implemented | explicit room-goal routing, semantic room-summary routing, next-hop explanation, cross-floor route examples where present | `logs/topology_v0_1.json`, `logs/committed_room_world_model_v0_1.json`, frozen `final/*.json` | `python3 stage_a_room_graph_vln_demo.py --scene-root <scene> --start-room ... --goal-room ...`; semantic mode via `--semantic-target ...` |
| Runtime / storage profiling | implemented | processed frames, duration, FPS, backend bytes, bytes per room, growth-risk flags, early/mid/late timing growth | `manifest.json`, `logs/runtime_growth_profile.csv`, `logs/runtime_growth_profile.json` | inspect `manifest.json`; optional aggregate tooling via `stage_a_eval/run_backend_eval.py` once registry/root is aligned |
| Working-vs-committed supporting ablation | implemented but debug-only | public-vs-working room/edge deltas, withheld-room counts, blocker summaries, route breakage when using weaker non-authoritative surface | `logs/working_topology_v0_1.json`, `logs/working_vs_committed_topology_report_v0_1.json`, `logs/working_vs_committed_topology_timeline_v0_1.json` | `python3 stage_a_working_vs_committed_topology_timeline_eval.py <scene>` |
| Publication-policy sensitivity study | implemented but debug-only | how hypothetical earlier-publication policies would differ from actual committed/public projection | `logs/online_topology_lifecycle_v0_1.json`, `logs/working_vs_committed_topology_timeline_v0_1.json`, optionally `logs/topology_v0_1.json` | `python3 stage_a_publication_policy_simulation.py <scene>` |
| Room-commit observability | implemented | observed rooms, candidate-complete rooms, committed rooms, public rooms, dominant blockers | `logs/room_commit_diagnosis_v0_1.json` | `python3 stage_a_room_commit_diagnosis.py <scene>` |
| Segmentation-cadence rerun study | partially implemented | effect of `--room-seg-interval` on public room counts, blockers, runtime growth, topology size | fresh reruns plus `logs/summary.json`, `logs/room_commit_diagnosis_v0_1.json` | `python3 stage_a_demo.py hm3d ... --room-seg-interval <N>` |
| Birth / room-fragment stability proxy study | partially implemented | region-birth persistence and support metrics as a proxy for segmentation stability | `debug_room/` artifacts from non-core-only or explicit debug capture | `python3 stage_a_eval/analyze_birth_baseline_generic.py --sequence-id <seq> --room-seg-interval <N>` |

## Experiment Categories That Are Not Yet Paper-Safe

| Experiment category | Current state | Why not paper-safe | Main blocker |
| --- | --- | --- | --- |
| Replay-backed closed-loop VLN execution on the frozen paper bundles | blocked on current frozen bundles | `boxfusion/world_model_eval.py` requires `logs/timeline.json`, but the frozen HM3D paper bundles are `core_only` and explicitly declare `checkpoint_replay=false` | current frozen bundles omit `logs/timeline.json` |
| Independent GT room segmentation accuracy on HM3D | not available | repo has no HM3D room-GT evaluator, no room IoU/ARI/VI scorer, and no bundled HM3D room-label alignment tooling | external annotation alignment and evaluator missing |
| Independent GT floor/room hierarchy accuracy | not available | floor ids and vertical transitions are exported, but no GT hierarchy scorer is implemented | GT hierarchy source plus scorer missing |
| Independent GT topology adjacency accuracy | not available | current adjacency/transition metrics are self-consistency or artifact-derived, not matched against external truth | no HM3D adjacency truth loader |
| End-to-end navigation/control metrics | not implemented for paper scope | no planner/controller, no BEV local planner, no actuator-level execution stack | out of scope by design |
| Full SLAM accuracy comparison | not paper-safe | current backend assumes provided pose and does not expose localization/loop-closure metrics | not the current system scope |
| Fair query/routing comparison against external systems without matched room-graph query APIs | risky | most candidate baselines do not share the same committed/public room-query surface or route-policy contract | adapter work and fairness protocol missing |

## Exact Artifacts Required Per Experiment Family

### 1. Final-state backend summary

Required:

- `manifest.json`
- `logs/summary.json`
- `logs/room_commit_diagnosis_v0_1.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`

Primary frozen scene roots:

- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V`
- `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi`
- `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`
- `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs`

### 2. Public query / routing / semantic room-summary demos

Required:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/committed_room_world_model_v0_1.json`
- frozen `final/*room_graph_vln*.json`

### 3. Object-to-room / anchor-to-room grounding

Required:

- `logs/topology_v0_1.json`
  - especially `entities.objects`
  - `entities.anchors`
  - room/object/anchor indices
- `logs/committed_room_world_model_v0_1.json`

### 4. Debug-only topology withholding / publication analysis

Required:

- `logs/online_topology_lifecycle_v0_1.json`
- `logs/working_topology_v0_1.json`
- `logs/working_vs_committed_topology_report_v0_1.json`
- `logs/working_vs_committed_topology_timeline_v0_1.json`

### 5. Replay-backed execution evaluation

Required but currently absent from the frozen paper bundles:

- `logs/timeline.json`
- optionally `logs/timeline.csv`

These are required by:

- `boxfusion/world_model_eval.py`
- `boxfusion/vln_closed_loop.py`
- `boxfusion/vln_end_to_end_demo.py`

## Exact Scripts And Commands Likely Involved

### Revalidate the existing public bundle

```bash
python3 stage_a_ros_query_server_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json
```

### Validate topology object / anchor / object-label queryability

```bash
python3 stage_a_topology_acceptance.py \
  --topology-json runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/logs/topology_v0_1.json \
  --acceptance
```

### Rebuild or refresh a room-graph demo page

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --goal-room room_13
```

### Inspect debug working-vs-committed withholding

```bash
python3 stage_a_working_vs_committed_topology_timeline_eval.py \
  runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

### Simulate alternative publication policies

```bash
python3 stage_a_publication_policy_simulation.py \
  runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

### Refresh or rerun a core-only Stage-A HM3D bundle

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

### Run the current compact backend evaluator

This is implemented, but the checked-in `stage_a_eval/scene_registry.json` points to a canonical root that is not present in this workspace.

```bash
python3 stage_a_eval/run_backend_eval.py \
  --registry ./stage_a_eval/scene_registry.json \
  --tasks ./stage_a_eval/backend_tasks_v0_1.jsonl \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes
```

### Run the richer world-model evaluator

This is blocked on `logs/timeline.json` for the current frozen `core_only` bundles.

```bash
python3 stage_a_world_model_backend_eval.py \
  --task-sheet evaluation/world_model_backend/tasks/multifloor_paper_eval_v0_2.json \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes
```

## Main Blockers Preventing Missing Experiments

### Blocker 1. No independent HM3D room / topology GT scorer in the repo

Impacted experiment types:

- room segmentation quality
- floor/room hierarchy accuracy
- adjacency/topology accuracy
- object-to-room grounding accuracy against external truth

What exists instead:

- frozen artifact summaries
- seeded overlap tasks in `stage_a_eval/backend_tasks_v0_1.jsonl`
- debug observability and stability proxies

### Blocker 2. Frozen paper bundles are `core_only`

Impacted experiment types:

- replay-backed symbolic execution
- end-to-end closed-loop VLN demo scoring with observation timelines

Evidence:

- `logs/summary.json` advertises `checkpoint_replay=false`
- `logs/timeline.json` is absent from the frozen HM3D paper bundles

### Blocker 3. Canonical 8-scene backend root is not materialized in this workspace

Impacted experiment types:

- direct execution of the current registry-driven compact backend scaffold without registry refresh

Evidence:

- `stage_a_eval/scene_registry.json` references `world_model_backend_outputs_v0_2_final/scenes`
- that directory is absent in this workspace

Practical effect:

- the current four frozen bundles are real evidence
- the checked-in 8-scene registry is planning metadata, not current local artifact coverage

### Blocker 4. Fusion-scope ablation is only partially instrumented

Impacted experiment types:

- clean "room-scoped selective retrieval vs broader retained history" method ablation

Evidence:

- `demo.py::_build_floor_scoped_candidate_mask(...)` exists
- shadow comparison / audit metrics exist
- there is no clean public CLI switch that reruns the full stack with floor-scoped tail pruning disabled as the primary path

### Blocker 5. Publication-policy alternatives are debug-only by design

Impacted experiment types:

- any ablation that tries to claim alternative publication policies as the public/default method

Evidence:

- `boxfusion/publication_policy_simulation.py` is explicitly offline simulation
- `boxfusion/room_scoped_runtime.py` still keeps committed/public semantics authoritative

## Readiness Bottom Line

Immediately paper-usable now:

- final-state committed/public HM3D backend summary on four frozen scenes
- public room-graph query/routing demos
- object/anchor/object-label grounding from exported topology
- runtime/storage profiling
- debug-only supporting ablation on working-vs-committed topology withholding

Not yet safely promotable to main-paper quantitative claims:

- external GT room/topology accuracy
- replay-backed execution metrics on the frozen core-only bundles
- any claim that the system already provides full navigation or full SLAM
