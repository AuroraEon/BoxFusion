# Metric Definition And Collection Plan

Date: 2026-04-20

## Assumptions

- Metric definitions below are scoped to the current Stage-A backend, not a full navigation stack.
- "Directly measurable" means measurable from current repo code and artifacts without inventing external labels.
- When a metric has only proxy support, that is called out explicitly.

## Metric Table

| Metric | Precise definition for this paper | Already directly measurable? | Code support exists now? | Current collection path | Additional tooling needed if not fully supported |
| --- | --- | --- | --- | --- | --- |
| Room segmentation quality | Prefer two levels: 1. proxy stability metrics from segmentation/debug artifacts, 2. external-GT room quality if added later | proxy only | partial | `logs/room_commit_diagnosis_v0_1.json`, `logs/floor_diagnostics_summary.json`, debug-room artifacts, `stage_a_eval/analyze_birth_baseline_generic.py` | HM3D room-GT alignment, room IoU / ARI / VI scorer |
| Floor/room hierarchy quality | correctness of `floor_id` assignment, floor count, room-to-floor consistency, and vertical-transition structure | partial | partial | `logs/topology_v0_1.json`, `logs/committed_room_world_model_v0_1.json`, `logs/vertical_transition_evidence.json`, `logs/floor_diagnostics_summary.json` | external floor/room GT hierarchy loader and scorer |
| Object-to-room grounding | fraction and quality of exported object records that resolve to the correct room, plus route-to-object / route-to-object-label success | internal consistency yes, external GT no | yes for internal consistency | `logs/topology_v0_1.json`, `stage_a_topology_acceptance.py`, `boxfusion/query_api.py` | external object-room truth mapping for HM3D |
| Room adjacency/topology quality | correctness of room-room edge set and relation types (`adjacent`, `transition`, `possible_connection`, `vertical_transition`) | internal artifact measure yes, external GT no | partial | `logs/topology_v0_1.json`, `logs/topology_query_report.json`, `logs/vertical_transition_evidence.json` | external adjacency truth graph and scorer |
| Query success | success rate of structured target resolution and structured room/object/anchor queries over committed/public exports | yes | yes | `stage_a_topology_acceptance.py`, `stage_a_topology_query.py`, `stage_a_ros_query_server_validation.py`, `stage_a_eval/run_backend_eval.py` | only registry/root refresh if the compact evaluator is to be run on current frozen roots |
| Routing success | success rate of room-level route finding, room hit/floor hit, hop count, cost, confidence | yes | yes | `boxfusion/query_api.py`, frozen `final/*room_graph_vln*.json`, `stage_a_room_graph_vln_demo.py`, `stage_a_eval/run_backend_eval.py` | external navigation-benchmark labels if broader claims are desired |
| Runtime | processed frames, duration, FPS, stage timing growth, query latency | yes | yes | `manifest.json`, `logs/summary.json`, `logs/runtime_growth_profile.csv`, `logs/runtime_growth_profile.json`, `stage_a_eval/run_backend_eval.py` | none for current scope |
| Storage / artifact size | backend bytes, optional demo bytes, total bytes, bytes per room | yes | yes | `manifest.json`, `compactness_summary`, `stage_a_eval/run_backend_eval.py` scene runtime rows | none for current scope |
| Topology stability / churn | how much provisional topology is withheld, later committed, or remains unstable across refreshes | yes as a stability proxy | yes | `logs/online_topology_lifecycle_v0_1.json`, `logs/working_vs_committed_topology_report_v0_1.json`, `logs/working_vs_committed_topology_timeline_v0_1.json`, `logs/room_commit_diagnosis_v0_1.json`, `stage_a_publication_policy_simulation.py` | none for proxy analysis; external GT needed only if claiming true topological accuracy rather than churn/stability |

## Metric-By-Metric Plan

## 1. Room Segmentation Quality

### Precise definition

For the current repo state, split this into:

1. Stability / quality proxy metrics:
   - observed room count
   - candidate-complete room count
   - committed/public room count
   - blocker histogram while candidate-complete
   - room-birth persistence metrics from debug-room analyses
2. Future external-GT metrics:
   - room IoU against annotated room regions
   - clustering consistency metrics such as ARI / VI if room labels are available

### Already directly measurable?

- Proxy: yes
- External-GT accuracy: no

### Code support exists now

- `boxfusion/room_commit_diagnosis.py`
- `stage_a_room_commit_diagnosis.py`
- `stage_a_eval/analyze_birth_baseline_generic.py`
- `stage_a_eval/analyze_00843_birth_baseline.py`
- `boxfusion/floor_aware_room_segmenter.py`

### How to collect now

Use:

- `logs/room_commit_diagnosis_v0_1.json`
  - `observed_room_count`
  - `candidate_complete_ever_count`
  - `public_topology_room_count`
  - blocker counts
- `logs/floor_diagnostics_summary.json`
- debug-room analyses when reruns are available

### Additional tooling needed

- HM3D room annotation alignment
- room-region scorer that consumes exported room polygons

## 2. Floor / Room Hierarchy Quality

### Precise definition

Measure whether:

- floor count is plausible and stable
- each room has a consistent `floor_id`
- vertical transitions connect different floors only
- same-floor edges do not accidentally cross floors

### Already directly measurable?

- structural consistency: yes
- external-GT hierarchy accuracy: no

### Code support exists now

- `boxfusion/floor_aware_room_segmenter.py`
- `boxfusion/floor_manager.py`
- `boxfusion/room_topology.py`
- `logs/vertical_transition_evidence.json`

### How to collect now

From:

- `logs/topology_v0_1.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/vertical_transition_evidence.json`
- `logs/floor_diagnostics_summary.json`

Recommended current metrics:

- public floor count
- public vertical-transition count
- cross-floor route-found rate on seeded tasks
- fraction of room records with valid `floor_id`

### Additional tooling needed

- external floor/room truth extraction
- floor-aware room-accuracy scorer

## 3. Object-To-Room Grounding

### Precise definition

Measure:

- fraction of exported objects with non-null `room_id`
- route-to-object-id success
- route-to-object-label success
- anchor-to-room success

For a future stronger version:

- exact room hit against external object-room labels

### Already directly measurable?

- internal consistency and task success: yes
- external GT accuracy: no

### Code support exists now

- `boxfusion/query_api.py`
- `stage_a_topology_acceptance.py`
- `boxfusion/room_topology.py`
- topology `entities.objects` and `entities.anchors`

### How to collect now

Run:

```bash
python3 stage_a_topology_acceptance.py \
  --topology-json <scene>/logs/topology_v0_1.json \
  --acceptance
```

Read:

- `object_count`
- `object_label_count`
- route-to-object-id pass/fail
- route-to-object-label pass/fail
- route-to-anchor pass/fail

### Additional tooling needed

- external HM3D object-room ground truth alignment

## 4. Room Adjacency / Topology Quality

### Precise definition

Current measurable form:

- public room count
- public edge count
- relation-type counts
- cross-floor edge count
- route-found rate and exact room/floor hit on seeded tasks

Future stronger form:

- precision / recall against an external room adjacency graph

### Already directly measurable?

- internal/export-level: yes
- external accuracy: no

### Code support exists now

- `boxfusion/room_topology.py`
- `boxfusion/query_api.py`
- `stage_a_eval/build_tasks_hierarchical_overlap.py`
- `stage_a_eval/run_backend_eval.py`

### How to collect now

Use:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `stage_a_eval/backend_tasks_v0_1.jsonl`
- `stage_a_eval/run_backend_eval.py`

### Additional tooling needed

- external adjacency truth graph

## 5. Query Success

### Precise definition

Success rate of:

- `resolve_room_target`
- `resolve_anchor_room`
- `resolve_object_room`
- `query_route`
- `query_route_to_anchor`
- `query_route_to_object`

### Already directly measurable?

- yes

### Code support exists now

- `boxfusion/query_api.py`
- `stage_a_topology_acceptance.py`
- `stage_a_topology_query.py`
- `stage_a_ros_query_server_validation.py`
- `stage_a_eval/run_backend_eval.py`

### How to collect now

Preferred minimal check:

```bash
python3 stage_a_topology_acceptance.py \
  --topology-json <scene>/logs/topology_v0_1.json \
  --acceptance
```

Preferred aggregate scaffold:

```bash
python3 stage_a_eval/run_backend_eval.py \
  --registry <registry.json> \
  --tasks stage_a_eval/backend_tasks_v0_1.jsonl \
  --scene-output-root <scene_root>
```

Current caution:

- the checked-in `stage_a_eval/scene_registry.json` points to `world_model_backend_outputs_v0_2_final/scenes`, which is absent in this workspace
- the evaluator is implemented, but a registry/root refresh is needed before using it directly here

### Additional tooling needed

- none for current structured-query scope

## 6. Routing Success

### Precise definition

Measure:

- `route_found`
- exact room hit
- exact floor hit
- route hop count
- total cost
- route confidence

This is room-level routing only.

### Already directly measurable?

- yes

### Code support exists now

- `boxfusion/query_api.py`
- frozen room-graph demo JSONs under `final/`
- `stage_a_room_graph_vln_demo.py`
- `stage_a_eval/run_backend_eval.py`

### How to collect now

From frozen demos:

- explicit room-goal route success
- semantic room-summary route success

From the compact evaluator:

- exact room-hit rate
- exact floor-hit rate
- route-found rate
- latency

### Additional tooling needed

- only if the paper wants broader navigation-benchmark framing

## 7. Runtime

### Precise definition

Measure:

- processed frames
- duration
- average FPS
- query latency
- stage timing growth:
  - total step
  - topology / room segmentation
  - feature / BoxFusion bucket

### Already directly measurable?

- yes

### Code support exists now

- `logs/summary.json`
- `manifest.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`
- `boxfusion/runtime_instrumentation.py`

### How to collect now

Read directly from:

- `manifest.json.runtime_summary`
- `logs/summary.json`
- `logs/runtime_growth_profile.*`

### Additional tooling needed

- none

## 8. Storage / Artifact Size

### Precise definition

Measure:

- backend artifact bytes
- optional demo artifact bytes
- total bytes
- bytes per public room

### Already directly measurable?

- yes

### Code support exists now

- `manifest.json`
- `compactness_summary`
- `stage_a_eval/run_backend_eval.py`

### How to collect now

Read:

- `manifest.json.runtime_summary.backend_artifact_size_total_bytes`
- `manifest.json.runtime_summary.optional_demo_artifact_size_total_bytes`
- `manifest.json.compactness_summary.bytes_per_room`

### Additional tooling needed

- none

## 9. Topology Stability / Churn

### Precise definition

Measure provisional-to-public dynamics using:

- rooms ever seen in working but not public
- rooms later committed after being withheld
- rooms remaining withheld at the end
- withheld edge count
- blocker reason histograms
- simulated publication-policy deltas

### Already directly measurable?

- yes, as a stability/churn proxy

### Code support exists now

- `boxfusion/working_vs_committed_topology_timeline.py`
- `boxfusion/publication_policy_simulation.py`
- `boxfusion/room_commit_diagnosis.py`

### How to collect now

Use:

- `logs/working_vs_committed_topology_report_v0_1.json`
- `logs/working_vs_committed_topology_timeline_v0_1.json`
- `logs/room_commit_diagnosis_v0_1.json`
- `stage_a_publication_policy_simulation.py`

Useful current scalar metrics:

- `withheld_room_count`
- `withheld_edge_count`
- `rooms_ever_working_only_count`
- `rooms_working_only_then_committed_count`
- `rooms_remaining_working_only_to_end_count`

### Additional tooling needed

- none for the current stability proxy story

## Ground Truth Alignment Summary

### Ground truth alignment feasible now

- seeded structured tasks from current exports:
  - `stage_a_eval/backend_tasks_v0_1.jsonl`
- object / anchor / object-label queryability checks from exported topology:
  - `stage_a_topology_acceptance.py`

These are useful for:

- backend consistency
- query/routing utility
- export completeness

### Ground truth alignment not yet directly supported

- HM3D room segmentation GT scoring
- HM3D adjacency/topology GT scoring
- external object-to-room GT scoring for the Stage-A HM3D paper subset

### Important caution

`stage_a_eval/build_tasks_hierarchical_overlap.py` and `stage_a_eval/run_backend_eval.py` create and evaluate seeded overlap probes from existing exports. That is useful, but it is not the same as an external human-annotated navigation benchmark.

## Immediate Paper-Safe Metric Set

Safest metric bundle for the current paper:

- public floor count
- public room count
- public edge count
- vertical-transition count
- route-found rate
- exact room-hit rate
- exact floor-hit rate on cross-floor tasks
- route-to-object-label success
- query latency
- runtime / FPS
- backend artifact bytes
- topology withholding / churn counts

## Metrics That Need More Work Before Main-Paper Use

- room segmentation accuracy against external GT
- hierarchy accuracy against external GT
- topology precision / recall against external GT
- replay-backed execution success on the current frozen paper bundles
