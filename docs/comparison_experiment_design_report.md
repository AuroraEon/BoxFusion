# Comparison Experiment Design Report

Date: 2026-04-20

## Assumptions

- This report evaluates comparison suitability for the current paper scope only: a pose-driven Stage-A room/floor-aware world-model backend with committed/public artifact export, structured query, and room-graph routing.
- Suitability below means "academically fair for this paper if carefully framed," not "already integrated in this repo."

## Recommended Comparison Strategy

Main-paper comparison priority:

1. Internal, repository-grounded comparisons first.
2. External baseline discussion second.
3. External numerical comparisons only when modality, supervision, and output contract are actually matched.

Recommended main-paper comparison layers:

- Primary:
  - our final committed/public backend on the four frozen HM3D scenes
  - internal supporting ablations that do not overclaim beyond the current system scope
- Optional quantitative appendix:
  - floor-aware vs floor-agnostic transformed topology
  - explicit vertical-transition handling vs remapped transition-only topology
  - working-vs-committed debug-only supporting comparison, clearly labeled non-authoritative
- Discussion section:
  - HOV-SG for floor-first room hierarchy context
  - ConceptGraphs / VLMaps for semantic-map representation context
  - Hydra / Kimera / vS-Graphs as broader scene-graph or SLAM context, not default head-to-head query/routing baselines

## Recommended Baseline Families

### Family A. Internal paper-safe baselines

Best fit for the current paper:

- committed/public final topology and world model
- floor-aware vs floor-agnostic transformed topology from `boxfusion/world_model_eval.py`
- explicit `vertical_transition` edges vs remapped generic transition edges from `boxfusion/world_model_eval.py`
- public committed topology vs weaker working/provisional projection using `logs/working_vs_committed_topology_report_v0_1.json`

Why this family is strongest:

- same sensor assumptions
- same pose assumptions
- same artifact contract
- same query API
- no fairness gap from missing planners or missing SLAM front ends

### Family B. Hierarchical world-model baselines

Safest external anchor:

- HOV-SG-style floor-first, room-first hierarchy discussion

Why:

- current repo explicitly reuses HOV-SG-style floor-first reasoning in `docs/floor_aware_world_graph_v0_1.md`
- `stage_a_eval/scene_registry.json` and `boxfusion/backend_eval_scaffold.py` already define an `hovsg_overlap` scene subset

### Family C. Semantic-map representation baselines

Safest external anchors:

- ConceptGraphs
- VLMaps

Why:

- both are better semantic-map references than room-routing references for the current paper
- they are useful for discussing how semantic information is stored and queried, but not directly equivalent to the repo's committed/public room-topology contract

### Family D. Broader scene-graph / SLAM context baselines

Best kept as context or carefully limited appendix discussion:

- Hydra
- Kimera
- vS-Graphs

Why:

- these systems sit closer to broader scene-graph or SLAM stacks
- this repo's current paper-safe scope is not a full SLAM or full navigation stack
- direct numerical head-to-head claims would be easy to overstate

## Suitability Matrix

| Baseline | Hierarchy / world-model comparison | Semantic representation comparison | Routing / query comparison | Discussion only | Notes for this paper |
| --- | --- | --- | --- | --- | --- |
| Hydra | partial, appendix-only | partial | no as a default numerical baseline | yes | Too close to a broader embodied stack; unfair unless reduced to the same pose-provided backend scope and export contract |
| Kimera | partial, appendix-only | weak | no as a default numerical baseline | yes | Strong SLAM context, but current BoxFusion paper scope is not a SLAM-accuracy paper |
| HOV-SG | yes, recommended | partial | not recommended as a direct route-success baseline | yes | Best external hierarchy/world-model comparison family for current scope |
| ConceptGraphs | no for room hierarchy | yes, recommended | no | yes | Good for semantic representation discussion, weaker match for room-finalization/public-topology claims |
| vS-Graphs | partial | partial | not recommended unless query contract is recreated | yes | Potential graph-level context, but not a clean current routing baseline in this repo |
| VLMaps | no for hierarchy | yes, recommended as semantic-map context | no | yes | Useful discussion anchor for semantic retrieval surfaces, not room-topology routing |

## Recommended Metrics For Comparisons

Use only metrics that match the current system scope.

### Main paper-safe metrics

- public floor count
- public room count
- public edge count
- vertical-transition count
- route-found rate on committed/public room graph
- exact room-hit rate on seeded tasks
- exact floor-hit rate on seeded cross-floor tasks
- object-label route success on committed/public exports
- query latency
- backend artifact bytes
- bytes per public room

Repo support:

- `manifest.json`
- `logs/summary.json`
- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `stage_a_topology_acceptance.py`
- `stage_a_eval/run_backend_eval.py`

### Supporting metrics

- committed/public room survival after filtering
  - source: `logs/room_commit_diagnosis_v0_1.json`
- withheld working-only room count
  - source: `logs/working_vs_committed_topology_report_v0_1.json`
- runtime growth ratios
  - source: `logs/runtime_growth_profile.json`
- blocker histograms
  - source: `logs/room_commit_diagnosis_v0_1.json`

### Metrics to avoid as main comparison claims

- localization or pose accuracy
- loop-closure accuracy
- controller success
- waypoint efficiency
- continuous navigation success
- navmesh quality
- dense BEV planner quality

Those are outside the current proven system scope.

## Risks Of Unfair Comparison

### 1. Pose assumption mismatch

Current BoxFusion paper scope uses provided pose.

Unfair comparison examples:

- comparing directly against full SLAM systems on localization quality
- claiming runtime wins against systems that also estimate pose

Safe framing:

- compare only backend world-modeling / query outputs given pose

### 2. Output contract mismatch

Current repo exports:

- committed/public room graph
- committed room summaries
- artifact-backed query API

Many external systems do not expose the same:

- room-finalization policy
- public-vs-working split
- room-level route API
- object-label route API

Safe framing:

- compare structural world-model outputs or semantic storage styles, not route success, unless a matched query layer exists

### 3. Online vs offline mismatch

Current runtime is incremental, but many external hierarchy systems are batch/offline.

Safe framing:

- if comparing to HOV-SG-like hierarchy baselines, make it explicit that the comparison is about resulting hierarchy/room structure, not matched online latency semantics

### 4. Independent GT vs self-seeded tasks

`stage_a_eval/backend_tasks_v0_1.jsonl` is built from current exports, not external human-annotated navigation labels.

Safe framing:

- seeded task success is a backend-consistency metric
- do not present it as full external navigation-benchmark performance

### 5. Debug surface vs public surface mismatch

`logs/working_topology_v0_1.json` and `logs/online_topology_lifecycle_v0_1.json` are not authoritative public surfaces.

Safe framing:

- keep committed/public outputs as the main method
- keep working/provisional comparisons as supporting systems analysis only

## Comparisons Recommended For This Paper

### Recommended main-text comparisons

1. Internal method vs internal weakened variants.
   Best candidates:
   - full committed/public backend
   - floor-agnostic transformed topology
   - no-explicit-vertical-transition transformed topology

2. Richer vs simpler frozen HM3D scenes.
   Suggested role split:
   - `00843` main result
   - `00824` supporting result
   - `00862` reserve complexity case
   - `00829` sanity cross-check

3. Hierarchy/world-model discussion against HOV-SG.
   Keep this at the level of:
   - floor-first decomposition
   - room hierarchy
   - room/object containment representation

### Recommended appendix-only comparisons

- route-policy comparison:
  - `strict`
  - `balanced`
  - `exploratory`
  - supported by `boxfusion/query_api.py` and `boxfusion/world_model_eval.py`
- working-vs-committed projection comparison
- publication-policy simulation outputs

## Comparisons That Should Be Avoided

- "BoxFusion is a full RAG-SLAM system" comparisons
- direct SLAM leaderboard comparisons
- planner/controller comparisons
- end-to-end VLN-agent comparisons
- BEV planner comparisons
- comparisons that use working/debug topology as if it were the public method
- route-success tables against systems that do not expose a matched room-graph query API
- dense semantic-map baselines presented as direct room-topology routing baselines

## Practical Recommendation By Baseline Name

### Hydra

Use:

- discussion only
- optional appendix hierarchy context if the protocol is explicitly backend-only and pose-provided

Do not use:

- as a primary route-success baseline
- as a SLAM-accuracy baseline

### Kimera

Use:

- discussion only

Do not use:

- as a main numerical comparison for this paper

### HOV-SG

Use:

- primary external hierarchy/world-model comparison family
- discussion of floor-first room decomposition

Do not use:

- as the default routing/query baseline unless a matched room-query layer is rebuilt

### ConceptGraphs

Use:

- semantic representation comparison
- semantic summary / label-storage discussion

Do not use:

- as the main room-routing baseline

### vS-Graphs

Use:

- discussion only, unless a matched room-query export can be recreated

Do not use:

- as a default quantitative routing baseline in this paper

### VLMaps

Use:

- semantic retrieval representation discussion

Do not use:

- for room hierarchy comparison
- for room-graph route-success comparison

## Bottom Line

Best paper-safe comparison story:

- main quantitative story: internal BoxFusion Stage-A backend plus internal weakened variants
- external academic anchor: HOV-SG for hierarchy/world-model discussion
- semantic representation context: ConceptGraphs and VLMaps
- keep Hydra, Kimera, and vS-Graphs in discussion or very carefully constrained appendix framing
