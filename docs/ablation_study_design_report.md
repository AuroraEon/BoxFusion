# Ablation Study Design Report

Date: 2026-04-20

## Scope

These ablations are limited to what the current implementation can support honestly.

Priority order requested by the paper task:

1. fusion scope
2. room finalization / publication
3. BoxFusion / semantic association
4. segmentation cadence
5. public-vs-working topology

## Read First

- A rerun-free supporting ablation already exists in frozen form through `logs/working_vs_committed_topology_report_v0_1.json`.
- A cleaner method-level ablation is only paper-safe when the weaker condition is produced by a controlled rerun or a controlled transformed export.
- The frozen HM3D paper bundles are `core_only`; they are strong for final-state topology/query/routing, but not for replay-backed execution ablations.

## Ablation 1. Fusion Scope Ablation

### Hypothesis

Restricting association/fusion to a floor-aware, room-aware, near-current subset should reduce late-stage object-side cost while preserving most committed/public topology and query utility. A broader retained-history pool should cost more and may add noisy semantic carryover.

### Code path involved

- `demo.py::_build_floor_scoped_candidate_mask(...)`
- `demo.py` calls to:
  - `Instances3D.spatial_association(...)`
  - `Instances3D.correspondence_association(...)`
  - `BoxFusion.boxfusion(...)`
- `boxfusion/runtime_instrumentation.py`

### Toggles / flags currently available or missing

Available now:

- floor-scoped candidate masking exists in the main path
- shadow reference instrumentation exists through:
  - `--enable-readonly-tail-reference-audit` in `stage_a_demo.py`
  - corresponding runtime metrics in `demo.py`

Missing now:

- no clean public CLI switch that reruns the main method with floor-scoped tail pruning disabled as the primary condition
- no prepackaged report script that compares "actual pruned path" vs "full retained-history path" as a paper table

### Required outputs

- `logs/summary.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`
- runtime instrumentation CSV/JSON referenced from `logs/summary.json`
- committed/public outputs:
  - `logs/topology_v0_1.json`
  - `logs/committed_room_world_model_v0_1.json`

### Expected interpretation

- If runtime drops while committed/public topology and object-label queryability stay stable, the selective fusion story is supported.
- If topology or object-label route success drops sharply, the pruning is too aggressive and should stay appendix-only.

### Paper status

- feasible only as a partial / engineering ablation today
- strong enough for a paper table only after a controlled rerun with an explicit off-switch

## Ablation 2. Room Finalization / Publication Ablation

### Hypothesis

The committed/public filter is doing useful work: publishing rooms only after lifecycle commit gates are satisfied should improve route stability and reduce premature topology exposure.

### Code path involved

- `boxfusion/online_topology_lifecycle.py`
- `boxfusion/room_scoped_runtime.py`
  - `build_public_topology_payload(...)`
  - `build_committed_world_snapshot(...)`
  - `build_committed_room_world_model(...)`
  - `export_artifacts(...)`
- `boxfusion/publication_policy_simulation.py`
- `boxfusion/working_vs_committed_topology_timeline.py`

### Toggles / flags currently available or missing

Available now:

- debug-only policy simulation is implemented:
  - `stage_a_publication_policy_simulation.py`
- working-vs-committed temporal comparison is implemented:
  - `stage_a_working_vs_committed_topology_timeline_eval.py`

Missing now:

- no runtime switch that makes an alternative early-publication policy the authoritative export
- no paper-safe direct claim that any alternative policy is the deployed method

### Required outputs

- `logs/online_topology_lifecycle_v0_1.json`
- `logs/working_topology_v0_1.json`
- `logs/working_vs_committed_topology_report_v0_1.json`
- `logs/working_vs_committed_topology_timeline_v0_1.json`
- `logs/room_commit_diagnosis_v0_1.json`
- optional simulated output:
  - `logs/publication_policy_simulation_v0_1.json`

### Expected interpretation

- If earlier-publication policies expose many withheld rooms but also carry blocker-heavy states, the current conservative publication policy is justified.
- This should be presented as a systems safety/supporting ablation, not as the new main method.

### Paper status

- feasible now as a supporting ablation
- not suitable as the main method comparison unless the paper explicitly keeps the deployed method on committed/public semantics

## Ablation 3. BoxFusion / Semantic Association Ablation

### Hypothesis

Turning off BoxFusion or weakening association should hurt object-level semantic grounding and room semantic summaries more than it hurts basic room-route existence.

### Code path involved

- `config/hm3d.yaml`
  - `box_fusion.use`
  - `association.rotation_gap`
  - `association.translation_gap`
  - `box_fusion.small_size`
- `demo.py`
  - `Instances3D.spatial_association(...)`
  - `Instances3D.correspondence_association(...)`
  - `BoxFusion.boxfusion(...)`
  - `text_prompt(...)`
- `boxfusion/scene_graph_builder.py`
  - semantic object fusion and normalized label handling

### Toggles / flags currently available or missing

Available now:

- `config/hm3d.yaml` exposes a real BoxFusion on/off switch:
  - `box_fusion.use: True/False`
- association thresholds are configurable in YAML

Missing now:

- no dedicated CLI flag for a BoxFusion-off paper rerun
- no prebuilt aggregation script specialized for semantic-grounding deltas under this ablation

### Required outputs

- rerun scene bundle
- `logs/topology_v0_1.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/topology_query_report.json`
- optional validation report from:
  - `stage_a_topology_acceptance.py --acceptance`

### Expected interpretation

- Compare:
  - object count
  - object label count
  - object-label route success
  - dominant room semantic labels
- If room counts stay similar but object-label queryability degrades, the effect is semantic-grounding-specific rather than topology-specific.

### Paper status

- feasible with reruns
- good appendix or secondary table
- stronger than a purely cosmetic ablation because it hits grounded semantic utility

## Ablation 4. Segmentation Cadence Ablation

### Hypothesis

`--room-seg-interval` trades off runtime against room stability and publication readiness.

- smaller interval:
  - more updates
  - potentially earlier stabilization
  - higher topology/segmentation cost
- larger interval:
  - cheaper runtime
  - risk of fewer stable refreshes before sequence end
  - worse public room survival

### Code path involved

- CLI flag in `stage_a_demo.py`
  - `--room-seg-interval`
- `demo.py`
  - periodic call to `room_segmenter.perform_segmentation(...)`
- `boxfusion/floor_aware_room_segmenter.py`
- debug birth/stability tools:
  - `stage_a_eval/analyze_birth_baseline_generic.py`
  - `stage_a_eval/analyze_00843_birth_baseline.py`

### Toggles / flags currently available or missing

Available now:

- direct CLI control through `--room-seg-interval`
- birth-baseline debug scripts also accept segmentation interval inputs

Missing now:

- no frozen sweep table already checked into the repo
- no automatic multi-interval rerun harness for paper tables

### Required outputs

- `logs/summary.json`
- `logs/room_commit_diagnosis_v0_1.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`
- debug-room artifacts if using birth/stability proxy scripts

### Expected interpretation

- main response variables:
  - public room count
  - room-commit diagnosis category
  - runtime growth ratios
  - withheld room count
- This is one of the most feasible true rerun ablations because the switch already exists.

### Paper status

- feasible now with reruns
- good main or appendix ablation, depending on rerun budget

## Ablation 5. Public-vs-Working Topology Ablation

### Hypothesis

Using the weaker provisional/working topology instead of the authoritative committed/public surface should make downstream routes less stable and less paper-safe.

### Code path involved

- `boxfusion/online_topology_working_snapshot.py`
- `boxfusion/working_vs_committed_topology_timeline.py`
- `boxfusion/room_scoped_runtime.py`
- downstream consumers:
  - `boxfusion/query_api.py`
  - `stage_a_room_graph_vln_demo.py`
  - `stage_a_topology_query.py`

### Toggles / flags currently available or missing

Available now:

- frozen weaker projections already exist in:
  - `logs/working_topology_v0_1.json`
  - `logs/working_vs_committed_topology_report_v0_1.json`
- these can be used as supporting-system comparison artifacts without rerunning

Missing now:

- no default public CLI path intentionally targets the working surface
- no paper-safe authorization to promote working topology to authoritative status

### Required outputs

- `logs/topology_v0_1.json`
- `logs/working_topology_v0_1.json`
- `logs/working_vs_committed_topology_report_v0_1.json`
- saved route demos or fresh route queries on both surfaces

### Expected interpretation

- This is already the safest current ablation to show that weaker/provisional topology can break or shrink route utility.
- It must stay clearly labeled:
  - authoritative surface = committed/public
  - weaker surface = debug/non-authoritative

### Paper status

- feasible immediately
- already supported by the frozen bundle analysis
- best presented as supporting systems evidence, not the main algorithm ablation

## Additional Ablations Already Encoded But Blocked On Current Frozen Bundles

`boxfusion/world_model_eval.py` already defines these variant keys:

- `full_system`
- `floor_agnostic`
- `no_explicit_vertical_transition`
- `query_only`
- `policy_strict`
- `policy_balanced`
- `policy_exploratory`

Why they are blocked on the current frozen paper bundles:

- `_load_runtime_context(...)` in `boxfusion/world_model_eval.py` requires both:
  - `logs/topology_v0_1.json`
  - `logs/timeline.json`
- the frozen HM3D paper bundles are `core_only` and do not contain `logs/timeline.json`

Practical consequence:

- floor-aware vs floor-agnostic is conceptually implemented
- no-explicit-vertical-transition is conceptually implemented
- but both require full-artifact reruns before they are paper-ready on the current bundle set

## Recommended Ablation Order For This Paper

1. Public-vs-working topology ablation.
   Why first:
   - rerun-free
   - already frozen
   - directly supports the public/working separation claim

2. Segmentation cadence ablation.
   Why second:
   - clean existing CLI toggle
   - directly affects room stabilization and publication

3. BoxFusion / semantic association ablation.
   Why third:
   - config toggle exists
   - likely strongest effect on object/semantic grounding

4. Room finalization / publication simulation.
   Why fourth:
   - useful supporting analysis
   - should remain debug/systems evidence

5. Fusion scope ablation.
   Why fifth:
   - technically important
   - but still missing a clean public rerun switch for the broad-history baseline

## Bottom Line

Most feasible today:

- public-vs-working topology
- segmentation cadence
- BoxFusion on/off / semantic association weakening

Most informative but still partially blocked:

- fusion-scope selective retrieval vs broader retained history

Most useful supporting systems ablation:

- publication-policy simulation over the working-vs-committed timeline
