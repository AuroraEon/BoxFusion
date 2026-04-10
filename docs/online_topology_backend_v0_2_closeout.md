# Online-Topology Backend v0.2 Closeout

## Status

Online-topology backend work is ready for milestone closeout.

This closeout is intentionally backend-first and conservative:

- World export / world graph remains the truth owner.
- Room-centric topology remains a derived layer.
- Default Query API public semantics remain unchanged.
- Working topology remains private/debug-only.
- Symbolic execution remains downstream-only.
- Runtime stage-1 remains frozen; this closeout does not reopen broad runtime optimization.

The result is not "real online public topology publication." The result is a mature backend line that can represent provisional topology internally, compare it against committed/public topology, and evaluate hypothetical early-publication policies without adopting them.

## Naming Note

Most checked-in implementation and artifact filenames on this line still carry `v0_1` names. This document is `v0.2` because it closes the milestone at the project level; it does not claim a new public file-format or Query API version.

## What Was Implemented

The repo now contains the following online-topology backend pieces.

### 1. Lifecycle scaffold and room-local readiness tracking

- `boxfusion/online_topology_lifecycle.py`
- `boxfusion/test_online_topology_lifecycle.py`

Implemented behavior:

- explicit room lifecycle states
- dirty-room bookkeeping
- trigger history retention
- candidate-complete versus commit-ready separation
- final committed-room reporting
- refresh-history retention for later temporal evaluation

This is the core backend addition. It gives the system a private notion of provisional topology readiness without changing default public behavior.

### 2. Working-versus-committed topology split

- `boxfusion/online_topology_working_snapshot.py`
- `boxfusion/working_vs_committed_topology_timeline.py`
- `boxfusion/test_online_topology_working_snapshot.py`
- `boxfusion/test_working_vs_committed_topology_timeline.py`
- `stage_a_online_topology_working_snapshot.py`
- `stage_a_working_vs_committed_topology_timeline_eval.py`

Implemented behavior:

- build a debug-only working topology snapshot from the current public topology plus lifecycle state
- build a debug-only committed projection aligned to lifecycle committed membership
- compute a static working-versus-committed comparison
- compute a temporal withheld-topology timeline over refresh history

This is the piece that makes the current answer explicit: working topology can contain useful provisional information that is intentionally withheld from default public semantics.

### 3. Non-default publication-policy simulation

- `boxfusion/publication_policy_simulation.py`
- `boxfusion/test_publication_policy_simulation.py`
- `stage_a_publication_policy_simulation.py`

Implemented behavior:

- replay refresh history under hypothetical policy gates
- compare simulated published topology against the actual committed/public projection
- measure additional visibility versus actual publication timing

This is simulation only. It does not modify adopted publication behavior.

### 4. Publication-candidate survivability evaluation

- `boxfusion/publication_candidate_survivability.py`
- `boxfusion/test_publication_candidate_survivability.py`
- `stage_a_publication_candidate_survivability.py`

Implemented behavior:

- evaluate what happened to rooms that would have been published earlier under simulated policies
- classify candidate outcomes such as `stable_survivor`, `present_but_changed`, `withheld_again_or_reblocked`, and `disappeared_or_not_terminal`
- summarize dominant post-publication blocker recurrence and structural change

This is the decisive closeout layer. It turns "could we publish earlier?" into an answerable backend question and shows why the answer is currently "not by default."

### 5. Stage-A wiring and retained artifact path

Online-topology lifecycle and related debug artifacts are wired into the Stage-A export/evaluation path through:

- `boxfusion/stage_a_demo.py`
- `demo.py`
- `boxfusion/backend_eval_scaffold.py`

The default committed/public topology export remains the same `topology_v0_1.json` style path. The online-topology additions sit beside it as private/debug or offline-analysis artifacts.

## What Was Validated

### Focused tests in the current workspace

The following focused tests pass in the current workspace:

- `python3 boxfusion/test_online_topology_lifecycle.py`
- `python3 boxfusion/test_online_topology_timeline_eval.py`
- `python3 boxfusion/test_online_topology_working_snapshot.py`
- `python3 boxfusion/test_publication_policy_simulation.py`
- `python3 boxfusion/test_publication_candidate_survivability.py`

`test_online_topology_lifecycle.py` reports that its snapshot-topology smoke test is skipped in this shell because `networkx` is not installed. The lifecycle validation itself still passes.

### Representative checked-in evidence

The strongest checked-in evidence for the closeout conclusion is under:

- `online_topology_v0_1_gateway_attribution_diag/representative_runs/00843-DYehNKdT76V/logs/`

Relevant artifacts:

- `online_topology_lifecycle_v0_1.json`
- `working_topology_v0_1.json`
- `working_vs_committed_topology_report_v0_1.json`
- `working_vs_committed_topology_timeline_v0_1.json`
- `publication_policy_simulation_v0_1.json`
- `publication_candidate_survivability_v0_1.json`
- `topology_v0_1.json`
- `summary.json`

The current final backend export root at:

- `world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/logs/`

still carries the unchanged committed/public outputs and does not expose these online-topology policy-analysis artifacts by default. That separation is itself part of the closeout conclusion.

## Key Evidence and Interpretation

### 1. Working topology is informative

For the checked-in 00843 representative run:

- `working_vs_committed_topology_report_v0_1.json` reports `working_topology.room_count = 5`
- the aligned committed projection reports `committed_topology_projection.room_count = 1`
- the difference summary reports `working_only_room_count = 4`
- the difference summary reports `edge_count_difference = 7`

Interpretation:

- the backend is carrying materially more provisional room and edge structure than the default committed/public projection
- the working layer is therefore useful
- but usefulness alone is not enough to justify public exposure

### 2. The withheld/public boundary is doing real work

For the same 00843 run:

- `withheld_topology_summary.withheld_room_count = 4`
- dominant commit blockers include `room_signature_not_stable`, `gateway_structure_not_stable`, `containment_not_stable`, and `room_currently_active`

Interpretation:

- withheld topology is not arbitrary suppression
- it is exactly the region where structural stability is still questionable

### 3. Simulated early publication adds visibility, but mostly unstable visibility

Under `publication_policy_simulation_v0_1.json` for 00843:

- policy `candidate_complete_blocker_free_n2` publishes `1` room earlier than actual
- that same policy publishes `5` rooms under simulation that never appear in the actual committed projection
- its terminal delta versus actual is `additional_room_count_vs_actual = 4` and `additional_edge_count_vs_actual = 7`

Even the more conservative `commit_ready_export_stable2_n2` policy still:

- publishes `1` room earlier than actual
- publishes `4` rooms under simulation that never appear in the actual committed projection

Interpretation:

- simulated policies can indeed expose more structure earlier
- but that extra exposure is not reliably aligned with the actual stable committed/public end state

### 4. Survivability is the deciding result

For 00843, `publication_candidate_survivability_v0_1.json` reports:

- `candidate_room_count = 6`
- `stable_survivor_count = 0`
- `present_but_changed_count = 1`
- `withheld_again_or_reblocked_count = 4`
- `disappeared_or_not_terminal_count = 1`
- `stable_survivor_rate = 0.0`
- `terminal_committed_projection_rate = 0.167`

Interpretation:

- none of the earliest unique early-publication candidates survive cleanly to terminal committed/public status
- the one room that does remain present still changes materially
- most candidates are reblocked or withheld again
- one disappears from the terminal export path entirely

This is the decisive reason early publication is not adopted.

### 5. 00843 provides a strong negative result for early-publication adoption

The backend answer for 00843 is now explicit:

- provisional topology exists and is informative
- provisional topology can be surfaced in debug-only working artifacts
- simulated publication policies can make that provisional structure visible earlier
- but the earliest-publication candidates do not survive stably enough to justify changing default committed/public behavior

That is a genuine milestone result, not a partial result.

## What The Milestone Concludes

The online-topology backend milestone now supports the following project-level claims.

### Supported now

- The backend can track provisional room-topology readiness online.
- The backend can distinguish working/provisional topology from committed/public topology.
- The backend can compute temporal withheld-topology diagnostics.
- The backend can simulate non-default publication policies offline.
- The backend can evaluate whether simulated early-publication candidates actually survive.
- The backend can justify keeping default committed/public semantics unchanged.
- The backend is ready for a conservative ROS/Gazebo-facing landing contract.

### Not supported now

- public/default exposure of working topology
- working topology as a truth-owning layer
- adoption of a real early-publication policy
- Query API semantic changes
- planner-facing reinterpretation of the backend
- reopening runtime optimization

## Why Early Publication Is Not Adopted

Early publication is not deferred because the team "has not finished the idea yet." It is deferred because the backend evidence is already strong enough to reject default adoption for now.

Reasons:

- the working layer contains provisional value, but provisional value is not the same as stable public semantics
- simulated early-publication candidates on 00843 have `stable_survivor_rate = 0.0`
- most early candidates are later reblocked, withheld again, changed materially, or disappear from the terminal path
- adopting early publication would change public semantics without support from the survivability evidence
- the current architecture already has a safe answer: keep working topology private and keep committed/public semantics unchanged

## Intentionally Deferred

The following are intentionally deferred after this closeout:

- any real online committed-publication policy
- broader publication-policy tuning or deeper early-publication algorithm work
- public/default working-topology APIs
- Query API contract changes
- making topology the truth owner
- planner or agent reframing
- runtime optimization-line work beyond the already frozen stage-1 line
- renewed local birth-suppression work

## What This Milestone Now Genuinely Supports

This milestone should now be treated as complete enough to support implementation-side landing work.

Concretely, it supports:

- a backend node that continues to own committed world export and committed/public topology outputs
- a query server that continues to answer against committed topology only
- a private debug path that can emit working-topology and withheld-topology diagnostics
- offline evaluation and replay tools for publication-policy simulation and survivability analysis
- ROS 2 / Gazebo integration planning that preserves the current semantic boundaries

## Recommended Next Step

Do not continue deeper policy work in this line right now.

The next step should be a conservative ROS/Gazebo landing pass that:

- keeps the backend as the authoritative world-model producer
- exposes only committed/public signals by default
- keeps working topology private/debug-only
- maps the existing Query API semantics into ROS-facing services without changing their meaning

