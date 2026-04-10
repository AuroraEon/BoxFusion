# Online Topology v0.1 Closeout

## Scope and Boundary

Online topology v0.1 is a backend/world-model milestone, not a planner or agent milestone.

- The World Graph / vector-map export path remains the truth owner.
- Room-centric topology remains a derived layer built from exported world evidence.
- The default Query API contract remains unchanged and continues to read the existing committed topology snapshot path.
- The symbolic executor remains downstream-only.
- This step does not reopen broad runtime optimization and does not attempt full online-public topology publication.

The practical meaning of "online topology" here is therefore narrow and private: the system can track room-local topology lifecycle state during runtime, but the committed public topology export still comes from the existing end-of-run committed path.

## What Online Topology v0.1 Actually Implemented

The current repo state adds a thin private lifecycle scaffold around derived room-topology readiness.

- `boxfusion/online_topology_lifecycle.py` introduces room lifecycle states, dirty-room bookkeeping, trigger history, candidate readiness scoring, candidate-complete semantics, and separate commit blocking semantics.
- Dirty rooms are tracked explicitly rather than treating topology as a monolithic always-rebuild artifact.
- Trigger bookkeeping records why rooms became dirty or blocked, which gives a lightweight diagnostic trail without changing public behavior.
- Candidate readiness and commit readiness are now explicitly separated. A room can be candidate-complete without being commit-ready.
- `boxfusion/stage_a_demo.py` wires the lifecycle manager into Stage-A snapshot/export flow and writes `logs/online_topology_lifecycle_v0_1.json`.
- `demo.py` and `boxfusion/backend_eval_scaffold.py` carry the reporting path so the lifecycle artifact is retained alongside the existing backend outputs.
- Public topology export still uses the existing `RoomTopologyBuilder` committed path and still writes `logs/topology_v0_1.json`.

This is intentionally minimal integration. It is a private runtime scaffold plus reporting artifact, not a new truth layer and not a new public topology API.

## What Was Validated

### Focused tests

The intended lifecycle behavior is covered by the current focused tests in:

- `boxfusion/test_online_topology_lifecycle.py`
- `boxfusion/test_room_topology.py`

The lifecycle test file explicitly checks:

- candidate-complete can appear before commit
- merge/split and partial vertical-transition conditions block commit
- merge-pending can clear after repeated stable refreshes
- default public topology/query semantics remain snapshot-based

In the current shell, `python3 boxfusion/test_online_topology_lifecycle.py` passes. Re-running `boxfusion/test_room_topology.py` in this shell is blocked by missing `networkx`, but that test remains part of the already-validated source-of-truth scope for this milestone.

### Representative run: `00843-DYehNKdT76V`

Validated artifacts under:

- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/summary.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/online_topology_lifecycle_v0_1.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/topology_v0_1.json`

Observed lifecycle summary:

- `room_count = 6`
- `dirty_room_count = 5`
- `candidate_complete_room_count = 1`
- `candidate_complete_room_count_pre_finalize = 2`
- `commit_ready_room_count_pre_finalize = 1`
- `committed_room_count = 1`
- `blocked_commit_room_count = 5`

Other outcome signals:

- topology export succeeded (`topology_export_error = null`)
- runtime growth summary does not flag risk in this scene (`runtime_risk_flag = false`)

### Representative run: `00824-Dd4bFSTQ8gi`

Validated artifacts under:

- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/summary.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/online_topology_lifecycle_v0_1.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/topology_v0_1.json`

Observed lifecycle summary:

- `room_count = 9`
- `dirty_room_count = 4`
- `candidate_complete_room_count = 0`
- `candidate_complete_room_count_pre_finalize = 5`
- `commit_ready_room_count_pre_finalize = 5`
- `committed_room_count = 5`
- `blocked_commit_room_count = 4`

Other outcome signals:

- topology export succeeded (`topology_export_error = null`)
- runtime growth summary flags heavier-scene growth risk (`runtime_risk_flag = true`) with reasons `feature_boxfusion_growth` and `topology_room_segmentation_growth`

## What the Evidence Means

The important result is not that both scenes produce the same counts. The important result is that they no longer collapse candidate semantics into "commit-ready right now."

- In `00843`, at least one room reaches candidate-complete while still blocked from commit. Concretely, `room_3` is candidate-complete with commit blockers including `gateway_structure_not_stable` and `merge_or_split_pending`.
- In `00824`, several rooms become candidate-ready and are then commit-ready enough to finalize into committed rooms in the same overall end-state. Concretely, `candidate_complete_room_count_pre_finalize = 5` and `committed_room_count = 5`.

This is a good sign, not a contradiction.

- `00843` demonstrates that candidate semantics can now express "looks locally ready, but do not publish yet."
- `00824` demonstrates that candidate semantics also allow the expected path where sufficiently stable rooms proceed through to commit.
- Taken together, the two runs show that candidate-complete is now a meaningful intermediate state rather than a synonym for commit-ready.

That is the main v0.1 closeout claim. It is narrow, but it is real.

## Interpretation of the Two Representative Scenes

### `00843`: candidate emerges but remains blocked

This scene shows the intended conservative behavior.

- Candidate readiness appears before full commit readiness.
- At least one room reaches candidate-complete while merge/split and gateway-stability blockers remain active.
- The public topology export still succeeds, but only one room is actually committed in the lifecycle report.

This is the right behavior for a conservative scaffold. The system is allowed to notice that a room is plausibly complete without being forced to publish it.

### `00824`: candidate-ready rooms then commit

This scene shows the complementary behavior.

- Several rooms reach candidate-ready status before finalization.
- Those same rooms are also commit-ready enough to move through to committed state by the final report.
- The candidate count drops to zero after finalize because those rooms have been committed, not because candidate semantics disappeared.

This is also the right behavior. A room that is both candidate-complete and commit-ready should not be artificially held back just to preserve a visible intermediate count in the final summary.

## What Is Still Intentionally Out of Scope

Online topology v0.1 does not claim to solve the full online-topology problem.

- No public exposure of working topology through the default Query API
- No incremental topology rebuild pipeline replacing the existing committed export path
- No full online committed-publication policy
- No robust room-completion detector beyond the current conservative heuristics
- No rewrite of `RoomTopologyBuilder`
- No promotion of topology into the truth-owning layer
- No frontier subsystem
- No planner/agent-style reframing
- No multi-scene or full-sequence formal evaluation campaign beyond the current focused validation evidence

## Current Risks and Limitations

The scaffold is useful, but it is still heuristic and sequence-sensitive.

- Lifecycle outcomes remain sensitive to upstream segmentation and export stability.
- Some rooms still remain in `merge_or_split_pending` too easily, which is conservative but can suppress commit.
- `00824` shows that heavier scenes can still produce runtime-growth risk signals even though the lifecycle story itself remains valid.
- Early segmentation failures or unstable upstream room identity can still dominate downstream lifecycle behavior.
- The lifecycle artifact is diagnostic, but its interpretability is still limited when trying to explain exactly why a room did or did not cross from candidate-complete to commit-ready at a specific time.

## Implemented Debug-Only Timeline Evaluator

The smallest next step has now been implemented as a debug-only commit-readiness timeline / snapshot evaluator.

- `boxfusion/online_topology_lifecycle.py` now retains a compact private `refresh_history` stream inside `logs/online_topology_lifecycle_v0_1.json`.
- `boxfusion/online_topology_timeline_eval.py` reconstructs room lifecycle evolution across refreshes and produces a compact teacher-facing summary.
- `stage_a_online_topology_timeline_eval.py` provides an offline CLI that reads a scene root, `logs/summary.json`, or `logs/online_topology_lifecycle_v0_1.json` and writes:
  - `logs/online_topology_timeline_eval_v0_1.json`
  - `logs/online_topology_timeline_eval_v0_1.md`

The evaluator summarizes, for each room:

- `first_seen_frame`
- `first_candidate_complete_frame`
- `first_commit_ready_frame`
- `first_committed_frame`
- `final_lifecycle_state`
- `major_blockers_encountered`
- `candidate_but_blocked_refresh_count`
- `merge_or_split_pending_refresh_count`
- `last_blocker_set_before_commit`
- `final_blocker_set`

This remains strictly diagnostic.

- It does not change truth ownership.
- It does not change the default Query API contract.
- It does not expose working topology publicly.
- It does not replace the committed topology export path.

Current representative checked-in artifacts from the earlier v0.1 validation runs predate the new `refresh_history` field. The evaluator still runs on them, but it reports `history_mode = final_report_only` and leaves unavailable timing fields as `null` / `n/a` rather than over-claiming.

## Appendix

### Touched implementation files for v0.1

- `boxfusion/online_topology_lifecycle.py`
- `boxfusion/online_topology_timeline_eval.py`
- `boxfusion/stage_a_demo.py`
- `demo.py`
- `boxfusion/backend_eval_scaffold.py`
- `boxfusion/test_online_topology_lifecycle.py`
- `boxfusion/test_online_topology_timeline_eval.py`
- `stage_a_online_topology_timeline_eval.py`

### Validation commands and checks

Focused commands used in the current workspace:

- `python3 boxfusion/test_online_topology_lifecycle.py`
- `python3 boxfusion/test_online_topology_timeline_eval.py`
- `python3 stage_a_online_topology_timeline_eval.py online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/summary.json`
- `python3 boxfusion/test_room_topology.py` with the caveat that this shell currently lacks `networkx`

Representative artifacts inspected:

- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/summary.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/online_topology_lifecycle_v0_1.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00843-DYehNKdT76V/logs/topology_v0_1.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/summary.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/online_topology_lifecycle_v0_1.json`
- `online_topology_v0_1_validation/representative_runs_after_candidate_fix/00824-Dd4bFSTQ8gi/logs/topology_v0_1.json`
