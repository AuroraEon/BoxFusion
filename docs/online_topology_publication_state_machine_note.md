# Online Topology Publication State Machine Note

## Scope

This change keeps the existing online-topology lifecycle narrow and debug-first.
It does not expand ROS integration, does not broaden the committed/public topology contract, and does not introduce a mature incremental public topology service.

The implementation adds a derived publication-state layer on top of the existing lifecycle/debug evidence.

## Proposed States

- `ACTIVE_OBSERVING`
  The room is still being observed, or we do not yet have enough candidate-room evidence to treat it as beyond early observation.

- `CANDIDATE_FORMED`
  A room candidate exists in the debug lifecycle view, but structural blockers still prevent private finalization.

- `FINALIZATION_PENDING`
  The room is already `candidate_complete`, but one or more structural finalization blockers still prevent a finalized-private state.

- `FINALIZED_PRIVATE`
  Structural blockers are clear, so the room is considered finalized in the private/debug lifecycle sense, but publication blockers still prevent commit/publication.

- `COMMIT_READY`
  Structural blockers and publication blockers are both clear. The room is eligible for commit/publication if the existing committed export path succeeds.

- `PUBLISHED`
  The room is actually committed/published. This is still the only state that maps to the public topology contract.

## Blocker Split

Finalization blockers:

- `room_signature_not_stable`
- `gateway_structure_not_stable`
- `containment_not_stable`
- `floor_status_not_stable`
- `merge_or_split_pending`
- `vertical_transition_partial`
- `room_floor_validation_failed`
- `room_missing_from_latest_export`

Publication blockers:

- `room_currently_active`
- `no_leave_like_signal`

Semantically, structural instability now mainly blocks finalization, while active-room / leave-like evidence mainly blocks publication after a room is already structurally finalized.

## Leave-Like Signal V1

`leave_like_signal_v1` is true when:

- the room is not the current active room, and
- either `last_departed_frame_idx` is known, or `export_observation_count >= stability_refresh_threshold`.

This stays intentionally conservative and debug-oriented. It is a lifecycle signal, not a public contract.

## Reopen / Rollback

- Before publication:
  A reappearing structural blocker can move a room back from `COMMIT_READY` or `FINALIZED_PRIVATE` to `FINALIZATION_PENDING`, `CANDIDATE_FORMED`, or `ACTIVE_OBSERVING`.

- Before publication:
  A reappearing publication blocker can move a room from `COMMIT_READY` back to `FINALIZED_PRIVATE`.

- After publication:
  This step does not add a new public unpublish contract. Revisit / reopen behavior remains debug-only lifecycle evidence, not a public rollback guarantee.

## Validation Pass

Validated against existing saved lifecycle artifacts already in the repo.

### `codex_acceptance_audit/core_only/00843-DYehNKdT76V/logs/online_topology_lifecycle_v0_1.json`

- `room_1` reaches `COMMIT_READY` at frame `25` and ends `PUBLISHED`.
- `room_2` ends `ACTIVE_OBSERVING` with finalization blockers:
  `containment_not_stable`, `gateway_structure_not_stable`, `room_signature_not_stable`
  and publication blocker:
  `no_leave_like_signal`
- `room_3` ends `ACTIVE_OBSERVING` with finalization blockers:
  `containment_not_stable`, `gateway_structure_not_stable`, `room_signature_not_stable`
  and publication blocker:
  `room_currently_active`

### `codex_perf_probe/server_preinfer_opt_pass/validation_candidate1_fast_gt_resize300/output/00843-DYehNKdT76V/logs/online_topology_lifecycle_v0_1.json`

- `room_1`, `room_2`, and `room_3` end at `CANDIDATE_FORMED`
- representative finalization blockers include:
  `merge_or_split_pending`, `room_signature_not_stable`, `gateway_structure_not_stable`, `containment_not_stable`
- representative publication blockers include:
  `no_leave_like_signal`, `room_currently_active`
- this sample does not reach finalized-private or commit-ready in its final state

### `codex_perf_probe/server_preinfer_opt_pass/validation_candidate2_fast_depth_kth150/output/00843-DYehNKdT76V/logs/online_topology_lifecycle_v0_1.json`

- all rooms remain earlier than finalization
- final states stay at `CANDIDATE_FORMED`
- dominant blockers remain structural, especially:
  `merge_or_split_pending`, `room_signature_not_stable`, `gateway_structure_not_stable`

Across the saved artifacts checked for this step, no artifact ends exactly at `FINALIZED_PRIVATE` or `COMMIT_READY`.
Those states are observable as intermediate lifecycle milestones, but not as retained final artifact end-states in the current saved samples.

## What Became Explicit

- the distinction between structural finalization and later publication
- a derived debug publication-state label per room
- explicit `finalization_blockers` and `publication_blockers`
- explicit `leave_like_signal_v1`
- timeline/debug visibility into when a room first becomes finalized-private, commit-ready, or published

## What Remains Heuristic

- `candidate_room_formed_v1`
- `leave_like_signal_v1`
- the exact blocker-to-bucket mapping as a debug lifecycle policy rather than a finalized public API guarantee

## What Is Debug-Only

- `publication_state`
- `finalization_blockers`
- `publication_blockers`
- `leave_like_signal_v1`
- `finalized_private`
- `commit_ready`
- timeline publication-state paths and blocker-bucket diagnostics

## Still Not Public Contract

- working topology
- candidate-room / finalized-private / commit-ready states
- any publication-state-machine fields in lifecycle or offline diagnostics
- any inference that non-`PUBLISHED` rooms belong to the public topology

Public topology still means committed/published only.
