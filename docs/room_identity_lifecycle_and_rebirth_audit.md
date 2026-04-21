# Room Identity Lifecycle And Rebirth Audit

## Executive summary

The current runtime does have real room lifecycle and publication machinery on the main Stage-A path, but it does not have a real room rebirth / dropped-room re-identification mechanism. What exists today is:

- room-ID continuity while segmentation tracking survives,
- lifecycle-state tracking and commit/publication gating for those room IDs,
- stability signals based on room signature, gateway structure, containment, floor status, and vertical-transition completeness,
- committed-room export and selective retrieval hooks,
- offline/debug analyses for birth, persistence, and withheld-topology behavior.

What does not exist today is a runtime layer that says: “this newly born room is probably the same logical room as a previously dropped room, so restore the old identity.” The repo contains useful hooks for that future work, but those hooks are currently used for stabilization, publication filtering, export summarization, and offline analysis rather than rebirth matching.

## Current room lifecycle model

The active main-path lifecycle model is a layered pipeline:

1. `DynamicRoomSegmenter` carries room IDs forward by mask-IoU tracking and retains missing rooms for a bounded number of segmentation cycles (`tracking_missed_cycles`) (`boxfusion/dynamic_room_segmenter.py:47-48`, `boxfusion/dynamic_room_segmenter.py:1000-1082`).
2. `FloorAwareRoomSegmenter` maps per-floor local room IDs into a world-room namespace through `local_to_world_room_id` (`boxfusion/floor_aware_room_segmenter.py:36`, `boxfusion/floor_aware_room_segmenter.py:1523-1540`).
3. `ClosedLoopDemoRecorder.record_snapshot(...)` is the active Stage-A hook point where runtime room identity is fed into lifecycle/publication logic (`boxfusion/stage_a_demo.py:872-886`).
4. `OnlineTopologyLifecycleManager` tracks lifecycle state, dirty reasons, stability counters, candidate/commit blockers, and publication diagnostics (`boxfusion/online_topology_lifecycle.py:49-52`, `boxfusion/online_topology_lifecycle.py:149-208`, `boxfusion/online_topology_lifecycle.py:318-350`, `boxfusion/online_topology_lifecycle.py:795-883`).
5. `RoomScopedRuntimeManager` commits rooms that have become candidate-complete with no commit blockers and exports the committed/public bundle (`boxfusion/room_scoped_runtime.py:185-290`, `boxfusion/room_scoped_runtime.py:390-534`).
6. Finalize-time export builds a topology from the final vector map and then projects it down to the committed/public room subset (`boxfusion/stage_a_demo.py:1349-1490`).

Important boundary:

- `FloorAwareRoomSegmenter.describe_topology_status(...)` explicitly reports:
  - `room_leave_signal_exists: False`
  - `topology_incremental_online: False`
  - `topology_update_mode: "scheduled_segmentation_refresh_plus_full_export_rebuild"`
  Evidence: `boxfusion/floor_aware_room_segmenter.py:1440-1467`.

So the current room lifecycle system is layered on top of scheduled segmentation refreshes and exported snapshots, not on a true online/incremental room-topology update loop inside the segmenter itself.

## Commit/public readiness signals and blockers

The current commit/public-readiness machinery is real and active on the main path.

### Candidate/finalization logic

- `OnlineTopologyLifecycleManager` uses:
  - `stability_refresh_threshold = 2`
  - `candidate_readiness_threshold = 4`
  Evidence: `boxfusion/online_topology_lifecycle.py:220`, `boxfusion/online_topology_lifecycle.py:227-242`.
- A room becomes candidate-complete only if enough readiness signals are present and candidate blockers are absent (`boxfusion/online_topology_lifecycle.py:1018-1077`).

### Current readiness signals

- Leave-like signal:
  - room is not the currently active room, and either `last_departed_frame_idx` exists or `export_observation_count` reached the stability threshold.
  - Evidence: `boxfusion/online_topology_lifecycle.py:1023-1028`, `boxfusion/online_topology_lifecycle.py:1279-1296`.
- Room signature stable:
  - Evidence: `boxfusion/online_topology_lifecycle.py:1031-1034`.
- Gateway structure stable:
  - Evidence: `boxfusion/online_topology_lifecycle.py:1037-1039`.
- Containment stable:
  - Evidence: `boxfusion/online_topology_lifecycle.py:1042-1044`.
- Floor assignment stable:
  - Evidence: `boxfusion/online_topology_lifecycle.py:1046-1051`.

### Current blockers

- `ROOM_CURRENTLY_ACTIVE`
- `NO_LEAVE_LIKE_SIGNAL`
- `ROOM_SIGNATURE_NOT_STABLE`
- `GATEWAY_STRUCTURE_NOT_STABLE`
- `CONTAINMENT_NOT_STABLE`
- `FLOOR_STATUS_NOT_STABLE`
- `MERGE_OR_SPLIT_PENDING`
- `VERTICAL_TRANSITION_PARTIAL`
- `ROOM_FLOOR_VALIDATION_FAILED`
- `ROOM_MISSING_FROM_LATEST_EXPORT`

Evidence: `boxfusion/online_topology_lifecycle.py:1018-1077`.

### Publication-state machine

Current publication diagnostics distinguish:

- `ACTIVE_OBSERVING`
- `CANDIDATE_FORMED`
- `FINALIZATION_PENDING`
- `FINALIZED_PRIVATE`
- `COMMIT_READY`
- `PUBLISHED`

Evidence: `boxfusion/online_topology_lifecycle.py:104-116`, `boxfusion/online_topology_lifecycle.py:1300-1362`.

This is publication-state machinery, not rebirth matching.

## Existing room stability signals

The runtime already computes several real stability signals:

- Room signature stability:
  - Hash over room ID plus quantized polygon, center, area, floor ID, room type, and status.
  - Evidence: `boxfusion/online_topology_lifecycle.py:1085-1111`.
- Gateway structure stability:
  - Hash over per-room gateway entries including count, paired room, type, relation scope, connects, floor ID, and quantized positions.
  - Evidence: `boxfusion/online_topology_lifecycle.py:1114-1164`.
- Containment stability:
  - Hash over object IDs and anchor IDs per room.
  - Evidence: `boxfusion/online_topology_lifecycle.py:1167-1196`.
- Floor-status and room-floor validation stability:
  - Evidence: `boxfusion/online_topology_lifecycle.py:1046-1061`; floor validation is sourced from vector-map export.
- Vertical-transition completeness:
  - Unsupported/partial vertical transitions block commit.
  - Evidence: `boxfusion/online_topology_lifecycle.py:1053-1058`.

Critical interpretation:

- These signals contribute to stabilization and publication filtering.
- They do not currently perform dropped-room re-identification.

## Existing room birth / persistence / fragmentation analyses

There are several real analyses, but they are not part of the main runtime identity mechanism.

### Debug-only runtime diagnostics

- Working-vs-committed topology snapshot/report/timeline:
  - Built during finalize only as rich debug artifacts.
  - Evidence: `boxfusion/stage_a_demo.py:1407-1490`, `boxfusion/working_vs_committed_topology_timeline.py`.
- Room commit diagnosis:
  - Debug/reporting layer over commit blockers and lifecycle evidence.
  - Evidence: `boxfusion/stage_a_demo.py:1485-1490`, `boxfusion/room_commit_diagnosis.py`.
- Online topology timeline evaluator:
  - Offline/debug evaluator over lifecycle history.
  - Evidence: `boxfusion/online_topology_timeline_eval.py`.

### Analysis-only birth / persistence / fragmentation studies

- `stage_a_eval/analyze_birth_baseline_generic.py`
  - Can reuse retained `debug_room/` artifacts or generate a temporary debug-room capture by replaying `FloorAwareRoomSegmenter`.
  - Evidence: `stage_a_eval/analyze_birth_baseline_generic.py:29-177`.
- `stage_a_eval/analyze_00843_birth_baseline.py`
  - Collects birth events from `debug_room/` tracking reports and probes future-matched persistence.
  - Evidence: `stage_a_eval/analyze_00843_birth_baseline.py:104-194`.
- `stage_a_eval/analyze_00843_local_room_debug.py`
  - Examines local birth geometry and stage deltas from debug-room artifacts.
  - Evidence: `stage_a_eval/analyze_00843_local_room_debug.py:1-220`.
- `stage_a_eval/analyze_00843_right_pocket_bridge_decomposition.py`
  - Further decomposition of birth support and fragmentation behavior from debug-room artifacts.
  - Evidence: `stage_a_eval/analyze_00843_right_pocket_bridge_decomposition.py:237-250`.
- `stage_a_eval/compare_00824_birth_baseline_vs_00843.py`
  - Compares birth-baseline measurements offline.
  - Evidence: `stage_a_eval/compare_00824_birth_baseline_vs_00843.py:10-28`, `stage_a_eval/compare_00824_birth_baseline_vs_00843.py:179-205`.

These analyses help characterize birth/persistence behavior, but they do not create runtime rebirth matching.

## Whether explicit room rebirth handling exists

No.

High-confidence repository-grounded answer:

- No explicit runtime mechanism was found that rematches a newly created room to a previously dropped room using:
  - room signature,
  - object composition,
  - room summary,
  - adjacency consistency,
  - gateway structure,
  - containment stability,
  - topology stability,
  - or any similar identity-recovery score.

What the code does instead:

- Signature/gateway/containment hashes are used to decide stability and commit readiness (`boxfusion/online_topology_lifecycle.py:524-560`, `boxfusion/online_topology_lifecycle.py:1018-1077`).
- `RoomScopedRuntimeManager` uses committed summaries for export and selective retrieval context, not for remapping a new room ID onto an old dropped room (`boxfusion/room_scoped_runtime.py:185-290`, `boxfusion/room_scoped_runtime.py:390-458`, `boxfusion/room_scoped_runtime.py:559-607`).
- `FloorAwareRoomSegmenter` also has room-signature/cache logic, but it is for export-cache reuse / locality bookkeeping, not dropped-room identity recovery (`boxfusion/floor_aware_room_segmenter.py:577-620`, `boxfusion/floor_aware_room_segmenter.py:1098-1185`).

## Whether implicit rebirth-like behavior exists

Partial, but only in a weak same-ID continuity sense.

### What does exist

- If IoU tracking matches a refreshed room mask to an existing tracked room, the same room ID persists (`boxfusion/dynamic_room_segmenter.py:1000-1054`).
- If a room disappears briefly but stays within `tracking_missed_cycles`, it can be retained and resume continuity under the same ID (`boxfusion/dynamic_room_segmenter.py:1071-1082`).
- If the same room ID is seen again after departure, lifecycle logic can mark it `REVISITABLE` and record a revisit dirty event (`boxfusion/online_topology_lifecycle.py:276-291`).

### What does not exist

- Once a room has actually dropped out of tracking and a later segmentation creates a fresh room ID, there is no runtime code that matches that new room back to the old one.

### Demo-layer heuristic that should not be overstated

- `RevisitDetector` does exist and runs in the recorder.
- It uses pose proximity and same-room re-entry heuristics to emit revisit events.
- It is not used to override room identity, restore an old dropped room ID, or change commit/public export behavior.
- Evidence: `boxfusion/stage_a_demo.py:611-708`, `boxfusion/stage_a_demo.py:830-849`.

## What is missing today

- No explicit dropped-room to new-room rebirth matcher.
- No use of room signatures as a rematching score.
- No use of gateway structure or adjacency consistency as a rematching score.
- No use of containment or semantic summaries as a rematching score.
- No separate “dead room memory” that a fresh room birth can query against for identity restoration.
- No segmenter-level online room leave/completion trigger structure; the segmenter explicitly reports those structures as missing (`boxfusion/floor_aware_room_segmenter.py:1440-1467`).

## Existing hooks/signals that could support a future rebirth mechanism

These hooks already exist, but they are not currently used for rebirth matching:

- Room signature payloads/hashes:
  - `boxfusion/online_topology_lifecycle.py:1085-1111`
- Gateway signature payloads/hashes:
  - `boxfusion/online_topology_lifecycle.py:1114-1164`
- Containment signatures:
  - `boxfusion/online_topology_lifecycle.py:1167-1196`
- Room summary exports with object composition and connectivity:
  - `boxfusion/room_scoped_runtime.py:390-458`
- Adjacency lookup and neighbor-room context:
  - `boxfusion/room_scoped_runtime.py:228-244`, `boxfusion/room_scoped_runtime.py:559-607`, `boxfusion/room_scoped_runtime.py:709-742`
- Gateway-derived and transition-derived topology evidence:
  - `boxfusion/room_topology.py:1442-1705`
- Floor ID, floor validation, and vertical-transition summaries:
  - `boxfusion/online_topology_lifecycle.py:1046-1061`
  - `boxfusion/room_topology.py:1627-1712`
- Selective committed-room retrieval:
  - `boxfusion/room_scoped_runtime.py:559-607`

## Paper-safe interpretation of the current system

Paper-safe current interpretation:

- The system has room tracking, room stabilization, commit/publication filtering, and committed-room export.
- It does not yet have a true room rebirth / room re-identification mechanism after identity loss.
- Claims about “room identity persistence” are safe only in the bounded sense that the same room ID is preserved while tracking continuity survives.
- Claims that room signatures, adjacency, gateway structure, or semantic summaries currently recover identity across room death/rebirth would overstate the implementation.

## Recommended next step if the team later wants stronger room identity / rebirth handling

The safest next step is not a broad redesign. It is to make the missing capability explicit in docs/tests first, then prototype any future rebirth matching as a separate, auditable layer that consumes already-existing signals:

- start from committed-room summaries plus lifecycle signature payloads,
- keep the first version debug-only,
- prove it only when a fresh room birth is being matched back to a previously dropped room,
- and keep publication semantics unchanged until validated.

That recommendation is grounded in the current repo because the necessary hooks already exist, but the actual rebirth matcher does not.

## Ranked room-identity gaps by practical importance

1. No explicit room rebirth / re-identification mechanism exists at all
   - Category:
     - not implemented at all
   - Why it matters:
     - This is the central missing capability if the team wants identity persistence across room death and later reappearance.

2. Existing signatures and topology cues are used only for stabilization/publication, not identity recovery
   - Category:
     - partially implemented but only for stabilization
   - Why it matters:
     - The raw ingredients exist, which makes the code look more identity-aware than it currently is.

3. Current persistence is bounded by IoU tracking continuity and `tracking_missed_cycles`
   - Category:
     - partially implemented but only for stabilization
   - Why it matters:
     - Same-ID continuity can be mistaken for rebirth handling, but it fails once the room truly drops and later returns as a fresh birth.

4. Birth/persistence/fragmentation studies exist only in debug/offline tooling
   - Category:
     - present only in analysis/debug tooling
   - Why it matters:
     - The team already has useful evidence tools, but not a runtime identity-restoration mechanism.

5. The segmenter itself explicitly reports no online leave signal / no incremental topology trigger structures
   - Category:
     - not implemented at all
   - Why it matters:
     - This makes it easier to overread later lifecycle layers as if they were full online topology/lifecycle control inside segmentation itself.

Direct answer:

- A real room-rebirth mechanism does not exist today.
- The closest current behavior is bounded same-ID continuity while tracking survives.
