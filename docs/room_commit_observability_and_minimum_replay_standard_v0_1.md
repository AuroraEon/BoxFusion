# Room Commit Observability And Minimum Replay Standard v0.1

## Why Empty Public Topology Can Be Valid

The public/default topology remains committed-only.

That means:

- candidate rooms may exist in lifecycle and working artifacts
- candidate rooms may even be structurally stable in private/debug views
- but `logs/topology_v0_1.json`, `logs/topology_query_report.json`, and the default query path still expose committed/published rooms only

So an empty public topology is a valid outcome whenever no room reaches the existing commit gate:

- `candidate_complete`
- no `commit_block_reasons`
- still present in the latest export

This change does not weaken that contract and does not make working rooms public.

## New Observability Outputs

Every finalized replay run now writes:

- `logs/room_commit_diagnosis_v0_1.json`
- `logs/room_commit_diagnosis_v0_1.md`

These diagnose, per run:

- how many observed rooms ever became `candidate_room_formed_v1`
- whether any room ever reached `candidate_complete`
- which `commit_block_reasons` appeared at any refresh and specifically while a room was already `candidate_complete`
- whether room-transition / leave-like evidence occurred
- how many rooms were committed internally
- how many committed rooms survived the committed/public topology filter
- why an empty public topology happened

The diagnosis category is explicitly separated into:

1. `no_useful_room_candidates`
2. `candidates_seen_but_never_candidate_complete`
3. `candidate_complete_reached_but_commit_blocked`
4. `commit_succeeded_but_public_export_or_filter_removed_everything`

There is also a non-failure case:

- `public_committed_rooms_available`

Existing artifacts remain useful and unchanged in meaning:

- `logs/online_topology_lifecycle_v0_1.json`
- `logs/room_scoped_runtime_state_v0_1.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/topology_v0_1.json`

## Replay Diagnosis Command

Run on a scene root:

```bash
python3 stage_a_room_commit_diagnosis.py /path/to/scene_root
```

Run on a logs directory:

```bash
python3 stage_a_room_commit_diagnosis.py /path/to/scene_root/logs
```

Run on a saved summary or lifecycle artifact:

```bash
python3 stage_a_room_commit_diagnosis.py /path/to/scene_root/logs/summary.json
python3 stage_a_room_commit_diagnosis.py /path/to/scene_root/logs/online_topology_lifecycle_v0_1.json
```

Optional explicit outputs:

```bash
python3 stage_a_room_commit_diagnosis.py /path/to/scene_root \
  --json-out /tmp/room_commit_diagnosis.json \
  --md-out /tmp/room_commit_diagnosis.md
```

The command prints the diagnosis category and writes the JSON + markdown report.

## Minimum Replay Sequence Standard

This standard is based on the current lifecycle/runtime behavior in `boxfusion/online_topology_lifecycle.py`, not generic SLAM advice.

Current runtime thresholds:

- `stability_refresh_threshold = 2`
- `candidate_readiness_threshold = 4`

For a replay sequence to reliably produce at least one committed/public room, it should satisfy all of the following:

1. Observe at least one room across two or more export refreshes.
   This is needed for room signature, gateway signature, and containment stability counters to reach the current threshold.

2. Include a real room exit or room-to-room transition.
   A room that stays active to the end is still blocked by `room_currently_active`, and often by `no_leave_like_signal`.

3. Keep the pre-commit room stable after the last structural delta.
   If room shape, gateway structure, tracking structure, or removal/merge evidence keeps changing near the end, the room stays blocked by reasons such as:
   - `merge_or_split_pending`
   - `room_signature_not_stable`
   - `gateway_structure_not_stable`
   - `containment_not_stable`

4. Provide enough within-room coverage and viewpoint diversity that the exported room signature stops changing.
   In practice, a short trajectory that only grazes a room boundary or never stabilizes the room polygon/gateway layout is likely to remain below `candidate_complete`.

5. Keep floor evidence stable.
   Sequences that end during unstable floor assignment or cross-floor ambiguity can be blocked by:
   - `floor_status_not_stable`
   - `room_floor_validation_failed`

6. Avoid unresolved partial vertical transitions in short demo sequences unless they are actually needed.
   Unsupported or partial vertical-transition evidence can block commit with `vertical_transition_partial`.

7. Do not stop immediately after the first room transition.
   After a room becomes inactive, the runtime still needs at least one stable refresh window to clear merge/gateway-style blockers and become commit-ready.

In practical replay-planning terms, the minimum demo-ready pattern is:

- enter room A
- cover room A enough for stable room/gateway/containment evidence
- transition into room B so room A is no longer active
- keep replaying long enough after that transition for room A to remain stable in at least one additional refresh

Very short single-room sequences or short sequences that end immediately after crossing a doorway are expected to fail this standard.

## Fixtures / Tests

`boxfusion/test_room_commit_diagnosis.py` now includes:

- a minimal replay-shaped success case that produces one committed/public room
- a short replay-shaped failure case that produces zero committed/public rooms because no room ever reaches `candidate_complete`
- synthetic classification checks for:
  - no useful candidates
  - candidate-complete but blocked commit
  - internal commit lost by export/filter mismatch

Recommended regression command:

```bash
python3 -m pytest \
  boxfusion/test_room_commit_diagnosis.py \
  boxfusion/test_room_scoped_runtime.py \
  boxfusion/test_online_topology_lifecycle.py \
  -q
```

## Files Changed

- `boxfusion/room_commit_diagnosis.py`
- `stage_a_room_commit_diagnosis.py`
- `boxfusion/stage_a_demo.py`
- `boxfusion/test_room_commit_diagnosis.py`
- `docs/room_commit_observability_and_minimum_replay_standard_v0_1.md`
