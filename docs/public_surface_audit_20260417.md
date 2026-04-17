# Public Surface Audit: 00843 `published_room_count` vs `room_count`

Date: 2026-04-17

## Conclusion

resolved by fix

## Root Cause

Classification: bug in diagnostics counting

The mismatch was not a coordinator/latest-pointer issue. Both consumers were resolving the same `00843-DYehNKdT76V` manifest-backed bundle.

The real split was:

- the public query server read the authoritative committed/public topology from `logs/topology_v0_1.json`
- the publication diagnostics server counted `published_room_count` from lifecycle-local `publication_state == PUBLISHED`

For the frozen `00843` bundle, those two sets diverged:

- lifecycle-only published rooms: `room_1`, `room_10`
- public topology room not lifecycle-published at the final refresh: `room_7`

That produced the old paper-facing contradiction:

- diagnostics `published_room_count = 12`
- query/public topology `room_count = 11`

The underlying authoritative public room set is the 11-room committed/public topology and matching committed world snapshot:

- `room_11`
- `room_13`
- `room_14`
- `room_2`
- `room_3`
- `room_4`
- `room_5`
- `room_6`
- `room_7`
- `room_8`
- `room_9`

## Affected Rooms

- `room_1`: lifecycle `PUBLISHED`, but `present_in_latest_export = false`; not on the authoritative public topology
- `room_10`: lifecycle `PUBLISHED`, but `present_in_latest_export = false`; not on the authoritative public topology
- `room_7`: present on the authoritative public topology and world snapshot, but final lifecycle publication state was `CANDIDATE_FORMED`

## Fix

Code changed: yes

The fix was deliberately narrow:

- publication diagnostics now resolve the sibling authoritative public bundle and derive `published` / `published_room_count` from that committed/public topology membership
- lifecycle-only `PUBLISHED` state is still preserved and surfaced separately as `lifecycle_published`
- coordinator runtime snapshots now use the authoritative public room set for their primary committed-room count and expose lifecycle/public deltas explicitly
- no committed/public export semantics were changed
- no working/debug artifact became default
- no ROS query or VLN routing logic was redesigned

## Files Changed

Code:

- `boxfusion/ros_publication_diagnostics_server.py`
- `boxfusion/runtime_snapshot.py`
- `stage_a_runtime_export_coordinator_validation.py`
- `boxfusion/test_ros_publication_diagnostics_server.py`

Validation outputs refreshed:

- `runtime_export_validation/paper_ros_query_validation_00843.json`
- `runtime_export_validation/paper_ros_publication_diagnostics_validation_00843.json`
- `runtime_export_validation/paper_eval_validation_report.json`
- `runtime_export_validation/paper_eval_validation/latest_export.json`
- `runtime_export_validation/paper_eval_validation/refresh_history/2026-04-17T134059.627385Z.json`
- `runtime_export_validation/paper_eval_validation/sidecar_shadow/runtime_snapshot_sidecar_shadow_subset_v0_1.json`

## Commands Run

```bash
python3 -m pytest \
  boxfusion/test_ros_publication_diagnostics_server.py \
  boxfusion/test_runtime_export_coordinator.py \
  boxfusion/test_room_scoped_runtime.py \
  -q
```

```bash
python3 stage_a_ros_query_server_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json \
  --json-out runtime_export_validation/paper_ros_query_validation_00843.json
```

```bash
python3 stage_a_ros_publication_diagnostics_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --json-out runtime_export_validation/paper_ros_publication_diagnostics_validation_00843.json
```

```bash
python3 stage_a_runtime_export_coordinator_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --coordination-root runtime_export_validation/paper_eval_validation \
  --json-out runtime_export_validation/paper_eval_validation_report.json
```

## Verification

After the fix:

- public query validation still resolves the same authoritative topology with `room_count = 11`
- publication diagnostics now report `published_room_count = 11`
- coordinator/latest validation now reports:
  - `published_room_count = 11`
  - `lifecycle_published_room_count = 12`
  - `lifecycle_published_but_not_public_room_ids = [room_1, room_10]`
  - `public_but_not_lifecycle_published_room_ids = [room_7]`

## Final Interpretation

The authoritative downstream-consumption story is now internally consistent again:

- ROS public query consumes the committed/public topology
- room-graph VLN continues to consume the same public room graph
- publication diagnostics remain debug-only, but now report authoritative public membership as the primary published count and expose lifecycle drift explicitly instead of silently overstating the public room count
