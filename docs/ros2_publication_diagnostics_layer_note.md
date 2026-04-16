# ROS2 Publication Diagnostics Layer Note

## Chosen Interface

This step adds a separate debug-only ROS2 publication diagnostics surface alongside the existing committed/public query server.

- Node: `boxfusion_publication_diagnostics_node`
- Default service prefix: `/boxfusion/debug/publication`
- Services:
  - `GetPublicationDiagnostics`
    - empty request
    - returns a JSON payload with per-room lifecycle/publication diagnostics summary
  - `GetRoomPublicationState`
    - request field: `room_id`
    - returns a JSON payload for one room, or `room_found=false` for an expected miss

The ROS transport stays intentionally thin:

- primitive request fields only
- `success`, `error_code`, and `json_response` in the response
- publication diagnostics remain JSON-backed instead of introducing a wide typed schema

## Why This Is Debug-Only

This node loads the lifecycle/debug artifact bundle, not the committed/public topology contract.

- accepted inputs resolve to `online_topology_lifecycle_v0_1.json`
- the payload advertises `artifact_surface=lifecycle`
- responses include `debug_only=true` and `public_contract=false`
- the default namespace is under `/boxfusion/debug/...`

The existing committed/public query node is unchanged and still only serves committed/public topology.

## How Separation Is Preserved

This layer does not widen public topology guarantees.

- `PUBLISHED` is the only state treated as public topology membership
- non-`PUBLISHED` rooms are returned only as debug lifecycle diagnostics
- `pre_publication_only=true` is explicit for non-public rooms
- working/candidate/finalized-private/commit-ready states are not reinterpreted as public map membership
- the diagnostics server never serves `working_topology_v0_1.json` as a public topology API

In other words:

- public topology still means committed/published only
- publication-state outputs are lifecycle diagnostics, not a new public map contract

## Validation On Saved Artifacts

Validated against saved lifecycle artifacts already present in the repo through the backend path used by the ROS wrapper.

### `codex_acceptance_audit/core_only/00843-DYehNKdT76V`

- `room_1`
  - `publication_state=PUBLISHED`
  - `finalization_blockers=[]`
  - `publication_blockers=[]`
  - `published=true`, `pre_publication_only=false`
- `room_2`
  - `publication_state=ACTIVE_OBSERVING`
  - `finalization_blockers=[containment_not_stable, gateway_structure_not_stable, room_signature_not_stable]`
  - `publication_blockers=[no_leave_like_signal]`
  - `published=false`, `pre_publication_only=true`
- `room_3`
  - `publication_state=ACTIVE_OBSERVING`
  - `publication_blockers=[room_currently_active]`
  - `published=false`, `pre_publication_only=true`

### `codex_perf_probe/server_preinfer_opt_pass/validation_candidate1_fast_gt_resize300/output/00843-DYehNKdT76V`

- `room_1`
  - `publication_state=CANDIDATE_FORMED`
  - finalization blockers include:
    `containment_not_stable`, `floor_status_not_stable`, `gateway_structure_not_stable`, `merge_or_split_pending`, `room_signature_not_stable`
  - publication blockers:
    `no_leave_like_signal`
- `room_2`
  - `publication_state=CANDIDATE_FORMED`
  - publication blockers:
    `[]`
- `room_3`
  - `publication_state=CANDIDATE_FORMED`
  - publication blockers:
    `[room_currently_active]`
- no room is public:
  - `published_room_count=0`

### `codex_perf_probe/server_preinfer_opt_pass/validation_candidate2_fast_depth_kth150/output/00843-DYehNKdT76V`

- rooms remain pre-publication
- representative states stay at `CANDIDATE_FORMED`
- representative blockers remain structural:
  `gateway_structure_not_stable`, `merge_or_split_pending`, `room_signature_not_stable`

## Deferred

This step still intentionally does not do the following:

- online incremental public topology publication
- any new public topology contract beyond committed/published exports
- working-topology or candidate-state publication as public map data
- runtime streaming topics or mature lifecycle publication services
- broader typed-message redesign
