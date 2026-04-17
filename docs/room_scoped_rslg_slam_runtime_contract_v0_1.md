# Room-Scoped RSLG-SLAM Runtime Contract v0.1

## Scope

This change adds the missing paper-core room-scoped online fusion and commit path without redesigning the existing backend.

It keeps the current stable semantics intact:

- public/default topology is committed/published only
- lifecycle and working artifacts may mention non-committed rooms, but those rooms are not queryable through the default public path
- the sidecar remains shadow-only and diagnostic-only
- the replay-based ROS ingress path remains unchanged and still works end to end

## New Runtime Contract

The runtime now has an explicit two-level state model.

### 1. Local working state

Implemented by `boxfusion/room_scoped_runtime.py` and driven from `ClosedLoopDemoRecorder` in `boxfusion/stage_a_demo.py`.

This state is intentionally bounded and non-public:

- active room id / floor id
- bounded local room window
- changed room ids from the existing room-local export profile
- recent room trace
- selectively retrieved committed room ids

The selective retrieval policy is:

- prefer committed rooms adjacent to the active room
- then prefer same-floor committed rooms
- then fall back to nearest committed rooms
- never retrieve the entire committed history indiscriminately

The diagnostic artifact is:

- `logs/room_scoped_runtime_state_v0_1.json`

### 2. Committed global memory

Also implemented by `boxfusion/room_scoped_runtime.py`.

This is the accepted room memory that backs the public/default export path:

- stable room ids
- floor id
- centroid / footprint / bbox
- committed adjacency summary
- object ids / anchor ids
- lightweight semantic summary
- commit timestamps and provenance
- minimal BEV/VLN-ready node hook

The public artifact is:

- `logs/committed_room_world_model_v0_1.json`

## Room Completion Logic

Commit triggering reuses the existing lifecycle state machine in `boxfusion/online_topology_lifecycle.py`.

A room is committed only when it is already:

- `candidate_complete`
- free of `commit_block_reasons`
- still present in the latest export

In practice that means the room has already passed the existing gates:

- left-active-room / leave-like signal
- room signature stability
- gateway structure stability
- containment stability
- stable floor assignment
- no merge/split pending
- no partial vertical-transition blocker
- no room-floor validation failure

The new runtime manager does not invent a second competing commit policy. It consumes the existing lifecycle decision and turns that decision into committed memory plus committed/public export artifacts.

## Commit / Public Export Semantics

`boxfusion/stage_a_demo.py` now builds two topology views at finalize time:

- full internal topology from the latest vector map
- committed-only public topology filtered to lifecycle-committed rooms

The full internal topology is still used for:

- working-topology debug export
- working-vs-committed comparison
- lifecycle/debug analysis

The committed-only projection is now the authoritative public export written to:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/topology_v0_1.graphml` when enabled

The committed/public world snapshot hook is now:

- `logs/committed_room_world_snapshot_v0_1.json`

`boxfusion/ros_query_server.py` was updated to prefer:

1. `public_world_snapshot_path`
2. `committed_world_snapshot_path`
3. legacy `final_vector_map_path`

This keeps `GetWorldSnapshot` aligned with committed/public semantics instead of leaking the full internal vector-map snapshot by default.

## Changed Files

- `boxfusion/room_scoped_runtime.py`
- `boxfusion/stage_a_demo.py`
- `boxfusion/ros_query_server.py`
- `boxfusion/test_room_scoped_runtime.py`

## Repro Commands

Minimal room-scoped commit/export test:

```bash
python3 -m pytest boxfusion/test_room_scoped_runtime.py -q
```

Regression checks for the existing replay/coordinator/query stack:

```bash
python3 -m pytest \
  boxfusion/test_ros_simulation_ingress.py \
  boxfusion/test_runtime_export_coordinator.py \
  boxfusion/test_ros_query_server.py \
  boxfusion/test_ros_publication_diagnostics_server.py \
  boxfusion/test_online_topology_lifecycle.py \
  -q
```

## Remaining Gaps For Future VLN Integration

Intentionally not implemented here:

- no global BEV raster or BEV planner
- no sidecar authority change
- no navigation policy layer
- no detector/backbone swap work
- no new live Gazebo integration path

What is now ready for the next VLN step:

- committed room ids with stable room-node summaries
- committed-only adjacency graph
- centroid / footprint / bbox summaries
- lightweight semantic/object summaries
- committed/public world snapshot hook
- per-room `bev_vln_hook` fields for a downstream lightweight room-graph consumer

The remaining likely next step is a thin VLN adapter that consumes `committed_room_world_model_v0_1.json` plus committed `topology_v0_1.json`, while keeping dense/global BEV generation outside this backend change.
