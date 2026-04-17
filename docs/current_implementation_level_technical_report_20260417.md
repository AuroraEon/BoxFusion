# Current Implementation-Level Technical Report

## Executive Summary

The current runnable project is built around one live runtime loop and one export/orchestration layer.

- The live loop is `demo.run()` in `demo.py`. It owns dataset iteration, RGB-D preprocessing, model inference, 3D box association, BoxFusion updates, floor-aware room segmentation, vector-map export, and periodic snapshot capture.
- The export/orchestration layer is `ClosedLoopDemoRecorder` in `boxfusion/stage_a_demo.py`. It is not the detector or mapper itself. It records runtime snapshots and replay frames during `demo.run()`, then writes the Stage-A bundle during `finalize()`.
- The top-level `stage_a_demo.py` is a thin CLI wrapper. It loads config, dataset, CLIP/text features, builds `ClosedLoopDemoRecorder`, and then calls `demo.run()`.
- The public query surface is snapshot-based. `room_topology.py`, `query_api.py`, the ROS query server, and the VLN/query demos all read exported artifacts rather than live in-memory runtime state.
- Working/debug topology and lifecycle are implemented, but they are exported as separate debug artifacts. The default public bundle is the committed/public projection written by `RoomScopedRuntimeManager.export_artifacts()`.

The important current split is:

- Live mutable runtime state: `demo.py`, `FloorAwareRoomSegmenter`, `BoxManager`, `OnlineTopologyLifecycleManager`, `RoomScopedRuntimeManager`
- Final exported/public query state: `logs/topology_v0_1.json`, `logs/topology_query_report.json`, `logs/committed_room_world_snapshot_v0_1.json`, `logs/committed_room_world_model_v0_1.json`
- Debug state/history: `logs/online_topology_lifecycle_v0_1.json`, `logs/working_topology_v0_1.json`, `logs/working_vs_committed_topology_*`, floor/runtime diagnostics

## Core Runtime Implementation

### Real execution path from `demo.py`

Relevant files:

- `demo.py`
- `boxfusion/instances.py`
- `boxfusion/box_manager.py`
- `boxfusion/box_fusion.py`
- `boxfusion/floor_aware_room_segmenter.py`
- `boxfusion/floor_manager.py`
- `boxfusion/dynamic_room_segmenter.py`
- `boxfusion/stage_a_demo.py`

Important functions/classes:

- `demo.run(...)`
- `Instances3D`
- `Instances3D.spatial_association(...)`
- `Instances3D.correspondence_association(...)`
- `BoxManager`
- `BoxFusion.boxfusion(...)`
- `FloorAwareRoomSegmenter.observe_frame(...)`
- `FloorAwareRoomSegmenter.perform_segmentation(...)`
- `FloorAwareRoomSegmenter.get_vector_map_data(...)`
- `ClosedLoopDemoRecorder.record_snapshot(...)`
- `ClosedLoopDemoRecorder.record_frame(...)`

What happens:

1. `demo.run()` constructs the long-lived runtime objects:
   - `box_manager = BoxManager(cfg)`
   - `Box_Fuser = BoxFusion(cfg)`
   - `room_segmenter = FloorAwareRoomSegmenter(...)`
   - `runtime_profiler = RuntimeInstrumentation(...)`
   - optional `demo_recorder`

2. It iterates `for sample in dataset`.

3. For every frame it:
   - unpacks sensor payloads from the dataset sample
   - reads RGB, depth, timestamp, and pose from `sample["wide"]`, `sample["sensor_info"]`, and `sample["meta"]`
   - packages and preprocesses the input through `Augmentor.package(...)`, `move_input_to_current_device(...)`, and `Preprocessor.preprocess(...)`

4. On keyframes (`count % gap == 0` or last frame) it runs the detector model:
   - `pred_instances = model(packaged)[0]`
   - score and geometry filters are applied through `box_manager.check_uv_bounds(...)` and `box_manager.check_floor_mask(...)`

5. Still on keyframes, it backprojects the depth map into a local point cloud and pushes the downsampled XYZRGB chunk into `room_segmenter.observe_frame(...)`.

6. On non-keyframes it still calls `room_segmenter.observe_frame(...)`, but with `points_xyzrgb=None`, so the floor/pose tracker advances even when no new point chunk is added.

7. Every `room_seg_interval` frames, `demo.run()` triggers room segmentation:
   - merges accumulated point chunks
   - downsamples them again
   - calls `room_segmenter.perform_segmentation(...)`
   - when segmentation succeeds, optionally calls `room_segmenter.get_vector_map_data(...)` immediately, or defers full export to snapshot capture

8. On keyframes, after segmentation/visualization, `demo.run()` runs the object association/update pipeline:
   - bootstrap case: classify first-frame boxes and initialize global state
   - normal case: concatenate current detections into `all_pred_box`/`per_frame_ins`
   - run `Instances3D.spatial_association(...)`
   - run `Instances3D.correspondence_association(...)` for small objects
   - update `BoxManager`
   - optionally run `BoxFusion.boxfusion(...)`
   - classify surviving new boxes with CLIP text matching

9. Snapshot/export hooks are called from inside `demo.run()`:
   - `maybe_capture_demo_snapshot(...)` calls `room_segmenter.get_vector_map_data(...)` when needed and hands the result to `demo_recorder.record_snapshot(...)`
   - `demo_recorder.record_frame(...)` records replay-only frames when full replay is enabled
   - `demo_recorder.record_runtime_growth(...)` samples growth/timing metadata

10. At the end of the sequence, `demo.run()`:
   - calls `room_segmenter.flush_pending_floor_segments(...)`
   - does one final `get_vector_map_data(...)`
   - writes floor diagnostics through `room_segmenter.save_floor_diagnostics(...)`
   - finalizes runtime instrumentation
   - calls `demo_recorder.finalize(...)`

### RGB-D frame ingestion

Relevant files:

- `demo.py`
- `boxfusion/preprocessor.py`
- `tools/utils.py`

Core logic:

- The dataset object is created outside `demo.run()` by `get_dataset(cfg)` in `stage_a_demo.py` or the legacy `demo.py` CLI path.
- `demo.run()` reads:
  - RGB from `sample["wide"]["image"][-1]`
  - depth from `sample["wide"]["depth"][-1]`
  - intrinsics from `sample["sensor_info"].wide.image.K[-1]` and `.depth.K[-1]`
  - pose from `sample["sensor_info"].gt.RT`
  - timestamp from `sample["meta"]["timestamp"]`
- `Augmentor.package(...)` and `Preprocessor.preprocess(...)` build the model input tensor package.
- Depth is also backprojected with `unproject(...)` from `tools.utils`.

Artifacts read/write:

- Reads dataset frames only
- No artifact output at this stage

Classification:

- Core logic

### BoxFusion processing

Relevant files:

- `demo.py`
- `boxfusion/instances.py`
- `boxfusion/box_manager.py`
- `boxfusion/box_fusion.py`

Core logic:

- `all_pred_box` is the retained global object store.
- `per_frame_ins` is the cumulative per-frame instance store used by association/fusion.
- `BoxManager` keeps:
  - `fusion_list`
  - `fusion_flag`
  - `already_fusion`
  - `num_record`
- `Instances3D.spatial_association(...)` performs 3D OBB NMS against the retained history, optionally masked by floor/room scoped candidate masks built in `demo.py`.
- `Instances3D.correspondence_association(...)` handles small-object correspondence after NMS.
- `BoxManager.update(...)` compacts the fusion bookkeeping after suppression.
- `BoxFusion.boxfusion(...)` performs the multi-view box optimization on eligible retained objects.
- New surviving detections are semantically classified by `text_prompt(...)` with CLIP features and written back into `all_pred_box.categories` and `all_pred_box.embeddings`.

Artifacts read/write:

- No direct file artifacts
- Produces in-memory box/object state later consumed by vector-map export

Classification:

- Core logic

### Association and tracking

Relevant files:

- `demo.py`
- `boxfusion/instances.py`
- `boxfusion/box_manager.py`
- `boxfusion/floor_aware_room_segmenter.py`

Core logic:

- Object association/tracking:
  - `Instances3D.spatial_association(...)`
  - `Instances3D.correspondence_association(...)`
  - `BoxManager.record(...)`
  - `BoxManager.record_corr(...)`
  - `BoxManager.check_valid_num(...)`
- Room tracking:
  - `FloorAwareRoomSegmenter._perform_floor_segmentation(...)` delegates to per-floor `DynamicRoomSegmenter.perform_segmentation(...)`
  - local room ids are mapped to cross-run world ids by `FloorAwareRoomSegmenter._map_tracking_report(...)` using `FloorState.local_to_world_room_id`
  - the last room tracking result is stored in `room_segmenter.last_tracking_report`

Artifacts read/write:

- Tracking reports are written into exported vector-map/topology/lifecycle artifacts later
- no standalone tracking artifact during the runtime loop

Classification:

- Core logic

### Room segmentation

Relevant files:

- `demo.py`
- `boxfusion/floor_aware_room_segmenter.py`
- `boxfusion/floor_manager.py`
- `boxfusion/dynamic_room_segmenter.py`

Core logic:

- `FloorAwareRoomSegmenter.observe_frame(...)` uses `FloorManager.observe_pose(...)` to maintain the active floor hypothesis per frame.
- Stable-floor point chunks are accumulated into `FloorState.pending_chunks`.
- `FloorAwareRoomSegmenter.perform_segmentation(...)` only runs on the current stable floor.
- `_perform_floor_segmentation(...)`:
  - merges `merged_points_xyzrgb` plus pending chunks
  - downsamples with Open3D
  - calls the per-floor `DynamicRoomSegmenter.perform_segmentation(...)`
  - records a segmentation-run report
  - maps the per-floor local tracking output into world room ids
- `flush_pending_floor_segments(...)` runs the same segmentation flow once more across floors at sequence end.

Artifacts read/write:

- In-memory segmentation markers and floor/room tracking state
- later feeds `get_vector_map_data(...)`
- optional debug room vectors under `debug_room/` are written by `demo.py`

Classification:

- Core logic

### Runtime state updates and key in-memory state

Key classes:

- `Instances3D`
- `BoxManager`
- `BoxFusion`
- `FloorAwareRoomSegmenter`
- `OnlineTopologyLifecycleManager`
- `RoomScopedRuntimeManager`
- `ClosedLoopDemoRecorder`

Key in-memory state in `demo.run()`:

- `all_pred_box`: retained global object hypotheses
- `all_poses`: retained per-object camera poses
- `per_frame_ins`: cumulative per-frame instance bank
- `all_kf_pose`: keyframe pose map
- `traj_xyz`: trajectory for visualization/export
- `accumulated_all_pts`: pending point-cloud chunks for room segmentation
- `latest_vector_map`: most recent exported world graph snapshot
- `segmentation_cycle_idx`, `last_segmentation_frame_idx`

Key in-memory state in `FloorAwareRoomSegmenter`:

- `floor_manager`
- `floor_states[floor_id]`
- `last_floor_observation`
- `last_tracking_report`
- `last_room_markers`
- `_latest_export_cache`
- `_latest_object_room_metadata_snapshot`

Key in-memory state in `OnlineTopologyLifecycleManager`:

- `room_statuses: Dict[str, RoomWorkingTopologyStatus]`
- `trigger_history`
- `refresh_history`
- `active_room_id`, `active_floor_id`, `active_floor_status`

Key in-memory state in `RoomScopedRuntimeManager`:

- `local_state`
- `committed_rooms`
- `commit_history`
- `refresh_history`
- `_last_vector_map`

Key in-memory state in `ClosedLoopDemoRecorder`:

- `snapshots`
- `replay_frames`
- `revisit_events`
- `latest_vector_map`
- `runtime_growth_records`

## Stage-A Export Implementation

### Real execution path from `stage_a_demo.py`

Relevant files:

- `stage_a_demo.py`
- `boxfusion/stage_a_demo.py`
- `demo.py`

Execution path:

1. Root `stage_a_demo.py` parses CLI arguments.
2. `_run_single_sequence(...)`:
   - loads the YAML config
   - builds the runtime artifact policy
   - creates the dataset with `get_dataset(cfg)`
   - creates `ClosedLoopDemoRecorder`
   - imports and calls `demo.run(...)`
3. `demo.run(...)` executes the runtime and hands snapshot/replay data into the recorder.
4. `ClosedLoopDemoRecorder.finalize(...)` writes the Stage-A bundle.
5. `write_scene_manifest(...)` in `boxfusion/backend_eval_scaffold.py` writes `manifest.json` after the logs exist.

`stage_a_demo.py` itself is wrapper logic. The bundle-writing implementation lives in `boxfusion/stage_a_demo.py`.

### Artifact writers and data flow

#### `summary.json`

Producer:

- `ClosedLoopDemoRecorder.finalize()` in `boxfusion/stage_a_demo.py`

What it does:

- Assembles the top-level run summary after all other export steps finish
- Records paths to almost every other artifact
- Includes runtime-growth summary, replay/timeline summary, topology export status, lifecycle counts, and room-scoped export paths

Inputs:

- `run_summary` from `demo.run()`
- final snapshot / final vector map
- topology/lifecycle/working export results
- timeline summary

Classification:

- Core export logic

#### `topology_v0_1.json`

Producer:

- `RoomScopedRuntimeManager.export_artifacts(...)`

Upstream builder:

- `RoomTopologyBuilder().build(...)` in `ClosedLoopDemoRecorder.finalize()`

What happens:

1. `final_vector_map` is passed to `RoomTopologyBuilder.build(...)`.
2. `RoomTopologyBuilder` converts:
   - rooms into graph nodes
   - object/anchor containment into indices
   - gateways, room proximity, trajectory transitions, and vertical transitions into edge evidence
3. `RoomScopedRuntimeManager.build_public_topology_payload(...)` filters that full topology to committed room ids only.
4. `RoomTopology.from_dict(...).export_json(topology_path)` writes the committed/public topology.

Classification:

- Builder is core logic
- commit/public projection is core logic

#### Committed snapshot

Producer:

- `RoomScopedRuntimeManager.build_committed_world_snapshot(...)`
- written by `RoomScopedRuntimeManager.export_artifacts(...)` to `logs/committed_room_world_snapshot_v0_1.json`

What it contains:

- committed floors
- committed rooms
- objects/anchors whose room assignment is committed
- gateways and vertical transitions whose endpoints are committed

Classification:

- Core export logic

#### Committed room world model

Producer:

- `RoomScopedRuntimeManager.build_committed_room_world_model(...)`
- written by `RoomScopedRuntimeManager.export_artifacts(...)` to `logs/committed_room_world_model_v0_1.json`

What it contains:

- room summaries copied from `committed_rooms`
- adjacency/connectivity derived from committed topology edges
- room-level semantic summaries
- `bev_vln_hook` room summaries

Classification:

- Core export logic

#### Lifecycle artifact

Producer:

- `OnlineTopologyLifecycleManager.export_json(...)`

What happens:

- `ClosedLoopDemoRecorder.finalize()` first calls `finalize_report(...)` for a pre-finalize working payload
- then calls `export_json(...)` with `public_topology_export_succeeded=<bool>`
- that second call writes `logs/online_topology_lifecycle_v0_1.json`

Classification:

- Core lifecycle/debug logic

#### Working topology and working-vs-committed artifacts

Producers:

- `build_working_topology_snapshot(...)` in `boxfusion/online_topology_working_snapshot.py`
- `build_working_vs_committed_report(...)`
- `build_working_vs_committed_timeline(...)` in `boxfusion/working_vs_committed_topology_timeline.py`

What happens:

- `ClosedLoopDemoRecorder.finalize()` uses:
  - public topology payload
  - pre-finalize lifecycle payload
  - final committed room ids from lifecycle JSON
- It writes:
  - `logs/working_topology_v0_1.json`
  - `logs/working_vs_committed_topology_report_v0_1.json`
  - `logs/working_vs_committed_topology_timeline_v0_1.json`
  - `logs/working_vs_committed_topology_timeline_v0_1.md`

Semantics:

- working topology is debug-only
- committed/public topology remains the authoritative default

Classification:

- Core debug/export logic

#### Timeline and render artifacts

Producers:

- `ClosedLoopDemoRecorder._build_timeline_rows(...)`
- `ClosedLoopDemoRecorder.finalize()`

What it writes when optional demo artifacts are enabled:

- `logs/timeline.json`
- `logs/timeline.csv`
- rendered frames under `rendered_frames/`
- final PNGs under `final/`
- MP4 demo under `final/`
- revisit diagnostics and markdown reports

Important implementation detail:

- `timeline.json` is not written in core-only mode.
- The row builder uses `with_timeline_row_contract(...)` from `boxfusion/artifact_contract.py`.
- Two modes exist:
  - snapshot-only rows from `self.snapshots`
  - dense replay rows from `self.replay_frames` with held semantic map state

Classification:

- Recorder-side export logic

#### `manifest.json`

Producer:

- `write_scene_manifest(...)` in `boxfusion/backend_eval_scaffold.py`

What it does:

- Scans the scene root for the declared artifact set
- loads summary/topology/timeline/diagnostic files when present
- infers artifact surfaces and semantics via `artifact_contract.py`
- writes an inventory-style manifest containing:
  - artifact existence and paths
  - artifact profile and capability flags
  - world model summary
  - runtime summary

Classification:

- Wrapper/inventory logic over already-written artifacts

## Topology / Query / Route Implementation

### `room_topology.py`

Relevant files:

- `boxfusion/room_topology.py`

Key classes/functions:

- `RoomTopology`
- `RoomTopologyBuilder`
- `RoomTopologyBuilder.build(...)`
- `RoomTopologyBuilder.build_from_stage_a_sequence_dir(...)`
- `RoomTopology.find_room_path(...)`
- `RoomTopology.find_candidate_room_paths(...)`
- `RoomTopology.explain_connection(...)`

What it builds:

- `RoomTopology.graph`: `networkx.MultiDiGraph`
- floor records
- `object_to_room`, `anchor_to_room`
- `room_to_objects`, `room_to_anchors`
- `object_records`, `anchor_records`
- evidence table keyed by `evidence_id`

How edges are built:

- `_collect_adjacency_support(...)`
  - gateway-derived adjacency
  - polygon boundary proximity
  - door/open-passage evidence
- `_collect_transition_support(...)`
  - debounced room-switch history from `transition_history`
- `_collect_vertical_transition_support(...)`
  - explicit cross-floor transitions exported by the world model
- `_promote_possible_connections(...)`
  - exposes weak same-floor adjacency as `possible_connection`
- `_materialize_edges(...)`
  - aggregates evidence into confidence and relation status

Route computation:

- `_build_route_graph(...)` converts the multigraph to a directed route graph filtered by:
  - allowed relation types
  - minimum confidence
- edge cost is:
  - base cost from `route_relation_costs`
  - plus a confidence penalty
- `find_room_path(...)` uses `nx.shortest_path(...)`
- `find_candidate_room_paths(...)` uses `nx.shortest_simple_paths(...)`

Artifacts read/write:

- Reads vector-map payloads or a Stage-A sequence directory
- Writes `topology_v0_1.json`, query report JSON, GraphML

Classification:

- Core logic

### `query_api.py`

Relevant files:

- `boxfusion/query_api.py`

Key class/functions:

- `RoomTopologyQueryAPI`
- `from_json(...)`
- `resolve_room_target(...)`
- `resolve_anchor_room(...)`
- `resolve_object_room(...)`
- `query_route(...)`
- `query_route_to_anchor(...)`
- `query_route_to_object(...)`
- `_execute_route_query(...)`

How it works:

- Always starts from a loaded `RoomTopology` snapshot.
- Target resolution is separate from route computation.
- Object lookup supports:
  - direct object id
  - label-based lookup over exported object metadata
- Route execution:
  - selects a route policy preset (`strict`, `balanced`, `exploratory`)
  - calls `RoomTopology.find_room_path(...)`
  - optionally asks for alternate candidate paths
  - attaches a human-readable explanation

Artifacts read/write:

- Reads only `topology_v0_1.json`
- Writes nothing

Classification:

- Core query logic

### `stage_a_topology_*.py`

Relevant files:

- `stage_a_topology_export.py`
- `stage_a_topology_query.py`
- `stage_a_topology_route.py`
- `stage_a_topology_acceptance.py`

What they do:

- `stage_a_topology_export.py`
  - thin CLI wrapper around `RoomTopologyBuilder.build_from_stage_a_sequence_dir(...)`
  - reads `logs/timeline.json` plus referenced vector-map snapshots from a Stage-A sequence dir
- `stage_a_topology_query.py`
  - thin CLI wrapper around `RoomTopologyQueryAPI`
- `stage_a_topology_route.py`
  - thin CLI wrapper around `RoomTopology.find_room_path(...)`
- `stage_a_topology_acceptance.py`
  - runs basic acceptance exercises over exported topology/query behavior

Classification:

- Wrapper logic

## Lifecycle / Publication Implementation

### Lifecycle state representation

Relevant files:

- `boxfusion/online_topology_lifecycle.py`
- `boxfusion/room_scoped_runtime.py`
- `boxfusion/online_topology_working_snapshot.py`

Key types:

- `RoomLifecycleState`
  - `discovering`
  - `active`
  - `candidate_complete`
  - `committed`
  - `revisitable`
  - `merge_or_split_pending`
- `RoomWorkingTopologyStatus`
- `CandidateBlockReason`
- `CommitBlockReason`
- `PublicationDiagnosticState`

What is mutable:

- `OnlineTopologyLifecycleManager.room_statuses[room_id]` holds the mutable per-room lifecycle status.
- `observe_room_tracking(...)` reacts to room entry/exit/revisit events.
- `observe_export(...)` reacts to each exported vector map and updates signatures, blockers, readiness, and dirty flags.

### Working / committed / published concepts in current code

Working:

- Implemented as debug-only derived snapshots from public topology plus lifecycle payload
- Produced by `build_working_topology_snapshot(...)`
- Written to `working_topology_v0_1.json`

Committed:

- Implemented as `RoomScopedRuntimeManager.committed_rooms`
- Commit occurs in `RoomScopedRuntimeManager.observe(...)` when lifecycle status satisfies:
  - `candidate_complete`
  - no `commit_block_reasons`
  - `present_in_latest_export`
- Exported public topology/snapshot/world-model are filtered to these committed room ids

Published:

- In public artifact terms, "published" means the committed/public projection that is actually exported
- In diagnostics terms, publication state is derived by `derive_publication_diagnostics_from_payload(...)`
- ROS publication diagnostics server serves this derived debug publication view from lifecycle JSON

### Boundary enforcement

Modules/functions enforcing boundaries:

- `RoomScopedRuntimeManager.build_public_topology_payload(...)`
  - filters full topology to committed room ids only
- `RoomScopedRuntimeManager.build_committed_world_snapshot(...)`
  - filters vector-map state to committed room ids only
- `RoomScopedRuntimeManager.export_artifacts(...)`
  - writes the authoritative public topology and committed snapshot
- `CommittedPublicBundle.from_manifest_path(...)` in `ros_query_server.py`
  - rejects non-public or non-committed topology artifacts
- `CommittedPublicBundle.from_input_path(...)`
  - explicitly rejects `working_topology_v0_1.json` as public query input
- `PublicationDiagnosticsBundle...`
  - loads lifecycle/debug artifacts separately

### Is there a real state machine?

Yes, but it is split into two layers.

1. Mutable lifecycle state machine:
   - implemented directly in `OnlineTopologyLifecycleManager`
   - state transitions are applied in:
     - `observe_room_tracking(...)`
     - `observe_export(...)`
     - `_evaluate_room_status(...)`
   - room state lives in `RoomWorkingTopologyStatus.lifecycle_state`

2. Derived publication-state machine:
   - implemented by `derive_publication_diagnostics_from_payload(...)`
   - output states:
     - `ACTIVE_OBSERVING`
     - `CANDIDATE_FORMED`
     - `FINALIZATION_PENDING`
     - `FINALIZED_PRIVATE`
     - `COMMIT_READY`
     - `PUBLISHED`
   - this is diagnostic/derived; it is not the main mutable storage

## ROS Implementation

### Query server

Relevant files:

- `boxfusion/ros_query_server.py`
- `ros_interfaces/srv/*.srv`

Services defined in `ros_interfaces/`:

- `GetWorldSnapshot`
- `GetTopology`
- `ResolveObjectRoom`
- `RouteToRoom`
- `RouteToObject`
- `ExplainConnection`
- `GetPublicationDiagnostics`
- `GetRoomPublicationState`

How the query server wraps the bundle:

- `CommittedPublicBundle` resolves a public scene bundle from:
  - `manifest.json`
  - or `summary.json`
  - or `topology_v0_1.json`
- `BoxFusionRosQueryServerBackend` loads `RoomTopologyQueryAPI` from the committed topology JSON
- Service methods are thin wrappers:
  - `get_world_snapshot()`
  - `get_topology()`
  - `resolve_object_room()`
  - `route_to_room()`
  - `route_to_object()`
  - `explain_connection()`
- ROS handlers just translate request fields into backend calls and return JSON strings

Reads:

- exported artifacts only
- not live runtime state

### Diagnostics server

Relevant files:

- `boxfusion/ros_publication_diagnostics_server.py`

How it wraps the bundle:

- `PublicationDiagnosticsBundle` resolves `online_topology_lifecycle_v0_1.json`
- `BoxFusionRosPublicationDiagnosticsBackend` loads the lifecycle payload and derives publication-state summaries
- Services:
  - `get_publication_diagnostics()`
  - `get_room_publication_state(room_id=...)`

Reads:

- exported lifecycle/debug artifacts only
- not live runtime state

### `runtime_export_coordinator.py`

Relevant files:

- `boxfusion/runtime_export_coordinator.py`

What it actually does:

- optionally runs a producer subprocess before each refresh
- refreshes `manifest.json` via `write_scene_manifest(...)`
- loads:
  - committed/public bundle through `load_committed_public_bundle(...)`
  - lifecycle/debug bundle through `load_publication_diagnostics_bundle(...)`
- builds a combined `RuntimeStateSnapshot` with `build_runtime_snapshot_from_bundles(...)`
- optionally writes a shadow sidecar subset
- updates stable pointers under `coordination_root`:
  - `latest` symlink to the scene root
  - `latest_manifest.json` symlink
  - `latest_export.json` metadata
  - timestamped refresh records in `refresh_history/`

It does not:

- run live queries against in-memory runtime state
- replace the synchronous Stage-A exporter
- publish incremental topology updates itself

Classification:

- Wrapper/coordinator logic around exported bundles

## Simulation Ingress / Handoff Implementation

### `ros_simulation_ingress.py`

Relevant files:

- `boxfusion/ros_simulation_ingress.py`
- `boxfusion/runtime_export_coordinator.py`
- root `stage_a_demo.py`

Key classes/functions:

- `SimulationImageInput`
- `SimulationCameraInfoInput`
- `SimulationPoseInput`
- `StageASimulationSnapshotWriter`
- `materialize_real_backend_input_sequence(...)`
- `run_real_backend_producer(...)`
- `write_stage_a_scene_root(...)`
- `refresh_current_backend(...)`

How ROS topics become saved samples:

- The ROS node caches latest RGB, depth, camera info, and pose.
- On timer ticks, `_try_record_snapshot()` checks synchronization and calls `writer.record_sample(...)`.
- `record_sample(...)`:
  - assigns a `frame_idx`
  - writes raw image payloads to `frames/frame_XXXXXX_{rgb|depth}.bin` when enabled
  - appends a JSONL row to `logs/simulation_ingress_frames.jsonl`

### Stub Stage-A bundle path

If `backend_mode == stub`:

- `write_stage_a_scene_root()` writes a minimal synthetic scene root directly:
  - `topology_v0_1.json`
  - `topology_query_report.json`
  - `vertical_transition_evidence.json`
  - `floor_diagnostics_summary.json`
  - `runtime_growth_profile.*`
  - `online_topology_lifecycle_v0_1.json`
  - `working_*`
  - `final_vector_map_snapshot.json`
  - `summary.json`
  - `manifest.json`

This stub path is wrapper logic, not the real backend.

### Real backend handoff

If `backend_mode == real_backend`:

- `materialize_real_backend_input_sequence(...)` converts captured ROS samples into a BoxFusion input directory:
  - writes `rgb/*.png`
  - writes `depth/*.png`
  - writes `pose/*.txt` for HM3D handoff, or `all_poses.npy` and `K_*.txt` for CA1M handoff
  - writes a generated YAML config
  - writes `simulation_ingress_backend_input_manifest.json`

- `run_real_backend_producer(...)` then shells out to the real producer.

Where `stage_a_demo.py` is called:

- `_format_real_backend_command(...)` builds the subprocess command.
- Default command is:
  - `python stage_a_demo.py hm3d ...` for HM3D handoff
  - or `python stage_a_demo.py CA1M ...` for CA1M handoff

The contract passed to `stage_a_demo.py` is file-based:

- generated dataset/config root under `real_backend_input_root`
- generated YAML config path
- model path
- CLIP checkpoint path
- text feature path
- output root
- `--core-only`
- `--runtime-artifact-mode`
- `--max-frames`, `--keyframe-gap`, `--room-seg-interval`, `--capture-stride`, `--runtime-profile-interval`

After producer completion:

- `refresh_coordinator(...)` points the export coordinator at either:
  - the real backend scene root
  - or the stub scene root on fallback

Classification:

- topic capture and handoff are wrapper logic
- the subprocess Stage-A run is the actual core backend path

## Replay / VLN Implementation

### Replay-source ROS path

Relevant files:

- `boxfusion/ros_simulation_replay_source.py`
- `boxfusion_ros_query_server/launch/simulation_replay_source.launch.py`

How it works:

- It does not read `timeline.json`.
- It replays `logs/simulation_ingress_frames.jsonl` produced by simulation ingress.
- `load_simulation_capture_sequence(...)` loads saved RGB/depth payload paths plus pose/camera info.
- `BoxFusionSimulationReplaySourceNode` publishes them back out as ROS messages on a timer.

So the ROS replay-source path is a topic replay wrapper for ingress capture logs, not a topology/timeline replay path.

### Replay-backed VLN / tool-use / end-to-end demo

Relevant files:

- `boxfusion/replay_timeline.py`
- `boxfusion/vln_closed_loop.py`
- `boxfusion/vln_tool_use.py`
- `boxfusion/vln_end_to_end_demo.py`
- `stage_a_minimal_vln_closed_loop.py`
- `stage_a_vln_tool_use_demo.py`
- `stage_a_end_to_end_vln_demo.py`

How it works:

- `load_replay_observations(timeline_json, query_api)` loads ordered observations from `timeline.json`.
- `VLNClosedLoopExecutor.from_paths(topology_json, timeline_json)` combines:
  - static query topology
  - ordered replay observations
- `build_symbolic_plan(...)` uses `RoomTopologyQueryAPI` to produce a room-level plan.
- `VLNClosedLoopExecutor.execute(...)` walks through replay observations and checks whether the observed room sequence completes the symbolic plan.
- `MinimalVLNToolUseAdapter` adds narrow NL parsing and tool selection.
- `VLNEndToEndDemoOrchestrator` packages the parsed request, selected backend call, route summary, execution trace, and teacher-facing outputs.

### Which routes require `timeline.json`, and why

Require `timeline.json`:

- `VLNClosedLoopExecutor.from_paths(...)`
- `stage_a_minimal_vln_closed_loop.py`
- `stage_a_vln_tool_use_demo.py` when execute mode needs a closed-loop executor
- `stage_a_end_to_end_vln_demo.py`
- `boxfusion/world_model_eval.py`

Reason:

- These routes need an ordered replay of observed rooms/floors across time.
- Static `topology_v0_1.json` is enough for final-state query and static route planning.
- It is not enough for:
  - checking whether a replay actually followed the route
  - reconstructing room transitions over time
  - deriving execution traces and completion frames

Do not require `timeline.json`:

- `RoomTopologyQueryAPI`
- `stage_a_topology_query.py`
- `stage_a_topology_route.py`
- ROS query server
- ROS publication diagnostics server

## Shared Backbone vs Wrappers

### Shared backbone modules

- `demo.py`
  - live runtime loop
- `boxfusion/floor_aware_room_segmenter.py`
  - floor-aware room segmentation and vector-map export
- `boxfusion/instances.py`
  - instance storage plus association routines
- `boxfusion/box_manager.py`
  - retained fusion bookkeeping
- `boxfusion/box_fusion.py`
  - multi-view box optimization
- `boxfusion/stage_a_demo.py`
  - snapshot/replay recorder and Stage-A export assembly
- `boxfusion/room_topology.py`
  - topology builder and route graph
- `boxfusion/query_api.py`
  - public snapshot query layer
- `boxfusion/online_topology_lifecycle.py`
  - lifecycle/debug state machine
- `boxfusion/room_scoped_runtime.py`
  - commit/public projection

These are the main current core modules reused across routes.

### Thin wrappers around shared logic

- root `stage_a_demo.py`
  - CLI/config wrapper around `demo.run()` and `ClosedLoopDemoRecorder`
- `stage_a_topology_export.py`
- `stage_a_topology_query.py`
- `stage_a_topology_route.py`
- `stage_a_topology_acceptance.py`
- `boxfusion/ros_query_server.py`
- `boxfusion/ros_publication_diagnostics_server.py`
- `boxfusion/runtime_export_coordinator.py`
- `boxfusion/ros_simulation_ingress.py`
- `boxfusion/ros_simulation_replay_source.py`
- `stage_a_minimal_vln_closed_loop.py`
- `stage_a_vln_tool_use_demo.py`
- `stage_a_end_to_end_vln_demo.py`

### Legacy shims or thin entrypoints

- root `demo.py` has both:
  - the real core function `run(...)`
  - a large older CLI block under `if __name__ == "__main__":`
- root `stage_a_demo.py` is a thin entrypoint over the real export/runtime pieces in:
  - `demo.py`
  - `boxfusion/stage_a_demo.py`

## Key Artifacts and Their Producers/Consumers

| Artifact | Producer | Main consumers | Surface |
| --- | --- | --- | --- |
| `logs/summary.json` | `ClosedLoopDemoRecorder.finalize()` | manifest builder, diagnostics tools, bundle resolvers | public |
| `logs/topology_v0_1.json` | `RoomScopedRuntimeManager.export_artifacts()` from `RoomTopologyBuilder.build()` output | `RoomTopology`, `RoomTopologyQueryAPI`, ROS query server, VLN planners | public |
| `logs/topology_query_report.json` | `RoomScopedRuntimeManager.export_artifacts()` | summary/inspection tooling, query server bundle metadata | public |
| `logs/committed_room_world_snapshot_v0_1.json` | `RoomScopedRuntimeManager.export_artifacts()` | ROS `GetWorldSnapshot`, bundle readers | public |
| `logs/committed_room_world_model_v0_1.json` | `RoomScopedRuntimeManager.export_artifacts()` | room-commit diagnosis, room-scoped consumers | public |
| `logs/online_topology_lifecycle_v0_1.json` | `OnlineTopologyLifecycleManager.export_json()` | ROS publication diagnostics server, working-topology builders, publication/commit diagnostics | lifecycle |
| `logs/working_topology_v0_1.json` | `build_working_topology_snapshot(...)` | debug comparison/report tooling | working |
| `logs/working_vs_committed_topology_report_v0_1.json` | `build_working_vs_committed_report(...)` | debug comparison/report tooling | working |
| `logs/working_vs_committed_topology_timeline_v0_1.json` | `build_working_vs_committed_timeline(...)` | publication-policy/debug tooling | working |
| `logs/timeline.json` | `ClosedLoopDemoRecorder._build_timeline_rows()` | replay/VLN/execution/eval modules | public |
| `manifest.json` | `write_scene_manifest(...)` | coordinator, ROS bundle loaders, registry/eval tooling | public |
| `logs/floor_diagnostics_summary.json` | `ClosedLoopDemoRecorder.finalize()` plus `room_segmenter.save_floor_diagnostics()` side outputs | diagnostics/eval tooling | diagnostic |
| `logs/runtime_growth_profile.json` and `.csv` | `ClosedLoopDemoRecorder.finalize()` | eval/diagnostics | diagnostic |
| render/video artifacts | `ClosedLoopDemoRecorder.finalize()` | human inspection only | public |

## Most Important Current Technical Bottlenecks (based on code structure only)

1. `demo.run()` is a single very large orchestrator function. Detection, point-cloud handling, segmentation scheduling, association, BoxFusion, export triggering, profiling, and recorder callbacks all live in one control loop.

2. Full export assembly still rebuilds several views from the final vector map at finalize time. Public topology, working topology, lifecycle reports, room-scoped exports, manifests, and optional replay/render artifacts are all layered after the runtime rather than being incrementally materialized through one shared export pipeline.

3. Similar state is tracked in multiple managers with different semantics. Room/floor state exists in `FloorAwareRoomSegmenter`, lifecycle state in `OnlineTopologyLifecycleManager`, committed/public state in `RoomScopedRuntimeManager`, and snapshot/replay state in `ClosedLoopDemoRecorder`.

4. The real simulation ingress path is file-and-subprocess based. `ros_simulation_ingress.py` materializes disk inputs, shells out to `stage_a_demo.py`, then refreshes a coordinator pointer. That is robust and explicit, but it is not an in-process handoff.

5. Replay-dependent routes and static-query routes diverge at the artifact boundary. Query/ROS/topology consumers only need `topology_v0_1.json`, while replay/VLN/eval routes additionally require `timeline.json`, which is only produced when optional demo artifacts are enabled.
