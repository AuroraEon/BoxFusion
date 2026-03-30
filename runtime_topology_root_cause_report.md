# Runtime / Topology Root Cause Report

Context note:
- Code inspected: `demo.py`, `boxfusion/instances.py`, `boxfusion/box_fusion.py`, `boxfusion/floor_aware_room_segmenter.py`, `boxfusion/dynamic_room_segmenter.py`, `boxfusion/scene_graph_builder.py`, `boxfusion/stage_a_demo.py`, `boxfusion/room_topology.py`.
- Concrete late-runtime evidence used: `world_model_backend_outputs_v0_2_final/scenes/00862-LT9Jq6dN3Ea/logs/runtime_growth_profile.csv`, `.../summary.json`, and `.../floor_diagnostics_summary.json`.
- I used scene `00862-LT9Jq6dN3Ea` for the late-frame evidence because it contains an actual logged frame `7000` sample.

## 1. Executive diagnosis

### Most likely root causes of the 56.8516 s feature extraction & BoxFusion cost

Primary diagnosis:
- The timing bucket is mislabeled in practice. `feature_boxfusion_sec` in `demo.py` covers not only CLIP feature extraction and BoxFusion, but also snapshot export work via `maybe_capture_demo_snapshot(...)`, which calls `room_segmenter.get_vector_map_data(...)` inside the stage-5 timer (`demo.py:135-161`, `demo.py:381-549`).
- `get_vector_map_data(...)` rebuilds a full world export from global state on every captured keyframe: all rooms, all gateways, all vertical transitions, all objects, a fresh `SemanticSceneGraph`, pairwise spatial relations, anchors, relationships, and floor diagnostics (`boxfusion/floor_aware_room_segmenter.py:282-407`).
- On segmentation refresh frames, this export happens twice on the same frame:
  - once in stage 3 immediately after segmentation (`demo.py:320-340`)
  - again in stage 5 snapshot capture (`demo.py:541-547` + `demo.py:135-148`).

Actual fusion / association growth also exists:
- Current keyframe detections are merged into `all_pred_box`, then `Instances3D.spatial_association(...)` runs 3D NMS over the full concatenated global set (`demo.py:457-470`, `boxfusion/instances.py:372-397`, `boxfusion/instances.py:22-101`).
- `correspondence_association(...)` projects all retained global boxes into the current frame for each small current box (`boxfusion/instances.py:411-490`).
- `BoxFusion.boxfusion(...)` scans every retained global object each keyframe, and for each eligible candidate runs up to `fusion_iters=20` optimization rounds, with `pst_size=1024` particles per round (`config/hm3d.yaml:49-61`, `boxfusion/box_fusion.py:417-465`, `boxfusion/box_fusion.py:626-728`).
- `per_frame_ins` is cumulative and never pruned in the main loop (`demo.py:115-116`, `demo.py:460-462`), so BoxFusion’s historical detection store grows with scene history even though each fused object uses only a small selected view set.

Important nuance:
- Literal CLIP feature extraction is local to the current kept boxes (`demo.py:438`, `demo.py:524`, `tools/utils.py:389-454`).
- The expensive part is not “all-history feature embedding comparison” in the CLIP sense.
- The expensive part is global object association, global historical detection retention, and repeated full world export inside the same timing bucket.

### Most likely root causes of the 13.3868 s topology generation & room segmentation cost

Primary diagnosis:
- Room segmentation is active-floor scoped, but within that floor it is global-growing, not local/windowed.
- `FloorAwareRoomSegmenter._merge_floor_points(...)` merges the retained floor cloud with all pending chunks and voxel-downsamples the whole thing every segmentation run (`boxfusion/floor_aware_room_segmenter.py:420-438`).
- `DynamicRoomSegmenter.perform_segmentation(...)` then reprocesses that full accumulated floor cloud into a growing histogram / grid / watershed map (`boxfusion/dynamic_room_segmenter.py:834-1106`).
- The grid only grows and never shrinks (`grid_width`, `grid_height`) (`boxfusion/dynamic_room_segmenter.py:19-27`, `boxfusion/dynamic_room_segmenter.py:893-914`).
- After segmentation, the code also rebuilds global vector-map / topology-like exports from scratch on the same frame via `get_vector_map_data(...)` (`demo.py:326-340`, `boxfusion/floor_aware_room_segmenter.py:282-407`).

Secondary contributors inside segmentation:
- `_build_segmentation_state(...)` runs full-map distance transform, contour extraction, watershed, and room-adjacency counting over the full floor grid (`boxfusion/dynamic_room_segmenter.py:600-685`).
- `_update_room_tracking(...)` compares current room masks against tracked historical masks by IoU over full marker arrays (`boxfusion/dynamic_room_segmenter.py:710-793`).
- `_extract_gateways(...)` does pairwise room-mask checks over all room pairs on the segmentation grid (`boxfusion/dynamic_room_segmenter.py:1108-1188`).

### Is the advisor’s suspicion correct?

Yes, broadly yes.

Precise statement:
- Current processing is mixed with retained global history in object association and fusion candidate generation.
- Room segmentation is mixed with retained global floor history, not current-room or short-window state.
- Full topology export is not online-incremental; the explicit `RoomTopologyBuilder` graph is built only at finalization (`boxfusion/stage_a_demo.py:1223-1238`).
- The only existing “current room” notion is mostly for recorder / diagnostic use (`boxfusion/stage_a_demo.py`, `RevisitDetector`, `_best_room_for_pose_floor_aware`), not for online backend triggering.

## 2. Code-path analysis

### A. Feature extraction & BoxFusion

Main entry point:
- `demo.py` stage 5 block (`demo.py:378-582`)

Important called functions:
- `text_prompt(...)` for current-frame CLIP classification (`demo.py:438`, `demo.py:524`, `tools/utils.py:441-454`)
- `Instances3D.spatial_association(...)` (`demo.py:469`, `boxfusion/instances.py:372-397`)
- `nms_3d(...)` and `calculate_obb_iou(...)` (`boxfusion/instances.py:22-101`)
- `Instances3D.correspondence_association(...)` (`demo.py:480-496`, `boxfusion/instances.py:411-490`)
- `BoxFusion.boxfusion(...)` (`demo.py:511-512`, `boxfusion/box_fusion.py:626-728`)
- `maybe_capture_demo_snapshot(...)` (`demo.py:135-161`, called at `demo.py:395-401`, `demo.py:541-547`)
- `FloorAwareRoomSegmenter.get_vector_map_data(...)` via snapshot capture (`demo.py:142-148`)

Important loops / growth points:
- `per_frame_ins = Instances3D.cat([per_frame_ins, pred_instances])` retains every keyframe’s raw detections (`demo.py:460-462`).
- `all_pred_box = Instances3D.cat([all_pred_box, pred_instances])` grows the retained global object set (`demo.py:457-463`).
- `nms_3d(...)` repeatedly computes IoU from one box to all remaining boxes (`boxfusion/instances.py:58-93`).
- `correspondence_association(...)` loops over current small boxes and projects all retained global boxes into the current frame (`boxfusion/instances.py:446-463`).
- `BoxFusion.boxfusion(...)` loops over `range(N_box)` for all retained global objects every keyframe (`boxfusion/box_fusion.py:635-639`).
- For each eligible fusion candidate, optimization loops:
  - over up to `fusion_iters=20` iterations (`boxfusion/box_fusion.py:668-718`)
  - over `pst_size=1024` search particles inside `evaluate_iou(...)` (`boxfusion/box_fusion.py:417-456`)
  - over `num_of_boxes` historical views in the CUDA grid (`boxfusion/box_fusion.py:454-455`).
- Snapshot export rebuilds all objects / anchors / relationships from scratch every capture (`boxfusion/floor_aware_room_segmenter.py:282-407`).

Complexity class:
- CLIP feature extraction: local, current-frame only.
- Object association: global-growing.
- BoxFusion historical store: global-growing.
- Snapshot / vector map export: global-growing.

Likely asymptotic growth drivers:
- retained global object count `|all_pred_box|`
- cumulative historical per-keyframe detection count `|per_frame_ins|`
- number of current-frame small detections
- number of unfused / newly eligible fusion candidates
- total exported object count / room count / anchor count during `get_vector_map_data(...)`

### B. Topology generation & room segmentation

Main entry point:
- `demo.py` stage 3 block (`demo.py:290-344`)

Important called functions:
- `room_segmenter.observe_frame(...)` (`demo.py:279-285`, `demo.py:295-302`)
- `FloorAwareRoomSegmenter.perform_segmentation(...)` (`demo.py:320-324`, `boxfusion/floor_aware_room_segmenter.py:108-134`)
- `FloorAwareRoomSegmenter._perform_floor_segmentation(...)` (`boxfusion/floor_aware_room_segmenter.py:202-280`)
- `FloorAwareRoomSegmenter._merge_floor_points(...)` (`boxfusion/floor_aware_room_segmenter.py:420-438`)
- `DynamicRoomSegmenter.perform_segmentation(...)` (`boxfusion/dynamic_room_segmenter.py:834-1106`)
- `_build_segmentation_state(...)` (`boxfusion/dynamic_room_segmenter.py:600-685`)
- `_update_room_tracking(...)` (`boxfusion/dynamic_room_segmenter.py:710-793`)
- `_extract_gateways(...)` (`boxfusion/dynamic_room_segmenter.py:1108-1188`)
- `get_vector_map_data(...)` after segmentation (`demo.py:326-340`)

Important loops / growth points:
- `observe_frame(...)` appends stable keyframe chunks into `pending_chunks` per floor (`boxfusion/floor_aware_room_segmenter.py:101-104`).
- `_merge_floor_points(...)` concatenates retained merged cloud plus pending chunks and downsamples the full retained floor cloud each run (`boxfusion/floor_aware_room_segmenter.py:420-438`).
- `DynamicRoomSegmenter.perform_segmentation(...)` computes histograms over all retained floor points (`boxfusion/dynamic_room_segmenter.py:904-980`).
- The grid expands to fit new max extents and never shrinks (`boxfusion/dynamic_room_segmenter.py:896-914`).
- `_build_segmentation_state(...)` operates over the full grid image, not just local ROI (`boxfusion/dynamic_room_segmenter.py:600-685`).
- `_update_room_tracking(...)` compares each current room mask to each historical tracked room mask (`boxfusion/dynamic_room_segmenter.py:715-791`).
- `_extract_gateways(...)` loops over every room-pair combination on the full marker map (`boxfusion/dynamic_room_segmenter.py:1127-1188`).
- `get_vector_map_data(...)` then rebuilds all-floor rooms, all objects, all anchors, all relationships, all vertical transitions (`boxfusion/floor_aware_room_segmenter.py:282-407`).

Complexity class:
- Segmentation cloud scope: active-floor global-growing.
- Export scope after segmentation: scene-global across all exported floors / rooms / objects.

Likely asymptotic growth drivers:
- active-floor retained point count
- active-floor raster grid area
- active-floor room count
- global object count
- global anchor count
- full `frame_history` length used for vertical-transition export

## 3. Dataflow analysis at late runtime

### Late frame used for concrete evidence

From `world_model_backend_outputs_v0_2_final/scenes/00862-LT9Jq6dN3Ea/logs/runtime_growth_profile.csv`:
- frame `7000`
- `global_box_count = 549`
- `room_count = 36`
- `object_count = 549`
- `anchor_count = 69`
- `topology_room_segmentation_sec = 12.829858`
- `feature_boxfusion_sec = 60.015179`

From `.../floor_diagnostics_summary.json`, segmentation run at frame `7000`:
- active floor: `floor_3` / display floor `floor_1`
- pending chunks fused this run: `4`
- pending chunk frame range: `6925 -> 7000`
- merged point count after floor merge/downsample: `154646`
- room count after segmentation: `7`
- tracking summary: `matched_count=6`, `new_room_count=1`

### What data structures are alive by late runtime

Object / semantic side:
- `all_pred_box`: retained global object set, length `549` at frame 7000.
- `per_frame_ins`: cumulative raw per-keyframe detections across the full history. Exact size at frame 7000 is not logged, but code proves it grows monotonically and is not pruned (`demo.py:460-462`).
- `all_kf_pose`: dictionary of all past keyframe poses.
- `box_manager.fusion_list`: one historical candidate-view list per retained object, pointing into `per_frame_ins`.

Topology / room side:
- `FloorAwareRoomSegmenter.floor_states`: one state per floor.
- Each `FloorState` retains:
  - `merged_points_xyzrgb`: retained downsampled floor cloud
  - `pending_chunks`
  - room-id tracking state
  - segmentation reports
- `frame_history`: full floor-assignment / pose history for the whole run.
- `DynamicRoomSegmenter.tracked_rooms`: retained room masks for room tracking on that floor.

### What current-frame data are produced

At a keyframe:
- `pred_instances` from the detector (`demo.py:234-245`)
- current-frame point cloud chunk `xyzrgb_down` (`demo.py:262-285`)
- current-frame pose / image / timestamp

### What historical / global data are accessed

For object association / fusion:
- `all_pred_box` global retained object history
- `per_frame_ins` global raw detection history
- `all_kf_pose` historical poses
- `box_manager.fusion_list` historical candidate groups

For room segmentation:
- retained active-floor `merged_points_xyzrgb`
- historical room masks in `tracked_rooms`
- full `frame_history` for later transition export
- global `all_pred_box` again, because door boxes are read during segmentation and object export

### What subset is actually fused against the current frame

Not a clean local subset.

Per substep:
- CLIP/text classification:
  - only current kept boxes.
- 3D spatial association:
  - current frame is fused against the full concatenated object set `all_pred_box + pred_instances`, so this is close to all retained history.
- small-object correspondence:
  - current small boxes are matched against all retained global boxes surviving keep-mask filtering.
- BoxFusion optimizer:
  - the optimizer for each object uses a selected historical view subset from `fusion_list`, typically capped to a small number of views by `BoxManager.record(...)`.
  - however, the system still scans all retained global objects each keyframe to find such candidates.
- snapshot / vector-map export:
  - full current world state, not a local room/window.

So the accurate answer is:
- not literally “all historical features are re-embedded every frame,”
- but yes, current-frame object association is checked against near-global retained object history,
- and the overall stage-5 runtime is dominated by global world-export work plus global association logic.

### What exact subset is processed at frame 7000

What is provable from code + logs:
- segmentation input: the full retained cloud for active floor `floor_3`, size `154646` points after merge/downsample, plus global `all_pred_box` for door carving and export
- retained object set: `549` objects
- exported room/object/anchor world state: `36 / 549 / 69`

What is not directly logged:
- exact `len(pred_instances)` for frame 7000
- exact `len(per_frame_ins)` at frame 7000
- exact number of objects entering the expensive BoxFusion optimization loop that frame

Those unknowns should be treated as hypotheses until instrumented.

## 4. Topology / room-segmentation diagnosis

### Does room segmentation reprocess an accumulated global point cloud / map?

Yes, but floor-scoped rather than whole-scene scoped.

Specifically:
- `perform_segmentation(...)` picks the active stable floor (`boxfusion/floor_aware_room_segmenter.py:108-134`).
- `_merge_floor_points(...)` merges the retained active-floor map with new chunks and voxel-downsamples the whole retained floor cloud every run (`boxfusion/floor_aware_room_segmenter.py:420-438`).
- `DynamicRoomSegmenter.perform_segmentation(...)` then rebuilds the floor grid / watershed state from that whole retained floor cloud (`boxfusion/dynamic_room_segmenter.py:834-1106`).

So this is:
- not all floors at once
- but also not current room, active room, or short temporal window

### Does topology generation rebuild too much from scratch?

Yes.

Online export path:
- `get_vector_map_data(...)` rebuilds the full export structure from scratch every call (`boxfusion/floor_aware_room_segmenter.py:282-407`).
- It re-exports all floors, rooms, objects, anchors, relationships, vertical transitions, and diagnostics.
- It rebuilds a fresh `SemanticSceneGraph`, recomputes pairwise spatial relations, and rebuilds the anchor layer from scratch.

Final explicit topology graph:
- `RoomTopologyBuilder().build(...)` is only called in `ClosedLoopDemoRecorder.finalize(...)` (`boxfusion/stage_a_demo.py:1223-1238`).
- That means the canonical room-topology graph is still post-hoc, not incremental online topology.

### Is there any existing notion of “current room”, “active room”, or “room completion”?

Partial notions exist, but not as backend control signals.

Existing notions:
- active floor: yes, via `FloorManager` / `last_floor_observation`
- current room id: yes, but mainly inside recorder / visualization / revisit diagnostics (`boxfusion/stage_a_demo.py`)
- room revisit detection: yes, in `RevisitDetector`

Missing notions:
- no room-completion signal used to freeze a room submap
- no room-leaving trigger used to incrementally finalize room topology
- no active-room semantic working set used to bound fusion or export

### What is missing for incremental online-topology trigger?

Missing pieces:
- a backend-owned `current_room_id` used during mapping, not just after export
- stable “entered room / left room / room complete” state machine
- room-local buffers / submaps that can be finalized incrementally
- incremental topology edge updates on room transitions
- a distinction between:
  - local working state
  - retained per-room memory
  - scene-level query/export graph

## 5. Evidence-backed advisor alignment

### Local/current vs global/history separation

Advisor target:
- separate current/local state from retained global memory

Current implementation:
- detector inference is current-frame local
- but object association immediately pulls current detections into global `all_pred_box`
- `per_frame_ins` retains raw detections across the full keyframe history
- floor segmentation retains and reuses the full floor cloud
- vector-map export rebuilds the full current world each time

Assessment:
- only partial separation exists
- the dominant runtime-critical paths are still global-history coupled

### Fusion with relevant subset vs fusion with all history

Advisor target:
- fuse current evidence only with a relevant subset, ideally current room or a short recent window

Current implementation:
- CLIP classification is current-only
- but geometric association is against the retained global object set
- small-object correspondence uses projected all-global boxes
- BoxFusion uses historical detection subsets selected from a global store
- no room-scoped or recency-window candidate filter is applied before global association

Assessment:
- the implementation differs materially from the advisor’s desired relevant-subset fusion design

### Room-scoped incremental topology vs retained/post-hoc topology

Advisor target:
- trigger topology incrementally from room completion / room leaving

Current implementation:
- segmentation trigger is fixed interval: `if count % room_seg_interval == 0` (`demo.py:306`)
- floor flush only happens at end of sequence (`demo.py:649-654`, `boxfusion/floor_aware_room_segmenter.py:136-200`)
- final `RoomTopologyBuilder` graph is built only during recorder finalization (`boxfusion/stage_a_demo.py:1223-1238`)

Assessment:
- online topology is still incomplete
- current implementation is interval-driven and post-hoc, not room-completion-driven

## 6. Minimal instrumentation plan

Code inspection is enough to identify the dominant structural causes, but not enough to attribute the exact 60 s split among stage-5 substeps. The smallest useful instrumentation would be:

### Stage-5 object / fusion instrumentation

Add per-keyframe logs for:
- `len(pred_instances)`
- `len(all_pred_box)` before and after association
- `len(per_frame_ins)`
- `len(cur_keep_idx)`
- `len(cur_success_nms)`
- number of `small_idx` candidates in `correspondence_association(...)`
- number of BoxFusion candidates where `len(fusion_list[i]) >= 3` and `not already_fused`
- histogram of `fusion_list` lengths

Add separate timers around:
- detector-to-CLIP classification
- `Instances3D.spatial_association(...)`
- `Instances3D.correspondence_association(...)`
- `Box_Fuser.boxfusion(...)`
- `maybe_capture_demo_snapshot(...)`
- inside `maybe_capture_demo_snapshot(...)`, time `get_vector_map_data(...)`

### Stage-3 topology / segmentation instrumentation

Add per-segmentation-run logs for:
- active floor id
- merged point count before / after downsample
- pending chunk count and frame span
- `grid_width`, `grid_height`, and `grid_width * grid_height`
- wall-slice point count and full-slice point count
- current room count after segmentation
- number of tracked rooms before / after tracking
- number of room pairs scanned in `_extract_gateways(...)`
- number of gateways produced

Add separate timers around:
- `_merge_floor_points(...)`
- `DynamicRoomSegmenter.perform_segmentation(...)`
- inside it:
  - histogram construction
  - `_build_segmentation_state(...)`
  - `_update_room_tracking(...)`
  - `_extract_gateways(...)`
- `get_vector_map_data(...)`
- inside `get_vector_map_data(...)`:
  - `_export_rooms_and_gateways(...)`
  - `_build_vertical_transitions(...)`
  - `_build_object_exports(...)`
  - `sg.compute_spatial_relations(...)`
  - `sg.build_anchor_layer(...)`

### One important instrumentation correction

Split the current `feature_boxfusion_sec` bucket into:
- `association_boxfusion_sec`
- `snapshot_export_sec`

Right now the label obscures the true root cause.

## 7. Minimal-change optimization roadmap

### Level 0: pure diagnosis / logging only

- Add the timers and cardinality logs listed above.
- Log `per_frame_ins` size explicitly.
- Log how often segmentation refresh frames trigger duplicate `get_vector_map_data(...)` calls.
- Log active-floor merged point counts and grid size every segmentation run.

### Level 1: local fixes with minimal behavior change

1. Stop double rebuilding the vector map on segmentation frames.
- Reuse the `vector_map` already produced in stage 3 instead of recomputing it again in `maybe_capture_demo_snapshot(...)`.

2. Time and optionally decouple snapshot export from the BoxFusion bucket.
- Even if behavior stays the same, separate timing immediately clarifies the real bottleneck.

3. Avoid full historical scan where cheap gates exist.
- Before 3D IoU / 2D reprojection, prefilter object candidates by floor, z band, and coarse XY distance.
- This is the smallest path toward relevant-history fusion without redesigning the whole backend.

4. Avoid rescanning all objects in `BoxFusion.boxfusion(...)`.
- Maintain a queue of newly eligible fusion candidates instead of scanning all retained objects every keyframe.

5. Reduce export cost without changing backend semantics.
- Skip embedding `.tolist()` conversion unless the export truly needs serialized embeddings.
- Consider caching unchanged room / floor diagnostics between segmentation refreshes.

6. Avoid full re-voxelization of the retained floor cloud every segmentation run.
- Maintain an incremental downsampled floor map instead of `concatenate + voxel_down_sample` on the entire retained cloud each time.

### Level 2: architecture-level direction toward advisor’s desired design

1. Introduce explicit state separation.
- `LocalActiveState`: current frame, recent window, active room / active floor working memory
- `GlobalSemanticMemory`: finalized retained objects / rooms / floors
- `QueryableWorldGraph`: export/query layer built from retained memory

2. Make current-frame association candidate-based.
- candidate subset = same floor
- then same active room if known
- else nearby rooms / spatial neighborhood
- with short temporal fallback window

3. Move from fixed-interval segmentation to event-driven segmentation.
- trigger on room leaving / room completion / strong boundary-crossing signal
- keep interval mode only as fallback / watchdog

4. Make topology incremental at room granularity.
- finalize room polygons / gateways when a room is left or stabilized
- update only affected room-to-room edges
- keep `RoomTopologyBuilder` as offline validation / export, not the online graph maintainer

5. Keep floor scope, but add room scope inside each floor.
- current implementation already has a useful floor partition
- the next backend-first step is room-local working sets within each floor, not a full-system redesign

## Optional file/function shortlist

Most likely responsible for global-history semantic fusion growth:
- `demo.py:381-549`
- `boxfusion/instances.py:372-490`
- `boxfusion/box_fusion.py:626-728`
- `demo.py:135-161`
- `boxfusion/floor_aware_room_segmenter.py:282-407`
- `boxfusion/scene_graph_builder.py:669-698`
- `boxfusion/scene_graph_builder.py:888-1013`

Most likely responsible for globally accumulated room segmentation cost:
- `demo.py:306-340`
- `boxfusion/floor_aware_room_segmenter.py:202-280`
- `boxfusion/floor_aware_room_segmenter.py:420-438`
- `boxfusion/dynamic_room_segmenter.py:834-1106`
- `boxfusion/dynamic_room_segmenter.py:600-685`
- `boxfusion/dynamic_room_segmenter.py:710-793`
- `boxfusion/dynamic_room_segmenter.py:1108-1188`

Most likely responsible for repeated topology rebuild / export cost:
- `demo.py:135-161`
- `demo.py:326-340`
- `demo.py:541-547`
- `boxfusion/floor_aware_room_segmenter.py:282-407`
- `boxfusion/stage_a_demo.py:1223-1238`
- `boxfusion/room_topology.py:1232-1247`
