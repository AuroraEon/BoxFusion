# Floor-aware World Graph / Room Segmentation v0.1

## Contract

This upgrade keeps the existing RSLG layering intact:

- World Graph remains the entity source-of-truth layer.
- Room-centric Queryable Topology remains the derived routing/query layer.
- NetworkX remains the graph/search backend.
- Cross-floor routing is only exposed through explicit `vertical_transition` room-room edges.

The world model now has four explicit entity types:

- `Floor`
- `Room`
- `Object`
- `Anchor`

Hard constraints in v0.1:

- A normal room has exactly one `floor_id`.
- Room segmentation is conditioned on floor assignment first.
- Stable same-floor room relations remain `adjacent`, `transition`, and `possible_connection`.
- Cross-floor movement is modeled separately as `vertical_transition`.

## Insertion Point In The Current Pipeline

The floor-assignment stage is inserted in the online path before room segmentation updates:

1. `demo.py` captures the current pose and optional keyframe point chunk.
2. `FloorAwareRoomSegmenter.observe_frame(...)` assigns the frame to a floor hypothesis through `FloorManager`.
3. Only frames with `status == "stable"` are allowed to contribute point chunks to a floor's room-segmentation state.
4. `FloorAwareRoomSegmenter.perform_segmentation(...)` runs segmentation only on the currently stable floor state.
5. Export builds a combined floor-aware world graph and a room-centric topology-friendly view.

This keeps the system stateful and incremental. It does not rebuild the whole scene from scratch on every frame.

## Runtime Design

### Floor Assignment

`boxfusion/floor_manager.py` introduces a lightweight `FloorManager`.

It maintains runtime floor hypotheses as height bands with:

- `floor_id`
- `floor_index`
- `z_min`, `z_max`, `z_center`
- `confidence`
- `status`
- `support_statistics`

The current implementation is deliberately conservative:

- pose height drives the primary floor hypothesis
- nearby-band observations become `transition` frames instead of forcing room updates
- a new floor is created only after repeated height evidence beyond the current floor bands

### Floor-conditioned Room Segmentation

`boxfusion/floor_aware_room_segmenter.py` wraps one `DynamicRoomSegmenter` per floor.

This is the main structural fix.

- Each floor has its own point accumulation and room-identity state.
- Stable keyframe chunks are appended only to that floor's state.
- Room ids are local within a floor segmenter and remapped to globally unique world-room ids on export.
- Transition-like frames are recorded for debugging and vertical-link inference, but they do not stabilize ordinary room identity.

### Floor-aware World Export

The combined export now includes:

- `floors`
- `rooms` with `floor_id`, `floor_index`, `floor_assignment_confidence`, `room_local_id`
- `objects` with inspectable floor assignment
- `anchors` with inspectable floor assignment
- `frame_floor_assignments`
- `room_floor_validation`
- `vertical_transitions`

The scene/world graph also now creates explicit `FloorNode` entities and `ON_FLOOR` relations for rooms, objects, and anchors.

## Minimal Topology Extension

`boxfusion/room_topology.py` stays room-centric.

The extension is intentionally small:

- floors are preserved as top-level metadata in the derived topology export
- room nodes carry `floor_id`, `floor_index`, `floor_assignment_confidence`, and `status`
- same-floor evidence collection skips cross-floor room pairs
- explicit `vertical_transition` support is derived only from `world["vertical_transitions"]`
- validation now checks that same-floor relations do not cross floors and that `vertical_transition` does

`boxfusion/query_api.py` is extended only by allowing route policies to traverse `vertical_transition` edges.

## What Was Reused From HOV-SG

Concepts reused from the HOV-SG reference files:

- the ordering: floor segmentation first, then per-floor room segmentation, then higher-level graph construction
- the idea that room segmentation should operate on a floor-restricted spatial slice instead of one unconstrained global room partition
- the use of lightweight 2D room-region reasoning after a floor partition exists

## What Was Not Reused From HOV-SG

The following HOV-SG behaviors were intentionally not copied into the online path:

- global full-scene floor histogram analysis as the primary runtime mechanism
- per-floor batch reconstruction from the full accumulated map on every update
- offline room extraction that assumes complete floor point clouds and global KD-tree backprojection passes
- global room-view embedding recomputation as part of room segmentation

Those assumptions are useful offline, but they are too batch-oriented for the current stateful runtime path.

## Inspectable Checks

The export and tests now make the new assumptions easy to inspect:

- floor bands: `floors[*].z_min/z_max/z_center`
- frame assignment: `frame_floor_assignments`
- room ownership: `rooms[*].floor_id`
- invalid multi-floor rooms: `room_floor_validation`
- explicit cross-floor links: `vertical_transitions`
- derived topology count: `inspect_export()["vertical_transition_edge_count"]`

## How To Run The Checks

```bash
python3 boxfusion/test_floor_aware_world_graph.py
python3 boxfusion/test_room_topology.py
python3 boxfusion/test_scene_graph.py
```

## Current Limitations

v0.1 is structurally honest, not geometrically ambitious.

- Floor assignment is height-band based and does not reconstruct full stair geometry.
- Vertical transitions are inferred from stable floor changes plus room lookup, not from a dedicated stair detector.
- The online room segmenter still uses the existing 2D room pipeline within each floor partition.
- There is no low-frequency offline rebuild path yet for retrospective cleanup.

## Reasonable Next Step

The next reasonable step is a small transition-aware floor module refinement:

- accumulate explicit stair / landing candidates during `transition` intervals
- attach those candidates to `vertical_transition` records as support geometry
- keep the same room-centric topology API, but expose richer vertical-edge metadata for later routing and demo explanations
