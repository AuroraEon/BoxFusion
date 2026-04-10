# Online Topology First-Step Design Note

## Scope

This note is intentionally limited to design and inspection.

- It does not refactor runtime code.
- It does not change the public Query API semantics.
- It does not make topology the truth owner.
- It does not reopen broad runtime optimization.

The current repo already has a strong retained-scene backend shape:

- the floor-aware world export is produced in `FloorAwareRoomSegmenter.get_vector_map_data(...)`
- the room-centric topology is derived afterward by `RoomTopologyBuilder`
- the Query API consumes the derived topology
- the symbolic executor consumes Query API outputs, not raw world state

That existing layering should remain the base contract.

## What "Online Topology" Should Mean Here

In this project, "online topology" should not mean "topology becomes the live world state."

It should mean:

- the World Graph / vector-map export path continues to own entity truth for floors, rooms, objects, anchors, gateways, and vertical-transition evidence
- topology gains a lightweight incremental staging layer that can notice room-local changes during runtime
- topology may maintain a working, mutable, not-yet-public view built from current world evidence
- topology only publishes committed room-topology snapshots to the existing Query API by default

So the online aspect is:

- online detection of "this room may need topology refresh"
- online assembly of room-local topology candidates
- conservative snapshot commit of derived topology

It is not:

- a second truth database
- a replacement for room segmentation or world export
- a planner/runtime-control layer
- a requirement to expose unstable topology directly to queries

## Current Repo Contract

### 1. World Graph / retained world export is the truth layer

The closest concrete truth-owner path in the repo today is the floor-aware export builder in `boxfusion/floor_aware_room_segmenter.py`.

- `get_vector_map_data(...)` emits `floors`, `rooms`, `gateways`, `vertical_transitions`, `objects`, `anchors`, and `relationships`, plus floor/segmentation diagnostics and cached export state.
- room/object/anchor export already has room-local reuse and delta-oriented export machinery, but that is export optimization, not a public online-topology contract.
- the file already reports that topology is not incrementally online yet and explicitly lists the missing trigger structures.

Relevant anchors:

- `boxfusion/floor_aware_room_segmenter.py:304`
- `boxfusion/floor_aware_room_segmenter.py:380`
- `boxfusion/floor_aware_room_segmenter.py:403`
- `boxfusion/floor_aware_room_segmenter.py:443`
- `boxfusion/floor_aware_room_segmenter.py:1291`

The scene/world graph entity schemas remain in `boxfusion/scene_graph_builder.py`:

- `FloorNode`
- `RoomNode`
- `ObjectNode`
- `AnchorNode`
- `SemanticSceneGraph`

Relevant anchors:

- `boxfusion/scene_graph_builder.py:273`
- `boxfusion/scene_graph_builder.py:300`
- `boxfusion/scene_graph_builder.py:333`
- `boxfusion/scene_graph_builder.py:365`
- `boxfusion/scene_graph_builder.py:563`

### 2. Floor / room / gateway / vertical-transition evidence already exists upstream

Floor evidence is stateful and conservative in `boxfusion/floor_manager.py`:

- stable vs transition floor observations
- tentative new-floor accumulation
- floor support statistics and events

Relevant anchors:

- `boxfusion/floor_manager.py:27`
- `boxfusion/floor_manager.py:53`
- `boxfusion/floor_manager.py:75`
- `boxfusion/floor_manager.py:147`
- `boxfusion/floor_manager.py:160`

Room, gateway, and vertical-transition exports are assembled in `boxfusion/floor_aware_room_segmenter.py`:

- `_export_rooms_and_gateways(...)` maps per-floor segmenter state into exported rooms/gateways
- `_build_floor_debug_summary(...)` already aggregates per-floor support, segmentation runs, fallback counts, pending chunks, and exported counts
- `_build_vertical_transitions(...)` already emits explicit cross-floor evidence with partial vs edge-eligible status

Relevant anchors:

- `boxfusion/floor_aware_room_segmenter.py:1444`
- `boxfusion/floor_aware_room_segmenter.py:1480`
- `boxfusion/floor_aware_room_segmenter.py:1671`
- `boxfusion/floor_aware_room_segmenter.py:2602`

### 3. Room-centric topology is already a derived layer

`boxfusion/room_topology.py` already encodes the correct ownership direction:

- `RoomTopologyBuilder.build(...)` takes an exported world map plus transition history
- rooms are synced from world export
- object/anchor containment is synced from world export
- edge support is derived from gateways, polygon proximity, room-transition history, and explicit vertical-transition exports
- `possible_connection` is explicitly a fallback derived relation

Relevant anchors:

- `boxfusion/room_topology.py:1225`
- `boxfusion/room_topology.py:1305`
- `boxfusion/room_topology.py:1342`
- `boxfusion/room_topology.py:1435`
- `boxfusion/room_topology.py:1448`
- `boxfusion/room_topology.py:1574`
- `boxfusion/room_topology.py:1627`
- `boxfusion/room_topology.py:1716`

### 4. Query API semantics are already stable and topology-facing

`boxfusion/query_api.py` exposes the current public contract:

- `resolve_room_target(...)`
- `resolve_anchor_room(...)`
- `resolve_object_room(...)`
- `query_route(...)`
- `query_route_to_anchor(...)`
- `query_route_to_object(...)`

The Query API assumes it is reading a topology snapshot, not participating in topology construction.

Relevant anchors:

- `boxfusion/query_api.py:54`
- `boxfusion/query_api.py:67`
- `boxfusion/query_api.py:115`
- `boxfusion/query_api.py:137`
- `boxfusion/query_api.py:156`
- `boxfusion/query_api.py:175`

### 5. Symbolic executor is downstream-only

`boxfusion/vln_closed_loop.py` confirms the intended system boundary:

- it resolves a structured task through the Query API
- it converts route edges into symbolic steps
- it validates execution against replay observations
- it does not own topology or room truth

Relevant anchors:

- `boxfusion/vln_closed_loop.py:146`
- `boxfusion/vln_closed_loop.py:172`
- `boxfusion/vln_closed_loop.py:306`
- `boxfusion/vln_closed_loop.py:334`

## Working Topology vs Committed Topology

### Working topology

Add only a design-level concept first:

- a private, mutable, room-scoped derived state
- fed by already-exported truth signals
- allowed to contain provisional room-room relations and provisional containment summaries
- not consumed by the default Query API

Recommended contents:

- per-room dirty flag
- last world-export version or room signature seen
- pending room-local adjacency/transition candidates
- candidate room-to-object and room-to-anchor stability counters
- candidate commit reasons and blocked commit reasons

### Committed topology snapshot

This should remain the current exported/public topology contract:

- a stable `RoomTopology` instance or JSON payload
- built from committed world-export facts
- consumed by `RoomTopologyQueryAPI` by default
- exported through the existing `topology_v0_1.json` path

Relevant current snapshot/export path:

- `boxfusion/stage_a_demo.py:1208`
- `boxfusion/stage_a_demo.py:1222`
- `stage_a_topology_export.py:7`
- `stage_a_topology_export.py:16`

### Proposed contract between the two

- World Graph truth updates may mark rooms or room-pairs dirty.
- Working topology may refresh room-local derived state opportunistically.
- Only commit gates may publish a new committed topology snapshot.
- Query API keeps reading committed topology unless an explicit non-default experimental path is added later.

## Provisional vs Stable Semantics

The repo already has a useful precedent:

- floor assignment has tentative/confirmed support
- vertical transitions have partial association vs edge-eligible support
- topology edges have weak/supported/confirmed confidence labels

Online topology should extend that pattern instead of inventing a binary "done" flag.

Recommended semantics:

- provisional semantics:
  - room membership still moving
  - gateway/transition evidence not yet stable
  - object or anchor containment still changing over recent refreshes
  - vertical-transition association incomplete or recently changed
- stable semantics:
  - room boundary/membership change has reduced across recent refreshes
  - object/anchor room assignment is no longer oscillating materially
  - same-floor adjacency/transition evidence is repeatable
  - cross-floor evidence is edge-eligible and not contradicted by later room assignment

Default Query API behavior should stay:

- committed snapshots only
- no silent consumption of provisional working state

## Recommended Room Lifecycle

Use room lifecycle states instead of a single `room_done` flag.

Recommended states:

- `discovering`
  - room first appears or is first split from unassigned space
  - boundaries and membership are still actively changing
- `active`
  - room is currently being observed or recently updated
  - dirty topology refreshes are allowed but not commit-ready by default
- `candidate_complete`
  - multiple signals suggest the room may be stable enough to evaluate for commit
  - not yet public
- `committed`
  - room semantics are stable enough for the next committed topology snapshot
  - Query API may consume this snapshot
- `revisitable`
  - previously committed room is being observed again
  - room remains committed unless strong evidence reopens it
- `merge_or_split_pending`
  - recent segmentation deltas suggest room identity or boundaries may need restructuring
  - block commit for affected local region until stabilized

Recommended lifecycle transitions:

- `discovering -> active`
  - first usable room polygon plus at least one stable-room assignment interval
- `active -> candidate_complete`
  - dirty signals quiet down and readiness signals accumulate
- `candidate_complete -> committed`
  - commit gates pass
- `committed -> revisitable`
  - revisit / re-entry without major structure change
- `revisitable -> active`
  - new evidence materially changes local room state
- `active|candidate_complete|committed -> merge_or_split_pending`
  - room signature, gateway structure, or membership changes sharply

## Conservative Room-Level Trigger Design

This repo should treat room completion as conservative multi-signal readiness, not a detector.

### Cheap triggers

These should only mark dirty or suggest refresh. They should not commit topology alone.

- segmentation refresh happened for the active floor
  - grounded in `room_segmentation_diagnostics` and per-floor segmentation runs
- current room changed in snapshot/replay tracking
  - grounded in `current_room_id` already written into timeline/snapshots
- doorway/gateway crossing implied by stable room switch or gateway-linked boundary contact
- floor transition observation or floor-status transition
- explicit vertical-transition evidence update
- object/anchor room assignment changed in a room-local delta export
- room polygon signature changed
- revisit into a previously departed room

Useful existing signals:

- `boxfusion/stage_a_demo.py:540`
- `boxfusion/stage_a_demo.py:572`
- `boxfusion/stage_a_demo.py:756`
- `boxfusion/floor_aware_room_segmenter.py:1444`
- `boxfusion/room_topology.py:1574`
- `boxfusion/room_topology.py:1627`

### Candidate-complete checks

These should decide whether a room is ready to be evaluated for commit.

- segmentation refresh completed and the room still exists with the same room id / local region mapping
- agent appears to have left the room, or the room is no longer the active room for a minimum dwell period
- gateway structure for the room is unchanged across recent refreshes
- floor assignment for the room is stable and not in transition
- vertical-transition evidence touching the room is either unchanged or explicitly partial and therefore commit-blocked
- object-to-room and anchor-to-room assignments for the room have stabilized across recent refreshes
- boundary/membership change has reduced over the last N refresh opportunities
- current-room frontier is reduced or exhausted
  - in this repo, this can start as a heuristic from repeated lack of changed-room rebuilds plus stable gateway/room signatures rather than a new full frontier subsystem

Recommended first-pass candidate-complete rule:

- require at least one leave-like signal or refresh separation
- require no recent merge/split pending flag
- require room signature stability across at least 2 consecutive refresh opportunities
- require object/anchor containment deltas below a conservative threshold

### Commit gates

These decide whether a working-topology update may become the next committed snapshot.

- world truth for affected rooms has been exported successfully
- affected rooms are not in `merge_or_split_pending`
- no floor mismatch or room-floor validation failure
- same-floor relations do not cross floors
- vertical-transition relations remain cross-floor and edge-eligible
- containment indices for affected rooms are internally consistent
- the candidate snapshot passes the same `RoomTopology.validate()` invariants as the current export path
- commit is local when possible
  - only affected rooms / neighboring relations should need re-materialization in v0.1 design

Practical v0.1 gate:

- build a candidate `RoomTopology` from the latest committed world export
- compare only affected room ids / edge neighborhoods
- replace committed snapshot only if validation passes and the affected subgraph is coherent

## Smallest Realistic "Online Topology v0.1" Milestone

The smallest realistic milestone for this repo is not a fully online topology builder.

It is:

1. Add a design-level room lifecycle state holder and dirty-room tracker beside the existing export/runtime instrumentation path.
2. Feed it only existing signals:
   - segmentation refresh
   - current-room change
   - floor transition
   - vertical-transition updates
   - object/anchor room-local delta changes
3. Maintain a private working-topology status record per room.
4. Keep Query API on committed topology only.
5. Reuse the existing full `RoomTopologyBuilder.build(...)` for commit-time snapshot materialization.
6. Add instrumentation/reporting that says:
   - which rooms are dirty
   - which rooms are candidate-complete
   - which commit gates blocked publication

This milestone is intentionally compatible with the frozen runtime baseline because:

- it does not change how world export is produced
- it does not require live partial Query API reads
- it does not force a new planner or execution path
- it can initially publish only end-of-run or low-frequency committed snapshots while still exercising the lifecycle/trigger design

## What Must Not Change

- World Graph / world export remains the truth owner.
- Room-centric topology remains a derived layer.
- Query API public semantics stay stable by default.
- Symbolic executor remains downstream-only.
- No planner/agent broadening.
- No assumption of a perfect room completion detector.
- No forced full online implementation in this step.

## Repo Module Map

### World Graph construction / truth state

- `boxfusion/scene_graph_builder.py`
  - canonical entity schemas and graph relations for floors, rooms, objects, anchors
- `boxfusion/floor_aware_room_segmenter.py`
  - actual retained world export assembly; closest thing to current truth export owner
- `boxfusion/floor_manager.py`
  - floor hypothesis state, stable/transition observations, new-floor confirmation

### Room / floor / anchor / gateway / vertical-transition evidence

- `boxfusion/floor_aware_room_segmenter.py`
  - room export, gateway export, vertical-transition inference, diagnostics, export caching
- `boxfusion/floor_artifacts.py`
  - floor canonicalization, display ordering, vertical-transition canonicalization
- `boxfusion/stage_a_demo.py`
  - snapshot/timeline export, current-room tracking, revisit signals, final topology export wiring

### Room-centric topology build / export

- `boxfusion/room_topology.py`
  - topology graph, evidence registry, validation, JSON/GraphML export, builder logic
- `stage_a_topology_export.py`
  - rebuild topology from an existing Stage-A export directory

### Query API

- `boxfusion/query_api.py`
  - stable structured resolution/routing interface over topology snapshots
- `stage_a_topology_query.py`
  - CLI wrapper over the Query API

### Symbolic executor

- `boxfusion/vln_closed_loop.py`
  - converts Query API results into symbolic steps and validates them against replay

### Snapshot / export paths already used by topology

- `boxfusion/stage_a_demo.py`
  - writes `timeline.json`, `summary.json`, `topology_v0_1.json`, `topology_query_report.json`, `vertical_transition_evidence.json`, and floor diagnostics
- `stage_a_multifloor_query_acceptance.py`
  - already checks consistency between final vector-map export, topology export, and floor diagnostics
- `boxfusion/test_room_topology.py`
  - already protects current topology/query semantics and JSON reload behavior
- `boxfusion/runtime_instrumentation.py`
  - already has placeholder fields for online-topology status/triggers

## Risks / Uncertainty

- The repo has good signals for floor transitions, segmentation runs, room-local export deltas, and current-room changes, but it does not yet have a first-class room lifecycle manager.
- "Frontier exhaustion" is not currently a direct exported primitive; v0.1 should treat it as optional heuristic evidence, not a hard dependency.
- Room identity stability is still tied to the current segmentation/export cadence; if cadence is sparse, candidate-complete timing may be conservative.
- Merge/split handling is the main structural risk. A local lifecycle layer should explicitly block commit when room signatures change sharply rather than trying to be clever.
- The current topology builder is snapshot-oriented. For v0.1, it is safer to keep incremental logic in the trigger/lifecycle layer and continue using snapshot rebuild for committed publication.

## Touched Files

- `docs/online_topology_first_step_design_note.md`
  - added this inspection/design note only

