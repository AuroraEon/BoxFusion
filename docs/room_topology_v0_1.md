# Room-Centric Queryable Topology v0.1

## What It Is

`Room-centric Queryable Topology v0.1` is a lightweight room-level topology layer derived from the existing BoxFusion world-model exports. It keeps rooms as the routing nodes, preserves object and anchor containment through indices, and attaches explicit evidence and confidence to room-room relations.

The topology layer is implemented in [`boxfusion/room_topology.py`](/home/aurora/workspace1/BoxFusion/boxfusion/room_topology.py) and uses `networkx.MultiDiGraph`.

## What It Is Not

- Not a live SLAM backend
- Not a navmesh or motion planner
- Not a geometric path executor
- Not a mandatory door-detector pipeline
- Not a replacement for the existing world graph

The heterogeneous scene/world graph remains the entity source of truth. The topology layer is a derived representation and query layer on top of existing room/object/anchor outputs plus lightweight room-transition history.

## What It Builds

- Room nodes
- Room-room relations: `adjacent`, `transition`, `possible_connection`
- Containment indices:
  - `object_to_room`
  - `anchor_to_room`
  - `room_to_objects`
  - `room_to_anchors`
- Evidence registry with:
  - `trajectory_transition`
  - `boundary_contact`
  - `repeated_crossing`
  - `shared_frontier`
  - `door_detection` when existing gateway output supports it

## How Confidence Works

Edge confidence uses a simple weighted sum of evidence scores, clamped to `[0, 1]`.

Default weights:

- `trajectory_transition`: `0.45`
- `boundary_contact`: `0.25`
- `repeated_crossing`: `0.20`
- `shared_frontier`: `0.15`
- `door_detection`: `0.60`

Default status labels:

- `weak`: `0.00` to `0.34`
- `supported`: `0.35` to `0.64`
- `confirmed`: `0.65` to `1.00`

## How It Is Built

1. Sync valid room nodes from the exported world map.
2. Build object/anchor containment indices from existing room assignments.
3. Derive adjacency evidence from room gateways and lightweight room-polygon proximity.
4. Derive transition evidence from debounced room-id history.
5. Promote weak links to `possible_connection` when stronger support is missing.
6. Export JSON, optional GraphML, and a small query report.

## Graph Search API

Main API on `RoomTopology`:

- `get_room(room_id)`
- `get_room_neighbors(room_id, relation_types=None, min_conf=0.0)`
- `get_room_of_object(object_id)`
- `get_room_of_anchor(anchor_id)`
- `get_room_contents(room_id)`
- `explain_connection(room_a, room_b)`
- `find_room_path(start_room_id, goal_room_id, allowed_relations=None, min_conf=0.0, method="shortest")`
- `find_candidate_room_paths(start_room_id, goal_room_id, ..., max_paths=3)`
- `explain_room_path(start_room_id, goal_room_id, ...)`
- `find_room_route(src_room, dst_room, allowed_relations=None, min_conf=0.0)`
- `summarize_room(room_id)`

Routing is room-level only. It prefers `transition` over `adjacent` over `possible_connection` by assigning lower cost to stronger relation types and adding only a small confidence penalty. Returned paths are abstract room routes only: room sequence, relation types, per-hop confidence/cost, and a failure reason when no route is available.

What the graph search does:

- Searches the existing room-centric topology directly with structured room ids
- Returns an abstract room-to-room route that downstream query or tool layers can consume
- Supports relation filtering and minimum-confidence filtering

What the graph search does not do:

- It does not parse natural language
- It does not produce executable geometric trajectories
- It does not replace motion planning or robot control

## Minimal Query API v0.1

`Minimal Query API v0.1` is the bridge layer above the room-topology graph search. It accepts already-structured requests, resolves target entities into destination rooms, and returns standardized query results for downstream tool-calling, demos, or later planner adapters.

Implementation files:

- [`boxfusion/query_api.py`](/home/aurora/workspace1/BoxFusion/boxfusion/query_api.py)
- [`stage_a_topology_query.py`](/home/aurora/workspace1/BoxFusion/stage_a_topology_query.py)

Supported structured calls:

- `query_route(start_room_id, goal_room_id, route_policy="balanced")`
- `query_route_to_anchor(start_room_id, anchor_id, route_policy="balanced")`
- `query_route_to_object(start_room_id, object_id=None, object_label=None, route_policy="balanced")`
- `resolve_room_target(goal_room_id)`
- `resolve_anchor_room(anchor_id)`
- `resolve_object_room(object_id=None, object_label=None)`
- `explain_route(query_result)`
- `summarize_target_resolution(...)`

What this bridge layer does:

- Resolves room, anchor, and object targets into graph-searchable room ids
- Reuses the existing `find_room_path(...)` and `find_candidate_room_paths(...)` internals
- Returns explicit success/failure objects instead of hiding expected query outcomes behind exceptions
- Exposes inspectable policy presets and target-resolution details

What it does not do:

- It does not parse free-form language
- It does not call an LLM
- It does not replace the topology graph search
- It does not produce executable robot trajectories

### Target Resolution Rules

- Room target: uses `goal_room_id` directly after canonicalization and topology membership checks.
- Anchor target: resolves `anchor_id -> anchor_to_room`; returns `anchor_not_found` or `anchor_has_no_resolved_room` when needed.
- Object target by id: resolves `object_id -> object_to_room`; returns `object_not_found` or `object_has_no_resolved_room` when needed.
- Object target by label: uses lightweight exact / normalized label matching from exported object metadata. If multiple matches land in different rooms, the API returns `object_label_ambiguous` plus ranked candidates.

The topology JSON now carries lightweight `entities.objects` and `entities.anchors` metadata so object-label resolution remains available after export / reload.

## Export Refresh And Acceptance

This step exists to validate the current topology + Query API stack on real exported data before moving on to grounding or natural-language-facing work.

Why this matters:

- older `topology_v0_1.json` exports may still contain room/object/anchor indices, but miss the inspectable object or anchor metadata needed for label-based inspection after reload
- the refresh step regenerates topology JSON from the current Stage A world export
- the acceptance step confirms that anchor, object-id, and object-label queries work on at least one real target when the export actually contains those targets

Recommended workflow:

1. Refresh the topology export from a Stage A sequence directory.
2. Inspect the resulting topology JSON for rooms, anchors, objects, and labels.
3. Run the acceptance check before starting grounding work.

Refresh from an existing Stage A sequence directory:

```bash
python stage_a_topology_export.py \
  --sequence-dir ./stage_a_outputs/42898867
```

Inspect only:

```bash
python stage_a_topology_acceptance.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --list all
```

Run acceptance and save a structured report:

```bash
python stage_a_topology_acceptance.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --acceptance \
  --report-json-out ./stage_a_outputs/42898867/logs/topology_acceptance_report.json
```

Optional targeted checks:

```bash
python stage_a_topology_acceptance.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --acceptance \
  --start room_10 \
  --anchor-id anchor_obj_13 \
  --object-id obj_13 \
  --object-label bathtub \
  --compare-policies
```

The acceptance report includes:

- topology file path
- room / anchor / object / object-label counts
- sample valid room ids, anchor ids, object ids, and object labels
- whether route-to-anchor succeeded on at least one real exported anchor
- whether route-to-object-id succeeded on at least one real exported object
- whether route-to-object-label succeeded on at least one real exported label
- explicit failure reasons and notes when a capability is unavailable

Interpret failures literally:

- `object label lookup unavailable`: the export has no inspectable object labels after reload; refresh the export or inspect the upstream world-map metadata
- `object metadata missing`: the export only preserved containment indices, not richer object metadata
- `anchor metadata missing`: the export only preserved anchor-room indices, not richer anchor metadata
- `no anchors present in this export`: there are no anchors to validate against
- `no routable target found`: real targets exist, but none produced a successful route under the attempted starts/policies

### Route Policies

- `strict`: allows `transition` and `adjacent`, requires stronger confidence, and rejects weak fallback edges.
- `balanced`: default preset; allows `transition`, `adjacent`, and `possible_connection`.
- `exploratory`: keeps weak fallback edges and also surfaces candidate route alternatives for inspection.

### Result Shape

Query calls return a standardized dictionary with:

- `success` / `found`
- `query_type`
- `query`
- `start_room_id`
- `resolved_goal_room_id`
- `target_resolution`
- `route`
- `explanation`
- `failure_reason`
- `metadata`

`target_resolution` separates entity lookup from path search and includes:

- `target_type`
- `resolved_room_id`
- `matched_entity_ids`
- `candidate_matches`
- `ambiguity`
- `failure_reason`

`route` reuses the existing room-path payload style and adds:

- `attempted`
- `route_policy`
- `policy_settings`
- `room_sequence`
- `used_relation_types`
- `edges`
- `total_cost`
- `route_confidence`

## How To Run

Fresh Stage A export:

```bash
python stage_a_demo.py CA1M \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/ca1m.yaml \
  --device cuda \
  --seq 42898867 \
  --output-root ./stage_a_outputs \
  --room-seg-interval 100
```

This now writes:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/topology_v0_1.graphml`

Post-hoc rebuild from an existing Stage A run:

```bash
python stage_a_topology_export.py \
  --sequence-dir ./stage_a_outputs/42898867
```

Route search from an exported topology JSON:

```bash
python stage_a_topology_route.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --start room_1 \
  --goal room_3
```

Structured query examples from an exported topology JSON:

```bash
python stage_a_topology_query.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --start room_1 \
  --goal-room room_3 \
  --route-policy balanced
```

```bash
python stage_a_topology_query.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --start room_1 \
  --anchor-id anchor_obj_201
```

```bash
python stage_a_topology_query.py \
  --topology-json ./stage_a_outputs/42898867/logs/topology_v0_1.json \
  --start room_3 \
  --object-label sofa \
  --route-policy exploratory
```

## Validation

Run the lightweight validation script:

```bash
python boxfusion/test_room_topology.py
```

That validation now covers:

- fresh export / reload preserving inspectable object and anchor metadata
- acceptance on real exported targets from a fresh JSON export
- graceful behavior on older exports with missing `entities.*` metadata

## Current Limitations

- Transition evidence depends on exported room-id history, not a live online SLAM state.
- Adjacency still uses lightweight gateway/proximity cues rather than a full traversability backend.
- Room geometry remains stepwise because it follows segmentation refresh cycles.
- `possible_connection` is intentionally conservative and explainable, not a learned connectivity predictor.
- Object-label lookup only uses lightweight exact / normalized label matching; it is not a semantic retrieval system.
- Older topology JSON exports without `entities.objects` metadata can still do room / anchor / object-id queries, but object-label lookup may be unavailable.
- The Query API is intentionally structured-only in v0.1; NL parsing and LLM tool-calling are deferred to a later layer.
