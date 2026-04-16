# Runtime Sidecar Minimal Public Topology Shadow Target

This step adds the first real-export shadow parity target without replacing the
current synchronous exporter.

The authoritative committed/public topology remains `logs/topology_v0_1.json`.
The sidecar materialization is shadow-only, diagnostic-only, and explicitly
non-authoritative.

## Chosen Minimal Subset

The sidecar now materializes `minimal_public_topology_subset` from the runtime
snapshot.

Included fields:

- schema identity:
  - `artifact_kind`
  - `scope`
  - `public_topology_meaning: committed_published_only`
  - `authoritative: false`
- public counts:
  - floor count
  - room count
  - undirected edge count
  - object-to-room membership count
- stable identifiers:
  - floor IDs
  - room IDs
  - object IDs with public room membership
- minimal records:
  - floors: `floor_id`, `display_floor_id`, `display_order`
  - rooms: `id`, `floor_id`, `display_floor_id`, `display_order`, `room_type`, `status`
  - edges: `source`, `target`, `relation_type`, `status`
  - object-room memberships: `object_id`, `room_id`, `floor_id`
- semantic summary:
  - edge relation-type counts
  - whether object-room membership is available

This is intentionally more than counts/IDs-only parity because it validates a
small topology-shaped artifact: room records, edge endpoint/relation semantics,
and object-room membership compatibility.

## Why This Target

This subset is the smallest useful committed/public topology export target that
can be compared meaningfully against the existing exporter.

It was selected because:

- it is already derivable from the current public topology JSON
- it exercises real public topology structure, not only counters
- it avoids high-risk or richer surfaces such as anchors, GraphML, scene graph
  relationships, evidence payloads, route diagnostics, and visual artifacts
- it keeps public topology scoped to committed/published rooms
- it remains easy to remove or revise if broader sidecar migration changes shape

## Snapshot Additions

`RuntimeStateSnapshot` now carries one additional optional field:

- `minimal_public_topology_subset`

The field is populated from:

- authoritative manifest-backed topology payloads in the coordinator path
- runtime vector-map-shaped state in direct shadow validation, filtered by
  committed room IDs when lifecycle data is available

No maintained candidate indices were added. The snapshot still avoids full
vector-map, anchor, scene-graph, GraphML, and rich diagnostic materialization.

## Parity Validation

`compare_minimal_public_topology_shadow_parity(...)` compares the sidecar subset
against the current authoritative topology export.

It checks:

- required schema keys are present
- public topology meaning is unchanged
- floor/room/edge/object-membership counts agree
- stable floor/room/object IDs agree
- minimal floor, room, edge, and object-room membership records agree
- edge relation-type summary agrees

All mismatches are diagnostic-only. They do not change the consumer-facing
public topology contract and do not make the sidecar a source of truth.

## Still Deferred

This step does not migrate:

- anchors
- scene graph relationships
- GraphML
- rich diagnostics or evidence formatting
- full vector-map export details
- visual/debug artifacts
- online publication redesign
- broad ROS API changes
- maintained candidate indices
- detector, CLIP, multi-GPU, or runtime optimization work

The next broader migration question remains hypothetical until more committed
public export subsets can be shadow-materialized and compared with the same
diagnostic-only discipline.
