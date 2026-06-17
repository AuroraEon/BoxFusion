# RSLG-SLAM Pipeline Skeleton

Task25g created the initial canonical `tools/rslg_pipeline/` namespace for
future RSLG-SLAM pipeline tools. Task25h adds the first real static validators
for the RSLG-SLAM docs/manifests/project contract. Task25i adds the first
manifest-aware dry-run wrapper for route-contract planning. Task25j extends
that wrapper to generate small route contract stubs as task evidence. BoxFusion
remains only the historical repository path.

Task25m created a lightweight Chinese AI handoff context pack for continuing RSLG-SLAM in a new conversation.

No real business logic was migrated in task25g. The new modules do not call old
scripts, rerun the World Model Layer, launch runtime systems, or write large
artifacts.

## Modules

| Module | Current behavior | Intended future role |
| --- | --- | --- |
| `tools/rslg_pipeline/common.py` | Lightweight repo, JSON, manifest, path, and stat helpers. | Shared utilities for canonical pipeline tools. |
| `tools/rslg_pipeline/artifact_registry.py` | Reads the manifest index, resolves manifest paths, and summarizes references. | Shared artifact and manifest registry. |
| `tools/rslg_pipeline/validate_artifacts.py` | Runs real static JSON, manifest-index, project-truth, layer-naming, stable-map, milestone, retention, skeleton, no-rerun, and no-runtime checks without requiring historical `stage_outputs/`. | Canonical static validator entrypoint. |
| `tools/rslg_pipeline/build_world_model.py` | Placeholder CLI only. | Future Layer 1: World Model Layer entrypoint; currently not a replacement for `stage_a_demo.py`. |
| `tools/rslg_pipeline/build_stable_maps.py` | Layer 2 stable occupancy map dry-run planner and metadata-only candidate package builder. | Future stable occupancy map builder from World Model Layer BEV/free-space/wall/gateway evidence. |
| `tools/rslg_pipeline/build_vertical_connectors.py` | Manifest-aware Layer 2 dry-run planner for vertical connector, connector graph, and cross-floor topology artifacts. | Future vertical connector builder from pose-height transition and/or semantic stair object evidence. |
| `tools/rslg_pipeline/build_object_interfaces.py` | Placeholder CLI only. | Future object query and approach candidate interface builder. |
| `tools/rslg_pipeline/build_route_contracts.py` | Manifest-aware dry-run planner and small route-contract stub generator; it reads docs/manifests and validated milestones without generating real routes. | Future room-level and object-level route contract builder. |
| `tools/rslg_pipeline/stable_map_schema.py` | Schema validator for stable map dry-run reports and metadata-only candidate package artifacts. | Future stable map package schema and claim-boundary validation utilities. |
| `tools/rslg_pipeline/route_contract_schema.py` | Lightweight schema validator for route contract stubs. | Future route contract schema and claim-boundary validation utilities. |
| `tools/rslg_pipeline/route_contract_promotion.py` | Dry-run promotion reviewer for route contract stubs. | Future controlled promotion checks before candidate route contract finalization. |
| `tools/rslg_pipeline/candidate_route_contract_schema.py` | Dry-run schema validator for candidate contract previews emitted by route-contract promotion reports. | Future candidate route contract schema validation before finalization. |
| `tools/rslg_pipeline/layer3_boundary_review.py` | Review-only Layer 3 boundary utility for route-contract dry-run status, route-kind dependency scoping, and Layer 2 handoff recommendation. | Boundary review before returning to Layer 2 formal artifact regeneration. |
| `tools/rslg_pipeline/export_runtime_inputs.py` | Placeholder CLI only. | Future runtime input exporter for validation inputs, not a runtime launcher. |

The static validators do not require `stage_outputs/` or historical generated
outputs to exist. Missing protected generated-output paths are warning-only
signals during this skeleton task.

## task25i first real dry-run wrapper

`tools/rslg_pipeline/build_route_contracts.py` now supports manifest-aware
dry-run planning for route-contract wrappers. It loads the manifest registry,
checks the RSLG-SLAM project and layer guardrails in-process, reads validated
milestone truth when a route kind requires it, and writes a small JSON plan for
future Layer 3 route-contract outputs.

This wrapper does not generate route contracts, call old route-generation
scripts, migrate historical business logic, require historical `stage_outputs/`,
rerun the World Model Layer, or launch Gazebo/RViz/Nav2/AMCL/runtime demos.

## task25j first generated route contract stubs

`tools/rslg_pipeline/build_route_contracts.py` can now generate small
`rslg_route_contract_stub` JSON artifacts for `cross_floor_room` and
`cross_floor_object` task evidence after the manifest-aware dry-run plan is
ready. These stubs are explicitly marked with `is_stub: true` and
`is_final_navigation_artifact: false`.

The stubs use docs/manifests and validated milestone truth only. They do not use
old `stage_outputs`, write into `clean_rerun`, generate A* routes, generate
stable occupancy maps, generate object approach geometry, call old scripts, or
launch runtime systems.

## task25k route contract stub schema validator

`tools/rslg_pipeline/route_contract_schema.py` validates route contract stubs
generated by `tools/rslg_pipeline/build_route_contracts.py`. It checks stub
identity, safety flags, transition truth, object truth, future-artifact path
boundaries, output-layer metadata, and forbidden positive claims.

The validator does not require historical `stage_outputs/`, generate routes,
generate stable maps, call old scripts, rerun the World Model Layer, or launch
runtime systems.

## task25l route contract stub to candidate promotion dry-run

`tools/rslg_pipeline/route_contract_promotion.py` evaluates validated
`cross_floor_room` and `cross_floor_object` stubs for future candidate contract
promotion. It generates promotion dry-run reports only, never final route
contracts, and does not require historical `stage_outputs/`.

The dry-run report separates fields that can be safely promoted from the
validated stub, fields blocked until real Layer 2 artifacts exist, and fields
that must remain claim-boundary-protected until real route generation happens.

## task25n candidate route contract schema dry-run

`tools/rslg_pipeline/candidate_route_contract_schema.py` validates candidate
contract previews from route-contract promotion dry-run reports. It checks the
future candidate contract identity shape, route truth, object truth, claim
boundary, and blocked Layer 2 dependencies while staying preview-only.

The validator does not generate final candidate route contracts, does not
require historical `stage_outputs/`, does not generate A* routes, and does not
create runtime artifacts.

## task25o Layer 3 boundary review before Layer 2 regeneration

`tools/rslg_pipeline/layer3_boundary_review.py` reviews the Layer 3
route-contract dry-run chain after candidate schema validation. The review
confirms that dry-run planning, stub generation, stub schema validation,
promotion dry-run, and candidate schema dry-run are sufficient to stop
expanding Layer 3 for now unless a new schema issue is found.

Final route contracts remain blocked by missing Layer 2 formal artifacts:
stable occupancy maps, vertical connector artifacts, cross-floor topology, and
for object routes, object query and object approach artifacts. The next phase
should return to Layer 2 formal artifact regeneration rather than continuing to
add Layer 3 abstractions.

## task25p Layer 2 vertical connector and topology artifact dry-run

`tools/rslg_pipeline/build_vertical_connectors.py` now supports
manifest-aware Layer 2 dry-run planning for future vertical connector,
connector graph, and cross-floor topology formal artifacts. It reads current
RSLG-SLAM docs/manifests and validated milestone truth for `vt_1`, including
the validated transition edge boundary, then writes preview-only dry-run
reports.

The dry-run proposes future `vertical_connectors_v0_1.json`,
`stairs_or_vertical_connector_graph_v0_1.json`, and
`cross_floor_topology_v0_1.json` artifacts. It does not generate final formal
artifacts, does not require historical `stage_outputs/`, does not rerun the
World Model Layer or Stage-A, and does not launch runtime systems.

## task25q Layer 2 vertical connector artifact schema validator

`tools/rslg_pipeline/vertical_connector_schema.py` validates Layer 2 vertical
connector/topology dry-run reports from `build_vertical_connectors.py`. It
checks validated connector truth, Layer 2 metadata, future artifact previews,
blocked fields, downstream Layer 3 dependencies, and forbidden claim
boundaries.

The validator writes validation/dry-run reports only. It does not generate
final formal artifacts, does not require historical `stage_outputs/`, does not
call old scripts, does not rerun the World Model Layer, and does not launch
runtime systems.

## task26 Layer 2 vertical connector/topology candidate artifacts

`tools/rslg_pipeline/build_vertical_connectors.py` can now generate non-final
candidate Layer 2 artifacts for vertical connectors, the stairs/vertical
connector graph, and cross-floor topology. The candidate files are task
evidence only: they are marked as candidates, are not final formal artifacts,
and are generated from validated milestone truth rather than a current Layer 1
World Model Layer rerun.

The candidate mode does not regenerate real connector geometry or topology from
raw RGB-D/depth/pose data, does not require historical `stage_outputs`, does
not write into `clean_rerun`, and does not produce route planner-ready routes
or runtime inputs.

## task27 Layer 2 stable map artifact dry-run and candidate package

`tools/rslg_pipeline/build_stable_maps.py` now supports stable occupancy map
dry-run reports and metadata-only candidate package generation for Layer 2. The
candidate package is not a final stable map artifact and is explicitly blocked
until current World Model Layer evidence and current Layer 2 map metadata and
provenance exist.

The task27 stable map candidate does not generate PGM files, map_server YAML,
map pixels, runtime costmaps, routes, or runtime validation artifacts. The
stable occupancy map remains distinct from semantic floorplans, room masks,
runtime costmaps, external GT maps, and simulator navmeshes.

## task28 Layer 2 object interface artifact dry-run and candidate package

`tools/rslg_pipeline/build_object_interfaces.py` now supports object interface
dry-run reports and non-final candidate package generation for Layer 2. The
candidate package covers object query resolution and the object approach
candidate needed by later `cross_floor_object` route-contract finalization.

The task28 candidates are generated from validated milestone truth, not from a
current Layer 1 World Model Layer rerun, and they are not final formal
artifacts. No object-navigation runtime, route generation, executable
waypoints, approach geometry regeneration, or direct object-centroid navigation
is performed.

## task29 Layer 2 candidate artifact integration review and Layer 3 readiness

`tools/rslg_pipeline/layer2_candidate_integration_review.py` reviews the Layer
2 candidate artifacts from vertical connector/topology, stable map package, and
object interface package generation together. The review checks whether
`cross_floor_room` and `cross_floor_object` have enough schema-valid candidate
dependencies for Layer 3 candidate route contract generation.

The task29 review is readiness-only. It does not generate final Layer 2 formal
artifacts, does not generate Layer 3 route contracts, and does not generate
routes, executable waypoints, map pixels, runtime input packages, or runtime
artifacts.

## task30 Layer 3 candidate route contract finalization from Layer 2 candidates

`tools/rslg_pipeline/build_route_contracts.py` can now generate non-final Layer
3 candidate route contracts from explicit Layer 2 candidate dependencies. The
candidate mode supports `cross_floor_room` and `cross_floor_object`, using the
task-local vertical connector/topology, stable occupancy map package, and
object-interface candidates reviewed by the Layer 2 integration review.

The task30 candidate contracts remain claim-boundary-protected. They do not
generate final route contracts, real A* routes, executable route waypoints,
runtime input packages, Gazebo/RViz/Nav2/AMCL outputs, object-navigation
runtime evidence, map pixels, PGM/YAML files, connector geometry, or object
approach geometry.

## task31 Layer 3 candidate route contract to route plan dry-run and schema

`tools/rslg_pipeline/build_route_plans.py` can now convert Layer 3 candidate
route contracts into route plan dry-run previews for `cross_floor_room` and
`cross_floor_object`. The previews record route intent, conceptual segment
scope, connector truth, and object approach binding for the object route while
remaining preview-only.

`tools/rslg_pipeline/route_plan_schema.py` validates those route plan previews.
Task31 does not generate real A* routes, executable routes, runtime input
packages, runtime validation artifacts, map pixels, connector geometry, or
object approach geometry.

## task33 Layer 2 final artifact boundary and canonical output plan

`tools/rslg_pipeline/final_artifact_boundary_review.py` defines the boundary
between task evidence, candidate artifacts, future final canonical artifacts,
and runtime artifacts. It records a canonical output plan for future generated
outputs without generating final Layer 2 formal artifacts.

Real route generation remains blocked until final Layer 2 artifacts exist.
Task evidence directories are audit locations, not final output directories.
Future one-command launch scripts should consume canonical outputs, not task
evidence directories.

## task34 Layer 2 final artifact minimal generation plan

`tools/rslg_pipeline/final_layer2_generation_plan.py` defines the minimal final
Layer 2 artifact chain needed before real route generation. It is a
planning/preflight report only and does not generate final Layer 2 artifacts,
final route contracts, route plans, A* routes, waypoint files, runtime inputs,
map pixels, connector geometry, or object approach geometry.

Task34 recommends a Layer 1 canonical rerun preflight before final Layer 2
generation, because final stable maps, connector/topology artifacts, and object
interfaces must be regenerated or verified from current World Model Layer
evidence rather than promoted directly from candidate task evidence.

## task36b Layer 2 stable-map/object coordinate alignment audit

`tools/rslg_pipeline/stable_map_object_alignment_audit.py` reads the canonical
Layer 2 stable map, object interface, room topology, and connector artifacts
without modifying them. It compares the project transform with floor-based
no-y-flip and y-flip transforms, probes occupancy/clearance/object rays, and
writes only small task evidence plus a debug overlay.

For scene `00843-DYehNKdT76V`, the project/no-y-flip convention aligns route
references with free space. The y-flip alternative places them in unknown
space, so the `generated_ring_037` blocker is classified as real rather than a
simple raster-orientation mismatch.
