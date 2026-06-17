# RSLG-SLAM Final Layer 2 Minimal Generation Plan

This document summarizes the task34 preflight plan for final `Layer 2: Formal Artifact Layer` generation. It is planning only: it does not generate final Layer 1 outputs, final Layer 2 formal artifacts, Layer 3 route contracts, route plans, A* routes, waypoints, runtime input packages, map pixels, connector geometry, or object approach geometry.

## Minimal Generation DAG

The minimal dependency chain before real route generation is:

1. `layer1_world_model_canonical_outputs`
2. `final_stable_occupancy_map_package`
3. `final_vertical_connector_artifact`
4. `final_connector_graph_artifact`
5. `final_cross_floor_topology_artifact`
6. `final_route_planner_graph_or_topology_package`
7. `final_object_query_resolution_artifact`
8. `final_object_approach_artifact`
9. `final_object_interface_package`
10. `final_layer3_route_contract_generation_ready`

For `cross_floor_room`, Layer 3 readiness depends on the final stable map package, vertical connector artifact, connector graph, cross-floor topology, and route planner graph/topology package.

For `cross_floor_object`, the room-route dependencies are required plus final object query resolution, final object approach, and the final object interface package.

## Final Layer 2 Artifact List

The required final Layer 2 artifact specs are:

- `stable_occupancy_map_package_v0_1`
- `vertical_connectors_v0_1`
- `stairs_or_vertical_connector_graph_v0_1`
- `cross_floor_topology_v0_1`
- `route_planner_graph_v0_1`
- `object_query_resolution_v0_1`
- `object_approach_v0_1`
- `object_interface_package_v0_1`

The stable occupancy map package must be a planner-compatible Layer 2 artifact derived from World Model Layer evidence. It is not a semantic floorplan, room mask, runtime costmap, external GT map, or simulator navmesh.

## Promotion Versus Regeneration

Candidate artifacts remain useful lineage, but they are not final canonical outputs.

- Stable map candidate: must regenerate from current Layer 1 outputs.
- Vertical connector candidate: must regenerate or verify from current Layer 1 outputs.
- Connector graph candidate: must regenerate from final Layer 2 inputs.
- Cross-floor topology candidate: must regenerate from final Layer 2 inputs.
- Object query resolution candidate: must regenerate or verify from current Layer 1 outputs.
- Object approach candidate: requires approach feasibility / clearance validation before final use.
- Object interface package candidate: must regenerate from final object query and approach artifacts.
- Route contract candidates: must regenerate from final Layer 2 inputs.
- Route plan previews: must regenerate from final route contracts and planner inputs.

## Canonical Output Paths

Future final generated outputs should live under:

`stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`

Planned subpaths include:

- `layer1_world_model/`
- `layer2_formal_artifacts/stable_maps/`
- `layer2_formal_artifacts/vertical_connectors/`
- `layer2_formal_artifacts/topology/`
- `layer2_formal_artifacts/object_interfaces/`
- `layer2_formal_artifacts/planner_graph/`
- `layer3_navigation_interface/route_contracts/`
- `layer3_navigation_interface/route_plans/`
- `layer3_navigation_interface/real_routes/`
- `layer4_runtime_validation/runtime_inputs/`
- `layer4_runtime_validation/rviz/`
- `layer4_runtime_validation/logs/`
- `manifests/`

Task evidence directories are not canonical final output directories. Future one-command launch scripts must consume canonical outputs, not task evidence outputs.

## Validation Sequence

Future final generation tasks should validate in this order:

1. Stable map schema validation.
2. Vertical connector schema validation.
3. Object interface schema validation.
4. Final Layer 2 integration validation.
5. Final route contract schema validation.
6. Route plan schema validation.
7. Route generation readiness validation.
8. JSON/static validation.

Existing validators mostly support candidate artifacts and previews today, so final-mode extensions are required before final artifact generation.

## Unblock Conditions

Real `cross_floor_room` route generation can begin only after the final stable map package, final vertical connector/topology artifacts, final route planner graph, final route contract regeneration, and route plan regeneration are complete.

Real `cross_floor_object` route generation additionally requires the final object interface package, final object approach artifact, and approach feasibility / clearance validation evidence.

## Next Recommended Task

The recommended next task is:

`task35_layer1_world_model_canonical_rerun_preflight_and_input_contract`
