# RSLG-SLAM Formal Artifact Index

This index points to the current 00843 static-artifact chain. Canonical Layer
1/2 artifacts are read-only frozen inputs for Layer 3 QueryTask to RouteResult
planning.

`docs/rslg_slam_planner/` is the current truth surface. The old
`docs/rslg_slam/` tree was migrated and deleted in task53b and must not be
recreated.

## Roots

- Canonical root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- Task root: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/`
- task54 demo pack: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task54_main_chain_handoff_pack/demo_pack/`

## Layer 1: World Model Evidence

- `canonical/layer1_world_model/manifests/canonical_layer1_world_model_manifest_v0_1.json`
- `canonical/layer1_world_model/reports/canonical_layer1_world_model_validation_report_v0_1.json`
- `canonical/layer1_world_model/logs/layer1_world_model_execution.log`

These files are frozen canonical input/evidence for current static planning.
They should not be modified during a static demo. Rebuilding Layer 1 from raw
RGB-D remains legacy Stage-A-backed.

## Layer 2: Formal Artifacts

- Stable maps: `canonical/layer2_formal_artifacts/stable_maps/`
- Object interfaces: `canonical/layer2_formal_artifacts/object_interfaces/`
- Object approach candidates: `canonical/layer2_formal_artifacts/object_interfaces/object_approach_candidates_recovery_v0_1.json`
- Selected object approach: `canonical/layer2_formal_artifacts/object_interfaces/object_approach_selected_v0_2.json`
- Planner graph: `canonical/layer2_formal_artifacts/planner_graph/route_planner_graph_v0_1.json`
- Topology: `canonical/layer2_formal_artifacts/topology/cross_floor_topology_v0_1.json`
- Vertical connector artifacts: `canonical/layer2_formal_artifacts/vertical_connectors/`
- Manifests and validation reports: `canonical/layer2_formal_artifacts/manifests/` and `canonical/layer2_formal_artifacts/reports/`

These are the frozen canonical Layer 2 inputs consumed by the current planner.
They should not be regenerated or edited during static handoff demos.

Stable maps are RSLG-SLAM formal artifacts derived from world-model evidence.
They are not external GT maps, semantic floorplans, room masks, runtime
costmaps, or active `map_server` products.

## Layer 3: QueryTasks And RouteResults

- QueryTask configs: `configs/rslg_queryset_v0/*.json`
- QuerySet manifest: `configs/rslg_queryset_v0/queryset_manifest.json`
- Current task54 RouteResults: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task54_main_chain_handoff_pack/demo_pack/route_results/`
- Historical canonical Layer 3 evidence: `canonical/layer3_navigation_interface/`

The QueryTask configs are current Layer 3 input. Generated task RouteResults are
task-local demo outputs and may be regenerated under task directories. Historical
canonical Layer 3 files are evidence/comparison only for the current frozen
static path.

## Layer 4: Adapter Inputs And Runtime Evidence

- Current task54 adapter inputs: `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task54_main_chain_handoff_pack/demo_pack/runtime_adapter_inputs/`
- Historical canonical Layer 4 evidence: `canonical/layer4_runtime_validation/`

Task-local Layer 4 adapter inputs are regenerated from RouteResults. They
include PID follower inputs, RViz marker inputs, and z-aware overlay inputs.
The z-aware vertical transition is visualization-only.

Historical canonical Layer 4 evidence remains evidence only and does not define
an active Nav2, AMCL, `map_server`, ROS lifecycle, `planner_server`,
`controller_server`, `bt_navigator`, `NavigateToPose`, or `FollowPath`
dependency for the current formal path.

## Guard Truth

- Selected object approach: `generated_ring_002`
- Blocked/rejected evidence only: `generated_ring_037`
- Transition edge: `vt_1_centerline_e001`
- Non-transition edge: `vt_1_centerline_e003`

`tools/rslg_pipeline/planning/artifact_loader.py` reads the Layer 2 object,
topology, connector, stable map, and comparison artifacts. The current planner
uses canonical real routes only as comparison/provenance; the RouteResult metric
path is dynamically stitched.
