# RSLG-SLAM Layer Entrypoints

`docs/rslg_slam_planner/` is the current truth surface. The old
`docs/rslg_slam/` tree was migrated and deleted in task53b and must not be
recreated.

The current static chain is frozen canonical mode:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

| Layer | Layer name | Conceptual input | Current frozen/static input | Command entrypoint | Output | Current status | Runs in task54 demo path | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Layer 0 | Input Layer | RGB-D frames, depth, provided poses, scene id, sequence id, dataset/config paths, model/checkpoint provenance, CLIP checkpoint provenance, semantic class text, text-feature provenance, and Layer 0 manifest | Provenance recorded in canonical artifacts and optional task-local manifest smoke | `tools/rslg_pipeline/build_input_manifest.py` | `rslg_layer0_input_manifest` JSON | formal provenance helper / manifest-only | Optional smoke only; no inference | QueryTask is not Layer 0. The helper does not run inference or rebuild the world model. |
| Layer 1 | World Model Layer | Layer 0 input manifest and raw scene inputs | Frozen canonical Layer 1 evidence | `tools/rslg_pipeline/build_world_model.py` / `tools/rslg_pipeline/run_layer1_world_model.sh` if present | canonical world-model evidence | legacy Stage-A-backed / not current static demo path | No | Full raw RGB-D to Layer 1 regeneration remains legacy-backed. |
| Layer 2 | Formal Artifact Layer | Canonical Layer 1 world-model evidence | Frozen canonical Layer 2 artifacts | `tools/rslg_pipeline/build_layer2_formal_artifacts.py` if present | stable maps, object interfaces, topology, vertical connectors, object approach candidates, planner graph | formal or near-formal builder; task54 consumes frozen canonical artifacts | Consumed, not regenerated | Do not regenerate canonical artifacts during static handoff demos. |
| Layer 3 | Navigation Interface Layer | QueryTask plus Layer 2 formal artifacts | QueryTask plus frozen canonical artifacts | `tools/rslg_pipeline/plan_query_static.py` / `tools/rslg_pipeline/batch_plan_query_static.py` | `RSLGRouteResult` | formal | Yes | QueryTask is the Layer 3 request object. |
| Layer 4 | Runtime Validation Layer | `RSLGRouteResult` | task-local RouteResults | `tools/rslg_pipeline/export_route_result_runtime_inputs.py` | PID follower, RViz marker, and z-aware overlay adapter inputs | formal | Yes | z-aware vertical transition output is visualization-only. |

`stage_a_demo.py` is legacy Stage-A provenance/exporter material, not the
current formal project entrypoint. The current formal chain has no active Nav2,
AMCL, `map_server`, ROS lifecycle, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, or `FollowPath` dependency.
