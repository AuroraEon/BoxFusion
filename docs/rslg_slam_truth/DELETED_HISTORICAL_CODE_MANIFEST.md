# Deleted Historical Code Manifest

This manifest records historical source cleanup performed during `task48a_aggressive_rslg_pipeline_convergence_refactor`.

No historical source paths had been deleted when this truth record was first created. Later deletions in this task are recorded both here and in:

- `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task48a_aggressive_rslg_pipeline_convergence_refactor/03_deleted_files_manifest.csv`
- `stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task48a_aggressive_rslg_pipeline_convergence_refactor/task48a_manifest.json`

## Deletion Policy

Historical source code may be deleted only after:

- essential logic is migrated, wrapped, or documented under `tools/rslg_pipeline/`
- formal `tools/rslg_pipeline/` imports do not depend on the deleted path
- the deletion is recorded with the previous role, reason, replacement, and confidence

Protected assets must not be deleted:

- `/home/ws/data/`
- model checkpoints
- `config/hm3d.yaml`
- `models/cutr_rgbd.pth`
- canonical artifacts under `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/`
- raw RGB/depth/pose data
- GLB/navmesh/semantic assets

## Deleted Paths

| Deleted path | Previous role | Reason for deletion | Replacement or truth record | Confidence |
| --- | --- | --- | --- | --- |
| `tools/object_nav/` | Historical object-navigation planner/runtime/demo prototypes | Useful occupancy planning, object approach, and runtime validation helpers were migrated under `tools/rslg_pipeline/`; remaining demos and live adapters were not formal RSLG-SLAM entrypoints | `tools/rslg_pipeline/planning/occupancy_planner.py`, `tools/rslg_pipeline/planning/approach_candidate_planner.py`, `tools/rslg_pipeline/runtime/route_result_runtime_adapter.py`, `docs/rslg_slam_truth/PROJECT_TRUTH.md` | High |
| `tools/stage1_nav/` | Historical 00824 Stage1 navigation reference | 00824 truth was preserved and the directory was not part of the formal five-layer command surface | `docs/rslg_slam_truth/LEGACY_00824_REFERENCE.md` | High |
| `tools/stage1_runtime/` | Historical Stage1 runtime/Nav2 prototypes | RouteResult-derived Layer 4 adapters now own static runtime validation inputs; live runtime prototypes were removed from the static convergence surface | `tools/rslg_pipeline/export_route_result_runtime_inputs.py`, `tools/rslg_pipeline/runtime/route_result_runtime_adapter.py`, `tools/rslg_pipeline/planning/occupancy_planner.py` | High |
| `tools/stage1_step30p1/` | Historical 00824 Stage1 step30p1 reference | 00824 reference truth was preserved and the implementation was not a formal RSLG-SLAM entrypoint | `docs/rslg_slam_truth/LEGACY_00824_REFERENCE.md` | High |
| `tools/vertical_connectors/` | Historical standalone cross-floor connector planning/replay demos | Cross-floor route and adapter ownership was centralized under the QueryTask planner and RouteResult adapter surface | `tools/rslg_pipeline/planning/floor_connector_route.py`, `tools/rslg_pipeline/planning/route_planner.py`, `tools/rslg_pipeline/runtime/route_result_z_aware_adapter.py` | High |
| `tools/rslg_pipeline/audit_stable_map_acceptance.py` | Obsolete task-specific acceptance audit | Superseded by formal truth docs, `audits/validate_project_truth.py`, and current Layer 2/3 builders | `tools/rslg_pipeline/audits/`, `docs/rslg_slam_truth/PROJECT_TRUTH.md` | High |
| `tools/rslg_pipeline/final_artifact_boundary_review.py` | Obsolete task-specific review | Superseded by formal README ownership boundaries and audit package | `tools/rslg_pipeline/README.md`, `tools/rslg_pipeline/audits/` | High |
| `tools/rslg_pipeline/final_layer2_generation_plan.py` | Obsolete task-specific plan | Superseded by formal Layer 2 entrypoint and project truth constants | `tools/rslg_pipeline/build_layer2_formal_artifacts.py`, `tools/rslg_pipeline/project_truth.py` | High |
| `tools/rslg_pipeline/finalize_task39_runtime_evidence.py` | Obsolete runtime evidence finalizer | Superseded by RouteResult-derived Layer 4 adapter validators and retained canonical artifacts | `tools/rslg_pipeline/export_route_result_runtime_inputs.py`, `tools/rslg_pipeline/audits/validate_route_result_runtime_adapters.py` | High |
| `tools/rslg_pipeline/export_task42_demo_figures.py` and task42 live/replay helpers | Obsolete live/demo presentation helpers | Superseded by RouteResult-derived RViz marker input generation and optional marker publishing; live launchers are outside this convergence task | `tools/rslg_pipeline/runtime/route_result_marker_adapter.py`, `tools/rslg_pipeline/viz/live_marker_publisher.py`, `docs/rslg_slam_planner/ROUTE_RESULT_LAYER4_ADAPTERS.md` | High |
| `tools/rslg_pipeline/export_runtime_inputs.py`, `tools/rslg_pipeline/run_layer4_runtime_validation_static.sh`, and `tools/rslg_pipeline/run_rslg_pipeline_static.sh` | Pre-RouteResult Layer 4 static wrappers and runtime-input exporter | Superseded by the formal `QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs` chain; removed to avoid stale map_server/Nav2-oriented packaging as an active command surface | `tools/rslg_pipeline/export_route_result_runtime_inputs.py`, `tools/rslg_pipeline/audits/validate_route_result_runtime_adapters.py`, `docs/rslg_slam_planner/ROUTE_RESULT_LAYER4_ADAPTERS.md` | High |
| `tools/**/__pycache__/`, `tools/**/*.pyc` | Generated Python bytecode | Bytecode is not project truth and must not be retained as source | Not replaced | High |
