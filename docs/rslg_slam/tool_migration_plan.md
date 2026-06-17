# RSLG-SLAM Tool Migration Plan

This is a planning document. Task25d does not move, rename, delete, or create pipeline tool modules.

## Target Layout

Future reusable pipeline tools should converge toward:

`tools/rslg_pipeline/`

The target should become a canonical, test-backed command surface for generating and validating RSLG-SLAM artifacts while preserving older task and experiment scripts as references until explicit migration work is authorized.

## Current Tool Area Classification

| Path | Status | Category | Direction |
| --- | --- | --- | --- |
| `tools/stage1_runtime` | present | `current_active` | migrate reusable runtime export/validation pieces after tests exist |
| `tools/stage1_nav` | present | `current_active` | migrate reusable navigation builders after interface audit |
| `tools/object_nav` | present | `current_active`, `duplicate_risk` | split durable object-interface logic from demo/task-specific wrappers |
| `tools/vertical_connectors` | present | `current_active` | migrate connector builders/exporters after task25b outputs are reviewed |
| `tools/stage1_step30p1` | present | `legacy_keep_for_reference`, `protected_reference` | keep as historical/reference implementation unless explicitly superseded |
| `tools/rslg_pipeline` | absent | `migration_candidate` | future canonical namespace |

## Future Canonical Modules

- `tools/rslg_pipeline/build_world_model.py`
- `tools/rslg_pipeline/build_stable_maps.py`
- `tools/rslg_pipeline/build_vertical_connectors.py`
- `tools/rslg_pipeline/build_object_interfaces.py`
- `tools/rslg_pipeline/build_route_contracts.py`
- `tools/rslg_pipeline/validate_artifacts.py`
- `tools/rslg_pipeline/export_runtime_inputs.py`
- `tools/rslg_pipeline/artifact_registry.py`
- `tools/rslg_pipeline/common.py`

## Migration Principles

- Do not move scripts until a future migration task defines source-to-target mappings and tests.
- Keep task evidence and staged rerun artifacts separate from canonical `clean_rerun` outputs until explicitly promoted.
- Treat stable occupancy map generation as Layer 2 and stable occupancy map consumption as Layer 3.
- Preserve project truth, protected assets, and validated milestones while allowing ordinary planning updates to cleanup, legacy inventory, gitignore plan, and tool migration plan manifests.
- Prefer wrappers or adapters during transition so existing task evidence remains reproducible.

## Recommended Next Migration Step

Perform a read-only source-to-target audit that maps active tool entrypoints to future `tools/rslg_pipeline/` modules, identifies duplicates, and proposes focused tests before any file movement.
