# RSLG-SLAM Gitignore Plan

This is a planning document. It does not delete local outputs, and task25d did not modify `.gitignore`.

## Intent

RSLG-SLAM should keep source, tests, configuration, and durable documentation tracked while keeping generated artifacts, runtime evidence, temporary outputs, and large binary products out of git history.

The historical repository path is `BoxFusion`, but the project name is RSLG-SLAM.

## Proposed Ignore Areas

- Generated stage and candidate outputs: `stage_outputs/`, `runtime_export_validation/`, `world_model_backend_outputs*/`, and candidate rerun directories.
- Runtime evidence and logs: `runtime/`, `runs/`, `logs/`, `log/`, ROS runtime traces, tracking logs, and demo evidence generated during validation tasks.
- Temporary and scratch files: `tmp/`, `scratch/`, cache directories, Python bytecode, pytest/mypy/coverage caches, and editor-local files.
- Large media and map artifacts: videos, screenshots when generated as runtime evidence, ROS bags, PGM/YAML map exports, point clouds, mesh exports, and model checkpoints.
- Build/install products: `build/`, `install/`, package metadata, and generated dependency caches.
- Task/candidate outputs: task-local evidence directories under `stage_outputs/` should generally remain local unless explicitly selected for tracking by a future documentation task.

## Intended Tracked Areas

- `boxfusion/`
- `tools/`
- `config/`
- `tests/`
- `docs/rslg_slam/`

## Stage Outputs Policy

`stage_outputs/` should generally be ignored by git because it contains generated evidence, staged reruns, screenshots, runtime logs, and large artifacts. Ignored does not mean disposable: protected local evidence and outputs must be preserved unless a future task explicitly authorizes cleanup.

Protected evidence includes the 00824 reference baseline, the 00843 `clean_rerun` canonical outputs, and protected task evidence paths listed in `protected_assets_manifest_v0_1.json`.

## Patch Candidate

See `docs/rslg_slam/gitignore_candidate.patch`. The candidate is intentionally not applied in task25d because the current `.gitignore` already has broad generated-output coverage, including the 00843 stage output tree, and an aggressive rewrite would exceed this task's safe minimal-change rule.
