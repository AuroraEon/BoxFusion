# Code Location and Old-version Policy Draft

## Purpose

This draft separates generated scene artifacts from reusable implementation code and defines how old implementation versions should be handled once a generalized runtime path exists.

## Code Must Not Live in stage_outputs

`stage_outputs` is for generated artifacts, logs, validation evidence, runtime outputs, diagnostics, and scene-specific reports. It must not become a home for reusable Python, shell, ROS launch, or runtime orchestration code.

## Future Centralized Code Location

Future generalized Stage1/Nav2/RViz runtime code should live under one centralized code path, for example:

- `tools/stage1_nav_generalized/`
- `tools/stage1_runtime/`

The exact path should be chosen when implementation begins. This task does not create or implement the generalized runtime code path and does not copy 00824 scripts into a new code directory.

## Scene-specific Runtime Assets

Scene-specific runtime assets may live under the scene output directory, preferably under:

- `runtime/gazebo`
- `runtime/nav2`
- `runtime/rviz`
- `runtime/overlay`
- `runtime/profiles`

These assets are generated or staged outputs, not reusable source code.

## 00824 Read-only Boundary

The accepted 00824 baseline and current runtime tools remain read-only:

- `stage_outputs/stage1_00824_step30p1/`
- `tools/stage1_nav/`
- `tools/stage1_step30p1/`

Do not modify, move, rename, delete, reformat, regenerate, clean, consolidate, or migrate these paths in the current 00843 governance step.

## Old-code Deletion Policy

Once a new generalized implementation is active and validated, old unused code versions should be deleted rather than accumulated. Historical evidence belongs in:

- `stage_outputs/.../runs`
- `stage_outputs/.../analysis`
- `stage_outputs/.../diagnostics`

It should not be preserved as obsolete source files unless the team intentionally keeps a small reference fixture or compatibility shim.

## Current Dirty Overlay Publisher

`tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py` is dirty in the working tree and was inspected read-only. The diff adds fallback lookup for reconstructed 00843 floor assets but keeps 00824 asset names first. It does not change topic names, stage-output-dir argument behavior, or marker namespace policy in the inspected diff. It does change fallback path handling for layered metadata and room masks.

Because this file lives in the 00824 runtime tool path and is dirty, it should not be treated as the generalized 00843 runtime implementation. Future 00843 runtime work should either review and intentionally port the needed behavior into a generalized tool path or replace it with a scene/floor-specific overlay implementation.
