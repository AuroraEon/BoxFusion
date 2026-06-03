# 00843 Scene Directory and Development Standard

## 1. Purpose

This draft establishes the 00843 scene-specific directory and development standard for the next Stage1 generalization step. It stages a governance contract for scene isolation, generated-artifact boundaries, code-location policy, floor-aware organization, cross-floor readiness, object-level artifact readiness, and future runtime compatibility.

This is not runtime evidence. It does not run Nav2, Gazebo, RViz, FollowPath, or NavigateToPose. It does not claim 00843 GUI/Nav2 success.

## 2. Scope and Non-goals

Scope:

- Use the accepted 00824 pipeline audit as the contract source for 00843 design.
- Define a scene-local 00843 artifact layout under `stage_outputs/stage1_generalization/00843-DYehNKdT76V`.
- Classify existing 00843 runtime leakage from copied or active 00824 assets.
- Stage policy drafts for generated artifacts, git submission, code location, and old-code deletion.
- Create future-compatible empty directory skeletons for floor-specific, cross-floor, object-level, runtime, run, validation, diagnostics, and analysis outputs.

Non-goals:

- No live ROS, Gazebo, Nav2, RViz, launch, route execution, FollowPath, or NavigateToPose.
- No DWB repair.
- No 00843 GUI/Nav2 success claim.
- No migration, cleanup, or restructuring of the accepted 00824 baseline.
- No modification of `tools/stage1_nav` or `tools/stage1_step30p1`.
- No new StepXX runtime evidence lane.
- No write under `runtime_stage1_frozen_evidence`.

## 3. Accepted 00824 Contract Source

The accepted 00824 baseline remains read-only:

- `stage_outputs/stage1_00824_step30p1/`
- `tools/stage1_nav/`
- `tools/stage1_step30p1/`

The 00824 read-only audit is accepted as the pipeline contract source for this 00843 design step:

- `analysis/00824_readonly_pipeline_audit.md`
- `analysis/00824_readonly_pipeline_audit.json`
- `analysis/00843_00824_string_leakage_scan.md`
- `analysis/00824_readonly_file_inventory.txt`

Contract caveats:

- Dual-wall visualizations are not necessarily runtime-loaded, but they are audit-critical and contract-critical because they preserve gateway/navigation wall provenance.
- Committed/public topology JSONs alone do not prove Nav2 execution.
- Runtime-loaded files and audit-critical files must be distinguished.
- The audit is sufficient for 00843 design, but not sufficient for a 00843 Nav2 rerun.
- The accepted 00824 wording remains: “Step30P1 repaired execution succeeded with clean forward-only fallback.”

## 4. Why 00843 Needs Scene-specific Runtime Isolation

00843 already contains useful canonical Stage1 and committed/public artifacts, reconstructed floor_1/floor_2 dual-wall raster diagnostics, a candidate floor_2 map, and candidate floor_2 route artifacts. The rejected floor_2 runtime evidence cannot be accepted because FollowPath aborted, room_14 was not physically visited, wall validation failed, route/trajectory samples hit occupied cells, GUI scene isolation failed, and copied 00824 runtime assets were actually launched.

Scene isolation is therefore a first-class requirement. Future 00843 runtime files must not depend on 00824 worlds, RViz configs, Nav2 launch defaults, Nav2 params, marker topics, marker namespaces, map defaults, route defaults, or validation-output paths.

## 5. Directory Principles

- `stage_outputs` is for generated artifacts, logs, validation evidence, runtime outputs, and scene-specific reports.
- Code must not live inside `stage_outputs`.
- Scene directories must distinguish source data, canonical Stage1 outputs, committed/public artifacts, process/intermediate artifacts, maps, routes, runtime assets, run logs, validation results, diagnostics, and analysis reports.
- Floor-specific artifacts must carry floor identity in their directory path and, where practical, in filenames or manifests.
- Active runtime profiles must be separate from rejected run evidence.
- Rejected evidence must remain marked rejected and must not be used as success evidence.
- Generated artifacts and logs should not be committed to git by default.
- Small schemas, README files, and curated contract reports may be committed only if the team decides they are useful.

## 6. Proposed 00843 Scene Directory Layout

Recommended target structure:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/
  input/
    dataset_manifest.json
    scene_config.yaml

  canonical_stage1/
    raw_outputs/
    logs/

  committed_public/

  process/
    floors/
      floor_1/
        room_segmentation/
          assets/
          visualizations/
        gateway/
          assets/
          visualizations/
        maps/
      floor_2/
        room_segmentation/
          assets/
          visualizations/
        gateway/
          assets/
          visualizations/
        maps/

    vertical_transitions/
      candidates/
      validated/
      visualizations/

    objects/
      detections_or_candidates/
      room_object_index/
      object_query_targets/
      visualizations/

  routes/
    room_routes/
    cross_floor_routes/
    object_routes/

  runtime/
    profiles/
      floor_1_nav2/
      floor_2_nav2/
      cross_floor_nav2/
      object_nav_room_level/
    gazebo/
    nav2/
    rviz/
    overlay/

  runs/
    rejected/
    accepted/
    active/

  validation/
    route/
    map/
    gateway/
    floor_transition/
    object_target/
    gui/

  diagnostics/
    leakage_scan/
    scene_isolation/
    failure_reports/

  analysis/
```

The directory skeleton has been created for future work. Existing 00843 artifacts were not moved into this layout during this governance step.

## 7. Artifact Categories

- `input`: source-scene references, dataset manifest, scene configuration, immutable scene identity.
- `canonical_stage1`: canonical Stage1 raw outputs and logs for this scene.
- `committed_public`: curated public semantic topology and room-world outputs.
- `process`: intermediate floor-specific room segmentation, gateway, map-building, vertical-transition, and object artifacts.
- `routes`: topology-query outputs, floor-specific room routes, cross-floor routes, and object routes.
- `runtime`: generated runtime assets used by Gazebo, Nav2, RViz, and overlay publishers.
- `runs`: accepted, rejected, and active runtime evidence lanes.
- `validation`: route, map, gateway, floor-transition, object-target, and GUI validation outputs.
- `diagnostics`: leakage scans, scene-isolation checks, failure reports, and supporting diagnostics.
- `analysis`: human-readable and machine-readable design reports.

## 8. Runtime-loaded Files vs Audit-critical Files

Runtime-loaded files are files that a future launch, route executor, overlay publisher, RViz session, validator, or runtime profile actively consumes. Examples include map YAML/PGM/NPZ, route and waypoint JSONs, Gazebo world files, Nav2 launch/config files, RViz configs, overlay topic configuration, spawn pose, static TF parameters, and validation-output paths.

Audit-critical files are files that establish provenance, contract shape, or validation context but may not be loaded at runtime. Examples include dual-wall visualizations, gateway-wall preclose exports, visual comparison overlays, source manifests, failure reports, and directory-governance reports.

00843 must keep these categories separate. A file can be audit-critical without being runtime-loaded. A committed/public topology JSON can be public and important without proving physical Nav2 execution.

## 9. Floor-specific Artifact Policy

Every floor-specific artifact must live under a floor-specific path or declare a floor id in its manifest:

- `process/floors/floor_1/...`
- `process/floors/floor_2/...`
- `routes/room_routes/...`
- `runtime/profiles/floor_1_nav2/...`
- `runtime/profiles/floor_2_nav2/...`
- `validation/map/...`
- `validation/gateway/...`

Floor-specific map YAML, map origin, route waypoints, spawn pose, static TF, room mask, gateway validation mask, and validation mask must all share the same 00843 floor frame before any future runtime attempt.

## 10. Vertical-transition / Cross-floor Navigation Policy

Cross-floor support must be staged explicitly:

- Put candidate stairs/elevator/transition artifacts under `process/vertical_transitions/candidates`.
- Put validated transition connectors under `process/vertical_transitions/validated`.
- Put visual evidence under `process/vertical_transitions/visualizations`.
- Put cross-floor routes under `routes/cross_floor_routes`.
- Put cross-floor runtime profiles under `runtime/profiles/cross_floor_nav2`.
- Put floor-transition validation under `validation/floor_transition`.

Full cross-floor Nav2 is not complete and must not be claimed until floor-specific maps, transition connectors, route outputs, runtime profiles, and floor-transition validation all pass.

## 11. Object-level Navigation Artifact Policy

Object-level navigation support must be staged without claiming completion:

- Put object candidates or detection-like artifacts under `process/objects/detections_or_candidates`.
- Put room-object indexing outputs under `process/objects/room_object_index`.
- Put object query target outputs under `process/objects/object_query_targets`.
- Put object visualizations under `process/objects/visualizations`.
- Put object routes under `routes/object_routes`.
- Put object-target validation under `validation/object_target`.
- Put object-room-level runtime profiles under `runtime/profiles/object_nav_room_level`.

Object-level navigation is not complete. Do not claim online object detection, object-level navigation completion, or physical object approach success until object candidates, object-room index, object query result, object target/approach point, object route, runtime profile, and validation evidence are all present and accepted.

## 12. Runtime Profile Policy

Runtime profiles must be scene-specific and floor-specific. A valid 00843 runtime profile must name:

- the exact 00843 map YAML/PGM/NPZ;
- the exact 00843 route and waypoint files;
- the 00843 Gazebo world;
- the 00843 Nav2 launch/config files;
- the 00843 RViz config;
- overlay topics and namespaces;
- spawn pose and static TF assumptions;
- validation-output directory;
- run id and floor id.

No active 00843 runtime profile may point to `stage_outputs/stage1_00824_step30p1`, `tools/stage1_nav`, `tools/stage1_step30p1`, copied 00824 world/config/RViz files, or 00824 marker conventions.

## 13. Run Log and Validation Evidence Policy

Runs must be separated by status:

- `runs/active`: currently staged or in-progress run outputs.
- `runs/rejected`: failed or invalid evidence retained for diagnosis.
- `runs/accepted`: future accepted evidence only.

The previous 00843 floor_2 Nav2 run remains rejected. It must not be promoted to accepted evidence because FollowPath aborted, room_14 was not physically visited, wall validation failed, route/trajectory samples hit occupied cells, and 00824 runtime assets leaked into GUI/runtime evidence.

Validation categories must be explicit: route, map, gateway, floor transition, object target, and GUI.

## 14. Generated-artifact and Git Submission Policy

Generated scene directories, runs, maps, route outputs, validation logs, diagnostics, generated runtime assets, screenshots, point clouds, and cache files should not be committed by default.

Candidate commit material may include:

- small schemas;
- README files;
- curated policy drafts;
- curated contract reports;
- machine-readable manifests that are intentionally small and stable.

Large or reproducible generated outputs should stay out of git unless the team explicitly curates them as reference fixtures.

## 15. Code Location Policy

Code must not live in `stage_outputs`. Future generalized runtime code should eventually live under a centralized code path such as:

- `tools/stage1_nav_generalized/`
- `tools/stage1_runtime/`

This task does not create or implement that code path. It also does not copy 00824 scripts into a new directory.

## 16. Old Code Version Deletion Policy

Once a new generalized implementation is active and validated, unused old code versions should be deleted rather than accumulated. Historical evidence should live in `stage_outputs/runs`, `analysis`, or `diagnostics`, not in obsolete code files.

Do not delete the accepted 00824 baseline or current 00824 runtime tools now. They remain the accepted read-only reference.

## 17. 00843 Current Leakage and Rejected Evidence Policy

The leakage scan found active 00824 runtime leakage in the 00843 scene directory, including copied or active references to:

- 00824 Nav2 config;
- 00824 Nav2 launch file;
- 00824 Gazebo world;
- 00824 RViz config;
- 00824 marker/topic conventions;
- 00824 map/path defaults;
- previous run logs proving those assets were actually launched.

These files are classified in `diagnostics/scene_isolation/00843_scene_isolation_gate_report.md`. They were not destructively remediated in this task, except for one generated `__pycache__` cache directory inside the 00843 scene output area.

## 18. Pre-Nav2 Scene Isolation Gate

Before any future 00843 Nav2/Gazebo/RViz run, all of the following must be true:

- no active 00824 Gazebo world is loaded;
- no active 00824 RViz config is loaded;
- no active 00824 Nav2 launch/config defaults are loaded;
- no active 00824 map path is loaded;
- no active 00824 route or waypoint file is loaded;
- no active 00824 marker namespace or topic is required;
- map YAML, map origin, route waypoints, spawn pose, static TF, room mask, gateway validation mask, and validation mask all share the same 00843 floor frame;
- RViz `/map` displays the 00843 floor map;
- overlay topics/namespaces are 00843/floor-specific;
- validation outputs are written under the 00843 run directory;
- rejected previous 00843 runs remain marked rejected.

Current gate status: FAIL.

## 19. What Must Not Be Claimed Yet

Do not claim:

- 00843 GUI/Nav2 success;
- AMCL usage;
- manual `cmd_vel`;
- physical robot deployment;
- full collision-free guarantee;
- online object detection;
- object-level navigation completion;
- full cross-floor Nav2 completion;
- full multi-scene gateway extraction generalization;
- physical room visit from topology route inclusion alone;
- GUI success from Gazebo-only display.

GUI success requires Gazebo plus RViz BEV semantic overlay with scene-isolated 00843 assets.

## 20. Next Engineering Step After This Standard

The next engineering task should be a non-runtime remediation pass:

1. Quarantine or replace active copied 00824 runtime assets under the 00843 scene directory.
2. Create 00843/floor-specific runtime profile manifests under `runtime/profiles`.
3. Stage 00843/floor-specific Gazebo, Nav2, RViz, overlay, map, route, frame, and validation path contracts.
4. Re-run the scene-isolation gate statically.
5. Only after the gate passes, consider a future runtime task.
