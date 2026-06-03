# Generated-artifact Git Policy Draft

## Purpose

This draft defines how generated scene artifacts should be handled for 00843 and future Stage1 generalization work.

## Default Rule

Generated artifacts and logs should not be committed to git by default. `stage_outputs` is an output area, not a source-code area.

## Generated Artifacts

Treat the following as generated unless explicitly curated:

- scene output directories under `stage_outputs`;
- maps, route outputs, waypoint outputs, and map previews;
- runtime assets generated for Gazebo, Nav2, RViz, and overlays;
- run logs, process snapshots, PIDs, command logs, and runtime diagnostics;
- validation outputs and failure reports;
- screenshots, visualizations, point clouds, and large binary products;
- cache files such as `__pycache__`, `.pyc`, and temporary launch/runtime products.

## Candidate Curated Material

The team may choose to commit small and stable files when they are useful as contracts or documentation:

- schemas;
- README files;
- curated policy drafts;
- curated contract reports;
- small manifests that document expected structure or provenance.

## 00843 Policy

For 00843, the reports in `analysis` and `diagnostics/scene_isolation` may be reviewed as curated documentation. Existing maps, runs, generated runtime files, diagnostics, visualizations, and logs should remain generated outputs unless explicitly promoted by the team.

## Cache Handling

Generated cache files inside a scene output area may be deleted as housekeeping when the deletion is documented. In this task, the 00843 `nav2/launch/__pycache__` directory was removed because it contained only a compiled cache copy of the copied 00824 launch file.

## Git Hygiene

Future `.gitignore` or export policy should keep large generated artifacts out of normal commits while allowing intentionally curated contract reports. This draft does not change the repository-level `.gitignore`.
