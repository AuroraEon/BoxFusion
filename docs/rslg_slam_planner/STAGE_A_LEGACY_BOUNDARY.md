# Stage-A Legacy Boundary

`stage_a_demo.py` is legacy provenance, not the current formal project
entrypoint for the planner or static demo pack.

Current planner/runtime demo tasks must not call it. It should not be deleted
by this documentation cleanup. Layer 0/1 provenance still depends on its
historical lineage for full raw RGB-D to Layer 1 reruns until a clean current
Layer 1 builder exists.

`docs/rslg_slam/` was migrated into `docs/rslg_slam_planner/` and deleted in
task53b. The old docs tree must not be restored as current truth.

## Migrate Conceptually

- dataset/config/checkpoint/class/text-feature provenance
- Layer 1 world-model output manifests
- room/floor/object/topology evidence
- free-space, wall, outside-boundary, gateway, and connector evidence

## Do Not Migrate As Current Truth

- old paper demo scoring
- old VLN/advisor bundles
- teacher-facing replay packaging
- service/debug artifact materialization
- Nav2, AMCL, or map_server implications
- old `00824`, `Step30P1`, `Stage1`, task39, task41, or task42 runtime-success
  wording as the current formal path
