# Layer 0-2 Provenance Audit

Project: RSLG-SLAM. This audit is static only; Stage-A was not executed.

`docs/rslg_slam_planner/` is the current truth surface for this provenance
contract. The old `docs/rslg_slam/` tree was migrated and deleted in task53b.

## Current Formal Truth

- Layer 0 is raw input and provenance: RGB-D frames, depth, provided poses,
  sequence id, dataset/config paths, semantic class text, model/checkpoint
  provenance, CLIP checkpoint provenance, text-feature provenance, and a Layer 0
  input manifest.
- QueryTask is not Layer 0. QueryTask is the request object consumed by Layer 3.
- `tools/rslg_pipeline/build_input_manifest.py` records Layer 0 provenance only.
  It does not run inference, import Stage-A, require GPU, or modify canonical
  artifacts.
- Layer 1 canonical artifacts live under
  `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer1_world_model/`.
- Layer 2 canonical artifacts live under
  `stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical/layer2_formal_artifacts/`.
- The current static planner consumes frozen canonical Layer 1/2 artifacts. It
  does not regenerate them.

## Frozen Canonical Mode

The task49-task52 static path is:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

This mode is acceptable for the current static demo pack and engineering
closure because canonical artifacts are the evidence source being validated by
the planner and adapter tools. It is not a claim of a full raw RGB-D to Layer 1
rerun.

## Legacy Provenance

- `stage_a_demo.py` preserves the old Stage-A world-model/demo exporter path:
  dataset config, checkpoint, CLIP/text features, `demo.run`, debug-room
  artifacts, instrumentation, and demo summaries.
- `demo.py` and older `boxfusion/` modules explain how posed RGB-D, semantic
  detections, topology snapshots, runtime instrumentation, and old
  room-graph/VLN evidence were originally produced.
- `tools/rslg_pipeline/build_world_model.py` is a legacy-backed Layer 1 wrapper.
  It delegates to `stage_a_demo.py` when a rerun is explicitly allowed and is
  not the current static demo entrypoint.
- `tools/rslg_pipeline/build_layer2_formal_artifacts.py` is the current final
  Layer 2 canonicalizer. It consumes canonical Layer 1 outputs and produces
  stable maps, topology, vertical connectors, object interface artifacts,
  reports, and manifests.

Stable maps are RSLG-SLAM formal artifacts derived from world-model evidence.
They are not external GT maps, semantic floorplans, room masks, runtime
costmaps, or active map_server products.

## Missing From The Clean Formal Surface

- A Stage-A-independent Layer 1 builder that exposes only RSLG-SLAM world-model
  products and does not carry paper-demo/exporter behavior.
- A narrow Layer 2 rebuild command for object resolution and approach candidates
  from current Layer 1/2 evidence, without older candidate-only assumptions.
- A reproducible provenance bundle that links Layer 0 input manifests to Layer 1
  raw outputs and Layer 2 final artifacts.

## Recommended Migration

Task53b migrated the useful old `docs/rslg_slam/` facts into the current planner
truth surface: project identity, the Layer 0 raw/provenance input contract,
official layer names, Stage-A boundary, stable-map distinction, 00843 object and
transition truth, generated-output policy, and claim boundaries.

Future Layer 0/1 work should still migrate dataset/config/checkpoint/class-text
and text-feature provenance from legacy Stage-A lineage into richer Layer 0
manifest records. The current QueryTask to RouteResult planner remains separate
from old demo/export behavior.
