# Feature Liveness And Dormant Mechanism Audit

## Executive summary

The current main runnable paper-safe backend is the Stage-A wrapper path rooted at `stage_a_demo.py`, which loads config/model assets, calls `demo.run(...)`, and records lifecycle/publication state through `ClosedLoopDemoRecorder` (`stage_a_demo.py:185-260`, `demo.py:808-909`, `boxfusion/stage_a_demo.py:872-886`). On that path, the truly authoritative outputs are the committed/public exports produced through `RoomTopologyBuilder` plus `RoomScopedRuntimeManager.export_artifacts(...)`, together with the non-public lifecycle history (`boxfusion/stage_a_demo.py:1349-1490`, `boxfusion/room_scoped_runtime.py:390-534`, `boxfusion/online_topology_lifecycle.py:612-705`).

The highest-confusion dormant-looking items are:

- HM3D Tier-2 room-repair auto-enablement, which is configured to look on but is effectively off in this workspace because the required analysis artifact path does not exist.
- `demo.py` and README quick-start references, which still look like first-class entrypoints even though the current main path is the Stage-A wrapper.
- Debug/working topology surfaces and service-side shadow exports, which look like alternate topology backends but are explicitly non-authoritative.
- Old/fallback CLIP and text-feature assets, which remain present and selectable but are not the current first-choice Stage-A assets.
- A full SegFormer package plus loader class, which is present in-repo but has no runnable Stage-A call site.

## What is truly active on the current main Stage-A path

- `stage_a_demo.py` is the main wrapper. It resolves config, CLIP, and text features, constructs `ClosedLoopDemoRecorder`, and then invokes `demo.run(...)` (`stage_a_demo.py:40-131`, `stage_a_demo.py:185-260`).
- `demo.run(...)` is the active backend loop. It instantiates `FloorAwareRoomSegmenter`, processes frames, exports vector-map snapshots, and passes those snapshots into the recorder (`demo.py:808-909`).
- `FloorAwareRoomSegmenter` and `DynamicRoomSegmenter` are the active room-segmentation/runtime-ID machinery on this path (`boxfusion/floor_aware_room_segmenter.py:51-66`, `boxfusion/dynamic_room_segmenter.py:12-48`, `boxfusion/dynamic_room_segmenter.py:1000-1082`).
- `ClosedLoopDemoRecorder.record_snapshot(...)` is where lifecycle/publication logic is actually attached to the active path. It calls:
  - `online_topology_lifecycle.observe_room_tracking(...)`
  - `online_topology_lifecycle.observe_export(...)`
  - `room_scoped_runtime.observe(...)`
  Evidence: `boxfusion/stage_a_demo.py:872-886`.
- Final authoritative export is built at finalize time through:
  - `RoomTopologyBuilder().build(...)`
  - `room_scoped_runtime.export_artifacts(...)`
  Evidence: `boxfusion/stage_a_demo.py:1349-1371`.
- The authoritative/public bundle is committed-room-only. `RoomScopedRuntimeManager` explicitly filters rooms, objects, anchors, gateways, and vertical transitions down to committed/published rooms and labels the payload as public committed-only (`boxfusion/room_scoped_runtime.py:318-387`, `boxfusion/room_scoped_runtime.py:390-458`).
- The active default CLIP/text-feature preference on the current wrapper path is:
  - `./models/ViT-B-32/open_clip_pytorch_model.bin`
  - `./data/class_features_small.pt`
  Evidence: `stage_a_demo.py:40-63`, `stage_a_demo.py:79-131`.

## Misleadingly active-looking mechanisms

### 1. HM3D Tier-2 room-repair policy

- Why it looks enabled:
  - `config/hm3d.yaml` sets `tier2_enablement_mode: auto`.
  - The same block provides a concrete-looking `tier2_policy_artifact` path.
- Why it is not effectively active here:
  - Auto mode requires the policy artifact to exist.
  - `boxfusion/tier2_enablement.py` explicitly blocks auto mode with `missing_policy_artifact_file` / `auto_policy_artifact_not_found` if that file is absent.
  - This workspace has no `analysis/` directory, so `./analysis/tier2_cross_scene_eval/output/summary.json` is missing.
- Evidence:
  - `config/hm3d.yaml:35-48`
  - `boxfusion/tier2_enablement.py:204-220`
  - repository check: `analysis/` absent
- Researcher confusion risk:
  - Very high. A reader of the HM3D YAML can reasonably conclude Tier-2 repair is live on the main path when it is not.

### 2. `demo.py` as an apparent first-class entrypoint

- Why it looks enabled:
  - It remains runnable.
  - README still advertises `python demo.py ...` quick-start commands alongside Stage-A commands.
- Why it is not the current main paper-safe path:
  - The active paper-safe wrapper is `stage_a_demo.py`, not `demo.py`.
  - `demo.py` still contains a hardcoded absolute CLIP path from an older local workspace.
- Evidence:
  - `README.md:57-59`, `README.md:127-142`
  - `stage_a_demo.py:185-260`
  - `demo.py:2275-2288`
- Researcher confusion risk:
  - Very high. This is the strongest entrypoint-level source of confusion in the repo.

### 3. Working topology / withheld-topology files

- Why they look enabled:
  - They are emitted under names like `working_topology_v0_1.json` and `working_vs_committed_topology_report_v0_1.json`, which look like alternate topology surfaces rather than debug artifacts.
- Why they are not effectively part of the authoritative main backend:
  - They are generated only as debug/rich diagnostic artifacts.
  - `ros_query_server.py` explicitly rejects `working_topology_v0_1.json` as public input.
  - Lifecycle export marks itself `debug_only: True` and `public_default: False`.
- Evidence:
  - `boxfusion/stage_a_demo.py:1407-1490`
  - `boxfusion/ros_query_server.py:361-367`
  - `boxfusion/online_topology_lifecycle.py:640-705`
- Researcher confusion risk:
  - Very high, because the filenames look equivalent to the public topology surface.

### 4. Service sidecar export

- Why it looks enabled:
  - The repo has a dedicated sidecar exporter and coordinator notes about parity subsets.
- Why it is not the main Stage-A export path:
  - The sidecar payload says `authoritative: False`.
  - Both the sidecar exporter and coordinator notes say the synchronous exporter remains authoritative.
- Evidence:
  - `boxfusion/sidecar_exporter.py:79-156`
  - `boxfusion/runtime_export_coordinator.py:250-260`
- Researcher confusion risk:
  - High, especially for anyone reading only filenames/contracts and not the payload notes.

### 5. ROS simulation ingress stub scene writer

- Why it looks enabled:
  - It writes `topology_v0_1.json`, `online_topology_lifecycle_v0_1.json`, `working_topology_v0_1.json`, `summary.json`, and other canonical-looking files.
- Why it is not the current main Stage-A backend:
  - The real backend path shells out to `stage_a_demo.py`.
  - The stub writer emits thin placeholder/minimal surfaces and explicitly defers working-vs-committed detail.
- Evidence:
  - `boxfusion/ros_simulation_ingress.py:1098-1145`
  - `boxfusion/ros_simulation_ingress.py:1078`
  - `boxfusion/ros_simulation_ingress.py:1661`
- Researcher confusion risk:
  - High, because the artifact names mimic the real backend.

## Conditional-but-currently-disabled mechanisms

- HM3D Tier-2 auto enablement:
  - Configured on the HM3D path, but blocked by missing policy artifact.
  - Evidence: `config/hm3d.yaml:35-48`, `boxfusion/tier2_enablement.py:204-220`.
- Readonly tail reference audit:
  - Exposed by CLI on both Stage-A wrapper and legacy `demo.py`, but disabled by default and turned off in service policy.
  - Evidence: `stage_a_demo.py:503-506`, `demo.py:2194-2200`, `boxfusion/runtime_artifact_policy.py:84-121`.
- Optional demo/showcase artifact family under `--core-only` or service mode:
  - RGB replay, scene-graph visuals, GraphML, debug-room artifacts, revisit reports, and related showcase outputs are all gated behind optional artifact policy.
  - In service mode they are deferred by default; `ClosedLoopDemoRecorder` disables them when `core_only` is true.
  - Evidence: `boxfusion/runtime_artifact_policy.py:84-143`, `boxfusion/stage_a_demo.py:734-745`, `boxfusion/stage_a_demo.py:1180-1270`.

## Present-but-unreferenced mechanisms

### SegFormer checkpoint package plus `SceneSegmenter`

- Why it looks enabled:
  - The repo contains a complete SegFormer checkpoint package under `models/segformer-b0-finetuned-ade-512-512/`.
  - `boxfusion/segmentor.py` defines `SceneSegmenter` that loads it by default.
- Why it is not effectively active:
  - Repo-wide search finds no runnable Stage-A import/call site for `SceneSegmenter`.
- Evidence:
  - `boxfusion/segmentor.py:7-48`
  - search results: only the defining file and existing docs reference it
- Researcher confusion risk:
  - High. It looks like a learned room-segmentation stage even though the current Stage-A room path is geometric/floor-aware.

### `gen_features.py`

- Why it looks enabled:
  - It writes the currently familiar `./data/class_features_small.pt` artifact name.
- Why it is not effectively active:
  - It has no runtime call site on the Stage-A path.
  - It uses an old absolute CLIP path.
  - The Stage-A wrapper can rebuild text features itself on mismatch.
- Evidence:
  - `gen_features.py:7-12`
  - `stage_a_demo.py:88-131`
- Researcher confusion risk:
  - Medium.

## Legacy / fallback / historical paths that should not be mistaken for the main path

- `demo.py` CLI:
  - Legacy/non-main entrypoint for this audit.
  - Evidence: `stage_a_demo.py:185-260`, `demo.py:2275-2288`, `README.md:57-59`.
- Root-level CLIP checkpoint fallback `./models/open_clip_pytorch_model.bin`:
  - Still selectable, but no longer first-choice on the wrapper path because `./models/ViT-B-32/open_clip_pytorch_model.bin` is preferred first.
  - Evidence: `stage_a_demo.py:40-49`.
- Legacy text-feature fallback `./data/class_features.pt`:
  - Still selectable, but only if `class_features_small.pt` is missing or explicitly bypassed.
  - Evidence: `stage_a_demo.py:54-83`.
- `archive/` Stage-A showcase scripts:
  - Historical/showcase material, not the current backend entrypoint.
  - Evidence: `archive/stage_a_end_to_end_showcase.py`, `archive/stage_a_execution_showcase.py`, `archive/stage_a_showcase_bundle.py`, related archive files.
- `broad_history` history scope:
  - Explicit ablation/baseline path, not default main behavior.
  - It disables the selective floor-aware pruning path.
  - Evidence: `stage_a_demo.py:492-497`, `demo.py:223-229`, `demo.py:555-557`.

## Safe-to-delete candidates

These are high-confidence only; anything else should be marked or quarantined before deletion.

- SegFormer checkpoint package and `boxfusion/segmentor.py`
  - Rationale:
    - Present but not referenced by the runnable Stage-A backend.
    - Strong confusion risk because it looks like an active learned room-segmentation stage.
  - Minimum safe action:
    - Delete after one final human confirmation that no private/off-repo workflow still imports `SceneSegmenter`.
- `gen_features.py`
  - Rationale:
    - Not on the runtime path and superseded by wrapper-side auto-rebuild logic.
  - Minimum safe action:
    - Prefer mark dormant first; delete only after confirming nobody still uses it for manual preprocessing.

## Keep-but-mark-dormant candidates

- HM3D Tier-2 auto policy block
- Working/withheld topology debug surfaces
- Legacy text-feature fallback `data/class_features.pt`
- Readonly tail reference audit
- Optional demo/showcase artifact family when evaluating paper-safe/core/service outputs
- `broad_history` ablation path
- `gen_features.py` if the team wants to keep it as a manual helper

## Keep-as-legacy candidates

- `demo.py` as an old/non-main entrypoint
- Root-level CLIP fallback `models/open_clip_pytorch_model.bin`
- `archive/` Stage-A showcase scripts

## Unclear items requiring human review

- Service sidecar export:
  - Clear that it is non-authoritative, but unclear whether the team intends to evolve it into a maintained service-facing surface or keep it as an experiment.
- ROS simulation ingress stub scene writer:
  - Clear that it is not the main backend, but unclear whether it remains required for current simulator integration, tests, or demos.

## Risks of deleting too early

- Deleting Tier-2-related code would remove a clearly intended but presently artifact-blocked path; this is not dead code in the same sense as an unreferenced loader.
- Deleting the service sidecar or simulation-ingress stub without human confirmation could break service/simulator experiments even though they are not the main paper-safe path.
- Deleting fallback CLIP/text-feature assets could break older local setups still using README-era assumptions.
- Deleting `demo.py` outright would remove a still-documented path before the README and any downstream scripts are cleaned up.

## Recommended cleanup order

1. Document the real Stage-A main path at the top of the README and de-emphasize `demo.py`.
2. Mark Tier-2 HM3D auto enablement as artifact-blocked/off in this workspace unless the required analysis summary is present.
3. Mark working/withheld topology artifacts and sidecar exports as non-authoritative in filenames/docs wherever possible.
4. Quarantine `archive/` showcase scripts and legacy fallbacks behind an explicit “historical/legacy” heading.
5. Delete only high-confidence unreferenced assets/loaders after confirmation:
   - `boxfusion/segmentor.py`
   - `models/segformer-b0-finetuned-ade-512-512/`
   - maybe `gen_features.py`

## Ranked cleanup candidates by confusion risk

1. `demo.py` as an apparent main entrypoint
   - Why ranked here:
     - README plus a runnable CLI make it look current.
     - It is not the current paper-safe Stage-A wrapper.
   - Minimum safe action:
     - de-emphasize/quarantine

2. HM3D Tier-2 auto enablement
   - Why ranked here:
     - The config reads as enabled.
     - The required artifact is missing, so the path is effectively off here.
   - Minimum safe action:
     - mark dormant

3. Working/withheld topology debug surfaces
   - Why ranked here:
     - Their filenames look like public topology products.
     - They are debug-only and explicitly rejected by the public query server.
   - Minimum safe action:
     - mark dormant

4. Service sidecar export
   - Why ranked here:
     - “export” plus coordinator wiring makes it look like an alternate official backend.
     - The payload explicitly says it is shadow-only and non-authoritative.
   - Minimum safe action:
     - document only

5. ROS simulation ingress stub scene root
   - Why ranked here:
     - It materializes canonical filenames.
     - It is a thin adapter/fallback, not the real Stage-A producer.
   - Minimum safe action:
     - de-emphasize/quarantine

6. SegFormer package plus `SceneSegmenter`
   - Why ranked here:
     - It looks like a major learned segmentation subsystem.
     - No runnable Stage-A call site was found.
   - Minimum safe action:
     - delete after confirmation

7. Root-level CLIP and old text-feature fallbacks
   - Why ranked here:
     - They remain present and documented, but they are no longer the preferred Stage-A assets.
   - Minimum safe action:
     - document only
