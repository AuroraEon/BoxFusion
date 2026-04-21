# Full Pipeline Performance Opportunity Audit

Date: 2026-04-21

## Executive summary

The current Stage-A benchmark/paper path is already in the "narrow residuals only" phase, not the "broad performance hunt" phase.

Repository-grounded conclusion:

- The two meaningful adopted-path wins are already in place:
  - `--suppress-service-debug-artifacts`
  - maintained Stage-5 candidate-index caching via `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
- The largest remaining real costs are still:
  - Stage-5 candidate-mask floor-assignment work in `demo.py::_build_floor_scoped_candidate_mask(...)`
  - snapshot-time vector-map export materialization in `FloorAwareRoomSegmenter.get_vector_map_data(...)`, especially object export and anchor build
- But the remaining safe opportunities are small.
- The larger remaining opportunities are either:
  - already partly optimized,
  - semantics-sensitive because they touch authoritative snapshot/export cadence,
  - or outside the allowed scope because they imply history-window changes, method changes, or broader architectural splits.

Bottom line:

- The current adopted path is good enough for paper/benchmark work.
- If the team insists on one more code change, only low-risk hot-path cleanup is justified.
- Otherwise the evidence says to stop performance work and return to paper experiments.

## Current adopted benchmark/paper runtime path

Current main runnable paper-safe path:

- CLI/wrapper: `stage_a_demo.py`
- runtime loop: `demo.run(...)`
- room/floor segmentation: `FloorAwareRoomSegmenter` / `DynamicRoomSegmenter`
- snapshot/export orchestration: `ClosedLoopDemoRecorder`
- lifecycle/publication: `OnlineTopologyLifecycleManager` / `RoomScopedRuntimeManager`
- authoritative outputs: finalize-time committed/public exports from `ClosedLoopDemoRecorder.finalize(...)`

Current adopted benchmark/paper assumptions reflected in repo code and recent reports:

- single GPU
- `CUDA_VISIBLE_DEVICES=0`
- `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
- `--core-only`
- `--suppress-service-debug-artifacts`
- `--room-seg-interval 100`
- fast GT RGB resize already adopted
- `00862` may need scene-specific `--capture-stride 25`

Active wiring:

- `stage_a_demo.py:_run_single_sequence(...)` loads config/assets, builds `ClosedLoopDemoRecorder`, and calls `demo.run(...)` (`stage_a_demo.py:184-279`)
- `demo.run(...)` owns frame iteration, preprocessing, inference, association, BoxFusion, segmentation, export calls, and recorder hooks (`demo.py:728-2169`)
- `ClosedLoopDemoRecorder.record_snapshot(...)` feeds lifecycle/publication state from the exported vector map (`boxfusion/stage_a_demo.py:799-893`)
- `ClosedLoopDemoRecorder.finalize(...)` writes the authoritative committed/public bundle via `RoomTopologyBuilder().build(...)` and `RoomScopedRuntimeManager.export_artifacts(...)` (`boxfusion/stage_a_demo.py:1143-1605`)

## Pipeline stage map

| stage | exact path(s) | current role on main path | current runtime note |
| --- | --- | --- | --- |
| Dataset/sample creation | `stage_a_demo.py:_run_single_sequence(...)`, `tools/utils.py:get_dataset(...)`, `boxfusion/capture_stream.py:HM3DDataset.__iter__(...)` | active | live HM3D/CA1M sample stream setup |
| RGB/depth loading and alignment | `boxfusion/capture_stream.py:717-910` | active | host-side `cv2.imread`, `cv2.resize`, pose load, tensor wrapping |
| Pre-infer prep | `demo.py:987-1067`, `boxfusion/preprocessor.py` | active | sample unpack, GT RGB/depth alignment, packaging, device move, normalization, batching |
| Detector call path | `demo.py:1108-1129` | active | detector inference on keyframes |
| Post-detector filtering | `demo.py:1113-1129` | active | score threshold, UV bound, floor mask |
| Room/floor observation | `demo.py:1131-1162`, `demo.py:1173-1181`, `FloorAwareRoomSegmenter.observe_frame(...)` | active | per-frame floor tracking and keyframe chunk accumulation |
| Segmentation refresh | `demo.py:1183-1297`, `FloorAwareRoomSegmenter.perform_segmentation(...)`, `DynamicRoomSegmenter.perform_segmentation(...)` | active | scheduled refresh every `room_seg_interval` |
| Object association | `demo.py:1483-1698`, `Instances3D.spatial_association(...)`, `Instances3D.correspondence_association(...)` | active | retained-history comparison and small-object correspondence |
| BoxFusion | `demo.py:1711-1825`, `boxfusion/box_fusion.py:628-781` | active | multi-view box optimization over eligible fusion lists |
| Semantic classification | `demo.py:1428-1441`, `demo.py:1831-1848` | active | CLIP text classification for first-frame and surviving new boxes |
| Snapshot/lifecycle/publication | `demo.py:908-961`, `boxfusion/stage_a_demo.py:799-893` | active | capture-stride snapshots and lifecycle/publication observation |
| Finalize/export/materialization | `demo.py:2080-2168`, `boxfusion/stage_a_demo.py:1143-1605`, `boxfusion/room_scoped_runtime.py:485-540` | active | authoritative committed/public export |
| Helper/eval/harness | `stage_a_eval/run_paper_ablation_harness.py` | partial | practical benchmark helper only; not runtime bottleneck |
| Shadow/sidecar/debug | `boxfusion/runtime_export_coordinator.py`, `boxfusion/sidecar_exporter.py`, `working_topology_*` | no for adopted path | non-authoritative or suppressed on adopted path |

## Already-resolved performance issues

### 1. Rich service/debug finalize materialization is already suppressible

Evidence:

- `RuntimeArtifactPolicy.materialize_rich_service_debug_artifacts` is driven by `--suppress-service-debug-artifacts` (`boxfusion/runtime_artifact_policy.py:72-150`, `stage_a_demo.py:155-165`, `stage_a_demo.py:519`)
- `ClosedLoopDemoRecorder.finalize(...)` now skips:
  - `room_scoped_runtime_state_v0_1.json`
  - `final_vector_map_snapshot.json`
  - `working_topology_v0_1.json`
  - `working_vs_committed_topology_*`
  - `room_commit_diagnosis_*`
  when suppression is enabled (`boxfusion/stage_a_demo.py:1317-1501`)
- Measured adopted-path finalize timing is already tiny:
  - `00843`: `total_finalize_export_sec = 0.209694s`
  - `00824`: `0.213277s`
  - `00862` adopted `capture-stride 100`: `0.815745s`

Conclusion:

- This optimization is already done.
- Further finalize work is not justified for wall-clock reasons.

### 2. Stage-5 candidate-index caching already removed the proven repeated floor-assignment hotspot

Evidence:

- cache path is active in `demo.py` via `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE` and `_build_floor_scoped_candidate_mask(...)` (`demo.py:54-56`, `demo.py:80-95`, `demo.py:153-352`, `demo.py:1495-1566`, `demo.py:1714-1746`)
- recent validated docs show strong A/B reductions across `00843`, `00824`, and `00862` prefix
- remaining readonly-room lookup cost is negligible:
  - `00843`: `stage5_candidate_readonly_room_lookup_sec mean = 0.000355`
  - `00824`: `0.000623`
  - `00862 capture-stride 25`: `0.001647`

Conclusion:

- The repeated-candidate-index issue is resolved.
- More work in this exact direction would be a new optimization project, not a missing closure step.

### 3. Duplicate same-frame export/materialization is already avoided on the adopted path

Evidence:

- Stage-3 full export deferral exists in `_should_defer_stage3_full_export(...)` (`demo.py:111-124`)
- same-frame export reuse logic exists in `FloorAwareRoomSegmenter.get_vector_map_data(...)` (`boxfusion/floor_aware_room_segmenter.py:468-542`, `816-867`)
- adopted-path runtime instrumentation shows this is not a live issue now:
  - `duplicate_export_frame_count = 0`
  - `duplicate_export_call_count = 0`
  - `same_frame_full_export_reuse_count = 0`
  on `00843`, `00824`, and both `00862` runs inspected

Conclusion:

- This code is already defensive enough.
- Further work on same-frame duplicate export avoidance is unlikely to pay off on the current path.

## Remaining performance opportunity candidates

### Candidate A. Stage-5 candidate-mask floor-assignment work still dominates Stage-5 on larger scenes

Exact path(s):

- `demo.py:_build_floor_scoped_candidate_mask(...)` (`demo.py:153-352`)
- call sites in:
  - association mask prep (`demo.py:1495-1566`)
  - BoxFusion mask prep (`demo.py:1714-1746`)

Current role in main path:

- Active on every keyframe where retained-history association or BoxFusion runs.
- This is still the largest measured Stage-5 sub-bucket after the adopted cache fix.

Evidence:

- `00843`:
  - `stage5_total_sec mean = 0.247164`
  - `stage5_candidate_mask_prep_sec mean = 0.070183`
  - `stage5_candidate_floor_assignment_eval_sec mean = 0.062542`
- `00824`:
  - `stage5_total_sec mean = 0.467085`
  - `stage5_candidate_mask_prep_sec mean = 0.110726`
  - `stage5_candidate_floor_assignment_eval_sec mean = 0.101582`
- `00862` adopted `capture-stride 100`:
  - `stage5_total_sec mean = 1.381908`
  - `stage5_candidate_mask_prep_sec mean = 0.781667`
  - `stage5_candidate_floor_assignment_eval_sec mean = 0.763562`
  - approximate profiled total for `stage5_candidate_floor_assignment_eval_sec`: `~190s`

Already partly optimized:

- Yes. Candidate-index caching is already active and validated.

What kind of optimization it would be now:

- more aggressive indexing/caching
- or a more restrictive retained-history/windowing scheme

Whether remaining issue is evidence-supported or speculative:

- Evidence-supported that this bucket is still large.
- Speculative that another safe win exists without changing method semantics.

Semantics / parity impact:

- Further caching that preserves exact mask semantics could be semantics-preserving in principle.
- In practice, the likely next wins involve stronger assumptions about object-state stability or retained-history pruning, which raises method/parity risk.

Worth doing now:

- Not as a default next step.
- Defer unless the team explicitly wants one more narrow Stage-5-only experiment.

### Candidate B. Snapshot export materialization remains the other real cost center

Exact path(s):

- `demo.py:923-956`, `1925-1933`, `2080-2088`
- `FloorAwareRoomSegmenter.get_vector_map_data(...)` (`boxfusion/floor_aware_room_segmenter.py:468-814`)
- object export reuse paths:
  - `_build_room_local_delta_object_exports(...)` (`boxfusion/floor_aware_room_segmenter.py:2499-2746`)
  - `_build_object_exports(...)` (`2437-2497`)
- anchor reuse path:
  - `_build_anchor_layer_with_cached_room_reuse(...)` (`980-1067`)

Current role in main path:

- Active whenever `ClosedLoopDemoRecorder.should_capture(...)` triggers a snapshot export.
- This is still synchronous and authoritative because lifecycle/publication observation consumes the exported vector map.

Evidence:

- `00862` adopted `capture-stride 100`:
  - `snapshot_export_sec mean = 2.176991` across `74` snapshot exports
  - approximate profiled total `~161s`
  - per-export-call averages:
    - `object_export_sec = 0.942342`
    - `anchor_build_sec = 0.911741`
    - `spatial_relations_sec = 0.135163`
    - `scene_graph_build_sec = 0.090554`
- `00824` per-export-call averages:
  - `object_export_sec = 0.173495`
  - `anchor_build_sec = 0.618895`
- `00843` per-export-call averages:
  - `object_export_sec = 0.113715`
  - `anchor_build_sec = 0.220037`

Already partly optimized:

- Yes.
- `room_local_delta_export_used` is already high on the adopted `capture-stride 100` path:
  - `00843`: `96.6%`
  - `00824`: `95.7%`
  - `00862`: `94.7%`
- anchor reuse is already implemented per room-signature cache (`boxfusion/floor_aware_room_segmenter.py:980-1067`)

What kind of optimization it would be now:

- more aggressive incremental caching
- or moving part of snapshot materialization off the synchronous hot path

Whether remaining issue is evidence-supported or speculative:

- Evidence-supported that export is still expensive.
- Only partial evidence that another safe win remains, because the obvious reuse layers are already present.

Semantics / parity impact:

- Medium risk.
- This export is authoritative on the current path, not a sidecar.
- Any async split or lighter snapshot surface risks changing lifecycle/publication timing or capture-stride behavior.

Worth doing now:

- Only if time allows.
- Not worth reopening as a broad export refactor.

### Candidate C. Core-only benchmark runs still do avoidable GT-visualization prep

Exact path(s):

- CLI flag default: `stage_a_demo.py:479`
- policy propagation: `stage_a_demo.py:155-165`, `246-269`
- hot path: `demo.py:1033-1048`

Current role in main path:

- Active on the adopted path because `--viz-on-gt-points` defaults true and benchmark/core-only mode does not currently auto-disable `prepare_gt_visualization_pointcloud`.

Evidence:

- `demo.py:1035-1047` backprojects GT depth and builds `xyzrgb` whenever `viz_on_gt_points` is true.
- On the adopted core-only benchmark path:
  - `re_vis` is normally off
  - `save_point_cloud` is off under core-only (`boxfusion/runtime_artifact_policy.py:131-149`)
- The resulting `xyzrgb` is then only used for:
  - rerun point-cloud logging (`demo.py:1322-1323`)
  - optional final point-cloud write (`demo.py:2043-2062`)
- Measured sampled cost:
  - `preinfer_gt_pointcloud_prep_sec mean ≈ 0.011s` across all three inspected scenes

Already partly optimized:

- Yes. Fast GT RGB resize is already adopted in `_resize_rgb_to_depth(...)` (`demo.py:98-108`, `1038-1044`).

What kind of optimization it would be:

- artifact suppression / branch cleanup

Whether remaining issue is evidence-supported or speculative:

- Evidence-supported.
- The cost exists and the produced artifact is unused on the adopted core-only benchmark path.

Semantics / parity impact:

- Low, if limited strictly to the current core-only benchmark path where rerun and point-cloud output stay disabled.

Worth doing now:

- Only if the team wants one last low-risk cleanup.

### Candidate D. Keyframe RGB-to-depth color alignment still does a redundant resize on the room-observation path

Exact path(s):

- keyframe backprojection path: `demo.py:1131-1140`
- HM3D dataset already resizes RGB and depth to configured size before yielding samples: `boxfusion/capture_stream.py:829`, `863`

Current role in main path:

- Active on every keyframe before `room_segmenter.observe_frame(...)`.

Evidence:

- `demo.py:1139` still does:
  - `Image.fromarray(image_rgb).resize((depth_map.shape[1], depth_map.shape[0]))`
- But `HM3DDataset.__iter__(...)` already resizes:
  - RGB to `(img_width, img_height)` at `boxfusion/capture_stream.py:829`
  - depth to `(img_width, img_height)` at `boxfusion/capture_stream.py:863`
- On the current HM3D adopted path, this makes the extra PIL conversion/resize an avoidable host-side copy.

Already partly optimized:

- Fast GT RGB resize exists for the separate GT visualization path, but not for this keyframe segmentation-observation path.

What kind of optimization it would be:

- code cleanup / avoid redundant conversion

Whether remaining issue is evidence-supported or speculative:

- Evidence-supported for HM3D.
- Benefit size is low.

Semantics / parity impact:

- Low, if implemented only as "skip resize when shapes already match" or reusing the same exact interpolation helper.

Worth doing now:

- Only if time allows.

### Candidate E. Capture cadence is a real runtime lever, but it is not a pure optimization knob

Exact path(s):

- snapshot cadence: `ClosedLoopDemoRecorder.should_capture(...)` (`boxfusion/stage_a_demo.py:792-797`)
- snapshot observation: `record_snapshot(...)` (`799-893`)
- capture-stride CLI: `stage_a_demo.py:496`

Current role in main path:

- Active and semantically relevant.
- Snapshot capture controls when lifecycle/publication sees exported vector maps.

Evidence:

- `00862` parity triage showed:
  - `capture-stride 100` dropped `room_40`
  - `capture-stride 25` restored committed/public parity
  - docs: `docs/00862_parity_triage_capture_stride_followup.md`
- runtime effect is also large:
  - `00862` export call count:
    - `capture-stride 100`: `75`
    - `capture-stride 25`: `297`

Already partly optimized:

- Not applicable. This is a method cadence knob, not an unoptimized code path.

What kind of optimization it would be:

- schedule/cadence change

Whether remaining issue is evidence-supported or speculative:

- Evidence-supported that capture cadence changes runtime and final exported state.

Semantics / parity impact:

- High.

Worth doing now:

- No as a general "performance optimization".
- Treat only as a scene-specific policy exception, exactly as already established for `00862`.

### Candidate F. Room-segmentation cadence is likely semantics-relevant, not just runtime-relevant

Exact path(s):

- scheduled segmentation trigger: `demo.py:1186`
- recorder/export/lifecycle consume those refreshed exports downstream

Current role in main path:

- Active.
- Controls when room geometry/topology is refreshed.

Evidence:

- The code path is explicitly schedule-driven: `if count % room_seg_interval == 0:`
- segmentation refresh updates `latest_vector_map`, `segmentation_cycle_idx`, and later lifecycle/publication observation
- no direct new experiment in this audit proves a current parity break from changing `--room-seg-interval`, but the repo structure strongly implies semantic sensitivity

Already partly optimized:

- Yes. `--room-seg-interval 100` is already the adopted balanced cadence.

What kind of optimization it would be:

- schedule/cadence change

Whether remaining issue is evidence-supported or speculative:

- Partial evidence only.

Semantics / parity impact:

- High or at least non-trivial.

Worth doing now:

- No.
- Keep as an experiment variable, not the next optimization target.

## Opportunity ranking by value and safety

| rank | candidate | value | safety | why |
| --- | --- | --- | --- | --- |
| 1 | Core-only dead GT pointcloud prep cleanup | low | high | real but small wasted host work; semantics-preserving if scoped to benchmark/core-only path |
| 2 | Redundant keyframe RGB resize cleanup | low | high | real micro-copy on active path; very small but safe |
| 3 | Further snapshot export incrementalization | medium | medium | real cost remains, but export is already heavily optimized and authoritative |
| 4 | Further Stage-5 candidate-floor-assignment reduction | medium | medium-low | still large on big scenes, but next wins likely require more aggressive state/index assumptions |
| 5 | Capture-stride tuning | high runtime effect | low safety | explicitly changes committed/public parity on `00862`; not a pure optimization |
| 6 | Room-segmentation cadence tuning | medium runtime effect | low safety | likely changes method behavior/export state |

## What should NOT be optimized further

### Finalize/export overhead

Why not:

- Already tiny on the adopted path.
- `total_finalize_export_sec` is sub-second even on the large `00862` adopted run.
- This is not where wall-clock time is going anymore.

Exact path(s):

- `boxfusion/stage_a_demo.py:1347-1501`
- `boxfusion/room_scoped_runtime.py:485-540`

### Same-frame export reuse / duplicate-export handling

Why not:

- Current adopted-path metrics show zero duplicate same-frame exports and zero cache hits.
- The defensive machinery exists, but the current path does not exercise it.

Exact path(s):

- `demo.py:111-124`
- `boxfusion/floor_aware_room_segmenter.py:482-542`, `816-867`

### Readonly tail reference audit

Why not:

- Disabled by default on the adopted path.
- Metrics show `audit_disabled`; it is not contributing to runtime.

Exact path(s):

- CLI exposure: `stage_a_demo.py:503`
- policy/config flow: `stage_a_demo.py:203-214`
- runtime branch: `demo.py:1496-1514`, `1877-1903`

### BoxFusion internals as a standalone target

Why not:

- `boxfusion_total_sec` is not the dominant Stage-5 bucket anymore.
- The bigger residual in Stage-5 is candidate-mask prep, not the optimization loop itself.

Exact path(s):

- `boxfusion/box_fusion.py:628-781`

### Shadow/sidecar/working-topology paths

Why not:

- Not authoritative and not on the adopted benchmark path.
- `runtime_export_coordinator` explicitly marks sidecar export as shadow-only and non-replacing (`boxfusion/runtime_export_coordinator.py:250-260`)
- `ros_query_server` rejects `working_topology_v0_1.json` as public query input (`boxfusion/ros_query_server.py:361-367`)

## What would require semantic/architectural changes

These should be deferred under the current constraints.

### 1. Moving lifecycle/publication observation off authoritative snapshot export

Why defer:

- `record_snapshot(...)` is the active hook where lifecycle/publication consumes the exported vector map (`boxfusion/stage_a_demo.py:872-893`)
- replacing that with a lighter parallel or sidecar path would alter the current method boundary

### 2. Pruning `per_frame_ins` or retained history more aggressively

Why defer:

- current code/comments already acknowledge cumulative history:
  - `demo.py:1822` notes `per_frame_ins` is cumulative and not window-pruned
- stronger windowing would change correspondence/association behavior, not just speed

### 3. Algorithmic/natively rewritten segmentation acceleration

Why defer:

- segmentation is cadence-limited and no longer the main bottleneck
- deeper wins would likely require algorithm changes or native-port work that the current task explicitly excludes

### 4. Detector/CLIP/model-stack changes

Why defer:

- model inference is still a significant absolute cost (`~0.346s mean` sampled on the inspected scenes), but changing it is out of scope and would alter the paper method stack

### 5. Candidate 2 (`BOXFUSION_FAST_DEPTH_STATS_MODE=kthvalue`) promotion

Why defer:

- This remains opt-in only and outside the adopted freeze path (`boxfusion/preprocessor.py:23`, `119-152`)
- it should stay a low-priority optional path, not the next benchmark recommendation

## Recommended next-step priority list

1. Do not reopen broad performance work.
2. If the team wants exactly one more narrow safe cleanup, do only:
   - core-only dead GT pointcloud prep suppression
   - redundant keyframe RGB resize cleanup
3. Keep the adopted benchmark recipe otherwise unchanged:
   - `CUDA_VISIBLE_DEVICES=0`
   - `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
   - `--core-only`
   - `--suppress-service-debug-artifacts`
   - `--room-seg-interval 100`
4. Keep treating `capture-stride` as semantics-relevant:
   - general adopted path stays unchanged
   - `00862` can remain a scene-specific `--capture-stride 25` exception if parity is required
5. Return to paper experiment execution unless a new measured blocker appears.

## Final recommendation

`only optional future optimizations remain`

Reason:

- The real remaining costs are known.
- The low-risk ones are small.
- The larger ones are either already partly optimized or tied to authoritative export/history semantics.
- Current evidence does not justify another broad optimization cycle.
