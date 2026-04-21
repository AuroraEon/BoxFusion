# Large Refactor Feasibility For Major Speedup

Date: 2026-04-21

## Executive summary

The current Stage-A benchmark/paper path is already close to its local optimum for "same system" comparisons.

Repository-grounded bottom line:

- The adopted paper-safe path is:
  - `stage_a_demo.py:_run_single_sequence(...)`
  - `demo.run(...)`
  - `FloorAwareRoomSegmenter` / `DynamicRoomSegmenter`
  - `ClosedLoopDemoRecorder`
  - `OnlineTopologyLifecycleManager` / `RoomScopedRuntimeManager`
  - finalize-time committed/public exports
- The meaningful safe wins already landed:
  - `--suppress-service-debug-artifacts`
  - maintained Stage-5 candidate-index caching via `BOXFUSION_STAGE5_CANDIDATE_INDEX_CACHE=1`
  - scene-specific `00862` `--capture-stride 25` exception for committed/public parity
- The remaining large costs are structural:
  - Stage-5 retained-history candidate-mask work in `demo.py:_build_floor_scoped_candidate_mask(...)`
  - synchronous snapshot export materialization in `FloorAwareRoomSegmenter.get_vector_map_data(...)`, especially object export and anchor build
- Those costs sit behind method-defining boundaries:
  - current association/fusion semantics use cumulative retained history
  - lifecycle/publication currently observe authoritative exported vector maps, not a lighter maintained state

Conservative conclusion:

- Major additional speedups are still plausible.
- But the plausible high-impact paths mostly stop being "the same system" for the current paper.
- The only directions that preserve current semantics well are low-return or at best moderate-return engineering work.

Recommended decision:

- For the current paper/benchmark path: stay with the current system and stop here.
- For future work: if one branch is worth prototyping, make it a post-paper room/floor-local retained-history indexing branch aimed at Stage-5.
- If the real target is something like 10+ FPS on hard scenes, only a next-generation redesign is likely to matter, and it should be treated as a new system version.

Speedup labels used below:

- `low`: modest
- `medium`: moderate
- `high`: major

## Current adopted path and why it is already near local optimum

### Current adopted Stage-A path

Paper-safe main path in the repository:

- CLI and asset loading:
  - `stage_a_demo.py:_run_single_sequence(...)` (`stage_a_demo.py:184-279`)
  - model / CLIP / text-feature loading in `stage_a_demo.py:547-580`
- Main runtime loop:
  - `demo.run(...)` (`demo.py:728-2169`)
- Room/floor segmentation and export surface:
  - `FloorAwareRoomSegmenter.get_vector_map_data(...)` (`boxfusion/floor_aware_room_segmenter.py:468-814`)
  - `DynamicRoomSegmenter.perform_segmentation(...)`
- Snapshot/lifecycle/publication:
  - `demo.py:maybe_capture_demo_snapshot(...)` (`demo.py:908-961`)
  - `ClosedLoopDemoRecorder.record_snapshot(...)` (`boxfusion/stage_a_demo.py:799-893`)
  - `OnlineTopologyLifecycleManager.observe_export(...)` (`boxfusion/online_topology_lifecycle.py:304-710`)
  - `RoomScopedRuntimeManager.observe(...)` (`boxfusion/room_scoped_runtime.py:208-292`)
- Authoritative finalize-time committed/public bundle:
  - `ClosedLoopDemoRecorder.finalize(...)` (`boxfusion/stage_a_demo.py:1143-1605`)
  - `RoomScopedRuntimeManager.export_artifacts(...)` (`boxfusion/room_scoped_runtime.py:485-540`)

### Why the current path is already near its local optimum

Recent repository evidence already closed the safe, obvious wins:

- rich non-authoritative finalize/service/debug materialization is suppressible on the benchmark path
- Stage-5 candidate-index caching already removed the proven repeated floor-assignment bottleneck
- `00862` already showed that capture cadence is parity-relevant, so cadence changes are not "free" runtime wins

Measured residual bottlenecks on the adopted path:

- `00862` full-scene adopted run:
  - `duration_sec = 2175.213`
  - `average_fps = 3.447`
  - `stage5_total_sec mean = 1.381908`
  - `stage5_candidate_mask_prep_sec mean = 0.781667`
  - `stage5_candidate_floor_assignment_eval_sec mean = 0.763562`
  - `snapshot_export_sec mean = 2.176991` across `75` export calls
  - export-call means:
    - `object_export_sec = 0.942342`
    - `anchor_build_sec = 0.911741`
    - `scene_graph_build_sec = 0.090554`
    - `spatial_relations_sec = 0.135163`
  - `room_local_delta_export_used` rate is already about `94.7%`
  - `same_frame_full_export_reuse_count = 0`
- `00824` adopted run:
  - `average_fps = 5.488`
  - `stage5_total_sec mean = 0.467085`
  - `snapshot_export_sec mean = 0.900171`
- `00843` adopted run:
  - `average_fps = 5.972`
  - `stage5_total_sec mean = 0.247164`
  - `snapshot_export_sec mean = 0.460659`

What these numbers imply:

- finalize-only work is no longer the problem
- same-frame duplicate export is not the problem
- the remaining real headroom is tied to:
  - how the live retained object history is searched and filtered
  - how much authoritative snapshot export/lifecycle observation still requires full object/anchor/scene-graph materialization

That is why further large wins now start to look like redesign rather than optimization.

## Large-refactor candidate directions

## 1. Async split limited to non-authoritative materialization

### Technical description

Move only optional disk writes and non-authoritative artifact materialization farther off the synchronous path.

This means keeping the current authoritative committed/public path intact while further deferring:

- optional demo rendering
- optional scene graph PNGs
- rich service/debug reports
- extra JSON copies that are not part of the committed/public contract

### Exact code paths it would touch

- `demo.py:940-942`, `1270-1272`, `2095-2109`
- `boxfusion/stage_a_demo.py:1180-1286`
- `boxfusion/stage_a_demo.py:1317-1501`
- `boxfusion/room_scoped_runtime.py:485-540`
- `boxfusion/runtime_artifact_policy.py`

### Problem it is trying to solve

Reduce synchronous materialization overhead that is not method-defining.

### Expected benefit

`low`

Reason:

- the adopted benchmark path already suppresses the major rich service/debug artifacts
- recent audits already showed finalize export is tiny compared with end-to-end runtime

### Semantic and parity risk

- preserves current method semantics: `yes`
- preserves committed/public parity assumptions: `yes`

### Paper-comparability impact

Direct comparison fairness remains `high`.

### Engineering burden

`medium`

### Assessment

This is still the "same system," but it is not where major speedup will come from anymore. The remaining headroom is too small to justify a large refactor campaign.

### Recommendation

`do not pursue`

## 2. Stronger async split across the authoritative snapshot boundary

### Technical description

Replace the current synchronous "build full vector map, then let lifecycle/publication observe it" boundary with a lighter maintained runtime state, and materialize heavier exports later or in parallel.

In practice, that means changing the method boundary from:

- full vector-map export
- scene graph build
- spatial relations
- anchor build
- then lifecycle / commit observation

to something closer to:

- update a maintained room/floor/object state
- let lifecycle/publication observe that lighter authoritative state
- materialize the heavier vector-map / scene-graph / anchor surface later

### Exact code paths it would touch

- `demo.py:908-961`
- `demo.py:1925-1947`
- `demo.py:2080-2088`
- `boxfusion/floor_aware_room_segmenter.py:468-814`
- `boxfusion/floor_aware_room_segmenter.py:980-1067`
- `boxfusion/floor_aware_room_segmenter.py:2499-2746`
- `boxfusion/stage_a_demo.py:799-893`
- `boxfusion/online_topology_lifecycle.py:304-710`
- `boxfusion/room_scoped_runtime.py:208-292`
- `boxfusion/runtime_snapshot.py`
- `boxfusion/runtime_export_coordinator.py:248-260`

### Problem it is trying to solve

The current adopted path still pays for authoritative snapshot export synchronously, and lifecycle/publication consume that export directly.

That matters because on `00862` the export call itself still averages about:

- `object_export_sec = 0.942342`
- `anchor_build_sec = 0.911741`
- `scene_graph_build_sec = 0.090554`
- `spatial_relations_sec = 0.135163`

### Expected benefit

`high`

But only if the refactor is allowed to shrink what lifecycle/publication need from the synchronous path. A shallow async file-write split would not be enough.

### Semantic and parity risk

- preserves current method semantics: `partial`
- preserves committed/public parity assumptions: `partial`

Why only partial:

- current lifecycle/publication state is defined by what `observe_export(...)` sees from the exported vector map
- changing that observation surface changes when rooms appear stable, commit-ready, or published
- `00862` already showed that snapshot observation cadence changes committed/public parity

### Paper-comparability impact

Direct comparison fairness drops to `low`.

### Same-system assessment

For the current paper, this is no longer comfortably "the same system." It becomes a new authoritative runtime contract, even if the downstream output filenames stay similar.

### Engineering burden

`very_high`

### Assessment

This is one of the few refactor families with real major-speedup potential on the current codebase, but the speedup comes from changing the authoritative runtime boundary itself.

### Recommendation

`next-generation system redesign`

## 3. Restructure lifecycle observation so it no longer depends on full export materialization

### Technical description

Keep the current detector/segmentation stack, but teach lifecycle/publication to observe a smaller authoritative room-state surface rather than the fully materialized vector map.

This is narrower than a fully async architecture, but it still changes what "authoritative observation" means.

The repository already contains a shadow-only insertion point for this idea:

- `boxfusion/runtime_snapshot.py`
- `boxfusion/runtime_export_coordinator.py`

But those paths are explicitly non-authoritative today.

### Exact code paths it would touch

- `boxfusion/stage_a_demo.py:799-893`
- `boxfusion/online_topology_lifecycle.py:245-710`
- `boxfusion/room_scoped_runtime.py:208-292`
- `boxfusion/floor_aware_room_segmenter.py:468-814`
- `boxfusion/runtime_snapshot.py`
- `docs/runtime_sidecar_phase1_contract_split_note.md`

### Problem it is trying to solve

Remove object/anchor-heavy export work from the synchronous lifecycle/publication hot path without necessarily rewriting every exporter.

### Expected benefit

`medium`

Potentially `high` if the lighter observation surface truly replaces full export observation, but at that point it is effectively the prior redesign family.

### Semantic and parity risk

- preserves current method semantics: `partial`
- preserves committed/public parity assumptions: `partial`

### Paper-comparability impact

Direct comparison fairness is `low`.

### Same-system assessment

Not safely the same paper system. The current paper system says lifecycle/publication observe the exported vector map. Replacing that with a smaller runtime summary changes the method definition.

### Engineering burden

`high`

### Assessment

Worth understanding as a future architecture, but not as a current-paper optimization. The speedup is real only if the system stops requiring the same authoritative export surface at snapshot time.

### Recommendation

`next-generation system redesign`

## 4. Deeper retained-history / association / fusion redesign

### Technical description

Redesign the live retained-object memory and association path rather than continuing to scan a cumulative global store plus conservative ambiguity tails.

Today the current method still depends on:

- global `all_pred_box`
- cumulative `per_frame_ins`
- `BoxManager.fusion_list`
- stage-local floor/recency/nearby filtering before NMS/correspondence/fusion

The code itself already notes that the historical `per_frame_ins` store is cumulative and not window-pruned.

### Exact code paths it would touch

- `demo.py:_build_floor_scoped_candidate_mask(...)` (`demo.py:153-352`)
- Stage-5 call sites:
  - `demo.py:1495-1566`
  - `demo.py:1614-1676`
  - `demo.py:1714-1746`
  - `demo.py:1822`
- `boxfusion/instances.py:404-554`
- `boxfusion/box_manager.py:24-174`

### Problem it is trying to solve

The adopted path still spends large time on candidate selection and global retained-history handling:

- `00862` `stage5_candidate_floor_assignment_eval_sec mean = 0.763562`
- `00862` `stage5_candidate_mask_prep_sec mean = 0.781667`
- worst profiled `00862` Stage-5 frame:
  - `total_retained_object_count = 479`
  - `per_frame_ins_count = 760`

### Expected benefit

`high`

This is one of the few directions that could materially change runtime on large scenes.

### Semantic and parity risk

- preserves current method semantics: `partial`
- preserves committed/public parity assumptions: `partial`

Why partial rather than yes:

- a pure indexing rewrite could preserve candidate membership exactly in principle
- but the realistic larger wins usually come from stronger history windowing, locality assumptions, or fusion-policy changes
- once that happens, it is no longer the same association/fusion method

### Paper-comparability impact

Direct comparison fairness is `low`.

### Same-system assessment

If the redesign is ambitious enough to deliver a major win, it is effectively a new system version for paper purposes.

### Engineering burden

`very_high`

### Assessment

This family has real upside, but it is squarely in redesign territory. It should not be presented as "the frozen Stage-A system, only faster."

### Recommendation

`next-generation system redesign`

## 5. More aggressive room/floor-local indexing or memory partitioning

### Technical description

Introduce first-class live indices for retained objects by floor, room, recency, and active-locality so that Stage-5 no longer starts from a near-global retained pool on large scenes.

This is narrower than a full association redesign:

- keep the current method logic as much as possible
- change how the candidate set is retrieved
- try to preserve current masks exactly where possible

### Exact code paths it would touch

- `demo.py:_build_floor_scoped_candidate_mask(...)` (`demo.py:153-352`)
- Stage-5 call sites in `demo.py:1495-1566` and `demo.py:1714-1746`
- `boxfusion/box_manager.py:138-174`
- `boxfusion/instances.py:453-554`
- `boxfusion/floor_aware_room_segmenter.py:902-978`
- likely new shared indexing state near `demo.py` retained-object bookkeeping

### Problem it is trying to solve

Current caching only memoizes floor assignment. It does not turn the retained object store itself into a truly local indexed structure.

This is why the remaining Stage-5 residual is still large even after candidate-index caching.

### Expected benefit

`medium`

Conservative view:

- exact indexing alone could reduce mask-prep overhead noticeably
- but the evidence does not justify promising a major win unless the branch also adopts stronger locality assumptions

### Semantic and parity risk

- preserves current method semantics: `partial`
- preserves committed/public parity assumptions: `partial`

### Paper-comparability impact

Direct comparison fairness is `medium`.

### Same-system assessment

This is the best candidate for a post-paper branch that still tries to stay close to the current method. It is not fully safe to call "the same system," but it is closer than the broader redesign paths.

### Engineering burden

`high`

### Assessment

If the team wants exactly one future prototype, this is the most plausible one:

- it targets the largest remaining live hot bucket
- it can start with semantics-preserving retrieval/indexing
- it does not immediately require changing the detector or the public export contract

### Recommendation

`post-paper engineering branch`

## 6. Replace selected host-side helpers with faster/native components

### Technical description

Reimplement the hottest Python-side geometry/indexing helpers in a native or compiled layer while keeping the surrounding method semantics fixed.

Likely targets:

- Stage-5 candidate-mask floor assignment and locality filtering
- scene-graph spatial relation computation
- anchor candidate generation / validation / scoring

### Exact code paths it would touch

- `demo.py:_build_floor_scoped_candidate_mask(...)` (`demo.py:153-352`)
- `boxfusion/scene_graph_builder.py:695-724`
- `boxfusion/scene_graph_builder.py:740-1107`
- `boxfusion/floor_aware_room_segmenter.py:468-814`
- possibly `boxfusion/room_topology.py:1157-1163`

### Problem it is trying to solve

Several residual hotspots are Python-heavy loops over growing candidate or object sets.

### Expected benefit

`medium`

Conservative reasoning:

- it could help both Stage-5 and export materialization
- but it does not remove the structural need to do the work
- so it is more likely to produce a moderate win than a major one

### Semantic and parity risk

- preserves current method semantics: `yes`
- preserves committed/public parity assumptions: `yes`

That assumes an exact reimplementation.

### Paper-comparability impact

Direct comparison fairness is `high`.

### Same-system assessment

Yes, if the native implementation is truly behavior-preserving. The difficulty is engineering, not method identity.

### Engineering burden

`very_high`

### Assessment

This is the cleanest way to chase more speed without changing the method, but it is also expensive and difficult to validate. Current repository evidence does not justify opening a native-port project before paper closure.

### Recommendation

`post-paper engineering branch`

## 7. Segmentation backend rewrite or native acceleration

### Technical description

Rewrite or natively accelerate the room/floor segmentation internals.

### Exact code paths it would touch

- `demo.py:1186-1297`
- `boxfusion/floor_aware_room_segmenter.py`
- `boxfusion/dynamic_room_segmenter.py`

### Problem it is trying to solve

Reduce Stage-3 room segmentation cost.

### Expected benefit

`low`

Reason:

- on the adopted path, Stage-3 is no longer the dominant residual
- `00862` `stage3_total_sec mean = 0.217078`, well below the remaining Stage-5 residual and below snapshot export cost

### Semantic and parity risk

- preserves current method semantics: `partial`
- preserves committed/public parity assumptions: `partial`

### Paper-comparability impact

Direct comparison fairness is `medium`.

### Engineering burden

`high`

### Assessment

Even a successful rewrite would not target the biggest remaining bottlenecks first.

### Recommendation

`do not pursue`

## 8. Detector / semantic-stack replacement (hypothetical future branch only)

### Technical description

Replace the detector and/or semantic stack to reduce model time and possibly shift the entire runtime profile.

This is outside the allowed scope for the current paper path, but it is still worth classifying because it is one of the few obvious ways to reach much higher FPS.

### Exact code paths it would touch

- `stage_a_demo.py:547-580`
- `demo.py:1108-1129`
- `demo.py:1428-1441`
- `demo.py:1831-1848`

### Problem it is trying to solve

The current stack still includes nontrivial model cost:

- `00862` profiled-frame means:
  - `model_bbox_inference_sec = 0.345801`
  - `clip_classification_sec = 0.034186`

### Expected benefit

`high`

### Semantic and parity risk

- preserves current method semantics: `no`
- preserves committed/public parity assumptions: `no`

### Paper-comparability impact

Direct comparison fairness is `low`.

### Same-system assessment

This is unambiguously a new system version, not the current paper system.

### Engineering burden

`high`

### Assessment

This can matter for a future runtime-focused branch, but it should not be mixed into the current benchmark/paper optimization story.

### Recommendation

`next-generation system redesign`

## Which directions still preserve the current paper system

Only a small subset stays clearly inside the current paper system definition:

- async split limited to non-authoritative materialization
- exact faster/native reimplementation of hot helpers

Even for those, the expected gain is not major:

- the first is low-return because the adopted path already suppresses most rich artifacts
- the second is expensive engineering and more likely to deliver a moderate gain than a major one

The important conclusion is:

- none of the high-speedup directions clearly preserve the current frozen paper system

## Which directions effectively create a new system version

The following should be treated as new-system work, not current-paper optimization:

- authoritative snapshot-surface split
- lifecycle observation rewrite around a lighter maintained state
- deeper retained-history / association / fusion redesign
- detector / semantic-stack replacement

These all change one of the current method-defining boundaries:

- what lifecycle/publication observes
- how retained history participates in association/fusion
- or what learned model stack defines detections/semantics

## Which directions are worth future exploration, and which should be rejected now

### Worth future exploration

Most plausible future prototype:

- room/floor-local retained-history indexing or memory partitioning
  - best balance of plausible benefit and bounded scope
  - should be treated as a post-paper engineering branch, not a current-paper optimization

Possible but expensive post-paper engineering:

- exact native/compiled hot-helper replacement
  - only if the team wants to keep the current method intact and can afford high validation cost

### Reject now

Reject for the current paper path:

- broader async/export redesign across the authoritative snapshot boundary
- lifecycle observation rewrite
- deeper association/fusion redesign
- detector/semantic replacement

Reason:

- these are not safely the same system anymore
- their evaluation burden is much larger than the current paper needs

Reject as not worth their ROI even post-paper unless priorities change:

- standalone segmentation rewrite
- more work on finalize-only materialization

## If the team’s true target were something like 10+ FPS, which refactor paths would even plausibly matter, and would that still be the same system?

Short answer:

- 10+ FPS on hard scenes is not a realistic target for the current frozen system definition
- the paths that might plausibly matter are next-generation paths

Why:

- current `00862` adopted runtime is about `3.447` FPS
- the hardest remaining structural costs already include:
  - `stage5_total_sec mean = 1.381908` on profiled frames
  - `snapshot_export_sec mean = 2.176991` per export call
  - `model_bbox_inference_sec mean = 0.345801` on profiled frames
- getting from roughly `3.4` FPS to `10+` FPS needs far more than micro-cleanups or finalize suppression

What would plausibly matter:

- a new authoritative runtime state that does not require full export materialization for lifecycle/publication
- a redesigned retained-history / association / fusion memory model
- possibly a detector / semantic-stack replacement

What would not plausibly be enough on its own:

- finalize-only cleanup
- more debug-artifact suppression
- minor helper cleanup
- segmentation-only rewrite

Would it still be the same system?

- almost certainly no

The specific reason is that the 10+ FPS-relevant paths are exactly the paths that change:

- the authoritative observation surface
- the retained-history behavior
- or the model stack

That crosses the line from "optimized implementation" to "new system version."

## Recommended decision

Primary decision:

- stay with the current system and stop here for paper/benchmark purposes

One future branch worth prototyping:

- a post-paper room/floor-local retained-history indexing branch aimed at Stage-5 candidate selection

What not to do now:

- do not reopen a broad current-paper optimization campaign
- do not present cadence changes or lifecycle-surface changes as "the same system, only faster"
- do not start a detector/semantic replacement branch under the current paper comparison framing

Plain-language summary:

- Major speedups are still plausible, but mostly only through redesign.
- The redesigns that plausibly matter would usually not still be the same system for the current paper.
- The team should treat such work as post-paper engineering at best, and often as next-generation system work.
