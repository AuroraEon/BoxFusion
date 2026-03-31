# Runtime / Topology Instrumented Report

## 1. Executive summary

- Across the representative scenes 00829-QaLdnwvtxbs, 00843-DYehNKdT76V, 00862-LT9Jq6dN3Ea, the dominant late-frame costs remain stage 5 object-side processing and stage 3 topology/segmentation refresh, with the heaviest object-side case in `00862-LT9Jq6dN3Ea` and the heaviest topology case in `00862-LT9Jq6dN3Ea`.
- The advisor’s local-vs-global suspicion is supported by the new scope logs: spatial association, small-object correspondence, and BoxFusion all report `global_retained_history` or `near_global_retained_history`, with no room-scoped or recent-window filter active in the current path.
- The previous coarse timing labels were misleading. Stage 5 includes non-trivial snapshot-triggered full export rebuilds, and stage 3 includes both segmentation work and a second full export rebuild on refresh frames.

## 2. Stage-5 object-side diagnosis

- `00829-QaLdnwvtxbs`: late stage-5 avg=2.307 s, corr(stage5, global_boxes)=0.887, corr(stage5, per_frame_ins)=0.932, mean snapshot-export share=0.670, mean BoxFusion share=0.030, worst frame=1700.
- `00843-DYehNKdT76V`: late stage-5 avg=2.392 s, corr(stage5, global_boxes)=0.667, corr(stage5, per_frame_ins)=0.770, mean snapshot-export share=0.773, mean BoxFusion share=0.034, worst frame=2650.
- `00862-LT9Jq6dN3Ea`: late stage-5 avg=40.562 s, corr(stage5, global_boxes)=0.733, corr(stage5, per_frame_ins)=0.809, mean snapshot-export share=0.469, mean BoxFusion share=0.013, worst frame=7425.
- Evidence from `history_scope.csv` shows current-frame object processing compares against retained history that is effectively scene-global. No instrumented step emitted a room-scoped or recent-window-scoped candidate pool.
- `snapshot_export_sec` and `vector_map_export_sec` confirm that stage 5 is not purely CLIP plus BoxFusion; it can include a full `get_vector_map_data(...)` rebuild when snapshots are captured.

## 3. Stage-3 room/topology-side diagnosis

- `00829-QaLdnwvtxbs`: late stage-3 avg=0.385 s, corr(stage3, merged_points)=0.951, corr(stage3, grid_area)=0.648, mean export share=0.755, mean segmentation share=0.168, worst frame=1600.
- `00843-DYehNKdT76V`: late stage-3 avg=0.547 s, corr(stage3, merged_points)=0.174, corr(stage3, grid_area)=-0.387, mean export share=0.721, mean segmentation share=0.193, worst frame=2700.
- `00862-LT9Jq6dN3Ea`: late stage-3 avg=1.277 s, corr(stage3, merged_points)=0.314, corr(stage3, grid_area)=0.448, mean export share=0.843, mean segmentation share=0.114, worst frame=7300.
- The fine-grained stage-3 timers separate floor merge/downsample, histogram/state construction, tracking, gateway extraction, and post-segmentation export. This makes it visible when the cost is dominated by global floor-cloud growth versus export rebuild work.
- The new segmentation-scale columns (`merged_point_count_after_downsample`, `grid_area`, `wall_slice_point_count`, `full_slice_point_count`) show that refresh cost grows with accumulated floor state rather than being bounded by a current-room subset.

## 4. Duplicate-work diagnosis

- Duplicate export frames observed across the representative set: 120.
- `00829-QaLdnwvtxbs` duplicate-export frames: [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800].
- `00843-DYehNKdT76V` duplicate-export frames: [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700].
- `00862-LT9Jq6dN3Ea` duplicate-export frames: [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 6000, 6100, 6200, 6300, 6400, 6500, 6600, 6700, 6800, 6900, 7000, 7100, 7200, 7300, 7400].
- `vector_map_export_calls.csv` records the exact `call_context` and `call_index_within_frame`, so same-frame rebuilds are explicit instead of being hidden inside a coarse bucket.
- The most common duplication pattern is a segmentation refresh export followed by a snapshot-triggered export on the same frame.

## 5. Structural interpretation

- Timer-label issue: the old `feature_boxfusion_sec` bucket conflated CLIP, spatial/correspondence association, BoxFusion optimization, snapshot capture, and full vector-map export.
- Duplicate work: segmentation refresh frames can rebuild vector-map export once in stage 3 and again in stage 5 snapshot capture on the same frame.
- Fundamentally global-growing work: retained object history, cumulative `per_frame_ins`, floor-level merged point clouds, and floor grids all grow with scene history in the current implementation.
- Architecture-level redesign later: room-scoped or recent-window object fusion, explicit current-room state, room-leave signals, and online topology delta updates are still absent rather than merely untimed.

## 6. Advisor alignment

- Current/local vs global/history separation: not yet aligned. Instrumented scope logs show object-side association and fusion remain tied to retained global history.
- Relevant-subset fusion vs near-global fusion: not aligned. The current path does not apply room-scoped, floor-scoped, or recent-window pruning before stage-5 candidate scans.
- Event-driven incremental topology vs interval/post-hoc topology: partially aligned at the observation level, but not at export/update level. Floors observe frames online, yet room segmentation and topology export remain scheduled refresh plus full rebuild rather than incremental triggers plus deltas.

## 7. Recommended next actions

- Immediate low-risk cleanup: keep the new fine-grained timers, preserve the coarse totals only as legacy convenience, and stop treating `feature_boxfusion_sec` as CLIP/BoxFusion-only in reports.
- Immediate low-risk cleanup: use the duplicate-export log to guard or memoize same-frame `get_vector_map_data(...)` rebuilds before deeper algorithm changes.
- Medium instrumentation-informed fixes: bound stage-5 candidate pools by floor or recent-window filters first, then compare late-frame curves against the new baseline.
- Medium instrumentation-informed fixes: split topology refresh into floor-merge, segmentation, and export phases in scheduling policy so we can independently rate-limit export rebuilds.
- Later architecture changes: introduce explicit active-room state, room-completion/leaving signals, and online topology delta structures before attempting a SLAM-style backend redesign.

## Reproducible commands

Instrumentation reruns write per-scene artifacts under `world_model_backend_outputs_v0_2_final/scenes/<sequence>/logs/runtime_instrumentation/` and aggregate outputs under `world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation`.

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00829-QaLdnwvtxbs \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --output-root ./world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation \
  --report-out ./runtime_topology_instrumented_report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

