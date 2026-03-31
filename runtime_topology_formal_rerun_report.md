# Runtime Topology Formal Rerun Report

## 1. Executive summary

- All three requested representative sequences completed as true reruns with the fixed instrumentation pipeline:
  - `00829-QaLdnwvtxbs`: `1804` processed frames
  - `00843-DYehNKdT76V`: `2710` processed frames
  - `00862-LT9Jq6dN3Ea`: `7498` processed frames
- The runtime instrumentation outputs are now trustworthy for this representative set. For every scene, all required runtime-instrumentation artifacts exist, `segmentation_runs.csv` is non-empty, and `summary.json` reports a non-zero `segmentation_run_count`.
- Aggregate artifacts were refreshed from the fresh reruns, not repaired scene artifacts. The combined row counts are internally consistent with the per-scene reruns.
- `blocker_summary.json` refreshed correctly. It now reports `status: cleared_no_active_blocker` with `generated_at: 2026-03-31T11:10:23+08:00`, superseding the earlier `2026-03-31T09:24:14+08:00` status instead of remaining stale.
- One nuance: `00862-LT9Jq6dN3Ea` logged an unsuccessful frame-0 scheduled segmentation diagnostic, but the rerun itself completed successfully, later segmentation runs succeeded, and the final end-of-sequence flush succeeded for the relevant floors.

## 2. Per-scene rerun status

- `00829-QaLdnwvtxbs`
  - Processed frame count: `1804`
  - Required `runtime_instrumentation` outputs: present
  - `segmentation_runs.csv`: non-empty (`20` data rows)
  - `summary.json segmentation_run_count`: `20`
  - Worst stage-5 frame: `1700` (`3.958 s`)
  - Worst stage-3 frame: `1600` (`1.927 s`)

- `00843-DYehNKdT76V`
  - Processed frame count: `2710`
  - Required `runtime_instrumentation` outputs: present
  - `segmentation_runs.csv`: non-empty (`29` data rows)
  - `summary.json segmentation_run_count`: `29`
  - Worst stage-5 frame: `2650` (`4.548 s`)
  - Worst stage-3 frame: `2700` (`2.736 s`)

- `00862-LT9Jq6dN3Ea`
  - Processed frame count: `7498`
  - Required `runtime_instrumentation` outputs: present
  - `segmentation_runs.csv`: non-empty (`76` data rows)
  - `summary.json segmentation_run_count`: `76`
  - Worst stage-5 frame: `7425` (`62.920 s`)
  - Worst stage-3 frame: `7300` (`13.325 s`)
  - Note: the first scheduled segmentation attempt at frame `0` reported `success=False`, but later scheduled runs and the end-of-sequence flush succeeded, so the final rerun evidence is still usable and internally consistent.

## 3. Aggregate validation

- `combined_per_profiled_frame.csv` is non-empty: `485` data rows
- `combined_segmentation_runs.csv` is non-empty: `125` data rows
- `combined_history_scope.csv` is non-empty: `1141` data rows
- `combined_vector_map_export_calls.csv` is non-empty: `601` data rows
- `aggregate_summary.json` is refreshed and populated
- `blocker_summary.json` is refreshed and no longer stale
  - New status: `cleared_no_active_blocker`
  - New timestamp: `2026-03-31T11:10:23+08:00`
  - Superseded prior timestamp: `2026-03-31T09:24:14+08:00`
- Required plots are present and non-empty:
  - `timing_breakdown_vs_frame.png`
  - `cardinality_growth_vs_frame.png`
  - `cost_vs_scale.png`
  - `export_duplication_markers.png`
- Internal consistency checks passed:
  - Profiled-frame rows add up exactly: `74 + 110 + 301 = 485`
  - Segmentation-run rows add up exactly: `20 + 29 + 76 = 125`
  - History-scope rows add up exactly: `190 + 211 + 740 = 1141`
  - Vector-export rows add up exactly: `93 + 138 + 370 = 601`

## 4. Final trusted runtime conclusions

- Stage 5 is still substantially dominated by export/rebuild work.
  - Mean stage-5 export share remains high in all three fresh reruns: `66.95%`, `77.34%`, and `46.92%`.
  - Mean stage-5 BoxFusion share is much smaller: `3.02%`, `3.41%`, and `1.27%`.
  - At each scene’s worst stage-5 frame, export exceeds BoxFusion by a wide margin:
    - `00829`: export `1.601 s` vs BoxFusion `0.732 s`
    - `00843`: export `2.347 s` vs BoxFusion `0.165 s`
    - `00862`: export `13.035 s` vs BoxFusion `0.014 s`

- Stage 3 is still largely export-heavy relative to segmentation core.
  - Mean stage-3 export share is `75.52%`, `72.09%`, and `84.32%`.
  - Mean stage-3 segmentation share is only `16.76%`, `19.28%`, and `11.36%`.
  - At each worst stage-3 frame, post-segmentation export dominates segmentation compute:
    - `00829`: export `1.725 s` vs segmentation `0.116 s`
    - `00843`: export `2.343 s` vs segmentation `0.261 s`
    - `00862`: export `12.274 s` vs segmentation `0.380 s`

- Duplicate export frames still exist in the fresh reruns.
  - Per-scene duplicate-export frame counts remain substantial: `19`, `28`, and `73`.
  - The refreshed aggregate summary still reports duplicate-export behavior, and the refreshed `export_duplication_markers.png` was generated from the fresh rerun outputs.

- Object-side scope is still overwhelmingly global or near-global.
  - Aggregate history-scope counts are:
    - `global_retained_history`: `772`
    - `near_global_retained_history`: `366`
    - `current_frame_only`: `3`
  - That means `1138 / 1141` recorded scope decisions are global or near-global, not local.

- The advisor’s local-vs-global suspicion remains supported.
  - The history-scope evidence is overwhelmingly global/near-global.
  - Stage-5 cost still tracks retained object scale and per-frame instance scale:
    - `corr(stage5, global boxes)`: `0.887`, `0.667`, `0.733`
    - `corr(stage5, per-frame instances)`: `0.932`, `0.770`, `0.809`
  - The fresh reruns therefore continue to support the conclusion that export/rebuild overhead and broad retained-history scope, not the core segmentation kernel, are the main runtime drivers.

## 5. Recommended next technical step

- Primary next step: eliminate same-frame duplicate export rebuilds.
  - Rationale: this is the strongest low-risk next move supported by the fresh reruns. Duplicate export frames are still present in all three representative sequences, stage 3 is dominated by post-segmentation export, and stage 5 is also export-heavy. Removing same-frame duplicate export rebuilds directly targets a now-measured inefficiency without changing mapper semantics or redesigning the backend.

## 6. Reproducibility

- Exact rerun commands:

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

/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --output-root ./world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation \
  --report-out ./runtime_topology_instrumented_report.md \
  --sequence-ids 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00862-LT9Jq6dN3Ea
```

- Exact output locations:
  - Per-scene outputs: `world_model_backend_outputs_v0_2_final/scenes/<sequence>/logs/runtime_instrumentation/`
  - Aggregate outputs: `world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation/`
  - This report: `runtime_topology_formal_rerun_report.md`

- Environment issues encountered:
  - No environment or dataset issue blocked the three requested reruns.
  - HM3D raw data was available for all three representative sequences.
  - A minor shell-path issue affected only an ad hoc reporting helper (`python` alias not present); reruns and aggregate analysis were executed successfully with the explicit BoxFusion conda interpreter above.
