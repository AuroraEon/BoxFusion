# Backend Eval v0.1

## Scope

This backend flow is for the floor-aware, room-centric world-model / semantic-SLAM backend.

It preserves the current project boundaries:

- World Graph stays the entity truth layer.
- Room-centric Queryable Topology stays the derived query / routing layer.
- Graph search stays on the existing NetworkX-backed topology code.
- Query API and closed-loop executor stay unchanged.
- Natural-language-facing layers remain constrained to intent / slot extraction, tool selection, and response organization.

This flow is not an open-ended LLM navigation agent benchmark.

## Output Root Policy

Canonical final root:

- `world_model_backend_outputs_v0_2_final/scenes/<sequence_name>/`
- `world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1/`

Previous root:

- `world_model_backend_outputs_v0_1/`

Legacy root:

- `stage_a_outputs_vt_fallback_v01_rerun2/`

Legacy policy:

- not the normal output root
- not required for normal backend evaluation
- allowed only as a migration-only read-only fallback when explicitly requested

Going forward, `world_model_backend_outputs_v0_2_final/` is the canonical project root for this stage.

## Artifact Tiers

### Tier 1: core backend artifacts

These are the only per-scene artifacts the refreshed backend benchmark flow should rely on:

- `manifest.json`
- `logs/summary.json`
- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/vertical_transition_evidence.json`
- `logs/floor_diagnostics_summary.json`
- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`

Tier 1 is used for:

- scene registry refresh
- task-sheet refresh
- backend evaluation
- aggregate backend report rendering
- runtime / compactness summaries

Tier 1 compactness now tracks backend bytes separately from optional demo bytes.

### Tier 2: optional demo artifacts

These are allowed, but backend evaluation must not require them:

- `logs/topology_v0_1.graphml`
- `logs/timeline.json`
- `logs/timeline.csv`
- `logs/room_segmentation_diagnostics.json`
- `final/*.png`
- `final/*.mp4`
- `report.md`
- showcase bundles, appendix visuals, spotlights, rendered frames, debug snapshots

## Core-Only Rerun Mode

`stage_a_demo.py` now supports a backend-oriented `--core-only` mode.

Core-only keeps the Tier 1 backend artifacts above and suppresses the main Tier 2 demo/showcase outputs by default:

- no final MP4
- no final PNGs
- no event spotlights
- no rendered/rgb frame bundles
- no `report.md`
- no debug-room snapshot dumps

This mode is intended for canonical backend refreshes under `world_model_backend_outputs_v0_2_final/scenes/`.

## Runtime-Growth Profiling

Each refreshed scene can now write:

- `logs/runtime_growth_profile.csv`
- `logs/runtime_growth_profile.json`

The profile records lightweight periodic samples for:

- frame index / cumulative processed frames
- room, object, anchor, node, and edge counts when available
- model/bbox inference time
- topology / room-segmentation time
- feature extraction / BoxFusion time
- total step time

`logs/summary.json` and `manifest.json` also carry a compact runtime-growth summary including early/mid/late averages, growth ratios, and a runtime-risk flag.

## Pruning

Use `stage_a_eval/prune_scene_outputs.py` to prune selected scene folders down to Tier 1 only.

Policy:

- dry-run by default
- `--apply` is required to actually delete files
- Tier 1 backend artifacts are preserved
- Tier 2 demo/showcase directories and files are safe prune targets after a successful rerun
- prune is enough when the goal is only to shrink an existing output tree
- rerun is required when new artifacts must be generated, especially `runtime_growth_profile.csv` and `runtime_growth_profile.json`
- `stage_a_eval/scene_retention_plan.json` is the machine-readable source for final retention defaults

Recommended retention split:

- showcase full-output scenes: `00873-bxsVRursffK`, `00843-DYehNKdT76V`, `00829-QaLdnwvtxbs`
- core-only scenes: `00824-Dd4bFSTQ8gi`, `00861-GLAQ4DNUx5U`, `00877-4ok3usBNeis`, `00890-6s7QHgap2fW`
- difficult core-only coverage scene: `00862-LT9Jq6dN3Ea`

## Recommended Commands

Canonical full-length core-only rerun of one scene:

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

Canonical full-length core-only batch rerun of all 8 active scenes:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seqs 00824-Dd4bFSTQ8gi 00829-QaLdnwvtxbs 00843-DYehNKdT76V 00861-GLAQ4DNUx5U 00862-LT9Jq6dN3Ea 00873-bxsVRursffK 00877-4ok3usBNeis 00890-6s7QHgap2fW \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --aggregate-name stage_a_multi_sequence
```

Showcase-only full-output rerun:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seqs 00873-bxsVRursffK 00843-DYehNKdT76V 00829-QaLdnwvtxbs \
  --output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --room-seg-interval 100 \
  --video-fps 12 \
  --full-rgb-replay \
  --aggregate-name stage_a_multi_sequence
```

Refresh registry, task sheet, and backend evaluation from the final root:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/build_scene_registry.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --registry-out ./stage_a_eval/scene_registry.json \
  --write-manifests
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/build_tasks_hierarchical_overlap.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --tasks-out ./stage_a_eval/backend_tasks_v0_1.jsonl
```

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/run_backend_eval.py \
  --registry ./stage_a_eval/scene_registry.json \
  --tasks ./stage_a_eval/backend_tasks_v0_1.jsonl \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --report-root ./world_model_backend_outputs_v0_2_final/eval/backend_eval_v0_1
```

Dry-run prune for non-showcase scenes:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/prune_scene_outputs.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --retention-plan ./stage_a_eval/scene_retention_plan.json \
  --plan-retention-tiers core_only difficult_core_only \
  --preserve-keep-tier2 \
  --report-json ./world_model_backend_outputs_v0_2_final/docs/prune_non_showcase_dry_run.json
```

Apply prune for non-showcase scenes:

```bash
/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/prune_scene_outputs.py \
  --scene-output-root ./world_model_backend_outputs_v0_2_final/scenes \
  --retention-plan ./stage_a_eval/scene_retention_plan.json \
  --plan-retention-tiers core_only difficult_core_only \
  --preserve-keep-tier2 \
  --apply \
  --report-json ./world_model_backend_outputs_v0_2_final/docs/prune_non_showcase_apply.json
```

## Formal Task Schema

The generated backend task sheet now carries a compact benchmark-oriented schema:

- `task_id`
- `split`
- `scene_id`
- `sequence_name`
- `task_family`
- `task_type`
- `hierarchy_level`
- `source_room_id`
- `target_floor_id`
- `target_room_id`
- `target_object_id`
- `target_object_label`
- `target_anchor_id`
- `target_anchor_label`
- `needs_route`
- `resolve_only`
- `target_granularity`
- `expected_floor_sensitive`
- `expected_room_sensitive`
- `expected_success`
- `policy`
- `hov_sg_overlap_split`
- `scene_role`
- `notes`

Compatibility fields such as `target_spec`, `route_policy`, `expected_target_room`, and `expected_floor_id` are still emitted so the existing evaluator and executor interfaces do not need a rewrite.

## Active Scene Priority

Priority rerun order:

1. `00843-DYehNKdT76V`
2. `00824-Dd4bFSTQ8gi`

Current active scene set:

- `00824-Dd4bFSTQ8gi`
- `00829-QaLdnwvtxbs`
- `00843-DYehNKdT76V`
- `00861-GLAQ4DNUx5U`
- `00862-LT9Jq6dN3Ea`
- `00873-bxsVRursffK`
- `00877-4ok3usBNeis`
- `00890-6s7QHgap2fW`

`00847-bCPU9suPUw9` is no longer in the active universe.

## Legacy / Secondary Scripts

Primary backend flow:

- `stage_a_demo.py`
- `stage_a_eval/build_scene_registry.py`
- `stage_a_eval/build_tasks_hierarchical_overlap.py`
- `stage_a_eval/run_backend_eval.py`

Secondary or demo-oriented scripts may still be useful, but they should not define the backend benchmark:

- `stage_a_end_to_end_vln_demo.py`
- `stage_a_minimal_vln_closed_loop.py`
- `stage_a_vln_tool_use_demo.py`
- `stage_a_multifloor_query_acceptance.py`
- `boxfusion/world_model_eval.py`

Treat those as showcase / secondary evaluation utilities rather than the main backend flow.
