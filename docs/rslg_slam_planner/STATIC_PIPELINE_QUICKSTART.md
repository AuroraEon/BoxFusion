# RSLG-SLAM Static Pipeline Quickstart

This quickstart runs the current frozen canonical handoff path for scene
`00843-DYehNKdT76V`. It writes only task-local outputs.

```bash
cd /home/ws/workspace/BoxFusion
TASK_DIR=stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task54_main_chain_handoff_pack
CANONICAL_ROOT=stage_outputs/rslg_slam/00843-DYehNKdT76V/canonical
PY=/home/ws/miniconda3/envs/boxfusion/bin/python
```

## 1. Optional Layer 0 Input Manifest Smoke

This records provenance only. It does not run inference or rebuild the world
model.

```bash
mkdir -p "$TASK_DIR/demo_pack/layer0_manifest_smoke"
"$PY" -m tools.rslg_pipeline.build_input_manifest \
  --output-json "$TASK_DIR/demo_pack/layer0_manifest_smoke/00843_layer0_input_manifest_smoke.json" \
  --scene-id 00843-DYehNKdT76V \
  --dataset-name hm3d \
  --sequence-id 00843-DYehNKdT76V \
  --rgb-root /home/ws/data/00843-DYehNKdT76V \
  --depth-root /home/ws/data/00843-DYehNKdT76V \
  --pose-root /home/ws/data/00843-DYehNKdT76V \
  --config-path config/hm3d.yaml \
  --model-checkpoint-path models/cutr_rgbd.pth \
  --clip-checkpoint-path models/ViT-B-32/open_clip_pytorch_model.bin \
  --class-text-path data/panoptic_categories_nomerge.txt \
  --text-features-path data/class_features_small.pt \
  --canonical-root "$CANONICAL_ROOT" \
  --provenance-notes "Task-local Layer 0 manifest smoke only; no inference." \
  --no-canonical-write
```

## 2. Generate RouteResults For The Current QuerySet

```bash
mkdir -p "$TASK_DIR/demo_pack/route_results"
"$PY" -m tools.rslg_pipeline.batch_plan_query_static \
  --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \
  --canonical-root "$CANONICAL_ROOT" \
  --output-dir "$TASK_DIR/demo_pack/route_results" \
  --no-canonical-write
```

## 3. Validate RouteResults

```bash
find "$TASK_DIR/demo_pack/route_results" -name '*_route_result.json' -print0 \
  | xargs -0 -I{} "$PY" tools/rslg_pipeline/audits/validate_route_result.py \
      --route-result-json {}
```

## 4. Export RouteResult-Derived Adapter Inputs

```bash
mkdir -p "$TASK_DIR/demo_pack/runtime_adapter_inputs"
"$PY" -m tools.rslg_pipeline.export_route_result_runtime_inputs \
  --route-results-dir "$TASK_DIR/demo_pack/route_results" \
  --output-dir "$TASK_DIR/demo_pack/runtime_adapter_inputs" \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --no-canonical-write
```

## 5. Validate Adapter Inputs

```bash
"$PY" tools/rslg_pipeline/audits/validate_route_result_runtime_adapters.py \
  --adapter-root "$TASK_DIR/demo_pack/runtime_adapter_inputs"
```

## 6. Compile Pipeline Tools

```bash
"$PY" -m compileall tools/rslg_pipeline
```

## 7. Optional PID Runtime Replay

After exporting adapter inputs, a task-local PID replay can validate same-floor
waypoint tracking and connector handoff events:

```bash
mkdir -p "$TASK_DIR/demo_pack/pid_replay"
"$PY" -m tools.rslg_pipeline.runtime.replay_pid_runtime_input \
  --pid-inputs-dir "$TASK_DIR/demo_pack/runtime_adapter_inputs/pid_follower_inputs" \
  --route-results-dir "$TASK_DIR/demo_pack/route_results" \
  --z-aware-inputs-dir "$TASK_DIR/demo_pack/runtime_adapter_inputs/z_aware_overlay_inputs" \
  --output-dir "$TASK_DIR/demo_pack/pid_replay" \
  --dt 0.1 \
  --max-linear-velocity 0.25 \
  --max-angular-velocity 0.8 \
  --linear-gain 0.8 \
  --angular-gain 1.5 \
  --waypoint-tolerance 0.12 \
  --yaw-tolerance 0.25 \
  --timeout-sec 240 \
  --stuck-window-sec 8 \
  --stuck-progress-epsilon 0.02 \
  --robot-radius 0.18 \
  --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \
  --title "RSLG-SLAM PID Runtime Replay"
```

See `docs/rslg_slam_planner/PID_RUNTIME_REPLAY_QUICKSTART.md` for interpretation
and claim boundaries.

This quickstart does not rerun Stage-A, raw RGB-D inference, Gazebo, RViz, ROS
nodes, Habitat, GPU jobs, Nav2, AMCL, `map_server`, or ROS lifecycle runtime
components.
