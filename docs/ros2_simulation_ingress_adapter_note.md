# ROS 2 Simulation Ingress Adapter Note

## 2026-04-15 HM3D Real Backend Handoff Update

The simulation ingress real-backend bridge now has an explicit producer handoff selector:

- `real_backend_handoff_format=hm3d`: preferred path for this phase.
- `real_backend_handoff_format=ca1m`: retained rollback path for the previously validated temporary bridge.

The backend mode surface stays narrow:

- `backend_mode=stub`: preserves the original minimal committed/public bundle writer.
- `backend_mode=real`: materializes a file-backed Stage A input sequence, invokes Stage A in `--core-only` mode, and refreshes the existing coordinator from the real Stage A scene root.

The preferred HM3D handoff shape is:

```text
<output_root>/stage_a_sequences/<sequence_name>/logs/simulation_ingress_frames.jsonl
        |
        v
<output_root>/stage_a_backend_inputs/<sequence_name>/
  rgb/<idx>.png
  depth/<idx>.png
  pose/<idx>.txt
  boxfusion_hm3d_config.yaml
  simulation_ingress_backend_input_manifest.json
        |
        v
stage_a_demo.py hm3d --core-only --config boxfusion_hm3d_config.yaml
        |
        v
<output_root>/stage_a_real_backend_sequences/<sequence_name>/
        |
        v
BoxFusionRuntimeExportCoordinator
        |
        v
<coordination_root>/latest
```

The HM3D config is generated at the materialization boundary and points `data.datadir` to the generated backend input root. It includes the static intrinsics expected by `HM3DDataset`: `cam.H`, `cam.W`, `cam.fx`, `cam.fy`, `cam.cx`, `cam.cy`, and `cam.png_depth_scale`.

The pose conversion is explicit and intentionally local to the handoff writer. The captured simulation pose is treated with the same narrow assumption as the previous CA1M bridge: camera-to-world, OpenCV camera axes, Z-up world. Because `HM3DDataset.load_poses()` applies:

```text
opencv_zup_pose = T_Yup2Zup @ pose_txt @ C_habitat2opencv
```

the writer stores:

```text
pose_txt = inv(T_Yup2Zup) @ simulation_opencv_zup_pose @ inv(C_habitat2opencv)
```

This produces HM3D/Habitat-shaped `pose/*.txt` without changing the HM3D dataset loader, Stage A mainline, coordinator, query server, publication diagnostics server, or sidecar behavior.

The CA1M rollback shape remains available only by selecting `real_backend_handoff_format=ca1m`. It still writes `rgb/`, `depth/`, `all_poses.npy`, `K_rgb.txt`, `K_depth.txt`, and `boxfusion_ca1m_config.yaml`, then invokes `stage_a_demo.py CA1M --core-only`.

Reused unchanged downstream:

- snapshot-style simulation ingestion and capture logs
- `BoxFusionRuntimeExportCoordinator`
- `coordination/latest`, `latest_manifest.json`, and `latest_export.json`
- committed/public query backend
- debug publication diagnostics backend
- public topology meaning: committed/published only
- sidecar target and sidecar-disabled simulation-ingress coordinator refresh

Intentionally deferred:

- sidecar expansion
- multi-GPU work
- detector / CLIP changes
- broad runtime optimization
- maintained candidate indices
- richer topology/vector-map migration
- public topology expansion
- online publication redesign
- navigation/control/VLN logic
- Gazebo-specific topology semantics

### HM3D Validation Report

Environment used:

- Python: `/home/ami/miniconda3/envs/boxfusion/bin/python`
- Python version: `3.10.20`
- `CUDA_HOME=/usr/local/cuda`
- `PATH` begins with `/usr/local/cuda/bin`
- `torch 2.5.1+cu124`
- CUDA available: true
- CUDA device count: 8

Dependency checks from the validation report:

- `torch`, `open_clip`, `cv2`, `yaml`, `numpy`, and `PIL`: available
- `stage_a_demo.py`: present
- `./models/cutr_rgbd.pth`: present
- `./models/ViT-B-32/open_clip_pytorch_model.bin`: present
- `./data/class_features_small.pt`: present

HM3D producer result:

- report: `runtime_export_validation/ros2_sim_ingress_hm3d_backend_report.json`
- input root: `runtime_export_validation/ros2_sim_ingress_hm3d_backend/stage_a_backend_inputs/gazebo_sim_hm3d_backend`
- generated `rgb/`, `depth/`, `pose/`, and `boxfusion_hm3d_config.yaml`
- invoked `stage_a_demo.py hm3d --core-only`
- producer return code: `0`
- real backend scene root: `runtime_export_validation/ros2_sim_ingress_hm3d_backend/stage_a_real_backend_sequences/gazebo_sim_hm3d_backend`
- backend topology exists: true
- backend lifecycle exists: true
- fallback to CA1M needed: no

Coordinator refresh result:

- `runtime_export_validation/ros2_sim_ingress_hm3d_backend/coordination/latest` exists
- `latest_manifest.json` exists
- `latest_export.json` exists
- `real_backend_export_produced=true`
- `stub_fallback_used=false`

Consumer result:

- committed/public query backend consumed `coordination/latest`
- query topology path resolved to the real HM3D backend scene root
- publication diagnostics backend consumed `coordination/latest`
- diagnostics preserved `public_topology_definition="committed/published only"`
- diagnostics preserved `non_published_rooms_remain_internal=true`

The synthetic three-frame validation sequence did not produce committed/public rooms, so the query and diagnostics room counts are `0`. This is expected for the tiny blank RGB-D probe and does not indicate a handoff or consumer failure.

CA1M rollback smoke result:

- report: `runtime_export_validation/ros2_sim_ingress_ca1m_rollback_report.json`
- selected `real_backend_handoff_format=ca1m`
- invoked `stage_a_demo.py CA1M --core-only`
- producer return code: `0`
- coordinator refreshed from the CA1M rollback real backend scene root
- query and diagnostics consumed `coordination/latest`

Exact validation commands:

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH /home/ami/miniconda3/envs/boxfusion/bin/python - <<'PY'
import torch, sys, os
print('python', sys.executable)
print('torch', torch.__version__)
print('cuda_available', torch.cuda.is_available())
print('cuda_device_count', torch.cuda.device_count())
print('CUDA_HOME', os.environ.get('CUDA_HOME'))
print('path_starts_cuda', os.environ.get('PATH','').split(':')[0])
PY

python3 -m pytest boxfusion/test_ros_simulation_ingress.py -q
python3 -m py_compile boxfusion/ros_simulation_ingress.py boxfusion_ros_query_server/launch/simulation_ingress.launch.py

CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_ros2_sim_ingress_validation.py \
  --validate-real-backend \
  --real-backend-handoff-format hm3d \
  --output-root runtime_export_validation/ros2_sim_ingress_hm3d_backend \
  --coordination-root runtime_export_validation/ros2_sim_ingress_hm3d_backend/coordination \
  --sequence-name gazebo_sim_hm3d_backend \
  --reset-output \
  --real-backend-python /home/ami/miniconda3/envs/boxfusion/bin/python \
  --real-backend-script stage_a_demo.py \
  --real-backend-model-path ./models/cutr_rgbd.pth \
  --real-backend-clip-path ./models/ViT-B-32/open_clip_pytorch_model.bin \
  --real-backend-text-features ./data/class_features_small.pt \
  --real-backend-device cuda \
  --json-out runtime_export_validation/ros2_sim_ingress_hm3d_backend_report.json

CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH /home/ami/miniconda3/envs/boxfusion/bin/python stage_a_ros2_sim_ingress_validation.py \
  --validate-real-backend \
  --real-backend-handoff-format ca1m \
  --output-root runtime_export_validation/ros2_sim_ingress_ca1m_rollback \
  --coordination-root runtime_export_validation/ros2_sim_ingress_ca1m_rollback/coordination \
  --sequence-name gazebo_sim_ca1m_rollback \
  --reset-output \
  --real-backend-python /home/ami/miniconda3/envs/boxfusion/bin/python \
  --real-backend-script stage_a_demo.py \
  --real-backend-model-path ./models/cutr_rgbd.pth \
  --real-backend-clip-path ./models/ViT-B-32/open_clip_pytorch_model.bin \
  --real-backend-text-features ./data/class_features_small.pt \
  --real-backend-device cuda \
  --json-out runtime_export_validation/ros2_sim_ingress_ca1m_rollback_report.json
```

Recommendation:

- HM3D should replace CA1M as the preferred simulation-ingress producer bridge for this phase.
- CA1M should be demoted to rollback-only status, not removed yet.

## 2026-04-15 Earlier CA1M Real Backend Handoff Update

At this earlier point, the simulation ingress had two explicit backend modes:

- `backend_mode=stub`: preserves the original minimal committed/public bundle writer.
- `backend_mode=real`: wrote the captured simulated RGB-D + pose sequence to a CA1M-style backend input directory, then invoked the existing `stage_a_demo.py CA1M --core-only` producer over that captured sequence.

The chosen real Stage A handoff is intentionally narrow:

```text
<output_root>/stage_a_sequences/<sequence_name>/logs/simulation_ingress_frames.jsonl
        |
        v
<output_root>/stage_a_backend_inputs/<sequence_name>/
  rgb/<idx>.png
  depth/<idx>.png
  all_poses.npy
  K_rgb.txt
  K_depth.txt
  boxfusion_ca1m_config.yaml
        |
        v
stage_a_demo.py CA1M --core-only --config boxfusion_ca1m_config.yaml
        |
        v
<output_root>/stage_a_real_backend_sequences/<sequence_name>/
        |
        v
BoxFusionRuntimeExportCoordinator
```

This keeps ingestion snapshot-style and keeps the coordinator, committed/public query server, and debug publication diagnostics server unchanged. The sidecar line remains frozen and is still disabled for this adapter path.

Stub mode remains available and reversible. Real mode can also be launched with `fallback_to_stub_on_real_backend_failure:=true`, which is useful for bring-up but is reported explicitly as fallback rather than as a real backend export.

## Scope

This step freezes the sidecar line and adds only a thin simulation ingress layer for RGB-D plus pose input. It does not change the synchronous manifest-backed exporter, the committed/public query server, the debug-only publication diagnostics server, or the `minimal_public_topology_subset` sidecar scope.

Public topology meaning remains unchanged:

- public topology is committed/published only
- lifecycle/debug diagnostics may describe non-public rooms
- non-PUBLISHED rooms are not promoted into the query server's public/default topology

## Chosen ROS 2 Inputs

The new adapter node is `boxfusion_simulation_ingress_node`.

Default topics:

- RGB image: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
- depth image: `/camera/depth/image_raw` (`sensor_msgs/msg/Image`)
- camera info: `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
- pose: `/boxfusion/sim/pose` (`geometry_msgs/msg/PoseStamped`)

The node also supports the narrow odometry alternative:

- set `pose_source:=odom`
- set `odom_topic:=/odom` or another `nav_msgs/msg/Odometry` topic

TF lookup is intentionally deferred. A pose or odometry topic is the narrowest practical first adapter for Gazebo-style validation without adding a transform policy surface.

## Synchronization Strategy

The adapter keeps a latest-message cache for RGB, depth, camera info, and pose. A periodic timer samples the cache using `snapshot_period_sec` and accepts a sample only when RGB, depth, and pose timestamps are within `max_sync_skew_sec`.

This is snapshot-style ingestion, not streaming semantics. It avoids message-filter policy complexity and keeps the adapter reversible.

## Stage A / Coordinator Hand-Off

Each accepted RGB-D + pose sample is first written under a workspace-local capture scene root:

```text
<output_root>/stage_a_sequences/<sequence_name>/
```

The capture root always contains:

- `frames/frame_<idx>_rgb.bin`
- `frames/frame_<idx>_depth.bin`
- `logs/simulation_ingress_frames.jsonl`
- `logs/simulation_ingress_capture_summary.json`

In `backend_mode=stub`, the adapter also writes the original minimal committed bundle in that same scene root:

- `logs/topology_v0_1.json`
- `logs/online_topology_lifecycle_v0_1.json`
- `logs/summary.json`
- `manifest.json`

The stub topology export remains intentionally minimal: a single committed/public room envelope derived from the pose samples, with no Gazebo-specific topology semantics.

In `backend_mode=real`, the minimal topology stub is bypassed for the authoritative refresh. By default, `real_backend_handoff_format=hm3d` materializes:

- `rgb/<idx>.png`
- `depth/<idx>.png`
- `pose/<idx>.txt`
- `boxfusion_hm3d_config.yaml`
- `simulation_ingress_backend_input_manifest.json`

Then it runs the existing Stage A producer:

```bash
python3 stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config <output_root>/stage_a_backend_inputs/<sequence_name>/boxfusion_hm3d_config.yaml \
  --clip-path ./models/ViT-B-32/open_clip_pytorch_model.bin \
  --clip-model-name ViT-B-32 \
  --text-features ./data/class_features_small.pt \
  --device cpu \
  --max-frames <captured_sample_count> \
  --keyframe-gap 1 \
  --room-seg-interval 1 \
  --capture-stride 1 \
  --runtime-profile-interval 1 \
  --runtime-artifact-mode benchmark \
  --core-only \
  --quiet \
  --output-root <output_root>/stage_a_real_backend_sequences
```

Selecting `real_backend_handoff_format=ca1m` keeps the earlier rollback bridge available. That mode materializes `all_poses.npy`, `K_rgb.txt`, `K_depth.txt`, and `boxfusion_ca1m_config.yaml`, then invokes `stage_a_demo.py CA1M --core-only`.

If the producer succeeds, the adapter calls the existing `BoxFusionRuntimeExportCoordinator` on:

```text
<output_root>/stage_a_real_backend_sequences/<sequence_name>
```

If the producer fails and fallback is disabled, no coordinator latest pointer is refreshed. If fallback is enabled, the fallback is reported in `logs/simulation_ingress_backend_handoff.json` and the coordinator refreshes from the original stub scene root.

The coordinator still refreshes:

```text
<coordination_root>/latest
<coordination_root>/latest_manifest.json
<coordination_root>/latest_export.json
```

Downstream consumers continue to read `coordination/latest`.

## Run Roots

Defaults are workspace-local and configurable:

- `output_root`: `ros2_sim_ingress_runs`
- `coordination_root`: `ros2_sim_ingress_runs/coordination`
- `sequence_name`: `gazebo_sim_sequence`
- `real_backend_handoff_format`: `hm3d`
- `real_backend_output_root`: `ros2_sim_ingress_runs/stage_a_real_backend_sequences`
- real backend input root: `ros2_sim_ingress_runs/stage_a_backend_inputs/<sequence_name>`

No `/tmp/...` path is hardcoded.

## Intentionally Deferred

- Gazebo-specific topology semantics
- TF transform policy
- full online robot stack behavior
- detector / CLIP changes
- multi-GPU work
- broad Stage A optimization
- maintained candidate indices
- anchor / scene graph / GraphML migration
- sidecar authoritative migration
- public topology expansion
- online publication redesign
- real ROS 2 / Gazebo runtime exercise in this environment

## Validation Report

### 2026-04-15 Real Backend Acceptance Rerun in Project ML Environment

Environment used:

- Python interpreter: `/home/ami/miniconda3/envs/boxfusion/bin/python`
- Python version: `3.10.20`
- CUDA compiler path required for the producer subprocess: `/usr/local/cuda/bin/nvcc`
- Invocation environment fix: `CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH`

Dependency checks:

- `torch` available: `2.5.1+cu124`
- `open_clip` available: `2.32.0`
- `cv2` available: `4.11.0`
- `PyYAML` available: `6.0.2`
- `numpy` available: `1.26.4`
- required model/text-feature files present:
  - `models/cutr_rgbd.pth`
  - `models/ViT-B-32/open_clip_pytorch_model.bin`
  - `data/class_features_small.pt`

Real-backend validation command:

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_ros2_sim_ingress_validation.py \
  --validate-real-backend \
  --reset-output \
  --output-root runtime_export_validation/ros2_sim_ingress_real_backend \
  --coordination-root runtime_export_validation/ros2_sim_ingress_real_backend/coordination \
  --sequence-name gazebo_sim_real_backend \
  --real-backend-python /home/ami/miniconda3/envs/boxfusion/bin/python \
  --json-out runtime_export_validation/ros2_sim_ingress_real_backend_report.json
```

Real backend producer result:

- real Stage A producer succeeded with `producer_returncode=0`
- generated CA1M-style backend input at `runtime_export_validation/ros2_sim_ingress_real_backend/stage_a_backend_inputs/gazebo_sim_real_backend`
- generated real backend scene root at `runtime_export_validation/ros2_sim_ingress_real_backend/stage_a_real_backend_sequences/gazebo_sim_real_backend`
- produced `manifest.json`, `logs/topology_v0_1.json`, and `logs/online_topology_lifecycle_v0_1.json`
- processed 3 simulated RGB-D + pose frames

Coordinator refresh result:

- `runtime_export_validation/ros2_sim_ingress_real_backend/coordination/latest` exists and resolves to the real backend scene root
- `latest_manifest.json` and `latest_export.json` were refreshed
- `real_backend_export_produced=true`
- `stub_fallback_used=false`

Query consumption result:

- the committed/public query backend consumed `coordination/latest`
- `GetTopology` resolved `logs/topology_v0_1.json`
- the synthetic 3-frame scene exported 0 committed/public rooms
- `get_world_snapshot` loaded the bundle but reported no final vector-map snapshot availability for this minimal real run

Diagnostics consumption result:

- the publication diagnostics backend consumed `coordination/latest`
- diagnostics reported 0 lifecycle/debug rooms and 0 published rooms for the synthetic 3-frame scene
- `public_topology_definition` remained `committed/published only`
- `non_published_rooms_remain_internal=true`

Fallback behavior result:

- successful real backend export is distinct from fallback: the successful report has `real_backend_export_produced=true` and `stub_fallback_used=false`
- a separate forced-failure fallback probe wrote `runtime_export_validation/ros2_sim_ingress_real_backend_forced_fallback_report.json`
- in that forced-failure probe, the producer returned 7 by design, `stub_fallback_used=true`, and the coordinator refreshed from the stub scene root with 1 committed/public room

Minimal fixes required:

- `CA1MDataset` now falls back to the dataset directory basename when the CA1M numeric scene-id regex does not match. This preserves canonical CA1M ids and lets generated simulation-ingress sequence names such as `gazebo_sim_real_backend` pass through Stage A export finalization.
- no sidecar, ROS/query/debug contract, detector, CLIP, topology, publication, navigation, or Stage A refactor changes were made.

CA1M-vs-HM3D/online handoff assessment:

- Keep the current CA1M-style file-backed handoff as the temporary narrow simulation-ingress producer bridge for this phase.
- It is acceptable now because it is the smallest adapter from captured simulated RGB-D + pose into the existing synchronous Stage A producer: `rgb/<idx>.png`, `depth/<idx>.png`, `all_poses.npy`, `K_rgb.txt`, `K_depth.txt`, and a generated config. It keeps the ROS2 ingress snapshot-style and leaves the coordinator, query backend, and diagnostics backend unchanged.
- HM3D is closer to the project's mainline benchmark/runtime evidence, but the current HM3D dataset entry point expects `pose/*.txt` plus static intrinsics in `config/hm3d.yaml`, and applies Habitat-to-OpenCV/Z-up conversions. Moving the simulation ingress to HM3D should be a deliberate narrow handoff change, not part of this validation step.
- The `online` dataset entry point is ROS1-topic oriented and would broaden this ROS2 adapter into live online ingestion semantics, which is explicitly out of scope here.
- The narrowest future migration point, if HM3D alignment becomes necessary, is the materialization boundary currently implemented by `StageASimulationSnapshotWriter.materialize_real_backend_input_sequence()`: emit an HM3D-shaped directory with `rgb/`, `depth/`, `pose/*.txt`, matching `fx/fy/cx/cy` config fields, and then invoke `stage_a_demo.py hm3d --core-only`. The coordinator and consumers should remain unchanged.

Remaining gaps:

- live ROS 2 / Gazebo topic ingestion was still not exercised in this workspace (`rclpy`, `ros2`, `gz`, and `gazebo` unavailable)
- the 3-frame synthetic run proves the real producer/export path, not meaningful topology quality
- future HM3D handoff evaluation should be a separate narrow step at the materialized input-sequence boundary

### Previous Local Validation Without Project ML Environment

Gazebo and ROS 2 were not available in this workspace:

```bash
python3 - <<'PY'
try:
    import rclpy
    print('rclpy_available=true')
except Exception as exc:
    print('rclpy_available=false')
    print(type(exc).__name__ + ': ' + str(exc))
PY

command -v ros2 || true
command -v gz || true
command -v gazebo || true
```

Observed result:

- `rclpy_available=false`
- `ros2`, `gz`, and `gazebo` commands were not found

Because Gazebo/ROS 2 were unavailable, validation used the local RGB-D + pose stub path in the same adapter writer:

```bash
python3 stage_a_ros2_sim_ingress_validation.py \
  --validate-stub \
  --reset-output \
  --output-root runtime_export_validation/ros2_sim_ingress_stub \
  --coordination-root runtime_export_validation/ros2_sim_ingress_stub/coordination \
  --sequence-name gazebo_sim_stub \
  --json-out runtime_export_validation/ros2_sim_ingress_stub_report.json
```

Stub validation result:

- wrote 3 simulated RGB-D + pose samples
- produced a fresh Stage-A-compatible scene root at `runtime_export_validation/ros2_sim_ingress_stub/stage_a_sequences/gazebo_sim_stub`
- refreshed coordinator latest pointer at `runtime_export_validation/ros2_sim_ingress_stub/coordination/latest`
- query backend consumed `coordination/latest` and loaded 1 committed/public room
- publication diagnostics consumed `coordination/latest` and reported 1 published room
- diagnostics response preserved `public_topology_definition="committed/published only"`
- sidecar shadow export remained disabled for this adapter path

Targeted regression command:

```bash
python3 -m pytest boxfusion/test_ros_simulation_ingress.py boxfusion/test_runtime_export_coordinator.py -q
```

Observed result:

- `9 passed`

Additional targeted regression after the real-backend mode update:

```bash
python3 -m pytest boxfusion/test_ros_simulation_ingress.py -q
```

Observed result:

- `3 passed`

Real-backend validation command:

```bash
python3 stage_a_ros2_sim_ingress_validation.py \
  --validate-real-backend \
  --reset-output \
  --output-root runtime_export_validation/ros2_sim_ingress_real_backend \
  --coordination-root runtime_export_validation/ros2_sim_ingress_real_backend/coordination \
  --sequence-name gazebo_sim_real_backend \
  --json-out runtime_export_validation/ros2_sim_ingress_real_backend_report.json
```

Observed result in this workspace:

- wrote 3 simulated RGB-D + pose samples
- materialized the real backend input sequence at `runtime_export_validation/ros2_sim_ingress_real_backend/stage_a_backend_inputs/gazebo_sim_real_backend`
- invoked `stage_a_demo.py CA1M --core-only` with the generated config
- real backend export was not produced because `open_clip` is not installed in the active Python interpreter; `torch` is also unavailable
- no coordinator latest pointer was refreshed in real mode because fallback was disabled
- query and diagnostics consumers therefore had no real backend bundle to consume in this environment

Fallback validation command:

```bash
python3 stage_a_ros2_sim_ingress_validation.py \
  --validate-real-backend \
  --fallback-to-stub-on-real-backend-failure \
  --reset-output \
  --output-root runtime_export_validation/ros2_sim_ingress_real_backend_fallback \
  --coordination-root runtime_export_validation/ros2_sim_ingress_real_backend_fallback/coordination \
  --sequence-name gazebo_sim_real_backend_fallback \
  --json-out runtime_export_validation/ros2_sim_ingress_real_backend_fallback_report.json
```

Observed result:

- real backend producer was attempted and failed for the same missing `open_clip` dependency
- fallback was explicitly reported
- coordinator latest pointer was refreshed from the stub scene root
- query backend consumed `coordination/latest` and loaded 1 committed/public room
- publication diagnostics consumed `coordination/latest` and reported 1 published room
- diagnostics still reported `public_topology_definition="committed/published only"`

What remains unverified:

- successful real Stage A export from simulated RGB-D + pose, blocked here by missing runtime ML dependencies
- live ROS 2 / Gazebo topic ingestion, because `rclpy`, `ros2`, `gz`, and `gazebo` were unavailable
- any full streaming robot semantics, intentionally out of scope

## Files Changed

- `boxfusion/ros_simulation_ingress.py`
- `boxfusion/test_ros_simulation_ingress.py`
- `boxfusion_ros_query_server/launch/simulation_ingress.launch.py`
- `boxfusion_ros_query_server/package.xml`
- `docs/ros2_simulation_ingress_adapter_note.md`
- `runtime_export_validation/ros2_sim_ingress_stub_report.json`
- `runtime_export_validation/ros2_sim_ingress_real_backend_report.json`
- `runtime_export_validation/ros2_sim_ingress_real_backend_fallback_report.json`
