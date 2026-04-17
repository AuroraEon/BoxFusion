# Demo-Ready ROS Entry And Room-Graph VLN Bundle Report

Date: 2026-04-17

## Scope

This note tightens the existing BoxFusion indoor RGB-D RSLG-SLAM demo path without changing public/default semantics.

Kept unchanged:

- public/default topology remains committed-only
- working/lifecycle/debug artifacts are not used as the default downstream routing surface
- no BEV
- no continuous navigation control
- no major backend redesign

## Exact HM3D Validation Directories

- `00843`: `/home/ami/zn_ws/hm3dsem_walks/val/00843-DYehNKdT76V`
- `00829`: `/home/ami/zn_ws/hm3dsem_walks/val/00829-QaLdnwvtxbs`
- `00862`: `/home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea`
- `00824`: `/home/ami/zn_ws/hm3dsem_walks/val/00824-Dd4bFSTQ8gi`

## Current Demo Selection

- Primary demo sequence: `00843-DYehNKdT76V`
- Current best supporting/backup sequence: `00824-Dd4bFSTQ8gi`

Why `00843` stays primary:

- 11 public committed rooms
- 25 public topology edges
- 18 room-transition events
- explicit room-target VLN demo works
- semantic room-summary VLN demo works
- already the strongest known-good two-floor advisor demo

Why `00824` is the strongest current backup:

- 8 public committed rooms
- 28 public topology edges
- 12 room-transition events
- explicit room-target VLN demo works
- semantic room-summary VLN demo works
- route structure is much richer than the 2-room `00829` cross-check

Why `00829` is not selected as the main backup:

- it is still valid as a sanity cross-check
- only 2 public rooms survived into the public topology on the checked run
- it is less presentation-friendly than `00824`

Why `00862` is not the default supporting page despite being successful:

- it preserved a much larger public graph than `00824`
- it also took `2260.149 s` to regenerate versus `450.378 s` for `00824`
- the resulting 30-room / 107-edge topology is useful as a reserve scene, but denser and less lightweight for a standard advisor walk-through
- `00824` is the better balance between richness, clarity, and rerun cost

## Sequence Comparison Snapshot

### `00843-DYehNKdT76V`

- output root:
  `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V`
- processed frames: `2710`
- duration: `783.198 s`
- average fps: `3.46`
- diagnosis category: `public_committed_rooms_available`
- public topology room count: `11`
- public topology edges: `25`
- room-transition events: `18`
- explicit demo path: `room_11 -> room_7 -> room_13`
- semantic demo path: `room_11 -> room_7 -> room_3`

### `00824-Dd4bFSTQ8gi`

- output root:
  `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi`
- processed frames: `2253`
- duration: `450.378 s`
- average fps: `5.002`
- diagnosis category: `public_committed_rooms_available`
- public topology room count: `8`
- public topology edges: `28`
- room-transition events: `12`
- explicit demo path: `room_8 -> room_11 -> room_7 -> room_14 -> room_16`
- semantic demo path: `room_8 -> room_11 -> room_7 -> room_15`

### `00829-QaLdnwvtxbs`

- output root:
  `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs`
- processed frames: `1804`
- duration: `552.318 s`
- average fps: `3.266`
- diagnosis category: `public_committed_rooms_available`
- public topology room count: `2`
- public topology edges: `2`
- room-transition events: `11`
- explicit demo path: `room_3 -> room_7`
- semantic demo path: `room_3 -> room_7`

### `00862-LT9Jq6dN3Ea`

- output root:
  `runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea`
- processed frames: `7498`
- duration: `2260.149 s`
- average fps: `3.317`
- diagnosis category: `public_committed_rooms_available`
- public topology room count: `30`
- public topology edges: `107`
- room-transition events: `63`
- explicit demo path: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3`
- semantic demo path: `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21`
- selection status: usable reserve scene, not the default advisor-facing backup page

## Authoritative Public ROS Entry Path

Preferred public-consumption bundle:

- `manifest.json`

Authoritative public artifacts behind that bundle:

- `logs/topology_v0_1.json`
- `logs/summary.json`
- optional `logs/committed_room_world_snapshot_v0_1.json`

Accepted fallback bring-up inputs:

- scene root
- `logs/summary.json`
- `logs/topology_v0_1.json`

Debug/diagnostic-only surfaces that are not the default public query input:

- `logs/working_topology_v0_1.json`
- `logs/online_topology_lifecycle_v0_1.json`
- `boxfusion_ros_query_server/launch/publication_diagnostics.launch.py`
- service prefix `/boxfusion/debug/publication`

Practical standard:

- use the scene `manifest.json` as the canonical ROS/query `artifact_path`
- use `query_server.launch.py` for advisor-facing public consumption
- keep publication diagnostics separate and explicitly debug-only

## Standard Commands

Environment used in this workspace:

```bash
export PYTHON=/home/ami/miniconda3/envs/boxfusion/bin/python
export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
```

### 1. Standard backend generation command

Primary demo scene:

```bash
$PYTHON stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00843-DYehNKdT76V \
  --output-root ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25
```

Supporting demo scene:

```bash
$PYTHON stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00824-Dd4bFSTQ8gi \
  --output-root ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

Reserve high-capability scene:

```bash
$PYTHON stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

### 2. Standard room-commit diagnosis command

```bash
$PYTHON stage_a_room_commit_diagnosis.py \
  ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

```bash
$PYTHON stage_a_room_commit_diagnosis.py \
  ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi
```

```bash
$PYTHON stage_a_room_commit_diagnosis.py \
  ./runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea
```

### 3. Standard ROS/query startup sequence

Non-ROS backend validation of the authoritative public bundle:

```bash
$PYTHON stage_a_ros_query_server_validation.py \
  --artifact-path ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json
```

ROS2 launch command for the same authoritative public bundle:

```bash
ros2 launch boxfusion_ros_query_server query_server.launch.py \
  artifact_path:=/home/ami/zn_ws/BoxFusion/runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json
```

Example ROS2 service calls on the public surface:

```bash
ros2 service call /boxfusion/query/get_topology ros_interfaces/srv/GetTopology "{}"
```

```bash
ros2 service call /boxfusion/query/route_to_room ros_interfaces/srv/RouteToRoom \
  "{start_room_id: 'room_11', goal_room_id: 'room_13', route_policy: 'balanced'}"
```

```bash
ros2 service call /boxfusion/query/route_to_object ros_interfaces/srv/RouteToObject \
  "{start_room_id: 'room_11', object_id: '', object_label: 'couch', route_policy: 'balanced'}"
```

Debug-only publication diagnostics launch:

```bash
ros2 launch boxfusion_ros_query_server publication_diagnostics.launch.py \
  artifact_path:=/home/ami/zn_ws/BoxFusion/runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V
```

Notes:

- the public query path above was validated in this workspace via `stage_a_ros_query_server_validation.py`
- `ros2` and `rclpy` are not installed in this shell, so the actual ROS2 launch was not executed here

### 4. Short advisor demo order

```bash
# 1. Validate the authoritative public query bundle
$PYTHON stage_a_ros_query_server_validation.py \
  --artifact-path ./runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json

# 2. Generate or refresh the advisor-facing VLN HTML bundle
$PYTHON stage_a_room_graph_vln_demo_bundle.py \
  --spec ./demo/room_graph_vln_advisor_bundle_20260417.json \
  --output-root ./runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417

# 3. Open the main bundle page
xdg-open ./runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/index.html

# 4. If ROS2 is available, launch the public query node on the same manifest
ros2 launch boxfusion_ros_query_server query_server.launch.py \
  artifact_path:=/home/ami/zn_ws/BoxFusion/runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json
```

## Presentation-Ready VLN Bundle

Bundle spec:

- `demo/room_graph_vln_advisor_bundle_20260417.json`

Bundle generation command:

```bash
$PYTHON stage_a_room_graph_vln_demo_bundle.py \
  --spec ./demo/room_graph_vln_advisor_bundle_20260417.json \
  --output-root ./runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417
```

Main bundle entry point:

- `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/index.html`

Primary scene page:

- `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00843-dyehnkdt76v/index.html`

Supporting scene page:

- `runtime_stage1_frozen_evidence/advisor_demo_bundle_20260417/00824-dd4bfstq8gi/index.html`

Primary scene query pages:

- `00843-DYehNKdT76V_room_graph_vln_demo_01_room_target_room_13.html`
  - explicit public room-goal routing from `room_11` to `room_13`
- `00843-DYehNKdT76V_room_graph_vln_demo_02_semantic_couch.html`
  - semantic-room-summary routing from `room_11` to the best `couch` room

Supporting scene query pages:

- `00824-Dd4bFSTQ8gi_room_graph_vln_demo_01_room_target_room_16.html`
  - longer explicit public room-goal route from `room_8` to `room_16`
- `00824-Dd4bFSTQ8gi_room_graph_vln_demo_02_semantic_bathtub.html`
  - semantic-room-summary routing from `room_8` to the best `bathtub` room

Each HTML page has a matching JSON summary beside it.

## Recommended Demo Semantic Targets

Global demo whitelist:

- `bed`
- `couch`
- `toilet`
- `sink`
- `bathtub`

Recommended primary-scene labels for `00843`:

- `couch`
- `bed`
- `toilet`
- `sink`

Recommended supporting-scene labels for `00824`:

- `bathtub`
- `toilet`
- `bed`
- `couch`

Labels intentionally avoided for demos unless needed:

- visibly noisy or obviously spurious labels such as `snow`, `sky`, `Oyster`, `Swan`, `Dessert`, `cat`

## Exact VLN Demo Commands Used

### `00843-DYehNKdT76V`

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --goal-room room_13 \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.json
```

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --semantic-target couch \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json
```

### `00824-Dd4bFSTQ8gi`

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi \
  --start-room room_8 \
  --goal-room room_16 \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_room_16.json
```

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi \
  --start-room room_8 \
  --semantic-target bathtub \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00824-Dd4bFSTQ8gi/final/00824-Dd4bFSTQ8gi_room_graph_vln_bathtub.json
```

### `00862-LT9Jq6dN3Ea`

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea \
  --start-room room_40 \
  --goal-room room_3 \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_room_3.json
```

```bash
$PYTHON stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea \
  --start-room room_40 \
  --semantic-target bathtub \
  --json-out runtime_stage1_frozen_evidence/demo_sequence_selection_20260417/scenes/00862-LT9Jq6dN3Ea/final/00862-LT9Jq6dN3Ea_room_graph_vln_bathtub.json
```

Conclusion:

- `00862` is demo-usable and should be kept as a reserve scene
- `00824` remains the better default supporting page for the advisor bundle
