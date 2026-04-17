# HM3D Room-Graph VLN Demo Report

Date: 2026-04-17

## Scope

This note records a lightweight room-graph VLN visualization pass over the existing BoxFusion indoor RGB-D RSLG-SLAM backend.

Immediate goal of this work:

- produce a fast, showable room-level VLN visualization from committed/public outputs
- do not add BEV navigation
- do not add a continuous navigation-control stack
- do not change committed/public semantics

Semantics kept unchanged:

- public/default topology remains committed-only
- working/lifecycle rooms are not intentionally exposed through public/default outputs
- working/lifecycle artifacts are not consumed for route computation in this demo

## Demo Contract

The visualization layer consumes only committed/public artifacts:

- `logs/topology_v0_1.json`
  - authoritative public room graph
  - public room ids, polygons, centers, floors, and room-to-room relations
- `logs/committed_room_world_model_v0_1.json`
  - committed room semantic summaries
  - dominant object labels and object-label counts used for semantic target selection
- `logs/summary.json`
  - optional convenience path resolver for the two artifacts above

Artifacts intentionally not used for routing:

- `logs/working_topology_v0_1.json`
- `logs/online_topology_lifecycle_v0_1.json`
- `logs/room_scoped_runtime_state_v0_1.json`

Routing behavior:

- start room resolves from a public room id or public room name if available
- explicit goal mode resolves to a public room id or public room name if available
- semantic goal mode ranks committed room semantic summaries, but only among rooms that are also present in the public topology export
- path is shortest-path search over the existing public topology with the existing `balanced` route-policy relation set

What the demo does:

- shows the committed/public room graph
- marks start room, target room, room-path sequence, and next hop
- renders both a spatial polygon view and an abstract topology graph
- supports explicit room targets and semantic/object-style room targets

What the demo does not do:

- no BEV planning
- no continuous control
- no local waypoint generation
- no backend graph redesign
- no exposure of working-only rooms through the public/default surface

## Implementation

New code:

- `boxfusion/room_graph_vln_demo.py`
- `stage_a_room_graph_vln_demo.py`
- `boxfusion/test_room_graph_vln_demo.py`

Output form:

- self-contained HTML page
- matching JSON summary

The HTML view includes:

- spatial room-footprint visualization from public polygons
- public graph edges
- highlighted path
- start / goal / next-hop markers
- semantic ranking table for semantic-object targets
- room summary cards from committed room summaries

## Primary Development Sequence: `00843-DYehNKdT76V`

Authoritative scene root used:

- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V`

The full-sequence rerun for this scene had already been completed earlier on 2026-04-17 and documented separately in:

- `docs/hm3d_00843_fullseq_rerun_room_commit_report_20260417.md`

### Demo Commands Executed On `00843`

Explicit room-target demo:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --goal-room room_13 \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.json
```

Semantic room-summary demo:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --semantic-target couch \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json
```

### `00843` Observed Result

- committed/public topology available: yes
- public topology room count: 11
- committed room world model available: yes
- explicit route demo: success
- semantic route demo: success

Observed demo outputs:

- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.html`
- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.json`
- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.html`
- `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json`

Observed paths:

- explicit room query: `room_11 -> room_7 -> room_13`
- semantic `couch` query: `room_11 -> room_7 -> room_3`
- next hop in both cases: `room_7`

Notes:

- `topology_v0_1.json` on this scene does not expose usable object entities for object-room grounding
- semantic goal mode therefore correctly uses `committed_room_world_model_v0_1.json` room summaries instead of working-state data

## Cross-Sequence Check: `00829-QaLdnwvtxbs`

### Full-Sequence Backend Command Executed

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00829-QaLdnwvtxbs \
  --output-root ./runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

Observed backend summary:

- processed frames: 1804
- duration: 552.318 s
- average fps: 3.266
- committed/public topology available: yes
- public topology room count: 2
- room-commit diagnosis category: `public_committed_rooms_available`

### Demo Commands Executed On `00829`

Explicit room-target demo:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs \
  --start-room room_3 \
  --goal-room room_7 \
  --json-out runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_room_7.json
```

Semantic room-summary demo:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs \
  --start-room room_3 \
  --semantic-target bed \
  --json-out runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_bed.json
```

### `00829` Observed Result

- committed/public topology available: yes
- usable room graph/path available: yes
- explicit route demo: success
- semantic route demo: success

Observed demo outputs:

- `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_room_7.html`
- `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_room_7.json`
- `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_bed.html`
- `runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes/00829-QaLdnwvtxbs/final/00829-QaLdnwvtxbs_room_graph_vln_bed.json`

Observed paths:

- explicit room query: `room_3 -> room_7`
- semantic `bed` query: `room_3 -> room_7`
- next hop in both cases: `room_7`

Notes:

- this scene is demo-usable, but the public topology is much smaller than `00843`
- only two rooms survived into the public topology on this run
- despite that, the room-graph VLN demo still works directly with no backend changes

## Cross-Sequence Check: `00862-LT9Jq6dN3Ea`

### Raw Data State Observed In This Workspace

- extracted directory initially contained no `rgb/`, `depth/`, or `pose/` triplets
- a local source zip was present:
  - `/home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea.zip`
- zip inventory:
  - about 7,498 frames
  - 22,497 archived files total

### Command Executed

Raw extraction attempt:

```bash
mkdir -p /home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea && \
unzip -n -q /home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea.zip \
  -d /home/ami/zn_ws/hm3dsem_walks/val/00862-LT9Jq6dN3Ea
```

### Current `00862` Status

- partial raw extraction attempted: yes
- extracted runnable HM3D tree complete: no
- full uncapped backend rerun: not completed in this turn
- committed/public topology availability: not yet established from a fresh rerun in this workspace
- room-graph demo status: blocked on missing completed rerun outputs

What currently prevents a direct demo result:

- the scene was not initially extracted into runnable HM3D directory form
- the attempted extraction was not carried through to a complete `rgb/`, `depth/`, `pose/` tree in this turn
- after normalization, the remaining step is a real uncapped backend rerun over about 7,498 frames
- this sequence is already documented in the repo as a very expensive representative rerun

Same-style rerun command prepared for the next step:

```bash
CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH \
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \
  --model-path ./models/cutr_rgbd.pth \
  --config ./config/hm3d.yaml \
  --device cuda \
  --seq 00862-LT9Jq6dN3Ea \
  --output-root ./runtime_stage1_frozen_evidence/room_graph_vln_crosscheck_20260417/scenes \
  --room-seg-interval 100 \
  --capture-stride 25 \
  --video-fps 12 \
  --core-only \
  --runtime-profile-interval 25 \
  --quiet
```

## Sequence Status Summary

| sequence | backend status | public committed rooms available | room-graph/path usable | demo status |
| --- | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | previously completed full-sequence rerun reused | yes | yes | success |
| `00829-QaLdnwvtxbs` | fresh uncapped rerun completed in this work | yes | yes | success |
| `00862-LT9Jq6dN3Ea` | raw extraction attempted but not completed; rerun not completed | not yet verified | not yet verified | blocked by missing completed rerun outputs |

## Reproducible `00843` Demo Command

If the goal is to reproduce the current lightweight demo quickly from the existing successful full-sequence export, the narrowest command is:

```bash
/home/ami/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --semantic-target couch \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json
```

This produces:

- `.../00843-DYehNKdT76V_room_graph_vln_couch.html`
- `.../00843-DYehNKdT76V_room_graph_vln_couch.json`

## Limitations Before BEV Integration

- pathing is room-to-room only
- next hop is a room-level decision, not a metric waypoint
- semantic targets are resolved from committed room summaries, not dense object-level public topology entities
- room names only work when public `room_type` labels are usable; many current HM3D exports still have `room_type=unknown`
- floor transitions are shown only through public topology relations; there is no geometric stair/elevator action model
- `00862` still needs a completed uncapped rerun before the same demo claim can be made for that scene

## Conclusion

The immediate lightweight demo goal is met on the primary scene:

- `00843` now produces a showable room-graph VLN visualization from committed/public outputs only
- both explicit room routing and semantic room-summary routing work end-to-end on `00843`
- `00829` also works under the same lightweight demo layer after a fresh uncapped rerun

Current cross-sequence status is therefore:

- succeeded: `00843-DYehNKdT76V`
- succeeded: `00829-QaLdnwvtxbs`
- not yet completed: `00862-LT9Jq6dN3Ea`
