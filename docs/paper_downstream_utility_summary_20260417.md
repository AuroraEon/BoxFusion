# Paper Downstream Utility Summary

Date: 2026-04-17

## Scope

This note summarizes what the current repo state already demonstrates downstream, using the authoritative committed/public export bundle only.
It does not add BEV planning, continuous navigation control, or any new backend semantics.

## ROS-Facing Public Query Path

Canonical artifact input:

- primary validated bundle:
  `runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json`
- stable latest-pointer validation:
  `runtime_export_validation/paper_eval_validation/latest_manifest.json`

Validation command executed in this workspace:

```bash
python3 stage_a_ros_query_server_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/manifest.json \
  --json-out runtime_export_validation/paper_ros_query_validation_00843.json
```

Coordinator validation command executed in this workspace:

```bash
python3 stage_a_runtime_export_coordinator_validation.py \
  --artifact-path runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --coordination-root runtime_export_validation/paper_eval_validation \
  --json-out runtime_export_validation/paper_eval_validation_report.json
```

Observed validation status:

- manifest-backed public query bundle resolves: yes
- stable latest-pointer coordinator bundle resolves: yes
- shadow sidecar parity check: passed
- minimal public topology parity check: passed

Example route-to-room query:

```bash
ros2 service call /boxfusion/query/route_to_room ros_interfaces/srv/RouteToRoom \
  "{start_room_id: 'room_11', goal_room_id: 'room_13', route_policy: 'balanced'}"
```

Validated backend result:

- route found: yes
- route: `room_11 -> room_7 -> room_13`

Example route-to-object query:

```bash
ros2 service call /boxfusion/query/route_to_object ros_interfaces/srv/RouteToObject \
  "{start_room_id: 'room_11', object_id: '', object_label: 'couch', route_policy: 'balanced'}"
```

Validated backend-style object route saved this turn:

- artifact: `runtime_export_validation/paper_ros_query_validation_00843.json`
- route found: yes
- validated object-id route: `room_11 -> room_7 -> room_3 -> room_2`

## Room-Graph VLN Demo

Primary explicit room-goal demo command already supported by the current pipeline:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --goal-room room_13 \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_room_13.json
```

Primary semantic room-summary demo command already supported by the current pipeline:

```bash
python3 stage_a_room_graph_vln_demo.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V \
  --start-room room_11 \
  --semantic-target couch \
  --json-out runtime_stage1_frozen_evidence/final_freeze_verification_fullseq_20260417/scenes/00843-DYehNKdT76V/final/00843-DYehNKdT76V_room_graph_vln_couch.json
```

Observed current demo behavior:

| Sequence | Explicit room-goal | Semantic room-summary | Example next hop | Vertical-transition evidence |
| --- | --- | --- | --- | --- |
| `00843-DYehNKdT76V` | success, `room_11 -> room_7 -> room_13` | success, `room_11 -> room_7 -> room_3` | `Go next to room_7.` | yes |
| `00824-Dd4bFSTQ8gi` | success, `room_8 -> room_11 -> room_7 -> room_14 -> room_16` | success, `room_8 -> room_11 -> room_7 -> room_15` | `Go next to room_11.` | no |
| `00862-LT9Jq6dN3Ea` | success, `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_15 -> room_1 -> room_3` | success, `room_40 -> room_33 -> room_34 -> room_23 -> room_12 -> room_21` | `Go next to room_33.` | yes |
| `00829-QaLdnwvtxbs` | success, `room_3 -> room_7` | success, `room_3 -> room_7` | `Go next to room_7.` | no |

Interpretability already demonstrated:

- next-hop advice is explicit in the saved VLN JSON outputs
- explicit room-goal and semantic room-summary routing both stay on the committed/public graph
- semantic routing uses committed room summaries but remains restricted to public topology rooms
- the main scene `00843-DYehNKdT76V` already contains supported cross-floor evidence through one vertical transition

## What Is Demonstrated vs Still Out Of Scope

Already demonstrated:

- manifest-backed public query consumption
- public room-to-room routing
- public object/semantic target routing
- room-graph visualization with path and next-hop explanation
- cross-floor evidence on the main scene and stronger multi-floor evidence on the reserve scene

Intentionally still out of scope before any BEV integration:

- BEV planning or raster navigation maps as a downstream planning surface
- continuous navigation control
- local waypoint generation or controller execution
- mixing working/lifecycle/debug rooms into the default public route surface
