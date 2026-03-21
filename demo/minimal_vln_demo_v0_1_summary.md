# Minimal VLN Demo v0.1

This package formalizes the already validated, teacher-facing VLN demo cases under the RSLG philosophy: Rich Semantics, Light Geometry.

- Default main sequence: `00843-DYehNKdT76V`
- Backup sequence: `00847-bCPU9suPUw9`
- Runner: `demo/run_minimal_vln_demo_v0_1.py`
- Manifest: `demo/minimal_vln_demo_v0_1_cases.json`
- Default route policy: `balanced`

## Demo Boundary

- Included capability families: explicit room target, room-to-room routing, object-label target, anchor-id target
- Excluded from v0.1 demos: semantic room-name instructions
- Reason: template syntax exists, but the validated real exports still expose `room_type=unknown` or otherwise missing usable room semantic metadata, so semantic room-name targets are not demo-honest yet

## Main Sequence: `00843-DYehNKdT76V`

| Case ID | Instruction | Family | Expected Goal | Expected Route | Demo Use |
| --- | --- | --- | --- | --- | --- |
| `main_go_to_room_3` | `go to room_3` | explicit room target | `room_3` | `room_2 -> room_3` | Direct room-target routing |
| `main_go_from_room_2_to_room_7` | `go from room_2 to room_7` | room-to-room | `room_7` | `room_2 -> room_3 -> room_7` | Explicit multi-hop routing |
| `main_go_to_room_with_sofa` | `go to the room with sofa` | object-label target | `room_3` | `room_2 -> room_3` | Short semantic grounding route |
| `main_go_to_room_with_nightstand` | `go to the room with nightstand` | object-label target | `room_7` | `room_2 -> room_3 -> room_7` | Longer semantic grounding route |
| `main_go_to_anchor_room_6` | `go to anchor anchor_room_6` | anchor-id target | `room_6` | `room_2 -> room_6` | Anchor grounding to navigation target |

## Backup Sequence: `00847-bCPU9suPUw9`

| Case ID | Instruction | Family | Expected Goal | Expected Route | Demo Use |
| --- | --- | --- | --- | --- | --- |
| `backup_go_to_room_3` | `go to room_3` | explicit room target | `room_3` | `room_2 -> room_3` | Direct room-target routing |
| `backup_go_from_room_2_to_room_8` | `go from room_2 to room_8` | room-to-room | `room_8` | `room_2 -> room_5 -> room_8` | Explicit multi-hop routing |
| `backup_go_to_room_with_chair` | `go to the room with chair` | object-label target | `room_6` | `room_2 -> room_5 -> room_6` | Semantic grounding route |
| `backup_go_to_anchor_room_6` | `go to anchor anchor_room_6` | anchor-id target | `room_6` | `room_2 -> room_5 -> room_6` | Anchor grounding to navigation target |

## Suggested Commands

```bash
python3 demo/run_minimal_vln_demo_v0_1.py --sequence main --all-cases
python3 demo/run_minimal_vln_demo_v0_1.py --sequence backup --all-cases
python3 demo/run_minimal_vln_demo_v0_1.py --sequence main --case main_go_to_anchor_room_6
```
