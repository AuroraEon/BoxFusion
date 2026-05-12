# Step 4 Regenerated Artifact Validation

All retained scenes now have readable committed/public artifacts for the Step 4 audit surface:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/committed_room_world_snapshot_v0_1.json`

| scene_id | scene root | topology rooms/floors/edges | query report objects | snapshot gateways | snapshot vertical transitions | JSON status |
|---|---|---:|---:|---:|---:|---|
| `00843-DYehNKdT76V` | `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V` | 11 / 2 / 25 | 86 | 12 | 1 | valid |
| `00824-Dd4bFSTQ8gi` | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi` | 8 / 1 / 28 | 141 | 3 | 0 | valid |
| `00862-LT9Jq6dN3Ea` | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea` | 30 / 3 / 107 | 467 | 32 | 2 | valid |
| `00829-QaLdnwvtxbs` | `runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs` | 2 / 1 / 2 | 144 | 6 | 0 | valid |

The committed room-world model row counts are separately recorded in the CSV. They can be larger than public topology room counts because the committed model may retain additional room records while `topology_v0_1.json` is the public committed topology graph.
