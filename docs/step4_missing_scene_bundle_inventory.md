# Step 4 Missing Scene Bundle Inventory

This inventory searched for the three retained paper scenes that Step 3 marked missing locally:

- `00824-Dd4bFSTQ8gi`
- `00862-LT9Jq6dN3Ea`
- `00829-QaLdnwvtxbs`

Search coverage included `runtime_stage1_frozen_evidence/`, `stage_a_eval/output/`, `stage_a_eval/scene_registry.json`, `demo/room_graph_vln_advisor_bundle_20260417.json`, `qualitative_assets_manifest.json`, `docs/`, path/name searches for the scene ids, artifact-name searches, and symlink checks under the relevant evidence/demo/doc roots.

| scene_id | existing committed/public bundle | existing bundle path | regeneration needed | reason |
|---|---:|---|---:|---|
| `00824-Dd4bFSTQ8gi` | false |  | true | Only manifest/report references were present locally; no scene root containing the four committed/public artifacts existed. |
| `00862-LT9Jq6dN3Ea` | false |  | true | Only manifest/report references were present locally; no scene root containing the four committed/public artifacts existed. |
| `00829-QaLdnwvtxbs` | false |  | true | Only manifest/report references were present locally; no scene root containing the four committed/public artifacts existed. |

The available retained scene from Step 3, `00843-DYehNKdT76V`, remained available at `runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V` and was not regenerated.
