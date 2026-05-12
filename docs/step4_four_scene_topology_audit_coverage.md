# Step 4 Four-Scene Topology Audit Coverage

Step 4 resolves the four-scene paper topology coverage blocker. All four retained paper scenes are now auditable through committed/public artifacts only.

Audit command run:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python tools/audit_committed_topology.py \
  --scene-root runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V \
  --scene-root runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi \
  --scene-root runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea \
  --scene-root runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs \
  --out-dir runtime_stage1_frozen_evidence/step4_topology_audit_batch \
  --emit-json \
  --emit-csv \
  --emit-html
```

| scene_id | auditable | rooms | floors | edges | objects | gateways | vertical transitions | strong topology risks | candidate-level issues |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| `00843-DYehNKdT76V` | true | 11 | 2 | 25 | 86 | 12 | 1 | none | 6 low-support edge warnings, 13 missing-edge candidates |
| `00824-Dd4bFSTQ8gi` | true | 8 | 1 | 28 | 141 | 3 | 0 | none | 11 low-support edge warnings, 30 missing-edge candidates |
| `00862-LT9Jq6dN3Ea` | true | 30 | 3 | 107 | 467 | 32 | 2 | none | 34 low-support edge warnings, 51 missing-edge candidates |
| `00829-QaLdnwvtxbs` | true | 2 | 1 | 2 | 144 | 6 | 0 | none | 0 edge warnings, 14 missing-edge candidates |

Strong topology risk checks were clean across the four scenes:

- connected component count is `1` for every scene
- no isolated public topology rooms
- no duplicate edges
- no self-loop edges
- no edges with missing source/target endpoints
- no rooms missing floor ids
- no cross-floor non-vertical edges
- no vertical-transition edges missing transition metadata
- no route warning/error rows in the deterministic/query-report route probes

Candidate-level rows are not confirmed ground-truth topology errors. They are audit candidates from public artifacts only:

- spurious candidates are low-support committed edges
- missing-edge candidates are committed room-world neighbor/connectivity mismatches and weak same-floor geometry proximity candidates
- no topology repair was attempted

Recommended next step: `paper-facing topology/VLN visualization extension`. A later topology export/gateway/vertical-transition investigation can use the candidate rows, but the missing-scene coverage blocker itself is resolved.
