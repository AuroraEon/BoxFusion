# Runtime Task Completion Report

Date: 2026-04-07

## Summary

The runtime stage-1 freeze task completed successfully. The current cleaned repo now has a conservative freeze package that documents the accepted runtime baseline, audits what still exists, records what is missing, and provides a reproducible compact verification path.

## What runtime work was completed successfully

- Audited the current cleaned repo state.
- Confirmed that the accepted 10-item runtime baseline is still reflected in code.
- Verified that compact runtime evidence still exists under `runtime_stage1_frozen_evidence/`.
- Reanalyzed the surviving compact representative bundle into a fresh verification root.
- Regenerated fresh compact representative runtime evidence for:
  - `00843-DYehNKdT76V`
  - `00862-LT9Jq6dN3Ea`
- Regenerated a fresh aggregate analysis over those new reruns.
- Verified live code sanity with `py_compile` and direct mock validation scripts.

## Final accepted runtime baseline

The frozen stage-1 baseline is:

1. floor-scoped pruning
2. within-floor retained-history filter (`recent<=25 OR XY<=4m`)
3. export-side structure reuse
4. changed-room rebuild reduction
5. AABB coarse reject before OBB / ConvexHull
6. unchanged-object eager export reuse
7. readonly-tail shadow-reference audit off by default
8. skip readonly reference-mask build when audit is off
9. per-floor wall-label vote cache
10. unchanged-room anchor reuse so rebuilt scoring/clearance work is avoided where possible

## Major bottlenecks reduced and by how much

The strongest defensible reductions are structural and evidence-backed:

- History-scope reduction:
  - surviving historical pre-baseline evidence was overwhelmingly `global_retained_history` / `near_global_retained_history`
  - current compact and fresh rerun evidence now records only `same_floor_recent_or_near_*` scope labels
- Duplicate export reduction:
  - historical full representative reruns previously showed duplicate export frames of `19`, `28`, and `73`
  - surviving compact post-baseline bundle shows `0`, `0`, and `0`
  - fresh reruns in this task also show `0` and `0`
- Export rebuild churn reduction:
  - current compact evidence shows changed-room local rebuild active on `4`, `4`, and `2` export calls
  - object export reuse totals are `103`, `83`, and `105`
  - anchor reuse room totals are `4`, `4`, and `0`

What I am not claiming:

- I am not claiming a single apples-to-apples full-scene end-to-end speedup factor from this task alone, because the strongest surviving pre-freeze numbers are full-scene historical runs while the fresh reruns performed here are compact 260-frame verification reruns.

## Lines that were tried and rejected

- The old global / near-global retained-history line is rejected as the accepted runtime baseline.
- Interpreting stage-5 cost as pure CLIP / BoxFusion remains rejected; export work inside that bucket is a known fact.
- Default-on readonly-tail shadow audit remains rejected for normal runtime; it stays opt-in only.
- Rebuilding unchanged rooms, unchanged objects, and unchanged-room anchors every export remains rejected.
- Opening a new broad optimization or topology redesign line in this task was intentionally rejected by project framing.

## What remains open but is no longer the immediate priority

- Full-length apples-to-apples reruns of every historical heavy representative scene under the frozen cleaned-repo package.
- Any broader online topology delta redesign or interface-planning work.
- Any future runtime work beyond the accepted stage-1 baseline.

These are no longer immediate blockers for freezing stage-1.

## Closure assessment

Runtime is now best treated as stage-1 closed / stabilized for the current repo state.

That does not mean runtime is perfect. It means:

- the accepted stage-1 line is clear,
- the code still matches it,
- the remaining compact evidence is enough to support it,
- and the repo now contains a practical reproduction guide for future audit.

## Next likely project direction

Per current project framing, the next likely direction after this runtime freeze is not another runtime-optimization pass. The more likely next phase is online topology design / interface planning on top of this frozen backend baseline.
