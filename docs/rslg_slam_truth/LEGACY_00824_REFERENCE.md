# Legacy 00824 Reference

This document preserves the historical 00824 reference context before aggressive RSLG-SLAM pipeline convergence.

## Historical Reference Scene

`00824-Dd4bFSTQ8gi` is a historical Stage1 reference scene.

The `tools/stage1_nav/` and `tools/stage1_step30p1/` directories were historical accepted reference implementations. They are not the formal RSLG-SLAM command surface after convergence.

## Reference-Only Evidence

The following evidence is reference only:

- 00824 stable map evidence
- 00824 GUI evidence
- Room8 evidence
- Room15 evidence

This evidence may be useful for migration rationale, but it does not define the current 00843 formal pipeline truth.

## Explicit Non-Claims

The legacy 00824 reference does not establish:

- AMCL success
- real robot deployment
- object-level navigation
- full collision-free guarantee

The formal RSLG-SLAM command surface is `tools/rslg_pipeline/`.
