# Paper Experiment Readiness Summary

The immediately paper-usable story is a final-state committed/public Stage-A backend, not a full SLAM or navigation stack. The strongest current evidence is the four frozen HM3D bundles under `runtime_stage1_frozen_evidence/...`: `00843`, `00824`, `00862`, and `00829`. All four expose committed/public topology, committed room summaries, runtime/storage summaries, room-commit diagnosis, working-vs-committed debug artifacts, and frozen room-graph routing demos.

What can already be evaluated safely:

- final committed/public world-model summary
- room/object/anchor queryability from exported topology
- room-level routing and semantic room-summary routing
- runtime, storage, and topology-churn diagnostics
- supporting ablation on public vs working/provisional topology

What should be finished next if the team wants a stronger paper:

- add one clean rerun ablation for `--room-seg-interval`
- add one clean rerun ablation for `box_fusion.use=False`
- refresh a registry/root so `stage_a_eval/run_backend_eval.py` can be run directly on current scene roots
- if a richer evaluation section is needed, produce at least one full-artifact HM3D rerun with `logs/timeline.json` so `boxfusion/world_model_eval.py` is no longer blocked
- do not claim external GT room/topology accuracy until a real HM3D alignment/scoring tool exists
