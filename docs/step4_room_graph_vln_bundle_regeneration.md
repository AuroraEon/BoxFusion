# Step 4 Room-Graph VLN Bundle Regeneration

The optional existing room-graph VLN advisor bundle regeneration was attempted because all four retained scene roots had the required committed/public artifacts.

The original `demo/room_graph_vln_advisor_bundle_20260417.json` was not edited in place because it points to old roots that were absent locally. A Step 4 spec was created instead:

- `demo/room_graph_vln_advisor_bundle_step4_regenerated.json`

Command run:

```bash
/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_room_graph_vln_demo_bundle.py \
  --spec demo/room_graph_vln_advisor_bundle_step4_regenerated.json \
  --output-root runtime_stage1_frozen_evidence/step4_advisor_demo_bundle
```

Result:

- attempted: true
- success: true
- bundle index: `runtime_stage1_frozen_evidence/step4_advisor_demo_bundle/index.html`
- bundle manifest: `runtime_stage1_frozen_evidence/step4_advisor_demo_bundle/demo_bundle_manifest.json`
- scene count: 4

This only regenerated the existing paper-facing room-graph visualization bundle from committed/public artifacts. It did not add new visualization features, BEV integration, ROS/Gazebo work, topology repair, or new navigation semantics.
