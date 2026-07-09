# Frozen Canonical Mode

Frozen canonical mode is the current task49-task52 static path.

It is documented under the current truth surface,
`docs/rslg_slam_planner/`. The old `docs/rslg_slam/` tree was migrated and
deleted in task53b.

It consumes existing canonical Layer 1/2 artifacts for scene
`00843-DYehNKdT76V`, accepts a Layer 3 QueryTask, emits an `RSLGRouteResult`, and
exports RouteResult-derived Layer 4 adapter inputs.

It does not rerun Layer 0-2, Stage-A, raw RGB-D inference, Gazebo, RViz, ROS
nodes, Habitat, GPU jobs, Nav2, AMCL, or map_server.

This mode is acceptable for current engineering closure and the static demo pack
because the canonical artifacts are the evidence source under validation. It is
not a full raw RGB-D to Layer 1 rerun claim.

Full raw RGB-D to Layer 1 regeneration still depends on the legacy Stage-A
lineage or a future clean Layer 1 builder.

QueryTask is Layer 3 input, not raw Layer 0 input. The formal current path is:

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`
