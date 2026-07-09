# RSLG-SLAM Legacy Boundary

`docs/rslg_slam_planner/` is the current RSLG-SLAM truth surface. The old
`docs/rslg_slam/` tree was migrated and deleted in task53b and must not be
recreated.

## Migrated And Deleted Truth Tree

The former `docs/rslg_slam/` documentation tree has been migrated into
`docs/rslg_slam_planner/` and deleted. Do not restore it as a parallel truth
source.

## Legacy Provenance

- `stage_a_demo.py` remains legacy Stage-A provenance/exporter material. It is
  not the current formal project entrypoint.
- `demo.py`, older `boxfusion/` modules, `config/`, `data/`, and `models/`
  preserve historical Stage-A and raw RGB-D provenance.
- Full raw RGB-D to Layer 1 reruns remain legacy Stage-A-backed until a clean
  current Layer 1 builder exists.
- Old `00824`, `Step30P1`, `Stage1`, and Nav2 material is historical only.
- task39, task41, and task42 runtime/showcase outputs are historical evidence
  only. They do not define the current formal path.
- task48 Go2/Gazebo/RViz showcase material is removed or historical. It does
  not define a current real robot, quadruped, or Gazebo runtime claim.

## Current Formal Path

`Frozen canonical Layer 1/2 artifacts + QueryTask -> RSLGRouteResult -> RouteResult-derived Layer 4 adapter inputs`

QueryTask is Layer 3 input. Layer 0 is raw/provenance input: RGB-D frames,
depth, provided poses, scene id, sequence id, dataset/config paths,
model/checkpoint provenance, CLIP checkpoint provenance, semantic class text,
text-feature provenance, and a Layer 0 manifest.

The current demo mode consumes frozen canonical artifacts. It does not
regenerate canonical Layer 1/2 artifacts.

## Runtime Boundary

The current formal chain has no active Nav2, AMCL, `map_server`,
`nav2_map_server`, ROS lifecycle, `planner_server`, `controller_server`,
`bt_navigator`, `NavigateToPose`, or `FollowPath` dependency. RViz and Gazebo
are optional visualization/runtime adapters only and are not required by the
static demo pack.

## Claim Boundary

RSLG-SLAM does not claim dense reconstruction, neural implicit SLAM, full
embodied navigation benchmark, full BEV planner, Nav2 success, AMCL success,
real robot deployment, physical stair climbing, Unitree Go2 control, quadruped
gait control, collision-free guarantee, LLM runtime navigation, or osmAG-Nav.

## Generated Output Boundary

Task evidence belongs under task directories. Old generated outputs are
historical evidence, not current truth, and should not be restored as the active
source for current docs or validators.
