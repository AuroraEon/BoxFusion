# RSLG-SLAM Truth Source Index

The current truth surface is `docs/rslg_slam_planner/`, with the repository
root `README.md` serving as the concise public entry page. Future truth-guard
greps must include `README.md` alongside the planner docs and current pipeline
surfaces.

`docs/rslg_slam/` was an old task25-task42 truth tree. Its useful facts were
migrated into the current planner docs in task53b, and the directory was
deleted. Do not restore it as a competing source of truth.

## Authoritative Current Docs

- Root `README.md`: current top-level RSLG-SLAM entry page, not a historical
  Stage1/Nav2 runbook.
- `PROJECT_TRUTH.md`: project identity, layer names, runtime policy, scene guard
  truth, and workspace policy.
- `MAIN_CHAIN_OVERVIEW.md`: current frozen static chain and Layer 0-4 roles.
- `LAYER_ENTRYPOINTS.md`: layer commands and current formal/legacy status.
- `LAYER0_2_PROVENANCE.md`: raw/provenance input contract and Stage-A boundary.
- `FROZEN_CANONICAL_MODE.md`: current static demo mode.
- `STAGE_A_LEGACY_BOUNDARY.md`: legacy Stage-A and `stage_a_demo.py` boundary.
- `FORMAL_ARTIFACT_INDEX.md`: canonical Layer 1/2 artifact groups and guard
  truth.
- `DEMO_PACK_README.md`: task52 static demo-pack regeneration and claim
  boundary.
- `LEGACY_BOUNDARY.md`: historical-only materials and non-claims.
- `NO_NAV2_NO_AMCL_POLICY.md`: active runtime dependency boundary.
- `QUERY_TASK_SCHEMA.md` and `ROUTE_RESULT_SCHEMA.md`: Layer 3 request and
  route-result contracts.
- `ROUTE_RESULT_LAYER4_ADAPTERS.md`: RouteResult-derived Layer 4 adapter inputs.

## Migrated Truth

- RSLG-SLAM is the project name; `BoxFusion` is only the historical repository
  path.
- Layer 0 is raw/provenance input, not QueryTask.
- QueryTask is Layer 3 input.
- The current static demo mode consumes frozen canonical Layer 1/2 artifacts and
  does not rerun Stage-A or raw RGB-D inference.
- Full raw RGB-D to Layer 1 rerun remains legacy Stage-A-backed.
- `stage_a_demo.py` is legacy provenance, not the current formal entrypoint.
- Stable maps are RSLG-SLAM formal artifacts derived from world-model evidence,
  not external GT maps, semantic floorplans, room masks, runtime costmaps, or
  active map_server products.
- Current 00843 guard truth: `obj_175` curtain in `room_14` on `floor_2`;
  selected approach `generated_ring_002`; blocked evidence
  `generated_ring_037`; true transition edge `vt_1_centerline_e001`; forbidden
  non-transition edge `vt_1_centerline_e003`.
- Task evidence belongs under task directories; old generated outputs are
  historical evidence, not current truth.

## Historical Only

- `00824`, `Step30P1`, and `Stage1` material.
- task39, task41, and task42 runtime/showcase success wording.
- Old Nav2, AMCL, map_server, ROS lifecycle, planner_server, controller_server,
  bt_navigator, NavigateToPose, and FollowPath paths.
- Old RViz/Gazebo/Go2 showcase commands.
- Old tool migration plans that describe `tools/rslg_pipeline/` as future or
  absent.
- Old AI handoff and current/final status files.
