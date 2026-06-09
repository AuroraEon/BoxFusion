# Obj178 Quadruped Visual-Kinematic Proxy Demo

This is the clean, Git-trackable demo entry for the RSLG-SLAM `obj178` bed in `room_14` on `00843` `floor_2`.

Claim boundary: `visual_kinematic_proxy_only`.

This demo proves that the RSLG-SLAM lightweight backend can drive a CHAMP-shaped quadruped visual-kinematic proxy through a `robot_profile` plus `robot_adapter`, resolve `obj178 bed / room_14`, execute the validated `dense_route_fallback`, reach the approach pose, and align yaw toward the object proxy.

It does not prove real quadruped gait, physical legged locomotion, visual object confirmation, real-world deployment, or stair climbing. It does not send joint, effort, torque, or position commands.

## Why This Entry Exists

The historical task22d reproduction script under `stage_outputs/.../task22d.../final_demo_package/final_reproduction_commands.sh` is deprecated as a formal demo entry. It used the current `champ_reference_kinematic_proxy.yaml` defaults and manual repeats exposed route-tracking timeouts for `obj178`.

The successful manual old-controller repeats passed 2/2 only when these obj178 dense-fallback controller parameters were passed explicitly:

```bash
--max-linear-speed 0.10
--lookahead-distance 0.60
--lookahead-min 0.40
--lookahead-max 1.00
--angular-smoothing-alpha 0.40
--angular-rate-limit 0.15
```

The scripts here keep those parameters explicit so the formal demo does not depend on mutable profile defaults.

## Headless Demo

```bash
tools/object_nav/demo_scripts/run_00843_floor2_quadruped_proxy_obj178_headless_demo.sh obj178_headless_repeat1
```

The headless script uses `/usr/bin/python3`, sources ROS Foxy, exports `ROS_DOMAIN_ID=84`, does not start `gzclient` or RViz, runs with `--execute`, and validates `runtime_result.json` plus `summary.json`.

## GUI Demo

```bash
tools/object_nav/demo_scripts/run_00843_floor2_quadruped_proxy_obj178_gui_demo.sh --run-id gui_smoke --keep-open 120
```

The GUI script runs the same reliable obj178 command with `--gui`. It starts `gzclient` through the quadruped proxy adapter and starts RViz when `stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun/runtime/rviz/00843_floor2_route_debug_safe_no_map.rviz` exists. If RViz is missing or unavailable, or if `gzclient` cannot stay alive because the display or graphics environment is unavailable, the script reports that directly instead of claiming GUI success.

Task23b GUI evidence is separated into:

- Headless/runtime validated: the shared obj178 runtime command passed `runtime_result.json` and `summary.json` validation.
- `gzclient` process observed by script: `gzclient` appeared in task23b process snapshots.
- `rviz2` process observed by script: `rviz2` appeared in task23b process snapshots.
- Manual GUI observation: optional human notes in `manual_gui_observation_template.md`; this does not replace process evidence.

## Outputs

Runtime outputs are written under:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23a_clean_obj178_quadruped_proxy_gui_demo_rebuild/runs/<run_id>
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/runs/<run_id>
```

Logs are written under:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23a_clean_obj178_quadruped_proxy_gui_demo_rebuild/logs
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/logs
```

Task23b GUI smoke evidence also writes:

```text
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/task23b_gui_smoke_report.json
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/task23b_gui_smoke_summary.md
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/process_snapshots/
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/gui_environment.txt
stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture/manual_gui_observation_template.md
```

## Validation

Validate a run directly:

```bash
/usr/bin/python3 tools/object_nav/demo_scripts/validate_00843_floor2_quadruped_proxy_obj178_outputs.py \
  --run-dir stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task23a_clean_obj178_quadruped_proxy_gui_demo_rebuild/runs/obj178_headless_repeat1
```

The validator requires success, object-facing approach success, approach pose reached, yaw aligned, wall-crossing validation passed, `dense_route_fallback`, no failure layer/reason, final distances within 0.35 m, runtime within 420 seconds, the visual proxy claim boundary when present, and no active Nav2 action servers when reported.
