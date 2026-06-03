#!/usr/bin/env python3
"""Audit the task14c obj_175 runtime failure and write task14c2 evidence."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, default: Any | None = None) -> Any:
    if not path.exists():
        if default is not None:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    candidate = Path(path)
    try:
        return str(candidate.resolve().relative_to(ROOT))
    except Exception:
        return str(path)


def command_contains(command: list[Any], flag: str, value: str | None = None) -> bool:
    items = [str(item) for item in command]
    if flag not in items:
        return False
    if value is None:
        return True
    return any(items[idx] == flag and idx + 1 < len(items) and items[idx + 1] == value for idx in range(len(items)))


def same_prefix(a: list[dict[str, Any]], b: list[dict[str, Any]], n: int) -> bool:
    for idx in range(n):
        ax = a[idx]
        bx = b[idx]
        if int(ax.get("waypoint_index", idx)) != int(bx.get("waypoint_index", idx)):
            return False
        if abs(float(ax.get("x")) - float(bx.get("x"))) > 1e-6:
            return False
        if abs(float(ax.get("y")) - float(bx.get("y"))) > 1e-6:
            return False
    return True


def pick_task12_success(stage_output_dir: Path) -> tuple[Path | None, dict[str, Any]]:
    candidates = sorted((stage_output_dir / "runs/active").glob("task12_00843_floor2_demo_task12_robust_*/route_execution_result_v0_1.json"))
    for path in reversed(candidates):
        payload = read_json(path, {})
        if (
            payload.get("execution_strategy") == "split_follow_path_with_current_pose_gateway_handoff"
            and payload.get("clean_runtime_success") is True
            and payload.get("terminal_reached") is True
            and payload.get("sparse_fallback_used") is False
        ):
            return path, payload
    task_path = stage_output_dir.parent / "tasks/task12_00843_floor2_controller_handoff_robustness/route_execution_result_v0_1.json"
    if task_path.exists():
        return task_path, read_json(task_path, {})
    return None, {}


def make_audit(args: argparse.Namespace) -> dict[str, Any]:
    object_id = args.object_id if args.object_id.startswith("obj_") else f"obj_{args.object_id}"
    route_path = args.stage_output_dir / "routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json"
    extended_path = args.task14c_output_dir / "runtime_runs" / object_id / f"extended_executable_route_{object_id}_generated_ring_037.json"
    runtime_probe_path = args.task14c_output_dir / "runtime_runs" / object_id / f"runtime_probe_result_{object_id}.json"
    task14c_route_result_path = args.task14c_output_dir / "runtime_runs" / object_id / f"route_execution_result_{object_id}.json"

    room_route = read_json(route_path, {})
    extended_route = read_json(extended_path, {})
    task14c_probe = read_json(runtime_probe_path, {})
    task14c_route_result = read_json(task14c_route_result_path, {})
    task12_path, task12_result = pick_task12_success(args.stage_output_dir)
    failed_command = task14c_probe.get("runtime_command") or []

    room_waypoints = list(room_route.get("waypoints") or [])
    extended_waypoints = list(extended_route.get("waypoints") or [])
    prefix_len = min(len(room_waypoints), len(extended_waypoints))
    route_prefix_unchanged = same_prefix(room_waypoints, extended_waypoints, prefix_len) if prefix_len else False
    appended_waypoint = extended_waypoints[len(room_waypoints)] if len(extended_waypoints) > len(room_waypoints) else None
    task14c_follow_result = task14c_route_result.get("follow_path_result") or {}
    task14c_anchor_validations = task14c_route_result.get("through_room_anchor_validation") or []

    repaired_command_shape = [
        "/usr/bin/python3",
        "tools/object_nav/run_objectnav_single_object_runtime.py",
        "--stage-output-dir",
        str(args.stage_output_dir),
        "--task14c-output-dir",
        str(args.task14c_output_dir),
        "--output-dir",
        str(args.output_dir),
        "--query",
        "curtain in room_14 on floor_2",
        "--object-id",
        object_id,
        "--approach-candidate-id",
        "generated_ring_037",
        "--floor-id",
        "floor_2",
        "--start-room",
        "room_11",
        "--target-room",
        "room_14",
        "--two-segment-execution",
    ]

    return {
        "artifact_type": "task14c2_root_cause_audit",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "scene_id": "00843-DYehNKdT76V",
        "object_id": object_id,
        "approach_candidate_id": "generated_ring_037",
        "script_invoked_the_failed_runtime": rel(ROOT / "tools/object_nav/run_objectnav_runtime_probe.py"),
        "failed_task14c_runtime_command_reconstructable": bool(failed_command),
        "failed_task14c_runtime_command": failed_command,
        "repaired_task14c2_runtime_command_shape": repaired_command_shape,
        "task14c_passed_expected_handoff_strategy": command_contains(
            failed_command,
            "--execution-strategy",
            "split_follow_path_with_current_pose_gateway_handoff",
        ),
        "task14c_passed_current_pose_gateway_handoff_flag": command_contains(failed_command, "--current-pose-gateway-handoff"),
        "run_scene_route_replaced_strategy": bool(
            task14c_route_result.get("execution_strategy")
            == "split_follow_path_with_through_room_dwell_then_forward_only_sparse_fallback"
        ),
        "run_scene_route_replacement_reason": (
            "run_scene_route.py rewrites the final execution_strategy to "
            "split_follow_path_with_through_room_dwell_then_forward_only_sparse_fallback when split execution "
            "fails and the generic sparse fallback block is entered."
        ),
        "final_failed_execution_strategy": task14c_route_result.get("execution_strategy"),
        "failed_runtime_summary": {
            "succeeded": task14c_route_result.get("succeeded"),
            "clean_runtime_success": task14c_route_result.get("clean_runtime_success"),
            "terminal_reached": task14c_route_result.get("terminal_reached"),
            "sparse_fallback_used": task14c_route_result.get("sparse_fallback_used"),
            "fallback_started_at_waypoint_index": task14c_route_result.get("fallback_started_at_waypoint_index"),
            "failure_layer": task14c_route_result.get("failure_layer"),
            "failure_reason": task14c_route_result.get("failure_reason"),
            "action_servers": task14c_route_result.get("action_servers"),
            "follow_path_result": task14c_follow_result,
            "through_room_anchor_validation": task14c_anchor_validations,
        },
        "sparse_fallback_trigger_classification": {
            "follow_path_send_goal_timeout": any(
                (attempt.get("failure_reason") == "FollowPath send_goal timeout")
                for attempt in task14c_route_result.get("follow_path_attempts", [])
            ),
            "compute_path_to_pose_send_goal_timeout": task14c_follow_result.get("failure_reason") == "ComputePathToPose send_goal timeout",
            "navigate_to_pose_aborted": any(
                ((row.get("navigate") or {}).get("failure_reason") == "NavigateToPose returned aborted")
                for row in task14c_route_result.get("waypoint_results", [])
            ),
            "semantic_split_anchor_validation_failed_first": any(
                item.get("completed") is False for item in task14c_anchor_validations
            ),
            "wrapper_logic_entered_generic_fallback_after_split_replan_failure": bool(task14c_route_result.get("sparse_fallback_used")),
            "classification": (
                "The first room_13 anchor validation failed after an initial FollowPath abort/distance miss, "
                "fallback then reached the anchor, but the outbound gateway handoff replan failed because "
                "ComputePathToPose send_goal timed out. run_scene_route.py then entered the generic sparse "
                "fallback block, and NavigateToPose aborted."
            ),
        },
        "timeout_or_composition_assessment": {
            "action_timeouts_too_short": "not proven",
            "event_loop_or_spinning_insufficient": "plausible for the ComputePathToPose send_goal timeout, but not proven from static artifacts alone",
            "command_composition_wrong": True,
            "command_composition_issue": (
                "The object-nav wrapper appended generated_ring_037 to the room route and sent one combined route "
                "to the room-route runner. This changed the semantic terminal from room_14 center waypoint 58 to "
                "object_approach_pose waypoint 59 and coupled target-room navigation to object approach."
            ),
        },
        "task12_success_comparison": {
            "task12_success_result": rel(task12_path),
            "task12_clean_runtime_success": task12_result.get("clean_runtime_success"),
            "task12_succeeded": task12_result.get("succeeded"),
            "task12_terminal_reached": task12_result.get("terminal_reached"),
            "task12_execution_strategy": task12_result.get("execution_strategy"),
            "task12_sparse_fallback_used": task12_result.get("sparse_fallback_used"),
            "task12_same_split_anchor_waypoint_index": 39 in (task12_result.get("semantic_goal_waypoint_indices") or []),
            "task12_handled_split_anchor_differently": bool(
                task12_result.get("clean_runtime_success") is True
                and task12_result.get("sparse_fallback_used") is False
            ),
        },
        "object_approach_append_analysis": {
            "original_route_waypoint_count": len(room_waypoints),
            "extended_route_waypoint_count": len(extended_waypoints),
            "first_59_waypoints_unchanged": route_prefix_unchanged and len(room_waypoints) == 59,
            "only_extra_waypoint": appended_waypoint,
            "manual_diagnosis_dir": rel(args.manual_diagnosis_dir),
        },
        "actual_fix_applied": (
            "task14c2 adds run_objectnav_single_object_runtime.py. It executes Segment A with the original "
            "room_11_to_room_14 route and task12 handoff behavior, validates target_room_arrival, then executes "
            "Segment B from the live current pose to generated_ring_037. The selected object approach pose is no "
            "longer appended to the room route."
        ),
    }


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# task14c2 Root-Cause Audit",
        "",
        f"- Project: `{payload['project_name']}`",
        f"- Failed runtime wrapper: `{payload['script_invoked_the_failed_runtime']}`",
        f"- task14c passed handoff strategy: `{payload['task14c_passed_expected_handoff_strategy']}`",
        f"- run_scene_route replaced final strategy: `{payload['run_scene_route_replaced_strategy']}`",
        f"- Final failed strategy: `{payload['final_failed_execution_strategy']}`",
        "",
        "## Root Cause",
        "",
        payload["timeout_or_composition_assessment"]["command_composition_issue"],
        "",
        payload["sparse_fallback_trigger_classification"]["classification"],
        "",
        "## Task12 Contrast",
        "",
        f"- Task12 result: `{payload['task12_success_comparison']['task12_success_result']}`",
        f"- Task12 clean runtime success: `{payload['task12_success_comparison']['task12_clean_runtime_success']}`",
        f"- Task12 sparse fallback used: `{payload['task12_success_comparison']['task12_sparse_fallback_used']}`",
        "",
        "## Route Append Check",
        "",
        f"- Original waypoint count: `{payload['object_approach_append_analysis']['original_route_waypoint_count']}`",
        f"- Extended waypoint count: `{payload['object_approach_append_analysis']['extended_route_waypoint_count']}`",
        f"- First 59 waypoints unchanged: `{payload['object_approach_append_analysis']['first_59_waypoints_unchanged']}`",
        "",
        "## Fix",
        "",
        payload["actual_fix_applied"],
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--task14c-output-dir", type=Path, required=True)
    parser.add_argument("--manual-diagnosis-dir", type=Path, required=True)
    parser.add_argument("--object-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    payload = make_audit(args)
    write_json(args.output_dir / "root_cause_audit.json", payload)
    write_md(args.output_dir / "root_cause_audit.md", payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
