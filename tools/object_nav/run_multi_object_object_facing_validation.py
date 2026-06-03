#!/usr/bin/env python3
"""Task14d multi-object object-facing navigation validation for RSLG-SLAM."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
OBJECT_NAV_DIR = Path(__file__).resolve().parent
if str(OBJECT_NAV_DIR) not in sys.path:
    sys.path.insert(0, str(OBJECT_NAV_DIR))

from object_nav_common import canonical_object_id, load_index, run_query  # noqa: E402


SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task14d_multi_object_object_facing_nav_validation"
EXPECTED_ROOM_ROUTE = ["room_11", "room_7", "room_13", "room_14"]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path | str | None) -> str | None:
    if path is None:
        return None
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except Exception:
        return str(path)


MISSING = object()


def read_json(path: Path, default: Any = MISSING) -> Any:
    if not path.exists():
        if default is not MISSING:
            return default
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def run_logged(cmd: list[str], log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as handle:
        handle.write("$ " + " ".join(cmd) + "\n")
        handle.flush()
        proc = subprocess.run(cmd, cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT, text=True)
    return int(proc.returncode)


def object_from_index(index: dict[str, Any], object_id: str) -> dict[str, Any] | None:
    oid = canonical_object_id(object_id)
    return next((obj for obj in index.get("objects", []) if obj.get("object_id") == oid), None)


def target_xy_for_yaw(obj: dict[str, Any], candidate: dict[str, Any]) -> tuple[dict[str, Any] | None, str]:
    proxy = candidate.get("visible_proxy_xy")
    if isinstance(proxy, list) and len(proxy) >= 2:
        return {"x": float(proxy[0]), "y": float(proxy[1]), "source": "visible_proxy_xy"}, "visible_proxy_xy"
    pose = obj.get("pose_xy")
    if isinstance(pose, list) and len(pose) >= 2:
        return {"x": float(pose[0]), "y": float(pose[1]), "source": "object_pose_xy"}, "object_pose_xy"
    footprint = obj.get("footprint_2d") or []
    if footprint:
        return {
            "x": sum(float(p[0]) for p in footprint) / len(footprint),
            "y": sum(float(p[1]) for p in footprint) / len(footprint),
            "source": "object_footprint_centroid",
        }, "object_footprint_centroid"
    return None, "missing_object_proxy"


def expected_yaw(candidate: dict[str, Any], target: dict[str, Any] | None) -> float | None:
    xy = candidate.get("world_xy")
    if target and isinstance(xy, list) and len(xy) >= 2:
        return math.atan2(float(target["y"]) - float(xy[1]), float(target["x"]) - float(xy[0]))
    if candidate.get("yaw") is not None:
        return float(candidate["yaw"])
    return None


def room_route_available(obj: dict[str, Any], executable_route: Path) -> tuple[bool, str, list[str]]:
    route = read_json(executable_route, {})
    route_rooms = list(route.get("room_sequence") or [])
    if obj.get("floor_id") != "floor_2":
        return False, "route unavailable: task14d only has a floor_2 runtime map and route", route_rooms
    if not executable_route.exists():
        return False, "route unavailable: executable room route asset is missing", route_rooms
    if obj.get("room_id") == "room_14" and route_rooms == EXPECTED_ROOM_ROUTE:
        return True, "existing room_11 -> room_7 -> room_13 -> room_14 route reaches room_14", route_rooms
    if obj.get("room_id") in route_rooms:
        return False, "route-supported room, but task14d Segment A is fixed to the room_14 terminal before object approach", route_rooms
    return False, "route unavailable: object room is not on the existing room_11_to_room14 route", route_rooms


def source_artifacts_for_object(index_obj: dict[str, Any] | None) -> dict[str, Any]:
    provenance = (index_obj or {}).get("source_artifact_provenance") or {}
    clean_root = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
    public = clean_root / "committed_public"
    return {
        "topology_v0_1": rel(public / "topology_v0_1.json") if provenance.get("topology_v0_1") else None,
        "committed_room_world_snapshot_v0_1": rel(public / "committed_room_world_snapshot_v0_1.json")
        if provenance.get("committed_room_world_snapshot_v0_1")
        else None,
        "final_vector_map_snapshot_comparison_only": rel(public / "final_vector_map_snapshot.json")
        if provenance.get("final_vector_map_snapshot")
        else None,
        "source_artifact_provenance": provenance,
    }


def default_query_specs() -> list[dict[str, Any]]:
    return [
        {
            "query": "curtain in room_14 on floor_2",
            "required_object_id": "obj_175",
            "selection_role": "known_successful_single_object_reference",
        },
        {
            "query": "nightstand in room_14 on floor_2",
            "required_object_id": "obj_177",
            "selection_role": "second_room14_route_supported_runtime_candidate",
        },
    ]


def build_candidate_plan(args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    index_path = args.task14a_output_dir / "object_candidate_index_v0_1.json"
    index = load_index(index_path)
    plans: list[dict[str, Any]] = []
    for spec in default_query_specs():
        query = spec["query"]
        result = run_query(index, query, top_k=10)
        selected = result.get("selected_candidate") or {}
        required_id = canonical_object_id(spec["required_object_id"])
        if selected.get("object_id") != required_id:
            forced = object_from_index(index, required_id)
            if forced:
                selected = {
                    "object_id": forced["object_id"],
                    "label": forced.get("label"),
                    "normalized_label": forced.get("normalized_label"),
                    "room_id": forced.get("room_id"),
                    "floor_id": forced.get("floor_id"),
                    "warnings": forced.get("warnings", []),
                    "forced_required_object": True,
                }
        idx_obj = object_from_index(index, selected.get("object_id")) if selected else None
        route_ok, route_reason, route_rooms = room_route_available(idx_obj or {}, args.executable_route)
        approach_report: dict[str, Any] | None = None
        candidate: dict[str, Any] | None = None
        if selected and route_ok:
            source_report_json = args.approach_report_source_dir / "approach_candidate_reports" / f"object_approach_report_{selected['object_id']}.json"
            source_report_md = args.approach_report_source_dir / "approach_candidate_reports" / f"object_approach_report_{selected['object_id']}.md"
            source_png = next(
                iter(sorted((args.approach_report_source_dir / "approach_candidate_visualizations").glob(f"object_approach_{selected['object_id']}_*.png"))),
                None,
            )
            if source_report_json.exists():
                approach_report = read_json(source_report_json, {})
                dst_json = output_dir / "approach_candidate_reports" / source_report_json.name
                shutil.copyfile(source_report_json, dst_json)
                if source_report_md.exists():
                    shutil.copyfile(source_report_md, output_dir / "approach_candidate_reports" / source_report_md.name)
                if source_png:
                    shutil.copyfile(source_png, output_dir / "approach_candidate_visualizations" / source_png.name)
                approach_report["report_json"] = rel(dst_json)
                candidate = approach_report.get("recommended_candidate")
        target_proxy, target_source = target_xy_for_yaw(idx_obj or {}, candidate or {})
        yaw = expected_yaw(candidate or {}, target_proxy)
        overlaps = (candidate or {}).get("semantic_object_footprint_overlaps") or []
        runnable = bool(selected and route_ok and candidate and not overlaps and target_proxy)
        not_runnable_reasons = []
        if not selected:
            not_runnable_reasons.append(result.get("failure_reason") or "query did not resolve")
        if selected and not route_ok:
            not_runnable_reasons.append(route_reason)
        if route_ok and not candidate:
            not_runnable_reasons.append((approach_report or {}).get("failure_reason") or "approach candidate unavailable")
        if overlaps:
            not_runnable_reasons.append("selected approach candidate overlaps another committed object footprint proxy")
        if candidate and not target_proxy:
            not_runnable_reasons.append("no usable object/proxy target for yaw alignment")
        plans.append(
            {
                "query": query,
                "selection_role": spec["selection_role"],
                "query_resolution": result,
                "resolved": bool(selected),
                "resolved_object_id": selected.get("object_id"),
                "label": selected.get("label"),
                "floor_id": selected.get("floor_id"),
                "room_id": selected.get("room_id"),
                "object_source_artifact": source_artifacts_for_object(idx_obj),
                "route_availability": {"available": route_ok, "reason": route_reason},
                "selected_room_route": {
                    "route_id": "floor_2_room11_to_room14",
                    "room_sequence": route_rooms,
                    "semantic_route": rel(args.semantic_route),
                    "executable_route": rel(args.executable_route),
                    "object_approach_appended_to_room_route": False,
                },
                "approach_candidate_available": bool(candidate),
                "selected_approach_candidate_id": (candidate or {}).get("candidate_id"),
                "approach_candidate_world_xy": (candidate or {}).get("world_xy"),
                "approach_candidate_source_report": (approach_report or {}).get("report_json"),
                "approach_candidate_generation_source": rel(args.approach_report_source_dir),
                "approach_candidate_overlap_count": len(overlaps),
                "object_proxy_target_for_yaw_alignment": target_proxy,
                "object_proxy_target_source": target_source,
                "expected_yaw_rad": round(yaw, 6) if yaw is not None else None,
                "expected_yaw_deg": round(math.degrees(yaw), 3) if yaw is not None else None,
                "runnable": runnable,
                "reason": "runnable: route, approach candidate, no proxy overlap, and yaw target are available"
                if runnable
                else "; ".join(not_runnable_reasons),
            }
        )
    payload = {
        "artifact_type": "task14d_selected_candidate_plan",
        "created_utc": utc_now(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "candidate_selection_policy": [
            "prefer floor_2 objects",
            "prefer existing room_11 -> room_14 route-supported room_14 objects",
            "require valid approach candidates",
            "reject selected candidates overlapping other committed object footprint proxies",
            "require a usable object/proxy target for yaw alignment",
        ],
        "source_index": rel(index_path),
        "approach_report_source_dir": rel(args.approach_report_source_dir),
        "queries_considered": plans,
    }
    write_json(output_dir / "selected_candidate_plan.json", payload)
    return payload


def action_result_summary(run_dir: Path) -> dict[str, Any]:
    room = read_json(run_dir / "room_segment_result.json", {})
    approach = read_json(run_dir / "approach_position_segment_result.json", {})
    yaw = read_json(run_dir / "yaw_alignment_segment_result.json", {})
    return {
        "room_route": {
            "returncode": (room.get("task14c3_segment_summary") or {}).get("returncode"),
            "terminal_reached": room.get("terminal_reached"),
            "clean_runtime_success": room.get("clean_runtime_success"),
            "sparse_fallback_used": room.get("sparse_fallback_used"),
            "failure_layer": room.get("failure_layer"),
            "failure_reason": room.get("failure_reason"),
        },
        "approach_position": {
            "compute_path_success": (approach.get("compute_path_to_pose") or {}).get("success"),
            "follow_path_success": (approach.get("follow_path") or {}).get("success"),
            "navigate_to_pose_fallback_success": (approach.get("navigate_to_pose_fallback") or {}).get("success"),
            "failure_layer": approach.get("failure_layer"),
            "failure_reason": approach.get("failure_reason"),
        },
        "yaw_alignment": {
            "method": yaw.get("yaw_alignment_method"),
            "orientation_only_success": (yaw.get("orientation_only_navigate_to_pose") or {}).get("success"),
            "short_follow_path_success": (yaw.get("short_follow_path_same_xy") or {}).get("success"),
            "direct_cmd_vel_success": (yaw.get("direct_cmd_vel_yaw_alignment") or {}).get("success"),
            "failure_reason": yaw.get("yaw_alignment_failure_reason"),
        },
    }


def normalized_runtime_result(plan: dict[str, Any], run_dir: Path, bringup: dict[str, Any], runtime: dict[str, Any] | None) -> dict[str, Any]:
    if runtime is None:
        return {
            "query": plan.get("query"),
            "object_id": plan.get("resolved_object_id"),
            "label": plan.get("label"),
            "target_room_arrival": False,
            "approach_position_reached": False,
            "approach_yaw_aligned": False,
            "object_facing_approach_success": False,
            "nav2_clean_approach_success": False,
            "room_route_sparse_fallback_used": False,
            "yaw_alignment_direct_cmd_vel_fallback_used": False,
            "fallback_used": False,
            "wall_crossing_validation_passed": False,
            "failure_layer": bringup.get("failure_layer") or "bringup_failure",
            "failure_reason": bringup.get("failure_reason") or "runtime command did not produce an object_facing_runtime_result.json",
            "bringup": bringup,
        }
    direct_yaw = bool(runtime.get("yaw_alignment_direct_cmd_vel_fallback_used", runtime.get("direct_cmd_vel_yaw_alignment_used")))
    room_sparse = bool(runtime.get("room_route_sparse_fallback_used"))
    fallback_used = bool(runtime.get("fallback_used") or room_sparse or direct_yaw)
    failure_layer = runtime.get("failure_layer")
    failure_reason = runtime.get("failure_reason")
    if runtime.get("object_facing_approach_success"):
        failure_layer = None
        failure_reason = None
    elif not runtime.get("target_room_arrival"):
        failure_layer = failure_layer or "target-room navigation failure"
    elif not runtime.get("approach_position_reached"):
        failure_layer = failure_layer or "approach position navigation failure"
    elif not runtime.get("approach_yaw_aligned"):
        failure_layer = failure_layer or "yaw alignment failure"
    elif runtime.get("wall_crossing_validation_passed") is False:
        failure_layer = failure_layer or "validation failure"
    return {
        "query": plan.get("query"),
        "object_id": plan.get("resolved_object_id"),
        "label": plan.get("label"),
        "floor_id": plan.get("floor_id"),
        "room_id": plan.get("room_id"),
        "approach_candidate_id": plan.get("selected_approach_candidate_id"),
        "runtime_output_dir": rel(run_dir),
        "per_query_runtime_json": rel(run_dir / "object_facing_runtime_result.json"),
        "target_room_arrival": bool(runtime.get("target_room_arrival")),
        "approach_position_reached": bool(runtime.get("approach_position_reached")),
        "approach_yaw_aligned": bool(runtime.get("approach_yaw_aligned")),
        "object_facing_approach_success": bool(runtime.get("object_facing_approach_success")),
        "nav2_clean_approach_success": bool(runtime.get("nav2_clean_approach_success")),
        "room_route_sparse_fallback_used": room_sparse,
        "yaw_alignment_direct_cmd_vel_fallback_used": direct_yaw,
        "fallback_used": fallback_used,
        "final_distance_to_target_room_terminal_m": runtime.get("final_distance_to_target_room_terminal_m", runtime.get("final_distance_to_room14_terminal_m")),
        "final_distance_to_approach_candidate_m": runtime.get("final_distance_to_selected_approach_candidate_m", runtime.get("final_distance_to_approach_position_m")),
        "final_yaw_error_rad": runtime.get("yaw_error_rad"),
        "final_yaw_error_deg": runtime.get("yaw_error_deg"),
        "yaw_tolerance_rad": runtime.get("yaw_alignment_tolerance_rad"),
        "xy_drift_during_yaw_alignment_m": runtime.get("xy_drift_during_yaw_alignment_m"),
        "wall_crossing_validation_passed": bool(runtime.get("wall_crossing_validation_passed")),
        "controller_action_result_summary": action_result_summary(run_dir),
        "failure_layer": failure_layer,
        "failure_reason": failure_reason,
        "bringup": bringup,
        "reset_policy": "fresh headless Gazebo/Nav2 bringup per query; no Gazebo reset was used",
    }


def run_runtime_probe(args: argparse.Namespace, output_dir: Path, plan: dict[str, Any]) -> dict[str, Any]:
    object_id = plan["resolved_object_id"]
    run_dir = output_dir / "runtime_runs" / object_id
    run_dir.mkdir(parents=True, exist_ok=True)
    bringup_dir = run_dir / "bringup_logs"
    readiness_json = run_dir / "runtime_bringup_readiness.json"
    readiness_md = run_dir / "runtime_bringup_readiness.md"
    bringup_cmd = [
        str(ROOT / "tools/stage1_runtime/launch_scene_gazebo_nav2.sh"),
        "--scene-id",
        SCENE_ID,
        "--floor-id",
        "floor_2",
        "--stage-output-dir",
        str(args.stage_output_dir),
        "--runtime-profile",
        str(args.runtime_profile),
        "--headless",
        "--log-dir",
        str(bringup_dir),
        "--readiness-timeout-sec",
        str(args.readiness_timeout_sec),
        "--readiness-output-json",
        str(readiness_json),
        "--readiness-output-md",
        str(readiness_md),
        "--run-id",
        f"{TASK_NAME}_{object_id}",
    ]
    bringup_rc = run_logged(bringup_cmd, output_dir / "run_logs" / f"bringup_{object_id}.log")
    bringup_payload = read_json(readiness_json, {})
    bringup = {
        "attempted": True,
        "returncode": bringup_rc,
        "succeeded": bringup_rc == 0 and bool(bringup_payload.get("readiness_succeeded", True)),
        "command": bringup_cmd,
        "readiness_json": rel(readiness_json),
        "readiness_md": rel(readiness_md),
        "bringup_log_dir": rel(bringup_dir),
        "failure_layer": None,
        "failure_reason": None,
    }
    if bringup_rc != 0:
        bringup["failure_layer"] = "bringup_failure"
        bringup["failure_reason"] = f"headless Gazebo/Nav2 bringup returned {bringup_rc}"
        normalized = normalized_runtime_result(plan, run_dir, bringup, None)
        write_json(output_dir / f"per_query_runtime_{object_id}.json", normalized)
        return normalized

    cmd = [
        "/usr/bin/python3",
        str(ROOT / "tools/object_nav/run_objectnav_single_object_runtime.py"),
        "--stage-output-dir",
        str(args.stage_output_dir),
        "--task14c-output-dir",
        str(output_dir),
        "--output-dir",
        str(run_dir),
        "--query",
        plan["query"],
        "--object-id",
        object_id,
        "--approach-candidate-id",
        plan["selected_approach_candidate_id"],
        "--floor-id",
        plan["floor_id"],
        "--start-room",
        args.start_room,
        "--target-room",
        plan["room_id"],
        "--stable-map",
        str(args.stable_map),
        "--semantic-route",
        str(args.semantic_route),
        "--room-route",
        str(args.executable_route),
        "--controller-profile",
        "task12_robust",
        "--execution-strategy",
        "split_follow_path_with_current_pose_gateway_handoff",
        "--two-segment-execution",
        "--object-facing-approach",
        "--position-first-approach",
        "--yaw-alignment-required",
        "--approach-position-tolerance-m",
        str(args.approach_position_tolerance_m),
        "--yaw-alignment-tolerance-rad",
        str(args.yaw_alignment_tolerance_rad),
        "--validate-wall-crossing",
        "--record-sent-controller-paths",
        "--record-executed-trajectory",
    ]
    write_json(run_dir / "task14d_runtime_command.json", {"command": cmd, "created_utc": utc_now()})
    runtime_rc = run_logged(cmd, output_dir / "run_logs" / f"runtime_probe_{object_id}.log")
    runtime = read_json(run_dir / "object_facing_runtime_result.json", None)
    normalized = normalized_runtime_result(plan, run_dir, bringup, runtime)
    normalized["runtime_returncode"] = runtime_rc
    write_json(output_dir / f"per_query_runtime_{object_id}.json", normalized)
    return normalized


def write_markdown_report(path: Path, summary: dict[str, Any]) -> None:
    rows = summary["per_query_results"]
    table = [
        "| query | object_id | candidate | target_room | approach | yaw | object_facing | nav2_clean | direct_yaw | failure_layer | failure_reason |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        table.append(
            "| "
            + " | ".join(
                str(value)
                for value in [
                    row.get("query"),
                    row.get("object_id"),
                    row.get("approach_candidate_id"),
                    row.get("target_room_arrival"),
                    row.get("approach_position_reached"),
                    row.get("approach_yaw_aligned"),
                    row.get("object_facing_approach_success"),
                    row.get("nav2_clean_approach_success"),
                    row.get("yaw_alignment_direct_cmd_vel_fallback_used"),
                    row.get("failure_layer"),
                    row.get("failure_reason"),
                ]
            )
            + " |"
        )
    lines = [
        "# task14d Multi-Object Object-Facing Navigation Validation",
        "",
        f"- Object queries considered: `{summary['object_queries_considered']}`",
        f"- Object queries resolved: `{summary['object_queries_resolved']}`",
        f"- Route-available queries: `{summary['route_available_queries']}`",
        f"- Approach-candidate-available queries: `{summary['approach_candidate_available_queries']}`",
        f"- Runtime probes attempted: `{summary['runtime_probes_attempted']}`",
        f"- Target-room arrivals: `{summary['target_room_arrivals']}`",
        f"- Approach positions reached: `{summary['approach_positions_reached']}`",
        f"- Yaw-aligned successes: `{summary['yaw_aligned_successes']}`",
        f"- Object-facing approach successes: `{summary['object_facing_approach_successes']}`",
        f"- Nav2-clean approach successes: `{summary['nav2_clean_approach_successes']}`",
        f"- Direct cmd_vel yaw alignment fallback uses: `{summary['direct_cmd_vel_yaw_alignment_fallback_uses']}`",
        f"- GUI/RViz validation claimed: `{summary['gui_rviz_validation_claimed']}`",
        "",
        "## Per-Query Results",
        "",
        "\n".join(table),
        "",
        "## Allowed Claims",
        "",
        *[f"- {claim}" for claim in summary["allowed_claims"]],
        "",
        "## Forbidden Claims",
        "",
        *[f"- {claim}" for claim in summary["forbidden_claims"]],
        "",
    ]
    write_text(path, "\n".join(lines))


def build_summary(output_dir: Path, plan_payload: dict[str, Any], runtime_results: list[dict[str, Any]]) -> dict[str, Any]:
    plans = plan_payload["queries_considered"]
    summary = {
        "artifact_type": "task14d_multi_object_object_facing_validation_summary",
        "created_utc": utc_now(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "active_clean_rerun_root": rel(ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"),
        "object_queries_considered": len(plans),
        "object_queries_resolved": sum(1 for p in plans if p.get("resolved")),
        "route_available_queries": sum(1 for p in plans if (p.get("route_availability") or {}).get("available")),
        "approach_candidate_available_queries": sum(1 for p in plans if p.get("approach_candidate_available")),
        "runtime_probes_attempted": len(runtime_results),
        "target_room_arrivals": sum(1 for r in runtime_results if r.get("target_room_arrival")),
        "approach_positions_reached": sum(1 for r in runtime_results if r.get("approach_position_reached")),
        "yaw_aligned_successes": sum(1 for r in runtime_results if r.get("approach_yaw_aligned")),
        "object_facing_approach_successes": sum(1 for r in runtime_results if r.get("object_facing_approach_success")),
        "nav2_clean_approach_successes": sum(1 for r in runtime_results if r.get("nav2_clean_approach_success")),
        "direct_cmd_vel_yaw_alignment_fallback_uses": sum(1 for r in runtime_results if r.get("yaw_alignment_direct_cmd_vel_fallback_used")),
        "gui_rviz_validation_claimed": False,
        "visual_object_confirmation_claimed": False,
        "per_query_results": runtime_results,
        "per_query_failure_layers": [
            {"query": r.get("query"), "object_id": r.get("object_id"), "failure_layer": r.get("failure_layer"), "failure_reason": r.get("failure_reason")}
            for r in runtime_results
        ],
        "allowed_claims": [
            "Artifact-backed object query resolution was evaluated for the selected floor_2 room_14 queries.",
            "Runtime target-room navigation, approach-position navigation, and object-facing yaw alignment were validated in Gazebo/Nav2 for successful probes.",
            "Object-facing approach success means the robot reached the selected approach candidate and aligned toward the committed object/proxy target within tolerance.",
            "Nav2-clean approach success is reported separately from direct /cmd_vel yaw-alignment fallback use.",
        ],
        "forbidden_claims": [
            "Semantic ground-truth accuracy.",
            "Open-vocabulary CLIP retrieval.",
            "Visual object found.",
            "Visual object arrival or visual confirmation.",
            "GUI/RViz validation, because no GUI/RViz run is claimed by this task14d runner.",
            "Cross-floor navigation or stair traversal.",
            "Modifications to the 00824 reference baseline.",
        ],
    }
    write_json(output_dir / "object_facing_multi_object_validation_summary.json", summary)
    write_json(output_dir / "per_query_runtime_results.json", {"artifact_type": "task14d_per_query_runtime_results", "created_utc": utc_now(), "results": runtime_results})
    write_markdown_report(output_dir / "object_facing_multi_object_validation_report.md", summary)
    shutil.copyfile(output_dir / "object_facing_multi_object_validation_report.md", output_dir / "completion_summary.md")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage-output-dir", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun")
    parser.add_argument("--task14a-output-dir", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks/task14a_object_nav_experiment_adapter")
    parser.add_argument("--task14b2-output-dir", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks/task14b2_object_approach_candidate_sanity")
    parser.add_argument("--approach-report-source-dir", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks/task14c_multi_query_objectnav_runtime_validation")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks" / TASK_NAME)
    parser.add_argument("--stable-map", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun/maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml")
    parser.add_argument("--semantic-route", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun/routes/room_routes/floor_2_room11_to_room14/semantic_route_waypoints_v0_1.json")
    parser.add_argument("--executable-route", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun/routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json")
    parser.add_argument("--runtime-profile", type=Path, default=ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun/runtime/profiles/floor_2_nav2_task12_controller_robust/runtime_profile.json")
    parser.add_argument("--start-room", default="room_11")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--approach-position-tolerance-m", type=float, default=0.35)
    parser.add_argument("--yaw-alignment-tolerance-rad", type=float, default=0.50)
    parser.add_argument("--skip-runtime", action="store_true")
    args = parser.parse_args()

    output_dir = args.output_dir
    for subdir in ("run_logs", "runtime_runs", "approach_candidate_reports", "approach_candidate_visualizations"):
        (output_dir / subdir).mkdir(parents=True, exist_ok=True)

    plan_payload = build_candidate_plan(args, output_dir)
    runtime_results: list[dict[str, Any]] = []
    commands = {"artifact_type": "task14d_exact_commands", "created_utc": utc_now(), "top_level_command": sys.argv, "probe_commands": []}
    for plan in plan_payload["queries_considered"]:
        if not plan.get("runnable"):
            runtime_results.append(
                {
                    "query": plan.get("query"),
                    "object_id": plan.get("resolved_object_id"),
                    "label": plan.get("label"),
                    "target_room_arrival": False,
                    "approach_position_reached": False,
                    "approach_yaw_aligned": False,
                    "object_facing_approach_success": False,
                    "nav2_clean_approach_success": False,
                    "room_route_sparse_fallback_used": False,
                    "yaw_alignment_direct_cmd_vel_fallback_used": False,
                    "fallback_used": False,
                    "wall_crossing_validation_passed": False,
                    "failure_layer": "candidate_not_runnable",
                    "failure_reason": plan.get("reason"),
                }
            )
            continue
        if args.skip_runtime:
            runtime_results.append(
                {
                    "query": plan.get("query"),
                    "object_id": plan.get("resolved_object_id"),
                    "label": plan.get("label"),
                    "failure_layer": "runtime_skipped",
                    "failure_reason": "--skip-runtime was requested",
                    "target_room_arrival": False,
                    "approach_position_reached": False,
                    "approach_yaw_aligned": False,
                    "object_facing_approach_success": False,
                    "nav2_clean_approach_success": False,
                    "room_route_sparse_fallback_used": False,
                    "yaw_alignment_direct_cmd_vel_fallback_used": False,
                    "fallback_used": False,
                    "wall_crossing_validation_passed": False,
                }
            )
            continue
        runtime_results.append(run_runtime_probe(args, output_dir, plan))

    stop_cmd = [
        str(ROOT / "tools/stage1_runtime/launch_scene_gazebo_nav2.sh"),
        "--stage-output-dir",
        str(args.stage_output_dir),
        "--runtime-profile",
        str(args.runtime_profile),
        "--stop",
    ]
    commands["stop_command"] = stop_cmd
    if not args.skip_runtime:
        run_logged(stop_cmd, output_dir / "run_logs" / "final_stop.log")
    write_json(output_dir / "exact_runtime_commands.json", commands)

    summary = build_summary(output_dir, plan_payload, runtime_results)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["object_facing_approach_successes"] >= 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
