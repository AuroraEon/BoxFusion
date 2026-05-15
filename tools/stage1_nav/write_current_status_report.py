#!/usr/bin/env python3
"""Aggregate current Stage1 stable-map validation evidence."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOM8_CHAIN = ["room_1", "room_3", "room_8", "room_11", "room_7", "room_14", "room_16"]
ROOM8_GATEWAYS = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r8_01",
    "gw_00824_r8_r11_01",
    "gw_00824_r7_r11_02",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01",
]
ROOM15_CHAIN = ["room_1", "room_3", "room_7", "room_15", "room_7", "room_14", "room_16"]
ROOM15_GATEWAYS = [
    "gw_00824_r1_r3_01",
    "gw_00824_r3_r7_01",
    "gw_00824_r7_r15_01",
    "gw_00824_r7_r15_01",
    "gw_00824_r7_r14_01",
    "gw_00824_r14_r16_01",
]
FORBIDDEN = ["r3_r11", "r8_r14", "r14_r15", "r15_r16", "r3_r15", "r7_r16"]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load(path: Path) -> Any:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_read_error": f"{type(exc).__name__}: {exc}"}


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""


def read_run(validation_dir: Path, run_id: str) -> dict[str, Any]:
    run_dir = validation_dir / run_id
    report = load(run_dir / "stable_map_gui_report_v0_1.json") or load(run_dir / "step30s7_request_aware_projection_report_v0_1.json")
    failure = str(report.get("failure_reason") or "")
    return {
        "run_id": run_id,
        "dir": run_dir.as_posix(),
        "report": report,
        "route_query": load(run_dir / "route_query_result_v0_1.json"),
        "waypoints": load(run_dir / "semantic_route_waypoints_v0_1.json"),
        "route_exec": load(run_dir / "route_execution_result_v0_1.json"),
        "trajectory": load(run_dir / "trajectory_sample_result_v0_1.json"),
        "through": load(run_dir / "through_room15_physical_visit_validation_v0_1.json"),
        "terminal": load(run_dir / "room16_terminal_quality_validation_v0_1.json"),
        "wall": load(run_dir / "trajectory_wall_crossing_validation_v0_2.json"),
        "spin": load(run_dir / "local_looping_spin_validation_v0_2.json"),
        "overlay": load(run_dir / "rviz_overlay_manifest_v0_3.json"),
        "lifecycle": load(run_dir / "lifecycle_readiness_report_v0_1.json"),
        "rviz_proc": load(run_dir / "rviz_process_check_v0_1.json"),
        "gazebo_proc": load(run_dir / "gazebo_process_check_v0_1.json"),
        "overlay_log": read_text(run_dir / "rviz_overlay_publisher.log") + read_text(run_dir / "rviz_overlay_publisher_after_validation.log"),
        "process_after_bringup": read_text(run_dir / "process_list_after_bringup_v0_1.txt"),
        "process_after_route": read_text(run_dir / "process_list_after_route_v0_1.txt"),
        "gui_blocked": "DISPLAY is not set" in failure or "GUI cannot open" in failure,
    }


def route_status(run: dict[str, Any], through_room: str | None) -> str:
    if not run["report"] and not run["route_exec"]:
        return "blocked"
    if run["gui_blocked"]:
        return "blocked"
    if run["wall"] and not run["wall"].get("wall_crossing_validation_passed"):
        return "failed"
    if run["spin"] and run["spin"].get("spinning_detected"):
        return "failed"
    if through_room:
        result = (run["through"].get("through_room_results") or {}).get(through_room, {})
        if result and not result.get("visual_through_room_success"):
            return "failed"
        if not result:
            return "partial"
    if run["terminal"] and not run["terminal"].get("terminal_visual_quality_passed"):
        return "partial"
    if run["route_exec"].get("succeeded") or run["report"].get("succeeded"):
        return "passed"
    return "partial" if run["route_exec"] or run["report"] else "blocked"


def topology_ok(run: dict[str, Any], chain: list[str], gateways: list[str]) -> bool:
    route = run["route_query"]
    seq = route.get("room_sequence") or []
    gws = route.get("gateway_sequence") or []
    text = " ".join(seq + gws + [str(w.get("pair_key")) for w in (run["waypoints"].get("waypoints") or [])])
    no_forbidden = all(item not in text for item in FORBIDDEN)
    no_bridge_goals = not bool(run["route_exec"].get("bridge_waypoints_used_as_goals"))
    return seq == chain and gws == gateways and no_forbidden and no_bridge_goals


def marker_count(text: str) -> int | None:
    matches = re.findall(r"marker_count=(\d+)", text)
    return int(matches[-1]) if matches else None


def process_summary(text: str) -> dict[str, Any]:
    names = ["gzclient", "gzserver", "rviz2", "map_server", "controller_server", "planner_server", "bt_navigator", "python3"]
    lines = text.splitlines()
    return {
        "line_count": len(lines),
        "matches": {name: sum(1 for line in lines if name in line) for name in names},
        "filtered_tail": lines[-80:],
    }


def live_cpu_memory_snapshot() -> dict[str, Any]:
    ps = subprocess.run(["ps", "-eo", "pid,pcpu,pmem,comm,args"], text=True, capture_output=True, check=False).stdout
    keys = ["gzclient", "gzserver", "rviz2", "map_server", "controller_server", "planner_server", "bt_navigator", "publish_stage1_step30p1_rviz_overlay"]
    return {
        "created_utc": now_iso(),
        "matching_lines": [line for line in ps.splitlines() if any(key in line for key in keys)],
    }


def worst_status(values: list[str]) -> str:
    if "failed" in values:
        return "failed"
    if "blocked" in values:
        return "blocked" if all(v == "blocked" for v in values) else "partial"
    if "partial" in values:
        return "partial"
    return "passed"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=Path("stage_outputs/stage1_00824_step30p1"))
    parser.add_argument("--room8-run-id", default="room8_lifecycle_ready_gui_check")
    parser.add_argument("--room15-run-id", default="room15_lifecycle_ready_gui_check")
    args = parser.parse_args()

    stage = args.stage_output_dir.resolve()
    validation = stage / "current_validation"
    room8 = read_run(validation, args.room8_run_id)
    room15 = read_run(validation, args.room15_run_id)
    provenance = load(validation / "stable_full_scene_occupancy_map_provenance.json") or load(stage / "maps/stage1_full_scene_occupancy_map_provenance_v0_1.json")
    cross = load(validation / "room15_cross_request_floorplan_consistency.json")

    stable_status = "passed" if all([
        provenance.get("validation_passed"),
        provenance.get("request_dependent") is False,
        provenance.get("through_rooms_used") is False,
        provenance.get("route_room_ids_whitelist_used") is False,
        provenance.get("step30s5_room15_patch_used") is False,
        (provenance.get("room_metrics") or {}).get("room_15", {}).get("stable_free_cells", 0) > 0,
    ]) else ("partial" if provenance else "blocked")
    floorplan_status = cross.get("status") or ("blocked" if not cross else "failed")
    display_status = "passed" if floorplan_status == "passed" and cross.get("primary_rviz_floorplan_is_request_aware_map") is False else floorplan_status
    room8_status = route_status(room8, "room_8")
    room15_status = route_status(room15, "room_15")
    topology_status = "passed" if topology_ok(room8, ROOM8_CHAIN, ROOM8_GATEWAYS) and topology_ok(room15, ROOM15_CHAIN, ROOM15_GATEWAYS) else "failed"

    overlay_source = stage.parents[1] / "tools/stage1_step30p1/publish_stage1_step30p1_rviz_overlay.py"
    overlay_text = read_text(overlay_source)
    lifecycle = {
        "artifact_type": "marker_lifecycle_report",
        "created_utc": now_iso(),
        "deleteall_in_overlay_source": "DELETEALL" in overlay_text,
        "room8_run_id": room8["overlay"].get("run_id"),
        "room15_run_id": room15["overlay"].get("run_id"),
        "room8_marker_count_from_log": marker_count(room8["overlay_log"]),
        "room15_marker_count_from_log": marker_count(room15["overlay_log"]),
        "marker_topic": "/stage1_nav/semantic_overlay_markers",
        "room8_process_snapshot_has_overlay": "publish_stage1_step30p1_rviz_overlay" in room8["process_after_route"],
        "room15_process_snapshot_has_overlay": "publish_stage1_step30p1_rviz_overlay" in room15["process_after_route"],
    }
    lifecycle["status"] = "passed" if all([
        lifecycle["deleteall_in_overlay_source"],
        lifecycle["room8_run_id"] == args.room8_run_id,
        lifecycle["room15_run_id"] == args.room15_run_id,
        lifecycle["room8_marker_count_from_log"] is not None,
        lifecycle["room15_marker_count_from_log"] is not None,
    ]) else "partial"
    write_json(validation / "marker_lifecycle_report.json", lifecycle)

    gui_blocked = room8["gui_blocked"] or room15["gui_blocked"]
    gui_seen = any([
        room8["gazebo_proc"].get("process_present") and room8["rviz_proc"].get("process_present"),
        room15["gazebo_proc"].get("process_present") and room15["rviz_proc"].get("process_present"),
    ])
    gui_status = "blocked" if gui_blocked else ("passed" if gui_seen else "partial")
    lifecycle_status = "passed" if all([
        room8["lifecycle"].get("succeeded"),
        room15["lifecycle"].get("succeeded"),
        room8["lifecycle"].get("map_received"),
        room15["lifecycle"].get("map_received"),
        (room8["lifecycle"].get("action_servers") or {}).get("/follow_path"),
        (room15["lifecycle"].get("action_servers") or {}).get("/follow_path"),
    ]) else ("blocked" if not room8["lifecycle"] or not room15["lifecycle"] else "failed")
    perf = {
        "artifact_type": "gui_performance_process_report",
        "created_utc": now_iso(),
        "status": gui_status,
        "room8_after_bringup": process_summary(room8["process_after_bringup"]),
        "room8_after_route": process_summary(room8["process_after_route"]),
        "room15_after_bringup": process_summary(room15["process_after_bringup"]),
        "room15_after_route": process_summary(room15["process_after_route"]),
        "cpu_memory_snapshot": live_cpu_memory_snapshot(),
    }
    write_json(validation / "gui_performance_process_report.json", perf)

    topo = {
        "artifact_type": "route_topology_integrity_report",
        "created_utc": now_iso(),
        "status": topology_status,
        "room8_route_matches_expected": topology_ok(room8, ROOM8_CHAIN, ROOM8_GATEWAYS),
        "room15_route_matches_expected": topology_ok(room15, ROOM15_CHAIN, ROOM15_GATEWAYS),
        "forbidden_shortcuts": FORBIDDEN,
        "bridge_waypoints_used_as_goals": {
            "room8": room8["route_exec"].get("bridge_waypoints_used_as_goals"),
            "room15": room15["route_exec"].get("bridge_waypoints_used_as_goals"),
        },
    }
    write_json(validation / "route_topology_integrity_report.json", topo)

    room8_diag = {
        "artifact_type": "room8_local_wall_spinning_diagnostic_evidence",
        "created_utc": now_iso(),
        "status": room8_status,
        "route": room8["route_query"].get("room_sequence"),
        "gateway_sequence": room8["route_query"].get("gateway_sequence"),
        "trajectory_sample_count": len(room8["trajectory"].get("samples") or []),
        "wall": room8["wall"],
        "spin": room8["spin"],
        "through_room8_result": (room8["through"].get("through_room_results") or {}).get("room_8"),
    }
    write_json(validation / "room8_local_wall_spinning_diagnostic_evidence.json", room8_diag)

    statuses = {
        "stable_occupancy_map_status": stable_status,
        "rviz_floorplan_consistency_status": display_status,
        "room8_behavior_status": room8_status,
        "room15_physical_visit_status": room15_status,
        "room15_cross_request_display_status": display_status,
        "marker_lifecycle_status": lifecycle["status"],
        "gui_performance_status": gui_status,
        "nav2_lifecycle_readiness_status": lifecycle_status,
        "route_topology_integrity_status": topology_status,
    }
    statuses["overall_status"] = worst_status(list(statuses.values()))

    status_report = {
        "artifact_type": "current_stage1_nav_status_report",
        "created_utc": now_iso(),
        **statuses,
        "run_ids": [args.room8_run_id, args.room15_run_id],
        "evidence_root": validation.as_posix(),
        "active_user_facing_runtime": "tools/stage1_nav",
        "current_validation_dir": validation.as_posix(),
        "stable_map_yaml": (stage / "maps/stage1_full_scene_occupancy_map.yaml").as_posix(),
        "follow_path_runtime_action_gates": {
            "hard_blockers": ["/follow_path"],
            "non_blocking_diagnostics": ["/compute_path_to_pose", "/navigate_to_pose"],
        },
        "stable_map_provenance": provenance,
        "evidence_files": {
            "stable_full_scene_occupancy_map_provenance": (validation / "stable_full_scene_occupancy_map_provenance.json").as_posix(),
            "occupancy_map_generation_audit": (validation / "occupancy_map_generation_audit.md").as_posix(),
            "rviz_floorplan_source_audit": (validation / "rviz_floorplan_source_audit.md").as_posix(),
            "room8_run_dir": room8["dir"],
            "room15_run_dir": room15["dir"],
            "room15_cross_request_floorplan_consistency": (validation / "room15_cross_request_floorplan_consistency.json").as_posix(),
            "room8_lifecycle_readiness_report": (validation / args.room8_run_id / "lifecycle_readiness_report_v0_1.json").as_posix(),
            "room15_lifecycle_readiness_report": (validation / args.room15_run_id / "lifecycle_readiness_report_v0_1.json").as_posix(),
            "room8_local_wall_spinning_diagnostic_evidence": (validation / "room8_local_wall_spinning_diagnostic_evidence.json").as_posix(),
            "marker_lifecycle_report": (validation / "marker_lifecycle_report.json").as_posix(),
            "gui_performance_process_report": (validation / "gui_performance_process_report.json").as_posix(),
            "route_topology_integrity_report": (validation / "route_topology_integrity_report.json").as_posix(),
            "command_transcript_or_run_log": (validation / "command_transcript_or_run_log.md").as_posix(),
        },
        "notes": {
            "accepted_step30p1_wording": "Step30P1 repaired execution succeeded with clean forward-only fallback.",
            "object_level_navigation_complete": False,
            "amcl_used": False,
            "manual_cmd_vel_used": False,
            "real_physical_robot_deployment_claim": False,
        },
    }
    write_json(validation / "current_status_report.json", status_report)

    lines = ["# Current Stage1 Navigation Status", ""]
    for key in [
        "stable_occupancy_map_status",
        "rviz_floorplan_consistency_status",
        "room8_behavior_status",
        "room15_physical_visit_status",
        "room15_cross_request_display_status",
        "marker_lifecycle_status",
        "gui_performance_status",
        "nav2_lifecycle_readiness_status",
        "route_topology_integrity_status",
        "overall_status",
    ]:
        lines.append(f"- `{key}`: `{status_report[key]}`")
    lines.extend([
        "",
        "## Active Map",
        "",
        f"- stable map: `{status_report['stable_map_yaml']}`",
        "- primary RViz floorplan: `/map` from the stable full-scene occupancy map",
        "- request-aware map: debug/reference only in the repaired default profile",
        "",
        "## Evidence",
        "",
    ])
    for key, value in status_report["evidence_files"].items():
        lines.append(f"- `{key}`: `{value}`")
    (validation / "current_status_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    for name, payload, title in [
        ("marker_lifecycle_report.md", lifecycle, "Marker Lifecycle Report"),
        ("gui_performance_process_report.md", perf, "GUI Performance / Process Report"),
        ("route_topology_integrity_report.md", topo, "Route Topology Integrity Report"),
        ("room8_local_wall_spinning_diagnostic_evidence.md", room8_diag, "Room8 Local Wall/Spinning Diagnostic Evidence"),
    ]:
        (validation / name).write_text(
            f"# {title}\n\n- status: `{payload.get('status')}`\n- json: `{validation / name.replace('.md', '.json')}`\n",
            encoding="utf-8",
        )

    print(json.dumps(status_report, indent=2, sort_keys=True))
    return 0 if statuses["overall_status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
