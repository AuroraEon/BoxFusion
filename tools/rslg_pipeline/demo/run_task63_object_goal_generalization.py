#!/usr/bin/env python3
"""RSLG-SLAM task63 object-goal generalization runner (dry-run + reporting).

Consumes the task63 selected goals, RouteResult generation summary, and runtime
adapter generation summary, and produces a per-goal dry-run classification for
every selected object goal. A dry-run passes when the RouteResult and PID runtime
input exist on disk, the runtime path has waypoints, the guardrails hold
(``generated_ring_037`` never a runtime goal, ``vt_1_centerline_e003`` never a
transition edge, ``physical_stair_climbing_claim`` false), and cross-floor routes
use the valid transition edge ``vt_1_centerline_e001``.

This is a pure-static classifier: it never launches ROS/Gazebo/RViz/Nav2/AMCL/
map_server and never runs Stage-A or raw RGB-D inference. RSLG-SLAM is the
project name; ``BoxFusion`` is only a historical repository path.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
VALID_TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"
FORBIDDEN_RUNTIME_GOAL = "generated_ring_037"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def abspath(rel: str | None) -> Path | None:
    if not rel:
        return None
    p = Path(rel)
    return p if p.is_absolute() else REPO_ROOT / p


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selected-json", required=True)
    parser.add_argument("--route-summary-json", required=True)
    parser.add_argument("--adapter-summary-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-md", required=True)
    args = parser.parse_args()

    sel = read_json(Path(args.selected_json))
    route = read_json(Path(args.route_summary_json))
    adapter = read_json(Path(args.adapter_summary_json))
    route_by_id = {r["generated_query_id"]: r for r in route["records"]}
    adapter_by_id = {r["generated_query_id"]: r for r in adapter["records"]}

    results: list[dict[str, Any]] = []
    for goal in sel["selected_goals"]:
        gid = goal["generated_query_id"]
        rr = route_by_id.get(gid, {})
        ar = adapter_by_id.get(gid, {})
        rsum = rr.get("route_summary") or {}

        route_result_path = abspath(rr.get("route_result_json"))
        pid_path = abspath(ar.get("pid_runtime_input_json"))
        route_result_exists = bool(route_result_path and route_result_path.is_file())
        runtime_input_exists = bool(pid_path and pid_path.is_file())

        floor_seq = rsum.get("floor_sequence") or []
        transition_edges = rsum.get("transition_edges") or []
        is_cross_floor = goal["is_cross_floor"]
        waypoint_count = rsum.get("waypoint_count") or 0

        checks = {
            "route_result_exists": route_result_exists,
            "runtime_input_exists": runtime_input_exists,
            "has_waypoints": waypoint_count > 0,
            "no_forbidden_transition_edge": NON_TRANSITION_EDGE not in transition_edges,
            "physical_stair_climbing_claim_false": ar.get("physical_stair_climbing_claim") is False,
            "forbidden_goal_not_selected": FORBIDDEN_RUNTIME_GOAL == ar.get("forbidden_runtime_goal_avoided"),
        }
        if is_cross_floor:
            checks["cross_floor_uses_valid_transition_edge"] = VALID_TRANSITION_EDGE in transition_edges

        dry_run_status = "passed" if all(checks.values()) else "failed"

        if not is_cross_floor:
            recommended = "same_floor_pid"
        elif goal["is_reference_case"]:
            recommended = "scripted_stair_transition_reference"
        else:
            recommended = "scripted_stair_transition"

        results.append({
            "query_id": gid,
            "object_id": goal["object_id"],
            "category": goal["category"],
            "room_id": goal["room_id"],
            "floor_id": goal["floor_id"],
            "start_room_id": goal["start_room_id"],
            "start_floor_id": goal["start_floor_id"],
            "route_mode": goal["expected_route_type"],
            "is_reference_case": goal["is_reference_case"],
            "dry_run_status": dry_run_status,
            "route_result_exists": route_result_exists,
            "runtime_input_exists": runtime_input_exists,
            "waypoint_count": waypoint_count,
            "metric_path_length": rsum.get("metric_path_length"),
            "floor_sequence": floor_seq,
            "room_sequence": rsum.get("room_sequence"),
            "transition_edge": transition_edges,
            "selected_approach_candidate": ("generated_ring_002" if goal["is_reference_case"]
                                            else "object_room_centroid_approach"),
            "final_target_object": goal["object_id"],
            "blocked_candidate_avoided": FORBIDDEN_RUNTIME_GOAL,
            "physical_stair_climbing_claim": False,
            "recommended_execution_mode": recommended,
            "skip_reason": None if dry_run_status == "passed" else "one or more dry-run checks failed",
            "checks": checks,
        })

    passed = [r for r in results if r["dry_run_status"] == "passed"]
    summary = {
        "schema_name": "rslg_task63_generalization_dry_run_results",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "total_goals": len(results),
        "dry_run_passed": len(passed),
        "dry_run_failed": len(results) - len(passed),
        "same_floor_passed": sum(1 for r in passed if r["route_mode"] == "same_floor_pid"),
        "cross_floor_passed": sum(1 for r in passed if r["route_mode"] == "scripted_stair_transition"),
        "distinct_categories_passed": len({r["category"] for r in passed}),
        "distinct_rooms_passed": len({r["room_id"] for r in passed}),
        "any_forbidden_transition_edge": any(
            NON_TRANSITION_EDGE in (r["transition_edge"] or []) for r in results
        ),
        "results": results,
    }
    Path(args.output_json).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    lines = ["# task63 Object-Goal Generalization Dry-Run Results", "",
             f"- Total goals: **{summary['total_goals']}**",
             f"- Dry-run passed: **{summary['dry_run_passed']}** | failed: **{summary['dry_run_failed']}**",
             f"- Same-floor passed: **{summary['same_floor_passed']}** | cross-floor passed: **{summary['cross_floor_passed']}**",
             f"- Distinct categories passed: **{summary['distinct_categories_passed']}** | rooms passed: **{summary['distinct_rooms_passed']}**",
             f"- Any forbidden transition edge: **{summary['any_forbidden_transition_edge']}**", "",
             "| query_id | object | category | room | floor | mode | waypoints | status | recommended |",
             "|----------|--------|----------|------|-------|------|-----------|--------|-------------|"]
    for r in results:
        lines.append(
            f"| `{r['query_id']}` | `{r['object_id']}` | {r['category']} | `{r['room_id']}` | "
            f"`{r['floor_id']}` | {r['route_mode']} | {r['waypoint_count']} | {r['dry_run_status']} | "
            f"{r['recommended_execution_mode']} |"
        )
    lines.append("")
    Path(args.output_md).write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps({
        "ok": summary["dry_run_failed"] == 0,
        "dry_run_passed": summary["dry_run_passed"],
        "dry_run_failed": summary["dry_run_failed"],
        "distinct_categories_passed": summary["distinct_categories_passed"],
        "distinct_rooms_passed": summary["distinct_rooms_passed"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
