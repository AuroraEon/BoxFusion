#!/usr/bin/env python3
"""RSLG-SLAM task63 RouteResult + runtime-adapter builder.

For each selected object goal it produces a schema-consistent
``rslg_route_result`` and the derived PID/RViz/z-aware runtime adapter inputs,
reusing the existing RSLG-SLAM tools (no routing logic is duplicated):

* Reference goal (curtain / obj_175 / room_14): reuses the previously validated
  task56c cross-floor RouteResult and PID runtime input (read-only reference,
  never overwritten).
* All other goals: routes to the *room containing the object* via the generic
  static planner ``tools.rslg_pipeline.plan_query_static`` (``room_gateway`` for
  same-floor, ``cross_floor_room`` for cross-floor), then exports runtime adapter
  inputs via ``tools.rslg_pipeline.export_route_result_runtime_inputs``.

The single-object canonical approach (obj_175 -> generated_ring_002) is not
mis-attributed to other objects: for non-reference goals the object centroid is
the in-room semantic target and the metric path ends at the room approach.

No canonical writes. No ROS/Gazebo/RViz/Nav2/AMCL/map_server/Stage-A.
RSLG-SLAM is the project name; ``BoxFusion`` is only a historical repository path.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]
PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"

TASK56C_ROOT = (
    "stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/"
    "task56c_pid_profile_promotion_and_regression_validation/regression_pack"
)
REFERENCE_QUERY_ID = "00843_cross_floor_object_curtain_room14"
FLOOR_Z_MAP = '{"floor_1": 0.0, "floor_2": 1.6}'
VALID_TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"
FORBIDDEN_RUNTIME_GOAL = "generated_ring_037"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def transition_edges(route_result: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    for seg in route_result.get("route_segments") or []:
        if seg.get("segment_type") in {"vertical_transition", "connector_handoff"}:
            e = seg.get("transition_edge") or seg.get("connector_edge")
            if e:
                edges.append(e)
    for c in (route_result.get("semantic_route") or {}).get("connector_sequence") or []:
        if isinstance(c, dict) and c.get("transition_edge"):
            edges.append(c["transition_edge"])
    return edges


def summarize_route(route_result: dict[str, Any]) -> dict[str, Any]:
    sem = route_result.get("semantic_route") or {}
    mp = route_result.get("metric_path") or {}
    val = route_result.get("validation") or {}
    edges = transition_edges(route_result)
    return {
        "room_sequence": sem.get("room_sequence"),
        "floor_sequence": sem.get("floor_sequence"),
        "vertical_connector_id": sem.get("vertical_connector_id"),
        "transition_edges": edges,
        "waypoint_count": len(mp.get("path") or []),
        "metric_path_length": mp.get("path_length"),
        "path_found": mp.get("path_found"),
        "validation_status": val.get("validation_status"),
        "uses_valid_transition_edge": (VALID_TRANSITION_EDGE in edges) if edges else None,
        "uses_forbidden_transition_edge": NON_TRANSITION_EDGE in edges,
    }


def run_planner(query_path: Path, canonical_root: Path, out_dir: Path) -> tuple[bool, str]:
    cmd = [
        PYTHON, "-m", "tools.rslg_pipeline.plan_query_static",
        "--query-json", query_path.as_posix(),
        "--canonical-root", canonical_root.as_posix(),
        "--output-dir", out_dir.as_posix(),
        "--no-canonical-write",
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    return proc.returncode == 0, (proc.stdout + proc.stderr)


def run_exporter(route_result_path: Path, out_dir: Path) -> tuple[bool, str]:
    cmd = [
        PYTHON, "-m", "tools.rslg_pipeline.export_route_result_runtime_inputs",
        "--route-result-json", route_result_path.as_posix(),
        "--output-dir", out_dir.as_posix(),
        "--floor-z-map", FLOOR_Z_MAP,
        "--no-canonical-write",
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    return proc.returncode == 0, (proc.stdout + proc.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selected-json", required=True)
    parser.add_argument("--query-summary-json", required=True)
    parser.add_argument("--canonical-root", required=True)
    parser.add_argument("--route-results-dir", required=True)
    parser.add_argument("--runtime-adapter-dir", required=True)
    parser.add_argument("--route-summary-json", required=True)
    parser.add_argument("--route-summary-md", required=True)
    parser.add_argument("--adapter-summary-json", required=True)
    parser.add_argument("--adapter-summary-md", required=True)
    args = parser.parse_args()

    sel = read_json(Path(args.selected_json))
    qsummary = read_json(Path(args.query_summary_json))
    qrec_by_id = {r["generated_query_id"]: r for r in qsummary["records"]}
    canonical_root = Path(args.canonical_root)
    route_dir = Path(args.route_results_dir)
    adapter_dir = Path(args.runtime_adapter_dir)
    route_dir.mkdir(parents=True, exist_ok=True)
    adapter_dir.mkdir(parents=True, exist_ok=True)

    route_records: list[dict[str, Any]] = []
    adapter_records: list[dict[str, Any]] = []

    for goal in sel["selected_goals"]:
        gid = goal["generated_query_id"]
        qrec = qrec_by_id[gid]
        record: dict[str, Any] = {
            "generated_query_id": gid,
            "object_id": goal["object_id"],
            "category": goal["category"],
            "room_id": goal["room_id"],
            "floor_id": goal["floor_id"],
            "expected_route_type": goal["expected_route_type"],
            "is_reference_case": goal["is_reference_case"],
        }

        if goal["reuse_existing_route"]:
            rr_path = REPO_ROOT / TASK56C_ROOT / "route_results" / f"{REFERENCE_QUERY_ID}_route_result.json"
            pid_path = (REPO_ROOT / TASK56C_ROOT / "runtime_adapter_inputs" /
                        "pid_follower_inputs" / f"{REFERENCE_QUERY_ID}_pid_runtime_input.json")
            if rr_path.is_file() and pid_path.is_file():
                rr = read_json(rr_path)
                record.update({
                    "route_generation_status": "reused_existing_task56c",
                    "route_result_json": rel(rr_path),
                    "route_result_source": "task56c_regression_pack (read-only reference)",
                    "route_summary": summarize_route(rr),
                    "failure_reason": None,
                })
                adapter_records.append({
                    "generated_query_id": gid,
                    "object_id": goal["object_id"],
                    "runtime_input_status": "reused_existing_task56c",
                    "pid_runtime_input_json": rel(pid_path),
                    "route_type": goal["expected_route_type"],
                    "floor_id_filter": None,
                    "physical_stair_climbing_claim": False,
                    "forbidden_runtime_goal_avoided": FORBIDDEN_RUNTIME_GOAL,
                    "uses_forbidden_transition_edge": NON_TRANSITION_EDGE in record["route_summary"]["transition_edges"],
                })
            else:
                record.update({
                    "route_generation_status": "failed",
                    "route_result_json": None,
                    "failure_reason": "reference task56c route/pid artifact not found",
                })
            route_records.append(record)
            continue

        # Generate route via room-target route-gen QueryTask.
        rg_path = Path(qrec["route_gen_query_path"])
        ok, log = run_planner(rg_path, canonical_root, route_dir)
        rr_file = route_dir / f"{rg_path.stem}_route_result.json"
        if not ok or not rr_file.is_file():
            record.update({
                "route_generation_status": "failed",
                "route_result_json": None,
                "failure_reason": f"plan_query_static failed: {log.strip()[-400:]}",
            })
            route_records.append(record)
            continue

        rr = read_json(rr_file)
        rsum = summarize_route(rr)
        record.update({
            "route_generation_status": "generated" if rsum["path_found"] else "generated_no_path",
            "route_result_json": rel(rr_file),
            "route_result_source": "task63 plan_query_static (room-target routing)",
            "route_summary": rsum,
            "object_goal_realized_as": "route_to_room_containing_object",
            "failure_reason": None if rsum["path_found"] else "planner returned no metric path",
        })
        route_records.append(record)

        # Export runtime adapters.
        aok, alog = run_exporter(rr_file, adapter_dir)
        pid_file = adapter_dir / "pid_follower_inputs" / f"{rr['identity']['query_id']}_pid_runtime_input.json"
        floor_filter = goal["floor_id"] if goal["expected_route_type"] == "same_floor_pid" else None
        arec: dict[str, Any] = {
            "generated_query_id": gid,
            "object_id": goal["object_id"],
            "route_type": goal["expected_route_type"],
            "floor_id_filter": floor_filter,
            "physical_stair_climbing_claim": False,
            "forbidden_runtime_goal_avoided": FORBIDDEN_RUNTIME_GOAL,
            "uses_forbidden_transition_edge": rsum["uses_forbidden_transition_edge"],
        }
        if aok and pid_file.is_file():
            pid = read_json(pid_file)
            arec.update({
                "runtime_input_status": "generated",
                "pid_runtime_input_json": rel(pid_file),
                "runtime_waypoint_count": pid.get("runtime_waypoint_count"),
                "requires_nav2": pid.get("requires_nav2"),
                "requires_amcl": pid.get("requires_amcl"),
                "compatible_with_pid_follower": pid.get("compatible_with_pid_follower"),
            })
        else:
            arec.update({
                "runtime_input_status": "failed",
                "pid_runtime_input_json": None,
                "failure_reason": f"exporter failed: {alog.strip()[-300:]}",
            })
        adapter_records.append(arec)

    # Route summary.
    generated = [r for r in route_records if r["route_generation_status"] in {"generated", "reused_existing_task56c"}]
    route_summary = {
        "schema_name": "rslg_task63_route_result_generation_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "route_results_dir": route_dir.as_posix(),
        "total_goals": len(route_records),
        "routes_generated_or_reused": len(generated),
        "routes_new_generated": sum(1 for r in route_records if r["route_generation_status"] == "generated"),
        "routes_reused": sum(1 for r in route_records if r["route_generation_status"] == "reused_existing_task56c"),
        "routes_failed": sum(1 for r in route_records if r["route_generation_status"] == "failed"),
        "any_forbidden_transition_edge": any(
            (r.get("route_summary") or {}).get("uses_forbidden_transition_edge") for r in route_records
        ),
        "records": route_records,
    }
    Path(args.route_summary_json).write_text(json.dumps(route_summary, indent=2) + "\n", encoding="utf-8")

    rlines = ["# task63 RouteResult Generation Summary", "",
              f"- Total goals: **{route_summary['total_goals']}**",
              f"- Routes generated (new): **{route_summary['routes_new_generated']}**",
              f"- Routes reused (task56c reference): **{route_summary['routes_reused']}**",
              f"- Routes failed: **{route_summary['routes_failed']}**",
              f"- Any forbidden transition edge used: **{route_summary['any_forbidden_transition_edge']}**", "",
              "| generated_query_id | object | room | floor | status | rooms | waypoints | transition_edge | valid |",
              "|--------------------|--------|------|-------|--------|-------|-----------|-----------------|-------|"]
    for r in route_records:
        rs = r.get("route_summary") or {}
        rlines.append(
            f"| `{r['generated_query_id']}` | `{r['object_id']}` | `{r['room_id']}` | `{r['floor_id']}` | "
            f"{r['route_generation_status']} | {rs.get('room_sequence')} | {rs.get('waypoint_count')} | "
            f"{rs.get('transition_edges')} | {rs.get('validation_status')} |"
        )
    rlines.append("")
    Path(args.route_summary_md).write_text("\n".join(rlines) + "\n", encoding="utf-8")

    # Adapter summary.
    adapter_summary = {
        "schema_name": "rslg_task63_runtime_adapter_generation_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "runtime_adapter_dir": adapter_dir.as_posix(),
        "profile_id": "practical_zero_collision",
        "profile_json": "configs/rslg_runtime_profiles/pid_profiles_v0_1.json",
        "total": len(adapter_records),
        "generated": sum(1 for r in adapter_records if r["runtime_input_status"] == "generated"),
        "reused": sum(1 for r in adapter_records if r["runtime_input_status"] == "reused_existing_task56c"),
        "failed": sum(1 for r in adapter_records if r["runtime_input_status"] == "failed"),
        "any_forbidden_transition_edge": any(r.get("uses_forbidden_transition_edge") for r in adapter_records),
        "all_physical_stair_climbing_claim_false": all(
            r.get("physical_stair_climbing_claim") is False for r in adapter_records
        ),
        "records": adapter_records,
    }
    Path(args.adapter_summary_json).write_text(json.dumps(adapter_summary, indent=2) + "\n", encoding="utf-8")

    alines = ["# task63 Runtime Adapter Generation Summary", "",
              f"- Profile: `{adapter_summary['profile_id']}` (`{adapter_summary['profile_json']}`)",
              f"- Adapters generated: **{adapter_summary['generated']}** | reused: **{adapter_summary['reused']}** | failed: **{adapter_summary['failed']}**",
              f"- Any forbidden transition edge: **{adapter_summary['any_forbidden_transition_edge']}**",
              f"- physical_stair_climbing_claim false for all: **{adapter_summary['all_physical_stair_climbing_claim_false']}**", "",
              "| generated_query_id | object | route_type | floor_filter | status | waypoints | nav2 | amcl |",
              "|--------------------|--------|------------|--------------|--------|-----------|------|------|"]
    for r in adapter_records:
        alines.append(
            f"| `{r['generated_query_id']}` | `{r['object_id']}` | {r['route_type']} | "
            f"{r.get('floor_id_filter')} | {r['runtime_input_status']} | {r.get('runtime_waypoint_count')} | "
            f"{r.get('requires_nav2')} | {r.get('requires_amcl')} |"
        )
    alines.append("")
    Path(args.adapter_summary_md).write_text("\n".join(alines) + "\n", encoding="utf-8")

    print(json.dumps({
        "ok": route_summary["routes_failed"] == 0 and adapter_summary["failed"] == 0,
        "routes_generated_or_reused": route_summary["routes_generated_or_reused"],
        "routes_failed": route_summary["routes_failed"],
        "adapters_generated": adapter_summary["generated"],
        "adapters_reused": adapter_summary["reused"],
        "adapters_failed": adapter_summary["failed"],
        "any_forbidden_transition_edge": route_summary["any_forbidden_transition_edge"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
