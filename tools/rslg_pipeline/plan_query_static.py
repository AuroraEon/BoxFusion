#!/usr/bin/env python3
"""QueryTask-driven static RSLGRouteResult planner for RSLG-SLAM.

This is the formal static Layer 3 command surface. It reads an ``rslg_query_task``
JSON (``configs/rslg_queryset_v0/*.json``), runs the generic static planner core
(:mod:`tools.rslg_pipeline.planning.route_planner`), and writes a schema-consistent
``rslg_route_result`` JSON under a task-local output directory.

Unlike the retired seed-wrapping dispatch, every query now flows through
``route_planner.plan_static_query`` — target resolution, room/floor route,
connector resolution, and approach selection are performed by the planner-core
modules from canonical Layer 2/3 artifacts, and the metric path is dynamically
stitched from per-floor occupancy A* segments (``metric_path_stitcher``). No
metric-path length is copied from a canonical real route; canonical real routes
appear only as comparison/provenance (``provenance.planner_core.canonical_comparison``).

It never launches ROS, Gazebo, RViz, Nav2, AMCL, or map_server, and never writes
under canonical. RSLG-SLAM is the project name; ``BoxFusion`` is only a historical
repository path.

Formal Layer 3 flow::

    QueryTask JSON -> plan_query_static -> route_planner -> RSLGRouteResult JSON

Example::

    python -m tools.rslg_pipeline.plan_query_static \\
        --query-json configs/rslg_queryset_v0/00843_cross_floor_object_curtain_room14.json \\
        --canonical-root .../canonical \\
        --output-dir .../route_results_single \\
        --no-canonical-write
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.rslg_pipeline.planning import query_task as qt
from tools.rslg_pipeline.planning import route_planner
from tools.rslg_pipeline.planning.route_result import dump, validate

# Query-type groups (object-centric vs structure-aware) for batch summaries.
OBJECT_QUERY_TYPES = route_planner.OBJECT_QUERY_TYPES
STRUCTURE_QUERY_TYPES = route_planner.STRUCTURE_QUERY_TYPES


def _is_under_canonical(output_dir: Path) -> bool:
    parts = [p.lower() for p in output_dir.resolve().parts]
    if "canonical" not in parts:
        return False
    # Task-local outputs live under .../tasks/...; canonical is a sibling root.
    return "tasks" not in parts


def plan_query(query_path: Path, canonical_root: Path, output_dir: Path) -> dict[str, Any]:
    """Plan a single QueryTask file into an RSLGRouteResult on disk."""

    query = qt.load(query_path)
    query_ok, query_errors = qt.validate(query)
    if not query_ok:
        return {
            "query_path": query_path.as_posix(),
            "ok": False,
            "stage": "query_task_validation",
            "errors": query_errors,
        }

    try:
        result = route_planner.plan_static_query(
            query, canonical_root, no_canonical_write=True
        )
    except route_planner.PlannerError as exc:
        return {
            "query_path": query_path.as_posix(),
            "query_id": query.get("query_id"),
            "ok": False,
            "stage": "planning",
            "errors": [str(exc)],
        }

    result_ok, result_errors = validate(result)
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{query['query_id']}_route_result.json"
    out_path.write_text(dump(result), encoding="utf-8")

    planner_mode = (
        (result.get("provenance") or {}).get("planner_core", {}).get("planner_mode")
    )
    return {
        "query_path": query_path.as_posix(),
        "query_id": query["query_id"],
        "query_type": query["query_type"],
        "planner_mode": planner_mode,
        "route_result_path": out_path.as_posix(),
        "route_result_valid": result_ok,
        "errors": result_errors,
        "ok": result_ok,
        "stage": "done",
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--query-json", required=True, help="Path to an rslg_query_task JSON.")
    parser.add_argument("--canonical-root", required=True, help="Read-only canonical root.")
    parser.add_argument("--output-dir", required=True, help="Task-local output directory.")
    parser.add_argument(
        "--no-canonical-write",
        action="store_true",
        help="Fail fast if the output directory would land under canonical.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    canonical_root = Path(args.canonical_root)
    output_dir = Path(args.output_dir)

    if args.no_canonical_write and _is_under_canonical(output_dir):
        print(
            json.dumps(
                {
                    "ok": False,
                    "error": "output-dir resolves under canonical while --no-canonical-write is set",
                    "output_dir": output_dir.as_posix(),
                },
                indent=2,
            )
        )
        return 2

    record = plan_query(Path(args.query_json), canonical_root, output_dir)
    report = {
        "schema_name": "rslg_route_result",
        "canonical_root": canonical_root.as_posix(),
        "output_dir": output_dir.as_posix(),
        "no_canonical_write": args.no_canonical_write,
        "planner_surface": "route_planner.plan_static_query (generic static planner core)",
        "result": record,
        "ok": bool(record.get("ok")),
        "classification": "plan_query_static_passed" if record.get("ok") else "plan_query_static_failed",
    }
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0 if record.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
