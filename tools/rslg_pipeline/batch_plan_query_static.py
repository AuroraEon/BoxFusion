#!/usr/bin/env python3
"""Batch static RSLGRouteResult planner for the RSLG-SLAM initial queryset.

Reads a queryset manifest (``configs/rslg_queryset_v0/queryset_manifest.json``),
loads and validates each ``rslg_query_task`` JSON, dispatches it through the
generic static planner core (:mod:`tools.rslg_pipeline.plan_query_static` ->
:mod:`tools.rslg_pipeline.planning.route_planner`), validates each generated
``rslg_route_result``, and writes batch summaries.

This does not launch ROS, Gazebo, RViz, Nav2, AMCL, or map_server, never writes
under canonical, and does not compute final paper tables. It only separates
object-centric and structure-aware queries in its summary.

Example::

    python -m tools.rslg_pipeline.batch_plan_query_static \\
        --queryset-manifest configs/rslg_queryset_v0/queryset_manifest.json \\
        --canonical-root .../canonical \\
        --output-dir .../batch_route_results \\
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

from tools.rslg_pipeline.plan_query_static import (
    OBJECT_QUERY_TYPES,
    STRUCTURE_QUERY_TYPES,
    _is_under_canonical,
    plan_query,
)
from tools.rslg_pipeline.planning.route_result import validate as validate_route_result


def _classify(query_type: str) -> str:
    if query_type in OBJECT_QUERY_TYPES:
        return "object_centric"
    if query_type in STRUCTURE_QUERY_TYPES:
        return "structure_aware"
    return "unknown"


def _render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# RSLG-SLAM Batch Static Planner Summary",
        "",
        f"- queryset: `{summary['queryset_id']}`",
        f"- scene_id: `{summary['scene_id']}`",
        f"- canonical_root: `{summary['canonical_root']}`",
        f"- output_dir: `{summary['output_dir']}`",
        f"- queries_total: {summary['queries_total']}",
        f"- queries_ok: {summary['queries_ok']}",
        f"- object_centric_query_count: {summary['object_centric_query_count']}",
        f"- structure_aware_query_count: {summary['structure_aware_query_count']}",
        f"- all_query_tasks_valid: {summary['all_query_tasks_valid']}",
        f"- all_route_results_valid: {summary['all_route_results_valid']}",
        f"- classification: `{summary['classification']}`",
        "",
        "This is the generic static planner core for the current 00843 scene.",
        "No Nav2/AMCL/map_server runtime dependency. Final paper tables are not",
        "computed here.",
        "",
        "## Object-centric queries",
        "",
        "| query_id | query_type | route_result_valid | route_result |",
        "|---|---|---|---|",
    ]
    for item in summary["results"]:
        if item["group"] != "object_centric":
            continue
        lines.append(
            f"| `{item.get('query_id')}` | `{item.get('query_type')}` | "
            f"{item.get('route_result_valid')} | `{item.get('route_result_path')}` |"
        )
    lines += [
        "",
        "## Structure-aware queries",
        "",
        "| query_id | query_type | route_result_valid | route_result |",
        "|---|---|---|---|",
    ]
    for item in summary["results"]:
        if item["group"] != "structure_aware":
            continue
        lines.append(
            f"| `{item.get('query_id')}` | `{item.get('query_type')}` | "
            f"{item.get('route_result_valid')} | `{item.get('route_result_path')}` |"
        )
    lines.append("")
    return "\n".join(lines)


def run_batch(manifest_path: Path, canonical_root: Path, output_dir: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_dir = manifest_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    results: list[dict[str, Any]] = []
    per_query_index: list[dict[str, Any]] = []
    object_count = 0
    structure_count = 0
    all_queries_valid = True
    all_results_valid = True

    for entry in manifest.get("queries", []):
        query_file = manifest_dir / entry["file"]
        record = plan_query(query_file, canonical_root, output_dir)
        query_type = record.get("query_type") or entry.get("query_type")
        group = _classify(query_type or "")
        if group == "object_centric":
            object_count += 1
        elif group == "structure_aware":
            structure_count += 1

        # Re-validate the generated route result from disk for the summary.
        route_result_valid = None
        route_path = record.get("route_result_path")
        if route_path and Path(route_path).is_file():
            data = json.loads(Path(route_path).read_text(encoding="utf-8"))
            route_result_valid, _ = validate_route_result(data)

        if record.get("stage") == "query_task_validation" and not record.get("ok"):
            all_queries_valid = False
        if route_result_valid is False or (route_path is None):
            all_results_valid = False

        item = {
            "query_id": entry.get("query_id"),
            "query_type": query_type,
            "group": group,
            "query_file": query_file.as_posix(),
            "planner_mode": record.get("planner_mode"),
            "route_result_path": route_path,
            "route_result_valid": route_result_valid,
            "planner_ok": record.get("ok"),
            "stage": record.get("stage"),
            "errors": record.get("errors", []),
        }
        results.append(item)
        per_query_index.append(
            {
                "query_id": entry.get("query_id"),
                "query_type": query_type,
                "group": group,
                "route_result_path": route_path,
                "route_result_valid": route_result_valid,
            }
        )

    queries_ok = sum(1 for r in results if r["planner_ok"] and r["route_result_valid"])
    overall_ok = all_queries_valid and all_results_valid and queries_ok == len(results)
    summary = {
        "schema_name": "rslg_batch_route_result_summary",
        "schema_version": "0.1",
        "queryset_id": manifest.get("queryset_id"),
        "scene_id": manifest.get("scene_id"),
        "canonical_root": canonical_root.as_posix(),
        "output_dir": output_dir.as_posix(),
        "queries_total": len(results),
        "queries_ok": queries_ok,
        "object_centric_query_count": object_count,
        "structure_aware_query_count": structure_count,
        "all_query_tasks_valid": all_queries_valid,
        "all_route_results_valid": all_results_valid,
        "results": results,
        "note": "generic static planner core for current 00843 scene; final paper tables not computed here",
        "ok": overall_ok,
        "classification": "batch_plan_query_static_passed" if overall_ok else "batch_plan_query_static_failed",
    }

    (output_dir / "batch_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=False) + "\n", encoding="utf-8"
    )
    (output_dir / "per_query_index.json").write_text(
        json.dumps({"queries": per_query_index}, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "batch_summary.md").write_text(_render_markdown(summary) + "\n", encoding="utf-8")
    return summary


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--queryset-manifest", required=True)
    parser.add_argument("--canonical-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--no-canonical-write",
        action="store_true",
        help="Fail fast if the output directory would land under canonical.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

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

    summary = run_batch(Path(args.queryset_manifest), Path(args.canonical_root), output_dir)
    print(json.dumps(summary, indent=2, sort_keys=False))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
