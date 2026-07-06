#!/usr/bin/env python3
"""Lightweight planner smoke validation for the RSLG-SLAM static planner core.

Performs simple correctness checks over a directory of generated
``rslg_route_result`` JSON files produced from the current queryset. It does
**not** compute final paper metrics or Table 2/3/4 results. RSLG-SLAM is the
project name; ``BoxFusion`` is only a historical repository path.

Checks:

* every route result validates against the RSLGRouteResult schema/rules
* ``generated_ring_037`` is a blocked legacy candidate (evidence only) and is
  never selected as a runtime goal
* ``vt_1_centerline_e003`` (the non-transition edge) is never used as a
  transition edge
* ``requires_nav2`` / ``requires_amcl`` are false
* ``path_found`` is true for the current seed queries
* ``validation_status`` is passed or explainable
* object queries resolve ``obj_175`` / ``curtain``
* connector/cross-floor queries use ``vt_1_centerline_e001``
* object-centric vs structure-aware counts reported separately
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning.route_result import validate as validate_route_result
from tools.rslg_pipeline.planning.route_planner import (
    OBJECT_QUERY_TYPES,
    STRUCTURE_QUERY_TYPES,
)
from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_ID,
    NON_TRANSITION_EDGE,
    OBJECT_ID,
    OBJECT_LABEL,
    TRUE_TRANSITION_EDGE,
)

CROSS_FLOOR_QUERY_TYPES = {"cross_floor_object", "cross_floor_room", "floor_connector"}
CANONICAL_FALLBACK_MODE = "generic_static_with_canonical_metric_fallback"
DYNAMIC_PATH_SOURCES = {"dynamic_stitched", "dynamic_local_approach"}


def _transition_edges(data: dict[str, Any]) -> list[str]:
    edges: list[str] = []
    for seg in data.get("route_segments") or []:
        if isinstance(seg, dict) and seg.get("segment_type") == "vertical_transition":
            edge = seg.get("transition_edge")
            if edge:
                edges.append(str(edge))
    for connector in (data.get("semantic_route") or {}).get("connector_sequence") or []:
        if isinstance(connector, dict) and connector.get("transition_edge"):
            edges.append(str(connector["transition_edge"]))
    return edges


def _check_one(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"path": path.as_posix(), "ok": False, "errors": [f"read/parse: {exc}"]}

    errors: list[str] = []
    schema_ok, schema_errors = validate_route_result(data)
    if not schema_ok:
        errors.extend(f"schema: {e}" for e in schema_errors)

    identity = data.get("identity") or {}
    query_type = identity.get("query_type")
    approach = data.get("approach") or {}
    selected = approach.get("selected_approach") or {}
    selected_goal = selected.get("candidate_id")

    # Blocked candidate never a runtime goal.
    if selected_goal in BLOCKED_LEGACY_APPROACH_IDS:
        errors.append(f"blocked candidate {selected_goal!r} selected as runtime goal")

    # Non-transition edge never used as a transition.
    edges = _transition_edges(data)
    if NON_TRANSITION_EDGE in edges:
        errors.append(f"non-transition edge {NON_TRANSITION_EDGE!r} used as a transition")

    # No Nav2/AMCL.
    runtime = data.get("runtime_interface") or {}
    if runtime.get("requires_nav2") is not False:
        errors.append("requires_nav2 is not false")
    if runtime.get("requires_amcl") is not False:
        errors.append("requires_amcl is not false")

    # path_found expected for the seed queries.
    metric = data.get("metric_path") or {}
    if metric.get("path_found") is not True:
        errors.append("metric_path.path_found is not true")

    # task49e: dynamic stitching must have replaced the canonical metric fallback.
    planner_mode = (data.get("provenance") or {}).get("planner_core", {}).get("planner_mode")
    if planner_mode == CANONICAL_FALLBACK_MODE:
        errors.append(f"planner_mode is retired canonical fallback {CANONICAL_FALLBACK_MODE!r}")

    # metric_path.path_source must be a dynamic source.
    path_source = metric.get("path_source")
    if path_source not in DYNAMIC_PATH_SOURCES:
        errors.append(f"metric_path.path_source {path_source!r} not in {DYNAMIC_PATH_SOURCES}")

    # path_length must be computed and positive.
    path_length = metric.get("path_length")
    if not isinstance(path_length, (int, float)) or path_length <= 0:
        errors.append(f"metric_path.path_length not positive: {path_length!r}")

    # route_segments must carry segment_id and path_found.
    segments = data.get("route_segments") or []
    segment_types = [s.get("segment_type") for s in segments if isinstance(s, dict)]
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            errors.append(f"route_segments[{i}] not an object")
            continue
        if not seg.get("segment_id"):
            errors.append(f"route_segments[{i}] missing segment_id")
        if "path_found" not in seg:
            errors.append(f"route_segments[{i}] missing path_found")

    # Cross-floor route results need at least one connector_handoff and one
    # same_floor_metric segment.
    connector_handoff_count = segment_types.count("connector_handoff")
    same_floor_metric_count = segment_types.count("same_floor_metric")
    if query_type in CROSS_FLOOR_QUERY_TYPES:
        if connector_handoff_count < 1:
            errors.append("cross-floor route result missing a connector_handoff segment")
        if same_floor_metric_count < 1:
            errors.append("cross-floor route result missing a same_floor_metric segment")

    # Object route results must select generated_ring_002.
    if query_type in OBJECT_QUERY_TYPES:
        if selected_goal != CURRENT_OBJECT_APPROACH_ID:
            errors.append(
                f"object route did not select {CURRENT_OBJECT_APPROACH_ID!r} "
                f"(got {selected_goal!r})"
            )

    # blocked_candidate_rejection: generated_ring_037 only ever rejected.
    query_id = identity.get("query_id") or ""
    rejected_ids = {
        r.get("candidate_id")
        for r in approach.get("rejected_candidates") or []
        if isinstance(r, dict)
    }
    if "blocked_candidate_rejection" in query_id:
        for blocked in BLOCKED_LEGACY_APPROACH_IDS:
            if blocked not in rejected_ids:
                errors.append(f"blocked-rejection route missing {blocked!r} in rejected candidates")

    # Canonical comparison, if present, must be provenance/comparison only.
    comparison = (data.get("provenance") or {}).get("planner_core", {}).get("canonical_comparison")
    if isinstance(comparison, dict) and comparison.get("role") != "comparison_provenance_only":
        errors.append("canonical_comparison present but not marked comparison_provenance_only")

    # validation_status must be passed or explainable.
    vstatus = (data.get("validation") or {}).get("validation_status")
    if vstatus not in {"passed", "partial", "not_validated"}:
        if vstatus == "failed" and not (data.get("validation") or {}).get("failure_reason"):
            errors.append("validation_status failed without failure_reason")

    group = (
        "object_centric"
        if query_type in OBJECT_QUERY_TYPES
        else "structure_aware"
        if query_type in STRUCTURE_QUERY_TYPES
        else "unknown"
    )

    # Object queries resolve obj_175 / curtain.
    object_resolved = None
    if query_type in OBJECT_QUERY_TYPES:
        target = data.get("target") or {}
        object_resolved = (
            target.get("target_object_id") == OBJECT_ID
            or target.get("target_object_category") == OBJECT_LABEL
        )
        if not object_resolved:
            errors.append("object query did not resolve obj_175/curtain")

    # Cross-floor / connector queries use the true transition edge.
    connector_edge_ok = None
    if query_type in CROSS_FLOOR_QUERY_TYPES:
        connector_edge_ok = TRUE_TRANSITION_EDGE in edges
        if not connector_edge_ok:
            errors.append(f"cross-floor query missing transition edge {TRUE_TRANSITION_EDGE!r}")

    return {
        "path": path.as_posix(),
        "query_id": identity.get("query_id"),
        "query_type": query_type,
        "group": group,
        "selected_runtime_goal": selected_goal,
        "transition_edges": edges,
        "path_found": metric.get("path_found"),
        "path_source": path_source,
        "path_length": path_length,
        "connector_handoff_count": connector_handoff_count,
        "same_floor_metric_count": same_floor_metric_count,
        "validation_status": vstatus,
        "object_resolved": object_resolved,
        "connector_edge_ok": connector_edge_ok,
        "planner_mode": planner_mode,
        "ok": not errors,
        "errors": errors,
    }


def run_smoke(route_results_dir: Path, manifest_path: Path | None) -> dict[str, Any]:
    files = sorted(p for p in route_results_dir.glob("*route_result*.json") if p.is_file())
    checks = [_check_one(p) for p in files]

    object_count = sum(1 for c in checks if c.get("group") == "object_centric")
    structure_count = sum(1 for c in checks if c.get("group") == "structure_aware")
    all_ok = bool(checks) and all(c["ok"] for c in checks)

    expected_total = None
    if manifest_path and manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        expected_total = len(manifest.get("queries") or [])

    count_ok = expected_total is None or len(checks) == expected_total

    return {
        "schema_name": "rslg_planner_smoke_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "route_results_dir": route_results_dir.as_posix(),
        "route_results_found": len(checks),
        "expected_total": expected_total,
        "count_matches_manifest": count_ok,
        "object_centric_query_count": object_count,
        "structure_aware_query_count": structure_count,
        "generated_ring_037_guard_status": "passed"
        if all(c.get("selected_runtime_goal") not in BLOCKED_LEGACY_APPROACH_IDS for c in checks)
        else "failed",
        "vt_1_centerline_e003_guard_status": "passed"
        if all(NON_TRANSITION_EDGE not in c.get("transition_edges", []) for c in checks)
        else "failed",
        "canonical_metric_fallback_status": "removed"
        if all(c.get("planner_mode") != CANONICAL_FALLBACK_MODE for c in checks)
        else "still_present",
        "metric_path_source_summary": {
            src: sum(1 for c in checks if c.get("path_source") == src)
            for src in sorted({c.get("path_source") for c in checks if c.get("path_source")})
        },
        "planner_mode_summary": {
            mode: sum(1 for c in checks if c.get("planner_mode") == mode)
            for mode in sorted({c.get("planner_mode") for c in checks if c.get("planner_mode")})
        },
        "all_route_results_valid": all_ok,
        "checks": checks,
        "ok": all_ok and count_ok,
        "classification": "planner_smoke_passed" if (all_ok and count_ok) else "planner_smoke_failed",
        "note": "lightweight smoke validation only; no final paper tables computed",
    }


def _render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# RSLG-SLAM Planner Smoke Validation",
        "",
        f"- route_results_dir: `{summary['route_results_dir']}`",
        f"- route_results_found: {summary['route_results_found']} (expected {summary['expected_total']})",
        f"- object_centric_query_count: {summary['object_centric_query_count']}",
        f"- structure_aware_query_count: {summary['structure_aware_query_count']}",
        f"- generated_ring_037_guard_status: `{summary['generated_ring_037_guard_status']}`",
        f"- vt_1_centerline_e003_guard_status: `{summary['vt_1_centerline_e003_guard_status']}`",
        f"- all_route_results_valid: {summary['all_route_results_valid']}",
        f"- classification: `{summary['classification']}`",
        "",
        "| query_id | query_type | group | runtime_goal | path_found | validation | planner_mode | ok |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for c in summary["checks"]:
        lines.append(
            f"| `{c.get('query_id')}` | `{c.get('query_type')}` | {c.get('group')} | "
            f"`{c.get('selected_runtime_goal')}` | {c.get('path_found')} | "
            f"{c.get('validation_status')} | `{c.get('planner_mode')}` | {c.get('ok')} |"
        )
    lines += ["", "This is lightweight smoke validation only; no final paper tables are computed.", ""]
    return "\n".join(lines)


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--queryset-manifest", required=False, default=None)
    parser.add_argument("--route-results-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(list(argv) if argv is not None else None)

    route_dir = Path(args.route_results_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = Path(args.queryset_manifest) if args.queryset_manifest else None

    summary = run_smoke(route_dir, manifest)
    (output_dir / "planner_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=False) + "\n", encoding="utf-8"
    )
    (output_dir / "planner_smoke_summary.md").write_text(
        _render_markdown(summary) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=False))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
