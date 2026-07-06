#!/usr/bin/env python3
"""Static export CLI: RSLGRouteResult -> Layer 4 runtime/visualization adapter inputs.

Generates all three RouteResult-derived Layer 4 adapter inputs (PID follower
runtime input, RViz marker input, z-aware overlay input) from one
``rslg_route_result`` JSON or a directory of them. This is a static export: it
never launches ROS, Gazebo, RViz, Nav2, AMCL, or ``map_server``, and refuses to
write under a canonical directory when ``--no-canonical-write`` is set.

Example (single)::

    python -m tools.rslg_pipeline.export_route_result_runtime_inputs \\
        --route-result-json <one_route_result.json> \\
        --output-dir <task_dir>/runtime_adapter_inputs \\
        --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \\
        --no-canonical-write

Example (batch)::

    python -m tools.rslg_pipeline.export_route_result_runtime_inputs \\
        --route-results-dir <task_dir>/route_results \\
        --output-dir <task_dir>/runtime_adapter_inputs \\
        --floor-z-map '{"floor_1": 0.0, "floor_2": 1.6}' \\
        --no-canonical-write
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.rslg_pipeline.runtime.route_result_adapter_common import (
    DEFAULT_FLOOR_Z_MAP,
    AdapterError,
    is_route_result,
)
from tools.rslg_pipeline.runtime.route_result_marker_adapter import build_rviz_marker_input
from tools.rslg_pipeline.runtime.route_result_runtime_adapter import build_pid_runtime_input
from tools.rslg_pipeline.runtime.route_result_z_aware_adapter import build_z_aware_overlay_input

PID_SUBDIR = "pid_follower_inputs"
MARKER_SUBDIR = "rviz_marker_inputs"
Z_AWARE_SUBDIR = "z_aware_overlay_inputs"
INDEX_NAME = "00_runtime_adapter_index.json"
SUMMARY_NAME = "01_runtime_adapter_summary.md"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def _parse_floor_z_map(raw: Optional[str]) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(k): float(v) for k, v in json.loads(raw).items()}


def _export_one(
    route_result_path: Path,
    output_dir: Path,
    floor_z_map: dict[str, float],
) -> dict[str, Any]:
    route_result = read_json(route_result_path)
    if not is_route_result(route_result):
        raise AdapterError(
            f"{route_result_path} is not an rslg_route_result (schema_name={route_result.get('schema_name')!r})"
        )
    query_id = (route_result.get("identity") or {}).get("query_id") or route_result_path.stem
    source_ref = route_result_path.as_posix()

    pid_payload = build_pid_runtime_input(route_result, floor_z_map=floor_z_map, source_ref=source_ref)
    marker_payload = build_rviz_marker_input(route_result, floor_z_map=floor_z_map, source_ref=source_ref)
    overlay_payload = build_z_aware_overlay_input(route_result, floor_z_map=floor_z_map, source_ref=source_ref)

    pid_path = output_dir / PID_SUBDIR / f"{query_id}_pid_runtime_input.json"
    marker_path = output_dir / MARKER_SUBDIR / f"{query_id}_rviz_marker_input.json"
    overlay_path = output_dir / Z_AWARE_SUBDIR / f"{query_id}_z_aware_overlay_input.json"
    write_json(pid_path, pid_payload)
    write_json(marker_path, marker_payload)
    write_json(overlay_path, overlay_payload)

    return {
        "query_id": query_id,
        "query_type": (route_result.get("identity") or {}).get("query_type"),
        "source_route_result": source_ref,
        "pid_follower_input": pid_path.as_posix(),
        "rviz_marker_input": marker_path.as_posix(),
        "z_aware_overlay_input": overlay_path.as_posix(),
        "runtime_segment_count": len(pid_payload["runtime_segments"]),
        "runtime_waypoint_count": pid_payload["runtime_waypoint_count"],
        "connector_handoff_count": len(pid_payload["connector_handoffs"]),
        "marker_count": marker_payload["marker_count"],
        "overlay_waypoint_count": overlay_payload["overlay_waypoint_count"],
        "overlay_z_min": overlay_payload["z_min"],
        "overlay_z_max": overlay_payload["z_max"],
        "selected_goal_candidate_id": (pid_payload.get("selected_goal") or {}).get("candidate_id")
        if pid_payload.get("selected_goal")
        else None,
        "blocked_legacy_rejected_present": bool(pid_payload["rejected_runtime_candidates"]),
    }


def _render_markdown(index: dict[str, Any]) -> str:
    lines = [
        "# RSLGRouteResult -> Layer 4 Runtime/Visualization Adapter Inputs",
        "",
        "- project: **RSLG-SLAM**",
        f"- generated_utc: `{index['generated_utc']}`",
        f"- output_dir: `{index['output_dir']}`",
        f"- floor_z_map: `{index['floor_z_map']}`",
        f"- route_results_processed: {index['route_results_processed']}",
        f"- classification: `{index['classification']}`",
        "",
        "Layer 3 emits the formal `rslg_route_result`. These are RouteResult-derived",
        "Layer 4 adapter inputs only (PID follower input, RViz marker input, z-aware",
        "overlay input). They are not planner schemas. No Nav2 / AMCL / map_server.",
        "Floor z values are visualization-only; the connector handoff is a semantic /",
        "visualization-only handoff and is not physical stair climbing.",
        "",
        "| query_id | query_type | pid_wps | segments | handoffs | markers | overlay z_min/z_max | goal | ring_037_rejected |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for item in index["results"]:
        lines.append(
            f"| `{item['query_id']}` | `{item['query_type']}` | {item['runtime_waypoint_count']} | "
            f"{item['runtime_segment_count']} | {item['connector_handoff_count']} | {item['marker_count']} | "
            f"`{item['overlay_z_min']}`/`{item['overlay_z_max']}` | `{item['selected_goal_candidate_id']}` | "
            f"{item['blocked_legacy_rejected_present']} |"
        )
    lines.append("")
    return "\n".join(lines)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--route-result-json", type=Path, default=None)
    source.add_argument("--route-results-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args(argv)

    if args.no_canonical_write and "canonical" in args.output_dir.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    floor_z_map = _parse_floor_z_map(args.floor_z_map)
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.route_result_json is not None:
        route_result_paths = [args.route_result_json]
    else:
        route_result_paths = sorted(args.route_results_dir.glob("*_route_result.json"))
        if not route_result_paths:
            route_result_paths = sorted(args.route_results_dir.glob("*.json"))

    results: list[dict[str, Any]] = []
    errors: list[dict[str, Any]] = []
    for path in route_result_paths:
        try:
            results.append(_export_one(path, output_dir, floor_z_map))
        except (AdapterError, OSError, json.JSONDecodeError) as exc:
            errors.append({"path": path.as_posix(), "error": str(exc)})

    ok = not errors and bool(results)
    index = {
        "schema_name": "rslg_runtime_adapter_index",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "generated_utc": utc_now(),
        "output_dir": output_dir.as_posix(),
        "floor_z_map": floor_z_map,
        "route_results_processed": len(results),
        "classification": "runtime_adapter_export_passed" if ok else "runtime_adapter_export_failed",
        "ok": ok,
        "results": results,
        "errors": errors,
    }
    write_json(output_dir / INDEX_NAME, index)
    (output_dir / SUMMARY_NAME).write_text(_render_markdown(index), encoding="utf-8")
    print(json.dumps({k: index[k] for k in ("classification", "route_results_processed", "ok", "errors")}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
