from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import collect_scene_manifest


DEFAULT_MANIFEST_GLOB = "runtime_stage1_frozen_evidence/**/manifest.json"
DEFAULT_REPORT_ROOT = Path("stage_a_eval/output/paper_scene_results")


def _discover_scene_roots(scene_roots: Sequence[str], manifest_globs: Sequence[str]) -> Dict[str, Path]:
    discovered: Dict[str, Path] = {}
    for raw_root in scene_roots:
        path = Path(str(raw_root))
        scene_root = path.parent if path.name == "manifest.json" else path
        if not scene_root.exists():
            continue
        manifest_path = scene_root / "manifest.json"
        sequence_name = scene_root.name
        if manifest_path.exists():
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            sequence_name = str(payload.get("sequence_name") or sequence_name)
        discovered.setdefault(sequence_name, scene_root)
    for pattern in manifest_globs:
        for match in sorted(glob.glob(str(pattern), recursive=True)):
            path = Path(match)
            scene_root = path.parent if path.name == "manifest.json" else path
            if not scene_root.exists():
                continue
            manifest_path = scene_root / "manifest.json"
            sequence_name = scene_root.name
            if manifest_path.exists():
                payload = json.loads(manifest_path.read_text(encoding="utf-8"))
                sequence_name = str(payload.get("sequence_name") or sequence_name)
            discovered.setdefault(sequence_name, scene_root)
    return discovered


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def _scene_eval_lookup(backend_eval_results_path: Path | None) -> Dict[str, Dict[str, Any]]:
    if backend_eval_results_path is None or not backend_eval_results_path.exists():
        return {}
    payload = json.loads(backend_eval_results_path.read_text(encoding="utf-8"))
    rows = list(((payload.get("aggregate_summary") or {}).get("scene_task_breakdown") or []))
    return {str(row.get("sequence_name")): dict(row) for row in rows if row.get("sequence_name")}


def _build_scene_row(scene_root: Path, *, scene_eval_lookup: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    manifest = collect_scene_manifest(scene_root)
    runtime = dict(manifest.get("runtime_summary") or {})
    world = dict(manifest.get("world_model_summary") or {})
    compactness = dict(manifest.get("compactness_summary") or {})
    topology_comparison = dict(manifest.get("topology_comparison_summary") or {})
    query_support = dict(manifest.get("query_support_summary") or {})
    scene_eval = dict(scene_eval_lookup.get(str(manifest.get("sequence_name"))) or {})
    return {
        "sequence_name": manifest.get("sequence_name"),
        "scene_id": manifest.get("scene_id"),
        "status": manifest.get("status"),
        "artifact_profile": manifest.get("artifact_profile"),
        "scene_root": str(scene_root),
        "processed_frames": runtime.get("processed_frames"),
        "duration_sec": runtime.get("duration_sec"),
        "average_fps": runtime.get("average_fps"),
        "backend_artifact_size_total_bytes": runtime.get("backend_artifact_size_total_bytes"),
        "artifact_size_total_bytes": runtime.get("artifact_size_total_bytes"),
        "public_room_count": topology_comparison.get("public_room_count", world.get("room_count")),
        "public_edge_count": topology_comparison.get("public_edge_count", world.get("edge_count")),
        "working_room_count": topology_comparison.get("working_room_count"),
        "working_edge_count": topology_comparison.get("working_edge_count"),
        "withheld_room_count": topology_comparison.get("withheld_room_count"),
        "withheld_edge_count": topology_comparison.get("withheld_edge_count"),
        "floor_count": world.get("floor_count"),
        "object_count": world.get("object_count"),
        "object_label_count": query_support.get("object_label_count"),
        "anchor_count": world.get("anchor_count"),
        "vertical_transition_count": world.get("vertical_transition_count"),
        "objects_per_room": compactness.get("objects_per_room"),
        "bytes_per_room": compactness.get("bytes_per_room"),
        "task_count": scene_eval.get("task_count"),
        "task_success_rate": scene_eval.get("task_success_rate"),
        "exact_room_hit_rate": scene_eval.get("exact_room_hit_rate"),
        "exact_floor_hit_rate": scene_eval.get("exact_floor_hit_rate"),
        "route_found_rate": scene_eval.get("route_found_rate"),
        "latency_mean_ms": scene_eval.get("latency_mean_ms"),
        "manifest_path": str(scene_root / "manifest.json"),
        "summary_json": str(scene_root / "logs" / "summary.json"),
        "topology_json": str(scene_root / "logs" / "topology_v0_1.json"),
        "working_vs_committed_json": str(scene_root / "logs" / "working_vs_committed_topology_report_v0_1.json"),
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aggregate paper-facing scene summaries from frozen or rerun Stage-A outputs.")
    parser.add_argument(
        "--scene-root",
        action="append",
        default=[],
        help="Explicit scene root or manifest.json path. Can be passed multiple times.",
    )
    parser.add_argument(
        "--manifest-glob",
        action="append",
        default=[DEFAULT_MANIFEST_GLOB],
        help="Recursive glob for manifest.json discovery.",
    )
    parser.add_argument(
        "--backend-eval-results",
        default="",
        help="Optional backend_eval_results.json path from stage_a_eval/run_backend_eval.py to merge route/query success rates.",
    )
    parser.add_argument(
        "--report-root",
        default=str(DEFAULT_REPORT_ROOT),
        help="Directory where summary JSON/CSV files should be written.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    scene_roots = _discover_scene_roots(args.scene_root, args.manifest_glob)
    backend_eval_results_path = Path(args.backend_eval_results) if str(args.backend_eval_results).strip() else None
    scene_eval_lookup = _scene_eval_lookup(backend_eval_results_path)
    rows = [_build_scene_row(scene_root, scene_eval_lookup=scene_eval_lookup) for _, scene_root in sorted(scene_roots.items())]

    report_root = Path(args.report_root)
    report_root.mkdir(parents=True, exist_ok=True)
    summary_json = report_root / "paper_scene_summary.json"
    summary_csv = report_root / "paper_scene_summary.csv"
    payload = {
        "scene_count": len(rows),
        "backend_eval_results": None if backend_eval_results_path is None else str(backend_eval_results_path),
        "rows": rows,
    }
    summary_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if rows:
        _write_csv(summary_csv, rows, list(rows[0].keys()))
    else:
        _write_csv(summary_csv, [], ["sequence_name"])

    print(json.dumps({"summary_json": str(summary_json), "summary_csv": str(summary_csv), "scene_count": len(rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
