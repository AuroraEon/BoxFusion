from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, List, Sequence

from boxfusion.runtime_instrumentation import RuntimeInstrumentation


DEFAULT_SCENE_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_SEQUENCE_IDS = [
    "00862-LT9Jq6dN3Ea",
    "00843-DYehNKdT76V",
    "00829-QaLdnwvtxbs",
]


def read_csv_rows(path: Path) -> List[Dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def maybe_number(value: Any) -> Any:
    if value in (None, "", "None"):
        return None
    text = str(value)
    if text in {"True", "False"}:
        return text == "True"
    try:
        if any(token in text for token in (".", "e", "E")):
            return float(text)
        return int(text)
    except Exception:
        return value


def normalize_rows(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [{key: maybe_number(value) for key, value in row.items()} for row in rows]


def load_segmentation_runs(scene_logs_dir: Path, sequence_id: str) -> List[Dict[str, Any]]:
    diagnostics_path = scene_logs_dir / "floor_diagnostics_summary.json"
    if not diagnostics_path.exists():
        raise FileNotFoundError(f"Missing floor diagnostics summary: {diagnostics_path}")
    diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
    run_rows = list((diagnostics.get("room_segmentation_diagnostics") or {}).get("runs") or [])
    if not run_rows:
        raise ValueError(f"No segmentation runs found in {diagnostics_path}")

    rows: List[Dict[str, Any]] = []
    for row in run_rows:
        row_copy = dict(row)
        row_copy["sequence_id"] = str(sequence_id)
        rows.append(row_copy)
    return rows


def refresh_scene_outputs(scene_output_root: Path, sequence_id: str) -> Dict[str, Any]:
    scene_logs_dir = scene_output_root / sequence_id / "logs"
    runtime_dir = scene_logs_dir / "runtime_instrumentation"
    frame_csv = runtime_dir / "per_profiled_frame.csv"
    export_csv = runtime_dir / "vector_map_export_calls.csv"
    history_csv = runtime_dir / "history_scope.csv"
    summary_json = runtime_dir / "summary.json"
    required_inputs = [frame_csv, export_csv, history_csv]
    missing_inputs = [str(path) for path in required_inputs if not path.exists()]
    if missing_inputs:
        raise FileNotFoundError(f"Missing runtime instrumentation inputs for {sequence_id}: {missing_inputs}")

    segmentation_rows = load_segmentation_runs(scene_logs_dir, sequence_id)
    write_csv(
        runtime_dir / "segmentation_runs.csv",
        segmentation_rows,
        RuntimeInstrumentation.SEGMENTATION_FIELDNAMES,
    )

    frame_rows = normalize_rows(read_csv_rows(frame_csv))
    export_rows = normalize_rows(read_csv_rows(export_csv))
    history_rows = normalize_rows(read_csv_rows(history_csv))
    normalized_segmentation_rows = normalize_rows(
        [
            {field: row.get(field) for field in RuntimeInstrumentation.SEGMENTATION_FIELDNAMES}
            for row in segmentation_rows
        ]
    )
    runtime_profiler = RuntimeInstrumentation(sequence_id=sequence_id, output_dir=runtime_dir)
    summary = runtime_profiler._build_summary(
        profiled_rows=frame_rows,
        export_rows=export_rows,
        history_rows=history_rows,
        segmentation_rows=normalized_segmentation_rows,
    )
    summary_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    scene_summary_path = scene_logs_dir / "summary.json"
    if scene_summary_path.exists():
        scene_summary = json.loads(scene_summary_path.read_text(encoding="utf-8"))
        scene_summary["runtime_instrumentation_summary"] = summary
        scene_summary["runtime_instrumentation_segmentation_csv"] = str(runtime_dir / "segmentation_runs.csv")
        scene_summary["runtime_instrumentation_summary_json"] = str(summary_json)
        scene_summary_path.write_text(json.dumps(scene_summary, indent=2), encoding="utf-8")

    return {
        "sequence_id": sequence_id,
        "segmentation_run_count": int(len(segmentation_rows)),
        "summary_json": str(summary_json),
        "segmentation_csv": str(runtime_dir / "segmentation_runs.csv"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Repair per-scene runtime instrumentation outputs from saved floor diagnostics.")
    parser.add_argument("--scene-output-root", default=str(DEFAULT_SCENE_OUTPUT_ROOT))
    parser.add_argument("--sequence-ids", nargs="+", default=list(DEFAULT_SEQUENCE_IDS))
    args = parser.parse_args()

    scene_output_root = Path(args.scene_output_root)
    reports = [refresh_scene_outputs(scene_output_root, sequence_id) for sequence_id in args.sequence_ids]
    print(json.dumps({"scene_repairs": reports}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
