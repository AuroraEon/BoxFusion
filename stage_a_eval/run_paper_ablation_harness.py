from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from boxfusion.backend_eval_scaffold import collect_scene_manifest


DEFAULT_OUTPUT_ROOT = Path("stage_a_eval/output/paper_ablations")


def _write_csv(path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def _slug(*parts: Any) -> str:
    token = "_".join(str(part) for part in parts if str(part).strip())
    token = "".join(ch.lower() if ch.isalnum() else "_" for ch in token)
    while "__" in token:
        token = token.replace("__", "_")
    return token.strip("_")


def _condition_key(room_seg_interval: int, box_fusion_mode: str, history_scope_mode: str) -> str:
    return _slug(f"seg{int(room_seg_interval)}", f"boxfusion_{box_fusion_mode}", f"history_{history_scope_mode}")


def _build_command(args: argparse.Namespace, *, condition_root: Path, room_seg_interval: int, box_fusion_mode: str, history_scope_mode: str) -> List[str]:
    cmd = [
        sys.executable,
        "stage_a_demo.py",
        args.dataset_path,
        "--model-path",
        args.model_path,
        "--config",
        args.config,
        "--output-root",
        str(condition_root / "scenes"),
        "--room-seg-interval",
        str(int(room_seg_interval)),
        "--box-fusion-mode",
        str(box_fusion_mode),
        "--history-scope-mode",
        str(history_scope_mode),
    ]
    if args.seqs:
        cmd.extend(["--seqs", *args.seqs])
    if args.device:
        cmd.extend(["--device", args.device])
    if args.clip_path:
        cmd.extend(["--clip-path", args.clip_path])
    if args.text_features:
        cmd.extend(["--text-features", args.text_features])
    if args.class_txt:
        cmd.extend(["--class-txt", args.class_txt])
    if args.every_nth_frame is not None:
        cmd.extend(["--every-nth-frame", str(int(args.every_nth_frame))])
    if args.max_frames is not None:
        cmd.extend(["--max-frames", str(int(args.max_frames))])
    if args.keyframe_gap is not None:
        cmd.extend(["--keyframe-gap", str(int(args.keyframe_gap))])
    if args.capture_stride is not None:
        cmd.extend(["--capture-stride", str(int(args.capture_stride))])
    if args.runtime_profile_interval is not None:
        cmd.extend(["--runtime-profile-interval", str(int(args.runtime_profile_interval))])
    if args.runtime_artifact_mode:
        cmd.extend(["--runtime-artifact-mode", args.runtime_artifact_mode])
    if not args.materialize_service_debug_artifacts:
        cmd.append("--suppress-service-debug-artifacts")
    if args.quiet:
        cmd.append("--quiet")
    if args.log_level:
        cmd.extend(["--log-level", args.log_level])
    if args.runtime_print_interval is not None:
        cmd.extend(["--runtime-print-interval", str(int(args.runtime_print_interval))])
    if args.no_per_profiled_frame_stdout:
        cmd.append("--no-per-profiled-frame-stdout")
    if args.enable_readonly_tail_reference_audit:
        cmd.append("--enable-readonly-tail-reference-audit")
    if args.core_only:
        cmd.append("--core-only")
    if args.full_rgb_replay:
        cmd.append("--full-rgb-replay")
    if args.enable_rerun:
        cmd.append("--enable-rerun")
    for extra_arg in args.extra_arg:
        cmd.append(str(extra_arg))
    return cmd


def _collect_condition_rows(
    *,
    condition_key: str,
    condition_root: Path,
    room_seg_interval: int,
    box_fusion_mode: str,
    history_scope_mode: str,
    command: Sequence[str],
    seqs: Sequence[str],
    execution_status: str,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    scenes_root = condition_root / "scenes"
    for sequence_name in seqs:
        scene_root = scenes_root / sequence_name
        manifest = collect_scene_manifest(scene_root, sequence_name=sequence_name) if scene_root.exists() else {}
        runtime = dict(manifest.get("runtime_summary") or {})
        topology_comparison = dict(manifest.get("topology_comparison_summary") or {})
        rows.append(
            {
                "condition_key": condition_key,
                "sequence_name": sequence_name,
                "room_seg_interval": int(room_seg_interval),
                "box_fusion_mode": str(box_fusion_mode),
                "history_scope_mode": str(history_scope_mode),
                "execution_status": execution_status,
                "scene_status": manifest.get("status"),
                "processed_frames": runtime.get("processed_frames"),
                "duration_sec": runtime.get("duration_sec"),
                "average_fps": runtime.get("average_fps"),
                "backend_artifact_size_total_bytes": runtime.get("backend_artifact_size_total_bytes"),
                "public_room_count": topology_comparison.get("public_room_count"),
                "public_edge_count": topology_comparison.get("public_edge_count"),
                "working_room_count": topology_comparison.get("working_room_count"),
                "working_edge_count": topology_comparison.get("working_edge_count"),
                "withheld_room_count": topology_comparison.get("withheld_room_count"),
                "withheld_edge_count": topology_comparison.get("withheld_edge_count"),
                "scene_root": str(scene_root),
                "manifest_path": str(scene_root / "manifest.json"),
                "summary_json": str(scene_root / "logs" / "summary.json"),
                "topology_json": str(scene_root / "logs" / "topology_v0_1.json"),
                "command": " ".join(command),
            }
        )
    return rows


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run small paper ablation sweeps and emit compact condition summary tables.")
    parser.add_argument("dataset_path", choices=["CA1M", "scannet", "online", "hm3d"])
    parser.add_argument("--model-path", required=True, help="Path to the BoxFusion / Cubify checkpoint")
    parser.add_argument("--config", required=True, help="Config path passed through to stage_a_demo.py")
    parser.add_argument("--seqs", nargs="+", required=True, help="Sequence ids to rerun under each condition")
    parser.add_argument("--experiment-name", default="paper_ablation", help="Name used under the output root")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Root directory for condition outputs and summaries")
    parser.add_argument("--room-seg-intervals", nargs="+", type=int, default=[100], help="Segmentation cadence sweep values")
    parser.add_argument("--box-fusion-modes", nargs="+", choices=["config", "on", "off"], default=["config"], help="BoxFusion on/off sweep values")
    parser.add_argument(
        "--history-scope-modes",
        nargs="+",
        choices=["selective_floor_aware", "broad_history"],
        default=["selective_floor_aware"],
        help="History-scope sweep values",
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--clip-path", default=None)
    parser.add_argument("--text-features", default=None)
    parser.add_argument("--class-txt", default="./data/panoptic_categories_nomerge.txt")
    parser.add_argument("--every-nth-frame", default=None, type=int)
    parser.add_argument("--max-frames", default=None, type=int)
    parser.add_argument("--keyframe-gap", default=None, type=int)
    parser.add_argument("--capture-stride", default=None, type=int)
    parser.add_argument("--runtime-profile-interval", default=None, type=int)
    parser.add_argument("--runtime-artifact-mode", default="benchmark")
    parser.add_argument(
        "--materialize-service-debug-artifacts",
        action="store_true",
        help=(
            "Opt out of the paper/benchmark default suppressor and keep the richer non-authoritative "
            "service/debug artifacts in ablation reruns."
        ),
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--log-level", choices=["summary", "verbose"], default="summary")
    parser.add_argument("--runtime-print-interval", default=50, type=int)
    parser.add_argument("--no-per-profiled-frame-stdout", action="store_true")
    parser.add_argument("--enable-readonly-tail-reference-audit", action="store_true")
    parser.add_argument("--core-only", action="store_true")
    parser.add_argument("--full-rgb-replay", action="store_true")
    parser.add_argument("--enable-rerun", action="store_true")
    parser.add_argument("--extra-arg", action="append", default=[], help="Extra argument passed through verbatim to stage_a_demo.py")
    parser.add_argument("--dry-run", action="store_true", help="Write the command matrix and expected output layout without executing reruns")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    experiment_root = Path(args.output_root) / args.experiment_name
    experiment_root.mkdir(parents=True, exist_ok=True)
    condition_rows: List[Dict[str, Any]] = []
    command_rows: List[Dict[str, Any]] = []

    for room_seg_interval, box_fusion_mode, history_scope_mode in itertools.product(
        args.room_seg_intervals,
        args.box_fusion_modes,
        args.history_scope_modes,
    ):
        condition_key = _condition_key(room_seg_interval, box_fusion_mode, history_scope_mode)
        condition_root = experiment_root / condition_key
        command = _build_command(
            args,
            condition_root=condition_root,
            room_seg_interval=room_seg_interval,
            box_fusion_mode=box_fusion_mode,
            history_scope_mode=history_scope_mode,
        )
        command_rows.append(
            {
                "condition_key": condition_key,
                "room_seg_interval": int(room_seg_interval),
                "box_fusion_mode": str(box_fusion_mode),
                "history_scope_mode": str(history_scope_mode),
                "condition_root": str(condition_root),
                "command": " ".join(command),
            }
        )

        execution_status = "dry_run"
        if not args.dry_run:
            condition_root.mkdir(parents=True, exist_ok=True)
            subprocess.run(command, cwd=PROJECT_ROOT, check=True)
            execution_status = "completed"

        condition_rows.extend(
            _collect_condition_rows(
                condition_key=condition_key,
                condition_root=condition_root,
                room_seg_interval=room_seg_interval,
                box_fusion_mode=box_fusion_mode,
                history_scope_mode=history_scope_mode,
                command=command,
                seqs=args.seqs,
                execution_status=execution_status,
            )
        )

    commands_json = experiment_root / "commands.json"
    summary_json = experiment_root / "ablation_summary.json"
    summary_csv = experiment_root / "ablation_summary.csv"
    commands_json.write_text(json.dumps(command_rows, indent=2), encoding="utf-8")
    summary_payload = {"condition_count": len(command_rows), "row_count": len(condition_rows), "rows": condition_rows}
    summary_json.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")
    if condition_rows:
        _write_csv(summary_csv, condition_rows, list(condition_rows[0].keys()))
    else:
        _write_csv(summary_csv, [], ["condition_key"])

    print(
        json.dumps(
            {
                "commands_json": str(commands_json),
                "summary_json": str(summary_json),
                "summary_csv": str(summary_csv),
                "condition_count": len(command_rows),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
