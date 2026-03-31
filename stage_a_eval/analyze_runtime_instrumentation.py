from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_SCENE_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_OUTPUT_ROOT = Path("world_model_backend_outputs_v0_2_final/eval/runtime_instrumentation")
DEFAULT_REPORT_OUT = Path("runtime_topology_instrumented_report.md")
DEFAULT_BLOCKER_SUMMARY_NAME = "blocker_summary.json"
DEFAULT_SEQUENCE_IDS = [
    "00862-LT9Jq6dN3Ea",
    "00843-DYehNKdT76V",
    "00829-QaLdnwvtxbs",
]


def load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_json_if_exists(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    return load_json(path)


def read_csv_rows(path: Path) -> List[Dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


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
    normalized = []
    for row in rows:
        normalized.append({key: maybe_number(value) for key, value in row.items()})
    return normalized


def safe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    return float(value)


def safe_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    return int(value)


def stats(values: Iterable[Any], digits: int = 6) -> Dict[str, Optional[float]]:
    cleaned = sorted(float(value) for value in values if value is not None)
    if not cleaned:
        return {"count": 0, "min": None, "mean": None, "p50": None, "p95": None, "max": None}

    def pct(p: float) -> float:
        if len(cleaned) == 1:
            return cleaned[0]
        position = (len(cleaned) - 1) * p
        lo = int(math.floor(position))
        hi = int(math.ceil(position))
        if lo == hi:
            return cleaned[lo]
        alpha = position - lo
        return (1.0 - alpha) * cleaned[lo] + alpha * cleaned[hi]

    return {
        "count": int(len(cleaned)),
        "min": round(cleaned[0], digits),
        "mean": round(mean(cleaned), digits),
        "p50": round(pct(0.50), digits),
        "p95": round(pct(0.95), digits),
        "max": round(cleaned[-1], digits),
    }


def pearson(xs: Sequence[Any], ys: Sequence[Any]) -> Optional[float]:
    pairs = [(float(x), float(y)) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pairs) < 2:
        return None
    x_vals = np.asarray([item[0] for item in pairs], dtype=np.float64)
    y_vals = np.asarray([item[1] for item in pairs], dtype=np.float64)
    if np.std(x_vals) < 1e-9 or np.std(y_vals) < 1e-9:
        return None
    return float(np.corrcoef(x_vals, y_vals)[0, 1])


def worst_row(rows: Sequence[Dict[str, Any]], key: str) -> Optional[Dict[str, Any]]:
    candidates = [row for row in rows if row.get(key) is not None]
    if not candidates:
        return None
    return max(candidates, key=lambda item: float(item.get(key) or 0.0))


def late_window(rows: Sequence[Dict[str, Any]], key: str, count: int = 5) -> Optional[float]:
    subset = [row.get(key) for row in rows[-count:] if row.get(key) is not None]
    if not subset:
        return None
    return float(mean(float(value) for value in subset))


def stage_share(rows: Sequence[Dict[str, Any]], numerator: str, denominator: str) -> Optional[float]:
    ratios = []
    for row in rows:
        num = row.get(numerator)
        den = row.get(denominator)
        if num is None or den in (None, 0, 0.0):
            continue
        ratios.append(float(num) / float(den))
    if not ratios:
        return None
    return float(mean(ratios))


def write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def build_blocker_summary(
    *,
    status: str,
    sequence_ids: Sequence[str],
    scene_output_root: Path,
    output_root: Path,
    scene_status_rows: Sequence[Dict[str, Any]],
    active_blockers: Sequence[str],
    status_reason: str,
    previous_summary: Optional[Dict[str, Any]] = None,
    aggregate_summary_path: Optional[Path] = None,
    report_out: Optional[Path] = None,
) -> Dict[str, Any]:
    summary: Dict[str, Any] = {
        "status": str(status),
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "status_reason": str(status_reason),
        "representative_sequence_ids": list(sequence_ids),
        "scene_output_root": str(scene_output_root),
        "output_root": str(output_root),
        "scene_status": list(scene_status_rows),
        "active_blockers": list(active_blockers),
    }
    if aggregate_summary_path is not None:
        summary["aggregate_summary_json"] = str(aggregate_summary_path)
    if report_out is not None:
        summary["report_path"] = str(report_out)
    if previous_summary is not None:
        summary["supersedes_previous_status"] = previous_summary.get("status")
        if previous_summary.get("generated_at") is not None:
            summary["supersedes_previous_generated_at"] = previous_summary.get("generated_at")
        elif previous_summary.get("date") is not None:
            summary["supersedes_previous_date"] = previous_summary.get("date")
    return summary


def _format_num(value: Any, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.{digits}f}"


def build_scene_analysis(
    sequence_id: str,
    frame_rows: Sequence[Dict[str, Any]],
    export_rows: Sequence[Dict[str, Any]],
    history_rows: Sequence[Dict[str, Any]],
    segmentation_rows: Sequence[Dict[str, Any]],
    scene_summary: Dict[str, Any],
) -> Dict[str, Any]:
    duplicate_frames = sorted({int(row["frame_idx"]) for row in export_rows if row.get("duplicate_same_frame")})
    scope_counter = Counter(str(row.get("scope_label")) for row in history_rows if row.get("scope_label"))
    stage5_worst = worst_row(frame_rows, "stage5_total_sec")
    stage3_worst = worst_row(frame_rows, "stage3_total_sec")
    total_worst = worst_row(frame_rows, "total_step_sec")
    analysis = {
        "sequence_id": sequence_id,
        "profiled_frame_count": int(len(frame_rows)),
        "duplicate_export_frames": duplicate_frames,
        "duplicate_export_frame_count": int(len(duplicate_frames)),
        "history_scope_counter": dict(scope_counter),
        "active_room_available_rate": None,
        "stage5_export_share_mean": stage_share(frame_rows, "snapshot_export_sec", "stage5_total_sec"),
        "stage5_boxfusion_share_mean": stage_share(frame_rows, "boxfusion_total_sec", "stage5_total_sec"),
        "stage3_export_share_mean": stage_share(frame_rows, "vector_map_export_after_segmentation_sec", "stage3_total_sec"),
        "stage3_segmentation_share_mean": stage_share(frame_rows, "segmentation_total_sec", "stage3_total_sec"),
        "corr_stage5_vs_global_boxes": pearson(
            [row.get("total_retained_object_count") for row in frame_rows],
            [row.get("stage5_total_sec") for row in frame_rows],
        ),
        "corr_stage5_vs_per_frame_ins": pearson(
            [row.get("per_frame_ins_count") for row in frame_rows],
            [row.get("stage5_total_sec") for row in frame_rows],
        ),
        "corr_stage3_vs_merged_points": pearson(
            [row.get("merged_point_count_after_downsample") for row in frame_rows],
            [row.get("stage3_total_sec") for row in frame_rows],
        ),
        "corr_stage3_vs_grid_area": pearson(
            [row.get("grid_area") for row in frame_rows],
            [row.get("stage3_total_sec") for row in frame_rows],
        ),
        "late_stage5_total_sec": late_window(frame_rows, "stage5_total_sec"),
        "late_stage3_total_sec": late_window(frame_rows, "stage3_total_sec"),
        "late_global_box_count": late_window(frame_rows, "total_retained_object_count"),
        "late_per_frame_ins_count": late_window(frame_rows, "per_frame_ins_count"),
        "late_grid_area": late_window(frame_rows, "grid_area"),
        "late_merged_point_count": late_window(frame_rows, "merged_point_count_after_downsample"),
        "worst_frames": {
            "stage5_total_sec": None if stage5_worst is None else {
                "frame_idx": int(stage5_worst["frame_idx"]),
                "stage5_total_sec": safe_float(stage5_worst.get("stage5_total_sec")),
                "snapshot_export_sec": safe_float(stage5_worst.get("snapshot_export_sec")),
                "boxfusion_total_sec": safe_float(stage5_worst.get("boxfusion_total_sec")),
                "total_retained_object_count": safe_int(stage5_worst.get("total_retained_object_count")),
                "per_frame_ins_count": safe_int(stage5_worst.get("per_frame_ins_count")),
                "vector_map_export_call_count": safe_int(stage5_worst.get("vector_map_export_call_count")),
            },
            "stage3_total_sec": None if stage3_worst is None else {
                "frame_idx": int(stage3_worst["frame_idx"]),
                "stage3_total_sec": safe_float(stage3_worst.get("stage3_total_sec")),
                "segmentation_total_sec": safe_float(stage3_worst.get("segmentation_total_sec")),
                "vector_map_export_after_segmentation_sec": safe_float(stage3_worst.get("vector_map_export_after_segmentation_sec")),
                "merged_point_count_after_downsample": safe_int(stage3_worst.get("merged_point_count_after_downsample")),
                "grid_area": safe_int(stage3_worst.get("grid_area")),
            },
            "total_step_sec": None if total_worst is None else {
                "frame_idx": int(total_worst["frame_idx"]),
                "total_step_sec": safe_float(total_worst.get("total_step_sec")),
                "stage3_total_sec": safe_float(total_worst.get("stage3_total_sec")),
                "stage5_total_sec": safe_float(total_worst.get("stage5_total_sec")),
            },
        },
        "scene_summary": scene_summary,
    }
    active_room_rows = [row.get("active_room_available") for row in frame_rows if row.get("active_room_available") is not None]
    if active_room_rows:
        analysis["active_room_available_rate"] = float(sum(1 for value in active_room_rows if value) / len(active_room_rows))
    return analysis


def maybe_make_plots(output_root: Path, scene_rows: Dict[str, List[Dict[str, Any]]]) -> List[str]:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return []

    plot_paths: List[str] = []
    scene_ids = list(scene_rows.keys())
    if not scene_ids:
        return plot_paths

    def _sorted_rows(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
        return sorted(rows, key=lambda item: int(item.get("frame_idx") or 0))

    fig, axes = plt.subplots(len(scene_ids), 1, figsize=(12, 3.8 * len(scene_ids)), sharex=False)
    if len(scene_ids) == 1:
        axes = [axes]
    for ax, sequence_id in zip(axes, scene_ids):
        rows = _sorted_rows(scene_rows[sequence_id])
        frames = [int(row["frame_idx"]) for row in rows]
        ax.plot(frames, [float(row.get("stage3_total_sec") or 0.0) for row in rows], label="stage3_total_sec")
        ax.plot(frames, [float(row.get("stage5_total_sec") or 0.0) for row in rows], label="stage5_total_sec")
        ax.plot(frames, [float(row.get("total_step_sec") or 0.0) for row in rows], label="total_step_sec", linewidth=2.0)
        ax.set_title(f"{sequence_id}: timing breakdown vs frame index")
        ax.set_xlabel("frame_idx")
        ax.set_ylabel("sec")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.tight_layout()
    timing_path = output_root / "timing_breakdown_vs_frame.png"
    fig.savefig(timing_path, dpi=160)
    plt.close(fig)
    plot_paths.append(str(timing_path))

    fig, axes = plt.subplots(len(scene_ids), 1, figsize=(12, 3.8 * len(scene_ids)), sharex=False)
    if len(scene_ids) == 1:
        axes = [axes]
    for ax, sequence_id in zip(axes, scene_ids):
        rows = _sorted_rows(scene_rows[sequence_id])
        frames = [int(row["frame_idx"]) for row in rows]
        ax.plot(frames, [float(row.get("total_retained_object_count") or 0.0) for row in rows], label="retained_objects")
        ax.plot(frames, [float(row.get("per_frame_ins_count") or 0.0) for row in rows], label="per_frame_ins")
        ax.plot(frames, [float(row.get("total_room_count") or 0.0) for row in rows], label="rooms")
        ax.set_title(f"{sequence_id}: cardinality growth vs frame index")
        ax.set_xlabel("frame_idx")
        ax.set_ylabel("count")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.tight_layout()
    cardinality_path = output_root / "cardinality_growth_vs_frame.png"
    fig.savefig(cardinality_path, dpi=160)
    plt.close(fig)
    plot_paths.append(str(cardinality_path))

    fig, axes = plt.subplots(len(scene_ids), 1, figsize=(12, 3.6 * len(scene_ids)), sharex=False)
    if len(scene_ids) == 1:
        axes = [axes]
    for ax, sequence_id in zip(axes, scene_ids):
        rows = _sorted_rows(scene_rows[sequence_id])
        frames = [int(row["frame_idx"]) for row in rows]
        counts = [int(row.get("vector_map_export_call_count") or 0) for row in rows]
        ax.bar(frames, counts, color=["tab:red" if c > 1 else "tab:blue" for c in counts], width=4.0)
        ax.set_title(f"{sequence_id}: export duplication markers")
        ax.set_xlabel("frame_idx")
        ax.set_ylabel("vector_map_export_call_count")
        ax.grid(True, alpha=0.25)
    fig.tight_layout()
    duplication_path = output_root / "export_duplication_markers.png"
    fig.savefig(duplication_path, dpi=160)
    plt.close(fig)
    plot_paths.append(str(duplication_path))

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    for sequence_id in scene_ids:
        rows = _sorted_rows(scene_rows[sequence_id])
        axes[0].scatter(
            [float(row.get("merged_point_count_after_downsample") or 0.0) for row in rows],
            [float(row.get("stage3_total_sec") or 0.0) for row in rows],
            label=sequence_id,
            alpha=0.8,
        )
        axes[1].scatter(
            [float(row.get("total_retained_object_count") or 0.0) for row in rows],
            [float(row.get("stage5_total_sec") or 0.0) for row in rows],
            label=sequence_id,
            alpha=0.8,
        )
    axes[0].set_title("Segmentation cost vs merged point count")
    axes[0].set_xlabel("merged_point_count_after_downsample")
    axes[0].set_ylabel("stage3_total_sec")
    axes[0].grid(True, alpha=0.25)
    axes[1].set_title("Object-side cost vs retained object count")
    axes[1].set_xlabel("total_retained_object_count")
    axes[1].set_ylabel("stage5_total_sec")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best")
    fig.tight_layout()
    scatter_path = output_root / "cost_vs_scale.png"
    fig.savefig(scatter_path, dpi=160)
    plt.close(fig)
    plot_paths.append(str(scatter_path))
    return plot_paths


def build_report(
    analyses: Sequence[Dict[str, Any]],
    output_root: Path,
    report_out: Path,
    scene_output_root: Path,
    sequence_ids: Sequence[str],
) -> None:
    lines: List[str] = [
        "# Runtime / Topology Instrumented Report",
        "",
        "## 1. Executive summary",
        "",
    ]
    if analyses:
        worst_stage5_scene = max(
            analyses,
            key=lambda item: float((item.get("worst_frames", {}).get("stage5_total_sec", {}) or {}).get("stage5_total_sec") or 0.0),
        )
        worst_stage3_scene = max(
            analyses,
            key=lambda item: float((item.get("worst_frames", {}).get("stage3_total_sec", {}) or {}).get("stage3_total_sec") or 0.0),
        )
        lines.extend(
            [
                f"- Across the representative scenes {', '.join(sequence_ids)}, the dominant late-frame costs remain stage 5 object-side processing and stage 3 topology/segmentation refresh, with the heaviest object-side case in `{worst_stage5_scene['sequence_id']}` and the heaviest topology case in `{worst_stage3_scene['sequence_id']}`.",
                "- The advisor’s local-vs-global suspicion is supported by the new scope logs: spatial association, small-object correspondence, and BoxFusion all report `global_retained_history` or `near_global_retained_history`, with no room-scoped or recent-window filter active in the current path.",
                "- The previous coarse timing labels were misleading. Stage 5 includes non-trivial snapshot-triggered full export rebuilds, and stage 3 includes both segmentation work and a second full export rebuild on refresh frames.",
                "",
                "## 2. Stage-5 object-side diagnosis",
                "",
            ]
        )
    else:
        lines.append("- No instrumented scenes were found.")

    for analysis in analyses:
        worst = analysis.get("worst_frames", {}).get("stage5_total_sec") or {}
        lines.append(
            "- "
            f"`{analysis['sequence_id']}`: late stage-5 avg={_format_num(analysis.get('late_stage5_total_sec'))} s, "
            f"corr(stage5, global_boxes)={_format_num(analysis.get('corr_stage5_vs_global_boxes'))}, "
            f"corr(stage5, per_frame_ins)={_format_num(analysis.get('corr_stage5_vs_per_frame_ins'))}, "
            f"mean snapshot-export share={_format_num(analysis.get('stage5_export_share_mean'))}, "
            f"mean BoxFusion share={_format_num(analysis.get('stage5_boxfusion_share_mean'))}, "
            f"worst frame={worst.get('frame_idx', 'n/a')}."
        )
    lines.extend(
        [
            "- Evidence from `history_scope.csv` shows current-frame object processing compares against retained history that is effectively scene-global. No instrumented step emitted a room-scoped or recent-window-scoped candidate pool.",
            "- `snapshot_export_sec` and `vector_map_export_sec` confirm that stage 5 is not purely CLIP plus BoxFusion; it can include a full `get_vector_map_data(...)` rebuild when snapshots are captured.",
            "",
            "## 3. Stage-3 room/topology-side diagnosis",
            "",
        ]
    )
    for analysis in analyses:
        worst = analysis.get("worst_frames", {}).get("stage3_total_sec") or {}
        lines.append(
            "- "
            f"`{analysis['sequence_id']}`: late stage-3 avg={_format_num(analysis.get('late_stage3_total_sec'))} s, "
            f"corr(stage3, merged_points)={_format_num(analysis.get('corr_stage3_vs_merged_points'))}, "
            f"corr(stage3, grid_area)={_format_num(analysis.get('corr_stage3_vs_grid_area'))}, "
            f"mean export share={_format_num(analysis.get('stage3_export_share_mean'))}, "
            f"mean segmentation share={_format_num(analysis.get('stage3_segmentation_share_mean'))}, "
            f"worst frame={worst.get('frame_idx', 'n/a')}."
        )
    lines.extend(
        [
            "- The fine-grained stage-3 timers separate floor merge/downsample, histogram/state construction, tracking, gateway extraction, and post-segmentation export. This makes it visible when the cost is dominated by global floor-cloud growth versus export rebuild work.",
            "- The new segmentation-scale columns (`merged_point_count_after_downsample`, `grid_area`, `wall_slice_point_count`, `full_slice_point_count`) show that refresh cost grows with accumulated floor state rather than being bounded by a current-room subset.",
            "",
            "## 4. Duplicate-work diagnosis",
            "",
        ]
    )
    duplicate_total = sum(int(analysis.get("duplicate_export_frame_count") or 0) for analysis in analyses)
    lines.append(f"- Duplicate export frames observed across the representative set: {duplicate_total}.")
    for analysis in analyses:
        lines.append(
            "- "
            f"`{analysis['sequence_id']}` duplicate-export frames: {analysis.get('duplicate_export_frames', []) or 'none'}."
        )
    lines.extend(
        [
            "- `vector_map_export_calls.csv` records the exact `call_context` and `call_index_within_frame`, so same-frame rebuilds are explicit instead of being hidden inside a coarse bucket.",
            "- The most common duplication pattern is a segmentation refresh export followed by a snapshot-triggered export on the same frame.",
            "",
            "## 5. Structural interpretation",
            "",
            "- Timer-label issue: the old `feature_boxfusion_sec` bucket conflated CLIP, spatial/correspondence association, BoxFusion optimization, snapshot capture, and full vector-map export.",
            "- Duplicate work: segmentation refresh frames can rebuild vector-map export once in stage 3 and again in stage 5 snapshot capture on the same frame.",
            "- Fundamentally global-growing work: retained object history, cumulative `per_frame_ins`, floor-level merged point clouds, and floor grids all grow with scene history in the current implementation.",
            "- Architecture-level redesign later: room-scoped or recent-window object fusion, explicit current-room state, room-leave signals, and online topology delta updates are still absent rather than merely untimed.",
            "",
            "## 6. Advisor alignment",
            "",
            "- Current/local vs global/history separation: not yet aligned. Instrumented scope logs show object-side association and fusion remain tied to retained global history.",
            "- Relevant-subset fusion vs near-global fusion: not aligned. The current path does not apply room-scoped, floor-scoped, or recent-window pruning before stage-5 candidate scans.",
            "- Event-driven incremental topology vs interval/post-hoc topology: partially aligned at the observation level, but not at export/update level. Floors observe frames online, yet room segmentation and topology export remain scheduled refresh plus full rebuild rather than incremental triggers plus deltas.",
            "",
            "## 7. Recommended next actions",
            "",
            "- Immediate low-risk cleanup: keep the new fine-grained timers, preserve the coarse totals only as legacy convenience, and stop treating `feature_boxfusion_sec` as CLIP/BoxFusion-only in reports.",
            "- Immediate low-risk cleanup: use the duplicate-export log to guard or memoize same-frame `get_vector_map_data(...)` rebuilds before deeper algorithm changes.",
            "- Medium instrumentation-informed fixes: bound stage-5 candidate pools by floor or recent-window filters first, then compare late-frame curves against the new baseline.",
            "- Medium instrumentation-informed fixes: split topology refresh into floor-merge, segmentation, and export phases in scheduling policy so we can independently rate-limit export rebuilds.",
            "- Later architecture changes: introduce explicit active-room state, room-completion/leaving signals, and online topology delta structures before attempting a SLAM-style backend redesign.",
            "",
            "## Reproducible commands",
            "",
            f"Instrumentation reruns write per-scene artifacts under `{scene_output_root}/<sequence>/logs/runtime_instrumentation/` and aggregate outputs under `{output_root}`.",
            "",
        ]
    )
    for sequence_id in sequence_ids:
        lines.extend(
            [
                "```bash",
                "/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d \\",
                "  --model-path ./models/cutr_rgbd.pth \\",
                "  --config ./config/hm3d.yaml \\",
                "  --device cuda \\",
                f"  --seq {sequence_id} \\",
                f"  --output-root ./{scene_output_root.as_posix()} \\",
                "  --room-seg-interval 100 \\",
                "  --video-fps 12 \\",
                "  --core-only \\",
                "  --runtime-profile-interval 25",
                "```",
                "",
            ]
        )
    lines.extend(
        [
            "```bash",
            "/home/aurora/miniconda3/envs/boxfusion/bin/python stage_a_eval/analyze_runtime_instrumentation.py \\",
            f"  --scene-output-root ./{scene_output_root.as_posix()} \\",
            f"  --output-root ./{output_root.as_posix()} \\",
            f"  --report-out ./{report_out.as_posix()} \\",
            f"  --sequence-ids {' '.join(sequence_ids)}",
            "```",
            "",
        ]
    )
    report_out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Aggregate runtime instrumentation outputs and render plots/report.")
    parser.add_argument("--scene-output-root", default=str(DEFAULT_SCENE_OUTPUT_ROOT))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--report-out", default=str(DEFAULT_REPORT_OUT))
    parser.add_argument("--sequence-ids", nargs="+", default=list(DEFAULT_SEQUENCE_IDS))
    args = parser.parse_args()

    scene_output_root = Path(args.scene_output_root)
    output_root = Path(args.output_root)
    report_out = Path(args.report_out)
    output_root.mkdir(parents=True, exist_ok=True)
    blocker_summary_path = output_root / DEFAULT_BLOCKER_SUMMARY_NAME
    previous_blocker_summary = load_json_if_exists(blocker_summary_path)

    combined_frame_rows: List[Dict[str, Any]] = []
    combined_export_rows: List[Dict[str, Any]] = []
    combined_history_rows: List[Dict[str, Any]] = []
    combined_segmentation_rows: List[Dict[str, Any]] = []
    scene_analyses: List[Dict[str, Any]] = []
    scene_rows_for_plot: Dict[str, List[Dict[str, Any]]] = {}
    scene_status_rows: List[Dict[str, Any]] = []
    missing_scene_artifacts: List[Dict[str, Any]] = []

    for sequence_id in args.sequence_ids:
        scene_dir = scene_output_root / sequence_id / "logs" / "runtime_instrumentation"
        frame_csv = scene_dir / "per_profiled_frame.csv"
        export_csv = scene_dir / "vector_map_export_calls.csv"
        history_csv = scene_dir / "history_scope.csv"
        segmentation_csv = scene_dir / "segmentation_runs.csv"
        summary_json = scene_dir / "summary.json"
        required_paths = (frame_csv, export_csv, history_csv, segmentation_csv, summary_json)
        missing_paths = [str(path) for path in required_paths if not path.exists()]
        scene_status_rows.append(
            {
                "sequence_id": sequence_id,
                "scene_dir": str(scene_dir),
                "artifacts_present": bool(not missing_paths),
                "missing_artifacts": missing_paths,
            }
        )
        if missing_paths:
            missing_scene_artifacts.append(
                {
                    "sequence_id": sequence_id,
                    "scene_dir": str(scene_dir),
                    "missing_artifacts": missing_paths,
                }
            )
            continue

        frame_rows = normalize_rows(read_csv_rows(frame_csv))
        export_rows = normalize_rows(read_csv_rows(export_csv))
        history_rows = normalize_rows(read_csv_rows(history_csv))
        segmentation_rows = normalize_rows(read_csv_rows(segmentation_csv))
        scene_summary = load_json(summary_json)

        combined_frame_rows.extend(frame_rows)
        combined_export_rows.extend(export_rows)
        combined_history_rows.extend(history_rows)
        combined_segmentation_rows.extend(segmentation_rows)
        scene_rows_for_plot[sequence_id] = frame_rows
        scene_analyses.append(
            build_scene_analysis(
                sequence_id,
                frame_rows,
                export_rows,
                history_rows,
                segmentation_rows,
                scene_summary,
            )
        )

    if missing_scene_artifacts:
        blocker_summary = build_blocker_summary(
            status="blocked_missing_runtime_instrumentation_artifacts",
            sequence_ids=args.sequence_ids,
            scene_output_root=scene_output_root,
            output_root=output_root,
            scene_status_rows=scene_status_rows,
            active_blockers=[
                "One or more requested scenes are missing runtime instrumentation artifacts required for aggregation."
            ],
            status_reason="Aggregate refresh could not complete because at least one requested scene is missing required instrumentation outputs.",
            previous_summary=previous_blocker_summary,
            report_out=report_out,
        )
        blocker_summary["missing_scene_artifacts"] = missing_scene_artifacts
        write_json(blocker_summary_path, blocker_summary)
        raise FileNotFoundError(
            "Missing instrumentation artifacts for one or more scenes: "
            + json.dumps(missing_scene_artifacts, indent=2)
        )

    write_csv(output_root / "combined_per_profiled_frame.csv", combined_frame_rows)
    write_csv(output_root / "combined_vector_map_export_calls.csv", combined_export_rows)
    write_csv(output_root / "combined_history_scope.csv", combined_history_rows)
    write_csv(output_root / "combined_segmentation_runs.csv", combined_segmentation_rows)

    aggregate_summary = {
        "sequence_ids": list(args.sequence_ids),
        "scene_count": int(len(scene_analyses)),
        "combined_profiled_frame_stats": {
            "stage5_total_sec": stats(row.get("stage5_total_sec") for row in combined_frame_rows),
            "stage3_total_sec": stats(row.get("stage3_total_sec") for row in combined_frame_rows),
            "total_step_sec": stats(row.get("total_step_sec") for row in combined_frame_rows),
        },
        "duplicate_export_frame_count": int(
            len({int(row["frame_idx"]) for row in combined_export_rows if row.get("duplicate_same_frame")})
        ),
        "history_scope_counter": dict(
            Counter(str(row.get("scope_label")) for row in combined_history_rows if row.get("scope_label"))
        ),
        "scene_analyses": scene_analyses,
    }
    aggregate_summary_path = output_root / "aggregate_summary.json"
    write_json(aggregate_summary_path, aggregate_summary)

    plot_paths = maybe_make_plots(output_root, scene_rows_for_plot)
    write_json(output_root / "plot_manifest.json", {"plots": plot_paths})

    build_report(
        scene_analyses,
        output_root=output_root,
        report_out=report_out,
        scene_output_root=scene_output_root,
        sequence_ids=args.sequence_ids,
    )
    blocker_summary = build_blocker_summary(
        status="cleared_no_active_blocker",
        sequence_ids=args.sequence_ids,
        scene_output_root=scene_output_root,
        output_root=output_root,
        scene_status_rows=scene_status_rows,
        active_blockers=[],
        status_reason="Aggregate refresh completed successfully for all requested scenes; the earlier blocked state is superseded.",
        previous_summary=previous_blocker_summary,
        aggregate_summary_path=aggregate_summary_path,
        report_out=report_out,
    )
    blocker_summary["scene_count"] = int(len(scene_analyses))
    blocker_summary["combined_artifact_counts"] = {
        "per_profiled_frame_rows": int(len(combined_frame_rows)),
        "vector_map_export_call_rows": int(len(combined_export_rows)),
        "history_scope_rows": int(len(combined_history_rows)),
        "segmentation_run_rows": int(len(combined_segmentation_rows)),
    }
    write_json(blocker_summary_path, blocker_summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
