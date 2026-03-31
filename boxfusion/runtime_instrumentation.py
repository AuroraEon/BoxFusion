from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import Counter
from contextlib import contextmanager
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional, Sequence


def _round_float(value: Any, digits: int = 6) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _safe_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    return int(value)


def _safe_counter(values: Iterable[Any]) -> Dict[str, int]:
    counter = Counter()
    for value in values:
        counter[str(value)] += 1
    return dict(sorted(counter.items(), key=lambda item: item[0]))


def _safe_stats(values: Sequence[Any], digits: int = 3) -> Dict[str, Optional[float]]:
    cleaned = sorted(float(value) for value in values if value is not None)
    if not cleaned:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "p50": None,
            "p90": None,
            "p95": None,
            "max": None,
        }

    def _percentile(pct: float) -> float:
        if len(cleaned) == 1:
            return cleaned[0]
        position = (len(cleaned) - 1) * pct
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return cleaned[lower]
        alpha = position - lower
        return (1.0 - alpha) * cleaned[lower] + alpha * cleaned[upper]

    return {
        "count": int(len(cleaned)),
        "min": round(cleaned[0], digits),
        "mean": round(mean(cleaned), digits),
        "p50": round(_percentile(0.50), digits),
        "p90": round(_percentile(0.90), digits),
        "p95": round(_percentile(0.95), digits),
        "max": round(cleaned[-1], digits),
    }


class RuntimeInstrumentation:
    """Collects per-frame, per-export, and per-segmentation runtime evidence."""

    FRAME_FIELDNAMES = [
        "sequence_id",
        "frame_idx",
        "sample_kind",
        "profiled_frame",
        "is_keyframe",
        "segmentation_refresh_frame",
        "segmentation_refresh_triggered_duplicate_export",
        "active_floor_id",
        "active_floor_status",
        "active_room_id",
        "active_room_available",
        "room_leave_signal_exists",
        "topology_incremental_online",
        "topology_update_mode",
        "topology_missing_online_trigger_structures",
        "data_preprocess_sec",
        "model_bbox_inference_sec",
        "stage3_total_sec",
        "observe_frame_sec",
        "floor_merge_sec",
        "floor_downsample_sec",
        "segmentation_total_sec",
        "histogram_build_sec",
        "segmentation_state_build_sec",
        "room_tracking_sec",
        "gateway_extraction_sec",
        "vector_map_export_after_segmentation_sec",
        "vertical_transition_export_sec",
        "room_export_sec",
        "object_export_sec",
        "diagnostics_export_sec",
        "rerun_visualization_sec",
        "stage5_total_sec",
        "clip_classification_sec",
        "spatial_association_sec",
        "correspondence_association_sec",
        "boxfusion_candidate_scan_sec",
        "boxfusion_optimization_sec",
        "boxfusion_total_sec",
        "snapshot_capture_sec",
        "snapshot_export_sec",
        "vector_map_export_sec",
        "scene_graph_build_sec",
        "spatial_relations_sec",
        "anchor_build_sec",
        "total_step_sec",
        "pred_instance_count",
        "all_pred_box_before_association_count",
        "all_pred_box_after_concat_count",
        "all_pred_box_after_association_count",
        "per_frame_ins_count",
        "cur_keep_idx_count",
        "cur_success_nms_count",
        "small_object_candidate_count",
        "boxfusion_candidates_scanned",
        "boxfusion_candidates_eligible",
        "boxfusion_candidates_optimized",
        "boxfusion_candidates_updated",
        "fusion_list_count",
        "fusion_list_ge3_count",
        "fusion_list_len_mean",
        "fusion_list_len_p50",
        "fusion_list_len_p95",
        "fusion_list_len_max",
        "total_retained_object_count",
        "total_retained_anchor_count",
        "total_room_count",
        "total_floor_count",
        "segmentation_pending_chunk_count",
        "segmentation_pending_chunk_frame_start",
        "segmentation_pending_chunk_frame_end",
        "merged_point_count_before_merge",
        "merged_point_count_after_downsample",
        "wall_slice_point_count",
        "full_slice_point_count",
        "grid_width",
        "grid_height",
        "grid_area",
        "room_count_before_tracking",
        "room_count_after_tracking",
        "tracked_room_count",
        "gateway_room_pair_checks",
        "gateway_count",
        "current_vertical_transition_count",
        "vector_map_export_call_count",
        "vector_map_export_call_contexts",
        "vector_map_export_duplicate_same_frame",
        "history_scope_step_count",
        "history_scope_labels",
    ]

    EXPORT_FIELDNAMES = [
        "sequence_id",
        "frame_idx",
        "call_index_global",
        "call_index_within_frame",
        "call_context",
        "call_origin",
        "segmentation_refresh_frame",
        "duplicate_same_frame",
        "duration_sec",
        "room_export_sec",
        "vertical_transition_export_sec",
        "object_export_sec",
        "scene_graph_build_sec",
        "spatial_relations_sec",
        "anchor_build_sec",
        "diagnostics_export_sec",
        "room_count",
        "gateway_count",
        "vertical_transition_count",
        "object_count",
        "anchor_count",
    ]

    HISTORY_SCOPE_FIELDNAMES = [
        "sequence_id",
        "frame_idx",
        "step_name",
        "scope_label",
        "candidate_pool_before",
        "candidate_pool_after",
        "retained_history_pool",
        "filter_description",
        "notes",
    ]

    SEGMENTATION_FIELDNAMES = [
        "sequence_id",
        "run_id",
        "frame_idx",
        "floor_id",
        "trigger_reason",
        "active_status",
        "success",
        "pending_chunk_count",
        "pending_chunk_frame_start",
        "pending_chunk_frame_end",
        "pending_chunk_frame_count",
        "merged_point_count_before_merge",
        "merged_point_count_after_downsample",
        "wall_slice_point_count",
        "full_slice_point_count",
        "grid_width",
        "grid_height",
        "grid_area",
        "room_count_before_tracking",
        "room_count_after_tracking",
        "tracked_room_count",
        "gateway_room_pair_checks",
        "gateway_count",
        "current_vertical_transition_count",
        "floor_merge_sec",
        "floor_downsample_sec",
        "segmentation_total_sec",
        "histogram_build_sec",
        "segmentation_state_build_sec",
        "room_tracking_sec",
        "gateway_extraction_sec",
        "vector_map_export_after_segmentation_sec",
        "vertical_transition_export_sec",
        "room_export_sec",
        "object_export_sec",
        "diagnostics_export_sec",
        "failure_reason",
        "slice_mode",
        "fallback_reason_summary",
    ]

    def __init__(self, sequence_id: str, output_dir: str | Path) -> None:
        self.sequence_id = str(sequence_id)
        self.output_dir = Path(output_dir)
        self.frames: Dict[int, Dict[str, Any]] = {}
        self.export_calls: List[Dict[str, Any]] = []
        self.history_scope_rows: List[Dict[str, Any]] = []
        self.segmentation_runs: List[Dict[str, Any]] = []
        self.final_summary: Dict[str, Any] = {}
        self._export_counter = 0
        self._segmentation_run_index: Dict[str, int] = {}

    def ensure_frame(self, frame_idx: int) -> Dict[str, Any]:
        frame_idx = int(frame_idx)
        row = self.frames.get(frame_idx)
        if row is None:
            row = {
                "sequence_id": self.sequence_id,
                "frame_idx": frame_idx,
                "sample_kind": "frame",
                "profiled_frame": False,
                "is_keyframe": False,
                "segmentation_refresh_frame": False,
                "segmentation_refresh_triggered_duplicate_export": False,
                "active_floor_id": None,
                "active_floor_status": None,
                "active_room_id": None,
                "active_room_available": False,
                "room_leave_signal_exists": False,
                "topology_incremental_online": False,
                "topology_update_mode": None,
                "topology_missing_online_trigger_structures": None,
                "vector_map_export_call_count": 0,
                "vector_map_export_call_contexts": "",
                "vector_map_export_duplicate_same_frame": False,
                "history_scope_step_count": 0,
                "history_scope_labels": "",
            }
            self.frames[frame_idx] = row
        return row

    def mark_frame(
        self,
        frame_idx: int,
        *,
        sample_kind: Optional[str] = None,
        profiled_frame: Optional[bool] = None,
        is_keyframe: Optional[bool] = None,
        segmentation_refresh_frame: Optional[bool] = None,
    ) -> None:
        row = self.ensure_frame(frame_idx)
        if sample_kind is not None:
            row["sample_kind"] = str(sample_kind)
        if profiled_frame is not None:
            row["profiled_frame"] = bool(profiled_frame)
        if is_keyframe is not None:
            row["is_keyframe"] = bool(is_keyframe)
        if segmentation_refresh_frame is not None:
            row["segmentation_refresh_frame"] = bool(segmentation_refresh_frame)

    def set_value(self, frame_idx: int, key: str, value: Any) -> None:
        self.ensure_frame(frame_idx)[key] = value

    def add_value(self, frame_idx: int, key: str, value: Any) -> None:
        if value is None:
            return
        row = self.ensure_frame(frame_idx)
        row[key] = float(row.get(key, 0.0) or 0.0) + float(value)

    @contextmanager
    def timer(self, frame_idx: int, key: str):
        start_t = time.perf_counter()
        try:
            yield
        finally:
            self.add_value(frame_idx, key, time.perf_counter() - start_t)

    def add_values(self, frame_idx: int, payload: Dict[str, Any]) -> None:
        row = self.ensure_frame(frame_idx)
        for key, value in payload.items():
            row[key] = value

    def log_export_call(
        self,
        frame_idx: int,
        *,
        call_context: str,
        call_origin: str,
        segmentation_refresh_frame: bool,
        metrics: Dict[str, Any],
    ) -> None:
        frame_row = self.ensure_frame(frame_idx)
        calls_for_frame = 1 + int(frame_row.get("vector_map_export_call_count") or 0)
        duplicate = calls_for_frame > 1
        self._export_counter += 1
        row = {
            "sequence_id": self.sequence_id,
            "frame_idx": int(frame_idx),
            "call_index_global": int(self._export_counter),
            "call_index_within_frame": int(calls_for_frame),
            "call_context": str(call_context),
            "call_origin": str(call_origin),
            "segmentation_refresh_frame": bool(segmentation_refresh_frame),
            "duplicate_same_frame": bool(duplicate),
            "duration_sec": _round_float(metrics.get("total_sec")),
            "room_export_sec": _round_float(metrics.get("room_export_sec")),
            "vertical_transition_export_sec": _round_float(metrics.get("vertical_transition_export_sec")),
            "object_export_sec": _round_float(metrics.get("object_export_sec")),
            "scene_graph_build_sec": _round_float(metrics.get("scene_graph_build_sec")),
            "spatial_relations_sec": _round_float(metrics.get("spatial_relations_sec")),
            "anchor_build_sec": _round_float(metrics.get("anchor_build_sec")),
            "diagnostics_export_sec": _round_float(metrics.get("diagnostics_export_sec")),
            "room_count": _safe_int(metrics.get("room_count")),
            "gateway_count": _safe_int(metrics.get("gateway_count")),
            "vertical_transition_count": _safe_int(metrics.get("vertical_transition_count")),
            "object_count": _safe_int(metrics.get("object_count")),
            "anchor_count": _safe_int(metrics.get("anchor_count")),
        }
        self.export_calls.append(row)

        frame_row["vector_map_export_call_count"] = int(calls_for_frame)
        contexts = [item for item in str(frame_row.get("vector_map_export_call_contexts") or "").split("|") if item]
        contexts.append(str(call_context))
        frame_row["vector_map_export_call_contexts"] = "|".join(contexts)
        frame_row["vector_map_export_duplicate_same_frame"] = bool(duplicate)
        if bool(segmentation_refresh_frame) and duplicate:
            frame_row["segmentation_refresh_triggered_duplicate_export"] = True

    def log_history_scope(
        self,
        frame_idx: int,
        *,
        step_name: str,
        scope_label: str,
        candidate_pool_before: Any,
        candidate_pool_after: Any,
        retained_history_pool: Any,
        filter_description: str,
        notes: str = "",
    ) -> None:
        self.history_scope_rows.append(
            {
                "sequence_id": self.sequence_id,
                "frame_idx": int(frame_idx),
                "step_name": str(step_name),
                "scope_label": str(scope_label),
                "candidate_pool_before": _safe_int(candidate_pool_before),
                "candidate_pool_after": _safe_int(candidate_pool_after),
                "retained_history_pool": _safe_int(retained_history_pool),
                "filter_description": str(filter_description),
                "notes": str(notes),
            }
        )
        frame_row = self.ensure_frame(frame_idx)
        frame_row["history_scope_step_count"] = int(frame_row.get("history_scope_step_count", 0) or 0) + 1
        labels = [item for item in str(frame_row.get("history_scope_labels") or "").split("|") if item]
        labels.append(str(scope_label))
        frame_row["history_scope_labels"] = "|".join(labels)

    def log_segmentation_run(self, payload: Dict[str, Any]) -> None:
        row = dict(payload)
        row["sequence_id"] = self.sequence_id
        run_key = self._segmentation_run_key(row)
        existing_index = self._segmentation_run_index.get(run_key)
        if existing_index is None:
            self._segmentation_run_index[run_key] = len(self.segmentation_runs)
            self.segmentation_runs.append(row)
            return
        self.segmentation_runs[existing_index] = row

    def sync_segmentation_runs(self, rows: Sequence[Dict[str, Any]]) -> None:
        for row in rows:
            self.log_segmentation_run(row)

    def finalize(self) -> Dict[str, Any]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        profiled_rows = [row for _, row in sorted(self.frames.items()) if row.get("profiled_frame")]
        export_rows = list(self.export_calls)
        history_rows = list(self.history_scope_rows)
        segmentation_rows = list(self.segmentation_runs)

        for row in profiled_rows:
            missing = [field for field in self.FRAME_FIELDNAMES if field not in row]
            for field in missing:
                row[field] = None

        self._write_csv(self.output_dir / "per_profiled_frame.csv", profiled_rows, self.FRAME_FIELDNAMES)
        self._write_csv(self.output_dir / "vector_map_export_calls.csv", export_rows, self.EXPORT_FIELDNAMES)
        self._write_csv(self.output_dir / "history_scope.csv", history_rows, self.HISTORY_SCOPE_FIELDNAMES)
        self._write_csv(self.output_dir / "segmentation_runs.csv", segmentation_rows, self.SEGMENTATION_FIELDNAMES)

        self.final_summary = self._build_summary(profiled_rows, export_rows, history_rows, segmentation_rows)
        with open(self.output_dir / "summary.json", "w", encoding="utf-8") as handle:
            json.dump(self.final_summary, handle, indent=2)
        return self.final_summary

    def _build_summary(
        self,
        profiled_rows: Sequence[Dict[str, Any]],
        export_rows: Sequence[Dict[str, Any]],
        history_rows: Sequence[Dict[str, Any]],
        segmentation_rows: Sequence[Dict[str, Any]],
    ) -> Dict[str, Any]:
        def _max_row(key: str) -> Optional[Dict[str, Any]]:
            candidates = [row for row in profiled_rows if row.get(key) is not None]
            if not candidates:
                return None
            return max(candidates, key=lambda item: float(item.get(key) or 0.0))

        worst_stage5 = _max_row("stage5_total_sec")
        worst_stage3 = _max_row("stage3_total_sec")
        worst_total = _max_row("total_step_sec")
        export_duplicates = [row for row in export_rows if row.get("duplicate_same_frame")]

        summary = {
            "sequence_id": self.sequence_id,
            "profiled_frame_count": int(len(profiled_rows)),
            "export_call_count": int(len(export_rows)),
            "history_scope_row_count": int(len(history_rows)),
            "segmentation_run_count": int(len(segmentation_rows)),
            "duplicate_export_frame_count": int(len({int(row["frame_idx"]) for row in export_duplicates})),
            "duplicate_export_call_count": int(len(export_duplicates)),
            "history_scope_labels": _safe_counter(row.get("scope_label") for row in history_rows),
            "export_call_contexts": _safe_counter(row.get("call_context") for row in export_rows),
            "stage5_total_stats_sec": _safe_stats([row.get("stage5_total_sec") for row in profiled_rows], digits=6),
            "stage3_total_stats_sec": _safe_stats([row.get("stage3_total_sec") for row in profiled_rows], digits=6),
            "total_step_stats_sec": _safe_stats([row.get("total_step_sec") for row in profiled_rows], digits=6),
            "box_count_stats": _safe_stats([row.get("total_retained_object_count") for row in profiled_rows]),
            "per_frame_ins_stats": _safe_stats([row.get("per_frame_ins_count") for row in profiled_rows]),
            "merged_point_stats": _safe_stats([row.get("merged_point_count_after_downsample") for row in profiled_rows]),
            "grid_area_stats": _safe_stats([row.get("grid_area") for row in profiled_rows]),
            "worst_frames": {
                "stage5_total_sec": None if worst_stage5 is None else {
                    "frame_idx": int(worst_stage5["frame_idx"]),
                    "value_sec": _round_float(worst_stage5.get("stage5_total_sec")),
                    "total_retained_object_count": _safe_int(worst_stage5.get("total_retained_object_count")),
                    "per_frame_ins_count": _safe_int(worst_stage5.get("per_frame_ins_count")),
                    "vector_map_export_call_count": _safe_int(worst_stage5.get("vector_map_export_call_count")),
                },
                "stage3_total_sec": None if worst_stage3 is None else {
                    "frame_idx": int(worst_stage3["frame_idx"]),
                    "value_sec": _round_float(worst_stage3.get("stage3_total_sec")),
                    "merged_point_count_after_downsample": _safe_int(worst_stage3.get("merged_point_count_after_downsample")),
                    "grid_area": _safe_int(worst_stage3.get("grid_area")),
                    "gateway_count": _safe_int(worst_stage3.get("gateway_count")),
                },
                "total_step_sec": None if worst_total is None else {
                    "frame_idx": int(worst_total["frame_idx"]),
                    "value_sec": _round_float(worst_total.get("total_step_sec")),
                    "stage3_total_sec": _round_float(worst_total.get("stage3_total_sec")),
                    "stage5_total_sec": _round_float(worst_total.get("stage5_total_sec")),
                },
            },
        }
        return summary

    def _write_csv(self, path: Path, rows: Sequence[Dict[str, Any]], fieldnames: Sequence[str]) -> None:
        with open(path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row.get(field) for field in fieldnames})

    def _segmentation_run_key(self, row: Dict[str, Any]) -> str:
        run_id = row.get("run_id")
        if run_id not in (None, ""):
            return str(run_id)
        return "|".join(
            [
                str(row.get("frame_idx")),
                str(row.get("floor_id")),
                str(row.get("trigger_reason")),
                str(row.get("active_status")),
            ]
        )
