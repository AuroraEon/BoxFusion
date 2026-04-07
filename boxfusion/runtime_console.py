from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional


def _normalize_log_level(value: Optional[str]) -> str:
    text = str(value or "summary").strip().lower()
    if text not in {"summary", "verbose"}:
        return "summary"
    return text


class RuntimeConsoleLogger:
    """Keeps runtime stdout concise while preserving important progress messages."""

    def __init__(
        self,
        *,
        sequence_id: Optional[str] = None,
        quiet: bool = False,
        log_level: str = "summary",
        runtime_print_interval: Optional[int] = None,
        per_profiled_frame_stdout: Optional[bool] = None,
    ) -> None:
        self.sequence_id = str(sequence_id or "")
        self.quiet = bool(quiet)
        self.log_level = _normalize_log_level(log_level)
        self.runtime_print_interval = None
        if runtime_print_interval not in (None, 0):
            self.runtime_print_interval = max(1, int(runtime_print_interval))
        if per_profiled_frame_stdout is None:
            per_profiled_frame_stdout = self.log_level == "verbose"
        self.per_profiled_frame_stdout = bool(per_profiled_frame_stdout and not self.quiet)
        self._last_progress_processed: Optional[int] = None

    def bind_sequence(self, sequence_id: Optional[str]) -> None:
        if sequence_id not in (None, ""):
            self.sequence_id = str(sequence_id)

    def is_verbose(self) -> bool:
        return (not self.quiet) and self.log_level == "verbose"

    def allows_profiled_frame_stdout(self) -> bool:
        return bool(self.per_profiled_frame_stdout)

    def info(self, message: str) -> None:
        if self.quiet:
            return
        print(message, flush=True)

    def warning(self, message: str) -> None:
        print(f"[warning] {message}", flush=True)

    def error(self, message: str) -> None:
        print(f"[error] {message}", flush=True)

    def scene_start(
        self,
        *,
        total_frames: Optional[int],
        keyframe_gap: int,
        room_seg_interval: int,
        instrumentation_dir: str,
    ) -> None:
        if self.quiet:
            return
        total_text = "unknown" if total_frames is None else str(int(total_frames))
        prefix = f"[scene] {self.sequence_id}" if self.sequence_id else "[scene]"
        print(
            f"{prefix} start | total_frames={total_text} | keyframe_gap={int(keyframe_gap)} | "
            f"room_seg_interval={int(room_seg_interval)}",
            flush=True,
        )
        print(f"[runtime] detailed instrumentation -> {instrumentation_dir}", flush=True)

    def progress(self, *, processed_frames: int, total_frames: Optional[int]) -> None:
        if self.quiet or self.runtime_print_interval is None:
            return
        processed_frames = int(processed_frames)
        if processed_frames <= 0:
            return
        if processed_frames % int(self.runtime_print_interval) != 0:
            return
        if self._last_progress_processed == processed_frames:
            return
        self._last_progress_processed = processed_frames
        if total_frames is None:
            suffix = f"{processed_frames} frames"
        else:
            suffix = f"{processed_frames}/{int(total_frames)} frames"
        prefix = f"[scene] {self.sequence_id}" if self.sequence_id else "[scene]"
        print(f"{prefix} progress | processed={suffix}", flush=True)

    def segmentation_refresh(
        self,
        *,
        frame_idx: int,
        active_chunks: int,
        updated: bool,
        room_count: Optional[int] = None,
        object_count: Optional[int] = None,
        anchor_count: Optional[int] = None,
    ) -> None:
        if not self.is_verbose():
            return
        counts = []
        if room_count is not None:
            counts.append(f"rooms={int(room_count)}")
        if object_count is not None:
            counts.append(f"objects={int(object_count)}")
        if anchor_count is not None:
            counts.append(f"anchors={int(anchor_count)}")
        counts_text = ""
        if counts:
            counts_text = " | " + " ".join(counts)
        status = "updated" if updated else "skipped"
        print(
            f"[segmentation] frame={int(frame_idx)} | pending_chunks={int(active_chunks)} | status={status}{counts_text}",
            flush=True,
        )

    def profiled_frame_timing(
        self,
        *,
        frame_idx: int,
        data_preprocess_sec: float,
        inference_sec: float,
        stage3_sec: float,
        rerun_sec: float,
        stage5_sec: float,
    ) -> None:
        if not self.allows_profiled_frame_stdout():
            return
        print(
            f"[profiled-frame] frame={int(frame_idx)} | "
            f"data={float(data_preprocess_sec):.4f}s | infer={float(inference_sec):.4f}s | "
            f"stage3={float(stage3_sec):.4f}s | rerun={float(rerun_sec):.4f}s | stage5={float(stage5_sec):.4f}s",
            flush=True,
        )

    def scene_finish(
        self,
        *,
        processed_frames: int,
        duration_sec: float,
        fps: float,
        output_paths: Optional[Dict[str, Any]] = None,
    ) -> None:
        prefix = f"[scene] {self.sequence_id}" if self.sequence_id else "[scene]"
        print(
            f"{prefix} finish | processed_frames={int(processed_frames)} | "
            f"duration_sec={float(duration_sec):.2f} | average_fps={float(fps):.2f}",
            flush=True,
        )
        if output_paths:
            for key, value in output_paths.items():
                if value in (None, "", []):
                    continue
                path_text = str(value)
                if key.endswith("_dir") or key.endswith("_path") or key.endswith("_csv") or key.endswith("_json"):
                    path_text = str(Path(path_text))
                print(f"[output] {key}={path_text}", flush=True)
