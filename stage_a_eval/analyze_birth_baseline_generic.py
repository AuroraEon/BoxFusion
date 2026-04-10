#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
from stage_a_demo import _load_config
from stage_a_eval.analyze_00843_birth_baseline import (
    _attach_birth_masks,
    _collect_birth_events,
    _compute_event_metrics,
    _probe_event_support,
)
from stage_a_eval.analyze_00843_right_pocket_bridge_decomposition import _load_frame_points
from tools.utils import get_dataset


DEFAULT_DEBUG_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_CAPTURE_ROOT = Path("tmp/analysis/generated_debug_room")
DEFAULT_OUTPUT = Path("tmp/analysis/birth_baseline_generic.json")


def _count_tracking_reports(debug_room_dir: Path) -> int:
    if not debug_room_dir.exists():
        return 0
    return sum(1 for _ in debug_room_dir.glob("floor_*/run_*_12_tracking_report.json"))


def _capture_debug_room_artifacts(
    sequence_id: str,
    config_path: str,
    output_scene_root: Path,
    room_seg_interval: int,
) -> Dict[str, Any]:
    output_scene_root.mkdir(parents=True, exist_ok=True)
    debug_room_dir = output_scene_root / "debug_room"
    debug_room_dir.mkdir(parents=True, exist_ok=True)

    cfg = _load_config("hm3d", config_path, sequence_id)
    dataset = get_dataset(cfg)
    if hasattr(dataset, "load_arkit_depth"):
        dataset.load_arkit_depth = True
    gap = int(cfg["data"]["gap"])
    total_frames = int(len(dataset.poses))
    room_segmenter = FloorAwareRoomSegmenter(resolution=0.05, config=cfg)

    for frame_idx in range(total_frames):
        pose = np.asarray(dataset.poses[frame_idx], dtype=np.float32)
        is_keyframe = frame_idx % gap == 0
        points = None
        if is_keyframe:
            points = _load_frame_points(dataset, frame_idx, voxel_size=0.05)
        room_segmenter.observe_frame(
            frame_idx=frame_idx,
            timestamp=float(frame_idx),
            pose_matrix=pose,
            points_xyzrgb=points,
            is_keyframe=bool(is_keyframe),
        )
        if frame_idx % int(room_seg_interval) == 0:
            room_segmenter.perform_segmentation(
                all_pred_box=None,
                debug_path=str(debug_room_dir),
                count=frame_idx,
            )

    final_frame_idx = max(total_frames - 1, 0)
    final_flush_report = room_segmenter.flush_pending_floor_segments(
        all_pred_box=None,
        debug_path=str(debug_room_dir),
        count=final_frame_idx,
    )

    latest_vector_map: Optional[Dict[str, Any]] = None
    if room_segmenter.last_room_markers is not None:
        latest_vector_map = room_segmenter.get_vector_map_data(
            all_pred_box=None,
            count=final_frame_idx,
            save_scene_graph_vis=False,
            scene_graph_vis_dir=str(debug_room_dir),
            instrumentation_context="birth_baseline_debug_capture",
        )
        floor_diag_dir = output_scene_root / "floor_diagnostics"
        room_segmenter.save_floor_diagnostics(
            output_dir=str(floor_diag_dir),
            vector_map=latest_vector_map,
            total_frames=total_frames,
            last_segmentation_frame_idx=int(final_frame_idx),
            room_seg_interval=int(room_seg_interval),
            sequence_id=sequence_id,
        )

    capture_summary = {
        "sequence_id": sequence_id,
        "config_path": config_path,
        "output_scene_root": str(output_scene_root),
        "debug_room_dir": str(debug_room_dir),
        "room_seg_interval": int(room_seg_interval),
        "total_frames": int(total_frames),
        "keyframe_gap": int(gap),
        "final_frame_idx": int(final_frame_idx),
        "final_flush_report": final_flush_report,
        "tracking_report_count": int(_count_tracking_reports(debug_room_dir)),
        "vector_map_written": bool(latest_vector_map is not None),
    }
    summary_path = output_scene_root / "capture_summary.json"
    summary_path.write_text(json.dumps(capture_summary, indent=2), encoding="utf-8")
    return capture_summary


def _ensure_debug_room_artifacts(
    sequence_id: str,
    config_path: str,
    debug_root: Path,
    capture_root: Path,
    room_seg_interval: int,
    force_recapture: bool,
) -> Dict[str, Any]:
    retained_scene_root = debug_root / sequence_id
    retained_debug_room_dir = retained_scene_root / "debug_room"
    retained_count = _count_tracking_reports(retained_debug_room_dir)
    if retained_count > 0 and not force_recapture:
        return {
            "mode": "retained_scene_root",
            "debug_root": str(debug_root),
            "scene_root": str(retained_scene_root),
            "debug_room_dir": str(retained_debug_room_dir),
            "tracking_report_count": int(retained_count),
            "room_seg_interval": int(room_seg_interval),
            "generated": False,
        }

    generated_scene_root = capture_root / sequence_id
    generated_debug_room_dir = generated_scene_root / "debug_room"
    generated_count = _count_tracking_reports(generated_debug_room_dir)
    if generated_count == 0 or force_recapture:
        capture_summary = _capture_debug_room_artifacts(
            sequence_id=sequence_id,
            config_path=config_path,
            output_scene_root=generated_scene_root,
            room_seg_interval=room_seg_interval,
        )
        generated_count = int(capture_summary["tracking_report_count"])
    if generated_count <= 0:
        raise RuntimeError(f"No tracking reports found for {sequence_id} after debug-room capture.")
    return {
        "mode": "generated_tmp_capture",
        "debug_root": str(capture_root),
        "scene_root": str(generated_scene_root),
        "debug_room_dir": str(generated_debug_room_dir),
        "tracking_report_count": int(generated_count),
        "room_seg_interval": int(room_seg_interval),
        "generated": True,
    }


def build_report(
    sequence_id: str,
    config_path: str,
    debug_root: Path,
    capture_root: Path,
    margin_px: int,
    stability_future_matched_runs: int,
    room_seg_interval: int,
    force_recapture: bool,
) -> Dict[str, Any]:
    debug_source = _ensure_debug_room_artifacts(
        sequence_id=sequence_id,
        config_path=config_path,
        debug_root=debug_root,
        capture_root=capture_root,
        room_seg_interval=room_seg_interval,
        force_recapture=force_recapture,
    )
    effective_debug_root = Path(debug_source["debug_root"])
    selection = _collect_birth_events(
        debug_root=effective_debug_root,
        sequence_id=sequence_id,
        stability_future_matched_runs=stability_future_matched_runs,
    )
    included_events = [
        _attach_birth_masks(event, debug_root=effective_debug_root, margin_px=margin_px)
        for event in selection["included_events"]
    ]
    if not included_events:
        raise RuntimeError("No included birth events found.")

    events_by_frame: Dict[int, List[Dict[str, Any]]] = {}
    for event in included_events:
        events_by_frame.setdefault(int(event["frame_idx"]), []).append(event)

    cfg = _load_config("hm3d", config_path, sequence_id)
    dataset = get_dataset(cfg)
    if hasattr(dataset, "load_arkit_depth"):
        dataset.load_arkit_depth = True
    gap = int(cfg["data"]["gap"])
    room_segmenter = FloorAwareRoomSegmenter(resolution=0.05, config=cfg)

    measured_events: List[Dict[str, Any]] = []
    max_frame = max(int(event["frame_idx"]) for event in included_events)
    for frame_idx in range(0, int(max_frame) + 1):
        pose = np.asarray(dataset.poses[frame_idx], dtype=np.float32)
        is_keyframe = frame_idx % gap == 0
        points = None
        if is_keyframe:
            points = _load_frame_points(dataset, frame_idx, voxel_size=0.05)
        room_segmenter.observe_frame(
            frame_idx=frame_idx,
            timestamp=float(frame_idx),
            pose_matrix=pose,
            points_xyzrgb=points,
            is_keyframe=bool(is_keyframe),
        )

        for event in events_by_frame.get(frame_idx, []):
            active_floor_id = str((room_segmenter.last_floor_observation or {}).get("floor_id"))
            if active_floor_id != str(event["floor_id"]):
                raise RuntimeError(
                    f"Expected active floor {event['floor_id']} at frame {frame_idx}, got {active_floor_id}"
                )
            probe = _probe_event_support(
                room_segmenter=room_segmenter,
                floor_id=str(event["floor_id"]),
                frame_idx=int(frame_idx),
                roi_bbox=event["roi_bbox_xyxy"],
            )
            measured_events.append(
                {
                    "event_id": event["event_id"],
                    "sequence_id": event["sequence_id"],
                    "floor_id": event["floor_id"],
                    "frame_idx": int(event["frame_idx"]),
                    "global_id": int(event["global_id"]),
                    "marker_label": int(event["marker_label"]),
                    "bootstrap_birth": bool(event["bootstrap_birth"]),
                    "later_matched_frames": list(event["later_matched_frames"]),
                    "later_observed_frames": list(event["later_observed_frames"]),
                    "later_retained_missing_frames": list(event["later_retained_missing_frames"]),
                    "later_dropped_frames": list(event["later_dropped_frames"]),
                    "roi_bbox_xyxy": list(event["roi_bbox_xyxy"]),
                    "metrics": _compute_event_metrics(event, probe),
                }
            )

        if frame_idx < int(max_frame) and frame_idx % int(room_seg_interval) == 0:
            room_segmenter.perform_segmentation(debug_path=None, count=frame_idx)

    measured_events.sort(key=lambda row: (row["floor_id"], int(row["frame_idx"]), int(row["global_id"])))
    compact_table = [
        {
            "event_id": row["event_id"],
            "floor_id": row["floor_id"],
            "frame_idx": int(row["frame_idx"]),
            "global_id": int(row["global_id"]),
            "later_matched_run_count": int(len(row["later_matched_frames"])),
            "later_observed_run_count": int(len(row["later_observed_frames"])),
            "birth_area_px": int(row["metrics"]["birth_mask"]["area_px"]),
            "seed_area_px": int(row["metrics"]["seed_support"]["area_px"]),
            "raw_birth_px": int(row["metrics"]["birth_mask_partition"]["raw_supported_px"]),
            "raw_seed_px": int(row["metrics"]["seed_support_partition"]["raw_supported_px"]),
            "raw_birth_frac": float(row["metrics"]["guard_readout"]["raw_supported_birth_fraction"]),
            "raw_seed_frac": float(row["metrics"]["guard_readout"]["raw_supported_seed_fraction"]),
            "raw_largest_seed_touching_px": int(row["metrics"]["connectivity"]["raw_largest_seed_touching_component_px"]),
            "blur_needed_for_seed_min_25": bool(row["metrics"]["guard_readout"]["blur_needed_for_seed_min_25"]),
            "close_needed_for_seed_min_25": bool(row["metrics"]["guard_readout"]["close_needed_for_seed_min_25"]),
            "blur_needed_for_any_seed_coverage": bool(row["metrics"]["guard_readout"]["blur_needed_for_any_seed_coverage"]),
            "close_needed_for_any_seed_coverage": bool(row["metrics"]["guard_readout"]["close_needed_for_any_seed_coverage"]),
        }
        for row in measured_events
    ]

    return {
        "sequence_id": sequence_id,
        "analysis_scope": {
            "stability_future_matched_runs": int(stability_future_matched_runs),
            "roi_margin_px": int(margin_px),
            "room_seg_interval": int(room_seg_interval),
        },
        "debug_artifact_source": debug_source,
        "candidate_selection": {
            "definition": {
                "included": (
                    "Post-bootstrap new_rooms events whose global room appears as a normal matched room in at least "
                    f"{int(stability_future_matched_runs)} later segmentation runs on the same floor."
                ),
                "excluded": [
                    "Floor-bootstrap births at the first segmentation on a floor.",
                    "New-room events that never stabilize into ordinary matched tracking presence.",
                ],
            },
            "floors_summary": selection["floors_summary"],
            "included_events": [
                {
                    key: value
                    for key, value in event.items()
                    if not key.startswith("_")
                }
                for event in included_events
            ],
            "excluded_events": selection["excluded_events"],
        },
        "measured_events": measured_events,
        "compact_table": compact_table,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a generic in-sequence room-birth baseline.")
    parser.add_argument("--sequence-id", required=True)
    parser.add_argument("--config", default="config/hm3d.yaml")
    parser.add_argument("--debug-root", default=str(DEFAULT_DEBUG_ROOT))
    parser.add_argument("--capture-root", default=str(DEFAULT_CAPTURE_ROOT))
    parser.add_argument("--roi-margin-px", type=int, default=24)
    parser.add_argument("--stability-future-matched-runs", type=int, default=2)
    parser.add_argument("--room-seg-interval", type=int, default=100)
    parser.add_argument("--force-recapture", action="store_true")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    report = build_report(
        sequence_id=str(args.sequence_id),
        config_path=str(args.config),
        debug_root=Path(args.debug_root),
        capture_root=Path(args.capture_root),
        margin_px=int(args.roi_margin_px),
        stability_future_matched_runs=int(args.stability_future_matched_runs),
        room_seg_interval=int(args.room_seg_interval),
        force_recapture=bool(args.force_recapture),
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
