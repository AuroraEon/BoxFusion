#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
from stage_a_demo import _load_config
from stage_a_eval.analyze_00843_right_pocket_bridge_decomposition import (
    _component_stats,
    _component_summaries,
    _downsample_points,
    _embed_mask_in_requested_roi,
    _labels_touching_seed,
    _largest_component_area,
    _load_frame_points,
    _mask_bbox,
    _seed_connected_area,
)
from tools.utils import get_dataset


DEFAULT_SEQUENCE_ID = "00843-DYehNKdT76V"
DEFAULT_DEBUG_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_OUTPUT = Path("tmp/analysis/00843_birth_baseline.json")
TRACKING_SUFFIX = "_12_tracking_report.json"
SEED_MIN_AREA_PX = 25


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _expanded_bbox(mask: np.ndarray, margin: int) -> Tuple[int, int, int, int]:
    ys, xs = np.where(mask)
    if len(xs) == 0 or len(ys) == 0:
        raise ValueError("mask is empty")
    x0 = max(0, int(xs.min()) - margin)
    y0 = max(0, int(ys.min()) - margin)
    x1 = min(mask.shape[1], int(xs.max()) + 1 + margin)
    y1 = min(mask.shape[0], int(ys.max()) + 1 + margin)
    return x0, y0, x1, y1


def _reverse_tracking_labels(tracking_payload: Dict[str, Any]) -> Dict[int, int]:
    lookup: Dict[int, int] = {}
    tracking = dict(tracking_payload.get("tracking") or {})
    for row in list(tracking.get("matched") or []) + list(tracking.get("new_rooms") or []):
        marker_label = row.get("marker_label")
        global_id = row.get("global_id")
        if marker_label is None or global_id is None:
            continue
        lookup[int(global_id)] = int(marker_label)
    return lookup


def _parse_frame_idx(path: Path) -> int:
    match = re.search(r"run_(\d+)_12_tracking_report\.json$", path.name)
    if match is None:
        raise ValueError(f"Could not parse frame index from {path}")
    return int(match.group(1))


def _list_tracking_paths(debug_floor_dir: Path) -> List[Path]:
    return sorted(debug_floor_dir.glob(f"*{TRACKING_SUFFIX}"), key=_parse_frame_idx)


def _largest_seed_touching_component_area(mask: np.ndarray, seed_mask: np.ndarray) -> int:
    labels, stats = _component_stats(mask)
    touched = _labels_touching_seed(mask, seed_mask)
    if not touched:
        return 0
    return max(int(stats[label_idx, cv2.CC_STAT_AREA]) for label_idx in touched)


def _seed_touching_component_areas(mask: np.ndarray, seed_mask: np.ndarray) -> List[int]:
    labels, stats = _component_stats(mask)
    touched = _labels_touching_seed(mask, seed_mask)
    return sorted((int(stats[label_idx, cv2.CC_STAT_AREA]) for label_idx in touched), reverse=True)


def _presence_frames_for_gid(runs: Sequence[Dict[str, Any]], gid: int, key: str) -> List[int]:
    frames: List[int] = []
    for run in runs:
        tracking = run["tracking"]
        for row in tracking.get(key, []):
            if int(row["global_id"]) == int(gid):
                frames.append(int(run["frame_idx"]))
    return frames


def _collect_birth_events(
    debug_root: Path,
    sequence_id: str,
    stability_future_matched_runs: int,
) -> Dict[str, Any]:
    base = debug_root / sequence_id / "debug_room"
    included: List[Dict[str, Any]] = []
    excluded: List[Dict[str, Any]] = []
    floors_summary: List[Dict[str, Any]] = []

    for debug_floor_dir in sorted(path for path in base.iterdir() if path.is_dir() and path.name.startswith("floor_")):
        tracking_paths = _list_tracking_paths(debug_floor_dir)
        if not tracking_paths:
            continue
        runs: List[Dict[str, Any]] = []
        for tracking_path in tracking_paths:
            payload = _load_json(tracking_path)
            runs.append(
                {
                    "frame_idx": _parse_frame_idx(tracking_path),
                    "tracking": dict(payload.get("tracking") or {}),
                }
            )

        floors_summary.append(
            {
                "floor_id": debug_floor_dir.name,
                "run_frames": [int(run["frame_idx"]) for run in runs],
            }
        )

        for run_index, run in enumerate(runs):
            frame_idx = int(run["frame_idx"])
            tracking = run["tracking"]
            for row in tracking.get("new_rooms", []):
                gid = int(row["global_id"])
                marker_label = int(row["marker_label"])
                later_matched_frames = [frame for frame in _presence_frames_for_gid(runs, gid, "matched") if frame > frame_idx]
                later_observed_frames = sorted(
                    {
                        frame
                        for key in ("matched", "retained_missing", "new_rooms")
                        for frame in _presence_frames_for_gid(runs, gid, key)
                        if frame > frame_idx
                    }
                )
                later_retained_missing_frames = [
                    frame for frame in _presence_frames_for_gid(runs, gid, "retained_missing") if frame > frame_idx
                ]
                later_dropped_frames = [frame for frame in _presence_frames_for_gid(runs, gid, "dropped_missing") if frame > frame_idx]
                bootstrap_birth = run_index == 0
                future_matched_ok = len(later_matched_frames) >= int(stability_future_matched_runs)
                include_event = (not bootstrap_birth) and future_matched_ok
                reason_parts: List[str] = []
                if bootstrap_birth:
                    reason_parts.append("bootstrap_floor_birth")
                if not future_matched_ok:
                    reason_parts.append(
                        f"future_matched_runs_lt_{int(stability_future_matched_runs)}"
                    )
                event = {
                    "event_id": f"{debug_floor_dir.name}_frame_{frame_idx}_gid_{gid}",
                    "sequence_id": sequence_id,
                    "floor_id": debug_floor_dir.name,
                    "frame_idx": int(frame_idx),
                    "global_id": int(gid),
                    "marker_label": int(marker_label),
                    "bootstrap_birth": bool(bootstrap_birth),
                    "later_matched_frames": [int(v) for v in later_matched_frames],
                    "later_observed_frames": [int(v) for v in later_observed_frames],
                    "later_retained_missing_frames": [int(v) for v in later_retained_missing_frames],
                    "later_dropped_frames": [int(v) for v in later_dropped_frames],
                    "stability_future_matched_runs_required": int(stability_future_matched_runs),
                    "include_in_baseline": bool(include_event),
                    "inclusion_reason": "incremental_birth_with_future_matched_presence"
                    if include_event
                    else ",".join(reason_parts),
                }
                if include_event:
                    included.append(event)
                else:
                    excluded.append(event)

    return {
        "included_events": included,
        "excluded_events": excluded,
        "floors_summary": floors_summary,
    }


def _attach_birth_masks(
    event: Dict[str, Any],
    debug_root: Path,
    margin_px: int,
) -> Dict[str, Any]:
    debug_floor_dir = debug_root / event["sequence_id"] / "debug_room" / event["floor_id"]
    frame_idx = int(event["frame_idx"])
    tracking_payload = _load_json(debug_floor_dir / f"run_{frame_idx}{TRACKING_SUFFIX}")
    label_by_gid = _reverse_tracking_labels(tracking_payload)
    marker_label = int(label_by_gid[int(event["global_id"])])
    final_labels = np.load(debug_floor_dir / f"run_{frame_idx}_09_final_labels.npy")
    pre_markers = np.load(debug_floor_dir / f"run_{frame_idx}_08_pre_watershed_markers.npy")
    birth_mask = final_labels == marker_label
    seed_support = pre_markers == marker_label
    focus_mask = birth_mask | seed_support
    if not np.any(focus_mask):
        focus_mask = birth_mask
    if not np.any(focus_mask):
        raise RuntimeError(f"Empty focus mask for {event['event_id']}")
    roi_bbox = _expanded_bbox(focus_mask, margin=margin_px)
    return {
        **event,
        "marker_label": int(marker_label),
        "roi_bbox_xyxy": [int(v) for v in roi_bbox],
        "birth_mask_shape_hw": [int(final_labels.shape[0]), int(final_labels.shape[1])],
        "_birth_mask": birth_mask,
        "_seed_support": seed_support,
    }


def _probe_event_support(
    room_segmenter: FloorAwareRoomSegmenter,
    floor_id: str,
    frame_idx: int,
    roi_bbox: Sequence[int],
) -> Dict[str, Any]:
    floor_state = room_segmenter.floor_states[str(floor_id)]
    existing_points = floor_state.merged_points_xyzrgb
    if existing_points is None or len(existing_points) == 0:
        raise RuntimeError(f"Expected merged floor points for {floor_id} at frame {frame_idx}")
    merged = np.concatenate([existing_points, *list(floor_state.pending_chunks)], axis=0)
    merged = _downsample_points(merged, voxel_size=float(room_segmenter.resolution))
    if merged is None:
        raise RuntimeError(f"Downsampled merged prefix was empty for {floor_id} at frame {frame_idx}")
    probe_segmenter = room_segmenter._clone_segmenter_for_debug(floor_state.segmenter)
    probe = probe_segmenter.inspect_pre_outside_boundary_debug(
        np.ascontiguousarray(merged[:, :3], dtype=np.float64),
        frame_id=int(frame_idx),
        roi_bbox=tuple(int(v) for v in roi_bbox),
    )
    if probe.get("failure_reason") is not None:
        raise RuntimeError(f"Probe failed for {floor_id} at frame {frame_idx}: {probe['failure_reason']}")
    return probe


def _fraction(num: int, den: int) -> float:
    if den <= 0:
        return 0.0
    return round(float(num) / float(den), 4)


def _compute_event_metrics(event: Dict[str, Any], probe: Dict[str, Any]) -> Dict[str, Any]:
    x0, y0, x1, y1 = [int(v) for v in event["roi_bbox_xyxy"]]
    offset_xy = (x0, y0)
    birth_roi = event["_birth_mask"][y0:y1, x0:x1]
    seed_roi = event["_seed_support"][y0:y1, x0:x1]
    actual_roi_bbox = probe["roi_bbox_xyxy"]
    masks = {
        stage_name: _embed_mask_in_requested_roi(mask, actual_roi_bbox, event["roi_bbox_xyxy"])
        for stage_name, mask in probe["_roi_masks"].items()
    }

    raw_birth = masks["raw_support"] & birth_roi
    blur_positive_birth = masks["blur_positive"] & birth_roi
    preclose_birth = masks["preclose"] & birth_roi
    postclose_birth = masks["postclose"] & birth_roi
    filled_birth = masks["filled"] & birth_roi
    blur_only_birth = preclose_birth & (~raw_birth)
    close_only_birth = postclose_birth & (~preclose_birth)
    unsupported_birth = birth_roi & (~postclose_birth)

    raw_seed = raw_birth & seed_roi
    preclose_seed = preclose_birth & seed_roi
    postclose_seed = postclose_birth & seed_roi
    blur_only_seed = blur_only_birth & seed_roi
    close_only_seed = close_only_birth & seed_roi

    raw_seed_connected_area = int(_seed_connected_area(raw_birth, seed_roi))
    preclose_seed_connected_area = int(_seed_connected_area(preclose_birth, seed_roi))
    postclose_seed_connected_area = int(_seed_connected_area(postclose_birth, seed_roi))
    raw_largest_seed_touching = int(_largest_seed_touching_component_area(raw_birth, seed_roi))
    preclose_largest_seed_touching = int(_largest_seed_touching_component_area(preclose_birth, seed_roi))
    postclose_largest_seed_touching = int(_largest_seed_touching_component_area(postclose_birth, seed_roi))

    birth_area = int(birth_roi.sum())
    seed_area = int(seed_roi.sum())
    raw_supported_birth_px = int(raw_birth.sum())
    preclose_supported_birth_px = int(preclose_birth.sum())
    postclose_supported_birth_px = int(postclose_birth.sum())
    raw_supported_seed_px = int(raw_seed.sum())
    preclose_supported_seed_px = int(preclose_seed.sum())
    postclose_supported_seed_px = int(postclose_seed.sum())

    return {
        "birth_mask": {
            "area_px": int(birth_area),
            "bbox_xyxy": _mask_bbox(birth_roi, offset_xy),
            **_component_summaries(birth_roi, offset_xy),
        },
        "seed_support": {
            "area_px": int(seed_area),
            "bbox_xyxy": _mask_bbox(seed_roi, offset_xy),
            **_component_summaries(seed_roi, offset_xy),
        },
        "birth_mask_partition": {
            "raw_supported_px": int(raw_supported_birth_px),
            "blur_positive_supported_px": int(blur_positive_birth.sum()),
            "preclose_supported_px": int(preclose_supported_birth_px),
            "postclose_supported_px": int(postclose_supported_birth_px),
            "filled_supported_px": int(filled_birth.sum()),
            "blur_only_px": int(blur_only_birth.sum()),
            "close_only_px": int(close_only_birth.sum()),
            "unsupported_by_postclose_px": int(unsupported_birth.sum()),
        },
        "seed_support_partition": {
            "raw_supported_px": int(raw_supported_seed_px),
            "preclose_supported_px": int(preclose_supported_seed_px),
            "postclose_supported_px": int(postclose_supported_seed_px),
            "blur_only_px": int(blur_only_seed.sum()),
            "close_only_px": int(close_only_seed.sum()),
        },
        "connectivity": {
            "raw_seed_connected_area_px": int(raw_seed_connected_area),
            "preclose_seed_connected_area_px": int(preclose_seed_connected_area),
            "postclose_seed_connected_area_px": int(postclose_seed_connected_area),
            "raw_largest_seed_touching_component_px": int(raw_largest_seed_touching),
            "preclose_largest_seed_touching_component_px": int(preclose_largest_seed_touching),
            "postclose_largest_seed_touching_component_px": int(postclose_largest_seed_touching),
            "raw_seed_touching_component_areas_px": _seed_touching_component_areas(raw_birth, seed_roi),
            "preclose_seed_touching_component_areas_px": _seed_touching_component_areas(preclose_birth, seed_roi),
            "postclose_seed_touching_component_areas_px": _seed_touching_component_areas(postclose_birth, seed_roi),
            "raw_birth_component_count": int(_component_summaries(raw_birth, offset_xy)["component_count"]),
            "preclose_birth_component_count": int(_component_summaries(preclose_birth, offset_xy)["component_count"]),
            "postclose_birth_component_count": int(_component_summaries(postclose_birth, offset_xy)["component_count"]),
            "raw_largest_component_px": int(_largest_component_area(raw_birth)),
            "preclose_largest_component_px": int(_largest_component_area(preclose_birth)),
            "postclose_largest_component_px": int(_largest_component_area(postclose_birth)),
        },
        "guard_readout": {
            "seed_min_area_px": int(SEED_MIN_AREA_PX),
            "raw_supported_birth_fraction": _fraction(raw_supported_birth_px, birth_area),
            "raw_supported_seed_fraction": _fraction(raw_supported_seed_px, seed_area),
            "preclose_supported_seed_fraction": _fraction(preclose_supported_seed_px, seed_area),
            "postclose_supported_seed_fraction": _fraction(postclose_supported_seed_px, seed_area),
            "raw_largest_seed_touching_vs_seed_min_ratio": _fraction(raw_largest_seed_touching, SEED_MIN_AREA_PX),
            "preclose_largest_seed_touching_vs_seed_min_ratio": _fraction(preclose_largest_seed_touching, SEED_MIN_AREA_PX),
            "postclose_largest_seed_touching_vs_seed_min_ratio": _fraction(postclose_largest_seed_touching, SEED_MIN_AREA_PX),
            "blur_needed_for_any_seed_coverage": bool(raw_supported_seed_px == 0 and preclose_supported_seed_px > 0),
            "close_needed_for_any_seed_coverage": bool(preclose_supported_seed_px == 0 and postclose_supported_seed_px > 0),
            "blur_needed_for_seed_min_25": bool(
                raw_largest_seed_touching < SEED_MIN_AREA_PX and preclose_largest_seed_touching >= SEED_MIN_AREA_PX
            ),
            "close_needed_for_seed_min_25": bool(
                preclose_largest_seed_touching < SEED_MIN_AREA_PX and postclose_largest_seed_touching >= SEED_MIN_AREA_PX
            ),
            "blur_only_birth_fraction": _fraction(int(blur_only_birth.sum()), birth_area),
            "close_only_birth_fraction": _fraction(int(close_only_birth.sum()), birth_area),
        },
        "roi_probe": {
            "requested_roi_bbox_xyxy": [int(v) for v in event["roi_bbox_xyxy"]],
            "actual_roi_bbox_xyxy": None if actual_roi_bbox is None else [int(v) for v in actual_roi_bbox],
            "roi_report": dict(probe.get("roi_report") or {}),
        },
    }


def _comparison_snapshot(values: List[float], target_value: float) -> Dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "median": None,
            "max": None,
            "target_value": None,
            "target_minus_median": None,
        }
    return {
        "count": int(len(values)),
        "min": round(float(min(values)), 4),
        "median": round(float(statistics.median(values)), 4),
        "max": round(float(max(values)), 4),
        "target_value": round(float(target_value), 4),
        "target_minus_median": round(float(target_value) - float(statistics.median(values)), 4),
    }


def _rank_snapshot(
    event_rows: Sequence[Dict[str, Any]],
    target_event_id: str,
    metric_path: Sequence[str],
    stronger_if_higher: bool,
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for row in event_rows:
        value: Any = row
        for key in metric_path:
            value = value[key]
        rows.append(
            {
                "event_id": row["event_id"],
                "value": float(value),
            }
        )
    ordered = sorted(
        rows,
        key=lambda item: (item["value"], item["event_id"]),
        reverse=bool(stronger_if_higher),
    )
    target_row = next(item for item in rows if item["event_id"] == target_event_id)
    stronger_rank = 1 + next(index for index, item in enumerate(ordered) if item["event_id"] == target_event_id)
    weaker_ordered = list(reversed(ordered))
    weaker_rank = 1 + next(index for index, item in enumerate(weaker_ordered) if item["event_id"] == target_event_id)
    values = [item["value"] for item in rows]
    return {
        "metric_path": list(metric_path),
        "stronger_if_higher": bool(stronger_if_higher),
        "stronger_rank_1_is_strongest": int(stronger_rank),
        "weaker_rank_1_is_weakest": int(weaker_rank),
        **_comparison_snapshot(values, target_row["value"]),
    }


def _build_comparison_report(
    event_rows: List[Dict[str, Any]],
    target_event_id: str,
) -> Dict[str, Any]:
    target_row = next(row for row in event_rows if row["event_id"] == target_event_id)
    same_floor_rows = [row for row in event_rows if row["floor_id"] == target_row["floor_id"]]
    metric_specs = [
        (("metrics", "birth_mask", "area_px"), True),
        (("metrics", "seed_support", "area_px"), True),
        (("metrics", "birth_mask_partition", "raw_supported_px"), True),
        (("metrics", "seed_support_partition", "raw_supported_px"), True),
        (("metrics", "guard_readout", "raw_supported_birth_fraction"), True),
        (("metrics", "guard_readout", "raw_supported_seed_fraction"), True),
        (("metrics", "connectivity", "raw_seed_connected_area_px"), True),
        (("metrics", "connectivity", "raw_largest_seed_touching_component_px"), True),
        (("metrics", "guard_readout", "blur_only_birth_fraction"), False),
        (("metrics", "guard_readout", "blur_needed_for_seed_min_25"), False),
        (("metrics", "guard_readout", "close_needed_for_seed_min_25"), False),
    ]
    return {
        "target_event_id": target_event_id,
        "same_floor_event_ids": [row["event_id"] for row in same_floor_rows],
        "all_event_ids": [row["event_id"] for row in event_rows],
        "same_floor": {
            "/".join(metric_path): _rank_snapshot(same_floor_rows, target_event_id, metric_path, stronger_if_higher)
            for metric_path, stronger_if_higher in metric_specs
        },
        "all_included": {
            "/".join(metric_path): _rank_snapshot(event_rows, target_event_id, metric_path, stronger_if_higher)
            for metric_path, stronger_if_higher in metric_specs
        },
    }


def build_report(
    sequence_id: str,
    config_path: str,
    debug_root: Path,
    margin_px: int,
    stability_future_matched_runs: int,
    target_floor_id: str,
    target_frame_idx: int,
    target_global_id: int,
) -> Dict[str, Any]:
    selection = _collect_birth_events(
        debug_root=debug_root,
        sequence_id=sequence_id,
        stability_future_matched_runs=stability_future_matched_runs,
    )
    included_events = [
        _attach_birth_masks(event, debug_root=debug_root, margin_px=margin_px)
        for event in selection["included_events"]
    ]
    if not included_events:
        raise RuntimeError("No included birth events found.")

    target_event_id = f"{target_floor_id}_frame_{int(target_frame_idx)}_gid_{int(target_global_id)}"
    if not any(event["event_id"] == target_event_id for event in included_events):
        raise RuntimeError(f"Target event {target_event_id} is not part of the included baseline.")

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

        if frame_idx < int(max_frame) and frame_idx % 100 == 0:
            room_segmenter.perform_segmentation(debug_path=None, count=frame_idx)

    measured_events.sort(key=lambda row: (row["floor_id"], int(row["frame_idx"]), int(row["global_id"])))
    comparison = _build_comparison_report(measured_events, target_event_id=target_event_id)
    compact_table = [
        {
            "event_id": row["event_id"],
            "floor_id": row["floor_id"],
            "frame_idx": int(row["frame_idx"]),
            "global_id": int(row["global_id"]),
            "birth_area_px": int(row["metrics"]["birth_mask"]["area_px"]),
            "seed_area_px": int(row["metrics"]["seed_support"]["area_px"]),
            "raw_birth_px": int(row["metrics"]["birth_mask_partition"]["raw_supported_px"]),
            "raw_seed_px": int(row["metrics"]["seed_support_partition"]["raw_supported_px"]),
            "raw_birth_frac": float(row["metrics"]["guard_readout"]["raw_supported_birth_fraction"]),
            "raw_seed_frac": float(row["metrics"]["guard_readout"]["raw_supported_seed_fraction"]),
            "raw_seed_connected_px": int(row["metrics"]["connectivity"]["raw_seed_connected_area_px"]),
            "raw_largest_seed_touching_px": int(row["metrics"]["connectivity"]["raw_largest_seed_touching_component_px"]),
            "blur_needed_for_seed_min_25": bool(row["metrics"]["guard_readout"]["blur_needed_for_seed_min_25"]),
            "close_needed_for_seed_min_25": bool(row["metrics"]["guard_readout"]["close_needed_for_seed_min_25"]),
        }
        for row in measured_events
    ]

    return {
        "sequence_id": sequence_id,
        "analysis_scope": {
            "target_floor_id": str(target_floor_id),
            "target_frame_idx": int(target_frame_idx),
            "target_global_id": int(target_global_id),
            "target_event_id": target_event_id,
            "stability_future_matched_runs": int(stability_future_matched_runs),
            "roi_margin_px": int(margin_px),
            "seed_min_area_px": int(SEED_MIN_AREA_PX),
        },
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
        "comparison": comparison,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Build an in-sequence room-birth baseline for 00843.")
    parser.add_argument("--sequence-id", default=DEFAULT_SEQUENCE_ID)
    parser.add_argument("--config", default="config/hm3d.yaml")
    parser.add_argument("--debug-root", default=str(DEFAULT_DEBUG_ROOT))
    parser.add_argument("--roi-margin-px", type=int, default=24)
    parser.add_argument("--stability-future-matched-runs", type=int, default=2)
    parser.add_argument("--target-floor-id", default="floor_1")
    parser.add_argument("--target-frame-idx", type=int, default=800)
    parser.add_argument("--target-global-id", type=int, default=6)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    report = build_report(
        sequence_id=str(args.sequence_id),
        config_path=str(args.config),
        debug_root=Path(args.debug_root),
        margin_px=int(args.roi_margin_px),
        stability_future_matched_runs=int(args.stability_future_matched_runs),
        target_floor_id=str(args.target_floor_id),
        target_frame_idx=int(args.target_frame_idx),
        target_global_id=int(args.target_global_id),
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
