#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import open3d as o3d
import torch

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from boxfusion.floor_aware_room_segmenter import FloorAwareRoomSegmenter
from stage_a_demo import _load_config
from tools.utils import get_dataset, unproject


DEFAULT_SEQUENCE_ID = "00843-DYehNKdT76V"
DEFAULT_FRAMES = (725, 750, 775, 800)
DEFAULT_ROI = (1078, 1009, 1143, 1072)
DEFAULT_DEBUG_ROOT = Path("world_model_backend_outputs_v0_2_final/scenes")
DEFAULT_OUTPUT = Path("tmp/analysis/00843_right_pocket_bridge_decomposition.json")


def _read_depth(path: str, depth_scale: float) -> np.ndarray:
    if path.endswith(".png"):
        depth = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    elif path.endswith(".exr"):
        depth = cv2.imread(path, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    else:
        raise ValueError(f"Unsupported depth format: {path}")
    if depth is None:
        raise FileNotFoundError(path)
    return depth.astype(np.float32) / float(depth_scale)


def _downsample_points(points_xyzrgb: np.ndarray, voxel_size: float) -> Optional[np.ndarray]:
    if points_xyzrgb.size == 0:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(points_xyzrgb[:, :3], dtype=np.float64))
    pcd.colors = o3d.utility.Vector3dVector(np.ascontiguousarray(points_xyzrgb[:, 3:6], dtype=np.float64))
    pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return None
    colors = np.asarray(pcd.colors)
    return np.concatenate([points, colors], axis=1)


def _load_frame_points(dataset: Any, frame_idx: int, voxel_size: float) -> Optional[np.ndarray]:
    color_bgr = cv2.imread(dataset.img_files[frame_idx], cv2.IMREAD_COLOR)
    if color_bgr is None:
        raise FileNotFoundError(dataset.img_files[frame_idx])
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    color_rgb = cv2.resize(color_rgb, (dataset.img_width, dataset.img_height))
    depth = _read_depth(dataset.depth_paths[frame_idx], dataset.depth_scale)
    depth = cv2.resize(depth, (dataset.img_width, dataset.img_height))

    depth_t = torch.from_numpy(depth).float()
    image_t = torch.from_numpy(color_rgb)
    pose_t = torch.from_numpy(dataset.poses[frame_idx].astype(np.float32))
    K = torch.tensor(
        [
            [dataset.fx, 0.0, dataset.cx],
            [0.0, dataset.fy, dataset.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=torch.float32,
    )

    xyz, valid = unproject(depth_t, K, pose_t, max_depth=8.0)
    xyzrgb = torch.cat((xyz, image_t / 255.0), dim=-1)[valid]
    if xyzrgb.numel() == 0:
        return None
    return _downsample_points(xyzrgb.cpu().numpy(), voxel_size)


def _mask_bbox(mask: np.ndarray, offset_xy: Tuple[int, int] = (0, 0)) -> Optional[List[int]]:
    ys, xs = np.where(mask)
    if len(xs) == 0 or len(ys) == 0:
        return None
    ox, oy = offset_xy
    return [
        int(xs.min()) + ox,
        int(ys.min()) + oy,
        int(xs.max()) + 1 + ox,
        int(ys.max()) + 1 + oy,
    ]


def _component_summaries(mask: np.ndarray, offset_xy: Tuple[int, int] = (0, 0)) -> Dict[str, Any]:
    mask_u8 = (mask > 0).astype(np.uint8)
    if int(mask_u8.sum()) == 0:
        return {"component_count": 0, "components": []}
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    ox, oy = offset_xy
    components: List[Dict[str, Any]] = []
    for label_idx in range(1, num_labels):
        x = int(stats[label_idx, cv2.CC_STAT_LEFT])
        y = int(stats[label_idx, cv2.CC_STAT_TOP])
        w = int(stats[label_idx, cv2.CC_STAT_WIDTH])
        h = int(stats[label_idx, cv2.CC_STAT_HEIGHT])
        component_mask = labels == label_idx
        components.append(
            {
                "label": int(label_idx),
                "area_px": int(stats[label_idx, cv2.CC_STAT_AREA]),
                "bbox_xyxy": [x + ox, y + oy, x + w + ox, y + h + oy],
            }
        )
    components.sort(key=lambda item: (-int(item["area_px"]), item["bbox_xyxy"]))
    return {"component_count": int(len(components)), "components": components}


def _component_stats(mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    mask_u8 = (mask > 0).astype(np.uint8)
    if int(mask_u8.sum()) == 0:
        return np.zeros(mask.shape, dtype=np.int32), np.zeros((1, 5), dtype=np.int32)
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    return labels, stats


def _labels_touching_seed(mask: np.ndarray, seed_mask: np.ndarray) -> List[int]:
    labels, _ = _component_stats(mask)
    touched = sorted(int(v) for v in np.unique(labels[seed_mask > 0]) if int(v) > 0)
    return touched


def _seed_connected_area(mask: np.ndarray, seed_mask: np.ndarray) -> int:
    labels, _ = _component_stats(mask)
    touched = _labels_touching_seed(mask, seed_mask)
    if not touched:
        return 0
    touched_set = set(touched)
    return int(np.isin(labels, list(touched_set)).sum())


def _largest_component_area(mask: np.ndarray) -> int:
    _, stats = _component_stats(mask)
    if len(stats) <= 1:
        return 0
    return int(np.max(stats[1:, cv2.CC_STAT_AREA]))


def _overlap_report(mask: np.ndarray, target_mask: np.ndarray, offset_xy: Tuple[int, int]) -> Dict[str, Any]:
    overlap = (mask > 0) & (target_mask > 0)
    report = {
        "overlap_px": int(overlap.sum()),
        "bbox_xyxy": _mask_bbox(overlap, offset_xy),
    }
    report.update(_component_summaries(overlap, offset_xy))
    return report


def _dilate(mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.dilate(mask.astype(np.uint8), kernel, iterations=1) > 0


def _bridge_components(
    delta_mask: np.ndarray,
    base_mask: np.ndarray,
    grown_mask: np.ndarray,
    seed_mask: np.ndarray,
    offset_xy: Tuple[int, int],
) -> List[Dict[str, Any]]:
    delta_u8 = (delta_mask > 0).astype(np.uint8)
    if int(delta_u8.sum()) == 0:
        return []
    base_labels, _ = _component_stats(base_mask)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(delta_u8, connectivity=8)
    ox, oy = offset_xy
    base_seed_connected_area = _seed_connected_area(grown_mask, seed_mask)
    components: List[Dict[str, Any]] = []
    for label_idx in range(1, num_labels):
        comp = labels == label_idx
        x = int(stats[label_idx, cv2.CC_STAT_LEFT])
        y = int(stats[label_idx, cv2.CC_STAT_TOP])
        w = int(stats[label_idx, cv2.CC_STAT_WIDTH])
        h = int(stats[label_idx, cv2.CC_STAT_HEIGHT])
        touched_base_labels = sorted(
            int(v) for v in np.unique(base_labels[_dilate(comp) & (base_labels > 0)]) if int(v) > 0
        )
        without = grown_mask & (~comp)
        without_seed_connected_area = _seed_connected_area(without, seed_mask)
        components.append(
            {
                "label": int(label_idx),
                "area_px": int(stats[label_idx, cv2.CC_STAT_AREA]),
                "bbox_xyxy": [x + ox, y + oy, x + w + ox, y + h + oy],
                "seed_overlap_px": int((comp & seed_mask).sum()),
                "birth_overlap_px": int((comp & grown_mask).sum()),
                "touching_base_component_labels": touched_base_labels,
                "touching_base_component_count": int(len(touched_base_labels)),
                "seed_connected_area_with_component_px": int(base_seed_connected_area),
                "seed_connected_area_without_component_px": int(without_seed_connected_area),
                "seed_connected_area_drop_px": int(base_seed_connected_area - without_seed_connected_area),
            }
        )
    components.sort(
        key=lambda item: (
            -int(item["seed_connected_area_drop_px"]),
            -int(item["seed_overlap_px"]),
            -int(item["area_px"]),
            item["bbox_xyxy"],
        )
    )
    return components


def _stage_summary(
    stage_mask: np.ndarray,
    birth_mask: np.ndarray,
    seed_mask: np.ndarray,
    offset_xy: Tuple[int, int],
) -> Dict[str, Any]:
    summary = {
        "roi_on_px": int((stage_mask > 0).sum()),
        "roi_bbox_xyxy": _mask_bbox(stage_mask > 0, offset_xy),
        "largest_component_px": int(_largest_component_area(stage_mask)),
        "seed_connected_area_px": int(_seed_connected_area(stage_mask, seed_mask)),
    }
    summary.update(_component_summaries(stage_mask > 0, offset_xy))
    summary["birth_overlap"] = _overlap_report(stage_mask > 0, birth_mask > 0, offset_xy)
    summary["seed_overlap"] = _overlap_report(stage_mask > 0, seed_mask > 0, offset_xy)
    return summary


def _extract_room6_masks(debug_root: Path, sequence_id: str, target_frame: int) -> Dict[str, Any]:
    base = debug_root / sequence_id / "debug_room" / "floor_1"
    tracking = json.loads((base / f"run_{target_frame}_12_tracking_report.json").read_text(encoding="utf-8"))
    label_by_gid = {
        int(row["global_id"]): int(row["marker_label"])
        for row in list(tracking["tracking"]["matched"]) + list(tracking["tracking"]["new_rooms"])
    }
    future_room_label = int(label_by_gid[6])
    final_labels = np.load(base / f"run_{target_frame}_09_final_labels.npy")
    pre_markers = np.load(base / f"run_{target_frame}_08_pre_watershed_markers.npy")
    birth_mask = final_labels == future_room_label
    seed_support = pre_markers == future_room_label
    return {
        "future_room_marker_label": int(future_room_label),
        "birth_mask": birth_mask,
        "seed_support": seed_support,
    }


def _embed_mask_in_requested_roi(
    mask: np.ndarray,
    actual_roi_bbox: Sequence[int],
    requested_roi_bbox: Sequence[int],
) -> np.ndarray:
    rx0, ry0, rx1, ry1 = [int(v) for v in requested_roi_bbox]
    ax0, ay0, ax1, ay1 = [int(v) for v in actual_roi_bbox]
    requested_h = int(ry1 - ry0)
    requested_w = int(rx1 - rx0)
    out = np.zeros((requested_h, requested_w), dtype=bool)
    if mask.size == 0:
        return out
    dx0 = max(0, ax0 - rx0)
    dy0 = max(0, ay0 - ry0)
    dx1 = min(requested_w, dx0 + mask.shape[1])
    dy1 = min(requested_h, dy0 + mask.shape[0])
    src_w = max(0, dx1 - dx0)
    src_h = max(0, dy1 - dy0)
    if src_w <= 0 or src_h <= 0:
        return out
    out[dy0:dy1, dx0:dx1] = mask[:src_h, :src_w] > 0
    return out


def _load_pending_prefix_probes(
    sequence_id: str,
    config_path: str,
    frames: Sequence[int],
    roi_bbox: Tuple[int, int, int, int],
    target_run_frame: int,
) -> Tuple[Dict[str, Any], FloorAwareRoomSegmenter]:
    cfg = _load_config("hm3d", config_path, sequence_id)
    dataset = get_dataset(cfg)
    if hasattr(dataset, "load_arkit_depth"):
        dataset.load_arkit_depth = True

    gap = int(cfg["data"]["gap"])
    room_segmenter = FloorAwareRoomSegmenter(resolution=0.05, config=cfg)
    frames_set = set(int(v) for v in frames)

    for frame_idx in range(0, int(target_run_frame) + 1):
        pose = np.asarray(dataset.poses[frame_idx], dtype=np.float32)
        is_keyframe = (frame_idx % gap == 0)
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
        if frame_idx < int(target_run_frame) and frame_idx % 100 == 0:
            room_segmenter.perform_segmentation(debug_path=None, count=frame_idx)

    floor_id = str((room_segmenter.last_floor_observation or {}).get("floor_id"))
    floor_state = room_segmenter.floor_states[floor_id]
    existing_points = floor_state.merged_points_xyzrgb
    if existing_points is None or len(existing_points) == 0:
        raise RuntimeError("Expected merged floor points from the frame-700 segmentation baseline.")

    ordered_pending = list(map(int, floor_state.pending_chunk_frame_indices))
    prefix_chunks: List[np.ndarray] = []
    frame_reports: Dict[str, Any] = {}
    for pending_frame, chunk in zip(ordered_pending, floor_state.pending_chunks):
        prefix_chunks.append(chunk)
        if int(pending_frame) not in frames_set:
            continue
        merged_prefix = np.concatenate([existing_points, *prefix_chunks], axis=0)
        merged_prefix = _downsample_points(merged_prefix, voxel_size=0.05)
        if merged_prefix is None:
            raise RuntimeError(f"Downsampled merged prefix was empty at frame {pending_frame}.")
        probe_segmenter = room_segmenter._clone_segmenter_for_debug(floor_state.segmenter)
        probe = probe_segmenter.inspect_pre_outside_boundary_debug(
            np.ascontiguousarray(merged_prefix[:, :3], dtype=np.float64),
            frame_id=int(pending_frame),
            roi_bbox=roi_bbox,
        )
        frame_reports[str(int(pending_frame))] = probe

    return frame_reports, room_segmenter


def build_report(
    sequence_id: str,
    config_path: str,
    debug_root: Path,
    frames: Sequence[int],
    roi_bbox: Tuple[int, int, int, int],
    target_run_frame: int,
) -> Dict[str, Any]:
    frame_reports, room_segmenter = _load_pending_prefix_probes(
        sequence_id=sequence_id,
        config_path=config_path,
        frames=frames,
        roi_bbox=roi_bbox,
        target_run_frame=target_run_frame,
    )
    room6 = _extract_room6_masks(debug_root, sequence_id, target_run_frame)
    x0, y0, x1, y1 = [int(v) for v in roi_bbox]
    offset_xy = (x0, y0)
    birth_roi = room6["birth_mask"][y0:y1, x0:x1]
    seed_roi = room6["seed_support"][y0:y1, x0:x1]

    per_frame: Dict[str, Any] = {}
    for frame_key in sorted(frame_reports, key=int):
        actual_roi_bbox = frame_reports[frame_key]["roi_bbox_xyxy"]
        masks = {
            name: _embed_mask_in_requested_roi(value, actual_roi_bbox, roi_bbox)
            for name, value in frame_reports[frame_key]["_roi_masks"].items()
        }
        stage_summaries = {
            stage_name: _stage_summary(stage_mask, birth_roi, seed_roi, offset_xy)
            for stage_name, stage_mask in masks.items()
        }
        per_frame[frame_key] = {
            "frame_id": int(frame_key),
            "roi_bbox_xyxy": [int(v) for v in roi_bbox],
            "stage_summaries": stage_summaries,
            "birth_overlap_progression_px": {
                stage_name: int(stage_summaries[stage_name]["birth_overlap"]["overlap_px"])
                for stage_name in ("raw_support", "blur_positive", "preclose", "postclose", "filled")
            },
            "seed_overlap_progression_px": {
                stage_name: int(stage_summaries[stage_name]["seed_overlap"]["overlap_px"])
                for stage_name in ("raw_support", "blur_positive", "preclose", "postclose", "filled")
            },
            "seed_connected_progression_px": {
                stage_name: int(stage_summaries[stage_name]["seed_connected_area_px"])
                for stage_name in ("raw_support", "blur_positive", "preclose", "postclose", "filled")
            },
        }

    frame800_masks = {
        name: _embed_mask_in_requested_roi(
            value,
            frame_reports[str(target_run_frame)]["roi_bbox_xyxy"],
            roi_bbox,
        )
        for name, value in frame_reports[str(target_run_frame)]["_roi_masks"].items()
    }
    raw_birth = frame800_masks["raw_support"] & birth_roi
    preclose_birth = frame800_masks["preclose"] & birth_roi
    postclose_birth = frame800_masks["postclose"] & birth_roi
    filled_birth = frame800_masks["filled"] & birth_roi
    blur_only_birth = preclose_birth & (~raw_birth)
    close_only_birth = postclose_birth & (~preclose_birth)
    unsupported_birth = birth_roi & (~postclose_birth)

    seed_raw = raw_birth & seed_roi
    seed_preclose = preclose_birth & seed_roi
    seed_postclose = postclose_birth & seed_roi
    blur_only_seed = blur_only_birth & seed_roi
    close_only_seed = close_only_birth & seed_roi

    raw_seed_connected_area = _seed_connected_area(raw_birth, seed_roi)
    preclose_seed_connected_area = _seed_connected_area(preclose_birth, seed_roi)
    postclose_seed_connected_area = _seed_connected_area(postclose_birth, seed_roi)

    blur_bridge_components = _bridge_components(
        delta_mask=blur_only_birth,
        base_mask=raw_birth,
        grown_mask=preclose_birth,
        seed_mask=seed_roi,
        offset_xy=offset_xy,
    )
    close_bridge_components = _bridge_components(
        delta_mask=close_only_birth,
        base_mask=preclose_birth,
        grown_mask=postclose_birth,
        seed_mask=seed_roi,
        offset_xy=offset_xy,
    )

    frame800 = {
        "birth_mask": {
            "area_px": int(birth_roi.sum()),
            "bbox_xyxy": _mask_bbox(birth_roi, offset_xy),
            **_component_summaries(birth_roi, offset_xy),
        },
        "seed_support": {
            "area_px": int(seed_roi.sum()),
            "bbox_xyxy": _mask_bbox(seed_roi, offset_xy),
            **_component_summaries(seed_roi, offset_xy),
        },
        "stage_support": {
            stage_name: per_frame[str(target_run_frame)]["stage_summaries"][stage_name]
            for stage_name in ("raw_support", "blur_positive", "preclose", "postclose", "filled")
        },
        "birth_mask_partition": {
            "raw_supported_px": int(raw_birth.sum()),
            "blur_only_px": int(blur_only_birth.sum()),
            "close_only_px": int(close_only_birth.sum()),
            "unsupported_by_postclose_px": int(unsupported_birth.sum()),
            "raw_supported_components": _component_summaries(raw_birth, offset_xy),
            "blur_only_components": _component_summaries(blur_only_birth, offset_xy),
            "close_only_components": _component_summaries(close_only_birth, offset_xy),
            "unsupported_by_postclose_components": _component_summaries(unsupported_birth, offset_xy),
        },
        "seed_support_partition": {
            "raw_supported_px": int(seed_raw.sum()),
            "blur_only_px": int(blur_only_seed.sum()),
            "close_only_px": int(close_only_seed.sum()),
            "postclose_supported_px": int(seed_postclose.sum()),
            "preclose_supported_px": int(seed_preclose.sum()),
            "raw_supported_components": _component_summaries(seed_raw, offset_xy),
            "blur_only_components": _component_summaries(blur_only_seed, offset_xy),
            "close_only_components": _component_summaries(close_only_seed, offset_xy),
        },
        "connectivity": {
            "raw_birth_component_count": int(_component_summaries(raw_birth, offset_xy)["component_count"]),
            "preclose_birth_component_count": int(_component_summaries(preclose_birth, offset_xy)["component_count"]),
            "postclose_birth_component_count": int(_component_summaries(postclose_birth, offset_xy)["component_count"]),
            "raw_seed_connected_area_px": int(raw_seed_connected_area),
            "preclose_seed_connected_area_px": int(preclose_seed_connected_area),
            "postclose_seed_connected_area_px": int(postclose_seed_connected_area),
            "raw_largest_component_px": int(_largest_component_area(raw_birth)),
            "preclose_largest_component_px": int(_largest_component_area(preclose_birth)),
            "postclose_largest_component_px": int(_largest_component_area(postclose_birth)),
        },
        "bridge_components": {
            "blur_only": blur_bridge_components,
            "close_only": close_bridge_components,
        },
        "guard_readout": {
            "seed_min_area_px": 25,
            "raw_seed_connected_area_px": int(raw_seed_connected_area),
            "preclose_seed_connected_area_px": int(preclose_seed_connected_area),
            "postclose_seed_connected_area_px": int(postclose_seed_connected_area),
            "raw_seed_participation_ratio": 0.0 if int(seed_roi.sum()) == 0 else round(float(seed_raw.sum()) / float(seed_roi.sum()), 4),
            "blur_only_seed_ratio": 0.0 if int(seed_roi.sum()) == 0 else round(float(blur_only_seed.sum()) / float(seed_roi.sum()), 4),
            "close_only_seed_ratio": 0.0 if int(seed_roi.sum()) == 0 else round(float(close_only_seed.sum()) / float(seed_roi.sum()), 4),
        },
    }

    return {
        "sequence_id": sequence_id,
        "roi_bbox_xyxy": [int(v) for v in roi_bbox],
        "frames": [int(v) for v in frames],
        "target_run_frame": int(target_run_frame),
        "active_floor_id": (room_segmenter.last_floor_observation or {}).get("floor_id"),
        "base_last_segmentation_frame_idx": int(
            room_segmenter.floor_states[str((room_segmenter.last_floor_observation or {}).get("floor_id"))].last_segmentation_frame_idx
        ),
        "frame_reports": per_frame,
        "frame_800_decomposition": frame800,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Quantify the 00843 frame-800 right-pocket raw/blur/close bridge decomposition.")
    parser.add_argument("--sequence-id", default=DEFAULT_SEQUENCE_ID)
    parser.add_argument("--config", default="config/hm3d.yaml")
    parser.add_argument("--debug-root", default=str(DEFAULT_DEBUG_ROOT))
    parser.add_argument("--frames", nargs="+", type=int, default=list(DEFAULT_FRAMES))
    parser.add_argument("--target-run-frame", type=int, default=800)
    parser.add_argument("--roi", nargs=4, type=int, default=list(DEFAULT_ROI))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    report = build_report(
        sequence_id=str(args.sequence_id),
        config_path=str(args.config),
        debug_root=Path(args.debug_root),
        frames=tuple(int(v) for v in args.frames),
        roi_bbox=tuple(int(v) for v in args.roi),
        target_run_frame=int(args.target_run_frame),
    )

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
