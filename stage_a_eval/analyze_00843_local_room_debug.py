#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np


def _load_gray(base: Path, frame_idx: int, suffix: str) -> np.ndarray:
    path = base / f"run_{frame_idx}_{suffix}"
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image


def _load_npy(base: Path, frame_idx: int, suffix: str) -> np.ndarray:
    path = base / f"run_{frame_idx}_{suffix}"
    if not path.exists():
        raise FileNotFoundError(path)
    return np.load(path)


def _load_json(base: Path, frame_idx: int, suffix: str) -> Dict:
    path = base / f"run_{frame_idx}_{suffix}"
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def _pad_to_shape(array: np.ndarray, shape: Tuple[int, int], fill: int = 0) -> np.ndarray:
    out = np.full(shape, fill, dtype=array.dtype)
    out[: array.shape[0], : array.shape[1]] = array
    return out


def _expanded_bbox(mask: np.ndarray, margin: int) -> Tuple[int, int, int, int]:
    ys, xs = np.where(mask)
    if len(xs) == 0 or len(ys) == 0:
        raise ValueError("mask is empty")
    x0 = max(0, int(xs.min()) - margin)
    y0 = max(0, int(ys.min()) - margin)
    x1 = min(mask.shape[1] - 1, int(xs.max()) + margin)
    y1 = min(mask.shape[0] - 1, int(ys.max()) + margin)
    return x0, y0, x1, y1


def _roi(array: np.ndarray, bbox: Tuple[int, int, int, int]) -> np.ndarray:
    x0, y0, x1, y1 = bbox
    return array[y0 : y1 + 1, x0 : x1 + 1]


def _binary_delta(before: np.ndarray, after: np.ndarray) -> Dict[str, int]:
    return {
        "0_to_255": int(((before == 0) & (after == 255)).sum()),
        "255_to_0": int(((before == 255) & (after == 0)).sum()),
        "changed": int((before != after).sum()),
        "before_on": int((before > 0).sum()),
        "after_on": int((after > 0).sum()),
    }


def _label_transition_counts(before: np.ndarray, after: np.ndarray) -> Dict[str, int]:
    transitions: Dict[str, int] = {}
    for src, dst in zip(before.ravel(), after.ravel()):
        if src == dst:
            continue
        key = f"{int(src)}->{int(dst)}"
        transitions[key] = transitions.get(key, 0) + 1
    return dict(sorted(transitions.items(), key=lambda item: (-item[1], item[0])))


def _reverse_tracking_labels(tracking_payload: Dict) -> Dict[int, int]:
    lookup: Dict[int, int] = {}
    tracking = dict(tracking_payload.get("tracking") or {})
    for row in list(tracking.get("matched") or []) + list(tracking.get("new_rooms") or []):
        marker_label = row.get("marker_label")
        global_id = row.get("global_id")
        if marker_label is None or global_id is None:
            continue
        lookup[int(global_id)] = int(marker_label)
    return lookup


def _door_delta_nonzero(base: Path, frame_idx: int, bbox: Tuple[int, int, int, int]) -> Dict[str, int]:
    image = _load_gray(base, frame_idx, "04c_door_carve_delta.png")
    region = _roi(image, bbox)
    return {
        "roi_nonzero": int((region > 0).sum()),
        "global_nonzero": int((image > 0).sum()),
    }


def build_report(debug_base: Path) -> Dict:
    tracking_700 = _load_json(debug_base, 700, "12_tracking_report.json")
    tracking_800 = _load_json(debug_base, 800, "12_tracking_report.json")
    tracking_900 = _load_json(debug_base, 900, "12_tracking_report.json")
    tracking_1000 = _load_json(debug_base, 1000, "12_tracking_report.json")

    label_by_gid_800 = _reverse_tracking_labels(tracking_800)
    label_by_gid_900 = _reverse_tracking_labels(tracking_900)

    future_room6_label = label_by_gid_800[6]
    room2_label = label_by_gid_900[2]
    room5_label = label_by_gid_900[5]

    final_700 = _load_npy(debug_base, 700, "09_final_labels.npy")
    final_800 = _load_npy(debug_base, 800, "09_final_labels.npy")
    final_900 = _load_npy(debug_base, 900, "09_final_labels.npy")
    final_1000 = _load_npy(debug_base, 1000, "09_final_labels.npy")
    pre_700 = _load_npy(debug_base, 700, "08_pre_watershed_markers.npy")
    pre_800 = _load_npy(debug_base, 800, "08_pre_watershed_markers.npy")
    pre_900 = _load_npy(debug_base, 900, "08_pre_watershed_markers.npy")
    pre_1000 = _load_npy(debug_base, 1000, "08_pre_watershed_markers.npy")

    birth_mask = final_800 == future_room6_label
    right_bbox = _expanded_bbox(birth_mask, margin=20)

    room2_mask_900 = final_900 == room2_label
    room5_mask_900 = final_900 == room5_label
    kernel = np.ones((3, 3), dtype=np.uint8)
    contact = (
        cv2.dilate(room2_mask_900.astype(np.uint8), kernel, iterations=1) > 0
    ) & (
        cv2.dilate(room5_mask_900.astype(np.uint8), kernel, iterations=1) > 0
    )
    left_bbox = _expanded_bbox(contact, margin=25)

    label_shape_700_800 = (
        max(final_700.shape[0], final_800.shape[0]),
        max(final_700.shape[1], final_800.shape[1]),
    )
    label_shape_900_1000 = (
        max(final_900.shape[0], final_1000.shape[0]),
        max(final_900.shape[1], final_1000.shape[1]),
    )
    stage_shape_700_800 = (
        max(_load_gray(debug_base, 700, "03_outside_boundary.png").shape[0], _load_gray(debug_base, 800, "03_outside_boundary.png").shape[0]),
        max(_load_gray(debug_base, 700, "03_outside_boundary.png").shape[1], _load_gray(debug_base, 800, "03_outside_boundary.png").shape[1]),
    )
    stage_shape_900_1000 = (
        max(_load_gray(debug_base, 900, "03_outside_boundary.png").shape[0], _load_gray(debug_base, 1000, "03_outside_boundary.png").shape[0]),
        max(_load_gray(debug_base, 900, "03_outside_boundary.png").shape[1], _load_gray(debug_base, 1000, "03_outside_boundary.png").shape[1]),
    )

    stage_suffixes = {
        "walls_skeleton": "02_walls_skeleton.png",
        "outside_boundary": "03_outside_boundary.png",
        "full_map": "04b_full_map_post_doors.png",
        "free_space": "05_free_space.png",
        "seed_mask": "07_seed_mask.png",
    }

    right_stage_deltas = {}
    for stage_name, suffix in stage_suffixes.items():
        before = _pad_to_shape(_load_gray(debug_base, 700, suffix), stage_shape_700_800)
        after = _pad_to_shape(_load_gray(debug_base, 800, suffix), stage_shape_700_800)
        right_stage_deltas[stage_name] = _binary_delta(_roi(before, right_bbox), _roi(after, right_bbox))

    right_birth_mask_deltas = {}
    birth_mask_padded = _pad_to_shape(birth_mask.astype(np.uint8), stage_shape_700_800).astype(bool)
    for stage_name, suffix in stage_suffixes.items():
        before = _pad_to_shape(_load_gray(debug_base, 700, suffix), stage_shape_700_800)
        after = _pad_to_shape(_load_gray(debug_base, 800, suffix), stage_shape_700_800)
        right_birth_mask_deltas[stage_name] = {
            "before_on_in_birth_mask": int(((before > 0) & birth_mask_padded).sum()),
            "after_on_in_birth_mask": int(((after > 0) & birth_mask_padded).sum()),
            "0_to_255_in_birth_mask": int(((before == 0) & (after == 255) & birth_mask_padded).sum()),
            "255_to_0_in_birth_mask": int(((before == 255) & (after == 0) & birth_mask_padded).sum()),
        }

    right_pre_700 = _pad_to_shape(pre_700, label_shape_700_800)
    right_pre_800 = _pad_to_shape(pre_800, label_shape_700_800)
    right_final_700 = _pad_to_shape(final_700, label_shape_700_800)
    right_final_800 = _pad_to_shape(final_800, label_shape_700_800)

    left_pairs = {}
    for before_frame, after_frame in ((800, 900), (900, 1000)):
        pair_key = f"{before_frame}_to_{after_frame}"
        left_pairs[pair_key] = {
            "stage_deltas": {},
        }
        for stage_name, suffix in stage_suffixes.items():
            before = _pad_to_shape(_load_gray(debug_base, before_frame, suffix), stage_shape_900_1000)
            after = _pad_to_shape(_load_gray(debug_base, after_frame, suffix), stage_shape_900_1000)
            left_pairs[pair_key]["stage_deltas"][stage_name] = _binary_delta(_roi(before, left_bbox), _roi(after, left_bbox))

        before_pre = _roi(_pad_to_shape(_load_npy(debug_base, before_frame, "08_pre_watershed_markers.npy"), label_shape_900_1000), left_bbox)
        after_pre = _roi(_pad_to_shape(_load_npy(debug_base, after_frame, "08_pre_watershed_markers.npy"), label_shape_900_1000), left_bbox)
        before_final = _roi(_pad_to_shape(_load_npy(debug_base, before_frame, "09_final_labels.npy"), label_shape_900_1000), left_bbox)
        after_final = _roi(_pad_to_shape(_load_npy(debug_base, after_frame, "09_final_labels.npy"), label_shape_900_1000), left_bbox)
        left_pairs[pair_key]["pre_marker_transitions"] = _label_transition_counts(before_pre, after_pre)
        left_pairs[pair_key]["final_label_transitions"] = _label_transition_counts(before_final, after_final)

    left_motion_900_1000 = {}
    left_900 = _roi(_pad_to_shape(final_900, label_shape_900_1000), left_bbox)
    left_1000 = _roi(_pad_to_shape(final_1000, label_shape_900_1000), left_bbox)
    for room_name, label in (("room_2", room2_label), ("room_5", room5_label), ("room_6", label_by_gid_900[6])):
        before = left_900 == label
        after = left_1000 == label
        left_motion_900_1000[room_name] = {
            "label": int(label),
            "lost_pixels": int((before & ~after).sum()),
            "gained_pixels": int((~before & after).sum()),
            "net_pixels": int(int(after.sum()) - int(before.sum())),
        }

    return {
        "scene_id": "00843-DYehNKdT76V",
        "floor_id": "floor_1",
        "tracked_labels": {
            "frame_800": {str(k): int(v) for k, v in sorted(label_by_gid_800.items())},
            "frame_900": {str(k): int(v) for k, v in sorted(label_by_gid_900.items())},
            "future_room_6_label_at_800": int(future_room6_label),
            "room_2_label_at_900": int(room2_label),
            "room_5_label_at_900": int(room5_label),
        },
        "right_pocket_birth": {
            "frames": [700, 800],
            "bbox_xyxy": list(map(int, right_bbox)),
            "birth_mask_area_px": int(birth_mask.sum()),
            "birth_mask_pixels_within_frame_700_width": int(birth_mask[:, : final_700.shape[1]].sum()),
            "birth_mask_pixels_beyond_frame_700_width": int(birth_mask[:, final_700.shape[1] :].sum()),
            "tracking_summary_700": {
                "valid_seed_count": int(tracking_700["valid_seed_count"]),
                "room_count": int(tracking_700["room_count"]),
            },
            "tracking_summary_800": {
                "valid_seed_count": int(tracking_800["valid_seed_count"]),
                "room_count": int(tracking_800["room_count"]),
            },
            "stage_deltas": right_stage_deltas,
            "birth_mask_stage_deltas": right_birth_mask_deltas,
            "pre_markers_in_bbox": {
                "frame_700": {str(int(v)): int((_roi(right_pre_700, right_bbox) == v).sum()) for v in np.unique(_roi(right_pre_700, right_bbox))},
                "frame_800": {str(int(v)): int((_roi(right_pre_800, right_bbox) == v).sum()) for v in np.unique(_roi(right_pre_800, right_bbox))},
            },
            "final_labels_in_bbox": {
                "frame_700": {str(int(v)): int((_roi(right_final_700, right_bbox) == v).sum()) for v in np.unique(_roi(right_final_700, right_bbox))},
                "frame_800": {str(int(v)): int((_roi(right_final_800, right_bbox) == v).sum()) for v in np.unique(_roi(right_final_800, right_bbox))},
            },
            "door_delta": _door_delta_nonzero(debug_base, 800, right_bbox),
        },
        "left_contact_strip": {
            "bbox_xyxy": list(map(int, left_bbox)),
            "pairs": left_pairs,
            "motion_900_to_1000": left_motion_900_1000,
            "door_delta_900": _door_delta_nonzero(debug_base, 900, left_bbox),
            "door_delta_1000": _door_delta_nonzero(debug_base, 1000, left_bbox),
        },
        "diagnosis": {
            "right_pocket": (
                "Frame 800 room_6 birth is created upstream before watershed: outside_boundary advances into the right pocket, "
                "full_map flips the entire 554 px future-room mask from occupied to free, and a new 59 px seed appears, "
                "which is above the fixed 25 px minimum-area threshold at 0.05 m resolution."
            ),
            "left_strip": (
                "The later room_2/room_5 strip jitter is not driven by outside_boundary or tier2 repair. "
                "It is caused by local walls_skeleton motion that perturbs full_map in the strip, collapses room_2 seed support "
                "before watershed, and only secondarily lets watershed reassign a few nearby pixels to room_5."
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze local 00843 room-segmentation upstream mechanisms from saved debug artifacts.")
    parser.add_argument(
        "--debug-base",
        default="world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/debug_room/floor_1",
        help="Directory containing per-run debug artifacts for 00843 floor_1.",
    )
    parser.add_argument(
        "--output",
        default="world_model_backend_outputs_v0_2_final/scenes/00843-DYehNKdT76V/logs/room_segmentation_local_upstream_diag_00843.json",
        help="Where to write the compact JSON report.",
    )
    args = parser.parse_args()

    debug_base = Path(args.debug_base)
    report = build_report(debug_base)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
