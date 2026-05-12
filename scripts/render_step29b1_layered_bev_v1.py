"""
Render Step29B1 layered BEV visualizations.

The visualizations emphasize global room IDs, not local watershed labels.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, Tuple

import cv2
import numpy as np

from build_step29b1_layered_bev_from_step29a3_v1 import (
    EXPECTED_KEY_GLOBAL_ROOM_IDS,
    KEY_ROOM_PAIRS,
    LAYERED_JSON,
    LAYERED_NPZ,
    SHORT_SCENE_ID,
    VERSION,
    VIS_DIR,
    read_json,
    rel,
)


def room_color(room_id: int) -> Tuple[int, int, int]:
    if room_id <= 0:
        return (0, 0, 0)
    b = (70 + 83 * room_id) % 230 + 20
    g = (45 + 47 * room_id) % 220 + 25
    r = (110 + 61 * room_id) % 225 + 20
    return int(b), int(g), int(r)


def colorize_labels(labels: np.ndarray) -> np.ndarray:
    vis = np.zeros((*labels.shape, 3), dtype=np.uint8)
    for label in sorted(int(v) for v in np.unique(labels) if int(v) > 0):
        vis[labels == label] = room_color(label)
    return vis


def put_label(image: np.ndarray, text: str, xy: Tuple[int, int], scale: float = 0.7) -> None:
    x, y = xy
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)


def annotate_room_ids(image: np.ndarray, labels: np.ndarray, prefix: str = "room_") -> np.ndarray:
    annotated = image.copy()
    for room_id in sorted(int(v) for v in np.unique(labels) if int(v) > 0):
        mask = labels == room_id
        if not np.any(mask):
            continue
        ys, xs = np.where(mask)
        cx = int(np.mean(xs))
        cy = int(np.mean(ys))
        put_label(annotated, f"{prefix}{room_id}", (max(cx - 45, 3), max(cy, 18)))
    return annotated


def add_title(image: np.ndarray, title: str) -> np.ndarray:
    out = image.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 34), (0, 0, 0), thickness=-1)
    cv2.putText(out, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def write_png(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"Failed to write {path}")
    print(f"Written: {rel(path)}")


def wall_visual(wall: np.ndarray) -> np.ndarray:
    vis = np.zeros((*wall.shape, 3), dtype=np.uint8)
    vis[wall > 0] = (255, 255, 255)
    return vis


def free_unknown_visual(free: np.ndarray, unknown: np.ndarray, wall: np.ndarray) -> np.ndarray:
    vis = np.zeros((*free.shape, 3), dtype=np.uint8)
    vis[unknown > 0] = (70, 70, 70)
    vis[free > 0] = (70, 180, 70)
    vis[wall > 0] = (245, 245, 245)
    return vis


def overlay_rooms_walls(global_mask: np.ndarray, wall: np.ndarray) -> np.ndarray:
    base = colorize_labels(global_mask)
    base = (base.astype(np.float32) * 0.72).astype(np.uint8)
    base[wall > 0] = (255, 255, 255)
    return annotate_room_ids(base, global_mask, prefix="room_")


def crop_to_rooms(mask: np.ndarray, room_ids: Iterable[int], margin: int = 60) -> Tuple[slice, slice]:
    combined = np.zeros(mask.shape, dtype=bool)
    for room_id in room_ids:
        combined |= mask == int(room_id)
    if not np.any(combined):
        return slice(0, mask.shape[0]), slice(0, mask.shape[1])
    ys, xs = np.where(combined)
    y0 = max(int(ys.min()) - margin, 0)
    y1 = min(int(ys.max()) + margin + 1, mask.shape[0])
    x0 = max(int(xs.min()) - margin, 0)
    x1 = min(int(xs.max()) + margin + 1, mask.shape[1])
    return slice(y0, y1), slice(x0, x1)


def pair_debug_visual(global_mask: np.ndarray, wall: np.ndarray, pair: Tuple[int, int]) -> np.ndarray:
    row_slice, col_slice = crop_to_rooms(global_mask, pair)
    mask_crop = global_mask[row_slice, col_slice]
    wall_crop = wall[row_slice, col_slice]
    vis = np.zeros((*mask_crop.shape, 3), dtype=np.uint8)
    vis[wall_crop > 0] = (230, 230, 230)
    colors = [(40, 190, 245), (245, 90, 70)]
    for idx, room_id in enumerate(pair):
        vis[mask_crop == room_id] = colors[idx]
    vis = annotate_room_ids(vis, mask_crop, prefix="room_")
    return add_title(vis, f"Global room IDs: room_{pair[0]} vs room_{pair[1]}")


def render_all() -> Dict[str, str]:
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    metadata = read_json(LAYERED_JSON)
    data = np.load(LAYERED_NPZ)

    local_raw = data["room_mask_local_label_raw"]
    local_repaired = data["room_mask_local_label_repaired"]
    global_mask = data["room_mask_global_id"]
    wall = data["structural_wall"]
    free = data["free_space"]
    unknown = data["unknown_layer"]

    outputs: Dict[str, Path] = {}

    global_vis = annotate_room_ids(colorize_labels(global_mask), global_mask, prefix="room_")
    outputs["global_room_mask"] = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_global_room_mask.png"
    write_png(outputs["global_room_mask"], add_title(global_vis, "room_mask_global_id"))

    local_vis = annotate_room_ids(colorize_labels(local_repaired), local_repaired, prefix="local_")
    global_side = annotate_room_ids(colorize_labels(global_mask), global_mask, prefix="room_")
    side_by_side = np.concatenate(
        [
            add_title(local_vis, "Local marker labels from repaired raster"),
            add_title(global_side, "Remapped persistent global room IDs"),
        ],
        axis=1,
    )
    outputs["local_vs_global_room_mask"] = (
        VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_local_vs_global_room_mask.png"
    )
    write_png(outputs["local_vs_global_room_mask"], side_by_side)

    outputs["structural_wall_layer"] = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_structural_wall_layer.png"
    write_png(outputs["structural_wall_layer"], add_title(wall_visual(wall), "structural_wall from final_walls_skeleton.png"))

    outputs["free_unknown_layer"] = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_free_unknown_layer.png"
    write_png(outputs["free_unknown_layer"], add_title(free_unknown_visual(free, unknown, wall), "free_space, unknown exterior, structural_wall"))

    outputs["room_wall_overlay_global_ids"] = (
        VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_room_wall_overlay_global_ids.png"
    )
    write_png(outputs["room_wall_overlay_global_ids"], add_title(overlay_rooms_walls(global_mask, wall), "Global room IDs with structural walls"))

    key_grid_cells = []
    for room_id in EXPECTED_KEY_GLOBAL_ROOM_IDS:
        row_slice, col_slice = crop_to_rooms(global_mask, [room_id])
        mask_crop = global_mask[row_slice, col_slice]
        wall_crop = wall[row_slice, col_slice]
        cell = np.zeros((*mask_crop.shape, 3), dtype=np.uint8)
        cell[wall_crop > 0] = (210, 210, 210)
        cell[mask_crop == room_id] = (45, 210, 120)
        key_grid_cells.append(add_title(annotate_room_ids(cell, mask_crop), f"room_{room_id}"))
    max_h = max(cell.shape[0] for cell in key_grid_cells)
    max_w = max(cell.shape[1] for cell in key_grid_cells)
    padded_cells = []
    for cell in key_grid_cells:
        padded = np.zeros((max_h, max_w, 3), dtype=np.uint8)
        padded[: cell.shape[0], : cell.shape[1]] = cell
        padded_cells.append(padded)
    top = np.concatenate(padded_cells[:3], axis=1)
    bottom = np.concatenate(
        padded_cells[3:] + [np.zeros_like(padded_cells[0]) for _ in range(3 - len(padded_cells[3:]))],
        axis=1,
    )
    outputs["key_rooms_debug"] = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_key_rooms_debug.png"
    write_png(outputs["key_rooms_debug"], np.concatenate([top, bottom], axis=0))

    for room_a, room_b in KEY_ROOM_PAIRS:
        key = f"room{room_a}_room{room_b}_debug"
        outputs[key] = VIS_DIR / f"{SHORT_SCENE_ID}_step29b1_room{room_a}_room{room_b}_debug.png"
        write_png(outputs[key], pair_debug_visual(global_mask, wall, (room_a, room_b)))

    print(f"Rendered {len(outputs)} Step29B1 visualizations from {rel(LAYERED_NPZ)}")
    print(f"Source metadata: {rel(LAYERED_JSON)}")
    print(f"Global IDs present: {metadata.get('global_room_ids_present')}")
    return {key: rel(path) for key, path in outputs.items()}


def main() -> None:
    render_all()


if __name__ == "__main__":
    main()
