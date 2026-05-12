"""
Render Step29B2 gateway debug visualizations without external imaging packages.
"""

from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np

from extract_step29b2_gateway_candidates_v1 import (
    ASSET_DIR,
    GATEWAY_CANDIDATES_JSON,
    KEY_ROOM_PAIRS,
    LABEL10_WALL_AUDIT_JSON,
    ORIGIN,
    PUBLIC_PATH_TRANSITIONS,
    REPO_ROOT,
    RESOLUTION,
    SHORT_SCENE_ID,
    STEP29B1_LAYERED_JSON,
    STEP29B1_LAYERED_NPZ,
    VIS_DIR,
    build_boundary_band,
    crop_slices_for_rooms,
    json_ready,
    meters_to_cells,
    normalize_pair,
    read_json,
    rel,
    shifted_or,
)


REQUIRED_VISUALIZATIONS = [
    f"{SHORT_SCENE_ID}_step29b2_gateway_candidates_overview.png",
    f"{SHORT_SCENE_ID}_step29b2_selected_vs_rejected_gateways.png",
    f"{SHORT_SCENE_ID}_step29b2_room_pair_boundary_bands.png",
    f"{SHORT_SCENE_ID}_step29b2_room3_room11_gateway_debug.png",
    f"{SHORT_SCENE_ID}_step29b2_room11_room7_gateway_debug.png",
    f"{SHORT_SCENE_ID}_step29b2_room3_room7_gateway_debug.png",
    f"{SHORT_SCENE_ID}_step29b2_room11_room8_gateway_debug.png",
    f"{SHORT_SCENE_ID}_step29b2_public_path_042_gateway_readiness.png",
    f"{SHORT_SCENE_ID}_step29b2_label10_audit.png",
    f"{SHORT_SCENE_ID}_step29b2_wall_room_overlap_audit.png",
]

STATUS_COLORS = {
    "valid": (25, 235, 120),
    "ambiguous": (255, 210, 45),
    "misleading": (255, 120, 30),
    "rejected": (240, 45, 70),
}

FONT: Dict[str, Sequence[str]] = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01110"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "10010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01111", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "11110"],
    " ": ["000", "000", "000", "000", "000", "000", "000"],
    "_": ["00000", "00000", "00000", "00000", "00000", "00000", "11111"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    ".": ["000", "000", "000", "000", "000", "011", "011"],
    ":": ["000", "011", "011", "000", "011", "011", "000"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "<": ["00010", "00100", "01000", "10000", "01000", "00100", "00010"],
    ">": ["01000", "00100", "00010", "00001", "00010", "00100", "01000"],
    "=": ["00000", "11111", "00000", "11111", "00000", "00000", "00000"],
    "(": ["0010", "0100", "1000", "1000", "1000", "0100", "0010"],
    ")": ["0100", "0010", "0001", "0001", "0001", "0010", "0100"],
    "?": ["01110", "10001", "00001", "00010", "00100", "00000", "00100"],
    ",": ["000", "000", "000", "000", "011", "011", "010"],
}


def png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)


def write_png(path: Path, image: np.ndarray) -> None:
    img = np.asarray(image, dtype=np.uint8)
    if img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("write_png expects an RGB uint8 image")
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[row].tobytes() for row in range(h))
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(raw, 6))
        + png_chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    print(f"Written: {rel(path)}")


def draw_rect(img: np.ndarray, r0: int, c0: int, r1: int, c1: int, color: Tuple[int, int, int], thickness: int = 1) -> None:
    h, w, _ = img.shape
    r0 = max(0, min(h - 1, int(r0)))
    r1 = max(0, min(h - 1, int(r1)))
    c0 = max(0, min(w - 1, int(c0)))
    c1 = max(0, min(w - 1, int(c1)))
    if r0 > r1:
        r0, r1 = r1, r0
    if c0 > c1:
        c0, c1 = c1, c0
    for t in range(thickness):
        rr0 = max(0, r0 - t)
        rr1 = min(h - 1, r1 + t)
        cc0 = max(0, c0 - t)
        cc1 = min(w - 1, c1 + t)
        img[rr0, cc0 : cc1 + 1] = color
        img[rr1, cc0 : cc1 + 1] = color
        img[rr0 : rr1 + 1, cc0] = color
        img[rr0 : rr1 + 1, cc1] = color


def draw_line(img: np.ndarray, r0: float, c0: float, r1: float, c1: float, color: Tuple[int, int, int], thickness: int = 1) -> None:
    r0_i, c0_i, r1_i, c1_i = int(round(r0)), int(round(c0)), int(round(r1)), int(round(c1))
    dr = abs(r1_i - r0_i)
    dc = abs(c1_i - c0_i)
    sr = 1 if r0_i < r1_i else -1
    sc = 1 if c0_i < c1_i else -1
    err = dc - dr
    r, c = r0_i, c0_i
    while True:
        for tr in range(-thickness + 1, thickness):
            for tc in range(-thickness + 1, thickness):
                rr = r + tr
                cc = c + tc
                if 0 <= rr < img.shape[0] and 0 <= cc < img.shape[1]:
                    img[rr, cc] = color
        if r == r1_i and c == c1_i:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def draw_cross(img: np.ndarray, r: float, c: float, color: Tuple[int, int, int], size: int = 4) -> None:
    draw_line(img, r - size, c, r + size, c, color, 1)
    draw_line(img, r, c - size, r, c + size, color, 1)


def draw_text(img: np.ndarray, text: str, r: int, c: int, color: Tuple[int, int, int] = (255, 255, 255), scale: int = 2) -> None:
    x = int(c)
    y = int(r)
    for ch in text.upper():
        glyph = FONT.get(ch, FONT["?"])
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit == "1":
                    rr0 = y + gy * scale
                    cc0 = x + gx * scale
                    rr1 = min(img.shape[0], rr0 + scale)
                    cc1 = min(img.shape[1], cc0 + scale)
                    if rr0 < img.shape[0] and cc0 < img.shape[1] and rr1 > 0 and cc1 > 0:
                        img[max(0, rr0) : rr1, max(0, cc0) : cc1] = color
        x += (len(glyph[0]) + 1) * scale


def add_panel(image: np.ndarray, title: str, legend_items: Sequence[Tuple[str, Tuple[int, int, int]]] = ()) -> np.ndarray:
    panel_h = 96
    out = np.zeros((image.shape[0] + panel_h, image.shape[1], 3), dtype=np.uint8)
    out[:panel_h] = (8, 10, 14)
    out[panel_h:] = image
    draw_text(out, title, 12, 12, (255, 255, 255), scale=2)
    x = 12
    y = 52
    for label, color in legend_items:
        draw_rect(out, y, x, y + 15, x + 22, color, thickness=8)
        draw_text(out, label, y, x + 32, (230, 235, 240), scale=1)
        x += max(110, len(label) * 7 + 56)
        if x > out.shape[1] - 180:
            x = 12
            y += 22
    return out


def room_color(room_id: int) -> Tuple[int, int, int]:
    colors = {
        1: (65, 155, 255),
        3: (80, 210, 190),
        7: (245, 180, 55),
        8: (210, 105, 235),
        11: (255, 105, 95),
        14: (120, 205, 90),
        15: (180, 150, 250),
        16: (245, 130, 180),
    }
    return colors.get(int(room_id), ((70 + room_id * 47) % 255, (110 + room_id * 71) % 255, (150 + room_id * 31) % 255))


def blend_mask(img: np.ndarray, mask: np.ndarray, color: Tuple[int, int, int], alpha: float) -> None:
    if not np.any(mask):
        return
    base = img[mask].astype(np.float32)
    col = np.array(color, dtype=np.float32)
    img[mask] = np.clip(base * (1.0 - alpha) + col * alpha, 0, 255).astype(np.uint8)


def base_visual(global_room: np.ndarray, wall: np.ndarray, free: np.ndarray, unknown: np.ndarray, outside: np.ndarray) -> np.ndarray:
    img = np.zeros((*global_room.shape, 3), dtype=np.uint8)
    img[:] = (6, 7, 9)
    img[outside] = (24, 28, 32)
    blend_mask(img, free, (35, 120, 55), 0.72)
    img[unknown] = (74, 74, 78)
    for room_id in sorted(int(v) for v in np.unique(global_room) if int(v) > 0):
        blend_mask(img, global_room == room_id, room_color(room_id), 0.50)
    img[wall] = (235, 238, 242)
    return img


def pose_to_row_col(pose: Dict[str, Any]) -> Tuple[float, float]:
    x = float(pose["x"])
    y = float(pose["y"])
    return (y - ORIGIN[1]) / RESOLUTION, (x - ORIGIN[0]) / RESOLUTION


def candidate_mask(global_shape: Tuple[int, int], candidate: Dict[str, Any]) -> np.ndarray:
    mask = np.zeros(global_shape, dtype=bool)
    r0, c0, _r1, _c1 = [int(v) for v in candidate["debug_component_rle"]["bbox_cells"]]
    for rr, cc, length in candidate["debug_component_rle"]["runs_row_offset_col_offset_length"]:
        row = r0 + int(rr)
        col0 = c0 + int(cc)
        col1 = col0 + int(length)
        if 0 <= row < mask.shape[0]:
            mask[row, max(0, col0) : min(mask.shape[1], col1)] = True
    return mask


def candidates_for_pair(candidates: Sequence[Dict[str, Any]], pair: Tuple[int, int]) -> List[Dict[str, Any]]:
    norm = normalize_pair(*pair)
    return [c for c in candidates if normalize_pair(int(c["room_a"]), int(c["room_b"])) == norm]


def draw_candidates(img: np.ndarray, candidates: Sequence[Dict[str, Any]], crop_origin: Tuple[int, int] = (0, 0), include_components: bool = True) -> None:
    row0, col0 = crop_origin
    full_shape = (img.shape[0] + row0, img.shape[1] + col0)
    for candidate in candidates:
        color = STATUS_COLORS.get(candidate["status"], (255, 255, 255))
        if include_components:
            mask_full = candidate_mask(full_shape, candidate)
            mask = mask_full[row0 : row0 + img.shape[0], col0 : col0 + img.shape[1]]
            blend_mask(img, mask, color, 0.85)
        r0, c0, r1, c1 = [int(v) for v in candidate["bbox_cells"]]
        rr0, cc0, rr1, cc1 = r0 - row0, c0 - col0, r1 - row0, c1 - col0
        draw_rect(img, rr0, cc0, rr1, cc1, (20, 20, 20), thickness=2)
        draw_rect(img, rr0, cc0, rr1, cc1, color, thickness=1)
        center_r, center_c = pose_to_row_col(candidate["center"])
        ar, ac = pose_to_row_col(candidate["approach_from_room_a"])
        br, bc = pose_to_row_col(candidate["approach_from_room_b"])
        draw_line(img, ar - row0, ac - col0, br - row0, bc - col0, (245, 245, 245), 1)
        draw_cross(img, center_r - row0, center_c - col0, (0, 0, 0), 5)
        draw_cross(img, center_r - row0, center_c - col0, color, 4)
        label = candidate["gateway_id"].split("_")[-1]
        draw_text(img, label, max(0, rr0 - 12), max(0, cc0), color, scale=1)
        if candidate.get("selected_for_topology"):
            draw_rect(img, rr0 - 3, cc0 - 3, rr1 + 3, cc1 + 3, (0, 245, 255), thickness=2)


def crop_with_candidates(global_room: np.ndarray, pair: Tuple[int, int], candidates: Sequence[Dict[str, Any]], margin: int = 100) -> Tuple[slice, slice]:
    room_slice_r, room_slice_c = crop_slices_for_rooms(global_room, pair, margin=margin)
    r0 = int(room_slice_r.start or 0)
    c0 = int(room_slice_c.start or 0)
    r1 = int(room_slice_r.stop or global_room.shape[0]) - 1
    c1 = int(room_slice_c.stop or global_room.shape[1]) - 1
    for c in candidates:
        br0, bc0, br1, bc1 = [int(v) for v in c["bbox_cells"]]
        r0 = min(r0, max(0, br0 - margin))
        c0 = min(c0, max(0, bc0 - margin))
        r1 = max(r1, min(global_room.shape[0] - 1, br1 + margin))
        c1 = max(c1, min(global_room.shape[1] - 1, bc1 + margin))
    return slice(r0, r1 + 1), slice(c0, c1 + 1)


def pair_debug_image(
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    unknown: np.ndarray,
    outside: np.ndarray,
    label10: np.ndarray,
    wall_overlap: np.ndarray,
    pair: Tuple[int, int],
    candidates: Sequence[Dict[str, Any]],
) -> np.ndarray:
    pair_candidates = candidates_for_pair(candidates, pair)
    rs, cs = crop_with_candidates(global_room, pair, pair_candidates)
    r0, c0 = int(rs.start or 0), int(cs.start or 0)
    g = global_room[rs, cs]
    img = base_visual(g, wall[rs, cs], free[rs, cs], unknown[rs, cs], outside[rs, cs])
    room_a, room_b = normalize_pair(*pair)
    band = build_boundary_band(global_room == room_a, global_room == room_b, meters_to_cells(0.40), outside)[rs, cs]
    blend_mask(img, band, (135, 75, 255), 0.45)
    blend_mask(img, wall_overlap[rs, cs], (255, 0, 255), 0.90)
    blend_mask(img, label10[rs, cs], (255, 255, 20), 0.90)
    draw_candidates(img, pair_candidates, (r0, c0), include_components=True)
    legend = [
        ("ROOMS", (80, 180, 220)),
        ("WALL", (235, 238, 242)),
        ("FREE", (35, 120, 55)),
        ("UNKNOWN", (74, 74, 78)),
        ("BAND", (135, 75, 255)),
        ("LABEL10", (255, 255, 20)),
        ("WALL-ROOM", (255, 0, 255)),
        ("VALID", STATUS_COLORS["valid"]),
        ("AMBIG", STATUS_COLORS["ambiguous"]),
        ("MISLEAD", STATUS_COLORS["misleading"]),
        ("REJECT", STATUS_COLORS["rejected"]),
        ("SELECT", (0, 245, 255)),
    ]
    title = f"STEP29B2 ROOM_{room_a} <-> ROOM_{room_b} GATEWAY DEBUG"
    return add_panel(img, title, legend)


def overview_image(
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    unknown: np.ndarray,
    outside: np.ndarray,
    candidates: Sequence[Dict[str, Any]],
    selected_only: bool = False,
) -> np.ndarray:
    img = base_visual(global_room, wall, free, unknown, outside)
    selected = [c for c in candidates if c.get("selected_for_topology")]
    rejected = [c for c in candidates if c["status"] in {"rejected", "misleading", "ambiguous"}]
    draw_candidates(img, selected if selected_only else candidates, (0, 0), include_components=False)
    if selected_only:
        draw_candidates(img, rejected, (0, 0), include_components=False)
    legend = [
        ("VALID", STATUS_COLORS["valid"]),
        ("AMBIG", STATUS_COLORS["ambiguous"]),
        ("MISLEAD", STATUS_COLORS["misleading"]),
        ("REJECT", STATUS_COLORS["rejected"]),
        ("SELECT", (0, 245, 255)),
    ]
    title = "STEP29B2 SELECTED VS REJECTED GATEWAYS" if selected_only else "STEP29B2 GATEWAY CANDIDATES OVERVIEW"
    return add_panel(img, title, legend)


def boundary_bands_image(global_room: np.ndarray, wall: np.ndarray, free: np.ndarray, unknown: np.ndarray, outside: np.ndarray) -> np.ndarray:
    img = base_visual(global_room, wall, free, unknown, outside)
    colors = [(255, 80, 80), (80, 255, 160), (255, 210, 40), (180, 110, 255), (70, 170, 255)]
    pairs = sorted({normalize_pair(*p) for p in KEY_ROOM_PAIRS + PUBLIC_PATH_TRANSITIONS})
    for idx, pair in enumerate(pairs):
        band = build_boundary_band(global_room == pair[0], global_room == pair[1], meters_to_cells(0.40), outside)
        blend_mask(img, band, colors[idx % len(colors)], 0.72)
        rows, cols = np.where(band)
        if rows.size:
            draw_text(img, f"{pair[0]}-{pair[1]}", int(np.mean(rows)), int(np.mean(cols)), colors[idx % len(colors)], scale=2)
    legend = [(f"R{a}-R{b}", colors[i % len(colors)]) for i, (a, b) in enumerate(pairs)]
    return add_panel(img, "STEP29B2 ROOM PAIR BOUNDARY BANDS R=0.40M", legend)


def public_path_image(
    global_room: np.ndarray,
    wall: np.ndarray,
    free: np.ndarray,
    unknown: np.ndarray,
    outside: np.ndarray,
    candidates: Sequence[Dict[str, Any]],
) -> np.ndarray:
    img = base_visual(global_room, wall, free, unknown, outside)
    colors = [(80, 220, 255), (255, 220, 60), (110, 255, 130), (255, 130, 220)]
    selected_by_pair = {
        normalize_pair(int(c["room_a"]), int(c["room_b"])): c for c in candidates if c.get("selected_for_topology")
    }
    for idx, pair in enumerate(PUBLIC_PATH_TRANSITIONS):
        norm = normalize_pair(*pair)
        color = colors[idx % len(colors)]
        band = build_boundary_band(global_room == norm[0], global_room == norm[1], meters_to_cells(0.40), outside)
        blend_mask(img, band, color, 0.55)
        selected = selected_by_pair.get(norm)
        rows, cols = np.where(band)
        text = f"{pair[0]}->{pair[1]} {'READY' if selected else 'MISSING'}"
        if selected:
            draw_candidates(img, [selected], (0, 0), include_components=True)
        if rows.size:
            draw_text(img, text, int(np.mean(rows)), int(np.mean(cols)), color, scale=2)
    legend = [(f"{a}->{b}", colors[i % len(colors)]) for i, (a, b) in enumerate(PUBLIC_PATH_TRANSITIONS)]
    return add_panel(img, "STEP29B2 PUBLIC_PATH_042 GATEWAY READINESS", legend)


def label10_image(global_room: np.ndarray, wall: np.ndarray, free: np.ndarray, unknown: np.ndarray, outside: np.ndarray, label10: np.ndarray) -> np.ndarray:
    rs, cs = crop_slices_for_rooms(global_room, [3, 7, 8, 11], margin=130)
    if np.any(label10):
        r10, c10 = np.where(label10)
        r0 = max(0, min(int(rs.start or 0), int(r10.min()) - 90))
        c0 = max(0, min(int(cs.start or 0), int(c10.min()) - 90))
        r1 = min(global_room.shape[0], max(int(rs.stop or global_room.shape[0]), int(r10.max()) + 91))
        c1 = min(global_room.shape[1], max(int(cs.stop or global_room.shape[1]), int(c10.max()) + 91))
        rs, cs = slice(r0, r1), slice(c0, c1)
    img = base_visual(global_room[rs, cs], wall[rs, cs], free[rs, cs], unknown[rs, cs], outside[rs, cs])
    blend_mask(img, shifted_or(label10, 1)[rs, cs], (255, 255, 20), 0.95)
    legend = [("LABEL10", (255, 255, 20)), ("WALL", (235, 238, 242)), ("FREE", (35, 120, 55)), ("UNKNOWN", (74, 74, 78))]
    return add_panel(img, "STEP29B2 UNMAPPED LOCAL LABEL 10 AUDIT", legend)


def wall_overlap_image(global_room: np.ndarray, wall: np.ndarray, free: np.ndarray, unknown: np.ndarray, outside: np.ndarray) -> np.ndarray:
    wall_overlap = wall & (global_room > 0)
    rs, cs = crop_slices_for_rooms(global_room, [3, 7, 8, 11], margin=130)
    img = base_visual(global_room[rs, cs], wall[rs, cs], free[rs, cs], unknown[rs, cs], outside[rs, cs])
    blend_mask(img, shifted_or(wall_overlap, 1)[rs, cs], (255, 0, 255), 0.95)
    legend = [("WALL-ROOM", (255, 0, 255)), ("WALL", (235, 238, 242)), ("FREE", (35, 120, 55)), ("UNKNOWN", (74, 74, 78))]
    return add_panel(img, "STEP29B2 WALL ROOM OVERLAP AUDIT", legend)


def render_all() -> Dict[str, str]:
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    data = np.load(STEP29B1_LAYERED_NPZ)
    candidates_payload = read_json(GATEWAY_CANDIDATES_JSON)
    _audit = read_json(LABEL10_WALL_AUDIT_JSON)
    candidates = candidates_payload["gateway_candidates"]
    global_room = data["room_mask_global_id"].astype(np.int32)
    wall = data["structural_wall"] > 0
    free = data["free_space"] > 0
    outside = data["outside_boundary"] > 0
    unknown = data["unknown_layer"] > 0
    label10 = data["room_mask_local_label_repaired"].astype(np.int32) == 10
    wall_overlap = wall & (global_room > 0)

    outputs: Dict[str, Path] = {
        "gateway_candidates_overview": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_gateway_candidates_overview.png",
        "selected_vs_rejected_gateways": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_selected_vs_rejected_gateways.png",
        "room_pair_boundary_bands": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_room_pair_boundary_bands.png",
        "room3_room11_gateway_debug": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_room3_room11_gateway_debug.png",
        "room11_room7_gateway_debug": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_room11_room7_gateway_debug.png",
        "room3_room7_gateway_debug": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_room3_room7_gateway_debug.png",
        "room11_room8_gateway_debug": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_room11_room8_gateway_debug.png",
        "public_path_042_gateway_readiness": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_public_path_042_gateway_readiness.png",
        "label10_audit": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_label10_audit.png",
        "wall_room_overlap_audit": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2_wall_room_overlap_audit.png",
    }
    write_png(outputs["gateway_candidates_overview"], overview_image(global_room, wall, free, unknown, outside, candidates))
    write_png(outputs["selected_vs_rejected_gateways"], overview_image(global_room, wall, free, unknown, outside, candidates, selected_only=True))
    write_png(outputs["room_pair_boundary_bands"], boundary_bands_image(global_room, wall, free, unknown, outside))
    write_png(outputs["room3_room11_gateway_debug"], pair_debug_image(global_room, wall, free, unknown, outside, label10, wall_overlap, (3, 11), candidates))
    write_png(outputs["room11_room7_gateway_debug"], pair_debug_image(global_room, wall, free, unknown, outside, label10, wall_overlap, (11, 7), candidates))
    write_png(outputs["room3_room7_gateway_debug"], pair_debug_image(global_room, wall, free, unknown, outside, label10, wall_overlap, (3, 7), candidates))
    write_png(outputs["room11_room8_gateway_debug"], pair_debug_image(global_room, wall, free, unknown, outside, label10, wall_overlap, (11, 8), candidates))
    write_png(outputs["public_path_042_gateway_readiness"], public_path_image(global_room, wall, free, unknown, outside, candidates))
    write_png(outputs["label10_audit"], label10_image(global_room, wall, free, unknown, outside, label10))
    write_png(outputs["wall_room_overlap_audit"], wall_overlap_image(global_room, wall, free, unknown, outside))
    return {key: rel(path) for key, path in outputs.items()}


def main() -> None:
    outputs = render_all()
    print(f"Rendered {len(outputs)} Step29B2 visualizations")


if __name__ == "__main__":
    main()
