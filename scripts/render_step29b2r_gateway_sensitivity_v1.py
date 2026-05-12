"""
Render Step29B2R diagnostic sensitivity visualizations.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from extract_step29b2_gateway_candidates_v1 import (
    SHORT_SCENE_ID,
    VERSION,
    build_boundary_band,
    crop_slices_for_rooms,
    meters_to_cells,
    normalize_pair,
    read_json,
    rel,
    shifted_or,
)
from render_step29b2_gateway_debug_v1 import (
    add_panel,
    base_visual,
    blend_mask,
    candidate_mask,
    crop_with_candidates,
    draw_candidates,
    draw_rect,
    draw_text,
    pair_debug_image,
    room_color,
    write_png,
)
from review_step29b2r_gateway_sensitivity_v1 import (
    ASSET_DIR,
    KEY_ROOM_PAIRS,
    PUBLIC_PATH_TRANSITIONS,
    REVIEW_JSON,
    STEP29B1_INPUTS,
    STEP29B2_INPUTS,
    VIS_DIR,
)


REQUIRED_VISUALIZATIONS = [
    f"{SHORT_SCENE_ID}_step29b2r_room11_room7_sensitivity.png",
    f"{SHORT_SCENE_ID}_step29b2r_room11_room8_sensitivity.png",
    f"{SHORT_SCENE_ID}_step29b2r_room3_room7_sensitivity.png",
    f"{SHORT_SCENE_ID}_step29b2r_room3_room11_sensitivity.png",
    f"{SHORT_SCENE_ID}_step29b2r_public_path_042_sensitivity.png",
    f"{SHORT_SCENE_ID}_step29b2r_wall_layer_comparison.png",
    f"{SHORT_SCENE_ID}_step29b2r_label10_unknown_impact.png",
    f"{SHORT_SCENE_ID}_step29b2r_rejected_candidate_montage.png",
]

RECLASS_COLORS = {
    "strict_valid_existing": (25, 235, 120),
    "too_narrow_candidate": (255, 180, 35),
    "wall_blocked_candidate": (70, 165, 255),
    "unknown_or_label10_blocked_candidate": (255, 255, 40),
    "one_sided_but_gateway_like": (255, 115, 200),
    "relaxed_two_sided_candidate": (120, 255, 235),
    "manual_review_needed": (255, 210, 60),
    "likely_false_positive": (160, 90, 90),
    "hard_rejected": (235, 45, 70),
}


def load_layers() -> Dict[str, Any]:
    data = np.load(STEP29B1_INPUTS["layered_bev_npz"])
    candidates = read_json(STEP29B2_INPUTS["gateway_candidates"])["gateway_candidates"]
    review = read_json(REVIEW_JSON)
    return {
        "global_room": data["room_mask_global_id"].astype(np.int32),
        "wall": data["structural_wall"] > 0,
        "free": data["free_space"] > 0,
        "outside": data["outside_boundary"] > 0,
        "unknown": data["unknown_layer"] > 0,
        "label10": data["room_mask_local_label_repaired"].astype(np.int32) == 10,
        "full_map_reference": data["full_map_post_doors_reference"] > 0,
        "candidates": candidates,
        "review": review,
    }


def pair_key(pair: Tuple[int, int]) -> str:
    a, b = normalize_pair(*pair)
    return f"room_{a}<->room_{b}"


def candidates_for_pair(candidates: Sequence[Dict[str, Any]], pair: Tuple[int, int]) -> List[Dict[str, Any]]:
    want = normalize_pair(*pair)
    return [c for c in candidates if normalize_pair(int(c["room_a"]), int(c["room_b"])) == want]


def review_for_pair(review: Dict[str, Any], pair: Tuple[int, int]) -> Dict[str, Any]:
    want = normalize_pair(*pair)
    for entry in review["pair_reviews"]:
        if normalize_pair(entry["room_pair"]["room_a"], entry["room_pair"]["room_b"]) == want:
            return entry
    return {}


def reclass_by_gateway(pair_review: Dict[str, Any]) -> Dict[str, str]:
    return {c["gateway_id"]: c["diagnostic_reclassification"] for c in pair_review.get("candidate_reclassifications", [])}


def draw_reclassified_candidates(
    img: np.ndarray,
    candidates: Sequence[Dict[str, Any]],
    reclasses: Dict[str, str],
    crop_origin: Tuple[int, int],
) -> None:
    row0, col0 = crop_origin
    full_shape = (img.shape[0] + row0, img.shape[1] + col0)
    for candidate in candidates:
        label = reclasses.get(candidate["gateway_id"], "hard_rejected")
        color = RECLASS_COLORS.get(label, (255, 255, 255))
        mask_full = candidate_mask(full_shape, candidate)
        mask = mask_full[row0 : row0 + img.shape[0], col0 : col0 + img.shape[1]]
        blend_mask(img, mask, color, 0.88)
        r0, c0, r1, c1 = [int(v) for v in candidate["bbox_cells"]]
        draw_rect(img, r0 - row0, c0 - col0, r1 - row0, c1 - col0, (0, 0, 0), thickness=2)
        draw_rect(img, r0 - row0, c0 - col0, r1 - row0, c1 - col0, color, thickness=1)
        suffix = candidate["gateway_id"].split("_")[-1]
        draw_text(img, suffix, max(0, r0 - row0 - 12), max(0, c0 - col0), color, scale=1)


def pair_sensitivity_image(layers: Dict[str, Any], pair: Tuple[int, int]) -> np.ndarray:
    global_room = layers["global_room"]
    candidates = candidates_for_pair(layers["candidates"], pair)
    pair_review = review_for_pair(layers["review"], pair)
    rs, cs = crop_with_candidates(global_room, pair, candidates, margin=115)
    r0, c0 = int(rs.start or 0), int(cs.start or 0)
    img = base_visual(global_room[rs, cs], layers["wall"][rs, cs], layers["free"][rs, cs], layers["unknown"][rs, cs], layers["outside"][rs, cs])
    room_a, room_b = normalize_pair(*pair)
    band_015 = build_boundary_band(global_room == room_a, global_room == room_b, meters_to_cells(0.15), layers["outside"])[rs, cs]
    band_060 = build_boundary_band(global_room == room_a, global_room == room_b, meters_to_cells(0.60), layers["outside"])[rs, cs]
    blend_mask(img, band_060, (120, 90, 255), 0.34)
    blend_mask(img, band_015, (255, 255, 80), 0.55)
    blend_mask(img, shifted_or(layers["label10"], 1)[rs, cs], (255, 255, 20), 0.90)
    blend_mask(img, layers["wall"][rs, cs] & (global_room[rs, cs] > 0), (255, 0, 255), 0.85)
    draw_reclassified_candidates(img, candidates, reclass_by_gateway(pair_review), (r0, c0))
    interpretation = pair_review.get("recommended_interpretation", "unknown")
    action = pair_review.get("recommended_next_action", "unknown")
    legend = [
        ("BAND015", (255, 255, 80)),
        ("BAND060", (120, 90, 255)),
        ("LABEL10", (255, 255, 20)),
        ("WALL-ROOM", (255, 0, 255)),
        ("VALID", RECLASS_COLORS["strict_valid_existing"]),
        ("NARROW", RECLASS_COLORS["too_narrow_candidate"]),
        ("ONE-SIDED", RECLASS_COLORS["one_sided_but_gateway_like"]),
        ("LABEL/UNK", RECLASS_COLORS["unknown_or_label10_blocked_candidate"]),
        ("HARD", RECLASS_COLORS["hard_rejected"]),
    ]
    return add_panel(img, f"STEP29B2R ROOM_{room_a} <-> ROOM_{room_b} {interpretation} {action}", legend)


def public_path_image(layers: Dict[str, Any]) -> np.ndarray:
    global_room = layers["global_room"]
    rs, cs = crop_slices_for_rooms(global_room, [1, 3, 7, 8, 11], margin=150)
    r0, c0 = int(rs.start or 0), int(cs.start or 0)
    img = base_visual(global_room[rs, cs], layers["wall"][rs, cs], layers["free"][rs, cs], layers["unknown"][rs, cs], layers["outside"][rs, cs])
    readiness = layers["review"]["public_path_042_sensitivity_readiness"]
    status_by_pair = {normalize_pair(t["room_a"], t["room_b"]): t["diagnostic_status"] for t in readiness["transitions"]}
    colors = {
        "strict_ready": (25, 235, 120),
        "relaxed_candidate_exists": (80, 185, 255),
        "manual_review_needed": (255, 210, 50),
        "still_missing": (235, 45, 70),
        "contradicted_by_evidence": (190, 80, 80),
    }
    for room_a, room_b in PUBLIC_PATH_TRANSITIONS:
        norm = normalize_pair(room_a, room_b)
        status = status_by_pair.get(norm, "still_missing")
        color = colors.get(status, (255, 255, 255))
        band = build_boundary_band(global_room == norm[0], global_room == norm[1], meters_to_cells(0.40), layers["outside"])
        blend_mask(img, band[rs, cs], color, 0.62)
        rows, cols = np.where(band[rs, cs])
        if rows.size:
            draw_text(img, f"{room_a}->{room_b} {status}", int(rows.mean()), int(cols.mean()), color, scale=2)
    return add_panel(
        img,
        f"STEP29B2R PUBLIC_PATH_042 {readiness['diagnostic_route_status']}",
        [(name.upper(), color) for name, color in colors.items()],
    )


def hstack(images: Sequence[np.ndarray], pad: int = 12) -> np.ndarray:
    height = max(img.shape[0] for img in images)
    width = sum(img.shape[1] for img in images) + pad * (len(images) - 1)
    out = np.zeros((height, width, 3), dtype=np.uint8)
    out[:] = (8, 10, 14)
    x = 0
    for img in images:
        out[: img.shape[0], x : x + img.shape[1]] = img
        x += img.shape[1] + pad
    return out


def vstack(images: Sequence[np.ndarray], pad: int = 12) -> np.ndarray:
    height = sum(img.shape[0] for img in images) + pad * (len(images) - 1)
    width = max(img.shape[1] for img in images)
    out = np.zeros((height, width, 3), dtype=np.uint8)
    out[:] = (8, 10, 14)
    y = 0
    for img in images:
        out[y : y + img.shape[0], : img.shape[1]] = img
        y += img.shape[0] + pad
    return out


def resize_nearest(img: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    scale = min(max_w / img.shape[1], max_h / img.shape[0], 1.0)
    if scale >= 0.999:
        return img
    new_h = max(1, int(round(img.shape[0] * scale)))
    new_w = max(1, int(round(img.shape[1] * scale)))
    rows = np.minimum((np.arange(new_h) / scale).astype(int), img.shape[0] - 1)
    cols = np.minimum((np.arange(new_w) / scale).astype(int), img.shape[1] - 1)
    return img[rows[:, None], cols[None, :]]


def wall_layer_comparison(layers: Dict[str, Any]) -> np.ndarray:
    global_room = layers["global_room"]
    rs, cs = crop_slices_for_rooms(global_room, [1, 3, 7, 8, 11], margin=125)
    strict = layers["wall"]
    eroded1 = np.logical_and(strict, np.zeros_like(strict, dtype=bool))
    eroded1 = np.asarray(__import__("extract_step29b2_gateway_candidates_v1").erode_disk(strict, 1), dtype=bool)
    eroded2 = np.asarray(__import__("extract_step29b2_gateway_candidates_v1").erode_disk(strict, 2), dtype=bool)
    panels = []
    for title, wall in [("STRICT", strict), ("ERODE1", eroded1), ("ERODE2", eroded2), ("IGNORED", np.zeros_like(strict, dtype=bool))]:
        img = base_visual(global_room[rs, cs], wall[rs, cs], layers["free"][rs, cs], layers["unknown"][rs, cs], layers["outside"][rs, cs])
        blend_mask(img, strict[rs, cs] & ~wall[rs, cs], (255, 90, 90), 0.88)
        panels.append(add_panel(resize_nearest(img, 420, 520), title, [("WALL", (235, 238, 242)), ("REMOVED", (255, 90, 90))]))
    return hstack(panels)


def label10_unknown_image(layers: Dict[str, Any]) -> np.ndarray:
    global_room = layers["global_room"]
    rs, cs = crop_slices_for_rooms(global_room, [3, 7, 8, 11], margin=150)
    img = base_visual(global_room[rs, cs], layers["wall"][rs, cs], layers["free"][rs, cs], layers["unknown"][rs, cs], layers["outside"][rs, cs])
    blend_mask(img, shifted_or(layers["label10"], 1)[rs, cs], (255, 255, 20), 0.95)
    for pair in [(7, 11), (11, 8), (3, 7), (3, 11)]:
        a, b = normalize_pair(*pair)
        band = build_boundary_band(global_room == a, global_room == b, meters_to_cells(0.40), layers["outside"])[rs, cs]
        blend_mask(img, band, (130, 110, 255), 0.30)
    return add_panel(img, "STEP29B2R LABEL10 UNKNOWN GATEWAY IMPACT", [("LABEL10", (255, 255, 20)), ("UNKNOWN", (74, 74, 78)), ("PAIR BAND", (130, 110, 255))])


def rejected_candidate_montage(layers: Dict[str, Any]) -> np.ndarray:
    panels = [
        resize_nearest(pair_sensitivity_image(layers, pair), 640, 430)
        for pair in [(7, 11), (11, 8), (3, 7), (3, 11)]
    ]
    return add_panel(vstack([hstack(panels[:2]), hstack(panels[2:])]), "STEP29B2R REJECTED CANDIDATE MONTAGE", [])


def render_all() -> Dict[str, str]:
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    layers = load_layers()
    outputs = {
        "room11_room7": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_room11_room7_sensitivity.png",
        "room11_room8": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_room11_room8_sensitivity.png",
        "room3_room7": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_room3_room7_sensitivity.png",
        "room3_room11": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_room3_room11_sensitivity.png",
        "public_path_042": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_public_path_042_sensitivity.png",
        "wall_layer_comparison": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_wall_layer_comparison.png",
        "label10_unknown_impact": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_label10_unknown_impact.png",
        "rejected_candidate_montage": VIS_DIR / f"{SHORT_SCENE_ID}_step29b2r_rejected_candidate_montage.png",
    }
    write_png(outputs["room11_room7"], pair_sensitivity_image(layers, (11, 7)))
    write_png(outputs["room11_room8"], pair_sensitivity_image(layers, (11, 8)))
    write_png(outputs["room3_room7"], pair_sensitivity_image(layers, (3, 7)))
    write_png(outputs["room3_room11"], pair_sensitivity_image(layers, (3, 11)))
    write_png(outputs["public_path_042"], public_path_image(layers))
    write_png(outputs["wall_layer_comparison"], wall_layer_comparison(layers))
    write_png(outputs["label10_unknown_impact"], label10_unknown_image(layers))
    write_png(outputs["rejected_candidate_montage"], rejected_candidate_montage(layers))
    return {key: rel(path) for key, path in outputs.items()}


def main() -> None:
    outputs = render_all()
    print(f"Rendered {len(outputs)} Step29B2R visualizations into {rel(VIS_DIR)}")


if __name__ == "__main__":
    main()
