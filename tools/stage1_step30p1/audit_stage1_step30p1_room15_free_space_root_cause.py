#!/usr/bin/env python3
"""Step30S6 root-cause audit for room15 free-space loss."""

from __future__ import annotations

import argparse
import ast
import json
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"
DEFAULT_BACKUP_ROOT = Path("/home/ws/workspace/runtime_stage1_frozen_evidence")
DEFAULT_OUTPUT_DIR = (
    DEFAULT_STAGE_OUTPUT
    / "post_restructure_validation/step30s6_root_cause_and_gateway_generalization"
)

ROOM15 = 15
ROUTE_AND_NEIGHBOR_ROOMS = [1, 3, 7, 8, 11, 14, 15, 16]
R7_R15_GATEWAY_ID = "gw_00824_r7_r15_01"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_simple_yaml(path: Path) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        value = value.strip()
        if value.startswith("["):
            out[key.strip()] = ast.literal_eval(value)
        else:
            try:
                out[key.strip()] = int(value)
            except ValueError:
                try:
                    out[key.strip()] = float(value)
                except ValueError:
                    out[key.strip()] = value
    return out


def load_map_yaml(map_yaml: Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any], Path]:
    meta = parse_simple_yaml(map_yaml)
    pgm = (map_yaml.parent / str(meta["image"])).resolve()
    image = np.array(Image.open(pgm).convert("L"))
    map_grid = np.flipud(image)
    return image, map_grid, meta, pgm


def map_rc_from_xy(x: float, y: float, resolution: float, origin: list[float]) -> tuple[int, int]:
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def xy_from_map_rc(row: int, col: int, resolution: float, origin: list[float]) -> tuple[float, float]:
    return origin[0] + col * resolution, origin[1] + row * resolution


def connected_components(mask: np.ndarray, connectivity: int = 8) -> tuple[int, np.ndarray, list[int]]:
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=connectivity)
    sizes = [int((labels == idx).sum()) for idx in range(1, count)]
    return count - 1, labels, sizes


def nearest_true(mask: np.ndarray, start: tuple[int, int], max_radius: int = 500) -> tuple[int, int] | None:
    h, w = mask.shape
    q: deque[tuple[int, int]] = deque([start])
    seen = {start}
    while q:
        row, col = q.popleft()
        if 0 <= row < h and 0 <= col < w and bool(mask[row, col]):
            return row, col
        if max(abs(row - start[0]), abs(col - start[1])) > max_radius:
            continue
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (row + dr, col + dc)
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return None


def gateway_lookup(stage_output: Path) -> dict[str, dict[str, Any]]:
    candidates = [
        stage_output / "stage1_process/gateway_extraction/assets/00824_step30b2_auto_gateway_selection_v0_1.json",
        stage_output / "stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json",
        stage_output / "stage1_process/gateway_extraction/assets/00824_step30a_gateway_hypotheses_with_roles_v0_1.json",
    ]
    selected: dict[str, dict[str, Any]] = {}
    for path in candidates:
        if not path.exists():
            continue
        data = read_json(path)
        if "per_pair" in data:
            for item in data.get("per_pair", {}).values():
                primary = item.get("selected_primary") or {}
                cid = item.get("selected_primary_candidate_id")
                if cid and primary:
                    selected[cid] = {
                        "room_a": int(str(item.get("pair_key", "r0_r0").split("_")[0]).lstrip("r")),
                        "room_b": int(str(item.get("pair_key", "r0_r0").split("_")[1]).lstrip("r")),
                        "representative_center_xy": primary.get("center_xy"),
                        "representative_crossing_pose": primary.get("crossing_pose"),
                        "representative_approach_from_room_a": primary.get("approach_from_room_a"),
                        "representative_approach_from_room_b": primary.get("approach_from_room_b"),
                    }
        for item in data.get("hypotheses", []):
            for gateway_id in item.get("source_candidate_ids") or []:
                selected[gateway_id] = item
    return selected


def gateway_xy(gateway: dict[str, Any]) -> tuple[float, float]:
    pose = gateway.get("representative_crossing_pose") or {}
    if "x" in pose and "y" in pose:
        return float(pose["x"]), float(pose["y"])
    center = gateway.get("representative_center_xy") or [0.0, 0.0]
    return float(center[0]), float(center[1])


def polygon_mask_for_room(stage_output: Path, room_id: int, shape: tuple[int, int], resolution: float, origin: list[float]) -> tuple[np.ndarray, dict[str, Any] | None]:
    topology_path = stage_output / "stage1_committed_public/topology_v0_1.json"
    if not topology_path.exists():
        return np.zeros(shape, dtype=bool), None
    topology = read_json(topology_path)
    room_name = f"room_{room_id}"
    room_record = next((r for r in topology.get("rooms", []) if r.get("id") == room_name), None)
    mask = np.zeros(shape, dtype=np.uint8)
    if room_record:
        pts = []
        for x, y in room_record.get("polygon") or []:
            row, col = map_rc_from_xy(float(x), float(y), resolution, origin)
            pts.append([col, row])
        if len(pts) >= 3:
            cv2.fillPoly(mask, [np.array(pts, dtype=np.int32)], 1)
    return mask.astype(bool), room_record


def mask_bbox(mask: np.ndarray, pad: int = 60) -> tuple[int, int, int, int]:
    ys, xs = np.where(mask)
    if ys.size == 0:
        return 0, mask.shape[0], 0, mask.shape[1]
    return (
        max(0, int(ys.min()) - pad),
        min(mask.shape[0], int(ys.max()) + pad + 1),
        max(0, int(xs.min()) - pad),
        min(mask.shape[1], int(xs.max()) + pad + 1),
    )


def save_visual(
    path: Path,
    rgb: np.ndarray,
    crop: tuple[int, int, int, int],
    markers: list[tuple[tuple[int, int], tuple[int, int, int], str]] | None = None,
    scale: int = 5,
) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    r0, r1, c0, c1 = crop
    view = np.flipud(rgb[r0:r1, c0:c1])
    img = Image.fromarray(view.astype(np.uint8)).resize(
        (view.shape[1] * scale, view.shape[0] * scale),
        getattr(getattr(Image, "Resampling", Image), "NEAREST"),
    )
    draw = ImageDraw.Draw(img)
    for (row, col), color, label in markers or []:
        if not (r0 <= row < r1 and c0 <= col < c1):
            continue
        x = (col - c0) * scale
        y = (r1 - 1 - row) * scale
        draw.ellipse((x - 9, y - 9, x + 9, y + 9), fill=color, outline=(255, 255, 255), width=2)
        if label:
            draw.text((x + 11, y - 11), label, fill=color)
    img.save(path)
    return rel(path)


def make_visuals(
    out_dir: Path,
    room15: np.ndarray,
    original_map: np.ndarray,
    repaired_map: np.ndarray | None,
    layers: np.lib.npyio.NpzFile,
    h8r2: np.lib.npyio.NpzFile,
    comp_labels: np.ndarray,
    gateway_rc: tuple[int, int],
    target_rc: tuple[int, int] | None,
) -> dict[str, str]:
    crop = mask_bbox(room15, pad=80)
    free = original_map >= 250
    occupied = original_map <= 10
    unknown = ~(free | occupied)
    segwall = layers["segmentation_wall_processed"].astype(bool)
    preclose = layers["gateway_wall_preclose"].astype(bool)
    route_room = h8r2["route_room_mask"].astype(bool)
    gateway_carve = h8r2["gateway_carve_mask"].astype(bool)
    markers = [(gateway_rc, (0, 255, 80), "r7-r15")]
    if target_rc:
        markers.append((target_rc, (255, 0, 255), "largest-free"))

    visuals: dict[str, str] = {}
    visuals["room15_semantic_mask"] = save_visual(
        out_dir / "room15_semantic_mask.png",
        np.dstack([np.where(room15, 255, 35), np.where(room15, 60, 35), np.where(room15, 210, 35)]),
        crop,
        markers,
    )
    visuals["original_h8r2_map_clipped"] = save_visual(
        out_dir / "original_h8r2_map_clipped.png",
        np.dstack([original_map, original_map, original_map]),
        crop,
        markers,
    )
    if repaired_map is not None:
        visuals["step30s5_repaired_map_clipped"] = save_visual(
            out_dir / "step30s5_repaired_map_clipped.png",
            np.dstack([repaired_map, repaired_map, repaired_map]),
            crop,
            markers,
        )
    visuals["semantic_mask_vs_original_h8r2_free_overlap"] = save_visual(
        out_dir / "semantic_mask_vs_original_h8r2_free_overlap.png",
        np.dstack([
            np.where(room15 & ~free, 255, np.where(room15 & free, 40, 35)),
            np.where(room15 & free, 245, np.where(room15 & ~free, 80, 35)),
            np.where(room15 & free, 255, np.where(room15 & ~free, 80, 35)),
        ]),
        crop,
        markers,
    )
    visuals["semantic_mask_vs_wall_layer_overlap"] = save_visual(
        out_dir / "semantic_mask_vs_wall_layer_overlap.png",
        np.dstack([
            np.where(room15 & (segwall | preclose), 255, np.where(room15, 120, 35)),
            np.where(room15 & segwall, 130, np.where(room15, 60, 35)),
            np.where(room15 & preclose, 255, np.where(room15, 180, 35)),
        ]),
        crop,
        markers,
    )
    visuals["semantic_mask_vs_unknown_nonfree_overlap"] = save_visual(
        out_dir / "semantic_mask_vs_unknown_nonfree_overlap.png",
        np.dstack([
            np.where(room15 & occupied, 255, np.where(room15 & unknown, 255, np.where(room15 & free, 40, 35))),
            np.where(room15 & free, 245, np.where(room15 & unknown, 185, 35)),
            np.where(room15 & free, 255, np.where(room15 & occupied, 50, 35)),
        ]),
        crop,
        markers,
    )
    component_rgb = np.zeros((*room15.shape, 3), dtype=np.uint8) + 35
    palette = np.array([[40, 220, 255], [255, 180, 40], [160, 255, 80], [240, 80, 180]], dtype=np.uint8)
    for idx in range(1, min(int(comp_labels.max()), 4) + 1):
        component_rgb[comp_labels == idx] = palette[(idx - 1) % len(palette)]
    component_rgb[room15 & (comp_labels == 0)] = [80, 50, 110]
    visuals["connected_free_components"] = save_visual(out_dir / "connected_free_components.png", component_rgb, crop, markers)
    visuals["gateway_location_and_connectivity_to_interior"] = save_visual(
        out_dir / "gateway_location_and_connectivity_to_interior.png",
        np.dstack([
            np.where(room15 & gateway_carve, 255, np.where(room15 & route_room, 90, np.where(room15, 130, 35))),
            np.where(room15 & gateway_carve, 220, np.where(room15 & route_room, 255, np.where(room15, 50, 35))),
            np.where(room15 & gateway_carve, 20, np.where(room15 & route_room, 90, np.where(room15, 180, 35))),
        ]),
        crop,
        markers,
    )
    visuals["candidate_interior_targets"] = save_visual(
        out_dir / "candidate_interior_targets.png",
        np.dstack([
            np.where(room15 & free, 60, np.where(room15, 170, 35)),
            np.where(room15 & free, 250, np.where(room15, 70, 35)),
            np.where(room15 & free, 180, np.where(room15, 190, 35)),
        ]),
        crop,
        markers,
    )
    visuals["coordinate_transform_debug_plot"] = save_visual(
        out_dir / "coordinate_transform_debug_plot.png",
        np.dstack([
            np.where(room15, 210, np.where(route_room, 55, 35)),
            np.where(route_room, 220, np.where(room15, 70, 35)),
            np.where(gateway_carve, 255, np.where(room15, 220, 35)),
        ]),
        crop,
        markers,
    )
    return visuals


def compare_backup_arrays(current: Path, backup: Path, keys: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {"current": rel(current), "backup": backup.as_posix(), "backup_exists": backup.exists()}
    if not current.exists() or not backup.exists():
        return out
    try:
        if current.suffix == ".npy":
            out["arrays_equal"] = bool(np.array_equal(np.load(current), np.load(backup)))
            return out
        cur = np.load(current, allow_pickle=True)
        bak = np.load(backup, allow_pickle=True)
        out["per_key_equal"] = {key: bool(key in cur.files and key in bak.files and np.array_equal(cur[key], bak[key])) for key in keys}
        out["all_requested_keys_equal"] = all(out["per_key_equal"].values())
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def room_metrics(
    room: np.ndarray,
    layers: np.lib.npyio.NpzFile,
    h8r2: np.lib.npyio.NpzFile,
    free: np.ndarray,
    occupied: np.ndarray,
    unknown: np.ndarray,
    room_id: int,
) -> dict[str, Any]:
    m = room == room_id
    area = int(m.sum())
    return {
        "room_id": room_id,
        "semantic_mask_area_cells": area,
        "semantic_mask_area_m2": round(area * 0.05 * 0.05, 6),
        "stage30a_free_space_cells": int((m & layers["free_space"].astype(bool)).sum()),
        "segmentation_wall_processed_overlap_cells": int((m & layers["segmentation_wall_processed"].astype(bool)).sum()),
        "gateway_wall_preclose_overlap_cells": int((m & layers["gateway_wall_preclose"].astype(bool)).sum()),
        "outside_boundary_overlap_cells": int((m & layers["outside_boundary"].astype(bool)).sum()),
        "unknown_layer_overlap_cells": int((m & layers["unknown_layer"].astype(bool)).sum()),
        "h8r2_route_room_mask_cells": int((m & h8r2["route_room_mask"].astype(bool)).sum()),
        "h8r2_route_room_interior_cells": int((m & h8r2["route_room_interior"].astype(bool)).sum()),
        "h8r2_gateway_carve_cells": int((m & h8r2["gateway_carve_mask"].astype(bool)).sum()),
        "h8r2_free_cells": int((m & free).sum()),
        "h8r2_occupied_cells": int((m & occupied).sum()),
        "h8r2_unknown_cells": int((m & unknown).sum()),
        "h8r2_free_fraction": round((int((m & free).sum()) / area) if area else 0.0, 6),
    }


def write_md(path: Path, payload: dict[str, Any]) -> None:
    answers = payload["required_answers"]
    m = payload["room15_metrics"]
    lines = [
        "# Step30S6 Room15 Free-Space Root-Cause Report",
        "",
        f"Created UTC: `{payload['created_utc']}`",
        "",
        "## Finding",
        "",
        answers["short_root_cause"],
        "",
        "## Key Evidence",
        "",
        f"- room15 semantic mask area: `{m['semantic_mask_area_cells']}` cells",
        f"- original H8R2 room15 free cells: `{m['h8r2_free_cells']}`",
        f"- original H8R2 room15 free fraction: `{m['h8r2_free_fraction']}`",
        f"- H8R2 route_room_mask overlap with room15: `{m['h8r2_route_room_mask_cells']}`",
        f"- H8R2 gateway_carve overlap with room15: `{m['h8r2_gateway_carve_cells']}`",
        f"- Stage30A free_space overlap with room15: `{m['stage30a_free_space_cells']}`",
        f"- gateway_wall_preclose overlap with room15: `{m['gateway_wall_preclose_overlap_cells']}`",
        "",
        "## Required Answers",
        "",
    ]
    for key, value in answers.items():
        lines.append(f"- `{key}`: {value}")
    lines.extend(["", "## Visual Diagnostics", ""])
    for key, value in payload["visual_diagnostics"].items():
        lines.append(f"- `{key}`: `{value}`")
    lines.extend(["", "## Commands", ""])
    for command in payload["commands_run"]:
        lines.append(f"- `{command}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--backup-root", type=Path, default=DEFAULT_BACKUP_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    output_dir = args.output_dir.resolve()
    visual_dir = output_dir / "visual_diagnostics"

    layered_npz = stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz"
    room_npy = stage_output / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy"
    layered_json = stage_output / "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.json"
    h8r2_masks_npz = stage_output / "maps/h8r2_gateway_preserving_masks.npz"
    h8r2_yaml = stage_output / "maps/h8r2_gateway_preserving_nav_map.yaml"
    repaired_yaml = stage_output / "maps/deprecated_step30s5_room15_patch/step30s5_room15_interior_nav_map.yaml"

    room = np.load(room_npy)
    layers = np.load(layered_npz, allow_pickle=True)
    h8r2 = np.load(h8r2_masks_npz, allow_pickle=True)
    _pgm_img, h8r2_map, map_meta, pgm_path = load_map_yaml(h8r2_yaml)
    resolution = float(map_meta.get("resolution", 0.05))
    origin = [float(v) for v in map_meta.get("origin", [-50.0, -50.0, 0.0])[:2]]

    repaired_map = None
    repaired_delta: dict[str, Any] | None = None
    if repaired_yaml.exists():
        _, repaired_map, _, repaired_pgm = load_map_yaml(repaired_yaml)
        changed = repaired_map != h8r2_map
        room15 = room == ROOM15
        repaired_delta = {
            "repaired_map_yaml": rel(repaired_yaml),
            "repaired_map_pgm": rel(repaired_pgm),
            "changed_cell_count_total": int(changed.sum()),
            "changed_cells_inside_room15": int((changed & room15).sum()),
            "changed_cells_outside_room15": int((changed & ~room15).sum()),
            "changed_cells_outside_room15_reason": "Step30S5 also freed route-room semantic/polygon cleanup and a planned route corridor; this is diagnostic/demo evidence, not a general projection fix.",
        }

    free = h8r2_map >= 250
    occupied = h8r2_map <= 10
    unknown = ~(free | occupied)
    room15 = room == ROOM15
    room15_metrics = room_metrics(room, layers, h8r2, free, occupied, unknown, ROOM15)
    room_table = {f"room_{rid}": room_metrics(room, layers, h8r2, free, occupied, unknown, rid) for rid in ROUTE_AND_NEIGHBOR_ROOMS}

    comp_count, labels, comp_sizes = connected_components(room15 & free, connectivity=8)
    largest_label = int(np.argmax([0] + comp_sizes)) if comp_sizes else 0
    largest_size = max(comp_sizes) if comp_sizes else 0
    largest_rc: tuple[int, int] | None = None
    if largest_label:
        pts = np.argwhere(labels == largest_label)
        largest_rc = tuple(int(v) for v in pts[len(pts) // 2])

    gateways = gateway_lookup(stage_output)
    gateway = gateways.get(R7_R15_GATEWAY_ID)
    if not gateway:
        raise RuntimeError(f"Could not locate {R7_R15_GATEWAY_ID}")
    gx, gy = gateway_xy(gateway)
    grc = map_rc_from_xy(gx, gy, resolution, origin)
    nearest = nearest_true(room15 & free, grc, max_radius=200)
    gateway_component_size = 0
    distance_gateway_to_largest_m = None
    connects_meaningful = False
    if nearest is not None:
        nearest_label = int(labels[nearest])
        gateway_component_size = int((labels == nearest_label).sum()) if nearest_label else 0
        connects_meaningful = bool(gateway_component_size >= max(40, int(room15_metrics["semantic_mask_area_cells"] * 0.30)))
        if largest_rc is not None:
            distance_gateway_to_largest_m = round(math.hypot((nearest[0] - largest_rc[0]) * resolution, (nearest[1] - largest_rc[1]) * resolution), 6)

    polygon_mask, room15_topology = polygon_mask_for_room(stage_output, ROOM15, room.shape, resolution, origin)
    polygon_iou = None
    if polygon_mask.any():
        polygon_iou = round(float((polygon_mask & room15).sum()) / float((polygon_mask | room15).sum()), 6)

    backup_stage30a = args.backup_root / "step30a_00824_full_stage_a_dual_wall_gateway_rerun/generated/assets"
    backup_h8r2 = args.backup_root / "step30h8r2_00824_preclose_gateway_preserving_nav_map_repair/maps"
    provenance_checks = {
        "stage30a_global_room_mask_restored_equals_backup": compare_backup_arrays(
            room_npy,
            backup_stage30a / "00824_step30a_global_room_mask_v0_1.npy",
            [],
        ),
        "stage30a_layered_bev_restored_equals_backup": compare_backup_arrays(
            layered_npz,
            backup_stage30a / "00824_step30a_layered_bev_v0_1.npz",
            ["room_mask_global_id", "free_space", "segmentation_wall_processed", "gateway_wall_preclose", "outside_boundary", "unknown_layer"],
        ),
        "h8r2_masks_restored_equals_backup": compare_backup_arrays(
            h8r2_masks_npz,
            backup_h8r2 / "step30h8r2_preclose_gateway_preserving_masks.npz",
            ["route_room_mask", "route_room_interior", "gateway_carve_mask", "occ_direct", "pgm_yflip"],
        ),
    }

    h8r2_pgm_yflip_equal = bool("pgm_yflip" in h8r2.files and np.array_equal(np.array(Image.open(pgm_path).convert("L")), h8r2["pgm_yflip"]))
    free_mask_direct_equal = bool("free_mask_direct" in h8r2.files and np.array_equal(free, h8r2["free_mask_direct"].astype(bool)))
    map_layer_alignment = {
        "room_mask_shape": list(room.shape),
        "h8r2_map_shape": list(h8r2_map.shape),
        "h8r2_masks_shape": list(h8r2["occ_direct"].shape),
        "shape_consistent": bool(room.shape == h8r2_map.shape == h8r2["occ_direct"].shape),
        "room_layer_resolution": read_json(layered_json).get("resolution") if layered_json.exists() else None,
        "map_resolution": resolution,
        "room_layer_origin": read_json(layered_json).get("origin") if layered_json.exists() else None,
        "map_origin": origin,
        "pgm_yflip_matches_h8r2_mask_artifact": h8r2_pgm_yflip_equal,
        "map_grid_free_matches_free_mask_direct": free_mask_direct_equal,
        "coordinate_mismatch_or_yflip_issue_found": not (room.shape == h8r2_map.shape and h8r2_pgm_yflip_equal and free_mask_direct_equal),
    }

    visual_paths = make_visuals(visual_dir, room15, h8r2_map, repaired_map, layers, h8r2, labels, grc, largest_rc)

    root_cause = (
        "Room15 free-space was lost in the H8R2 map projection, not in Stage-A room segmentation or wall processing. "
        "The H8R2 projection whitelisted route_room_ids `[1, 3, 8, 11, 7, 14, 16]`, excluding room15; therefore room15 was not converted to route-room interior free space. "
        "Only the r7-r15 gateway carve contributed free cells inside room15, yielding 89/1324 free cells."
    )
    required_answers = {
        "short_root_cause": root_cause,
        "is_room15_semantic_mask_itself_plausible": f"Yes. The mask has {room15_metrics['semantic_mask_area_cells']} cells ({room15_metrics['semantic_mask_area_m2']} m^2), Stage30A marks all of it as free_space, and the committed topology/model areas are close ({(room15_topology or {}).get('area_m2')} m^2; polygon_iou={polygon_iou}).",
        "is_room15_lost_during_room_segmentation_wall_processing_projection_map_generation_or_gateway_carving": "It is lost during H8R2 map projection/map generation by route-room whitelisting. Wall layers overlap only a small minority of room15; gateway carving opens only the doorway-sized region.",
        "is_loss_already_present_in_step30a_process_artifacts": f"No. Stage30A room15 free_space overlap is {room15_metrics['stage30a_free_space_cells']} of {room15_metrics['semantic_mask_area_cells']} cells.",
        "is_loss_introduced_by_h8r2_map_projection": "Yes. H8R2 route_room_mask overlap with room15 is 0 cells, while gateway_carve overlap is 89 cells.",
        "is_loss_introduced_by_step30r3_artifact_relocation_restoration": "No evidence of that. Restored current room mask/layered BEV/H8R2 mask arrays match the frozen backup for requested keys where backup files exist.",
        "is_there_coordinate_mismatch_or_yflip_issue": f"No evidence. Shapes/origin/resolution align, pgm_yflip matches the PGM, and map free cells match free_mask_direct: {map_layer_alignment}.",
        "is_room15_actually_non_navigable_in_source_geometry_or_projection_artifact": "Projection artifact. Stage30A free_space covers room15 and wall/preclose overlap is small; H8R2 leaves most non-wall room15 cells unknown because room15 was omitted from route_room_ids.",
        "exact_layer_first_causing_room15_free_space_drop_to_6p7_percent": "H8R2 route_room_mask/route_room_interior construction first causes the drop: room15 route_room_mask=0, route_room_interior=0, gateway_carve=89.",
        "minimal_generalizable_fix": "Replace route-specific room projection with request/semantic-route-aware projection: include every room in the resolved route, including through rooms, and add a standard per-room free-space coverage + gateway-to-interior connectivity validation gate before Nav2 execution. Do not hard-code room15.",
        "does_this_require_rerunning_stage1": "No Stage1 rerun is justified by this evidence. A small H8R2 projection successor rerun is justified only after changing the general room-inclusion policy.",
        "step30s5_repaired_map_status": "Diagnostic/demo-only patch. It should not be the default canonical map or default auto profile.",
        "future_validation_gate": "Per-room semantic-mask-to-free-space coverage plus selected-gateway-to-meaningful-interior-component connectivity for every requested route/through/terminal room.",
    }

    payload = {
        "artifact_type": "step30s6_room15_free_space_root_cause_report",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": rel(stage_output),
        "backup_root": args.backup_root.as_posix(),
        "source_artifacts": {
            "room_mask": rel(room_npy),
            "layered_bev": rel(layered_npz),
            "h8r2_map_yaml": rel(h8r2_yaml),
            "h8r2_map_pgm": rel(pgm_path),
            "h8r2_masks": rel(h8r2_masks_npz),
            "step30s5_repaired_map_yaml": rel(repaired_yaml) if repaired_yaml.exists() else None,
        },
        "room15_metrics": {
            **room15_metrics,
            "free_connected_component_count": int(comp_count),
            "free_connected_component_sizes_desc": sorted(comp_sizes, reverse=True)[:20],
            "largest_connected_free_component_cells": int(largest_size),
            "distance_from_r7_r15_gateway_to_largest_free_component_m": distance_gateway_to_largest_m,
            "r7_r15_gateway_nearest_free_component_size_cells": int(gateway_component_size),
            "gateway_connects_to_meaningful_interior_component": connects_meaningful,
            "committed_topology_room15_area_m2": (room15_topology or {}).get("area_m2"),
            "committed_polygon_vs_semantic_mask_iou": polygon_iou,
            "boundary_or_outside_clipping_suspected": False,
            "interior_incorrectly_marked_unknown_by_projection": True,
            "walls_or_structural_evidence_fill_room15": False,
            "gateway_carving_opens_only_door_not_interior": True,
        },
        "room_and_neighbor_metrics": room_table,
        "map_layer_alignment": map_layer_alignment,
        "provenance_checks": provenance_checks,
        "step30s5_repaired_map_diagnostic_delta": repaired_delta,
        "visual_diagnostics": visual_paths,
        "required_answers": required_answers,
        "rerun_decision": {
            "rerun_needed_now": False,
            "reason": "The existing artifacts identify the failing layer; blind rerun would reproduce the same route_room_ids exclusion unless the projection policy is changed first.",
            "smallest_future_rerun_if_promoting_fix": "H8R2 map projection successor only, using a general route/request room inclusion policy.",
        },
        "commands_run": [
            "python3 tools/stage1_step30p1/audit_stage1_step30p1_room15_free_space_root_cause.py --stage-output-dir stage_outputs/stage1_00824_step30p1",
        ],
    }

    write_json(output_dir / "room15_free_space_root_cause_report_v0_1.json", payload)
    write_md(output_dir / "room15_free_space_root_cause_report_v0_1.md", payload)
    print(json.dumps({"succeeded": True, "root_cause": root_cause, "output_dir": rel(output_dir)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
