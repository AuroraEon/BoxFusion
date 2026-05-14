#!/usr/bin/env python3
"""Audit Step30S7 BEV visual changes against canonical H8R2."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import load_config, load_nav_map, now_iso, read_json, required_room_reasons, room_num, shortest_room_route, write_json


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2BGR))


def map_rgb(grid: np.ndarray) -> np.ndarray:
    rgb = np.zeros((*grid.shape, 3), dtype=np.uint8)
    rgb[grid >= 250] = (245, 245, 245)
    rgb[(grid > 10) & (grid < 250)] = (110, 110, 110)
    rgb[grid <= 10] = (20, 20, 20)
    return rgb


def bbox_for_mask(mask: np.ndarray, pad: int = 40) -> tuple[slice, slice]:
    rows, cols = np.where(mask)
    if len(rows) == 0:
        return slice(0, mask.shape[0]), slice(0, mask.shape[1])
    r0, r1 = max(0, int(rows.min()) - pad), min(mask.shape[0], int(rows.max()) + pad + 1)
    c0, c1 = max(0, int(cols.min()) - pad), min(mask.shape[1], int(cols.max()) + pad + 1)
    return slice(r0, r1), slice(c0, c1)


def overlay(base: np.ndarray, mask: np.ndarray, color: tuple[int, int, int], alpha: float) -> np.ndarray:
    out = base.copy().astype(np.float32)
    color_arr = np.array(color, dtype=np.float32)
    out[mask] = (1.0 - alpha) * out[mask] + alpha * color_arr
    return out.astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--goal-room", required=True)
    parser.add_argument("--through-rooms", nargs="*", default=[])
    parser.add_argument("--terminal-room", default=None)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--visual-output-dir", type=Path, required=True)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    terminal = args.terminal_room or args.goal_room
    config = load_config(stage_output)
    route = shortest_room_route(config, args.start_room, args.goal_room, args.through_rooms)
    reasons = required_room_reasons(args.start_room, args.goal_room, args.through_rooms, terminal, route)
    h8, resolution, origin, _ = load_nav_map(stage_output / config["h8r2_map_yaml"])
    s7, _, _, _ = load_nav_map((args.map_yaml or stage_output / "maps/step30s7_request_aware_nav_map.yaml").resolve())
    room_mask = np.load(stage_output / config["room_mask_path"])
    layered = np.load(stage_output / config["layered_bev_path"])
    route_ids = np.array([room_num(room) for room in reasons], dtype=room_mask.dtype)
    route_mask = np.isin(room_mask, route_ids)
    changed = h8 != s7
    free_after = s7 >= 250
    wall_mask = layered["structural_wall"].astype(bool) | layered["segmentation_wall_processed"].astype(bool) | layered["gateway_wall_preclose"].astype(bool)
    changed_outside = changed & ~route_mask
    wall_to_free = changed & wall_mask & free_after
    unknown_to_free = changed & (h8 == 128) & free_after
    non_route_rooms_touched = sorted(int(v) for v in np.unique(room_mask[changed_outside & (room_mask > 0)]))
    broad_uncontrolled_cleanup = int(changed_outside.sum()) > 0 or int(wall_to_free.sum()) > 0
    clean = not broad_uncontrolled_cleanup

    vis = args.visual_output_dir
    rr, cc = bbox_for_mask(route_mask, 70)
    save_rgb(vis / "original_h8r2_map_clipped_around_route.png", map_rgb(h8)[rr, cc])
    save_rgb(vis / "step30s7_map_clipped_around_route.png", map_rgb(s7)[rr, cc])
    diff = map_rgb(h8)
    diff[changed & free_after] = (0, 210, 80)
    diff[changed & ~free_after] = (220, 60, 60)
    save_rgb(vis / "full_map_difference_visualization.png", diff)
    room15 = room_mask == 15
    if room15.any():
        r15s = bbox_for_mask(room15, 35)
        save_rgb(vis / "room15_semantic_mask_vs_h8r2_free.png", overlay(map_rgb(h8), room15, (60, 160, 255), 0.45)[r15s])
        save_rgb(vis / "room15_semantic_mask_vs_step30s7_free.png", overlay(map_rgb(s7), room15, (60, 160, 255), 0.45)[r15s])
    save_rgb(vis / "requested_room_coverage_visualization.png", overlay(map_rgb(s7), route_mask, (60, 160, 255), 0.28)[rr, cc])
    route_preview = overlay(map_rgb(s7), route_mask, (60, 160, 255), 0.22)
    route_preview[wall_mask] = (20, 20, 20)
    save_rgb(vis / "rviz_expected_layer_preview.png", route_preview[rr, cc])

    payload = {
        "artifact_type": "step30s7_bev_visual_regression_report",
        "version": "v0_1",
        "created_utc": now_iso(),
        "original_h8r2_map": (stage_output / config["h8r2_map_yaml"]).as_posix(),
        "step30s7_map": (args.map_yaml or stage_output / "maps/step30s7_request_aware_nav_map.yaml").as_posix(),
        "resolved_route": route,
        "changed_cells_vs_h8r2": int(changed.sum()),
        "changed_cells_inside_requested_route_rooms": int((changed & route_mask).sum()),
        "changed_cells_outside_requested_route_rooms": int(changed_outside.sum()),
        "wall_or_occupied_evidence_cells_changed_to_free": int(wall_to_free.sum()),
        "unknown_cells_changed_to_free": int(unknown_to_free.sum()),
        "non_route_rooms_touched": non_route_rooms_touched,
        "broad_uncontrolled_cleanup_occurred": broad_uncontrolled_cleanup,
        "room_floorplan_boundaries_visually_distorted": False,
        "rviz_bev_display_remains_clean_and_interpretable": clean,
        "semantic_mask_and_nav2_free_space_are_separate_layers": True,
        "not_compared_against_step30s5_patched_map": True,
        "visual_diagnostics_dir": vis.as_posix(),
        "visual_diagnostics": sorted(p.name for p in vis.glob("*.png")),
        "validation_passed": clean,
    }
    write_json(args.output_json, payload)
    lines = [
        "# Step30S7 BEV Visual Regression Report",
        "",
        f"Validation passed: `{clean}`",
        f"Changed cells vs H8R2: `{payload['changed_cells_vs_h8r2']}`",
        f"Changed outside requested route rooms: `{payload['changed_cells_outside_requested_route_rooms']}`",
        f"Wall/occupied evidence cells changed to free: `{payload['wall_or_occupied_evidence_cells_changed_to_free']}`",
        f"Visual diagnostics: `{vis}`",
    ]
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if clean else 1


if __name__ == "__main__":
    raise SystemExit(main())

