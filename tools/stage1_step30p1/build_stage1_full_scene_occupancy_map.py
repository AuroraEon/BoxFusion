#!/usr/bin/env python3
"""Build the stable Stage1 full-scene occupancy/floorplan map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import (
    load_config,
    load_nav_map,
    now_iso,
    read_json,
    write_json,
    write_pgm,
    write_yaml_for_pgm,
)


OUT_STEM = "stage1_full_scene_occupancy_map"
ALLOWED_GATEWAYS = ["r1_r3", "r3_r7", "r3_r8", "r7_r11", "r7_r14", "r14_r16", "r7_r15", "r8_r11"]
FORBIDDEN_GATEWAYS = ["r3_r11", "r8_r14", "r14_r15", "r15_r16", "r3_r15", "r7_r16"]


def bool_layer(layered: np.lib.npyio.NpzFile, key: str) -> np.ndarray:
    if key not in layered.files:
        raise RuntimeError(f"Layered Stage1 artifact is missing key: {key}")
    return layered[key].astype(bool)


def component_metrics(mask: np.ndarray) -> dict[str, Any]:
    count, labels = cv2.connectedComponents(mask.astype(np.uint8), 8)
    sizes = [int((labels == idx).sum()) for idx in range(1, count)]
    return {
        "connected_component_count": max(0, count - 1),
        "largest_component_cells": max(sizes) if sizes else 0,
        "component_sizes_desc": sorted(sizes, reverse=True)[:12],
    }


def crop_visual(path: Path, grid: np.ndarray, room_mask: np.ndarray, room_id: int) -> str | None:
    mask = room_mask == room_id
    rows, cols = np.where(mask)
    if len(rows) == 0:
        return None
    pad = 35
    r0, r1 = max(0, int(rows.min()) - pad), min(grid.shape[0], int(rows.max()) + pad + 1)
    c0, c1 = max(0, int(cols.min()) - pad), min(grid.shape[1], int(cols.max()) + pad + 1)
    rgb = np.zeros((*grid.shape, 3), dtype=np.uint8)
    rgb[grid >= 250] = (245, 245, 245)
    rgb[(grid > 10) & (grid < 250)] = (112, 112, 112)
    rgb[grid <= 10] = (20, 20, 20)
    overlay = rgb.copy().astype(np.float32)
    overlay[mask] = 0.65 * overlay[mask] + 0.35 * np.array((65, 160, 255), dtype=np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), cv2.cvtColor(overlay[r0:r1, c0:c1].astype(np.uint8), cv2.COLOR_RGB2BGR))
    return path.as_posix()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    config = load_config(stage_output)
    maps_dir = stage_output / "maps"
    validation_dir = stage_output / "current_validation"

    h8_yaml = stage_output / config["h8r2_map_yaml"]
    h8_grid, resolution, origin, meta = load_nav_map(h8_yaml)
    h8_masks_path = stage_output / config["h8r2_masks_npz"]
    h8_masks = np.load(h8_masks_path)
    layered_path = stage_output / config["layered_bev_path"]
    layered = np.load(layered_path)
    room_mask_path = stage_output / config["room_mask_path"]
    room_mask = np.load(room_mask_path)

    stage_free = bool_layer(layered, "free_space")
    outside_boundary = bool_layer(layered, "outside_boundary")
    gateway_wall_preclose = bool_layer(layered, "gateway_wall_preclose")
    segmentation_wall = bool_layer(layered, "segmentation_wall_processed")
    structural_wall = bool_layer(layered, "structural_wall")
    wall_evidence = gateway_wall_preclose | segmentation_wall | structural_wall
    gateway_carve = h8_masks["gateway_carve_mask"].astype(bool) if "gateway_carve_mask" in h8_masks.files else np.zeros_like(stage_free)

    all_stage1_rooms = room_mask > 0
    full_scene_projectable = all_stage1_rooms & stage_free & ~wall_evidence

    grid = h8_grid.copy()
    grid[full_scene_projectable] = 254
    grid[gateway_carve] = 254
    grid[wall_evidence] = 0
    grid[gateway_carve] = 254

    out_pgm = maps_dir / f"{OUT_STEM}.pgm"
    out_yaml = maps_dir / f"{OUT_STEM}.yaml"
    out_npz = maps_dir / f"{OUT_STEM}.npz"
    write_pgm(out_pgm, grid)
    write_yaml_for_pgm(out_yaml, out_pgm.name, meta)

    free = grid >= 250
    occupied = grid <= 10
    unknown = (grid > 10) & (grid < 250)
    room_ids = [int(v) for v in sorted(np.unique(room_mask)) if int(v) > 0]
    room_metrics: dict[str, Any] = {}
    for rid in room_ids:
        mask = room_mask == rid
        area = int(mask.sum())
        free_cells = int((mask & free).sum())
        room_metrics[f"room_{rid}"] = {
            "semantic_mask_area_cells": area,
            "stable_free_cells": free_cells,
            "stable_free_fraction": round(free_cells / max(1, area), 6),
            "stable_occupied_cells": int((mask & occupied).sum()),
            "stable_unknown_cells": int((mask & unknown).sum()),
            "stage1_free_space_cells": int((mask & stage_free).sum()),
            "wall_evidence_cells": int((mask & wall_evidence).sum()),
            "gateway_carve_cells": int((mask & gateway_carve).sum()),
            **component_metrics(mask & free),
        }

    selected_pairs = set((config.get("selected_gateway_pairs") or {}).keys())
    gateway_checks = {
        "allowed_gateway_pairs_configured": sorted(selected_pairs & set(ALLOWED_GATEWAYS)),
        "allowed_gateway_pairs_missing": sorted(set(ALLOWED_GATEWAYS) - selected_pairs),
        "forbidden_gateway_pairs_configured": sorted(selected_pairs & set(FORBIDDEN_GATEWAYS)),
        "gateway_carve_cells": int(gateway_carve.sum()),
        "passed": not bool(selected_pairs & set(FORBIDDEN_GATEWAYS)) and set(ALLOWED_GATEWAYS) <= selected_pairs,
    }

    changed = grid != h8_grid
    np.savez_compressed(
        out_npz,
        full_scene_room_mask=all_stage1_rooms,
        full_scene_projectable=full_scene_projectable,
        free_mask_direct=free,
        occupied_mask=occupied,
        unknown_mask=unknown,
        gateway_carve_mask=gateway_carve,
        wall_evidence_mask=wall_evidence,
        outside_boundary=outside_boundary,
        occ_direct=np.where(free, 0, np.where(occupied, 100, -1)).astype(np.int16),
        pgm_yflip=np.flipud(grid).astype(np.uint8),
        room_ids=np.array(room_ids, dtype=np.int32),
        changed_vs_h8r2=changed,
    )

    vis_dir = validation_dir / "stable_full_scene_map_visual_diagnostics"
    room15_crop = crop_visual(vis_dir / "room15_stable_base_floorplan.png", grid, room_mask, 15)
    full_preview = vis_dir / "stage1_full_scene_occupancy_map_preview.png"
    rgb = np.zeros((*grid.shape, 3), dtype=np.uint8)
    rgb[free] = (245, 245, 245)
    rgb[unknown] = (112, 112, 112)
    rgb[occupied] = (20, 20, 20)
    full_preview.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(full_preview.as_posix(), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    visual_paths = {
        "full_scene_preview": full_preview.as_posix(),
        "room15_stable_base_floorplan": room15_crop,
    }
    provenance = {
        "artifact_type": "stable_full_scene_occupancy_map_provenance",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "map_policy": "stable_full_scene_from_stage1_global_geometry",
        "request_dependent": False,
        "through_rooms_used": False,
        "route_room_ids_whitelist_used": False,
        "step30s5_room15_patch_used": False,
        "source_inputs": {
            "h8r2_reference_map_yaml_for_metadata_and_baseline": h8_yaml.as_posix(),
            "h8r2_masks_npz_for_gateway_carves_only": h8_masks_path.as_posix(),
            "stage1_layered_bev_npz": layered_path.as_posix(),
            "stage1_room_mask": room_mask_path.as_posix(),
            "final_gateway_wall_preclose_thr_0p25_png": (stage_output / "stage1_process/room_segmentation/visualizations/final_gateway_wall_preclose_thr_0p25.png").as_posix(),
            "outside_boundary_png": (stage_output / "stage1_process/room_segmentation/visualizations/outside_boundary.png").as_posix(),
            "free_space_png": (stage_output / "stage1_process/room_segmentation/visualizations/free_space.png").as_posix(),
            "gateway_registry": (stage_output / "gateway/gateway_registry_v0_1.json").as_posix(),
            "gateway_projection_config": (stage_output / "config/step30s7_gateway_projection_config_v0_1.json").as_posix(),
        },
        "source_inputs_exist": {},
        "outputs": {
            "map_yaml": out_yaml.as_posix(),
            "map_pgm": out_pgm.as_posix(),
            "map_npz": out_npz.as_posix(),
            **visual_paths,
        },
        "map_generation": {
            "initial_grid": "h8r2 map geometry and metadata",
            "free_rule": "all Stage1 room-mask cells that are in free_space and not wall evidence become free; no non-room outside cells are introduced as free",
            "wall_rule": "structural_wall, segmentation_wall_processed, and gateway_wall_preclose remain occupied; existing H8R2 outside occupancy is preserved as the baseline",
            "gateway_rule": "H8R2 selected gateway_carve_mask is restored as free after wall occupancy, preserving selected gateway openings",
            "all_room_ids_included": room_ids,
        },
        "cell_counts": {
            "free": int(free.sum()),
            "occupied": int(occupied.sum()),
            "unknown": int(unknown.sum()),
            "changed_vs_h8r2": int(changed.sum()),
            "unknown_to_free_vs_h8r2": int((changed & (h8_grid == 128) & free).sum()),
            "wall_evidence_cells": int(wall_evidence.sum()),
            "outside_boundary_cells": int(outside_boundary.sum()),
            "stage1_free_space_cells": int(stage_free.sum()),
            "gateway_carve_cells": int(gateway_carve.sum()),
        },
        "room_metrics": room_metrics,
        "gateway_checks": gateway_checks,
        "validation_passed": bool(
            room_metrics.get("room_15", {}).get("stable_free_cells", 0) > 0
            and gateway_checks["passed"]
            and out_yaml.exists()
            and out_pgm.exists()
        ),
    }
    for key, value in provenance["source_inputs"].items():
        provenance["source_inputs_exist"][key] = Path(value).exists()

    maps_manifest_json = maps_dir / f"{OUT_STEM}_provenance_v0_1.json"
    maps_manifest_md = maps_dir / f"{OUT_STEM}_provenance_v0_1.md"
    write_json(maps_manifest_json, provenance)
    lines = [
        "# Stable Full-Scene Occupancy Map Provenance",
        "",
        f"Map: `{out_yaml}`",
        "Policy: stable full-scene Stage1 global geometry, request-independent.",
        "",
        "## Inputs",
        "",
    ]
    for key, value in provenance["source_inputs"].items():
        lines.append(f"- `{key}`: `{value}` exists `{provenance['source_inputs_exist'][key]}`")
    lines.extend([
        "",
        "## Checks",
        "",
        f"- room15 free cells: `{room_metrics.get('room_15', {}).get('stable_free_cells')}`",
        f"- through rooms used: `{provenance['through_rooms_used']}`",
        f"- route room whitelist used: `{provenance['route_room_ids_whitelist_used']}`",
        f"- Step30S5 room15 patch used: `{provenance['step30s5_room15_patch_used']}`",
        f"- validation passed: `{provenance['validation_passed']}`",
    ])
    maps_manifest_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    if args.output_json:
        write_json(args.output_json, provenance)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0 if provenance["validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
