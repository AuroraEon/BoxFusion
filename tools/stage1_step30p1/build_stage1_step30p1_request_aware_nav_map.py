#!/usr/bin/env python3
"""Build the Step30S7 request-aware H8R2 projection successor."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import (
    load_config,
    load_gateway_lookup,
    load_nav_map,
    now_iso,
    pair_key,
    read_json,
    required_room_reasons,
    room_num,
    shortest_room_route,
    write_json,
    write_pgm,
    write_yaml_for_pgm,
)


OUT_STEM = "step30s7_request_aware_nav_map"


def connected_component_metrics(room_mask: np.ndarray, free: np.ndarray) -> dict[str, Any]:
    labels_count, labels = cv2.connectedComponents((room_mask & free).astype(np.uint8), 8)
    sizes = [int((labels == idx).sum()) for idx in range(1, labels_count)]
    return {
        "connected_component_count": max(0, labels_count - 1),
        "largest_free_component_area_cells": max(sizes) if sizes else 0,
        "free_connected_component_sizes_desc": sorted(sizes, reverse=True)[:8],
    }


def gateway_points_for_route(stage_output: Path, config: dict[str, Any], route: list[str]) -> tuple[list[str], dict[str, dict[str, Any]]]:
    by_id = load_gateway_lookup(stage_output, config)
    selected_pairs = config.get("selected_gateway_pairs") or {}
    gateway_ids = []
    details = {}
    for a, b in zip(route, route[1:]):
        gid = selected_pairs[pair_key(a, b)]
        gateway_ids.append(gid)
        details[gid] = by_id[gid]
    return gateway_ids, details


def forbidden_shortcut_checks(config: dict[str, Any], selected_gateway_ids: list[str]) -> dict[str, Any]:
    selected_pairs = set((config.get("selected_gateway_pairs") or {}).keys())
    selected_ids = set(selected_gateway_ids)
    checks = {}
    for key in config.get("forbidden_gateway_pairs") or []:
        checks[key] = {
            "selected_by_config": key in selected_pairs,
            "selected_gateway_id_in_route": any(key in gid for gid in selected_ids),
            "passed": key not in selected_pairs,
        }
    return {
        "checks": checks,
        "passed": all(item["passed"] for item in checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--start-room", required=True)
    parser.add_argument("--goal-room", required=True)
    parser.add_argument("--through-rooms", nargs="*", default=[])
    parser.add_argument("--terminal-room", default=None)
    parser.add_argument("--route-json", type=Path)
    args = parser.parse_args()

    stage_output = args.stage_output_dir.resolve()
    terminal_room = args.terminal_room or args.goal_room
    config = load_config(stage_output)
    if args.route_json and args.route_json.exists():
        route_payload = read_json(args.route_json)
        route = route_payload.get("room_sequence") or route_payload.get("resolved_route")
        if not route:
            raise RuntimeError(f"{args.route_json} does not contain room_sequence/resolved_route")
    else:
        route = shortest_room_route(config, args.start_room, args.goal_room, args.through_rooms)
    reasons = required_room_reasons(args.start_room, args.goal_room, args.through_rooms, terminal_room, route)
    included_ids = sorted([room_num(room) for room in reasons], key=int)

    h8r2_yaml = stage_output / config["h8r2_map_yaml"]
    h8_grid, resolution, origin, meta = load_nav_map(h8r2_yaml)
    h8_masks = np.load(stage_output / config["h8r2_masks_npz"])
    layered = np.load(stage_output / config["layered_bev_path"])
    room_mask = np.load(stage_output / config["room_mask_path"])
    stage_free = layered["free_space"].astype(bool)
    structural_wall = layered["structural_wall"].astype(bool)
    gateway_wall = layered["gateway_wall_preclose"].astype(bool)
    segmentation_wall = layered["segmentation_wall_processed"].astype(bool)
    navigation_wall = h8_masks["navigation_wall_mask"].astype(bool) if "navigation_wall_mask" in h8_masks.files else (structural_wall | gateway_wall | segmentation_wall)
    outside = layered["outside_boundary"].astype(bool)

    route_room_mask = np.isin(room_mask, np.array(included_ids, dtype=room_mask.dtype))
    wall_evidence = navigation_wall | structural_wall | gateway_wall | segmentation_wall
    request_room_projectable = route_room_mask & stage_free & ~wall_evidence

    before = h8_grid.copy()
    after = h8_grid.copy()
    after[request_room_projectable] = 254
    after[h8_masks["gateway_carve_mask"].astype(bool)] = 254

    maps_dir = stage_output / "maps"
    out_pgm = maps_dir / f"{OUT_STEM}.pgm"
    out_yaml = maps_dir / f"{OUT_STEM}.yaml"
    out_npz = maps_dir / f"{OUT_STEM}.npz"
    write_pgm(out_pgm, after)
    write_yaml_for_pgm(out_yaml, out_pgm.name, meta)

    free_before = before >= 250
    free_after = after >= 250
    changed = before != after
    gateway_ids, gateway_details = gateway_points_for_route(stage_output, config, route)
    room_metrics = {}
    for rid in included_ids:
        m = room_mask == rid
        room_metrics[f"room_{rid}"] = {
            "semantic_mask_area_cells": int(m.sum()),
            "h8r2_free_cells": int((m & free_before).sum()),
            "h8r2_free_fraction": round(float((m & free_before).sum() / max(1, m.sum())), 6),
            "step30s7_free_cells": int((m & free_after).sum()),
            "step30s7_free_fraction": round(float((m & free_after).sum() / max(1, m.sum())), 6),
            "changed_cells": int((m & changed).sum()),
            "unknown_to_free_cells": int((m & changed & (before == 128) & free_after).sum()),
            "occupied_to_free_cells": int((m & changed & (before <= 10) & free_after).sum()),
            **connected_component_metrics(m, free_after),
            "included_because": reasons.get(f"room_{rid}", []),
        }

    route_mask = np.isin(room_mask, np.array(included_ids, dtype=room_mask.dtype))
    changed_outside_route = changed & ~route_mask
    wall_changed_to_free = changed & free_after & (structural_wall | segmentation_wall | gateway_wall)
    unknown_to_free = changed & (before == 128) & free_after
    occupied_to_free = changed & (before <= 10) & free_after
    forbidden = forbidden_shortcut_checks(config, gateway_ids)
    manifest = {
        "artifact_type": "step30s7_request_aware_nav_map_manifest",
        "version": "v0_1",
        "created_utc": now_iso(),
        "stage_output_dir": stage_output.as_posix(),
        "request_rooms": {
            "start_room": args.start_room,
            "goal_room": args.goal_room,
            "through_rooms": args.through_rooms,
            "terminal_room": terminal_room,
        },
        "resolved_route": route,
        "included_room_ids": included_ids,
        "included_room_names": [f"room_{rid}" for rid in included_ids],
        "included_room_reasons": reasons,
        "selected_gateway_ids": gateway_ids,
        "changed_cell_counts_vs_original_h8r2": {
            "total_changed_cells": int(changed.sum()),
            "changed_cells_inside_requested_route_rooms": int((changed & route_mask).sum()),
            "changed_cells_outside_requested_route_rooms": int(changed_outside_route.sum()),
            "changed_wall_or_occupied_evidence_cells_to_free": int(wall_changed_to_free.sum()),
            "changed_occupied_cells_to_free": int(occupied_to_free.sum()),
            "changed_unknown_cells_to_free": int(unknown_to_free.sum()),
        },
        "changed_cells_by_room": {room: data["changed_cells"] for room, data in room_metrics.items()},
        "free_space_coverage_by_requested_room": room_metrics,
        "gateway_to_interior_connectivity_by_requested_room": {},
        "forbidden_shortcut_checks": forbidden,
        "general_request_aware_successor": True,
        "validation_passed": None,
        "eligible_for_active_use": None,
        "not_room15_only_patched_map": True,
        "explicit_statement": "This is a general request-aware H8R2 projection successor. It includes room15 only because room_15 is present in the request/resolved route; it is not a room15-only patched map.",
        "source_inputs": {
            "h8r2_map_yaml": h8r2_yaml.as_posix(),
            "h8r2_masks_npz": (stage_output / config["h8r2_masks_npz"]).as_posix(),
            "stage1_room_mask": (stage_output / config["room_mask_path"]).as_posix(),
            "stage1_layered_bev": (stage_output / config["layered_bev_path"]).as_posix(),
            "gateway_hypotheses": (stage_output / config["gateway_hypotheses_path"]).as_posix(),
        },
        "outputs": {
            "map_yaml": out_yaml.as_posix(),
            "map_pgm": out_pgm.as_posix(),
            "map_npz": out_npz.as_posix(),
        },
    }
    np.savez_compressed(
        out_npz,
        request_route_room_mask=route_room_mask,
        request_room_projectable=request_room_projectable,
        free_mask_direct=free_after,
        occ_direct=np.where(free_after, 0, np.where(after <= 10, 100, -1)).astype(np.int16),
        pgm_yflip=np.flipud(after).astype(np.uint8),
        changed_vs_h8r2=changed,
        included_room_ids=np.array(included_ids, dtype=np.int32),
    )
    manifest_json = maps_dir / "step30s7_request_aware_nav_map_manifest_v0_1.json"
    manifest_md = maps_dir / "step30s7_request_aware_nav_map_manifest_v0_1.md"
    write_json(manifest_json, manifest)
    lines = [
        "# Step30S7 Request-Aware Nav Map Manifest",
        "",
        f"Map: `{out_yaml}`",
        f"Route: `{' -> '.join(route)}`",
        f"Included rooms: `{', '.join(manifest['included_room_names'])}`",
        f"Changed cells vs H8R2: `{manifest['changed_cell_counts_vs_original_h8r2']['total_changed_cells']}`",
        f"Changed outside requested route rooms: `{manifest['changed_cell_counts_vs_original_h8r2']['changed_cells_outside_requested_route_rooms']}`",
        "",
        "This is not a room15-only patched map; requested room inclusion is derived from the route request and topology route.",
    ]
    manifest_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
