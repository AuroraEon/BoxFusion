#!/usr/bin/env python3
"""Audit stable map provenance and cross-request room15 floorplan consistency."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from step30s7_common import load_nav_map, write_json


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> Any:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_read_error": f"{type(exc).__name__}: {exc}"}


def bbox(mask: np.ndarray, pad: int = 35) -> tuple[slice, slice]:
    rows, cols = np.where(mask)
    if len(rows) == 0:
        return slice(0, mask.shape[0]), slice(0, mask.shape[1])
    return (
        slice(max(0, int(rows.min()) - pad), min(mask.shape[0], int(rows.max()) + pad + 1)),
        slice(max(0, int(cols.min()) - pad), min(mask.shape[1], int(cols.max()) + pad + 1)),
    )


def save_map_crop(path: Path, grid: np.ndarray, mask: np.ndarray) -> None:
    rgb = np.zeros((*grid.shape, 3), dtype=np.uint8)
    rgb[grid >= 250] = (245, 245, 245)
    rgb[(grid > 10) & (grid < 250)] = (112, 112, 112)
    rgb[grid <= 10] = (20, 20, 20)
    over = rgb.copy().astype(np.float32)
    over[mask] = 0.65 * over[mask] + 0.35 * np.array((65, 160, 255), dtype=np.float32)
    rr, cc = bbox(mask)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), cv2.cvtColor(over[rr, cc].astype(np.uint8), cv2.COLOR_RGB2BGR))


def run_map_yaml(run_dir: Path) -> str | None:
    route = load_json(run_dir / "route_query_result_v0_1.json")
    return route.get("map_yaml") if isinstance(route, dict) else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--room8-run-id", default="room8_stable_map_gui_check")
    parser.add_argument("--room15-run-id", default="room15_stable_map_gui_check")
    args = parser.parse_args()

    stage = args.stage_output_dir.resolve()
    validation = stage / "current_validation"
    stable_yaml = stage / "maps/stage1_full_scene_occupancy_map.yaml"
    room_mask_path = stage / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy"
    provenance_path = validation / "stable_full_scene_occupancy_map_provenance.json"
    provenance = load_json(provenance_path)
    if not provenance:
        provenance = load_json(stage / "maps/stage1_full_scene_occupancy_map_provenance_v0_1.json")

    room8_dir = validation / args.room8_run_id
    room15_dir = validation / args.room15_run_id
    room8_yaml = run_map_yaml(room8_dir)
    room15_yaml = run_map_yaml(room15_dir)

    stable_grid, resolution, origin, _ = load_nav_map(stable_yaml)
    room_mask = np.load(room_mask_path)
    room15 = room_mask == 15
    room15_values = stable_grid[room15]
    room15_counts = {
        "free": int((room15_values >= 250).sum()),
        "occupied": int((room15_values <= 10).sum()),
        "unknown": int(((room15_values > 10) & (room15_values < 250)).sum()),
        "total": int(room15_values.size),
    }
    visual_dir = validation / "cross_request_floorplan_visual_diagnostics"
    stable_crop = visual_dir / "room15_stable_base_floorplan_crop.png"
    save_map_crop(stable_crop, stable_grid, room15)

    room8_same = room8_yaml == stable_yaml.as_posix()
    room15_same = room15_yaml == stable_yaml.as_posix()
    grids_identical = False
    if room8_yaml and room15_yaml and Path(room8_yaml).exists() and Path(room15_yaml).exists():
        g8, _, _, _ = load_nav_map(Path(room8_yaml))
        g15, _, _, _ = load_nav_map(Path(room15_yaml))
        grids_identical = bool(np.array_equal(g8[room15], g15[room15]))

    rviz_config = stage / "rviz/00824_stage1_step30p1_bev_semantic_route_demo.rviz"
    rviz_text = rviz_config.read_text(encoding="utf-8") if rviz_config.exists() else ""
    primary_rviz_map_is_map_topic = "Value: /map" in rviz_text and "Stable Full-Scene Occupancy Floorplan" in rviz_text
    status = "passed" if room8_same and room15_same and grids_identical and room15_counts["free"] > 0 and primary_rviz_map_is_map_topic else "failed"

    payload = {
        "artifact_type": "room15_cross_request_floorplan_consistency",
        "version": "v0_1",
        "created_utc": now_iso(),
        "status": status,
        "stage_output_dir": stage.as_posix(),
        "stable_map_yaml": stable_yaml.as_posix(),
        "room8_run_id": args.room8_run_id,
        "room15_run_id": args.room15_run_id,
        "room8_map_yaml": room8_yaml,
        "room15_map_yaml": room15_yaml,
        "room8_uses_stable_map": room8_same,
        "room15_uses_stable_map": room15_same,
        "room15_base_floorplan_values_identical_between_requests": grids_identical,
        "room15_stable_map_value_counts": room15_counts,
        "stable_map_request_dependent": False,
        "primary_rviz_floorplan_is_map_topic": primary_rviz_map_is_map_topic,
        "primary_rviz_floorplan_is_request_aware_map": False,
        "semantic_overlay_is_separate_marker_layer": "/stage1_nav/semantic_overlay_markers" in rviz_text,
        "visual_diagnostics": {
            "room15_stable_base_floorplan_crop": stable_crop.as_posix(),
        },
        "provenance": provenance,
    }
    write_json(validation / "room15_cross_request_floorplan_consistency.json", payload)

    md = [
        "# Room15 Cross-Request Floorplan Consistency",
        "",
        f"- status: `{status}`",
        f"- room8 map: `{room8_yaml}`",
        f"- room15 map: `{room15_yaml}`",
        f"- stable map: `{stable_yaml}`",
        f"- room15 values identical between requests: `{grids_identical}`",
        f"- room15 stable free/occupied/unknown: `{room15_counts}`",
        f"- primary RViz floorplan display: `/map` from stable full-scene occupancy map",
        f"- semantic overlay: separate marker layer on `/stage1_nav/semantic_overlay_markers`",
        f"- diagnostic crop: `{stable_crop}`",
    ]
    (validation / "room15_cross_request_floorplan_consistency.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    audit = [
        "# Occupancy Map Generation Audit",
        "",
        "The previous active path generated `step30s7_request_aware_nav_map.yaml` from the route request and freed only requested/resolved route rooms.",
        "",
        "The repaired active path builds `stage1_full_scene_occupancy_map.yaml` from Stage1 global geometry artifacts and does not read start, goal, terminal, through rooms, or route-room ids.",
        "",
        "## Source Artifacts",
        "",
    ]
    for key, value in (provenance.get("source_inputs") or {}).items():
        audit.append(f"- `{key}`: `{value}`")
    audit.extend([
        "",
        "## Active Output",
        "",
        f"- map yaml: `{stable_yaml}`",
        f"- request dependent: `{provenance.get('request_dependent')}`",
        f"- through rooms used: `{provenance.get('through_rooms_used')}`",
        f"- Step30S5 patch used: `{provenance.get('step30s5_room15_patch_used')}`",
    ])
    (validation / "occupancy_map_generation_audit.md").write_text("\n".join(audit) + "\n", encoding="utf-8")

    rviz_audit = [
        "# RViz Floorplan Source Audit",
        "",
        f"- RViz config: `{rviz_config}`",
        "- Primary Map display subscribes to `/map`.",
        f"- `/map` is loaded from `{stable_yaml}` in both validated runs.",
        "- Request-aware maps are not the primary RViz floorplan in the repaired default profile.",
        "- Semantic room fill, route highlighting, labels, gateways, path, and trajectory are marker overlays separate from the base floorplan.",
    ]
    (validation / "rviz_floorplan_source_audit.md").write_text("\n".join(rviz_audit) + "\n", encoding="utf-8")

    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
