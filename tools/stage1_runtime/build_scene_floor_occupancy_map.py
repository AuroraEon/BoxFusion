#!/usr/bin/env python3
"""Validate an existing generalized Stage1 floor occupancy map."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from scene_runtime_common import derive_paths, load_nav_map, now_iso, write_json


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--stage-a-output-dir", type=Path)
    parser.add_argument("--map-yaml", type=Path)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args()

    paths = derive_paths(args)
    map_yaml = paths["map_yaml"]
    assert isinstance(map_yaml, Path)
    grid, resolution, origin, meta = load_nav_map(map_yaml)
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    npz_path = map_yaml.with_suffix(".npz")
    provenance_path = map_yaml.parent / f"{map_yaml.stem}_provenance.json"
    preview_path = map_yaml.parent / f"{map_yaml.stem}_preview.png"
    free = grid >= 250
    occupied = grid <= 10
    unknown = (grid > 10) & (grid < 250)
    npz_keys: list[str] = []
    if npz_path.exists():
        with np.load(npz_path) as data:
            npz_keys = sorted(data.files)
    payload = {
        "artifact_type": "scene_floor_occupancy_map_validation",
        "created_utc": now_iso(),
        "scene_id": args.scene_id,
        "floor_id": args.floor_id,
        "stage_output_dir": Path(paths["stage_output"]).as_posix(),
        "stage_a_output_dir": Path(paths["stage_a_output"]).as_posix() if paths.get("stage_a_output") else None,
        "map_yaml": map_yaml.as_posix(),
        "image": image_path.as_posix(),
        "npz": npz_path.as_posix(),
        "provenance": provenance_path.as_posix(),
        "preview": preview_path.as_posix(),
        "resolution": resolution,
        "origin": list(origin),
        "shape_hw": list(grid.shape),
        "counts": {
            "free": int(free.sum()),
            "occupied": int(occupied.sum()),
            "unknown": int(unknown.sum()),
        },
        "npz_keys": npz_keys,
        "checks": {
            "map_yaml_exists": map_yaml.exists(),
            "map_image_exists": image_path.exists(),
            "map_npz_exists": npz_path.exists(),
            "provenance_exists": provenance_path.exists(),
            "preview_exists": preview_path.exists(),
            "has_free_cells": int(free.sum()) > 0,
            "has_occupied_cells": int(occupied.sum()) > 0,
        },
    }
    payload["passed"] = all(payload["checks"].values())
    write_json(args.output_json, payload)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.write_text(
        "\n".join([
            "# Scene Floor Occupancy Map Validation",
            "",
            f"Scene: `{args.scene_id}`",
            f"Floor: `{args.floor_id}`",
            f"Passed: `{payload['passed']}`",
            f"Map: `{map_yaml}`",
            f"Shape: `{grid.shape[0]}x{grid.shape[1]}`",
            f"Free/occupied/unknown: `{payload['counts']['free']}` / `{payload['counts']['occupied']}` / `{payload['counts']['unknown']}`",
        ]) + "\n",
        encoding="utf-8",
    )
    print(payload)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
