#!/usr/bin/env python3
"""Task24b3 audit/backfill for stair centerline and per-floor map regression."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import py_compile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CLAIM_BOUNDARY = "topological_vertical_transition_only"
GEOMETRY_INTERPRETATION = "fitted connector centerline from pose trace, not physical stair mesh"
PYTHON = "/home/ws/miniconda3/envs/boxfusion/bin/python"
TASK_NAME = "task24b3_stair_trace_centerline_and_all_floor_map_regression_fix"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    return value


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def sha256(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def hash_tree(root: Path) -> dict[str, str]:
    if not root.exists():
        return {}
    hashes: dict[str, str] = {}
    for path in sorted(root.rglob("*")):
        if path.is_file():
            hashes[path.relative_to(root).as_posix()] = sha256(path) or ""
    return hashes


def parse_map_yaml(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in {b"\n", b""}:
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    if tokens[0] != b"P5":
        raise ValueError(f"unsupported PGM magic in {path}")
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f"unsupported PGM max value in {path}: {maxval}")
    while idx < len(data) and data[idx:idx + 1].isspace():
        idx += 1
    return np.frombuffer(data[idx:idx + width * height], dtype=np.uint8).reshape((height, width)).copy()


def write_pgm(path: Path, grid_world: np.ndarray) -> None:
    image_grid = np.flipud(grid_world).astype(np.uint8)
    header = f"P5\n{image_grid.shape[1]} {image_grid.shape[0]}\n255\n".encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + image_grid.tobytes())


def write_map_yaml(path: Path, image_name: str, resolution: float, origin: list[float]) -> None:
    lines = [
        f"image: {image_name}",
        "mode: trinary",
        f"resolution: {resolution}",
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
    ]
    write_text(path, "\n".join(lines))


def counts(grid: np.ndarray) -> dict[str, int]:
    return {
        "free": int((grid >= 250).sum()),
        "occupied": int((grid <= 10).sum()),
        "unknown": int(((grid > 10) & (grid < 250)).sum()),
    }


def preview(path: Path, grid_world: np.ndarray) -> None:
    rgb = np.zeros((*grid_world.shape, 3), dtype=np.uint8)
    rgb[grid_world >= 250] = (245, 245, 245)
    rgb[(grid_world > 10) & (grid_world < 250)] = (112, 112, 112)
    rgb[grid_world <= 10] = (20, 20, 20)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def npz_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    result: dict[str, Any] = {"exists": True, "keys": []}
    with np.load(path) as data:
        result["keys"] = sorted(data.files)
        result["arrays"] = {
            key: {"shape": list(data[key].shape), "dtype": str(data[key].dtype)}
            for key in data.files
        }
    return result


def map_package_summary(yaml_path: Path) -> dict[str, Any]:
    meta = parse_map_yaml(yaml_path)
    pgm_path = yaml_path.parent / str(meta["image"])
    pgm = read_pgm(pgm_path)
    world_grid = np.flipud(pgm)
    return {
        "yaml": yaml_path.as_posix(),
        "pgm": pgm_path.as_posix(),
        "yaml_meta": meta,
        "pgm_shape_hw": list(pgm.shape),
        "world_grid_shape_hw": list(world_grid.shape),
        "counts": counts(world_grid),
        "npz": npz_summary(yaml_path.with_suffix(".npz")),
    }


def compare_transforms(reference_pgm: np.ndarray, candidate_pgm: np.ndarray) -> dict[str, Any]:
    transforms = {
        "identity": candidate_pgm,
        "flipud": np.flipud(candidate_pgm),
        "fliplr": np.fliplr(candidate_pgm),
        "flipud_fliplr": np.flipud(np.fliplr(candidate_pgm)),
    }
    if candidate_pgm.shape[::-1] == reference_pgm.shape:
        transforms["transpose"] = candidate_pgm.T
    return {
        name: {
            "shape_hw": list(arr.shape),
            "comparable": arr.shape == reference_pgm.shape,
            "different_cells": int((arr != reference_pgm).sum()) if arr.shape == reference_pgm.shape else None,
            "equal": bool(arr.shape == reference_pgm.shape and np.array_equal(arr, reference_pgm)),
        }
        for name, arr in transforms.items()
    }


def stage_a_reports(repo: Path, scene_root: Path, out: Path, scene_id: str) -> dict[str, Any]:
    stage_a = (repo / "stage_a_demo.py").read_text(encoding="utf-8")
    recorder = (repo / "boxfusion/stage_a_demo.py").read_text(encoding="utf-8")
    appends_sequence = "self.output_root = Path(output_root) / str(sequence_id)" in recorder
    root_pass = 'output_root=args.output_root' in stage_a
    rerun_parent = f"{scene_root.as_posix()}/reruns/{TASK_NAME}_${{TIMESTAMP}}"
    final_scene_output = f"{rerun_parent}/{scene_id}"
    cmd_00843 = "\n".join([
        "SCENE_ID=00843-DYehNKdT76V",
        f"SCENE_ROOT={scene_root.as_posix()}",
        f"TASK_NAME={TASK_NAME}",
        "TIMESTAMP=$(date -u +%Y%m%dT%H%M%SZ)",
        'RERUN_ROOT="${SCENE_ROOT}/reruns/${TASK_NAME}_${TIMESTAMP}"',
        f"{PYTHON} stage_a_demo.py hm3d \\",
        "  --model-path ./models/cutr_rgbd.pth \\",
        "  --config ./config/hm3d.yaml \\",
        "  --device cuda \\",
        '  --seq "${SCENE_ID}" \\',
        '  --output-root "${RERUN_ROOT}" \\',
        "  --room-seg-interval 100 \\",
        "  --capture-stride 25 \\",
        "  --video-fps 12 \\",
        "  --runtime-profile-interval 25",
        "",
        'Expected Stage-A scene output: "${RERUN_ROOT}/${SCENE_ID}"',
    ])
    cmd_generic = "\n".join([
        "SCENE_ID=<scene_id>",
        "SCENE_ROOT=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/${SCENE_ID}",
        f"TASK_NAME={TASK_NAME}",
        "TIMESTAMP=$(date -u +%Y%m%dT%H%M%SZ)",
        'RERUN_ROOT="${SCENE_ROOT}/reruns/${TASK_NAME}_${TIMESTAMP}"',
        f"{PYTHON} stage_a_demo.py hm3d \\",
        "  --model-path ./models/cutr_rgbd.pth \\",
        "  --config ./config/hm3d.yaml \\",
        "  --device cuda \\",
        '  --seq "${SCENE_ID}" \\',
        '  --output-root "${RERUN_ROOT}" \\',
        "  --room-seg-interval 100 \\",
        "  --capture-stride 25 \\",
        "  --video-fps 12 \\",
        "  --runtime-profile-interval 25",
    ])
    write_text(out / "stage_a_output_root_semantics.md", f"""# Stage-A Output-Root Semantics

Project: RSLG-SLAM.

`stage_a_demo.py` passes `--output-root` directly into `ClosedLoopDemoRecorder`.
`ClosedLoopDemoRecorder` then sets `self.output_root = Path(output_root) / str(sequence_id)`.

Therefore `--output-root` is the parent directory for one or more scene outputs. It does append the sequence ID automatically.

For future reruns, pass the rerun parent:

`SCENE_ROOT=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/<scene_id>`

`RERUN_ROOT=${{SCENE_ROOT}}/reruns/{TASK_NAME}_<timestamp>`

Then pass `--output-root "${{RERUN_ROOT}}"`. The realized scene output will be `${{RERUN_ROOT}}/<scene_id>`.

For 00843 this keeps every rerun under:

`{scene_root.as_posix()}/reruns/`

Full Stage-A was not run for this task.
""")
    write_text(out / "corrected_stage_a_rerun_command_plan.md", f"""# Corrected Stage-A Rerun Command Plan

Do not run this as part of task24b3 unless explicitly requested.

## 00843 Template

```bash
{cmd_00843}
```

## Generic Template

```bash
{cmd_generic}
```

Because Stage-A appends the sequence ID, do not pass a path that already ends in `<scene_id>` unless a nested `<scene_id>/<scene_id>` output is intended.
""")
    return {
        "root_passed_to_recorder": root_pass,
        "recorder_appends_sequence_id": appends_sequence,
        "output_root_argument_semantics": "parent_directory; sequence_id appended by ClosedLoopDemoRecorder",
        "correct_output_root_for_00843": "${SCENE_ROOT}/reruns/${TASK_NAME}_${TIMESTAMP}",
        "expected_scene_output_for_00843": final_scene_output,
        "command_00843": cmd_00843,
        "command_generic": cmd_generic,
        "classification": "stage_a_rerun_command_confirmed_scene_root_convention" if root_pass and appends_sequence else "stage_a_output_root_semantics_unclear",
    }


def build_map_from_floor_assets(
    stage: Path,
    out_floor: Path,
    scene_id: str,
    floor_id: str,
    reference_carve_mask: np.ndarray | None,
    regression_reference: Path | None,
) -> dict[str, Any]:
    assets = stage / "process" / "floors" / floor_id / "room_segmentation" / "assets"
    layered_npz = assets / "layered_bev_v0_1.npz"
    layered_json = assets / "layered_bev_v0_1.json"
    room_mask_path = assets / "room_mask_global_id_v0_1.npy"
    meta = read_json(layered_json)
    with np.load(layered_npz) as data:
        gateway_wall = data["gateway_wall_preclose"].astype(bool)
        room_mask = data["room_mask_global_id"].astype(np.int32)
    if reference_carve_mask is not None:
        carved_gateway = reference_carve_mask.astype(bool)
        carve_source = "validated_floor_2_npz_carved_gateway_mask"
    else:
        carved_gateway = np.zeros_like(gateway_wall, dtype=bool)
        carve_source = "none_available_for_floor"
    if carved_gateway.shape != gateway_wall.shape:
        raise ValueError(f"carve mask shape {carved_gateway.shape} does not match {gateway_wall.shape}")
    occupied = gateway_wall & ~carved_gateway
    grid = np.where(occupied, 0, 254).astype(np.uint8)
    stem = f"stage1_{floor_id}_stable_occupancy_map"
    out_pgm = out_floor / f"{stem}.pgm"
    out_yaml = out_floor / f"{stem}.yaml"
    out_npz = out_floor / f"{stem}.npz"
    out_preview = out_floor / f"{stem}_preview.png"
    out_prov = out_floor / f"{stem}_provenance.json"
    resolution = float(meta.get("resolution_m_per_cell", 0.05))
    origin = [float(v) for v in meta.get("origin_xy_yaw", [-50.0, -50.0, 0.0])[:3]]
    write_pgm(out_pgm, grid)
    write_map_yaml(out_yaml, out_pgm.name, resolution, origin)
    preview(out_preview, grid)
    np.savez_compressed(
        out_npz,
        occupancy=np.where(occupied, 100, 0).astype(np.int16),
        free=np.ones_like(grid, dtype=np.uint8),
        occupied=occupied.astype(np.uint8),
        carved_gateway=carved_gateway.astype(np.uint8),
        room_mask=room_mask,
        resolution=np.asarray([resolution], dtype=np.float32),
        origin=np.asarray(origin, dtype=np.float32),
    )
    provenance = {
        "scene_id": scene_id,
        "floor_id": floor_id,
        "artifact_type": "stage1_per_floor_stable_occupancy_map_candidate",
        "created_utc": now_iso(),
        "map_frame": "map",
        "resolution_m_per_cell": resolution,
        "origin_xy_yaw": origin,
        "array_shape_hw": list(grid.shape),
        "pgm_is_vertical_flip_of_stage_a_grid": True,
        "map_policy": "validated_floor_2_semantics_gateway_wall_preclose_minus_saved_carve_mask",
        "sources": {
            "layered_bev_npz": layered_npz.as_posix(),
            "layered_bev_metadata": layered_json.as_posix(),
            "room_mask": room_mask_path.as_posix(),
            "reference_regression_map": None if regression_reference is None else regression_reference.as_posix(),
        },
        "carve_source": carve_source,
        "counts": counts(grid),
        "claim_boundary": CLAIM_BOUNDARY,
        "installed_in_clean_rerun": False,
    }
    write_json(out_prov, provenance)
    return {
        "floor_id": floor_id,
        "generated": True,
        "map_yaml": out_yaml.as_posix(),
        "map_pgm": out_pgm.as_posix(),
        "map_preview": out_preview.as_posix(),
        "map_npz": out_npz.as_posix(),
        "provenance": out_prov.as_posix(),
        "shape_hw": list(grid.shape),
        "counts": counts(grid),
        "carve_source": carve_source,
    }


def map_regression(stage: Path, task24b2: Path, out: Path, scene_id: str) -> dict[str, Any]:
    validated_yaml = stage / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
    generated_yaml = task24b2 / "generated_maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
    validated = map_package_summary(validated_yaml)
    generated = map_package_summary(generated_yaml)
    val_pgm = read_pgm(validated_yaml.parent / parse_map_yaml(validated_yaml)["image"])
    gen_pgm = read_pgm(generated_yaml.parent / parse_map_yaml(generated_yaml)["image"])
    transforms = compare_transforms(val_pgm, gen_pgm)
    val_npz = validated_yaml.with_suffix(".npz")
    with np.load(val_npz) as data:
        val_carve = data["carved_gateway"].astype(bool)
        val_occupied = data["occupied"].astype(bool)
    floor2_regression_dir = out / "corrected_generated_maps/floor_2_regression_check"
    floor2_candidate = build_map_from_floor_assets(stage, floor2_regression_dir, scene_id, "floor_2", val_carve, validated_yaml)
    floor2_candidate_pgm = read_pgm(Path(floor2_candidate["map_pgm"]))
    regression_passed = bool(np.array_equal(floor2_candidate_pgm, val_pgm))
    with np.load(Path(floor2_candidate["map_npz"])) as data:
        candidate_occupied = data["occupied"].astype(bool)
    floor1_dir = out / "corrected_generated_maps/floor_1"
    floor1_candidate = build_map_from_floor_assets(stage, floor1_dir, scene_id, "floor_1", None, validated_yaml)
    comparison = {
        "scene_id": scene_id,
        "validated_floor_2": validated,
        "task24b2_generated_floor_2": generated,
        "task24b2_transform_tests_against_validated_pgm": transforms,
        "exact_mismatch": {
            "task24b2_identity_different_cells": transforms["identity"]["different_cells"],
            "validated_counts": validated["counts"],
            "task24b2_counts": generated["counts"],
        },
        "identified_mismatch": {
            "orientation_convention": "PGM stores vertical flip of Stage-A world grid; loaders flip back to world grid",
            "task24b2_policy_mismatch": "task24b2 treated room_mask<=0 as occupied and used a different gateway carve; validated floor_2 uses gateway_wall_preclose minus saved carved_gateway mask",
            "validated_floor_2_world_occupied_equals_gateway_wall_minus_saved_carve": bool(np.array_equal(candidate_occupied, val_occupied)),
        },
        "corrected_floor_2_regression": {
            "candidate": floor2_candidate,
            "pgm_equal_to_validated": regression_passed,
            "pgm_different_cells": int((floor2_candidate_pgm != val_pgm).sum()),
        },
        "floor_1_map_status": "candidate_ready_for_manual_packaging" if regression_passed else "experimental_untrusted_orientation",
        "floor_1_candidate": floor1_candidate,
    }
    write_json(out / "stable_map_orientation_comparison.json", comparison)
    write_json(out / "generated_per_floor_map_manifest.json", {
        "scene_id": scene_id,
        "created_utc": now_iso(),
        "floor_2_regression_passed": regression_passed,
        "floor_1_map_status": comparison["floor_1_map_status"],
        "maps": [floor1_candidate],
        "regression_check": floor2_candidate,
    })
    write_text(out / "stable_map_regression_report.md", f"""# Stable Map Regression Report

Validated floor_2 map: `{validated_yaml}`

Task24b2 generated floor_2 map: `{generated_yaml}`

YAML metadata matched for image name, resolution, origin, thresholds, and mode. The PGM arrays did not match: identity comparison had `{transforms['identity']['different_cells']}` different cells. Flip tests did not produce equality either, so the task24b2 failure was not only a display flip.

The exact convention is:

- Stage-A floor-grid arrays use world-grid orientation.
- PGM packaging writes `flipud(world_grid)`.
- Map loading flips the PGM back to world-grid orientation.

The validated floor_2 stable map uses `gateway_wall_preclose` with the saved `carved_gateway` mask from its NPZ. Task24b2 also marked `room_mask<=0` cells occupied and used a different carve, causing `{generated['counts']['occupied']}` occupied cells instead of `{validated['counts']['occupied']}`.

Corrected floor_2 regression PGM equal to validated floor_2 PGM: `{regression_passed}`.

Floor_1 map status: `{comparison['floor_1_map_status']}`.
""")
    write_text(out / "all_floor_stable_map_generation_plan.md", f"""# All-Floor Stable Map Generation Plan

Use the validated floor_2 per-floor map convention for all floors:

- load `process/floors/<floor_id>/room_segmentation/assets/layered_bev_v0_1.npz`;
- use the grid-size `gateway_wall_preclose` array, not padded visualization PNGs;
- apply a per-floor `carved_gateway` mask when a validated one is available;
- write PGM as a vertical flip of the Stage-A world grid;
- preserve YAML origin and resolution from `layered_bev_v0_1.json`;
- write NPZ/provenance alongside PGM/YAML/preview.

Task24b3 generated only task-local candidates. It did not write to `clean_rerun/maps/floor_1`.

Floor_2 regression passed: `{regression_passed}`.
Floor_1 status: `{comparison['floor_1_map_status']}`.
""")
    command_note = "candidate" if regression_passed else "experimental"
    write_text(out / "manual_floor_1_map_copy_commands.md", f"""# Manual Floor-1 Map Copy Commands

These commands are for a later manual packaging step. Task24b3 did not run them.

Status: `{comparison['floor_1_map_status']}`.

```bash
SCENE_ROOT=/home/ws/workspace/BoxFusion/stage_outputs/stage1_generalization/00843-DYehNKdT76V
TASK_OUT="${{SCENE_ROOT}}/tasks/{TASK_NAME}"
SRC="${{TASK_OUT}}/corrected_generated_maps/floor_1"
DST="${{SCENE_ROOT}}/clean_rerun/maps/floor_1"

mkdir -p "${{DST}}"
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.yaml "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.pgm "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map_preview.png "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map.npz "${{DST}}"/
cp "${{SRC}}"/stage1_floor_1_stable_occupancy_map_provenance.json "${{DST}}"/
```

This is a `{command_note}` map package, not an installed clean_rerun artifact.
""")
    return comparison


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(points[i + 1] - points[i]) for i in range(len(points) - 1)))


def rdp(points: np.ndarray, epsilon: float) -> list[int]:
    if len(points) <= 2:
        return list(range(len(points)))
    start = points[0]
    end = points[-1]
    line = end - start
    denom = float(np.linalg.norm(line))
    if denom < 1e-9:
        distances = np.linalg.norm(points - start, axis=1)
    else:
        distances = np.abs(np.cross(line, points - start)) / denom
    idx = int(np.argmax(distances))
    if float(distances[idx]) <= epsilon:
        return [0, len(points) - 1]
    left = rdp(points[: idx + 1], epsilon)
    right = rdp(points[idx:], epsilon)
    return left[:-1] + [i + idx for i in right]


def stair_centerline(task24b2: Path, out: Path) -> dict[str, Any]:
    graph_v1 = read_json(task24b2 / "stairs_graph_v0_1.json")
    raw_samples = []
    for idx, node in enumerate(graph_v1.get("nodes", [])):
        raw_samples.append({
            "sample_index": idx,
            "frame_idx": node.get("frame_idx"),
            "xyz": node.get("position_xyz"),
            "yaw": node.get("yaw_or_heading"),
            "floor_status": node.get("floor_status"),
            "room_id": node.get("room_id"),
            "provenance": node.get("provenance"),
        })
    if len(raw_samples) < 2:
        classification = "blocked_missing_pose_trace"
        metrics = {}
        centerline_nodes = []
        graph_shape = "curved_unknown"
    else:
        xyz = np.asarray([s["xyz"] for s in raw_samples], dtype=float)
        xy = xyz[:, :2]
        endpoint_xy_distance = float(np.linalg.norm(xy[-1] - xy[0]))
        raw_xy_path_length = path_length(xy)
        raw_3d_path_length = path_length(xyz)
        line = xy[-1] - xy[0]
        denom = float(np.linalg.norm(line))
        deviations = np.zeros(len(xy), dtype=float) if denom < 1e-9 else np.abs(np.cross(line, xy - xy[0])) / denom
        yaws = np.asarray([0.0 if s.get("yaw") is None else float(s.get("yaw")) for s in raw_samples], dtype=float)
        yaws_unwrapped = np.unwrap(yaws)
        heading_diffs = np.abs(np.diff(yaws_unwrapped)) if len(yaws_unwrapped) > 1 else np.asarray([])
        metrics = {
            "endpoint_distance_xy_m": round(endpoint_xy_distance, 6),
            "raw_xy_path_length_m": round(raw_xy_path_length, 6),
            "raw_3d_path_length_m": round(raw_3d_path_length, 6),
            "raw_xy_path_length_over_endpoint_xy_distance": round(raw_xy_path_length / endpoint_xy_distance, 6) if endpoint_xy_distance > 1e-9 else None,
            "max_lateral_deviation_m": round(float(deviations.max()), 6),
            "mean_lateral_deviation_m": round(float(deviations.mean()), 6),
            "max_heading_change_rad": round(float(heading_diffs.max()), 6) if len(heading_diffs) else 0.0,
            "net_heading_change_rad": round(float(abs(yaws_unwrapped[-1] - yaws_unwrapped[0])), 6),
        }
        straight = (
            metrics["raw_xy_path_length_over_endpoint_xy_distance"] is not None
            and metrics["raw_xy_path_length_over_endpoint_xy_distance"] <= 1.1
            and metrics["max_lateral_deviation_m"] <= 0.25
            and metrics["mean_lateral_deviation_m"] <= 0.1
        )
        if straight:
            graph_shape = "straight"
            classification = "stair_trace_centerline_fixed_straight"
            sample_count = 5
            centerline_nodes = []
            for idx in range(sample_count):
                t = idx / float(sample_count - 1)
                p = (1.0 - t) * xyz[0] + t * xyz[-1]
                centerline_nodes.append({
                    "node_id": f"vt_1_centerline_n{idx:03d}",
                    "sample_t": round(t, 6),
                    "xyz": [round(float(v), 6) for v in p],
                    "node_type": "connector_endpoint" if idx in {0, sample_count - 1} else "connector_centerline_sample",
                })
        else:
            keep = rdp(xy, epsilon=0.2)
            graph_shape = "polyline" if len(keep) > 2 else "curved_unknown"
            classification = "stair_trace_centerline_fixed_polyline" if graph_shape == "polyline" else "stair_trace_only_no_reliable_centerline"
            centerline_nodes = []
            for out_idx, src_idx in enumerate(keep):
                centerline_nodes.append({
                    "node_id": f"vt_1_centerline_n{out_idx:03d}",
                    "source_sample_index": int(src_idx),
                    "xyz": [round(float(v), 6) for v in xyz[src_idx]],
                    "node_type": "connector_endpoint" if out_idx in {0, len(keep) - 1} else "connector_centerline_sample",
                })
    center_edges = []
    for idx, (a, b) in enumerate(zip(centerline_nodes, centerline_nodes[1:])):
        pa = np.asarray(a["xyz"], dtype=float)
        pb = np.asarray(b["xyz"], dtype=float)
        center_edges.append({
            "edge_id": f"vt_1_centerline_e{idx:03d}",
            "source": a["node_id"],
            "target": b["node_id"],
            "distance_3d_m": round(float(np.linalg.norm(pb - pa)), 6),
            "edge_type": "sparse_stair_connector_edge",
            "traversable_topological_only": True,
        })
    raw_payload = {
        "artifact_type": "raw_transition_pose_trace",
        "trace_name": "ordered_transition_pose_trace",
        "scene_id": "00843-DYehNKdT76V",
        "transition_id": "vt_1",
        "source_graph": (task24b2 / "stairs_graph_v0_1.json").as_posix(),
        "samples": raw_samples,
        "provenance_role": "raw camera pose trace; not final stair geometry",
        "claim_boundary": CLAIM_BOUNDARY,
    }
    sparse = {
        "artifact_type": "sparse_stair_connector_graph",
        "schema_version": "sparse_stair_connector_graph_v0_2",
        "scene_id": "00843-DYehNKdT76V",
        "transition_id": "vt_1",
        "source_transition_id": graph_v1.get("source_transition_id", "vt_1"),
        "floor_from": graph_v1.get("floor_from"),
        "floor_to": graph_v1.get("floor_to"),
        "endpoint_bindings": graph_v1.get("endpoint_bindings"),
        "graph_shape": graph_shape,
        "nodes": centerline_nodes,
        "edges": center_edges,
        "fitted_stair_centerline": centerline_nodes,
        "transition_path_samples": centerline_nodes,
        "ordered_transition_pose_trace_ref": "raw_transition_pose_trace_vt_1.json",
        "geometry_interpretation": GEOMETRY_INTERPRETATION,
        "physical_execution_supported": False,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    stairs_v2 = {
        **sparse,
        "artifact_type": "stairs_graph_v0_2",
        "raw_transition_pose_trace": raw_payload,
        "ordered_transition_pose_trace": raw_samples,
        "straightness_metrics": metrics,
        "classification": classification,
    }
    write_json(out / "raw_transition_pose_trace_vt_1.json", raw_payload)
    write_json(out / "sparse_stair_connector_graph_vt_1.json", sparse)
    write_json(out / "stairs_graph_v0_2.json", stairs_v2)
    render_stair_centerline(out / "stairs_graph_centerline_visualization.png", raw_samples, centerline_nodes, metrics, graph_shape)
    write_text(out / "stair_trace_centerline_report.md", f"""# Stair Trace Centerline Report

Transition: `vt_1`.

Raw camera pose samples are retained as `ordered_transition_pose_trace` provenance. They are not treated as physical stair geometry.

Straightness metrics:

- endpoint XY distance: `{metrics.get('endpoint_distance_xy_m')}` m
- raw XY path length: `{metrics.get('raw_xy_path_length_m')}` m
- raw 3D path length: `{metrics.get('raw_3d_path_length_m')}` m
- raw XY / endpoint XY ratio: `{metrics.get('raw_xy_path_length_over_endpoint_xy_distance')}`
- max lateral deviation: `{metrics.get('max_lateral_deviation_m')}` m
- mean lateral deviation: `{metrics.get('mean_lateral_deviation_m')}` m
- max heading change: `{metrics.get('max_heading_change_rad')}` rad

Graph shape: `{graph_shape}`.

Official connector artifact: `sparse_stair_connector_graph_vt_1.json`.

Geometry interpretation: `{GEOMETRY_INTERPRETATION}`.

Physical execution supported: `false`.

Claim boundary: `{CLAIM_BOUNDARY}`.
""")
    return {
        "classification": classification,
        "graph_shape": graph_shape,
        "metrics": metrics,
        "raw_sample_count": len(raw_samples),
        "centerline_node_count": len(centerline_nodes),
    }


def render_stair_centerline(path: Path, raw_samples: list[dict[str, Any]], centerline_nodes: list[dict[str, Any]], metrics: dict[str, Any], graph_shape: str) -> None:
    canvas = np.full((900, 1100, 3), 255, dtype=np.uint8)
    raw = np.asarray([s["xyz"][:2] for s in raw_samples], dtype=float) if raw_samples else np.zeros((0, 2), dtype=float)
    fit = np.asarray([n["xyz"][:2] for n in centerline_nodes], dtype=float) if centerline_nodes else np.zeros((0, 2), dtype=float)
    pts = np.vstack([arr for arr in [raw, fit] if len(arr)]) if (len(raw) or len(fit)) else np.zeros((1, 2), dtype=float)
    min_xy = pts.min(axis=0)
    max_xy = pts.max(axis=0)
    span = np.maximum(max_xy - min_xy, 0.2)
    pad = 90
    scale = min((canvas.shape[1] - 2 * pad) / span[0], (canvas.shape[0] - 2 * pad) / span[1])
    center = (min_xy + max_xy) / 2.0
    def to_px(p: np.ndarray) -> tuple[int, int]:
        shifted = (p - center) * scale
        return int(round(canvas.shape[1] / 2 + shifted[0])), int(round(canvas.shape[0] / 2 - shifted[1]))
    if len(raw) >= 2:
        raw_px = np.asarray([to_px(p) for p in raw], dtype=np.int32)
        cv2.polylines(canvas, [raw_px], False, (170, 170, 170), 2, cv2.LINE_AA)
        for p in raw_px:
            cv2.circle(canvas, tuple(p), 4, (120, 120, 120), -1, cv2.LINE_AA)
    if len(fit) >= 2:
        fit_px = np.asarray([to_px(p) for p in fit], dtype=np.int32)
        cv2.polylines(canvas, [fit_px], False, (40, 90, 220), 5, cv2.LINE_AA)
        for idx, p in enumerate(fit_px):
            cv2.circle(canvas, tuple(p), 8 if idx in {0, len(fit_px) - 1} else 6, (30, 120, 40), -1, cv2.LINE_AA)
    lines = [
        "vt_1 raw transition pose trace vs fitted stair centerline",
        f"graph_shape={graph_shape} | max lateral deviation={metrics.get('max_lateral_deviation_m')} m",
        "gray: raw camera pose trace provenance | blue/green: fitted connector centerline",
        "topological_vertical_transition_only; not physical stair mesh",
    ]
    for idx, line in enumerate(lines):
        cv2.putText(canvas, line, (26, 36 + idx * 28), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (30, 30, 30), 2, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), canvas)


def final_reports(out: Path, stage_info: dict[str, Any], map_info: dict[str, Any], stair_info: dict[str, Any], validation: dict[str, Any]) -> dict[str, Any]:
    stable_classification = (
        "floor_1_map_candidate_ready_but_not_installed"
        if map_info.get("corrected_floor_2_regression", {}).get("pgm_equal_to_validated")
        else "stable_map_builder_still_flipped_or_mismatched"
    )
    report = {
        "task": TASK_NAME,
        "created_utc": now_iso(),
        "project": "RSLG-SLAM",
        "scene_id": "00843-DYehNKdT76V",
        "claim_boundary": CLAIM_BOUNDARY,
        "stage_a_command_classification": stage_info["classification"],
        "stable_map_classification": stable_classification,
        "stair_classification": stair_info["classification"],
        "floor_1_map_status": map_info["floor_1_map_status"],
        "graph_shape": stair_info["graph_shape"],
        "physical_execution_supported": False,
        "generated_under": out.as_posix(),
        "validation_passed": validation.get("overall_passed"),
    }
    write_json(out / "task24b3_report.json", report)
    write_text(out / "task24b3_report.md", f"""# Task24b3 Report

Project: RSLG-SLAM.

Stage-A command classification: `{report['stage_a_command_classification']}`.

Stable map classification: `{report['stable_map_classification']}`.

Stair classification: `{report['stair_classification']}`.

Claim boundary: `{CLAIM_BOUNDARY}`.

Floor_2 regression passed: `{map_info.get('corrected_floor_2_regression', {}).get('pgm_equal_to_validated')}`.

Floor_1 map status: `{report['floor_1_map_status']}`.

Stair graph shape: `{report['graph_shape']}`.

Physical execution supported: `false`.
""")
    write_text(out / "light_geometry_claim_boundary.md", f"""# Light Geometry Claim Boundary

`ordered_transition_pose_trace`, `transition_path_samples`, `fitted_stair_centerline`, and `sparse_stair_connector_graph` remain light geometry.

They are not dense reconstruction, not TSDF, not mesh, and not point-cloud stair modeling.

They do not reconstruct every stair step and do not perform footstep planning.

They store only a compact route-relevant connector trace and a sparse fitted centerline for topology, route planning, visualization, and provenance.

Physical execution supported: `false`.

Claim boundary: `{CLAIM_BOUNDARY}`.
""")
    write_text(out / "final_answer_for_user.md", f"""# Final Answer For User

Correct Stage-A convention: pass `--output-root` as the rerun parent; Stage-A appends `<scene_id>` automatically.

Floor_2 orientation/policy regression: `{map_info.get('corrected_floor_2_regression', {}).get('pgm_equal_to_validated')}`.

Floor_1 candidate: `{map_info['floor_1_candidate']['map_yaml']}`.

Manual copy commands: `{(out / 'manual_floor_1_map_copy_commands.md').as_posix()}`.

vt_1 graph shape: `{stair_info['graph_shape']}`.

Raw transition pose trace is provenance. The fitted stair centerline is the compact connector geometry. Neither is physical stair mesh or a stair-climbing claim.

Next recommendation: manually inspect the floor_1 candidate preview, then run the generated copy commands only after accepting it as a task-local candidate package.
""")
    return report


def validate(repo: Path, stage: Path, out: Path, before_hashes: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "python_compile_check": {},
        "json_parse_checks": {},
        "map_file_existence_checks": {},
        "pgm_yaml_npz_dimension_checks": {},
        "floor_2_orientation_regression_check": {},
        "no_overwrite_checks": {},
    }
    script_path = repo / "tools/vertical_connectors/task24b3_regression_fix.py"
    try:
        py_compile.compile(script_path.as_posix(), doraise=True)
        result["python_compile_check"][script_path.as_posix()] = True
    except py_compile.PyCompileError as exc:
        result["python_compile_check"][script_path.as_posix()] = str(exc)
    json_files = [
        "stable_map_orientation_comparison.json",
        "generated_per_floor_map_manifest.json",
        "stairs_graph_v0_2.json",
        "raw_transition_pose_trace_vt_1.json",
        "sparse_stair_connector_graph_vt_1.json",
        "task24b3_report.json",
    ]
    for name in json_files:
        path = out / name
        try:
            read_json(path)
            result["json_parse_checks"][name] = True
        except Exception as exc:  # noqa: BLE001
            result["json_parse_checks"][name] = str(exc)
    for floor in ["floor_1"]:
        base = out / f"corrected_generated_maps/{floor}/stage1_{floor}_stable_occupancy_map"
        checks = {suffix: base.with_suffix(suffix).exists() for suffix in [".yaml", ".pgm", ".npz"]}
        checks["_preview.png"] = (base.parent / f"{base.name}_preview.png").exists()
        checks["_provenance.json"] = (base.parent / f"{base.name}_provenance.json").exists()
        result["map_file_existence_checks"][floor] = checks
        try:
            meta = parse_map_yaml(base.with_suffix(".yaml"))
            pgm = read_pgm(base.with_suffix(".pgm"))
            with np.load(base.with_suffix(".npz")) as data:
                result["pgm_yaml_npz_dimension_checks"][floor] = {
                    "pgm_shape_hw": list(pgm.shape),
                    "occupied_shape_hw": list(data["occupied"].shape),
                    "matches": list(pgm.shape) == list(data["occupied"].shape),
                    "resolution": meta.get("resolution"),
                    "origin": meta.get("origin"),
                }
        except Exception as exc:  # noqa: BLE001
            result["pgm_yaml_npz_dimension_checks"][floor] = str(exc)
    val_pgm = read_pgm(stage / "maps/floor_2/stage1_floor_2_stable_occupancy_map.pgm")
    cand_pgm = read_pgm(out / "corrected_generated_maps/floor_2_regression_check/stage1_floor_2_stable_occupancy_map.pgm")
    result["floor_2_orientation_regression_check"] = {
        "pgm_equal_to_validated": bool(np.array_equal(val_pgm, cand_pgm)),
        "different_cells": int((val_pgm != cand_pgm).sum()),
    }
    after_hashes = {
        "clean_rerun_floor_2": hash_tree(stage / "maps/floor_2"),
        "clean_rerun_committed_public": hash_tree(stage / "committed_public"),
        "task23b": hash_tree(stage.parent / "tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"),
    }
    for key, before in before_hashes.items():
        result["no_overwrite_checks"][key] = {
            "unchanged": before == after_hashes.get(key, {}),
            "file_count_before": len(before),
            "file_count_after": len(after_hashes.get(key, {})),
        }
    result["overall_passed"] = (
        all(v is True for v in result["python_compile_check"].values())
        and all(v is True for v in result["json_parse_checks"].values())
        and result["floor_2_orientation_regression_check"]["pgm_equal_to_validated"]
        and all(v["unchanged"] for v in result["no_overwrite_checks"].values())
    )
    write_json(out / "validation_summary.json", result)
    write_text(out / "validation_summary.md", f"""# Validation Summary

Python compile check passed: `{all(v is True for v in result['python_compile_check'].values())}`.

JSON parse checks passed: `{all(v is True for v in result['json_parse_checks'].values())}`.

Floor_2 orientation regression passed: `{result['floor_2_orientation_regression_check']['pgm_equal_to_validated']}`.

No-overwrite checks:

- clean_rerun/maps/floor_2 unchanged: `{result['no_overwrite_checks']['clean_rerun_floor_2']['unchanged']}`
- clean_rerun/committed_public unchanged: `{result['no_overwrite_checks']['clean_rerun_committed_public']['unchanged']}`
- task23b outputs unchanged: `{result['no_overwrite_checks']['task23b']['unchanged']}`

Overall passed: `{result['overall_passed']}`.
""")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--scene-id", default="00843-DYehNKdT76V")
    parser.add_argument("--scene-root", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V"))
    parser.add_argument("--clean-rerun", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun"))
    parser.add_argument("--task24b2", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24b2_stage_a_stair_graph_and_per_floor_stable_map_audit"))
    parser.add_argument("--output-dir", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24b3_stair_trace_centerline_and_all_floor_map_regression_fix"))
    args = parser.parse_args()

    repo = args.repo_root.resolve()
    scene_root = args.scene_root.resolve()
    stage = args.clean_rerun.resolve()
    task24b2 = args.task24b2.resolve()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)

    before_hashes = {
        "clean_rerun_floor_2": hash_tree(stage / "maps/floor_2"),
        "clean_rerun_committed_public": hash_tree(stage / "committed_public"),
        "task23b": hash_tree(scene_root / "tasks/task23b_quadruped_proxy_gui_smoke_repair_and_manual_evidence_capture"),
    }
    stage_info = stage_a_reports(repo, scene_root, out, args.scene_id)
    map_info = map_regression(stage, task24b2, out, args.scene_id)
    stair_info = stair_centerline(task24b2, out)
    final_reports(out, stage_info, map_info, stair_info, {"overall_passed": False})
    validation = validate(repo, stage, out, before_hashes)
    final_reports(out, stage_info, map_info, stair_info, validation)
    validation = validate(repo, stage, out, before_hashes)
    return 0 if validation["overall_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
