#!/usr/bin/env python3
"""Validate Stage1 route trajectories against the occupancy map."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scene_runtime_common import load_nav_map, map_value, read_json, write_json


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def load_samples(trace_jsonl: Path | None, trajectory_json: Path | None) -> tuple[list[dict[str, Any]], str | None]:
    if trace_jsonl and trace_jsonl.exists():
        return read_jsonl(trace_jsonl), trace_jsonl.as_posix()
    if trajectory_json and trajectory_json.exists():
        payload = read_json(trajectory_json)
        return list(payload.get("samples") or []), trajectory_json.as_posix()
    return [], None


def infer_group_labels(samples: list[dict[str, Any]]) -> list[str]:
    labels: list[str] = []
    follow_run = -1
    previous_phase: str | None = None
    for sample in samples:
        phase = str(sample.get("phase") or "unknown")
        if phase == "follow_path":
            explicit = sample.get("slice_index")
            if explicit is None and previous_phase != "follow_path":
                follow_run += 1
            slice_index = explicit if explicit is not None else max(follow_run, 0)
            labels.append(f"slice_{slice_index}_follow_path")
        elif phase in {"post", "dwell", "room13_dwell"}:
            labels.append("room13_dwell_post")
        elif phase == "fallback":
            labels.append("fallback")
        else:
            labels.append("unknown")
        previous_phase = phase
    return labels


def spin_like(sample: dict[str, Any], linear_threshold: float, angular_threshold: float) -> bool:
    lin = sample.get("cmd_vel_linear_x")
    ang = sample.get("cmd_vel_angular_z")
    if lin is None or ang is None:
        return False
    return abs(float(lin)) < linear_threshold and abs(float(ang)) > angular_threshold


def segment_hits_occupied(
    grid: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    a: dict[str, Any],
    b: dict[str, Any],
    occupied_threshold_pixel_le: int,
) -> tuple[bool, list[dict[str, Any]]]:
    ax, ay = float(a["x"]), float(a["y"])
    bx, by = float(b["x"]), float(b["y"])
    dist = math.hypot(bx - ax, by - ay)
    steps = max(2, int(math.ceil(dist / max(resolution * 0.5, 1e-6))) + 1)
    hits: list[dict[str, Any]] = []
    for idx in range(steps):
        t = idx / float(steps - 1)
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        value = map_value(grid, x, y, resolution, origin)
        if value is None or int(value) <= occupied_threshold_pixel_le:
            hits.append({"x": round(x, 6), "y": round(y, 6), "map_value": value})
            break
    return bool(hits), hits


def validate_point_sequence(
    points: list[dict[str, Any]],
    map_yaml: Path,
    *,
    occupied_threshold_pixel_le: int = 100,
) -> dict[str, Any]:
    grid, resolution, origin, meta = load_nav_map(map_yaml)
    occupied_points = []
    occupied_segments = []
    for idx, point in enumerate(points):
        value = map_value(grid, float(point["x"]), float(point["y"]), resolution, origin)
        if value is None or int(value) <= occupied_threshold_pixel_le:
            occupied_points.append({"point_index": idx, "x": point.get("x"), "y": point.get("y"), "map_value": value})
    for idx, (a, b) in enumerate(zip(points, points[1:])):
        hit, hits = segment_hits_occupied(grid, resolution, origin, a, b, occupied_threshold_pixel_le)
        if hit:
            occupied_segments.append({
                "from_index": idx,
                "to_index": idx + 1,
                "from_xy": [round(float(a["x"]), 6), round(float(a["y"]), 6)],
                "to_xy": [round(float(b["x"]), 6), round(float(float(b["y"])), 6)],
                "first_hit": hits[0] if hits else None,
            })
    return {
        "map_yaml": map_yaml.as_posix(),
        "map_image": meta.get("image"),
        "occupied_threshold_pixel_le": occupied_threshold_pixel_le,
        "point_count": len(points),
        "occupied_sample_points": len(occupied_points),
        "occupied_segment_samples": len(occupied_segments),
        "wall_crossing_validation_passed": len(occupied_points) == 0 and len(occupied_segments) == 0,
        "occupied_point_examples_first20": occupied_points[:20],
        "occupied_segment_examples_first20": occupied_segments[:20],
    }


def validate_samples(
    samples: list[dict[str, Any]],
    map_yaml: Path,
    *,
    occupied_threshold_pixel_le: int = 100,
    spin_linear_threshold: float = 0.03,
    spin_angular_threshold: float = 0.5,
) -> dict[str, Any]:
    grid, resolution, origin, meta = load_nav_map(map_yaml)
    labels = infer_group_labels(samples)
    stats: dict[str, dict[str, Any]] = {}
    occupied_examples: list[dict[str, Any]] = []
    crossing_examples: list[dict[str, Any]] = []
    samples_by_group: dict[str, list[tuple[int, dict[str, Any]]]] = defaultdict(list)
    for idx, (sample, group) in enumerate(zip(samples, labels)):
        samples_by_group[group].append((idx, sample))

    for group, rows in samples_by_group.items():
        point_count = 0
        spin_count = 0
        first_idx = rows[0][0] if rows else None
        last_idx = rows[-1][0] if rows else None
        for idx, sample in rows:
            value = map_value(grid, float(sample["x"]), float(sample["y"]), resolution, origin)
            if value is None or int(value) <= occupied_threshold_pixel_le:
                point_count += 1
                occupied_examples.append({
                    "group": group,
                    "sample_index": sample.get("sample_index", idx),
                    "x": round(float(sample["x"]), 6),
                    "y": round(float(sample["y"]), 6),
                    "map_value": value,
                    "nearest_wp": sample.get("nearest_wp_index"),
                })
            if spin_like(sample, spin_linear_threshold, spin_angular_threshold):
                spin_count += 1
        segment_count = 0
        segment_pairs = list(zip(rows, rows[1:]))
        if rows and rows[0][0] > 0:
            # Charge the handoff into a phase to that destination phase. This makes
            # fallback validation catch crossings that occur exactly at recovery start.
            first_global_idx = rows[0][0]
            segment_pairs.insert(0, ((first_global_idx - 1, samples[first_global_idx - 1]), rows[0]))
        for (from_idx, a), (to_idx, b) in segment_pairs:
            hit, hits = segment_hits_occupied(grid, resolution, origin, a, b, occupied_threshold_pixel_le)
            if hit:
                segment_count += 1
                crossing_examples.append({
                    "group": group,
                    "phase": a.get("phase"),
                    "from_idx": a.get("sample_index", from_idx),
                    "to_idx": b.get("sample_index", to_idx),
                    "from_xy": [round(float(a["x"]), 3), round(float(a["y"]), 3)],
                    "to_xy": [round(float(b["x"]), 3), round(float(b["y"]), 3)],
                    "from_nearest_wp": a.get("nearest_wp_index"),
                    "to_nearest_wp": b.get("nearest_wp_index"),
                    "first_hit": hits[0] if hits else None,
                })
        stats[group] = {
            "sample_count": len(rows),
            "occupied_sample_points": point_count,
            "occupied_segment_samples": segment_count,
            "spin_like_samples": spin_count,
            "first_idx": first_idx,
            "last_idx": last_idx,
        }

    total_occupied_points = sum(int(item["occupied_sample_points"]) for item in stats.values())
    total_occupied_segments = sum(int(item["occupied_segment_samples"]) for item in stats.values())
    return {
        "artifact_type": "scene_trajectory_wall_crossing_validation",
        "version": "v0_1",
        "created_utc": now_iso(),
        "map_yaml": map_yaml.as_posix(),
        "map_image": meta.get("image"),
        "map_resolution_m": resolution,
        "occupied_threshold_pixel_le": occupied_threshold_pixel_le,
        "sample_count": len(samples),
        "phase_stats": stats,
        "per_phase_wall_crossing_counts": {
            group: {
                "occupied_sample_points": item["occupied_sample_points"],
                "occupied_segment_samples": item["occupied_segment_samples"],
            }
            for group, item in stats.items()
        },
        "spin_like_sample_counts": {group: item["spin_like_samples"] for group, item in stats.items()},
        "occupied_point_examples_first20": occupied_examples[:20],
        "crossing_examples_first20": crossing_examples[:20],
        "wall_crossing_detected": total_occupied_points > 0 or total_occupied_segments > 0,
        "wall_crossing_validation_passed": total_occupied_points == 0 and total_occupied_segments == 0,
    }


def load_route_points(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists():
        return []
    payload = read_json(path)
    return list(payload.get("waypoints") or payload.get("poses") or [])


def plot_report(
    samples: list[dict[str, Any]],
    map_yaml: Path,
    output_png: Path,
    *,
    waypoints_json: Path | None = None,
    occupied_threshold_pixel_le: int = 100,
) -> None:
    grid, resolution, origin, _meta = load_nav_map(map_yaml)
    labels = infer_group_labels(samples)
    width = grid.shape[1] * resolution
    height = grid.shape[0] * resolution
    extent = [origin[0], origin[0] + width, origin[1], origin[1] + height]
    fig, ax = plt.subplots(figsize=(12, 12))
    wall = np.ma.masked_where(grid > occupied_threshold_pixel_le, grid)
    ax.imshow(wall, extent=extent, origin="lower", cmap="gray_r", alpha=0.75, interpolation="nearest")

    route = load_route_points(waypoints_json)
    if route:
        ax.plot([float(p["x"]) for p in route], [float(p["y"]) for p in route], color="#1f77b4", linewidth=1.5, alpha=0.65, label="planned route")
        ax.scatter([float(p["x"]) for p in route], [float(p["y"]) for p in route], color="#1f77b4", s=8, alpha=0.45)

    colors = {
        "slice_0_follow_path": "#2ca02c",
        "slice_1_follow_path": "#d62728",
        "room13_dwell_post": "#9467bd",
        "fallback": "#ff7f0e",
        "unknown": "#7f7f7f",
    }
    for group in sorted(set(labels)):
        rows = [sample for sample, label in zip(samples, labels) if label == group]
        if not rows:
            continue
        ax.plot(
            [float(s["x"]) for s in rows],
            [float(s["y"]) for s in rows],
            ".-",
            markersize=3,
            linewidth=1.0,
            color=colors.get(group, "#17becf"),
            label=group,
            alpha=0.85,
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.legend(loc="best")
    ax.set_title("Trajectory wall-crossing diagnostic")
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_png, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--trace-jsonl", type=Path)
    parser.add_argument("--trajectory-json", type=Path)
    parser.add_argument("--waypoints-json", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path)
    parser.add_argument("--occupied-threshold-pixel-le", type=int, default=100)
    parser.add_argument("--spin-linear-threshold", type=float, default=0.03)
    parser.add_argument("--spin-angular-threshold", type=float, default=0.5)
    args = parser.parse_args()

    samples, source = load_samples(args.trace_jsonl, args.trajectory_json)
    report = validate_samples(
        samples,
        args.map_yaml,
        occupied_threshold_pixel_le=args.occupied_threshold_pixel_le,
        spin_linear_threshold=args.spin_linear_threshold,
        spin_angular_threshold=args.spin_angular_threshold,
    )
    report["source_samples"] = source
    if args.output_png:
        plot_report(
            samples,
            args.map_yaml,
            args.output_png,
            waypoints_json=args.waypoints_json,
            occupied_threshold_pixel_le=args.occupied_threshold_pixel_le,
        )
        report["output_plot"] = args.output_png.as_posix()
    write_json(args.output_json, report)
    print(json.dumps({
        "wall_crossing_validation_passed": report["wall_crossing_validation_passed"],
        "phase_stats": report["phase_stats"],
        "output_json": args.output_json.as_posix(),
        "output_png": args.output_png.as_posix() if args.output_png else None,
    }, indent=2, sort_keys=True))
    return 0 if report["wall_crossing_validation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
