#!/usr/bin/env python3
"""Plot Stage1 route, sent controller paths, trajectory, and replayed controller paths."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scene_runtime_common import load_nav_map, read_json, write_json


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists():
        return rows
    for raw in path.read_text(encoding="utf-8").splitlines():
        raw = raw.strip()
        if raw:
            rows.append(json.loads(raw))
    return rows


def load_points_payload(path: Path | None) -> list[dict[str, Any]]:
    if not path or not path.exists():
        return []
    payload = read_json(path)
    return list(payload.get("waypoints") or payload.get("samples") or payload.get("points") or [])


def sent_path_files(sent_dir: Path | None, summary_json: Path | None) -> list[Path]:
    if summary_json and summary_json.exists():
        payload = read_json(summary_json)
        return [Path(row["path"]) for row in payload.get("paths") or [] if row.get("path") and Path(row["path"]).exists()]
    if sent_dir and sent_dir.exists():
        return sorted(sent_dir.glob("*.json"))
    return []


def plot_line(ax: Any, points: list[dict[str, Any]], *args: Any, **kwargs: Any) -> None:
    if len(points) < 1:
        return
    ax.plot([float(p["x"]) for p in points], [float(p["y"]) for p in points], *args, **kwargs)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--waypoints-json", type=Path, required=True)
    parser.add_argument("--trace-jsonl", type=Path)
    parser.add_argument("--trajectory-json", type=Path)
    parser.add_argument("--sent-controller-paths-dir", type=Path)
    parser.add_argument("--sent-controller-paths-summary", type=Path)
    parser.add_argument("--controller-paths-json", type=Path)
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--occupied-threshold-pixel-le", type=int, default=100)
    parser.add_argument("--crop-padding-m", type=float, default=1.75)
    parser.add_argument("--max-controller-paths-per-topic", type=int, default=120)
    args = parser.parse_args()

    grid, resolution, origin, _meta = load_nav_map(args.map_yaml)
    width = grid.shape[1] * resolution
    height = grid.shape[0] * resolution
    extent = [origin[0], origin[0] + width, origin[1], origin[1] + height]
    route = load_points_payload(args.waypoints_json)
    trace = read_jsonl(args.trace_jsonl) if args.trace_jsonl else load_points_payload(args.trajectory_json)
    controller = read_json(args.controller_paths_json) if args.controller_paths_json and args.controller_paths_json.exists() else {}
    sent_files = sent_path_files(args.sent_controller_paths_dir, args.sent_controller_paths_summary)
    sent_payloads = [read_json(path) for path in sent_files]

    all_crop_points: list[dict[str, Any]] = []
    all_crop_points.extend(route)
    all_crop_points.extend(trace)
    for payload in sent_payloads:
        all_crop_points.extend(payload.get("points") or [])
    for topic_paths in (controller.get("paths") or {}).values():
        for item in topic_paths[-args.max_controller_paths_per_topic:]:
            all_crop_points.extend(item.get("points") or [])

    fig, ax = plt.subplots(figsize=(12, 10))
    wall = np.ma.masked_where(grid > args.occupied_threshold_pixel_le, grid)
    ax.imshow(wall, extent=extent, origin="lower", cmap="gray_r", alpha=0.72, interpolation="nearest")

    plot_line(ax, route, color="#1f77b4", linewidth=1.4, alpha=0.55, label="original executable route")
    if route:
        ax.scatter([float(p["x"]) for p in route], [float(p["y"]) for p in route], color="#1f77b4", s=9, alpha=0.45)

    if trace:
        plot_line(ax, trace, color="#111111", linewidth=2.0, alpha=0.9, label="actual trajectory")
        ax.scatter([float(trace[0]["x"])], [float(trace[0]["y"])], marker="o", color="#111111", s=55, label="trajectory start")
        ax.scatter([float(trace[-1]["x"])], [float(trace[-1]["y"])], marker="X", color="#111111", s=70, label="trajectory end")

    sent_colors = ["#e377c2", "#17becf", "#bcbd22", "#ff7f0e", "#8c564b", "#2ca02c"]
    for idx, payload in enumerate(sent_payloads):
        points = payload.get("points") or []
        label = f"sent {payload.get('path_id')}"
        plot_line(ax, points, color=sent_colors[idx % len(sent_colors)], linewidth=2.0, alpha=0.85, label=label)

    topic_colors = {
        "/plan": "#d62728",
        "/received_global_plan": "#9467bd",
        "/transformed_global_plan": "#2ca02c",
        "/local_plan": "#ff7f0e",
    }
    for topic, paths in (controller.get("paths") or {}).items():
        for item in paths[-args.max_controller_paths_per_topic:]:
            plot_line(ax, item.get("points") or [], color=topic_colors.get(topic, "#7f7f7f"), linewidth=0.8, alpha=0.12)
        latest = paths[-1] if paths else None
        if latest:
            plot_line(ax, latest.get("points") or [], color=topic_colors.get(topic, "#7f7f7f"), linewidth=2.2, alpha=0.9, label=f"{topic} latest")

    key_indices = {33, 39, 51, 58}
    for point in route:
        if int(point.get("waypoint_index", -1)) in key_indices:
            ax.scatter([float(point["x"])], [float(point["y"])], s=80, marker="D", edgecolors="black", facecolors="white", zorder=6)
            ax.text(float(point["x"]) + 0.06, float(point["y"]) + 0.06, f"wp{point['waypoint_index']}", fontsize=9, weight="bold")

    if all_crop_points:
        xs = [float(p["x"]) for p in all_crop_points if p.get("x") is not None]
        ys = [float(p["y"]) for p in all_crop_points if p.get("y") is not None]
        if xs and ys:
            pad = float(args.crop_padding_m)
            ax.set_xlim(min(xs) - pad, max(xs) + pad)
            ax.set_ylim(min(ys) - pad, max(ys) + pad)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title("Controller handoff overlay")
    handles, labels = ax.get_legend_handles_labels()
    dedup: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        dedup.setdefault(label, handle)
    ax.legend(dedup.values(), dedup.keys(), loc="best", fontsize=8)
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.output_png, dpi=180)
    plt.close(fig)

    cmd_vel_rows = controller.get("cmd_vel") or []
    cmd_vel_summary = controller.get("cmd_vel_summary")
    if cmd_vel_summary is None and cmd_vel_rows:
        cmd_vel_summary = {
            "message_count": len(cmd_vel_rows),
            "spin_like_count": sum(
                1 for row in cmd_vel_rows
                if row.get("spin_like") or (abs(float(row.get("linear_x", 0.0))) < 0.03 and abs(float(row.get("angular_z", 0.0))) > 0.35)
            ),
            "moving_turn_count": sum(
                1 for row in cmd_vel_rows
                if row.get("moving_turn") or (abs(float(row.get("linear_x", 0.0))) >= 0.03 and abs(float(row.get("angular_z", 0.0))) > 0.35)
            ),
        }
    report = {
        "artifact_type": "controller_overlay_validation",
        "version": "v0_1",
        "created_utc": now_iso(),
        "output_png": args.output_png.as_posix(),
        "route_point_count": len(route),
        "trajectory_sample_count": len(trace),
        "sent_controller_path_count": len(sent_payloads),
        "controller_topic_path_counts": {topic: len(paths) for topic, paths in (controller.get("paths") or {}).items()},
        "cmd_vel_summary": cmd_vel_summary,
        "key_waypoints_plotted": sorted(key_indices),
    }
    write_json(args.output_json, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
