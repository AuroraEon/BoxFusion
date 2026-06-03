"""Visualization: route, approach, yaw local zoom."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .schemas import angle_wrap, distance


def plot_visualization(
    planner,
    dense_route: dict[str, Any],
    control_path: list[dict[str, Any]],
    semantic: dict[str, Any],
    candidate: dict[str, Any] | None,
    trajectory: list[dict[str, Any]],
    proxy: dict[str, Any] | None,
    path: Path,
    simplified_path: list[dict[str, Any]] | None = None,
    control_path_label: str = "rounded control path",
) -> None:
    """Generate route visualization plot."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return

    image = np.flipud(planner.grid)
    extent = [planner.origin[0], planner.origin[0] + planner.grid.shape[1] * planner.resolution,
              planner.origin[1], planner.origin[1] + planner.grid.shape[0] * planner.resolution]
    fig, axis = plt.subplots(figsize=(11, 8))
    axis.imshow(image, cmap="gray", extent=extent, origin="upper")

    # Dense route
    pts = dense_route.get("waypoints") or []
    axis.plot([p["x"] for p in pts], [p["y"] for p in pts], color="#1679c4", linewidth=1.0, alpha=0.4, label="dense A* route")

    # Simplified control path (before rounding)
    if simplified_path:
        axis.plot([p["x"] for p in simplified_path], [p["y"] for p in simplified_path],
                  color="#ccaa00", linewidth=1.2, alpha=0.6, linestyle="--", label="simplified path (pre-rounding)")

    # Final control path
    axis.plot([p["x"] for p in control_path], [p["y"] for p in control_path],
              color="#ff6600", linewidth=2.0, label=control_path_label, marker=".", markersize=2)

    # Semantic anchors
    anchors = semantic.get("waypoints") or []
    gw_anchors = [a for a in anchors if a.get("source") == "gateway"]
    other_anchors = [a for a in anchors if a.get("source") != "gateway"]
    axis.scatter([a["x"] for a in other_anchors], [a["y"] for a in other_anchors],
                 color="#ee7c20", s=30, zorder=5, label="semantic anchors")
    axis.scatter([a["x"] for a in gw_anchors], [a["y"] for a in gw_anchors],
                 marker="D", color="#ff3399", s=35, zorder=5, label="gateways")

    # Executed trajectory
    if trajectory:
        axis.plot([t["x"] for t in trajectory], [t["y"] for t in trajectory],
                  color="#6f2dbd", linewidth=1.3, label="executed trajectory")

    # Approach candidate
    if candidate:
        xy = candidate["world_xy"]
        axis.scatter([xy[0]], [xy[1]], marker="X", color="#169c4b", s=65, zorder=6, label="approach candidate")
        if proxy:
            axis.plot([xy[0], proxy["yaw_proxy_xy"][0]], [xy[1], proxy["yaw_proxy_xy"][1]],
                      "--", color="#cb2364", linewidth=1.2, label="object-facing ray")

    # Start
    if pts:
        axis.scatter([pts[0]["x"]], [pts[0]["y"]], marker="o", color="green", s=50, zorder=6, label="start")

    axis.set_title("RSLG-SLAM: Curvature tracking with corner-rounded control path")
    axis.set_aspect("equal")
    axis.legend(fontsize=7, loc="best")
    axis.set_xlim(-10.2, 2.0)
    axis.set_ylim(-0.8, 7.2)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_approach_yaw_local_zoom(
    planner,
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    trajectory: list[dict[str, Any]],
    yaw_result: dict[str, Any],
    path: Path,
) -> None:
    """Generate a local zoom visualization showing approach candidate, yaw facing, and final pose."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return

    cand_xy = candidate["world_xy"]
    proxy_xy = proxy["yaw_proxy_xy"]
    zoom_r = 1.8
    cx, cy = cand_xy[0], cand_xy[1]

    image = np.flipud(planner.grid)
    extent = [planner.origin[0], planner.origin[0] + planner.grid.shape[1] * planner.resolution,
              planner.origin[1], planner.origin[1] + planner.grid.shape[0] * planner.resolution]
    fig, axis = plt.subplots(figsize=(8, 8))
    axis.imshow(image, cmap="gray", extent=extent, origin="upper", alpha=0.7)

    # Local trajectory
    local_traj = [t for t in trajectory if abs(t["x"] - cx) < zoom_r * 1.5 and abs(t["y"] - cy) < zoom_r * 1.5]
    if local_traj:
        axis.plot([t["x"] for t in local_traj], [t["y"] for t in local_traj],
                  color="#6f2dbd", linewidth=1.8, label="trajectory")

    # Approach candidate
    axis.scatter([cand_xy[0]], [cand_xy[1]], marker="X", color="#169c4b", s=120, zorder=6, label="approach candidate")
    # Object/yaw proxy
    axis.scatter([proxy_xy[0]], [proxy_xy[1]], marker="*", color="#ff0000", s=150, zorder=6, label="object/yaw proxy")
    # Facing ray
    axis.plot([cand_xy[0], proxy_xy[0]], [cand_xy[1], proxy_xy[1]],
              "--", color="#cb2364", linewidth=1.5, label="expected facing ray")

    # Final robot pose
    final_pose = yaw_result.get("final_pose")
    if final_pose:
        rx, ry, ryaw = float(final_pose["x"]), float(final_pose["y"]), float(final_pose["yaw"])
        axis.scatter([rx], [ry], marker="o", color="#0066ff", s=100, zorder=7, label="final robot pose")
        arrow_len = 0.35
        axis.annotate("", xy=(rx + arrow_len * math.cos(ryaw), ry + arrow_len * math.sin(ryaw)),
                      xytext=(rx, ry),
                      arrowprops=dict(arrowstyle="->", color="#0066ff", lw=2.5))
        fd = distance(final_pose, cand_xy)
        axis.annotate(f"dist={fd:.3f}m", xy=(rx, ry), xytext=(rx + 0.15, ry - 0.15),
                      fontsize=8, color="#0066ff")
        target_yaw = math.atan2(proxy_xy[1] - ry, proxy_xy[0] - rx)
        yaw_err = abs(angle_wrap(target_yaw - ryaw))
        axis.annotate(f"yaw_err={yaw_err:.3f}rad", xy=(rx, ry), xytext=(rx + 0.15, ry - 0.30),
                      fontsize=8, color="#aa0000")

    axis.set_xlim(cx - zoom_r, cx + zoom_r)
    axis.set_ylim(cy - zoom_r, cy + zoom_r)
    axis.set_title("Approach + Yaw Local Zoom")
    axis.set_aspect("equal")
    axis.legend(fontsize=8, loc="best")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170)
    plt.close(fig)
