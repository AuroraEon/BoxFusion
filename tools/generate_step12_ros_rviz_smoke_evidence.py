#!/usr/bin/env python
"""Generate Step 12 ROS/RViz smoke-test evidence and fallback previews.

This script is intentionally conservative.  It never launches ROS, Gazebo, RViz,
Nav2, or any robot-control process.  It records whether ROS 2 is available and,
when it is not, renders committed marker_specs.json files into simple static
HTML/SVG/PNG previews using only the Python standard library.
"""

from __future__ import annotations

import csv
import hashlib
import html
import importlib.util
import json
import os
import shutil
import struct
import sys
import zlib
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
STEP11_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step11_ros_bridge_validation"
OUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step12_ros_rviz_smoke_test"

SCENES = [
    "00843-DYehNKdT76V",
    "00824-Dd4bFSTQ8gi",
    "00862-LT9Jq6dN3Ea",
    "00829-QaLdnwvtxbs",
]

ALLOWED_SOURCE_ARTIFACTS = {
    "topology_v0_1.json",
    "topology_query_report.json",
    "committed_room_world_model_v0_1.json",
    "committed_room_world_snapshot_v0_1.json",
    "topology_v0_1.json+committed_room_world_model_v0_1.json",
    "route_to_waypoints",
}

GROUP_ORDER = [
    "rooms",
    "topology_edges",
    "gateways",
    "vertical_transitions",
    "anchors",
    "objects",
    "route",
    "waypoints",
    "room_labels",
]

GROUP_FALLBACK_COLORS = {
    "rooms": (65, 145, 220, 230),
    "topology_edges": (150, 165, 175, 210),
    "gateways": (242, 140, 46, 240),
    "vertical_transitions": (185, 82, 240, 240),
    "anchors": (55, 170, 105, 220),
    "objects": (225, 110, 105, 210),
    "route": (255, 55, 35, 245),
    "waypoints": (255, 185, 35, 250),
    "room_labels": (240, 240, 240, 255),
}


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def file_sha256(path: Path) -> Optional[str]:
    if not path.exists() or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return payload


def inventory_step11_outputs() -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    summary: Dict[str, Any] = {"scenes": {}, "canonical_inputs_complete": True}
    canonical_names = {"marker_specs.json", "route_waypoints_preview.json"}

    for scene in SCENES:
        scene_root = STEP11_ROOT / scene
        scene_summary = {
            "canonical_marker_specs": str(scene_root / "marker_specs.json"),
            "canonical_route_waypoints_preview": str(scene_root / "route_waypoints_preview.json"),
            "legacy_nested_outputs": [],
            "canonical_missing": [],
        }
        for name in sorted(canonical_names):
            if not (scene_root / name).exists():
                scene_summary["canonical_missing"].append(name)
                summary["canonical_inputs_complete"] = False

        if scene_root.exists():
            for path in sorted(item for item in scene_root.rglob("*") if item.is_file()):
                rel = path.relative_to(scene_root)
                is_direct = len(rel.parts) == 1
                is_canonical = is_direct and path.name in canonical_names
                is_nested_duplicate_shape = len(rel.parts) > 1 and rel.parts[0] == scene
                canonical_peer = scene_root / path.name
                row = {
                    "scene_id": scene,
                    "relative_path": str(rel),
                    "path": str(path),
                    "bytes": path.stat().st_size,
                    "sha256": file_sha256(path),
                    "role": "canonical_step12_input" if is_canonical else "legacy_or_supporting_output",
                    "direct_scene_file": is_direct,
                    "nested_duplicate_shape": is_nested_duplicate_shape,
                    "matches_direct_peer": (
                        file_sha256(path) == file_sha256(canonical_peer)
                        if is_nested_duplicate_shape and canonical_peer.exists()
                        else None
                    ),
                }
                rows.append(row)
                if is_nested_duplicate_shape:
                    scene_summary["legacy_nested_outputs"].append(row)
        summary["scenes"][scene] = scene_summary
    return rows, summary


def detect_environment() -> Dict[str, Any]:
    modules = ["rclpy", "visualization_msgs", "geometry_msgs", "std_msgs", "nav_msgs"]
    return {
        "cwd": str(REPO_ROOT),
        "python": sys.executable,
        "ros2_path": shutil.which("ros2"),
        "rviz2_path": shutil.which("rviz2"),
        "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
        "module_available": {name: importlib.util.find_spec(name) is not None for name in modules},
        "ros2_available": shutil.which("ros2") is not None,
        "rviz2_available": shutil.which("rviz2") is not None,
    }


def flatten_markers(marker_specs: Mapping[str, Any]) -> List[Tuple[str, Dict[str, Any]]]:
    groups = marker_specs.get("groups") or {}
    result: List[Tuple[str, Dict[str, Any]]] = []
    if not isinstance(groups, Mapping):
        return result
    for group in GROUP_ORDER:
        for item in groups.get(group) or []:
            if isinstance(item, Mapping):
                result.append((group, dict(item)))
    for group, items in groups.items():
        if group in GROUP_ORDER:
            continue
        for item in items or []:
            if isinstance(item, Mapping):
                result.append((str(group), dict(item)))
    return result


def marker_points(marker: Mapping[str, Any]) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    raw_points = marker.get("points")
    if isinstance(raw_points, list):
        for item in raw_points:
            if isinstance(item, Mapping) and "x" in item and "y" in item:
                points.append((float(item["x"]), float(item["y"])))
    position = marker.get("position")
    if isinstance(position, Mapping) and "x" in position and "y" in position:
        points.append((float(position["x"]), float(position["y"])))
    return points


def rgba(marker: Mapping[str, Any], group: str) -> Tuple[int, int, int, int]:
    color = marker.get("color")
    if isinstance(color, Mapping):
        return (
            int(max(0, min(255, float(color.get("r", 1.0)) * 255))),
            int(max(0, min(255, float(color.get("g", 1.0)) * 255))),
            int(max(0, min(255, float(color.get("b", 1.0)) * 255))),
            int(max(0, min(255, float(color.get("a", 1.0)) * 255))),
        )
    return GROUP_FALLBACK_COLORS.get(group, (220, 220, 220, 220))


def color_css(color: Tuple[int, int, int, int]) -> str:
    return f"rgba({color[0]},{color[1]},{color[2]},{color[3] / 255.0:.3f})"


def bounds_for(markers: Sequence[Tuple[str, Dict[str, Any]]]) -> Tuple[float, float, float, float]:
    points = [point for _, marker in markers for point in marker_points(marker)]
    if not points:
        return -1.0, -1.0, 1.0, 1.0
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad = max(max_x - min_x, max_y - min_y, 1.0) * 0.08
    return min_x - pad, min_y - pad, max_x + pad, max_y + pad


def make_transform(bounds: Tuple[float, float, float, float], width: int, height: int):
    min_x, min_y, max_x, max_y = bounds
    span_x = max(max_x - min_x, 1.0e-6)
    span_y = max(max_y - min_y, 1.0e-6)
    scale = min((width - 80) / span_x, (height - 80) / span_y)
    offset_x = (width - span_x * scale) / 2
    offset_y = (height - span_y * scale) / 2

    def tx(point: Tuple[float, float]) -> Tuple[float, float]:
        x = offset_x + (point[0] - min_x) * scale
        y = height - (offset_y + (point[1] - min_y) * scale)
        return x, y

    return tx


def render_svg(scene: str, marker_specs: Mapping[str, Any], out_path: Path) -> Dict[str, Any]:
    width, height = 1400, 1000
    markers = flatten_markers(marker_specs)
    bounds = bounds_for(markers)
    tx = make_transform(bounds, width, height)
    lines: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#101318"/>',
        f'<text x="28" y="40" fill="#f2f4f8" font-family="monospace" font-size="22">{html.escape(scene)} committed marker_specs preview</text>',
        '<text x="28" y="70" fill="#aeb7c2" font-family="monospace" font-size="14">Fallback static preview; visualization only; not collision-free navigation.</text>',
    ]
    rendered = 0
    for group, marker in markers:
        marker_type = str(marker.get("type") or "").upper()
        color = color_css(rgba(marker, group))
        points = marker_points(marker)
        if marker_type in {"LINE_STRIP", "LINE_LIST"} and len(points) >= 2:
            screen = [tx(point) for point in points]
            if marker_type == "LINE_LIST":
                for first, second in zip(screen[0::2], screen[1::2]):
                    lines.append(
                        f'<line x1="{first[0]:.2f}" y1="{first[1]:.2f}" x2="{second[0]:.2f}" y2="{second[1]:.2f}" '
                        f'stroke="{color}" stroke-width="2.2" stroke-linecap="round"/>'
                    )
                    rendered += 1
            else:
                path_points = " ".join(f"{x:.2f},{y:.2f}" for x, y in screen)
                lines.append(
                    f'<polyline points="{path_points}" fill="none" stroke="{color}" stroke-width="2.4" '
                    'stroke-linejoin="round" stroke-linecap="round"/>'
                )
                rendered += 1
        elif points:
            x, y = tx(points[0])
            radius = 6.0
            if group in {"waypoints", "gateways", "vertical_transitions"}:
                radius = 8.0
            elif group == "objects":
                radius = 3.0
            shape = "rect" if marker_type == "CUBE" else "circle"
            if shape == "rect":
                lines.append(
                    f'<rect x="{x - radius:.2f}" y="{y - radius:.2f}" width="{radius * 2:.2f}" height="{radius * 2:.2f}" '
                    f'fill="{color}" stroke="#111" stroke-width="0.7"/>'
                )
            else:
                lines.append(
                    f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius:.2f}" fill="{color}" stroke="#111" stroke-width="0.7"/>'
                )
            rendered += 1
            if group in {"waypoints", "room_labels"} and marker.get("label"):
                label = html.escape(str(marker.get("label"))[:48])
                lines.append(f'<text x="{x + 9:.2f}" y="{y - 7:.2f}" fill="#f8f8f2" font-family="monospace" font-size="11">{label}</text>')
    legend_y = 104
    counts = marker_specs.get("counts") or {}
    for group in GROUP_ORDER:
        count = int(counts.get(group, 0) or 0) if isinstance(counts, Mapping) else 0
        if count <= 0:
            continue
        color = color_css(GROUP_FALLBACK_COLORS.get(group, (220, 220, 220, 220)))
        lines.append(f'<circle cx="32" cy="{legend_y}" r="6" fill="{color}"/>')
        lines.append(f'<text x="46" y="{legend_y + 4}" fill="#cbd3dd" font-family="monospace" font-size="13">{html.escape(group)}: {count}</text>')
        legend_y += 20
    lines.append("</svg>")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "path": str(out_path),
        "bytes": out_path.stat().st_size,
        "sha256": file_sha256(out_path),
        "rendered_primitives": rendered,
        "bounds_xy": {"min_x": bounds[0], "min_y": bounds[1], "max_x": bounds[2], "max_y": bounds[3]},
    }


def blend_pixel(dst: Tuple[int, int, int, int], src: Tuple[int, int, int, int]) -> Tuple[int, int, int, int]:
    alpha = src[3] / 255.0
    inv = 1.0 - alpha
    return (
        int(src[0] * alpha + dst[0] * inv),
        int(src[1] * alpha + dst[1] * inv),
        int(src[2] * alpha + dst[2] * inv),
        255,
    )


def set_pixel(img: List[List[Tuple[int, int, int, int]]], x: int, y: int, color: Tuple[int, int, int, int]) -> None:
    if 0 <= y < len(img) and 0 <= x < len(img[0]):
        img[y][x] = blend_pixel(img[y][x], color)


def draw_line(img: List[List[Tuple[int, int, int, int]]], p0: Tuple[float, float], p1: Tuple[float, float], color: Tuple[int, int, int, int], width: int = 2) -> None:
    x0, y0 = int(round(p0[0])), int(round(p0[1]))
    x1, y1 = int(round(p1[0])), int(round(p1[1]))
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        for ox in range(-width, width + 1):
            for oy in range(-width, width + 1):
                if ox * ox + oy * oy <= width * width:
                    set_pixel(img, x0 + ox, y0 + oy, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def draw_circle(img: List[List[Tuple[int, int, int, int]]], center: Tuple[float, float], radius: int, color: Tuple[int, int, int, int]) -> None:
    cx, cy = int(round(center[0])), int(round(center[1]))
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                set_pixel(img, x, y, color)


def png_chunk(tag: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)


def write_png(path: Path, img: List[List[Tuple[int, int, int, int]]]) -> None:
    height = len(img)
    width = len(img[0])
    raw = bytearray()
    for row in img:
        raw.append(0)
        for r, g, b, a in row:
            raw.extend((r, g, b, a))
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + png_chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def render_png(scene: str, marker_specs: Mapping[str, Any], out_path: Path) -> Dict[str, Any]:
    width, height = 1400, 1000
    background = (16, 19, 24, 255)
    img = [[background for _ in range(width)] for _ in range(height)]
    markers = flatten_markers(marker_specs)
    tx = make_transform(bounds_for(markers), width, height)
    rendered = 0
    for group, marker in markers:
        marker_type = str(marker.get("type") or "").upper()
        points = marker_points(marker)
        color = rgba(marker, group)
        if marker_type in {"LINE_STRIP", "LINE_LIST"} and len(points) >= 2:
            screen = [tx(point) for point in points]
            if marker_type == "LINE_LIST":
                pairs = zip(screen[0::2], screen[1::2])
            else:
                pairs = zip(screen, screen[1:])
            for first, second in pairs:
                draw_line(img, first, second, color, width=2 if group != "route" else 3)
                rendered += 1
        elif points:
            radius = 6
            if group in {"waypoints", "gateways", "vertical_transitions"}:
                radius = 8
            elif group == "objects":
                radius = 3
            draw_circle(img, tx(points[0]), radius, color)
            rendered += 1
    write_png(out_path, img)
    return {
        "path": str(out_path),
        "bytes": out_path.stat().st_size,
        "sha256": file_sha256(out_path),
        "rendered_primitives": rendered,
    }


def render_html(scene: str, marker_specs: Mapping[str, Any], svg_path: Path, png_path: Path, out_path: Path) -> Dict[str, Any]:
    counts = marker_specs.get("counts") or {}
    rows = "\n".join(
        f"<tr><td>{html.escape(str(group))}</td><td>{int(counts.get(group, 0) or 0)}</td></tr>"
        for group in GROUP_ORDER
        if isinstance(counts, Mapping) and int(counts.get(group, 0) or 0) > 0
    )
    body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{html.escape(scene)} Step 12 Marker Preview</title>
  <style>
    body {{ margin: 0; background: #101318; color: #eef2f5; font-family: system-ui, sans-serif; }}
    main {{ max-width: 1440px; margin: 0 auto; padding: 24px; }}
    a {{ color: #8fc8ff; }}
    table {{ border-collapse: collapse; margin: 16px 0; }}
    td, th {{ border: 1px solid #39424e; padding: 6px 10px; }}
    .preview {{ width: 100%; border: 1px solid #39424e; background: #101318; }}
    .note {{ color: #b8c1cc; max-width: 960px; }}
  </style>
</head>
<body>
<main>
  <h1>{html.escape(scene)} Step 12 Marker Preview</h1>
  <p class="note">Static fallback evidence rendered from the canonical Step 11 marker_specs.json. Visualization only; not collision-free navigation, Nav2, Gazebo, BEV, or robot control.</p>
  <p><a href="{html.escape(svg_path.name)}">SVG</a> | <a href="{html.escape(png_path.name)}">PNG</a></p>
  <table><thead><tr><th>Group</th><th>Markers</th></tr></thead><tbody>{rows}</tbody></table>
  <img class="preview" src="{html.escape(svg_path.name)}" alt="{html.escape(scene)} marker preview">
</main>
</body>
</html>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(body, encoding="utf-8")
    return {"path": str(out_path), "bytes": out_path.stat().st_size, "sha256": file_sha256(out_path)}


def provenance_summary(scene: str, marker_specs: Mapping[str, Any], route_waypoints: Mapping[str, Any]) -> Dict[str, Any]:
    markers = flatten_markers(marker_specs)
    source_counts: Dict[str, int] = {}
    forbidden_sources: List[str] = []
    non_visualization_markers: List[str] = []
    collision_claim_markers: List[str] = []
    for _, marker in markers:
        source = str(marker.get("source_artifact") or "")
        source_counts[source] = source_counts.get(source, 0) + 1
        if source not in ALLOWED_SOURCE_ARTIFACTS:
            forbidden_sources.append(source)
        if marker.get("visualization_only") is not True:
            non_visualization_markers.append(str(marker.get("marker_id") or ""))
        if marker.get("not_collision_free") is False and marker.get("group") in {"route", "waypoints", "topology_edges", "gateways", "vertical_transitions", "objects", "anchors"}:
            collision_claim_markers.append(str(marker.get("marker_id") or ""))
    safety = route_waypoints.get("paper_safety_summary") or {}
    return {
        "scene_id": scene,
        "marker_count_total": len(markers),
        "marker_group_counts": dict(marker_specs.get("counts") or {}),
        "source_artifact_counts": source_counts,
        "all_marker_sources_allowed": not forbidden_sources,
        "forbidden_sources": sorted(set(forbidden_sources)),
        "all_markers_visualization_only": not non_visualization_markers,
        "non_visualization_markers": non_visualization_markers[:25],
        "route_waypoints_visualization_only": safety.get("visualization_only") is True,
        "route_waypoints_not_collision_free": safety.get("not_collision_free") is True,
        "route_waypoint_count": len(route_waypoints.get("waypoints") or []),
        "route_segment_count": len(route_waypoints.get("segments") or []),
        "route_warnings": list(route_waypoints.get("warnings") or []),
        "collision_claim_markers": collision_claim_markers[:25],
        "passed": not forbidden_sources
        and not non_visualization_markers
        and safety.get("visualization_only") is True
        and safety.get("not_collision_free") is True,
    }


def write_inventory_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scene_id",
        "relative_path",
        "path",
        "bytes",
        "sha256",
        "role",
        "direct_scene_file",
        "nested_duplicate_shape",
        "matches_direct_peer",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_summary_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scene_id",
        "ros2_available",
        "rviz2_available",
        "smoke_mode",
        "marker_count_total",
        "route_waypoint_count",
        "route_segment_count",
        "sources_allowed",
        "route_not_collision_free",
        "route_visualization_only",
        "html_preview",
        "svg_preview",
        "png_preview",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def main() -> int:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    inventory_rows, inventory_summary = inventory_step11_outputs()
    environment = detect_environment()
    write_json(OUT_ROOT / "environment_detection.json", environment)
    write_json(OUT_ROOT / "step11_output_inventory.json", {"summary": inventory_summary, "files": inventory_rows})
    write_inventory_csv(OUT_ROOT / "step11_output_inventory.csv", inventory_rows)

    scene_rows: List[Dict[str, Any]] = []
    provenance_rows: List[Dict[str, Any]] = []
    for scene in SCENES:
        scene_in = STEP11_ROOT / scene
        marker_specs_path = scene_in / "marker_specs.json"
        route_waypoints_path = scene_in / "route_waypoints_preview.json"
        marker_specs = load_json(marker_specs_path)
        route_waypoints = load_json(route_waypoints_path)

        scene_out = OUT_ROOT / "fallback_previews" / scene
        svg_path = scene_out / f"{scene}_marker_preview.svg"
        png_path = scene_out / f"{scene}_marker_preview.png"
        html_path = scene_out / f"{scene}_marker_preview.html"
        svg_result = render_svg(scene, marker_specs, svg_path)
        png_result = render_png(scene, marker_specs, png_path)
        html_result = render_html(scene, marker_specs, svg_path, png_path, html_path)
        provenance = provenance_summary(scene, marker_specs, route_waypoints)
        provenance["canonical_inputs"] = {
            "marker_specs": str(marker_specs_path),
            "route_waypoints_preview": str(route_waypoints_path),
            "marker_specs_sha256": file_sha256(marker_specs_path),
            "route_waypoints_preview_sha256": file_sha256(route_waypoints_path),
        }
        provenance["preview_outputs"] = {"svg": svg_result, "png": png_result, "html": html_result}
        write_json(scene_out / "preview_provenance.json", provenance)
        provenance_rows.append(provenance)
        scene_rows.append(
            {
                "scene_id": scene,
                "ros2_available": environment["ros2_available"],
                "rviz2_available": environment["rviz2_available"],
                "smoke_mode": "non_ros_fallback_static_preview",
                "marker_count_total": provenance["marker_count_total"],
                "route_waypoint_count": provenance["route_waypoint_count"],
                "route_segment_count": provenance["route_segment_count"],
                "sources_allowed": provenance["all_marker_sources_allowed"],
                "route_not_collision_free": provenance["route_waypoints_not_collision_free"],
                "route_visualization_only": provenance["route_waypoints_visualization_only"],
                "html_preview": html_result["path"],
                "svg_preview": svg_result["path"],
                "png_preview": png_result["path"],
            }
        )

    report = {
        "step": 12,
        "goal": "ROS/RViz artifact marker bridge smoke test and demo-evidence packaging",
        "environment": environment,
        "mode": "non_ros_fallback_static_preview" if not environment["ros2_available"] else "ros_available_not_launched_by_this_script",
        "strict_boundaries": {
            "gazebo_launched": False,
            "rviz_launched": False,
            "nav2_launched": False,
            "cmd_vel_published": False,
            "robot_control_added": False,
            "stage_a_runtime_modified": False,
            "artifact_schemas_modified": False,
        },
        "canonical_inputs": {
            scene: {
                "marker_specs": str(STEP11_ROOT / scene / "marker_specs.json"),
                "route_waypoints_preview": str(STEP11_ROOT / scene / "route_waypoints_preview.json"),
            }
            for scene in SCENES
        },
        "inventory_summary": inventory_summary,
        "scene_results": provenance_rows,
    }
    write_json(OUT_ROOT / "step12_ros_rviz_smoke_test_report.json", report)
    write_summary_csv(OUT_ROOT / "step12_ros_rviz_smoke_test_summary.csv", scene_rows)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
