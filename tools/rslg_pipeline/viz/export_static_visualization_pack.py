#!/usr/bin/env python3
"""Export a static RSLG-SLAM visualization pack from RouteResult artifacts.

This is a Layer 4 presentation surface for the current formal static chain:

    frozen canonical Layer 1/2 artifacts + QueryTask
    -> RSLGRouteResult
    -> RouteResult-derived Layer 4 adapter inputs
    -> static visualization pack

It reads RouteResult JSON files and, when available, RouteResult-derived PID,
RViz marker, and z-aware overlay inputs. It writes a self-contained HTML index,
Markdown index, JSON summary, and one SVG plus Markdown note per query.

The exporter is intentionally ROS-free and GPU-free. It does not launch RViz,
Gazebo, Nav2, AMCL, map_server, or any live robot process.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_ID,
    MAIN_SCENE_ID,
    NON_TRANSITION_EDGE,
    OBJECT_FLOOR,
    OBJECT_ID,
    OBJECT_LABEL,
    OBJECT_ROOM,
    PROJECT_NAME,
    TRUE_TRANSITION_EDGE,
)

ROUTE_RESULT_SCHEMA_NAME = "rslg_route_result"
DEFAULT_FLOOR_Z_MAP = {"floor_1": 0.0, "floor_2": 1.6}

PID_SUBDIR = "pid_follower_inputs"
MARKER_SUBDIR = "rviz_marker_inputs"
Z_AWARE_SUBDIR = "z_aware_overlay_inputs"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def parse_floor_z_map(raw: Optional[str]) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(k): float(v) for k, v in json.loads(raw).items()}


def route_result_paths(route_results_dir: Path) -> list[Path]:
    paths = sorted(route_results_dir.glob("*_route_result.json"))
    if paths:
        return paths
    return sorted(
        path
        for path in route_results_dir.glob("*.json")
        if path.name not in {"batch_summary.json", "per_query_index.json"}
    )


def query_id(route_result: dict[str, Any], fallback: str) -> str:
    return str((route_result.get("identity") or {}).get("query_id") or fallback)


def adapter_path(adapter_root: Optional[Path], subdir: str, qid: str, suffix: str) -> Optional[Path]:
    if adapter_root is None:
        return None
    path = adapter_root / subdir / f"{qid}_{suffix}.json"
    return path if path.exists() else None


def load_adapter(adapter_root: Optional[Path], subdir: str, qid: str, suffix: str) -> tuple[Optional[Path], dict[str, Any]]:
    path = adapter_path(adapter_root, subdir, qid, suffix)
    if path is None:
        return None, {}
    return path, read_json(path)


def selected_approach(route_result: dict[str, Any], pid_input: dict[str, Any]) -> dict[str, Any]:
    selected = (route_result.get("approach") or {}).get("selected_approach")
    if isinstance(selected, dict) and selected:
        return dict(selected)
    goal = pid_input.get("selected_goal")
    return dict(goal) if isinstance(goal, dict) else {}


def rejected_candidates(route_result: dict[str, Any], pid_input: dict[str, Any]) -> list[dict[str, Any]]:
    rejected: list[dict[str, Any]] = []
    for item in (route_result.get("approach") or {}).get("rejected_candidates") or []:
        if isinstance(item, dict):
            rejected.append(dict(item))
    seen = {item.get("candidate_id") for item in rejected}
    for item in pid_input.get("rejected_runtime_candidates") or []:
        if isinstance(item, dict) and item.get("candidate_id") not in seen:
            rejected.append(dict(item))
    return rejected


def blocked_evidence_records(rejected: list[dict[str, Any]]) -> list[dict[str, Any]]:
    blocked_ids = set(BLOCKED_LEGACY_APPROACH_IDS)
    return [item for item in rejected if item.get("candidate_id") in blocked_ids]


def candidate_by_id(route_result: dict[str, Any], candidate_id: str) -> dict[str, Any]:
    for item in (route_result.get("approach") or {}).get("approach_candidates") or []:
        if isinstance(item, dict) and item.get("candidate_id") == candidate_id:
            return dict(item)
    return {}


def route_points(route_result: dict[str, Any], z_overlay: dict[str, Any]) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    for segment in route_result.get("route_segments") or []:
        if not isinstance(segment, dict):
            continue
        floor_id = segment.get("floor_id")
        segment_id = segment.get("segment_id")
        segment_type = segment.get("segment_type")
        for point in segment.get("waypoints") or []:
            if isinstance(point, dict) and "x" in point and "y" in point:
                points.append(
                    {
                        "x": float(point["x"]),
                        "y": float(point["y"]),
                        "floor_id": floor_id,
                        "segment_id": segment_id,
                        "segment_type": segment_type,
                    }
                )
    if points:
        return points
    for point in z_overlay.get("overlay_waypoints") or []:
        if isinstance(point, dict) and "x" in point and "y" in point:
            points.append(
                {
                    "x": float(point["x"]),
                    "y": float(point["y"]),
                    "floor_id": point.get("floor_id"),
                    "segment_id": point.get("segment_id"),
                    "segment_type": "z_aware_overlay",
                }
            )
    return points


def segment_polylines(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    lines: list[dict[str, Any]] = []
    for segment in route_result.get("route_segments") or []:
        if not isinstance(segment, dict):
            continue
        points = []
        for point in segment.get("waypoints") or []:
            if isinstance(point, dict) and "x" in point and "y" in point:
                points.append({"x": float(point["x"]), "y": float(point["y"])})
        if len(points) >= 2:
            lines.append(
                {
                    "points": points,
                    "floor_id": segment.get("floor_id"),
                    "segment_id": segment.get("segment_id"),
                    "segment_type": segment.get("segment_type"),
                }
            )
    return lines


def connector_segments(route_result: dict[str, Any]) -> list[dict[str, Any]]:
    connectors_by_id: dict[str, dict[str, Any]] = {}
    for connector in (route_result.get("semantic_route") or {}).get("connector_sequence") or []:
        if isinstance(connector, dict) and connector.get("connector_id"):
            connectors_by_id[str(connector["connector_id"])] = connector

    segments: list[dict[str, Any]] = []
    for segment in route_result.get("route_segments") or []:
        if not isinstance(segment, dict):
            continue
        if segment.get("segment_type") not in {"vertical_transition", "connector_handoff"}:
            continue
        start = segment.get("start")
        end = segment.get("end")
        if not (
            isinstance(start, list)
            and isinstance(end, list)
            and len(start) >= 2
            and len(end) >= 2
        ):
            continue
        connector = connectors_by_id.get(str(segment.get("connector_id"))) or {}
        segments.append(
            {
                "start": {"x": float(start[0]), "y": float(start[1])},
                "end": {"x": float(end[0]), "y": float(end[1])},
                "connector_id": segment.get("connector_id"),
                "segment_id": segment.get("segment_id"),
                "transition_edge": segment.get("transition_edge") or connector.get("transition_edge"),
                "non_transition_edge": connector.get("non_transition_edge"),
                "from_floor": connector.get("from_floor"),
                "to_floor": connector.get("to_floor"),
            }
        )
    return segments


def floor_color(floor_id: Any) -> str:
    if floor_id == "floor_1":
        return "#1976b9"
    if floor_id == "floor_2":
        return "#d86f1d"
    return "#367c69"


def svg_escape(text: Any) -> str:
    return html.escape("" if text is None else str(text), quote=True)


class ViewBox:
    def __init__(self, points: list[dict[str, float]], width: int = 980, height: int = 620) -> None:
        self.width = width
        self.height = height
        self.pad = 58
        xs = [float(p["x"]) for p in points if "x" in p]
        ys = [float(p["y"]) for p in points if "y" in p]
        if not xs:
            xs = [0.0, 1.0]
        if not ys:
            ys = [0.0, 1.0]
        self.min_x = min(xs)
        self.max_x = max(xs)
        self.min_y = min(ys)
        self.max_y = max(ys)
        if abs(self.max_x - self.min_x) < 1e-6:
            self.min_x -= 1.0
            self.max_x += 1.0
        if abs(self.max_y - self.min_y) < 1e-6:
            self.min_y -= 1.0
            self.max_y += 1.0
        self.scale = min(
            (self.width - 2 * self.pad) / (self.max_x - self.min_x),
            (self.height - 2 * self.pad) / (self.max_y - self.min_y),
        )

    def xy(self, x: float, y: float) -> tuple[float, float]:
        sx = self.pad + (x - self.min_x) * self.scale
        sy = self.height - self.pad - (y - self.min_y) * self.scale
        return sx, sy


def polyline(points: list[dict[str, float]], view: ViewBox) -> str:
    return " ".join(f"{view.xy(p['x'], p['y'])[0]:.2f},{view.xy(p['x'], p['y'])[1]:.2f}" for p in points)


def point_from_candidate(candidate: dict[str, Any]) -> Optional[dict[str, float]]:
    world_xy = candidate.get("world_xy")
    if isinstance(world_xy, list) and len(world_xy) >= 2:
        return {"x": float(world_xy[0]), "y": float(world_xy[1])}
    if "x" in candidate and "y" in candidate:
        return {"x": float(candidate["x"]), "y": float(candidate["y"])}
    return None


def arrow_endpoint(point: dict[str, float], yaw: Any, length: float = 0.35) -> Optional[dict[str, float]]:
    try:
        yaw_value = float(yaw)
    except (TypeError, ValueError):
        return None
    return {
        "x": point["x"] + math.cos(yaw_value) * length,
        "y": point["y"] + math.sin(yaw_value) * length,
    }


def render_route_svg(query: dict[str, Any]) -> str:
    qid = query["query_id"]
    identity = query["identity"]
    target = query["target"]
    semantic = query["semantic_route"]
    selected = query["selected_approach"]
    selected_point = point_from_candidate(selected)
    blocked_points = [
        point
        for point in (point_from_candidate(item) for item in query["blocked_evidence"])
        if point is not None
    ]
    connectors = query["connector_segments"]
    all_points = query["route_points"] + blocked_points
    if selected_point is not None:
        all_points.append(selected_point)
    for connector in connectors:
        all_points.extend([connector["start"], connector["end"]])
    view = ViewBox(all_points)

    parts: list[str] = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="980" height="620" viewBox="0 0 980 620" role="img">',
        "<style>",
        ".label{font:13px system-ui,Segoe UI,Arial,sans-serif;fill:#1e2630}",
        ".small{font:11px system-ui,Segoe UI,Arial,sans-serif;fill:#415160}",
        ".title{font:17px system-ui,Segoe UI,Arial,sans-serif;font-weight:700;fill:#111827}",
        ".panel{fill:#f7fafc;stroke:#ccd6df;stroke-width:1}",
        ".grid{stroke:#e7edf2;stroke-width:1}",
        "</style>",
        '<rect x="0" y="0" width="980" height="620" fill="#ffffff"/>',
        '<rect x="18" y="18" width="944" height="584" rx="6" class="panel"/>',
        f'<text x="36" y="48" class="title">{svg_escape(qid)}</text>',
        f'<text x="36" y="72" class="small">{svg_escape(identity.get("query_text"))}</text>',
        '<text x="36" y="96" class="small">Layer 4 static visualization: route/interface presentation only; no live runtime claim.</text>',
    ]

    for gx in range(0, 11):
        x = view.pad + gx * (view.width - 2 * view.pad) / 10
        parts.append(f'<line x1="{x:.2f}" y1="{view.pad}" x2="{x:.2f}" y2="{view.height - view.pad}" class="grid"/>')
    for gy in range(0, 8):
        y = view.pad + gy * (view.height - 2 * view.pad) / 7
        parts.append(f'<line x1="{view.pad}" y1="{y:.2f}" x2="{view.width - view.pad}" y2="{y:.2f}" class="grid"/>')

    for line in query["segment_polylines"]:
        points = line["points"]
        color = floor_color(line.get("floor_id"))
        dash = " stroke-dasharray=\"7 5\"" if line.get("segment_type") in {"object_approach", "object_approach_metric"} else ""
        parts.append(
            f'<polyline points="{polyline(points, view)}" fill="none" stroke="{color}" '
            f'stroke-width="5" stroke-linecap="round" stroke-linejoin="round"{dash}/>'
        )

    for connector in connectors:
        start = view.xy(connector["start"]["x"], connector["start"]["y"])
        end = view.xy(connector["end"]["x"], connector["end"]["y"])
        label_x = (start[0] + end[0]) / 2 + 8
        label_y = (start[1] + end[1]) / 2 - 8
        parts.append(
            f'<line x1="{start[0]:.2f}" y1="{start[1]:.2f}" x2="{end[0]:.2f}" y2="{end[1]:.2f}" '
            'stroke="#7a3db8" stroke-width="5" stroke-dasharray="4 5" stroke-linecap="round"/>'
        )
        parts.append(f'<circle cx="{start[0]:.2f}" cy="{start[1]:.2f}" r="6" fill="#7a3db8"/>')
        parts.append(f'<circle cx="{end[0]:.2f}" cy="{end[1]:.2f}" r="6" fill="#7a3db8"/>')
        parts.append(
            f'<text x="{label_x:.2f}" y="{label_y:.2f}" class="small">'
            f'{svg_escape(connector.get("transition_edge") or TRUE_TRANSITION_EDGE)} visualization-only handoff</text>'
        )

    if selected_point is not None:
        sx, sy = view.xy(selected_point["x"], selected_point["y"])
        parts.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="9" fill="#0c9b54" stroke="#063b22" stroke-width="2"/>')
        parts.append(
            f'<text x="{sx + 12:.2f}" y="{sy - 10:.2f}" class="label">'
            f'{svg_escape(selected.get("candidate_id"))} selected approach</text>'
        )
        arrow = arrow_endpoint(selected_point, selected.get("yaw"))
        if arrow is not None:
            ax, ay = view.xy(arrow["x"], arrow["y"])
            parts.append(
                f'<line x1="{sx:.2f}" y1="{sy:.2f}" x2="{ax:.2f}" y2="{ay:.2f}" '
                'stroke="#0c9b54" stroke-width="3" stroke-linecap="round"/>'
            )

    for item, point in zip(query["blocked_evidence"], blocked_points):
        bx, by = view.xy(point["x"], point["y"])
        parts.append(f'<line x1="{bx - 7:.2f}" y1="{by - 7:.2f}" x2="{bx + 7:.2f}" y2="{by + 7:.2f}" stroke="#c73333" stroke-width="4"/>')
        parts.append(f'<line x1="{bx + 7:.2f}" y1="{by - 7:.2f}" x2="{bx - 7:.2f}" y2="{by + 7:.2f}" stroke="#c73333" stroke-width="4"/>')
        parts.append(f'<text x="{bx + 12:.2f}" y="{by + 4:.2f}" class="label">{svg_escape(item.get("candidate_id"))} blocked evidence</text>')

    legend_x = 688
    legend_y = 112
    parts.extend(
        [
            f'<rect x="{legend_x}" y="{legend_y}" width="244" height="168" rx="6" fill="#ffffff" stroke="#cbd5df"/>',
            f'<text x="{legend_x + 14}" y="{legend_y + 26}" class="label">Semantic route</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 50}" class="small">rooms: {svg_escape(" -> ".join(semantic.get("room_sequence") or []))}</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 72}" class="small">floors: {svg_escape(" -> ".join(semantic.get("floor_sequence") or []))}</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 94}" class="small">target: {svg_escape(target.get("target_object_id") or target.get("target_room_id") or target.get("target_type"))}</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 116}" class="small">selected: {svg_escape(selected.get("candidate_id") or "none")}</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 138}" class="small">blocked evidence: {svg_escape(query["blocked_status_label"])}</text>',
            f'<text x="{legend_x + 14}" y="{legend_y + 160}" class="small">z: floor_1=0.0, floor_2=1.6</text>',
        ]
    )

    parts.extend(
        [
            '<rect x="38" y="552" width="22" height="6" fill="#1976b9"/>',
            '<text x="68" y="560" class="small">floor_1 route</text>',
            '<rect x="172" y="552" width="22" height="6" fill="#d86f1d"/>',
            '<text x="202" y="560" class="small">floor_2 route</text>',
            '<rect x="306" y="552" width="22" height="6" fill="#7a3db8"/>',
            '<text x="336" y="560" class="small">connector handoff, visualization-only</text>',
            '<circle cx="574" cy="555" r="6" fill="#0c9b54"/>',
            '<text x="588" y="560" class="small">selected object approach</text>',
        ]
    )

    if not blocked_points and query["blocked_evidence"]:
        parts.append(
            '<text x="38" y="586" class="small">'
            f'{svg_escape(BLOCKED_LEGACY_APPROACH_IDS[0])} is blocked/rejected evidence only; no runtime-goal pose is drawn.</text>'
        )
    else:
        parts.append('<text x="38" y="586" class="small">Blocked candidates are never drawn as selected goals.</text>')

    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def route_markdown(query: dict[str, Any]) -> str:
    selected = query["selected_approach"]
    target = query["target"]
    semantic = query["semantic_route"]
    connector_rows = []
    for connector in semantic.get("connector_sequence") or []:
        connector_rows.append(
            f"- `{connector.get('connector_id')}`: `{connector.get('from_floor')}` -> `{connector.get('to_floor')}`, "
            f"transition `{connector.get('transition_edge')}`, non-transition `{connector.get('non_transition_edge')}`"
        )
    if not connector_rows:
        connector_rows.append("- none")

    blocked_lines = []
    for item in query["blocked_evidence"]:
        blocked_lines.append(
            f"- `{item.get('candidate_id')}`: `{item.get('status')}`, "
            f"is_runtime_goal=`{item.get('is_runtime_goal')}`, reason={item.get('reason')}"
        )
    if not blocked_lines:
        blocked_lines.append("- none in this query")

    lines = [
        f"# {query['query_id']}",
        "",
        f"- query_text: {query['identity'].get('query_text')}",
        f"- query_type: `{query['identity'].get('query_type')}`",
        f"- target: `{target.get('target_type')}` / `{target.get('target_object_id') or target.get('target_room_id')}`",
        f"- room_sequence: `{' -> '.join(semantic.get('room_sequence') or [])}`",
        f"- floor_sequence: `{' -> '.join(semantic.get('floor_sequence') or [])}`",
        f"- selected_approach: `{selected.get('candidate_id')}`",
        f"- selected_approach_xy: `{selected.get('world_xy')}`",
        f"- selected_approach_yaw: `{selected.get('yaw')}`",
        f"- selected_approach_clearance_m: `{selected.get('clearance_m')}`",
        f"- svg: `{query['svg_path']}`",
        "",
        "## Connector Handoff",
        "",
        *connector_rows,
        "",
        f"`{TRUE_TRANSITION_EDGE}` is the transition edge when present. `{NON_TRANSITION_EDGE}` is recorded only as a forbidden/non-transition edge.",
        "The connector handoff is visualization-only and is not a physical stair-climbing claim.",
        "",
        "## Blocked Evidence",
        "",
        *blocked_lines,
        "",
        f"`{BLOCKED_LEGACY_APPROACH_IDS[0]}` must never be selected as a runtime goal.",
        "",
        "## Claim Boundary",
        "",
        "This static surface does not claim Nav2, AMCL, map_server, physical robot deployment, physical stair climbing, or collision-free navigation.",
        "",
    ]
    return "\n".join(lines)


def summarize_query(
    route_path: Path,
    route_result: dict[str, Any],
    adapter_root: Optional[Path],
    output_dir: Path,
    floor_z_map: dict[str, float],
) -> dict[str, Any]:
    qid = query_id(route_result, route_path.stem.replace("_route_result", ""))
    pid_path, pid_input = load_adapter(adapter_root, PID_SUBDIR, qid, "pid_runtime_input")
    marker_path, marker_input = load_adapter(adapter_root, MARKER_SUBDIR, qid, "rviz_marker_input")
    z_path, z_input = load_adapter(adapter_root, Z_AWARE_SUBDIR, qid, "z_aware_overlay_input")
    selected = selected_approach(route_result, pid_input)
    selected_candidate = candidate_by_id(route_result, selected.get("candidate_id", ""))
    if selected_candidate and "a_star_waypoints" in selected_candidate:
        selected["a_star_waypoints"] = selected_candidate["a_star_waypoints"]
    rejected = rejected_candidates(route_result, pid_input)
    blocked = blocked_evidence_records(rejected)
    semantic = route_result.get("semantic_route") or {}
    connectors = semantic.get("connector_sequence") or []
    transition_edges = [c.get("transition_edge") for c in connectors if isinstance(c, dict) and c.get("transition_edge")]
    non_transition_edges = [
        c.get("non_transition_edge") for c in connectors if isinstance(c, dict) and c.get("non_transition_edge")
    ]

    svg_path = output_dir / "routes" / f"{qid}.svg"
    md_path = output_dir / "routes" / f"{qid}.md"
    query = {
        "query_id": qid,
        "identity": route_result.get("identity") or {},
        "target": route_result.get("target") or {},
        "semantic_route": semantic,
        "selected_approach": selected,
        "rejected_candidates": rejected,
        "blocked_evidence": blocked,
        "blocked_status_label": "present" if blocked else "none",
        "route_points": route_points(route_result, z_input),
        "segment_polylines": segment_polylines(route_result),
        "connector_segments": connector_segments(route_result),
        "route_result_path": route_path.as_posix(),
        "pid_input_path": pid_path.as_posix() if pid_path else None,
        "rviz_marker_input_path": marker_path.as_posix() if marker_path else None,
        "z_aware_overlay_input_path": z_path.as_posix() if z_path else None,
        "svg_path": svg_path.as_posix(),
        "markdown_path": md_path.as_posix(),
        "rviz_marker_count": marker_input.get("marker_count"),
        "z_aware_overlay_waypoint_count": z_input.get("overlay_waypoint_count"),
        "z_min": z_input.get("z_min"),
        "z_max": z_input.get("z_max"),
        "floor_z_map": floor_z_map,
        "selected_generated_ring_002": selected.get("candidate_id") == CURRENT_OBJECT_APPROACH_ID,
        "blocked_generated_ring_037_present": bool(blocked),
        "blocked_generated_ring_037_selected": selected.get("candidate_id") in BLOCKED_LEGACY_APPROACH_IDS,
        "vt_1_centerline_e001_transition": TRUE_TRANSITION_EDGE in transition_edges
        or z_input.get("transition_edge_used") == TRUE_TRANSITION_EDGE,
        "vt_1_centerline_e003_non_transition": NON_TRANSITION_EDGE in non_transition_edges
        or z_input.get("non_transition_edge") == NON_TRANSITION_EDGE,
        "vt_1_centerline_e003_transition": NON_TRANSITION_EDGE in transition_edges
        or z_input.get("transition_edge_used") == NON_TRANSITION_EDGE,
    }
    write_text(svg_path, render_route_svg(query))
    write_text(md_path, route_markdown(query))
    return query


def html_card(query: dict[str, Any], output_dir: Path) -> str:
    svg_rel = Path(query["svg_path"]).relative_to(output_dir).as_posix()
    md_rel = Path(query["markdown_path"]).relative_to(output_dir).as_posix()
    identity = query["identity"]
    target = query["target"]
    selected = query["selected_approach"]
    semantic = query["semantic_route"]
    return f"""
<article class="route-card">
  <header>
    <h2>{html.escape(query['query_id'])}</h2>
    <p>{html.escape(str(identity.get('query_text') or ''))}</p>
  </header>
  <div class="meta-grid">
    <div><span>Query</span><strong>{html.escape(str(identity.get('query_type')))}</strong></div>
    <div><span>Target</span><strong>{html.escape(str(target.get('target_object_id') or target.get('target_room_id') or target.get('target_type')))}</strong></div>
    <div><span>Selected approach</span><strong>{html.escape(str(selected.get('candidate_id') or 'none'))}</strong></div>
    <div><span>Blocked evidence</span><strong>{html.escape(query['blocked_status_label'])}</strong></div>
  </div>
  <img src="{html.escape(svg_rel)}" alt="Static route visualization for {html.escape(query['query_id'])}">
  <p class="sequence">Rooms: {html.escape(' -> '.join(semantic.get('room_sequence') or []))}</p>
  <p class="sequence">Floors: {html.escape(' -> '.join(semantic.get('floor_sequence') or []))}</p>
  <p><a href="{html.escape(md_rel)}">Per-query Markdown details</a></p>
</article>
"""


def render_index_html(title: str, output_dir: Path, summary: dict[str, Any], queries: list[dict[str, Any]]) -> str:
    cards = "\n".join(html_card(query, output_dir) for query in queries)
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      color-scheme: light;
      --ink: #17212b;
      --muted: #5c6977;
      --line: #cdd8e3;
      --surface: #f6f9fb;
      --blue: #1976b9;
      --orange: #d86f1d;
      --green: #0c9b54;
      --purple: #7a3db8;
    }}
    body {{
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: #ffffff;
    }}
    header.page {{
      padding: 28px 34px 18px;
      border-bottom: 1px solid var(--line);
      background: var(--surface);
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 30px;
      line-height: 1.15;
      letter-spacing: 0;
    }}
    h2 {{
      margin: 0 0 6px;
      font-size: 18px;
      line-height: 1.25;
      letter-spacing: 0;
    }}
    p {{
      margin: 0 0 10px;
      color: var(--muted);
      line-height: 1.45;
    }}
    main {{
      max-width: 1180px;
      margin: 0 auto;
      padding: 24px;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(178px, 1fr));
      gap: 10px;
      margin-bottom: 20px;
    }}
    .stat, .boundary {{
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 12px;
      background: #fff;
    }}
    .stat span, .meta-grid span {{
      display: block;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 2px;
    }}
    .stat strong, .meta-grid strong {{
      font-size: 15px;
      word-break: break-word;
    }}
    .route-card {{
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 16px;
      margin: 18px 0;
      background: #fff;
    }}
    .route-card img {{
      display: block;
      width: 100%;
      max-width: 980px;
      height: auto;
      border: 1px solid var(--line);
      border-radius: 4px;
      margin: 12px 0;
    }}
    .meta-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 8px;
      margin: 12px 0;
    }}
    .meta-grid div {{
      border-left: 4px solid var(--blue);
      padding: 8px 10px;
      background: var(--surface);
    }}
    .sequence {{
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      color: #273340;
      overflow-wrap: anywhere;
    }}
    a {{ color: #145f92; }}
  </style>
</head>
<body>
  <header class="page">
    <h1>{html.escape(title)}</h1>
    <p>Static Layer 4 visualization surface for {PROJECT_NAME}: semantic-topological route presentation, object approach evidence, RViz marker input references, and z-aware connector overlays.</p>
  </header>
  <main>
    <section class="stats">
      <div class="stat"><span>QueryTasks visualized</span><strong>{summary['query_count']}</strong></div>
      <div class="stat"><span>RViz marker inputs</span><strong>{summary['rviz_marker_input_count']}</strong></div>
      <div class="stat"><span>Z-aware overlays</span><strong>{summary['z_aware_overlay_input_count']}</strong></div>
      <div class="stat"><span>Floor z range</span><strong>{summary['floor_z_min']} to {summary['floor_z_max']}</strong></div>
    </section>
    <section class="boundary">
      <p>This pack is generated from task-local RouteResults and RouteResult-derived adapter inputs. It does not run Stage-A, raw RGB-D inference, ROS, RViz, Gazebo, Nav2, AMCL, map_server, physical robot code, or any live navigation action.</p>
      <p>{BLOCKED_LEGACY_APPROACH_IDS[0]} is shown only as blocked/rejected evidence. {TRUE_TRANSITION_EDGE} is the transition edge; {NON_TRANSITION_EDGE} is only the forbidden/non-transition edge.</p>
    </section>
    {cards}
  </main>
</body>
</html>
"""


def render_index_md(title: str, summary: dict[str, Any], queries: list[dict[str, Any]]) -> str:
    lines = [
        f"# {title}",
        "",
        f"- project: **{PROJECT_NAME}**",
        f"- scene_id: `{summary['scene_id']}`",
        f"- generated_utc: `{summary['generated_utc']}`",
        f"- query_tasks_visualized: {summary['query_count']}",
        f"- static_index_html: `{summary['index_html']}`",
        f"- static_index_md: `{summary['index_md']}`",
        f"- rviz_marker_input_count: {summary['rviz_marker_input_count']}",
        f"- z_aware_overlay_input_count: {summary['z_aware_overlay_input_count']}",
        f"- floor_z_range: `{summary['floor_z_min']}` to `{summary['floor_z_max']}`",
        "",
        "This is a static Layer 4 presentation surface. It does not launch RViz, Gazebo, Nav2, AMCL, map_server, or physical robot code.",
        "",
        "## Per Query",
        "",
        "| query_id | query_type | svg | markdown | selected | blocked evidence | transition | non-transition |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for query in queries:
        identity = query["identity"]
        lines.append(
            f"| `{query['query_id']}` | `{identity.get('query_type')}` | `{query['svg_path']}` | "
            f"`{query['markdown_path']}` | `{query['selected_approach'].get('candidate_id')}` | "
            f"`{query['blocked_status_label']}` | `{TRUE_TRANSITION_EDGE if query['vt_1_centerline_e001_transition'] else 'none'}` | "
            f"`{NON_TRANSITION_EDGE if query['vt_1_centerline_e003_non_transition'] else 'none'}` |"
        )
    lines.extend(
        [
            "",
            "## Claim Boundary",
            "",
            f"- `{CURRENT_OBJECT_APPROACH_ID}` is the selected valid object approach when an object approach is required.",
            f"- `{BLOCKED_LEGACY_APPROACH_IDS[0]}` remains blocked/rejected evidence only and is never selected.",
            f"- `{TRUE_TRANSITION_EDGE}` is the true transition edge when a floor transition is present.",
            f"- `{NON_TRANSITION_EDGE}` remains the forbidden/non-transition edge and is never used as a transition.",
            "- Floor z values are visualization-only: `floor_1=0.0`, `floor_2=1.6`.",
            "- The connector handoff is not a physical stair-climbing, real-robot, or collision-free navigation claim.",
            "",
        ]
    )
    return "\n".join(lines)


def build_summary(
    output_dir: Path,
    route_results_dir: Path,
    adapter_inputs_dir: Optional[Path],
    floor_z_map: dict[str, float],
    queries: list[dict[str, Any]],
) -> dict[str, Any]:
    rviz_count = sum(1 for query in queries if query["rviz_marker_input_path"])
    z_count = sum(1 for query in queries if query["z_aware_overlay_input_path"])
    floor_values = [float(value) for value in floor_z_map.values()]
    object_queries = [
        query
        for query in queries
        if query["target"].get("target_object_id") == OBJECT_ID
        or query["target"].get("target_object_category") == OBJECT_LABEL
    ]
    selected_ring_002 = [query for query in queries if query["selected_generated_ring_002"]]
    blocked_present = [query for query in queries if query["blocked_generated_ring_037_present"]]
    e001 = [query for query in queries if query["vt_1_centerline_e001_transition"]]
    e003_bad = [query for query in queries if query["vt_1_centerline_e003_transition"]]
    return {
        "schema_name": "rslg_static_visualization_pack_summary",
        "schema_version": "0.1",
        "project_name": PROJECT_NAME,
        "scene_id": MAIN_SCENE_ID,
        "generated_utc": utc_now(),
        "classification": "static_visualization_pack_exported",
        "output_dir": output_dir.as_posix(),
        "route_results_dir": route_results_dir.as_posix(),
        "adapter_inputs_dir": adapter_inputs_dir.as_posix() if adapter_inputs_dir else None,
        "index_html": (output_dir / "index.html").as_posix(),
        "index_md": (output_dir / "index.md").as_posix(),
        "summary_json": (output_dir / "summary.json").as_posix(),
        "floor_z_map": floor_z_map,
        "floor_z_min": min(floor_values) if floor_values else None,
        "floor_z_max": max(floor_values) if floor_values else None,
        "query_count": len(queries),
        "per_query_visual_file_count": len(queries),
        "per_query_markdown_count": len(queries),
        "rviz_marker_input_count": rviz_count,
        "z_aware_overlay_input_count": z_count,
        "object_target_visualization_status": {
            "target_object_id": OBJECT_ID,
            "target_label": OBJECT_LABEL,
            "target_room": OBJECT_ROOM,
            "target_floor": OBJECT_FLOOR,
            "object_query_count": len(object_queries),
            "status": "visualized_as_semantic_target_panel_and_selected_approach_marker",
        },
        "generated_ring_002_visualization_status": {
            "selected_count": len(selected_ring_002),
            "status": "selected_valid_object_approach_visualized" if selected_ring_002 else "not_present",
        },
        "generated_ring_037_rejected_blocked_visualization_status": {
            "blocked_evidence_count": len(blocked_present),
            "selected_count": sum(1 for query in queries if query["blocked_generated_ring_037_selected"]),
            "status": "blocked_rejected_evidence_only",
        },
        "vt_1_centerline_e001_connector_visualization_status": {
            "transition_count": len(e001),
            "status": "true_transition_edge_visualized" if e001 else "not_present",
        },
        "vt_1_centerline_e003_forbidden_non_transition_status": {
            "non_transition_count": sum(1 for query in queries if query["vt_1_centerline_e003_non_transition"]),
            "transition_count": len(e003_bad),
            "status": "forbidden_non_transition_only",
        },
        "claim_boundary": {
            "no_stage_a_rerun": True,
            "no_raw_rgbd_inference": True,
            "no_ros_live_execution": True,
            "no_rviz_launch": True,
            "no_gazebo_launch": True,
            "no_nav2_runtime_claim": True,
            "no_amcl_claim": True,
            "no_map_server_claim": True,
            "no_physical_stair_climbing_claim": True,
            "no_collision_free_guarantee": True,
        },
        "queries": [
            {
                "query_id": query["query_id"],
                "query_type": query["identity"].get("query_type"),
                "route_result_path": query["route_result_path"],
                "pid_input_path": query["pid_input_path"],
                "rviz_marker_input_path": query["rviz_marker_input_path"],
                "z_aware_overlay_input_path": query["z_aware_overlay_input_path"],
                "svg_path": query["svg_path"],
                "markdown_path": query["markdown_path"],
                "selected_approach_id": query["selected_approach"].get("candidate_id"),
                "blocked_generated_ring_037_present": query["blocked_generated_ring_037_present"],
                "blocked_generated_ring_037_selected": query["blocked_generated_ring_037_selected"],
                "vt_1_centerline_e001_transition": query["vt_1_centerline_e001_transition"],
                "vt_1_centerline_e003_non_transition": query["vt_1_centerline_e003_non_transition"],
                "vt_1_centerline_e003_transition": query["vt_1_centerline_e003_transition"],
                "rviz_marker_count": query["rviz_marker_count"],
                "z_aware_overlay_waypoint_count": query["z_aware_overlay_waypoint_count"],
                "z_min": query["z_min"],
                "z_max": query["z_max"],
            }
            for query in queries
        ],
        "ok": len(queries) >= 1 and not e003_bad,
    }


def export_pack(
    route_results_dir: Path,
    adapter_inputs_dir: Optional[Path],
    output_dir: Path,
    floor_z_map: dict[str, float],
    title: str,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = route_result_paths(route_results_dir)
    if not paths:
        raise SystemExit(f"no RouteResult JSON files found in {route_results_dir}")

    queries: list[dict[str, Any]] = []
    for path in paths:
        route_result = read_json(path)
        if not isinstance(route_result, dict) or route_result.get("schema_name") != ROUTE_RESULT_SCHEMA_NAME:
            continue
        queries.append(summarize_query(path, route_result, adapter_inputs_dir, output_dir, floor_z_map))

    summary = build_summary(output_dir, route_results_dir, adapter_inputs_dir, floor_z_map, queries)
    write_json(output_dir / "summary.json", summary)
    write_text(output_dir / "index.html", render_index_html(title, output_dir, summary, queries))
    write_text(output_dir / "index.md", render_index_md(title, summary, queries))
    return summary


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-results-dir", type=Path, required=True)
    parser.add_argument("--adapter-inputs-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--title", default="RSLG-SLAM Static Visualization Pack")
    args = parser.parse_args(argv)

    floor_z_map = parse_floor_z_map(args.floor_z_map)
    summary = export_pack(
        route_results_dir=args.route_results_dir,
        adapter_inputs_dir=args.adapter_inputs_dir,
        output_dir=args.output_dir,
        floor_z_map=floor_z_map,
        title=args.title,
    )
    print(json.dumps({"classification": summary["classification"], "query_count": summary["query_count"], "ok": summary["ok"]}, indent=2))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
