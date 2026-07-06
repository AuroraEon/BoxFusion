#!/usr/bin/env python3
"""RSLGRouteResult -> RViz marker input adapter (Layer 4 visualization).

Converts one Layer 3 ``rslg_route_result`` into a RouteResult-derived RViz marker
input JSON (schema ``rslg_route_result_rviz_marker_input``). The resulting marker
records are plain dicts that ``tools/rslg_pipeline/viz/live_marker_publisher.py``
can publish later via ``marker_from_dict`` (ns / id / type / frame_id / points /
pose / scale / color_rgba / text / yaw / metadata).

This adapter does not launch RViz and never publishes ROS messages. It is
ROS/rclpy-free so the conda offline Python can build/validate the marker input.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.runtime.route_result_adapter_common import (
    DEFAULT_FLOOR_Z_MAP,
    PROJECT,
    base_claim_boundary,
    blocked_legacy_rejected,
    connector_handoff_records,
    executable_segments,
    floor_z,
    guard_route_result,
    identity_block,
    provenance_block,
    query_id,
    segment_waypoints,
    selected_goal,
    utc_now,
)

SCHEMA_NAME = "rslg_route_result_rviz_marker_input"
SCHEMA_VERSION = "0.1"
DEFAULT_TOPIC = "/rslg_slam/live_route_markers"
DEFAULT_FRAME_ID = "map"

FLOOR_COLORS = {
    "floor_1": [0.10, 0.55, 0.95, 1.0],
    "floor_2": [0.95, 0.50, 0.10, 1.0],
}
SEGMENT_COLOR = [0.30, 0.75, 0.95, 0.9]
METRIC_PATH_COLOR = [0.20, 0.90, 0.55, 1.0]
CONNECTOR_COLOR = [0.72, 0.20, 0.95, 1.0]
APPROACH_COLOR = [0.05, 0.85, 0.30, 1.0]
BLOCKED_COLOR = [0.95, 0.08, 0.08, 0.85]
LABEL_COLOR = [0.92, 0.92, 0.92, 1.0]


def _xyz(x: float, y: float, z: float) -> dict[str, float]:
    return {"x": round(float(x), 6), "y": round(float(y), 6), "z": round(float(z), 6)}


def _marker(
    ns: str,
    marker_id: int,
    marker_type: str,
    frame_id: str,
    *,
    points: Optional[list[dict[str, float]]] = None,
    position: Optional[dict[str, float]] = None,
    scale: Optional[dict[str, float]] = None,
    color_rgba: Optional[list[float]] = None,
    text: Optional[str] = None,
    yaw: Optional[float] = None,
    floor_id: Optional[str] = None,
    metadata: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    return {
        "ns": ns,
        "id": marker_id,
        "type": marker_type,
        "frame_id": frame_id,
        "floor_id": floor_id,
        "points": points or [],
        "pose": {"position": position} if position is not None else None,
        "scale": scale or {"x": 0.12, "y": 0.12, "z": 0.12},
        "color_rgba": color_rgba or [1.0, 1.0, 1.0, 1.0],
        "text": text,
        "yaw": yaw,
        "metadata": metadata or {},
    }


def build_rviz_marker_input(
    route_result: dict[str, Any],
    *,
    floor_z_map: Optional[dict[str, float]] = None,
    source_ref: Optional[str] = None,
    frame_id: str = DEFAULT_FRAME_ID,
    topic: str = DEFAULT_TOPIC,
) -> dict[str, Any]:
    """Build a ``rslg_route_result_rviz_marker_input`` payload from a RouteResult."""

    guard_route_result(route_result)
    fzmap = dict(floor_z_map or DEFAULT_FLOOR_Z_MAP)
    qid = query_id(route_result)
    query_type = (route_result.get("identity") or {}).get("query_type")

    markers: list[dict[str, Any]] = []
    marker_id = 1

    # --- Semantic route markers (room/floor sequence text) ---
    semantic = route_result.get("semantic_route") or {}
    room_seq = semantic.get("room_sequence") or []
    if room_seq:
        markers.append(
            _marker(
                f"rslg_{qid}_semantic_room_sequence",
                marker_id,
                "TEXT_VIEW_FACING",
                frame_id,
                position=_xyz(0.0, 0.0, max(fzmap.values(), default=1.6) + 0.6),
                scale={"x": 0.2, "y": 0.2, "z": 0.2},
                color_rgba=LABEL_COLOR,
                text=" -> ".join(str(r) for r in room_seq),
                metadata={
                    "visualization_role": "semantic_route_marker",
                    "room_sequence": room_seq,
                    "floor_sequence": semantic.get("floor_sequence"),
                },
            )
        )
        marker_id += 1

    # --- Metric path marker (full stitched path, per-floor z) ---
    metric_path = route_result.get("metric_path") or {}
    path = metric_path.get("path")
    if isinstance(path, list) and path:
        floor_seq = semantic.get("floor_sequence") or []
        default_floor = floor_seq[0] if floor_seq else "floor_1"
        path_points = [
            _xyz(p["x"], p["y"], floor_z(p.get("floor_id", default_floor), fzmap))
            for p in path
            if isinstance(p, dict) and "x" in p and "y" in p
        ]
        if path_points:
            markers.append(
                _marker(
                    f"rslg_{qid}_metric_path",
                    marker_id,
                    "LINE_STRIP",
                    frame_id,
                    points=path_points,
                    scale={"x": 0.05, "y": 0.05, "z": 0.05},
                    color_rgba=METRIC_PATH_COLOR,
                    metadata={
                        "visualization_role": "metric_path_marker",
                        "metric_path_scope": metric_path.get("metric_path_scope"),
                        "path_source": metric_path.get("path_source"),
                    },
                )
            )
            marker_id += 1

    # --- Per-segment markers (executable same-floor / object-approach) ---
    for seg in executable_segments(route_result):
        floor_id = seg.get("floor_id")
        z = floor_z(floor_id, fzmap)
        pts = segment_waypoints(seg)
        if len(pts) < 2:
            continue
        markers.append(
            _marker(
                f"rslg_{qid}_segment_{seg.get('segment_id')}",
                marker_id,
                "LINE_STRIP",
                frame_id,
                points=[_xyz(p["x"], p["y"], z) for p in pts],
                scale={"x": 0.04, "y": 0.04, "z": 0.04},
                color_rgba=FLOOR_COLORS.get(str(floor_id), SEGMENT_COLOR),
                floor_id=floor_id,
                metadata={
                    "visualization_role": "segment_marker",
                    "segment_type": seg.get("segment_type"),
                    "segment_id": seg.get("segment_id"),
                },
            )
        )
        marker_id += 1

    # --- Connector handoff markers (semantic / visualization-only) ---
    for handoff in connector_handoff_records(route_result, fzmap):
        start = handoff.get("start")
        end = handoff.get("end")
        if not start or not end:
            continue
        markers.append(
            _marker(
                f"{handoff.get('transition_edge')}_connector_handoff",
                marker_id,
                "LINE_STRIP",
                frame_id,
                points=[_xyz(start["x"], start["y"], start["z"]), _xyz(end["x"], end["y"], end["z"])],
                scale={"x": 0.08, "y": 0.08, "z": 0.08},
                color_rgba=CONNECTOR_COLOR,
                metadata={
                    "visualization_role": "connector_handoff",
                    "transition_edge": handoff.get("transition_edge"),
                    "non_transition_edge": handoff.get("non_transition_edge"),
                    "handoff_type": "semantic_handoff",
                    "visualization_only": True,
                    "not_physical_stair_climbing": True,
                },
            )
        )
        marker_id += 1

    # --- Selected object-approach goal marker ---
    goal = selected_goal(route_result, fzmap)
    if goal and "x" in goal:
        markers.append(
            _marker(
                f"{goal.get('candidate_id')}_selected_approach_marker",
                marker_id,
                "SPHERE",
                frame_id,
                position=_xyz(goal["x"], goal["y"], goal["z"] + 0.12),
                scale={"x": 0.28, "y": 0.28, "z": 0.28},
                color_rgba=APPROACH_COLOR,
                yaw=goal.get("yaw"),
                floor_id=goal.get("floor_id"),
                metadata={
                    "visualization_role": "selected_object_approach_goal",
                    "candidate_id": goal.get("candidate_id"),
                    "used_as_runtime_goal": True,
                    "object_centroid": bool(goal.get("is_object_centroid")),
                },
            )
        )
        marker_id += 1

    # --- Rejected blocked-legacy candidate marker (generated_ring_037) ---
    for rejected in blocked_legacy_rejected(route_result):
        xy = rejected.get("world_xy") or [0.0, 0.0]
        markers.append(
            _marker(
                f"{rejected.get('candidate_id')}_blocked_only_marker",
                marker_id,
                "CUBE",
                frame_id,
                position=_xyz(xy[0], xy[1], fzmap.get("floor_2", 1.6) + 0.1),
                scale={"x": 0.22, "y": 0.22, "z": 0.22},
                color_rgba=BLOCKED_COLOR,
                floor_id="floor_2",
                metadata={
                    "visualization_role": "blocked_legacy_evidence_only",
                    "candidate_id": rejected.get("candidate_id"),
                    "used_as_runtime_goal": False,
                    "status": rejected.get("status"),
                },
            )
        )
        marker_id += 1

    # --- Floor z reference labels (visualization-only) ---
    for floor_id, z in sorted(fzmap.items()):
        markers.append(
            _marker(
                f"{floor_id}_z_label",
                marker_id,
                "TEXT_VIEW_FACING",
                frame_id,
                position=_xyz(3.0, -0.8 if floor_id == "floor_1" else -1.4, float(z) + 0.12),
                scale={"x": 0.22, "y": 0.22, "z": 0.22},
                color_rgba=LABEL_COLOR,
                text=f"{floor_id} z={float(z):.1f} m (visualization only)",
                floor_id=floor_id,
                metadata={"visualization_role": "floor_z_reference"},
            )
        )
        marker_id += 1

    roles = [(m.get("metadata") or {}).get("visualization_role") for m in markers]
    payload: dict[str, Any] = {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT,
        "generated_utc": utc_now(),
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "source_route_result": source_ref,
        "identity": identity_block(route_result),
        "route_type": query_type,
        "frame_id": frame_id,
        "topic": topic,
        "rviz_map_display_required": False,
        "requires_nav2": False,
        "requires_amcl": False,
        "requires_map_server": False,
        "requires_nav2_map_server": False,
        "floor_z_map": fzmap,
        "metric_path_scope": metric_path.get("metric_path_scope"),
        "markers": markers,
        "marker_count": len(markers),
        "marker_summary": {
            "semantic_route_marker": roles.count("semantic_route_marker"),
            "metric_path_marker": roles.count("metric_path_marker"),
            "segment_marker": roles.count("segment_marker"),
            "connector_handoff": roles.count("connector_handoff"),
            "selected_object_approach_goal": roles.count("selected_object_approach_goal"),
            "blocked_legacy_evidence_only": roles.count("blocked_legacy_evidence_only"),
            "floor_z_reference": roles.count("floor_z_reference"),
        },
        "claim_boundary": base_claim_boundary(),
        "provenance": provenance_block(route_result, source_ref),
    }
    return payload


def read_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def _parse_floor_z_map(raw: Optional[str]) -> dict[str, float]:
    if not raw:
        return dict(DEFAULT_FLOOR_Z_MAP)
    return {str(k): float(v) for k, v in json.loads(raw).items()}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route-result-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--floor-z-map", default=None)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args(argv)

    if args.no_canonical_write and "canonical" in args.output_json.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    route_result = read_json(args.route_result_json)
    payload = build_rviz_marker_input(
        route_result,
        floor_z_map=_parse_floor_z_map(args.floor_z_map),
        source_ref=args.route_result_json.as_posix(),
        frame_id=args.frame_id,
        topic=args.topic,
    )
    write_json(args.output_json, payload)
    print(
        json.dumps(
            {
                "schema_name": payload["schema_name"],
                "query_id": payload["identity"]["query_id"],
                "marker_count": payload["marker_count"],
                "marker_summary": payload["marker_summary"],
                "output_json": args.output_json.as_posix(),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
