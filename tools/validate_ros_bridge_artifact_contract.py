#!/usr/bin/env python
"""Validate and preview the Step 11 artifact-backed ROS/RViz bridge contract."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from boxfusion.ros_artifact_bridge import (
    DEFAULT_FRAME_ID,
    CommittedArtifactBundle,
    build_marker_specs,
    validate_coordinates,
)
from boxfusion.route_to_waypoints import load_route_json, route_to_waypoints


RETAINED_SCENE_ROOTS = [
    Path("runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V"),
    Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00824-Dd4bFSTQ8gi"),
    Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00862-LT9Jq6dN3Ea"),
    Path("runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs"),
]


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _scene_slug(bundle: CommittedArtifactBundle) -> str:
    return str(bundle.scene_id or bundle.scene_root.name).replace("/", "_")


def _default_route(bundle: CommittedArtifactBundle) -> Dict[str, Any]:
    first_route = bundle.query_report.get("first_route")
    if isinstance(first_route, dict):
        return dict(first_route)
    rooms = [room.get("id") or room.get("room_id") for room in bundle.topology_rooms]
    rooms = [room for room in rooms if room]
    if len(rooms) >= 2:
        return {"room_sequence": [rooms[0], rooms[1]], "used_relation_types": [], "edge_confidences": []}
    return {"room_sequence": rooms}


def _marker_count_total(marker_specs: Dict[str, Any]) -> int:
    return sum(int(count) for count in dict(marker_specs.get("counts") or {}).values())


def _combined_validation_warnings(
    coordinate_warnings: Sequence[Dict[str, Any]],
    waypoint_payload: Dict[str, Any],
    marker_specs: Dict[str, Any],
) -> List[Dict[str, Any]]:
    warnings: List[Dict[str, Any]] = []
    index_by_key: Dict[str, int] = {}

    def add_warning(source: str, payload: Any) -> None:
        if isinstance(payload, dict):
            item = dict(payload)
        else:
            item = {
                "severity": "warning",
                "code": "route_to_waypoints_warning",
                "message": str(payload),
            }
        item.setdefault("severity", "warning")
        key_payload = {
            "severity": item.get("severity"),
            "code": item.get("code"),
            "message": item.get("message"),
            "source_artifact": item.get("source_artifact"),
            "source_ids": item.get("source_ids"),
        }
        key = json.dumps(key_payload, sort_keys=True, default=str)
        if key in index_by_key:
            existing = warnings[index_by_key[key]]
            sources = list(existing.get("sources") or [])
            if source not in sources:
                sources.append(source)
            existing["sources"] = sources
            return
        item["sources"] = [source]
        index_by_key[key] = len(warnings)
        warnings.append(item)

    for warning in coordinate_warnings:
        add_warning("coordinate_validation", warning)
    for warning in waypoint_payload.get("warnings") or []:
        add_warning("route_to_waypoints", warning)
    for warning in marker_specs.get("warnings") or []:
        add_warning("marker_specs", warning)
    return warnings


def validate_scene(
    scene_root: Path,
    *,
    out_dir: Path,
    route_json: Optional[Path],
    frame_id: str,
    include_debug: bool,
) -> Dict[str, Any]:
    bundle = CommittedArtifactBundle.from_scene_root(scene_root)
    route_payload = load_route_json(route_json) if route_json is not None else _default_route(bundle)
    waypoint_payload = route_to_waypoints(route_payload, bundle, frame_id=frame_id)
    marker_specs = build_marker_specs(
        bundle,
        route_waypoints=waypoint_payload,
        frame_id=frame_id,
        include_debug=include_debug,
    )
    coordinate_warnings = validate_coordinates(bundle)
    validation_warnings = _combined_validation_warnings(
        coordinate_warnings,
        waypoint_payload,
        marker_specs,
    )

    slug = _scene_slug(bundle)
    scene_out_dir = out_dir / slug
    _write_json(scene_out_dir / "artifact_bridge_summary.json", bundle.describe())
    _write_json(scene_out_dir / "coordinate_validation.json", {"warnings": coordinate_warnings})
    _write_json(scene_out_dir / "validation_warnings.json", validation_warnings)
    _write_json(scene_out_dir / "route_to_waypoints_preview.json", waypoint_payload)
    _write_json(scene_out_dir / "route_waypoints_preview.json", waypoint_payload)
    _write_json(scene_out_dir / "marker_specs_preview.json", marker_specs)
    _write_json(scene_out_dir / "marker_specs.json", marker_specs)

    severity_counts: Dict[str, int] = {}
    for warning in validation_warnings:
        severity = str(warning.get("severity") or "warning")
        severity_counts[severity] = severity_counts.get(severity, 0) + 1

    return {
        "scene_id": bundle.scene_id,
        "scene_root": str(bundle.scene_root),
        "counts": bundle.counts(),
        "route_room_sequence": list(route_payload.get("room_sequence") or (route_payload.get("route") or {}).get("room_sequence") or []),
        "waypoint_count": len(waypoint_payload.get("waypoints") or []),
        "segment_count": len(waypoint_payload.get("segments") or []),
        "marker_group_counts": dict(marker_specs.get("counts") or {}),
        "marker_count_total": _marker_count_total(marker_specs),
        "validation_issue_count": len(validation_warnings),
        "validation_severity_counts": severity_counts,
        "outputs": {
            "summary": str(scene_out_dir / "artifact_bridge_summary.json"),
            "coordinate_validation": str(scene_out_dir / "coordinate_validation.json"),
            "validation_warnings": str(scene_out_dir / "validation_warnings.json"),
            "route_to_waypoints_preview": str(scene_out_dir / "route_to_waypoints_preview.json"),
            "route_waypoints_preview": str(scene_out_dir / "route_waypoints_preview.json"),
            "marker_specs_preview": str(scene_out_dir / "marker_specs_preview.json"),
            "marker_specs": str(scene_out_dir / "marker_specs.json"),
        },
    }


def _write_csv_summary(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scene_id",
        "rooms",
        "edges",
        "floors",
        "gateways",
        "vertical_transitions",
        "anchors",
        "objects",
        "waypoint_count",
        "segment_count",
        "marker_count_total",
        "validation_issue_count",
        "validation_severity_counts",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            counts = dict(row.get("counts") or {})
            writer.writerow(
                {
                    "scene_id": row.get("scene_id"),
                    "rooms": counts.get("rooms", 0),
                    "edges": counts.get("edges", 0),
                    "floors": counts.get("floors", 0),
                    "gateways": counts.get("gateways", 0),
                    "vertical_transitions": counts.get("vertical_transitions", 0),
                    "anchors": counts.get("anchors", 0),
                    "objects": counts.get("objects", 0),
                    "waypoint_count": row.get("waypoint_count", 0),
                    "segment_count": row.get("segment_count", 0),
                    "marker_count_total": row.get("marker_count_total", 0),
                    "validation_issue_count": row.get("validation_issue_count", 0),
                    "validation_severity_counts": json.dumps(row.get("validation_severity_counts") or {}, sort_keys=True),
                }
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate committed/public BoxFusion artifacts and build ROS-independent "
            "RViz marker/route waypoint previews."
        )
    )
    parser.add_argument(
        "--scene-root",
        action="append",
        default=[],
        help="Scene root containing logs/topology_v0_1.json and the other retained committed/public artifacts. May be repeated.",
    )
    parser.add_argument(
        "--all-retained-scenes",
        action="store_true",
        help="Validate the four retained HM3D scenes from the Step 11 contract.",
    )
    parser.add_argument("--out-dir", required=True, help="Directory for validation and marker preview JSON outputs.")
    parser.add_argument("--route-json", default=None, help="Optional route JSON to preview for every selected scene.")
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--include-debug", action="store_true", help="Include optional debug marker groups; off by default.")
    return parser


def _selected_scene_roots(args: argparse.Namespace) -> List[Path]:
    roots = [Path(item) for item in args.scene_root]
    if args.all_retained_scenes or not roots:
        roots.extend(RETAINED_SCENE_ROOTS)
    seen = set()
    ordered: List[Path] = []
    for root in roots:
        resolved = root.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        ordered.append(resolved)
    return ordered


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    out_dir = Path(args.out_dir).resolve()
    route_json = None if args.route_json is None else Path(args.route_json).resolve()
    rows = [
        validate_scene(
            scene_root,
            out_dir=out_dir,
            route_json=route_json,
            frame_id=args.frame_id,
            include_debug=bool(args.include_debug),
        )
        for scene_root in _selected_scene_roots(args)
    ]
    report = {
        "scene_count": len(rows),
        "frame_id": args.frame_id,
        "committed_public_only": True,
        "rows": rows,
    }
    _write_json(out_dir / "ros_bridge_artifact_contract_report.json", report)
    _write_csv_summary(out_dir / "ros_bridge_artifact_contract_report.csv", rows)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
