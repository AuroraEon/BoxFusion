#!/usr/bin/env python3
"""Z-aware route projection for RSLG-SLAM vertical-transition visualization.

This module converts a Layer 3 ``rslg_route_result`` (or a RouteResult-derived
z-aware overlay input, schema ``rslg_route_result_z_aware_overlay_input``) into a
z-aware waypoint sequence used only for *visualization* of the vertical connector
``vt_1_centerline_e001``. It preserves x/y base-level route following and derives a
per-waypoint visualization ``z`` from the RouteResult floor labels, rising from
floor_1 ``z=0.0`` to floor_2 ``z=1.6`` only when the RouteResult contains an
explicit connector handoff route segment.

It consumes the formal RouteResult fields:

* ``route_segments`` (executable same-floor / object-approach segments)
* explicit connector-handoff ``route_segments`` (vertical connector)
* ``metric_path`` (full stitched 2D path, provenance/summary)

The projection itself lives in the RouteResult-derived z-aware overlay adapter
(:mod:`tools.rslg_pipeline.runtime.route_result_z_aware_adapter`); this module is a
thin projection/CLI wrapper plus the follower-facing ``visual_z_for_segment``.

This is a visualization overlay. It does NOT implement physical stair climbing,
quadruped gait control, or Unitree Go2 real-robot control, and it is importable by
the ROS route follower as well as runnable standalone (no rclpy dependency). The
old task48 route-executor input shape is no longer a formal input here.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import NON_TRANSITION_EDGE, TRUE_TRANSITION_EDGE
from tools.rslg_pipeline.runtime.route_result_adapter_common import is_route_result
from tools.rslg_pipeline.runtime.route_result_z_aware_adapter import (
    OVERLAY_SCHEMA,
    build_z_aware_overlay_input,
)

FLOOR_Z_DEFAULTS = {"floor_1": 0.0, "floor_2": 1.6, "floor_transition": 0.8}
PHASE_FLOOR_1 = "floor_1"
PHASE_TRANSITION = "vertical_transition"
PHASE_FLOOR_2 = "floor_2"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def _floor_z_map(floor1_z: float, floor2_z: float) -> dict[str, float]:
    return {"floor_1": float(floor1_z), "floor_2": float(floor2_z)}


def project_route_result(
    route_result: dict[str, Any],
    *,
    floor1_z: float = 0.0,
    floor2_z: float = 1.6,
    transition_edge: str = TRUE_TRANSITION_EDGE,
    source_ref: Optional[str] = None,
) -> dict[str, Any]:
    """Project an ``rslg_route_result`` into the z-aware overlay input payload.

    Delegates to the RouteResult-derived z-aware overlay adapter and exposes the
    ``overlay_waypoints`` list (each carrying index, x, y, z, yaw, floor_id,
    route_phase, segment_id) plus summary/validation fields.
    """

    return build_z_aware_overlay_input(
        route_result,
        floor_z_map=_floor_z_map(floor1_z, floor2_z),
        source_ref=source_ref,
        transition_edge=transition_edge,
    )


def overlay_waypoints(overlay_or_route_result: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the z-aware overlay waypoint list from an overlay input or RouteResult."""

    if overlay_or_route_result.get("schema_name") == OVERLAY_SCHEMA:
        return list(overlay_or_route_result.get("overlay_waypoints") or [])
    if is_route_result(overlay_or_route_result):
        return list(project_route_result(overlay_or_route_result)["overlay_waypoints"])
    raise ValueError(
        "expected an rslg_route_result or rslg_route_result_z_aware_overlay_input payload"
    )


def _interp_z(z_from: float, z_to: float, frac: float) -> float:
    frac = max(0.0, min(1.0, frac))
    return float(z_from) + (float(z_to) - float(z_from)) * frac


def visual_z_for_segment(
    z_aware_waypoints: list[dict[str, Any]],
    segment_target_index: int,
    pose_x: float,
    pose_y: float,
) -> float:
    """Interpolate the visual z at the robot's current pose along a segment.

    ``segment_target_index`` is the index (in the z-aware list) of the waypoint the
    follower is currently tracking. The robot's visual z is the interpolation of the
    previous and target waypoint z by the fraction of the segment covered.
    """
    if not z_aware_waypoints:
        return 0.0
    idx = max(0, min(segment_target_index, len(z_aware_waypoints) - 1))
    target = z_aware_waypoints[idx]
    if idx == 0:
        return float(target["z"])
    prev = z_aware_waypoints[idx - 1]
    seg_len = math.hypot(float(target["x"]) - float(prev["x"]), float(target["y"]) - float(prev["y"]))
    if seg_len < 1e-6:
        return float(target["z"])
    covered = math.hypot(pose_x - float(prev["x"]), pose_y - float(prev["y"]))
    frac = covered / seg_len
    return round(_interp_z(float(prev["z"]), float(target["z"]), frac), 6)


def build_projection_markdown(summary: dict[str, Any]) -> str:
    handoff = summary.get("connector_handoff") or {}
    return (
        "# Z-Aware Route Projection Report\n\n"
        "- project: **RSLG-SLAM**\n"
        f"- source_route_result: `{summary.get('source_route_result')}`\n"
        f"- route_type: `{summary.get('route_type')}`\n"
        f"- overlay_waypoint_count: `{summary.get('overlay_waypoint_count')}`\n"
        f"- floor_z_map: `{summary.get('floor_z_map')}`\n"
        f"- z_min / z_max: `{summary.get('z_min')}` / `{summary.get('z_max')}`\n"
        f"- z_aware_visual_transition_detected: `{summary.get('z_aware_visual_transition_detected')}`\n"
        f"- transition_edge_used: `{summary.get('transition_edge_used')}`\n"
        f"- vertical_transition_edge_is_e001: `{summary.get('vertical_transition_edge_is_e001')}`\n"
        f"- connector_handoff.handoff_type: `{handoff.get('handoff_type')}`\n"
        f"- connector_handoff.not_physical_stair_climbing: `{handoff.get('not_physical_stair_climbing')}`\n"
        f"- physical_climb_claimed: `{summary.get('physical_climb_claimed')}`\n\n"
        "This projection is a visualization overlay derived from the Layer 3\n"
        "RSLGRouteResult route_segments and metric_path. It preserves base-level\n"
        "x/y route following and derives z from the route floor labels. It rises\n"
        "from floor_1 z=0.0 to floor_2 z=1.6 only across an explicit connector\n"
        "handoff route segment using vt_1_centerline_e001. The non-transition\n"
        "edge vt_1_centerline_e003 is never used as the transition. It does not\n"
        "implement or claim physical stair climbing, quadruped gait control, or\n"
        "Unitree Go2 real-robot control.\n"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--route-result-json", type=Path, default=None)
    source.add_argument("--overlay-input-json", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--report-md", type=Path, default=None)
    parser.add_argument("--floor1-z", type=float, default=0.0)
    parser.add_argument("--floor2-z", type=float, default=1.6)
    parser.add_argument("--vertical-transition-edge", default=TRUE_TRANSITION_EDGE)
    parser.add_argument("--no-canonical-write", action="store_true")
    args = parser.parse_args()

    if args.no_canonical_write and "canonical" in args.output_json.resolve().parts:
        raise SystemExit("--no-canonical-write refused output under a canonical directory")

    if args.overlay_input_json is not None:
        summary = read_json(args.overlay_input_json)
        if summary.get("schema_name") != OVERLAY_SCHEMA:
            raise SystemExit(
                f"--overlay-input-json must be a {OVERLAY_SCHEMA} payload, "
                f"got {summary.get('schema_name')!r}"
            )
    else:
        route_result = read_json(args.route_result_json)
        summary = project_route_result(
            route_result,
            floor1_z=args.floor1_z,
            floor2_z=args.floor2_z,
            transition_edge=args.vertical_transition_edge,
            source_ref=args.route_result_json.as_posix(),
        )

    write_json(args.output_json, summary)
    if args.report_md:
        Path(args.report_md).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report_md).write_text(build_projection_markdown(summary), encoding="utf-8")
    print(
        json.dumps(
            {
                "route_type": summary.get("route_type"),
                "overlay_waypoint_count": summary.get("overlay_waypoint_count"),
                "z_min": summary.get("z_min"),
                "z_max": summary.get("z_max"),
                "z_aware_visual_transition_detected": summary.get("z_aware_visual_transition_detected"),
                "transition_edge_used": summary.get("transition_edge_used"),
                "output_json": args.output_json.as_posix(),
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
