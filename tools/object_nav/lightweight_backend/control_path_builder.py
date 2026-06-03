"""Control path simplification, corner rounding, and validation-backed fallback."""

from __future__ import annotations

import math
from typing import Any

from .occupancy_planner import OccupancyPlanner
from .schemas import ControlPathResult, clamp, distance, now_iso, route_length


def simplify_control_path(
    dense_points: list[dict[str, Any]],
    planner: OccupancyPlanner,
    semantic_anchors_list: list[dict[str, Any]],
    normal_spacing: float = 0.60,
    gateway_spacing: float = 0.30,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Generate a simplified control path from the dense A* route using line-of-sight simplification."""
    if len(dense_points) < 3:
        return list(dense_points), {"method": "too_short", "original_count": len(dense_points), "simplified_count": len(dense_points)}

    # Identify anchor indices (points closest to semantic anchors like gateways)
    anchor_indices: set[int] = {0, len(dense_points) - 1}
    for anchor in semantic_anchors_list:
        if "x" not in anchor or "y" not in anchor:
            continue
        best_idx = min(range(len(dense_points)), key=lambda i: distance(dense_points[i], anchor))
        anchor_indices.add(best_idx)

    # Line-of-sight simplification preserving anchors
    simplified_indices: list[int] = [0]
    current = 0
    while current < len(dense_points) - 1:
        farthest = current + 1
        for candidate in range(len(dense_points) - 1, current, -1):
            if planner.line_of_sight(
                float(dense_points[current]["x"]), float(dense_points[current]["y"]),
                float(dense_points[candidate]["x"]), float(dense_points[candidate]["y"]),
            ):
                farthest = candidate
                break
        # Don't skip past any anchor index
        next_anchor = None
        for ai in sorted(anchor_indices):
            if ai > current:
                next_anchor = ai
                break
        if next_anchor is not None and farthest > next_anchor:
            farthest = next_anchor
        simplified_indices.append(farthest)
        current = farthest

    if simplified_indices[-1] != len(dense_points) - 1:
        simplified_indices.append(len(dense_points) - 1)

    # Resample long segments
    final_indices: list[int] = []
    for i in range(len(simplified_indices) - 1):
        si, ei = simplified_indices[i], simplified_indices[i + 1]
        seg_len = sum(distance(dense_points[j], dense_points[j + 1]) for j in range(si, ei))
        near_anchor = si in anchor_indices or ei in anchor_indices
        spacing = gateway_spacing if near_anchor else normal_spacing
        if seg_len > spacing * 1.5:
            accumulated = 0.0
            final_indices.append(si)
            for j in range(si, ei):
                accumulated += distance(dense_points[j], dense_points[j + 1])
                if accumulated >= spacing:
                    final_indices.append(j + 1)
                    accumulated = 0.0
        else:
            final_indices.append(si)
    final_indices.append(len(dense_points) - 1)

    # Deduplicate while preserving order
    seen: set[int] = set()
    unique_indices: list[int] = []
    for idx in final_indices:
        if idx not in seen:
            seen.add(idx)
            unique_indices.append(idx)

    control_points = [dict(dense_points[i]) for i in unique_indices]

    # Assign yaw
    for i in range(len(control_points) - 1):
        control_points[i]["yaw"] = round(math.atan2(
            float(control_points[i + 1]["y"]) - float(control_points[i]["y"]),
            float(control_points[i + 1]["x"]) - float(control_points[i]["x"]),
        ), 6)
    if len(control_points) > 1:
        control_points[-1]["yaw"] = control_points[-2]["yaw"]

    # Add waypoint_index
    for i, pt in enumerate(control_points):
        pt["waypoint_index"] = i

    validation = planner.validate_polyline(control_points, require_inflated=True)
    report = {
        "method": "line_of_sight_with_anchor_preservation_and_resampling",
        "original_dense_count": len(dense_points),
        "simplified_count": len(control_points),
        "anchor_indices_preserved": sorted(anchor_indices),
        "normal_spacing_m": normal_spacing,
        "gateway_spacing_m": gateway_spacing,
        "control_path_length_m": round(route_length(control_points), 6),
        "dense_route_length_m": round(route_length(dense_points), 6),
        "wall_crossing_validation_passed": validation["wall_crossing_validation_passed"],
        "minimum_clearance_m": validation.get("minimum_clearance_m"),
        "tested_sample_count": validation["tested_sample_count"],
    }
    return control_points, report


def round_control_path(
    control_points: list[dict[str, Any]],
    planner: OccupancyPlanner,
    semantic_anchors_list: list[dict[str, Any]],
    min_clearance_threshold: float = 0.10,
    chaikin_iterations: int = 2,
    resample_spacing: float = 0.25,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Apply corner rounding via Chaikin smoothing where safe, preserving semantic anchors."""
    if len(control_points) < 3:
        return list(control_points), {"method": "too_short", "corners_considered": 0, "corners_rounded": 0, "corners_rejected": 0}

    # Identify semantic anchor indices
    anchor_indices: set[int] = {0, len(control_points) - 1}
    for anchor in semantic_anchors_list:
        if "x" not in anchor or "y" not in anchor:
            continue
        best_idx = min(range(len(control_points)), key=lambda i: distance(control_points[i], anchor))
        anchor_indices.add(best_idx)

    # Identify corners
    corner_threshold_rad = 0.25
    corner_indices: list[int] = []
    for i in range(1, len(control_points) - 1):
        ax, ay = float(control_points[i - 1]["x"]), float(control_points[i - 1]["y"])
        bx, by = float(control_points[i]["x"]), float(control_points[i]["y"])
        cx, cy = float(control_points[i + 1]["x"]), float(control_points[i + 1]["y"])
        v1 = (bx - ax, by - ay)
        v2 = (cx - bx, cy - by)
        len1 = math.hypot(*v1)
        len2 = math.hypot(*v2)
        if len1 < 1e-6 or len2 < 1e-6:
            continue
        cos_angle = clamp((v1[0] * v2[0] + v1[1] * v2[1]) / (len1 * len2), -1.0, 1.0)
        turn_angle = math.acos(cos_angle)
        if turn_angle >= corner_threshold_rad:
            corner_indices.append(i)

    corners_considered = len(corner_indices)
    corners_rounded = 0
    corners_rejected = 0
    rejection_reasons: list[dict[str, Any]] = []

    sorted_anchors = sorted(anchor_indices)
    segments: list[tuple[int, int]] = []
    for j in range(len(sorted_anchors) - 1):
        segments.append((sorted_anchors[j], sorted_anchors[j + 1]))

    rounded_points: list[tuple[float, float]] = []

    for seg_start, seg_end in segments:
        seg_pts = [(float(control_points[k]["x"]), float(control_points[k]["y"])) for k in range(seg_start, seg_end + 1)]
        seg_corners = [c for c in corner_indices if seg_start < c < seg_end]

        if not seg_corners or len(seg_pts) < 3:
            if rounded_points and rounded_points[-1] == seg_pts[0]:
                for pt in seg_pts[1:]:
                    rounded_points.append(pt)
            else:
                for pt in seg_pts:
                    rounded_points.append(pt)
            continue

        # Chaikin subdivision
        smoothed = list(seg_pts)
        for _iteration in range(chaikin_iterations):
            if len(smoothed) < 3:
                break
            new_pts: list[tuple[float, float]] = [smoothed[0]]
            for k in range(len(smoothed) - 1):
                p0 = smoothed[k]
                p1 = smoothed[k + 1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                new_pts.append(q)
                new_pts.append(r)
            new_pts.append(smoothed[-1])
            smoothed = new_pts

        # Validate smoothed segment
        smoothed_dicts = [{"x": p[0], "y": p[1]} for p in smoothed]
        validation = planner.validate_polyline(smoothed_dicts, require_inflated=True)

        clearances = []
        for p in smoothed:
            rc = planner.rc(p[0], p[1])
            if planner.in_bounds(rc) and planner.free(rc):
                clearances.append(float(planner.clearance[rc[0], rc[1]]))

        min_clr = min(clearances) if clearances else 0.0

        if validation["wall_crossing_validation_passed"] and min_clr >= min_clearance_threshold:
            corners_rounded += len(seg_corners)
            if rounded_points and math.hypot(rounded_points[-1][0] - smoothed[0][0], rounded_points[-1][1] - smoothed[0][1]) < 1e-4:
                for pt in smoothed[1:]:
                    rounded_points.append(pt)
            else:
                for pt in smoothed:
                    rounded_points.append(pt)
        else:
            corners_rejected += len(seg_corners)
            reason = "wall_crossing" if not validation["wall_crossing_validation_passed"] else f"clearance_too_low_{min_clr:.4f}"
            rejection_reasons.append({"segment": f"{seg_start}-{seg_end}", "corners": seg_corners, "reason": reason})
            if rounded_points and rounded_points[-1] == seg_pts[0]:
                for pt in seg_pts[1:]:
                    rounded_points.append(pt)
            else:
                for pt in seg_pts:
                    rounded_points.append(pt)

    # Deduplicate consecutive duplicates
    deduped: list[tuple[float, float]] = [rounded_points[0]]
    for pt in rounded_points[1:]:
        if math.hypot(pt[0] - deduped[-1][0], pt[1] - deduped[-1][1]) > 1e-5:
            deduped.append(pt)

    # Resample at uniform spacing
    resampled: list[tuple[float, float]] = [deduped[0]]
    accumulated = 0.0
    for i in range(len(deduped) - 1):
        seg_len = math.hypot(deduped[i + 1][0] - deduped[i][0], deduped[i + 1][1] - deduped[i][1])
        if seg_len < 1e-9:
            continue
        remaining_in_seg = seg_len
        dx = (deduped[i + 1][0] - deduped[i][0]) / seg_len
        dy = (deduped[i + 1][1] - deduped[i][1]) / seg_len
        offset = 0.0
        while True:
            needed = resample_spacing - accumulated
            if needed <= remaining_in_seg:
                offset += needed
                resampled.append((deduped[i][0] + offset * dx, deduped[i][1] + offset * dy))
                remaining_in_seg -= needed
                accumulated = 0.0
            else:
                accumulated += remaining_in_seg
                break
    if math.hypot(resampled[-1][0] - deduped[-1][0], resampled[-1][1] - deduped[-1][1]) > 0.05:
        resampled.append(deduped[-1])
    else:
        resampled[-1] = deduped[-1]

    # Convert back to dicts with yaw
    result_points: list[dict[str, Any]] = []
    for i, (x, y) in enumerate(resampled):
        result_points.append({"x": round(x, 6), "y": round(y, 6), "waypoint_index": i})
    for i in range(len(result_points) - 1):
        result_points[i]["yaw"] = round(math.atan2(
            float(result_points[i + 1]["y"]) - float(result_points[i]["y"]),
            float(result_points[i + 1]["x"]) - float(result_points[i]["x"]),
        ), 6)
    if len(result_points) > 1:
        result_points[-1]["yaw"] = result_points[-2]["yaw"]

    # Final validation
    final_validation = planner.validate_polyline(result_points, require_inflated=True)

    report = {
        "method": "chaikin_corner_rounding_with_occupancy_validation",
        "chaikin_iterations": chaikin_iterations,
        "resample_spacing_m": resample_spacing,
        "min_clearance_threshold_m": min_clearance_threshold,
        "input_control_path_length_m": round(route_length(control_points), 6),
        "rounded_control_path_length_m": round(route_length(result_points), 6),
        "corners_considered": corners_considered,
        "corners_rounded": corners_rounded,
        "corners_rejected": corners_rejected,
        "rejection_reasons": rejection_reasons,
        "input_point_count": len(control_points),
        "output_point_count": len(result_points),
        "wall_crossing_validation_passed": final_validation["wall_crossing_validation_passed"],
        "minimum_clearance_m": final_validation.get("minimum_clearance_m"),
        "tested_sample_count": final_validation["tested_sample_count"],
    }

    # If the rounded path fails validation, fall back to the original
    if not final_validation["wall_crossing_validation_passed"]:
        report["fallback_to_original"] = True
        return list(control_points), report

    return result_points, report


def build_control_path(
    dense_points: list[dict[str, Any]],
    planner: OccupancyPlanner,
    semantic_anchors_list: list[dict[str, Any]],
    params,
) -> ControlPathResult:
    """Build control path: simplify → round → validate → fallback if needed."""
    # Simplify
    simplified_path, simplification_report = simplify_control_path(
        dense_points, planner, semantic_anchors_list,
        normal_spacing=params.control_path_spacing,
        gateway_spacing=params.control_path_gateway_spacing,
    )
    simplification_report["created_utc"] = now_iso()
    simplification_report["artifact_type"] = "task17c_control_path_simplification_report"

    # Round
    control_path, rounding_report = round_control_path(
        simplified_path, planner, semantic_anchors_list,
        min_clearance_threshold=params.inflation_radius_m * 0.6,
        chaikin_iterations=params.corner_rounding_chaikin_iterations,
        resample_spacing=params.corner_rounding_resample_spacing,
    )
    rounding_report["created_utc"] = now_iso()
    rounding_report["artifact_type"] = "task17c_control_path_rounding_report"

    # Determine final source
    rounding_attempted = True
    rounding_applied = rounding_report.get("corners_rounded", 0) > 0 and not rounding_report.get("fallback_to_original", False)
    fallback_to_original = rounding_report.get("fallback_to_original", False)
    rounded_candidate_validation_passed = rounding_report.get("wall_crossing_validation_passed", False) if not fallback_to_original else False

    if fallback_to_original:
        final_control_path_source = "fallback_simplified_control_path"
        fallback_reason = "rounded path failed wall-crossing validation"
    elif rounding_applied:
        final_control_path_source = "rounded_control_path"
        fallback_reason = None
    else:
        final_control_path_source = "simplified_control_path"
        fallback_reason = "no corners rounded" if rounding_report.get("corners_considered", 0) > 0 else "no corners to round"

    return ControlPathResult(
        control_path=control_path,
        simplified_path=simplified_path,
        simplification_report=simplification_report,
        rounding_report=rounding_report,
        rounding_attempted=rounding_attempted,
        rounding_applied=rounding_applied,
        fallback_to_original=fallback_to_original,
        final_control_path_source=final_control_path_source,
        rounded_candidate_validation_passed=rounded_candidate_validation_passed,
        fallback_reason=fallback_reason,
    )
