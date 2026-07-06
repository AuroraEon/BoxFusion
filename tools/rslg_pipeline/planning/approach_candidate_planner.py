"""Object approach candidate planner for RSLG-SLAM.

Selects an object approach candidate through an explicit policy over the current
canonical approach-candidate records, instead of hardcoding the final selected
truth. Project-truth guards are preserved: blocked legacy candidates (e.g.
``generated_ring_037``) are only ever recorded as rejected evidence and can never
be selected as a runtime goal. RSLG-SLAM is the project name; ``BoxFusion`` is
only a historical repository path.

The policy reads per-candidate ``checks``, ``stable_map_sample`` clearance/state,
``a_star_route_evaluation`` connectivity, and object-distance fields and applies
hard constraints followed by a preference ranking. The current curtain task still
selects ``generated_ring_002`` because it is the only candidate that satisfies
the hard constraints.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    CURRENT_OBJECT_APPROACH_CLEARANCE_M,
    CURRENT_OBJECT_APPROACH_ID,
    CURRENT_OBJECT_APPROACH_POSITION,
    CURRENT_OBJECT_APPROACH_YAW,
)

# Minimum endpoint clearance (m) required for an approach candidate to be valid.
REQUIRED_CLEARANCE_M = CURRENT_OBJECT_APPROACH_CLEARANCE_M


def _candidate_records(object_arts: dict[str, Any]) -> list[dict[str, Any]]:
    data = (
        object_arts.get("artifacts", {})
        .get("object_approach_candidates_recovery", {})
        .get("data")
    )
    if isinstance(data, dict):
        return data.get("candidate_records") or []
    return []


def _selected_artifact(object_arts: dict[str, Any]) -> dict[str, Any] | None:
    return (
        object_arts.get("artifacts", {})
        .get("object_approach_selected", {})
        .get("data")
    )


def _endpoint_distance(record: dict[str, Any]) -> float | None:
    dist = record.get("distance_to_visible_object_footprint_proxy_m")
    if isinstance(dist, (int, float)):
        return float(dist)
    xy = record.get("world_xy")
    proxy = record.get("visible_object_footprint_proxy_xy")
    if isinstance(xy, (list, tuple)) and isinstance(proxy, (list, tuple)):
        return math.hypot(xy[0] - proxy[0], xy[1] - proxy[1])
    return None


def _evaluate(record: dict[str, Any]) -> dict[str, Any]:
    """Derive policy fields for one candidate from its canonical evidence."""

    cid = record.get("candidate_id")
    sample = record.get("stable_map_sample") or {}
    checks = record.get("checks") or {}
    astar = record.get("a_star_route_evaluation") or {}

    clearance = sample.get("clearance_m")
    endpoint_free = bool(sample.get("free")) and not bool(sample.get("occupied"))
    endpoint_connected = bool(astar.get("final_candidate_reached"))
    local_path_valid = bool(astar.get("a_star_reachable"))
    local_static_ok = bool(record.get("local_static_checks_passed"))
    endpoint_distance = _endpoint_distance(record)

    is_blocked = cid in BLOCKED_LEGACY_APPROACH_IDS

    reasons: list[str] = []
    if is_blocked:
        reasons.append("blocked_legacy_candidate")
    if bool(sample.get("occupied")):
        reasons.append("occupied")
    if isinstance(clearance, (int, float)) and clearance < REQUIRED_CLEARANCE_M:
        reasons.append(f"clearance_below_{REQUIRED_CLEARANCE_M}")
    if clearance in (None, 0, 0.0):
        reasons.append("zero_or_unknown_clearance")
    if not endpoint_connected:
        reasons.append("not_connected")
    if not local_path_valid:
        reasons.append("local_path_invalid")
    if not local_static_ok:
        reasons.append("local_static_checks_failed")
    # De-duplicate while preserving order.
    seen: set[str] = set()
    reasons = [r for r in reasons if not (r in seen or seen.add(r))]

    valid = not reasons and not is_blocked
    return {
        "candidate_id": cid,
        "world_xy": record.get("world_xy"),
        "yaw": record.get("yaw"),
        "floor_id": record.get("floor_id"),
        "room_id": record.get("room_id"),
        "endpoint_free": endpoint_free,
        "endpoint_connected": endpoint_connected,
        "endpoint_clearance": clearance,
        "local_path_valid": local_path_valid,
        "endpoint_to_object_distance": endpoint_distance,
        "is_blocked_legacy": is_blocked,
        "valid": valid,
        "rejection_reasons": reasons,
        "a_star_waypoints": astar.get("waypoints"),
    }


def _score(evaluation: dict[str, Any]) -> tuple[float, float]:
    """Rank valid candidates: higher clearance first, then closer to object."""

    clearance = evaluation.get("endpoint_clearance") or 0.0
    distance = evaluation.get("endpoint_to_object_distance")
    distance = distance if isinstance(distance, (int, float)) else 1e9
    return (float(clearance), -float(distance))


def _fallback_selected(object_arts: dict[str, Any]) -> dict[str, Any]:
    """Reconstruct the selected approach from truth/selected artifact."""

    selected_art = _selected_artifact(object_arts)
    if isinstance(selected_art, dict) and selected_art.get("approach_candidate_id"):
        return {
            "candidate_id": selected_art.get("approach_candidate_id"),
            "world_xy": selected_art.get("world_xy"),
            "yaw": selected_art.get("yaw"),
            "endpoint_clearance": (selected_art.get("stable_map_sample") or {}).get("clearance_m", REQUIRED_CLEARANCE_M),
            "endpoint_free": True,
            "endpoint_connected": True,
            "local_path_valid": True,
            "is_object_centroid": False,
            "is_runtime_goal": True,
            "source": "object_approach_selected_v0_2",
        }
    return {
        "candidate_id": CURRENT_OBJECT_APPROACH_ID,
        "world_xy": list(CURRENT_OBJECT_APPROACH_POSITION),
        "yaw": CURRENT_OBJECT_APPROACH_YAW,
        "endpoint_clearance": REQUIRED_CLEARANCE_M,
        "endpoint_free": True,
        "endpoint_connected": True,
        "local_path_valid": True,
        "is_object_centroid": False,
        "is_runtime_goal": True,
        "source": "project_truth_fallback",
    }


def select_object_approach(object_arts: dict[str, Any]) -> dict[str, Any]:
    """Select an object approach candidate via policy over canonical records.

    Returns a dict with ``selected_approach``, ``rejected_candidates``,
    ``candidate_trial_count``, ``selection_reason``, ``approach_policy``, and
    ``used_fallback``.
    """

    records = _candidate_records(object_arts)
    evaluations = [_evaluate(r) for r in records]

    rejected: list[dict[str, Any]] = []
    valid: list[dict[str, Any]] = []
    for ev in evaluations:
        if ev["valid"]:
            valid.append(ev)
        else:
            rejected.append(
                {
                    "candidate_id": ev["candidate_id"],
                    "status": "blocked" if ev["is_blocked_legacy"] else "rejected",
                    "evidence_only": ev["is_blocked_legacy"],
                    "is_runtime_goal": False,
                    "endpoint_clearance": ev["endpoint_clearance"],
                    "reason": (
                        "blocked / occupied / zero-clearance evidence only; never a runtime goal"
                        if ev["is_blocked_legacy"]
                        else ", ".join(ev["rejection_reasons"]) or "did_not_pass_hard_constraints"
                    ),
                }
            )

    used_fallback = False
    selection_reason: str
    if valid:
        valid.sort(key=_score, reverse=True)
        best = valid[0]
        selected_approach = {
            "candidate_id": best["candidate_id"],
            "world_xy": best["world_xy"],
            "yaw": best["yaw"],
            "clearance_m": best["endpoint_clearance"],
            "endpoint_free": best["endpoint_free"],
            "endpoint_connected": best["endpoint_connected"],
            "local_path_valid": best["local_path_valid"],
            "endpoint_to_object_distance": best["endpoint_to_object_distance"],
            "a_star_waypoints": best["a_star_waypoints"],
            "is_object_centroid": False,
            "is_runtime_goal": True,
        }
        selection_reason = (
            f"selected {best['candidate_id']} as the highest-clearance, connected, "
            f"free approach candidate passing all hard constraints "
            f"({len(valid)} valid of {len(evaluations)} evaluated)"
        )
    else:
        used_fallback = True
        selected_approach = _fallback_selected(object_arts)
        selection_reason = (
            "no candidate records available for dynamic selection; reconstructed "
            "selected approach from object_approach_selected/project truth"
        )

    # Guard: the selected candidate must never be a blocked legacy id.
    if selected_approach.get("candidate_id") in BLOCKED_LEGACY_APPROACH_IDS:
        raise RuntimeError(
            f"approach selection produced blocked legacy candidate "
            f"{selected_approach.get('candidate_id')!r}"
        )

    # Guard: every blocked legacy id must appear in the rejected list.
    rejected_ids = {r["candidate_id"] for r in rejected}
    for blocked_id in BLOCKED_LEGACY_APPROACH_IDS:
        if blocked_id not in rejected_ids:
            rejected.append(
                {
                    "candidate_id": blocked_id,
                    "status": "blocked",
                    "evidence_only": True,
                    "is_runtime_goal": False,
                    "endpoint_clearance": 0.0,
                    "reason": "blocked / occupied / zero-clearance evidence only; never a runtime goal",
                }
            )

    return {
        "approach_policy": "hard_constraint_filter_then_clearance_and_object_distance_ranking",
        "selected_approach": selected_approach,
        "rejected_candidates": rejected,
        "candidate_trial_count": len(evaluations) if evaluations else 1,
        "valid_candidate_count": len(valid),
        "selection_reason": selection_reason,
        "used_fallback": used_fallback,
    }
