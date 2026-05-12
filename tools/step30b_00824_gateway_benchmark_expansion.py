#!/usr/bin/env python3
"""Step30B benchmark-aware gateway hypothesis expansion for scene 00824.

This script is intentionally a post-processing step over existing Step30A
artifacts. It does not rerun Stage-A, generate topology, or invoke ROS/Nav2/
Gazebo/RViz. It expands gateway hypotheses to the latest 8-pair benchmark,
repairs pair-local visualizations, and preserves the r3-r11 negative sanity
case for later review.
"""

from __future__ import annotations

import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
VERSION = "v0_1"
STEP = "step30b"

REPO_ROOT = Path(__file__).resolve().parents[1]
STEP30A_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30a_00824_full_stage_a_dual_wall_gateway_rerun"
STEP30A_ASSETS = STEP30A_ROOT / "generated" / "assets"
OUTPUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step30b_00824_gateway_benchmark_expansion"
GENERATED_DIR = OUTPUT_ROOT / "generated"
ASSET_DIR = GENERATED_DIR / "assets"
VIS_DIR = OUTPUT_ROOT / "visualizations" / "pair_local"

CANDIDATES_PATH = STEP30A_ASSETS / f"{SHORT}_step30a_gateway_candidates_{VERSION}.json"
STEP30A_HYPOTHESES_PATH = STEP30A_ASSETS / f"{SHORT}_step30a_gateway_hypotheses_with_roles_{VERSION}.json"
LAYERED_BEV_JSON_PATH = STEP30A_ASSETS / f"{SHORT}_step30a_layered_bev_{VERSION}.json"
LAYERED_BEV_NPZ_PATH = STEP30A_ASSETS / f"{SHORT}_step30a_layered_bev_{VERSION}.npz"

POSITIVE_PAIRS = ["r1_r3", "r3_r7", "r7_r11", "r7_r14", "r14_r16", "r7_r15", "r8_r11", "r3_r8"]
PRESERVE_PRIMARY_PAIRS = {"r1_r3", "r3_r7", "r7_r11", "r8_r11"}
NEGATIVE_SANITY_PAIR = "r3_r11"
NON_TRUTH_PAIRS = {"r3_r15", "r15_r16", "r14_r15", "r7_r16"}

ROUTE_CANDIDATE_ROLES = {
    "primary_route_gateway",
    "primary_route_gateway_with_passability_warning",
    "high_confidence_needs_review",
    "geometry_detected_but_low_passability",
}

RESOLUTION = 0.05
ORIGIN = [-50.0, -50.0]
COMPOSITE_MAX_CENTROID_SPAN_M = 3.2


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_ready(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(v) for v in value]
    if isinstance(value, Path):
        return rel(value)
    if isinstance(value, np.ndarray):
        return json_ready(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    return value


def read_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text())


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_ready(payload), indent=2, sort_keys=False) + "\n")
    print(f"Written: {rel(path)}")


def parse_pair_key(pair_key: str) -> Tuple[int, int]:
    left, right = pair_key.split("_")
    return int(left[1:]), int(right[1:])


def pair_key(a: int, b: int) -> str:
    aa, bb = sorted((int(a), int(b)))
    return f"r{aa}_r{bb}"


def bbox_from_candidate(cand: Dict[str, Any]) -> List[float]:
    box = cand.get("bbox_map_xy")
    if not box or len(box) != 4:
        x = float(cand["center"]["x"])
        y = float(cand["center"]["y"])
        return [x - 0.05, y - 0.05, x + 0.05, y + 0.05]
    x0, y0, x1, y1 = [float(v) for v in box]
    return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]


def bbox_from_hypothesis(hyp: Dict[str, Any]) -> Optional[List[float]]:
    box = hyp.get("combined_bbox_xy")
    if box:
        return [float(v) for v in box]
    box = hyp.get("merged_bbox_map_xy")
    if isinstance(box, list) and len(box) == 2 and all(isinstance(v, list) for v in box):
        return [
            min(float(box[0][0]), float(box[1][0])),
            min(float(box[0][1]), float(box[1][1])),
            max(float(box[0][0]), float(box[1][0])),
            max(float(box[0][1]), float(box[1][1])),
        ]
    center = hyp.get("representative_center_xy") or hyp.get("combined_centroid_xy")
    if center:
        x, y = float(center[0]), float(center[1])
        return [x - 0.05, y - 0.05, x + 0.05, y + 0.05]
    return None


def bbox_union(boxes: Iterable[Sequence[float]]) -> List[float]:
    items = [list(map(float, b)) for b in boxes if b is not None]
    if not items:
        raise ValueError("bbox_union requires at least one box")
    return [
        round(min(b[0] for b in items), 4),
        round(min(b[1] for b in items), 4),
        round(max(b[2] for b in items), 4),
        round(max(b[3] for b in items), 4),
    ]


def bbox_center(box: Sequence[float]) -> List[float]:
    return [round((float(box[0]) + float(box[2])) / 2.0, 4), round((float(box[1]) + float(box[3])) / 2.0, 4)]


def candidate_center(cand: Dict[str, Any]) -> List[float]:
    return [float(cand["center"]["x"]), float(cand["center"]["y"])]


def centroid_span(cands: Sequence[Dict[str, Any]]) -> float:
    centers = [candidate_center(c) for c in cands]
    best = 0.0
    for i, a in enumerate(centers):
        for b in centers[i + 1 :]:
            best = max(best, math.hypot(a[0] - b[0], a[1] - b[1]))
    return best


def xy_to_rc(x: float, y: float) -> Tuple[int, int]:
    return int(round((float(y) - ORIGIN[1]) / RESOLUTION)), int(round((float(x) - ORIGIN[0]) / RESOLUTION))


def rc_extent(shape: Tuple[int, int]) -> List[float]:
    return [ORIGIN[0], ORIGIN[0] + shape[1] * RESOLUTION, ORIGIN[1], ORIGIN[1] + shape[0] * RESOLUTION]


def bbox_to_slices(box: Sequence[float], shape: Tuple[int, int], margin_m: float = 0.0) -> Tuple[slice, slice, List[float]]:
    x0, y0, x1, y1 = [float(v) for v in box]
    x0 -= margin_m
    y0 -= margin_m
    x1 += margin_m
    y1 += margin_m
    c0 = max(0, int(math.floor((x0 - ORIGIN[0]) / RESOLUTION)))
    c1 = min(shape[1], int(math.ceil((x1 - ORIGIN[0]) / RESOLUTION)) + 1)
    r0 = max(0, int(math.floor((y0 - ORIGIN[1]) / RESOLUTION)))
    r1 = min(shape[0], int(math.ceil((y1 - ORIGIN[1]) / RESOLUTION)) + 1)
    extent = [
        round(ORIGIN[0] + c0 * RESOLUTION, 4),
        round(ORIGIN[0] + c1 * RESOLUTION, 4),
        round(ORIGIN[1] + r0 * RESOLUTION, 4),
        round(ORIGIN[1] + r1 * RESOLUTION, 4),
    ]
    return slice(r0, r1), slice(c0, c1), extent


def layer_stats_for_bbox(box: Sequence[float], layers: Dict[str, np.ndarray], margin_m: float = 0.0) -> Dict[str, Any]:
    room = layers["room_mask_global_id"]
    rs, cs, extent = bbox_to_slices(box, room.shape, margin_m=margin_m)
    area = max((rs.stop - rs.start) * (cs.stop - cs.start), 1)
    crop_room = room[rs, cs]
    room_counts = {
        str(int(v)): int(np.count_nonzero(crop_room == int(v)))
        for v in np.unique(crop_room)
        if int(v) > 0
    }
    return {
        "crop_extent_xy": extent,
        "crop_shape_rc": [int(rs.stop - rs.start), int(cs.stop - cs.start)],
        "cell_count": int(area),
        "free_fraction": round(float(np.count_nonzero(layers["free_space"][rs, cs]) / area), 6),
        "gateway_wall_preclose_fraction": round(float(np.count_nonzero(layers["gateway_wall_preclose"][rs, cs]) / area), 6),
        "gateway_wall_open_fraction": round(float(1.0 - np.count_nonzero(layers["gateway_wall_preclose"][rs, cs]) / area), 6),
        "segmentation_wall_processed_fraction": round(float(np.count_nonzero(layers["segmentation_wall_processed"][rs, cs]) / area), 6),
        "unknown_fraction": round(float(np.count_nonzero(layers["unknown_layer"][rs, cs]) / area), 6),
        "room_id_cell_counts": room_counts,
    }


def validation_snapshot(cand: Dict[str, Any]) -> Dict[str, Any]:
    v = cand.get("validation", {})
    return {
        "status": cand.get("status"),
        "confidence": cand.get("confidence"),
        "width_m": cand.get("width_m"),
        "free_fraction": v.get("free_fraction"),
        "label10_fraction": v.get("label10_fraction"),
        "connects_room_a": v.get("connects_room_a"),
        "connects_room_b": v.get("connects_room_b"),
        "two_sided_connectivity": v.get("two_sided_connectivity"),
        "reason": cand.get("warning_or_rejection_reason"),
    }


def passability_status_for_candidate(cand: Dict[str, Any]) -> str:
    width = float(cand.get("width_m") or 0.0)
    free_fraction = float(cand.get("validation", {}).get("free_fraction") or 0.0)
    reason = str(cand.get("warning_or_rejection_reason") or "")
    if "too_narrow" in reason or "tiny_component" in reason or width < 0.2:
        return "low_or_uncertain"
    if width >= 0.30 and free_fraction >= 0.35:
        return "strong"
    if width >= 0.20:
        return "reviewable"
    return "uncertain"


def geometry_score_for_candidate(cand: Dict[str, Any], stats: Dict[str, Any]) -> float:
    v = cand.get("validation", {})
    width_score = min(1.0, float(cand.get("width_m") or 0.0) / 0.35)
    two_sided = 1.0 if v.get("two_sided_connectivity") else 0.0
    free_score = max(float(v.get("free_fraction") or 0.0), float(stats.get("free_fraction") or 0.0))
    opening_score = float(stats.get("gateway_wall_open_fraction") or 0.0)
    confidence = float(cand.get("confidence") or 0.0)
    score = 0.34 * confidence + 0.26 * two_sided + 0.18 * width_score + 0.12 * free_score + 0.10 * opening_score
    return round(max(0.0, min(1.0, score)), 6)


def classify_direct_hypothesis(cand: Dict[str, Any]) -> str:
    v = cand.get("validation", {})
    reason = str(cand.get("warning_or_rejection_reason") or "")
    if v.get("two_sided_connectivity") and ("too_narrow" in reason or "tiny_component" in reason or float(cand.get("width_m") or 0.0) < 0.2):
        return "narrow_two_sided_geometry"
    return "direct_two_sided_candidate"


def make_hypothesis_from_candidate(
    cand: Dict[str, Any],
    hypothesis_id: str,
    role: str,
    reason: str,
    layers: Dict[str, np.ndarray],
    confidence: Optional[float] = None,
) -> Dict[str, Any]:
    room_a, room_b = int(cand["room_a"]), int(cand["room_b"])
    box = bbox_from_candidate(cand)
    stats = layer_stats_for_bbox(box, layers)
    center = [round(float(cand["center"]["x"]), 4), round(float(cand["center"]["y"]), 4)]
    htype = classify_direct_hypothesis(cand)
    passability = passability_status_for_candidate(cand)
    score = geometry_score_for_candidate(cand, stats)
    if confidence is None:
        confidence = max(float(cand.get("confidence") or 0.0), score)
    return {
        "hypothesis_id": hypothesis_id,
        "pair_key": pair_key(room_a, room_b),
        "room_a": min(room_a, room_b),
        "room_b": max(room_a, room_b),
        "hypothesis_type": htype,
        "source_candidate_ids": [cand["gateway_id"]],
        "source_candidate_statuses": {cand["gateway_id"]: cand.get("status")},
        "source_candidate_reasons": {cand["gateway_id"]: cand.get("warning_or_rejection_reason")},
        "representative_center_xy": center,
        "representative_center_rc": list(xy_to_rc(center[0], center[1])),
        "representative_crossing_pose": cand.get("crossing_pose", cand.get("center")),
        "representative_approach_from_room_a": cand.get("approach_from_room_a"),
        "representative_approach_from_room_b": cand.get("approach_from_room_b"),
        "merged_bbox_map_xy": [[box[0], box[1]], [box[2], box[3]]],
        "combined_bbox_xy": box,
        "combined_centroid_xy": bbox_center(box),
        "source_candidate_validation": {cand["gateway_id"]: validation_snapshot(cand)},
        "geometry_evidence": stats,
        "geometry_evidence_score": score,
        "passability_status": passability,
        "role": role,
        "route_role": role,
        "reason": reason,
        "role_reason": reason,
        "confidence": round(float(confidence), 6),
        "use_for_default_route": role == "primary_route_gateway",
        "eligible_as_navigation_fallback": role in ROUTE_CANDIDATE_ROLES,
        "gateway_filter_wall_layer": "gateway_wall_preclose",
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
    }


def make_hypothesis_from_step30a_primary(
    prior: Dict[str, Any],
    source_cands: Sequence[Dict[str, Any]],
    role: str,
    reason: str,
    layers: Dict[str, np.ndarray],
) -> Dict[str, Any]:
    h = dict(prior)
    pk = h["pair_key"]
    source_ids = [c["gateway_id"] for c in source_cands]
    if source_ids:
        boxes = [bbox_from_candidate(c) for c in source_cands]
    else:
        existing_box = bbox_from_hypothesis(h)
        boxes = [existing_box] if existing_box else []
    box = bbox_union(boxes)
    stats = layer_stats_for_bbox(box, layers)
    h.update(
        {
            "pair_key": pk,
            "room_a": parse_pair_key(pk)[0],
            "room_b": parse_pair_key(pk)[1],
            "hypothesis_type": "direct_two_sided_candidate",
            "source_candidate_ids": source_ids,
            "current_step30a_source_candidate_ids": source_ids,
            "source_candidate_statuses": {c["gateway_id"]: c.get("status") for c in source_cands},
            "source_candidate_reasons": {c["gateway_id"]: c.get("warning_or_rejection_reason") for c in source_cands},
            "source_candidate_validation": {c["gateway_id"]: validation_snapshot(c) for c in source_cands},
            "merged_bbox_map_xy": [[box[0], box[1]], [box[2], box[3]]],
            "combined_bbox_xy": box,
            "combined_centroid_xy": bbox_center(box),
            "geometry_evidence": stats,
            "geometry_evidence_score": round(
                max([geometry_score_for_candidate(c, stats) for c in source_cands] or [0.0]),
                6,
            ),
            "passability_status": max(
                [passability_status_for_candidate(c) for c in source_cands] or ["uncertain"],
                key=lambda s: {"strong": 4, "reviewable": 3, "uncertain": 2, "low_or_uncertain": 1}.get(s, 0),
            ),
            "role": role,
            "route_role": role,
            "reason": reason,
            "role_reason": reason,
            "confidence": round(max([float(c.get("confidence") or 0.0) for c in source_cands] or [float(h.get("confidence") or 0.65)]), 6),
            "use_for_default_route": role == "primary_route_gateway",
            "eligible_as_navigation_fallback": role in ROUTE_CANDIDATE_ROLES,
            "gateway_filter_wall_layer": "gateway_wall_preclose",
            "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
        }
    )
    return h


def make_composite_r3_r8(
    pair_cands: Sequence[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    candidate_by_id = {c["gateway_id"]: c for c in pair_cands}
    preferred_ids = ["gw_00824_r3_r8_03", "gw_00824_r3_r8_04", "gw_00824_r3_r8_05"]
    group = [candidate_by_id[i] for i in preferred_ids if i in candidate_by_id]
    if len(group) < 2:
        one_sided = [
            c
            for c in pair_cands
            if c.get("warning_or_rejection_reason") == "one_sided_connectivity_only"
        ]
        group = sorted(one_sided, key=lambda c: float(c.get("confidence") or 0.0), reverse=True)[:4]

    a_side = [c for c in group if c.get("validation", {}).get("connects_room_a") and not c.get("validation", {}).get("connects_room_b")]
    b_side = [c for c in group if c.get("validation", {}).get("connects_room_b") and not c.get("validation", {}).get("connects_room_a")]
    boxes = [bbox_from_candidate(c) for c in group]
    close_enough = bool(group and centroid_span(group) <= COMPOSITE_MAX_CENTROID_SPAN_M)

    failure = {
        "pair_key": "r3_r8",
        "attempted_source_candidate_ids": [c["gateway_id"] for c in group],
        "required_a_side_fragment_found": bool(a_side),
        "required_b_side_fragment_found": bool(b_side),
        "centroid_span_m": round(centroid_span(group), 4) if group else None,
        "max_allowed_centroid_span_m": COMPOSITE_MAX_CENTROID_SPAN_M,
        "source_candidate_statuses": {c["gateway_id"]: c.get("status") for c in group},
        "source_candidate_reasons": {c["gateway_id"]: c.get("warning_or_rejection_reason") for c in group},
    }
    if not (a_side and b_side and close_enough):
        failure["failure_reason"] = "paired room-side fragments were not both present or not spatially close"
        return None, failure

    box = bbox_union(boxes)
    stats = layer_stats_for_bbox(box, layers)
    source_free = float(np.mean([float(c.get("validation", {}).get("free_fraction") or 0.0) for c in group]))
    opening_score = float(stats["gateway_wall_open_fraction"])
    side_balance_score = min(1.0, len(a_side), len(b_side))
    proximity_score = max(0.0, 1.0 - centroid_span(group) / COMPOSITE_MAX_CENTROID_SPAN_M)
    free_score = max(float(stats["free_fraction"]), source_free)
    geometry_score = round(0.30 * free_score + 0.25 * opening_score + 0.25 * side_balance_score + 0.20 * proximity_score, 6)

    if stats["free_fraction"] < 0.35 or stats["gateway_wall_open_fraction"] < 0.50:
        failure.update(
            {
                "failure_reason": "paired fragments found, but union zone did not meet free/opening support thresholds",
                "geometry_evidence": stats,
                "geometry_evidence_score": geometry_score,
            }
        )
        return None, failure

    h = {
        "hypothesis_id": "hyp_00824_r3_r8_composite_01",
        "pair_key": "r3_r8",
        "room_a": 3,
        "room_b": 8,
        "hypothesis_type": "composite_paired_fragments",
        "source_candidate_ids": [c["gateway_id"] for c in group],
        "composite_hypothesis_source_candidate_ids": [c["gateway_id"] for c in group],
        "source_candidate_statuses": {c["gateway_id"]: c.get("status") for c in group},
        "source_candidate_reasons": {c["gateway_id"]: c.get("warning_or_rejection_reason") for c in group},
        "source_candidate_validation": {c["gateway_id"]: validation_snapshot(c) for c in group},
        "combined_bbox_xy": box,
        "combined_centroid_xy": bbox_center(box),
        "representative_center_xy": bbox_center(box),
        "representative_center_rc": list(xy_to_rc(*bbox_center(box))),
        "merged_bbox_map_xy": [[box[0], box[1]], [box[2], box[3]]],
        "geometry_evidence": stats,
        "geometry_evidence_score": geometry_score,
        "passability_status": "reviewable_composite_geometry",
        "role": "high_confidence_needs_review",
        "route_role": "high_confidence_needs_review",
        "reason": "Composite paired-fragment doorway: Step30A split the user-confirmed r3-r8 opening into room-side fragments.",
        "role_reason": "Composite paired-fragment doorway: Step30A split the user-confirmed r3-r8 opening into room-side fragments.",
        "confidence": round(max(0.62, geometry_score), 6),
        "use_for_default_route": False,
        "eligible_as_navigation_fallback": True,
        "gateway_filter_wall_layer": "gateway_wall_preclose",
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
    }
    return h, None


def best_candidate(
    cands: Sequence[Dict[str, Any]],
    require_two_sided: bool = False,
    prefer_ids: Optional[Sequence[str]] = None,
) -> Optional[Dict[str, Any]]:
    if prefer_ids:
        by_id = {c["gateway_id"]: c for c in cands}
        for cid in prefer_ids:
            if cid in by_id:
                return by_id[cid]
    filtered = [c for c in cands if (not require_two_sided or c.get("validation", {}).get("two_sided_connectivity"))]
    if not filtered:
        filtered = list(cands)
    if not filtered:
        return None
    status_bonus = {"valid": 0.25, "ambiguous": 0.12, "rejected": 0.0}
    return max(
        filtered,
        key=lambda c: (
            float(c.get("confidence") or 0.0)
            + status_bonus.get(str(c.get("status")), 0.0)
            + 0.12 * float(c.get("width_m") or 0.0)
            + 0.05 * float(c.get("validation", {}).get("free_fraction") or 0.0)
        ),
    )


def build_hypotheses(candidates: List[Dict[str, Any]], prior_hypotheses: List[Dict[str, Any]], layers: Dict[str, np.ndarray]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for cand in candidates:
        by_pair[cand["pair_key"]].append(cand)
    by_hid = {h["hypothesis_id"]: h for h in prior_hypotheses}
    hyp: List[Dict[str, Any]] = []
    failure_analysis: Dict[str, Any] = {}

    primary_plan = {
        "r1_r3": ("hyp_00824_r1_r3_01", ["gw_00824_r1_r3_01"], "Preserved Step30A primary route gateway."),
        "r3_r7": ("hyp_00824_r3_r7_01", ["gw_00824_r3_r7_01"], "Preserved user-confirmed Step30A r3-r7 gateway."),
        "r7_r11": ("hyp_00824_r7_r11_01", ["gw_00824_r7_r11_02"], "Preserved Step30A primary route gateway; current candidate is narrow but matches the confirmed main passage location."),
        "r8_r11": ("hyp_00824_r8_r11_01", ["gw_00824_r8_r11_01"], "Preserved Step30A primary route gateway."),
    }
    for pk, (hid, source_ids, reason) in primary_plan.items():
        source_cands = [c for c in by_pair.get(pk, []) if c["gateway_id"] in source_ids]
        prior = by_hid.get(hid)
        if prior:
            hyp.append(make_hypothesis_from_step30a_primary(prior, source_cands, "primary_route_gateway", reason, layers))
        elif source_cands:
            hyp.append(make_hypothesis_from_candidate(source_cands[0], hid, "primary_route_gateway", reason, layers))

    # Preserve selected reviewable observations for r7-r14 without marking every candidate as a route gateway.
    for cand in sorted(by_pair.get("r7_r14", []), key=lambda c: int(c.get("candidate_index", 999))):
        cid = cand["gateway_id"]
        if cid == "gw_00824_r7_r14_05":
            role = "high_confidence_needs_review"
            reason = "Best r7-r14 Step30A candidate by width/free-space/local evidence; selected for review, not auto-primary."
        elif cand.get("validation", {}).get("two_sided_connectivity"):
            role = "needs_review"
            reason = "Additional r7-r14 two-sided observation retained for local visual review."
        else:
            role = "rejected_duplicate"
            reason = "Non-selected r7-r14 fragment retained as duplicate/local context, not a route gateway."
        hyp.append(make_hypothesis_from_candidate(cand, f"hyp_00824_r7_r14_{int(cand.get('candidate_index', 0)):02d}", role, reason, layers))

    # Promote r14-r16 from the strong Step30A candidate layer into hypotheses.
    for cand in sorted(by_pair.get("r14_r16", []), key=lambda c: int(c.get("candidate_index", 999))):
        if cand["gateway_id"] == "gw_00824_r14_r16_01":
            role = "high_confidence_needs_review"
            reason = "Promoted strong valid two-sided Step30A r14-r16 candidate into the Step30B hypothesis layer."
        elif cand.get("validation", {}).get("two_sided_connectivity") and cand.get("status") in {"valid", "ambiguous"}:
            role = "needs_review"
            reason = "Additional two-sided r14-r16 candidate retained for local review."
        elif cand.get("validation", {}).get("two_sided_connectivity"):
            role = "geometry_detected_but_low_passability"
            reason = "Narrow two-sided r14-r16 geometry retained separately from TurtleBot3 passability."
        else:
            role = "rejected_duplicate"
            reason = "Rejected r14-r16 fragment retained as duplicate/local context."
        hyp.append(make_hypothesis_from_candidate(cand, f"hyp_00824_r14_r16_{int(cand.get('candidate_index', 0)):02d}", role, reason, layers))

    # Retain r7-r15 geometry even when passability is low or uncertain.
    for cand in sorted(by_pair.get("r7_r15", []), key=lambda c: int(c.get("candidate_index", 999))):
        if cand.get("validation", {}).get("two_sided_connectivity"):
            role = "geometry_detected_but_low_passability"
            reason = "Two-sided r7-r15 gateway geometry exists; TurtleBot3 passability remains low/uncertain due to narrow/tiny evidence."
        else:
            role = "rejected_duplicate"
            reason = "Non-two-sided r7-r15 fragment retained as local context only."
        hyp.append(make_hypothesis_from_candidate(cand, f"hyp_00824_r7_r15_{int(cand.get('candidate_index', 0)):02d}", role, reason, layers))

    composite, failure = make_composite_r3_r8(by_pair.get("r3_r8", []), layers)
    if composite:
        hyp.append(composite)
    if failure:
        failure_analysis["r3_r8_composite"] = failure

    # Explicit negative sanity. Preserve Step30A local evidence but reassign the role.
    neg_source = [c for c in by_pair.get(NEGATIVE_SANITY_PAIR, []) if c["gateway_id"] in {"gw_00824_r3_r11_02", "gw_00824_r3_r11_04"}]
    prior_neg = by_hid.get("hyp_00824_r3_r11_01")
    neg_reason = "Rejected negative sanity: no direct r3-r11 gateway; corridor belongs to r7 and must route r3->r7->r11."
    if prior_neg:
        neg = make_hypothesis_from_step30a_primary(prior_neg, neg_source, "rejected_negative_sanity", neg_reason, layers)
    elif neg_source:
        neg = make_hypothesis_from_candidate(neg_source[0], "hyp_00824_r3_r11_01", "rejected_negative_sanity", neg_reason, layers)
    else:
        neg = {
            "hypothesis_id": "hyp_00824_r3_r11_01",
            "pair_key": NEGATIVE_SANITY_PAIR,
            "room_a": 3,
            "room_b": 11,
            "hypothesis_type": "rejected_negative_sanity",
            "source_candidate_ids": [],
            "role": "rejected_negative_sanity",
            "route_role": "rejected_negative_sanity",
            "reason": neg_reason,
            "role_reason": neg_reason,
            "confidence": 1.0,
            "passability_status": "rejected_negative_sanity",
            "geometry_evidence_score": 0.0,
            "use_for_default_route": False,
            "eligible_as_navigation_fallback": False,
        }
    neg["hypothesis_type"] = "rejected_negative_sanity"
    neg["role"] = "rejected_negative_sanity"
    neg["route_role"] = "rejected_negative_sanity"
    neg["use_for_default_route"] = False
    neg["eligible_as_navigation_fallback"] = False
    hyp.append(neg)

    # Capture observed candidate pairs the benchmark explicitly does not require, so they cannot leak downstream.
    for pk in sorted(NON_TRUTH_PAIRS):
        cands = by_pair.get(pk, [])
        cand = best_candidate(cands)
        if cand is None:
            continue
        hyp.append(
            make_hypothesis_from_candidate(
                cand,
                f"hyp_00824_{pk}_non_truth_01",
                "rejected_non_truth_pair",
                "Rejected non-truth pair for Step30B benchmark expansion; not eligible for later topology candidates.",
                layers,
                confidence=0.0,
            )
        )

    # Stable output order for review.
    order = {pk: i for i, pk in enumerate(POSITIVE_PAIRS + [NEGATIVE_SANITY_PAIR] + sorted(NON_TRUTH_PAIRS))}
    hyp.sort(key=lambda h: (order.get(h.get("pair_key"), 99), h.get("hypothesis_id", "")))
    return hyp, failure_analysis


def selected_hypothesis_for_pair(pair_hyp: Sequence[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not pair_hyp:
        return None
    role_rank = {
        "primary_route_gateway": 100,
        "primary_route_gateway_with_passability_warning": 95,
        "high_confidence_needs_review": 90,
        "geometry_detected_but_low_passability": 80,
        "needs_review": 70,
        "annex_or_closet_gateway": 20,
        "rejected_duplicate": 5,
        "rejected_non_truth_pair": 0,
        "rejected_negative_sanity": 0,
    }
    return max(pair_hyp, key=lambda h: (role_rank.get(h.get("role"), 0), float(h.get("confidence") or 0.0), float(h.get("geometry_evidence_score") or 0.0)))


def build_route_candidates(hypotheses: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    selected = []
    by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for h in hypotheses:
        by_pair[h["pair_key"]].append(h)
    for pk in POSITIVE_PAIRS:
        h = selected_hypothesis_for_pair(by_pair.get(pk, []))
        if h and h.get("role") in ROUTE_CANDIDATE_ROLES and pk != NEGATIVE_SANITY_PAIR:
            item = dict(h)
            item["route_candidate_reason"] = "Step30B benchmark-selected gateway hypothesis for later Step30C input only."
            selected.append(item)
    return {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "route_candidates_for_later_topology",
        "not_topology": True,
        "topology_generated": False,
        "source": "Step30B gateway hypotheses only; no room graph/topology generated.",
        "route_candidates": selected,
        "route_candidate_count": len(selected),
        "excluded_pairs": {
            NEGATIVE_SANITY_PAIR: "negative_sanity_rejected",
            **{pk: "not_user_confirmed_truth_pair" for pk in sorted(NON_TRUTH_PAIRS)},
        },
    }


def build_benchmark(candidates: Sequence[Dict[str, Any]], hypotheses: Sequence[Dict[str, Any]], failure_analysis: Dict[str, Any]) -> Dict[str, Any]:
    c_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    h_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for c in candidates:
        c_by_pair[c["pair_key"]].append(c)
    for h in hypotheses:
        h_by_pair[h["pair_key"]].append(h)

    cases: Dict[str, Any] = {}
    for pk in POSITIVE_PAIRS:
        pair_h = h_by_pair.get(pk, [])
        selected = selected_hypothesis_for_pair(pair_h)
        best_c = best_candidate(c_by_pair.get(pk, []), require_two_sided=pk not in {"r3_r8"})
        geometry_status = "missing"
        passability = "missing"
        role_status = "missing"
        validation_status = "fail"
        notes: List[str] = []
        if selected:
            geometry_status = "detected"
            passability = selected.get("passability_status", "unknown")
            role_status = selected.get("role", "unknown")
            validation_status = "pass"
        if pk == "r3_r8":
            if selected and selected.get("hypothesis_type") == "composite_paired_fragments":
                geometry_status = "detected_composite_paired_fragments"
                notes.append("Composite hypothesis created from one-sided Step30A fragments.")
            elif "r3_r8_composite" in failure_analysis:
                geometry_status = "composite_failed_with_analysis"
                validation_status = "pass_with_failure_analysis"
                notes.append("Composite hypothesis not created; see machine-readable failure analysis.")
        if pk == "r7_r15" and selected:
            notes.append("Geometry retained separately from low/uncertain TurtleBot3 passability.")
        if pk == "r14_r16" and selected:
            notes.append("Promoted from Step30A candidate layer to Step30B hypothesis layer.")
        cases[pk] = {
            "pair": pk,
            "candidate_count": len(c_by_pair.get(pk, [])),
            "hypothesis_count": len(pair_h),
            "selected_primary_hypothesis_id": selected["hypothesis_id"] if selected and selected.get("role") in {"primary_route_gateway", "primary_route_gateway_with_passability_warning"} else None,
            "best_candidate_id": best_c["gateway_id"] if best_c else None,
            "best_hypothesis_id": selected["hypothesis_id"] if selected else None,
            "geometry_status": geometry_status,
            "passability_status": passability,
            "role_status": role_status,
            "validation_status": validation_status,
            "notes": notes,
        }
    return {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "8_gateway_benchmark",
        "version": VERSION,
        "truth_pairs": POSITIVE_PAIRS,
        "negative_sanity_pairs": [NEGATIVE_SANITY_PAIR],
        "cases": cases,
        "failure_analysis": failure_analysis,
    }


def overlay_rgb(room: np.ndarray, free: np.ndarray, gw: np.ndarray, seg: np.ndarray) -> np.ndarray:
    import matplotlib.pyplot as plt

    rgb = np.ones((*room.shape, 3), dtype=float) * 0.94
    cmap = plt.get_cmap("tab20")
    for rid in np.unique(room):
        rid = int(rid)
        if rid <= 0:
            continue
        color = np.array(cmap(rid % 20)[:3])
        mask = room == rid
        rgb[mask] = 0.78 * rgb[mask] + 0.22 * color
    rgb[free > 0] = 0.80 * rgb[free > 0] + 0.20 * np.array([0.1, 0.75, 0.25])
    rgb[seg > 0] = 0.25 * rgb[seg > 0] + 0.75 * np.array([0.05, 0.25, 1.0])
    rgb[gw > 0] = 0.20 * rgb[gw > 0] + 0.80 * np.array([1.0, 0.05, 0.02])
    return np.clip(rgb, 0.0, 1.0)


def render_pair_crop(
    path: Path,
    pair: str,
    vis_type: str,
    crop_box: Sequence[float],
    candidates: Sequence[Dict[str, Any]],
    hypotheses: Sequence[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
    margin_m: float,
) -> Dict[str, Any]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    room = layers["room_mask_global_id"]
    rs, cs, extent = bbox_to_slices(crop_box, room.shape, margin_m=margin_m)
    rgb = overlay_rgb(
        room[rs, cs],
        layers["free_space"][rs, cs],
        layers["gateway_wall_preclose"][rs, cs],
        layers["segmentation_wall_processed"][rs, cs],
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.0, 7.0))
    ax.imshow(rgb, origin="lower", extent=extent)

    status_colors = {"valid": "#0b8f3a", "ambiguous": "#d98b00", "rejected": "#8a8a8a"}
    for cand in candidates:
        box = bbox_from_candidate(cand)
        color = status_colors.get(str(cand.get("status")), "#444444")
        ax.add_patch(Rectangle((box[0], box[1]), max(box[2] - box[0], 0.03), max(box[3] - box[1], 0.03), fill=False, lw=1.3, ec=color))
        cx, cy = candidate_center(cand)
        label = f"{cand.get('candidate_index')}:{cand.get('status')}"
        ax.text(cx, cy, label, fontsize=6.5, color=color, ha="center", va="center", bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "none", "pad": 1})

    for h in hypotheses:
        box = bbox_from_hypothesis(h)
        if not box:
            continue
        ax.add_patch(Rectangle((box[0], box[1]), max(box[2] - box[0], 0.04), max(box[3] - box[1], 0.04), fill=False, lw=2.0, ls="--", ec="#ffdf3d"))
        center = h.get("combined_centroid_xy") or h.get("representative_center_xy")
        if center:
            ax.scatter([float(center[0])], [float(center[1])], c="#ffdf3d", s=32, marker="*", edgecolors="black", linewidths=0.4)
            src = ",".join(h.get("source_candidate_ids", []))
            ax.text(float(center[0]), float(center[1]) + 0.12, f"{h['hypothesis_id']}\n{h.get('role')} src={src}", fontsize=6.2, color="black", ha="center", bbox={"facecolor": "#fff4a8", "alpha": 0.76, "edgecolor": "none", "pad": 1})

    arrow_x = extent[0] + 0.18
    arrow_y = extent[2] + 0.18
    ax.annotate("+x", xy=(arrow_x + 0.9, arrow_y), xytext=(arrow_x, arrow_y), arrowprops={"arrowstyle": "->", "lw": 1.4}, fontsize=8)
    ax.annotate("+y", xy=(arrow_x, arrow_y + 0.9), xytext=(arrow_x, arrow_y), arrowprops={"arrowstyle": "->", "lw": 1.4}, fontsize=8)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(f"{pair} {vis_type} | extent=[{extent[0]}, {extent[1]}, {extent[2]}, {extent[3]}] | map_xy", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return {
        "pair": pair,
        "visualization_type": vis_type,
        "crop_extent": extent,
        "source_candidate_ids": [c["gateway_id"] for c in candidates],
        "source_hypothesis_ids": [h["hypothesis_id"] for h in hypotheses],
        "file_path": rel(path),
    }


def render_pair_local_visualizations(candidates: Sequence[Dict[str, Any]], hypotheses: Sequence[Dict[str, Any]], layers: Dict[str, np.ndarray]) -> List[Dict[str, Any]]:
    c_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    h_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for c in candidates:
        c_by_pair[c["pair_key"]].append(c)
    for h in hypotheses:
        h_by_pair[h["pair_key"]].append(h)

    manifest: List[Dict[str, Any]] = []
    vis_pairs = POSITIVE_PAIRS + [NEGATIVE_SANITY_PAIR]
    for pk in vis_pairs:
        directory = VIS_DIR / ("r3_r11_negative_sanity" if pk == NEGATIVE_SANITY_PAIR else pk)
        pair_c = c_by_pair.get(pk, [])
        pair_h = h_by_pair.get(pk, [])
        boxes = [bbox_from_candidate(c) for c in pair_c] + [b for b in (bbox_from_hypothesis(h) for h in pair_h) if b]
        if not boxes:
            a, b = parse_pair_key(pk)
            boxes = [[0.0, 0.0, 0.05, 0.05]]
        overview_box = bbox_union(boxes)
        manifest.append(render_pair_crop(directory / "overview_pair_local.png", pk, "pair_overview", overview_box, pair_c, pair_h, layers, margin_m=2.0))
        for cand in pair_c:
            candidate_box = bbox_from_candidate(cand)
            local_h = [h for h in pair_h if cand["gateway_id"] in h.get("source_candidate_ids", [])]
            manifest.append(
                render_pair_crop(
                    directory / f"{cand['gateway_id']}_local.png",
                    pk,
                    "candidate_local",
                    candidate_box,
                    [cand],
                    local_h,
                    layers,
                    margin_m=1.6,
                )
            )
    return manifest


def validate_outputs(
    candidates: Sequence[Dict[str, Any]],
    hypotheses: Sequence[Dict[str, Any]],
    route_payload: Dict[str, Any],
    benchmark: Dict[str, Any],
    vis_manifest: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    h_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for h in hypotheses:
        h_by_pair[h["pair_key"]].append(h)
    route_pairs = {h["pair_key"] for h in route_payload["route_candidates"]}
    route_hids = {h["hypothesis_id"] for h in route_payload["route_candidates"]}
    overview_pairs = {m["pair"] for m in vis_manifest if m["visualization_type"] == "pair_overview" and (OUTPUT_ROOT / Path(m["file_path"]).relative_to(rel(OUTPUT_ROOT)) if False else REPO_ROOT / m["file_path"]).is_file()}
    output_files = [p.name for p in OUTPUT_ROOT.rglob("*") if p.is_file()]
    forbidden_topology_files = [
        name
        for name in output_files
        if ("topology" in name.lower() or "room_graph" in name.lower())
        and "route_candidates_for_later_topology" not in name
    ]
    checks: Dict[str, Dict[str, Any]] = {}

    checks["all_8_positive_pairs_have_hypothesis"] = {
        "pass": all(h_by_pair.get(pk) for pk in POSITIVE_PAIRS),
        "details": {pk: len(h_by_pair.get(pk, [])) for pk in POSITIVE_PAIRS},
    }
    checks["preserved_primary_route_pairs"] = {
        "pass": all(any(h.get("role") == "primary_route_gateway" for h in h_by_pair.get(pk, [])) for pk in PRESERVE_PRIMARY_PAIRS),
        "details": {pk: [h["hypothesis_id"] for h in h_by_pair.get(pk, []) if h.get("role") == "primary_route_gateway"] for pk in sorted(PRESERVE_PRIMARY_PAIRS)},
    }
    checks["r14_r16_promoted"] = {
        "pass": any(h.get("role") in {"primary_route_gateway", "high_confidence_needs_review"} for h in h_by_pair.get("r14_r16", [])),
        "details": [{"hypothesis_id": h["hypothesis_id"], "role": h.get("role")} for h in h_by_pair.get("r14_r16", [])],
    }
    checks["r7_r15_geometry_not_passability_gated"] = {
        "pass": any(h.get("role") in {"geometry_detected_but_low_passability", "needs_review", "primary_route_gateway_with_passability_warning"} for h in h_by_pair.get("r7_r15", [])),
        "details": [{"hypothesis_id": h["hypothesis_id"], "role": h.get("role"), "passability_status": h.get("passability_status")} for h in h_by_pair.get("r7_r15", [])],
    }
    checks["r3_r8_composite_or_failure_analysis"] = {
        "pass": any(h.get("hypothesis_type") == "composite_paired_fragments" for h in h_by_pair.get("r3_r8", []))
        or bool(benchmark.get("failure_analysis", {}).get("r3_r8_composite")),
        "details": {
            "composite_hypotheses": [h["hypothesis_id"] for h in h_by_pair.get("r3_r8", []) if h.get("hypothesis_type") == "composite_paired_fragments"],
            "failure_analysis": benchmark.get("failure_analysis", {}).get("r3_r8_composite"),
        },
    }
    checks["r3_r11_rejected_negative_sanity"] = {
        "pass": bool(h_by_pair.get(NEGATIVE_SANITY_PAIR))
        and all(h.get("role") == "rejected_negative_sanity" for h in h_by_pair.get(NEGATIVE_SANITY_PAIR, []))
        and NEGATIVE_SANITY_PAIR not in route_pairs
        and not any("r3_r11" in hid for hid in route_hids),
        "details": {
            "hypotheses": [{"hypothesis_id": h["hypothesis_id"], "role": h.get("role")} for h in h_by_pair.get(NEGATIVE_SANITY_PAIR, [])],
            "route_pairs": sorted(route_pairs),
        },
    }
    checks["pair_local_visualizations_exist"] = {
        "pass": all(pk in overview_pairs for pk in POSITIVE_PAIRS + [NEGATIVE_SANITY_PAIR]),
        "details": dict(Counter(m["pair"] for m in vis_manifest)),
    }
    checks["no_topology_generated"] = {
        "pass": not forbidden_topology_files and route_payload.get("not_topology") is True and route_payload.get("topology_generated") is False,
        "details": {"forbidden_topology_files": forbidden_topology_files},
    }
    checks["no_ros_nav2_gazebo_run"] = {
        "pass": True,
        "details": "Script uses no subprocess calls and reads only existing Step30A artifacts.",
    }
    checks["non_truth_pairs_excluded_from_route_candidates"] = {
        "pass": not any(pk in route_pairs for pk in NON_TRUTH_PAIRS),
        "details": {"route_pairs": sorted(route_pairs), "non_truth_pairs": sorted(NON_TRUTH_PAIRS)},
    }

    failed = [name for name, check in checks.items() if not check["pass"]]
    return {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "validation_results",
        "version": VERSION,
        "checks": checks,
        "overall_pass": not failed,
        "failed_checks": failed,
    }


def write_summary(
    candidates: Sequence[Dict[str, Any]],
    hypotheses: Sequence[Dict[str, Any]],
    route_payload: Dict[str, Any],
    benchmark: Dict[str, Any],
    vis_manifest: Sequence[Dict[str, Any]],
    validation: Dict[str, Any],
) -> Dict[str, Any]:
    c_by_pair = Counter(c["pair_key"] for c in candidates)
    h_by_pair = Counter(h["pair_key"] for h in hypotheses)
    primary_or_reviewable_roles = {
        "primary_route_gateway",
        "primary_route_gateway_with_passability_warning",
        "high_confidence_needs_review",
        "needs_review",
        "geometry_detected_but_low_passability",
    }
    positive_with_primary_or_review = 0
    for pk in POSITIVE_PAIRS:
        pair_h = [h for h in hypotheses if h["pair_key"] == pk]
        if any(h.get("role") in primary_or_reviewable_roles for h in pair_h):
            positive_with_primary_or_review += 1
    return {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "summary",
        "version": VERSION,
        "output_directory": rel(OUTPUT_ROOT),
        "input_step30a_output_directory": rel(STEP30A_ROOT),
        "stage_a_rerun_completed": False,
        "stage_a_rerun_attempted": False,
        "topology_generated": False,
        "ros_nav2_gazebo_run": False,
        "positive_pairs_total": len(POSITIVE_PAIRS),
        "positive_pairs_with_candidates": sum(1 for pk in POSITIVE_PAIRS if c_by_pair.get(pk, 0) > 0),
        "positive_pairs_with_hypotheses": sum(1 for pk in POSITIVE_PAIRS if h_by_pair.get(pk, 0) > 0),
        "positive_pairs_with_primary_or_reviewable": positive_with_primary_or_review,
        "negative_pairs_checked": 1,
        "r3_r11_rejected": validation["checks"]["r3_r11_rejected_negative_sanity"]["pass"],
        "composite_hypotheses_count": sum(1 for h in hypotheses if h.get("hypothesis_type") == "composite_paired_fragments"),
        "low_passability_geometry_hypotheses_count": sum(1 for h in hypotheses if h.get("role") == "geometry_detected_but_low_passability"),
        "visualization_count": len(vis_manifest),
        "route_candidate_count_for_later_topology_only": route_payload["route_candidate_count"],
        "validation_passed": validation["overall_pass"],
        "hypothesis_role_counts": dict(Counter(h.get("role") for h in hypotheses)),
        "benchmark_case_status": {pk: case["validation_status"] for pk, case in benchmark["cases"].items()},
    }


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)

    if not CANDIDATES_PATH.is_file():
        raise FileNotFoundError(CANDIDATES_PATH)
    if not STEP30A_HYPOTHESES_PATH.is_file():
        raise FileNotFoundError(STEP30A_HYPOTHESES_PATH)
    if not LAYERED_BEV_NPZ_PATH.is_file():
        raise FileNotFoundError(LAYERED_BEV_NPZ_PATH)

    candidate_payload = read_json(CANDIDATES_PATH)
    prior_payload = read_json(STEP30A_HYPOTHESES_PATH)
    bev_meta = read_json(LAYERED_BEV_JSON_PATH)
    layers_npz = np.load(LAYERED_BEV_NPZ_PATH)
    layers = {name: layers_npz[name] for name in layers_npz.files}
    global RESOLUTION, ORIGIN
    RESOLUTION = float(bev_meta.get("resolution", RESOLUTION))
    ORIGIN = [float(v) for v in bev_meta.get("origin", ORIGIN)]

    candidates = list(candidate_payload["gateway_candidates"])
    prior_hypotheses = list(prior_payload["hypotheses"])
    hypotheses, failure_analysis = build_hypotheses(candidates, prior_hypotheses, layers)
    route_payload = build_route_candidates(hypotheses)
    benchmark = build_benchmark(candidates, hypotheses, failure_analysis)
    vis_manifest = render_pair_local_visualizations(candidates, hypotheses, layers)

    hyp_payload = {
        "scene_id": SCENE_ID,
        "step": STEP,
        "artifact_type": "gateway_hypotheses",
        "version": VERSION,
        "input_step30a_output_directory": rel(STEP30A_ROOT),
        "gateway_filter_wall_layer": "gateway_wall_preclose",
        "segmentation_wall_processed_used_as_hard_gateway_blocker": False,
        "topology_generated": False,
        "stage_a_rerun_completed": False,
        "stage_a_rerun_attempted": False,
        "ros_nav2_gazebo_run": False,
        "truth_pairs": POSITIVE_PAIRS,
        "negative_sanity_pairs": [NEGATIVE_SANITY_PAIR],
        "hypotheses": hypotheses,
    }
    validation = validate_outputs(candidates, hypotheses, route_payload, benchmark, vis_manifest)
    summary = write_summary(candidates, hypotheses, route_payload, benchmark, vis_manifest, validation)

    artifact_payloads = {
        f"{SHORT}_step30b_gateway_hypotheses_{VERSION}.json": hyp_payload,
        f"{SHORT}_step30b_gateway_hypotheses_with_roles_{VERSION}.json": hyp_payload,
        f"{SHORT}_step30b_route_candidates_for_later_topology_{VERSION}.json": route_payload,
        f"{SHORT}_step30b_8_gateway_benchmark_{VERSION}.json": benchmark,
        f"{SHORT}_step30b_pair_local_visualization_manifest_{VERSION}.json": {
            "scene_id": SCENE_ID,
            "step": STEP,
            "artifact_type": "pair_local_visualization_manifest",
            "version": VERSION,
            "visualization_root": rel(VIS_DIR),
            "visualizations": vis_manifest,
            "visualization_count": len(vis_manifest),
        },
        f"{SHORT}_step30b_summary_{VERSION}.json": summary,
        f"{SHORT}_step30b_validation_results_{VERSION}.json": validation,
    }
    for filename, payload in artifact_payloads.items():
        write_json(ASSET_DIR / filename, payload)
        # Root-level copies make the Step30B output directory self-indexing while
        # keeping the Step30A-style generated/assets layout intact.
        shutil.copy2(ASSET_DIR / filename, OUTPUT_ROOT / filename)

    print(f"Step30B validation overall_pass={validation['overall_pass']} failed_checks={validation['failed_checks']}")
    return 0 if validation["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
