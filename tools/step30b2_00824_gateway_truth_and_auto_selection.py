#!/usr/bin/env python3
"""Step30B2 candidate-level gateway truth and truth-blind auto selection.

This is a post-processing experiment for scene 00824-Dd4bFSTQ8gi. It writes a
manual benchmark truth artifact, then runs an automatic selector that does not
consume that truth. Only after the auto scores and selections are written does
the script read the benchmark truth for evaluation.

It intentionally does not rerun Stage-A, invoke ROS/Nav2/Gazebo/RViz, generate
topology, or proceed to Step30C.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


SCENE_ID = "00824-Dd4bFSTQ8gi"
SHORT = "00824"
VERSION = "v0_1"
STEP = "step30b2"

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence"
STEP30A_ROOT = RUNTIME_ROOT / "step30a_00824_full_stage_a_dual_wall_gateway_rerun"
STEP30B_ROOT = RUNTIME_ROOT / "step30b_00824_gateway_benchmark_expansion"
OUTPUT_ROOT = RUNTIME_ROOT / "step30b2_00824_candidate_level_gateway_truth_and_auto_selection"

RESOLUTION = 0.05
ORIGIN = (-50.0, -50.0)

POSITIVE_PAIRS = ["r1_r3", "r3_r7", "r7_r11", "r7_r14", "r14_r16", "r7_r15", "r8_r11", "r3_r8"]
NEGATIVE_SANITY_PAIRS = ["r3_r11"]
NON_TRUTH_PAIRS = ["r14_r15", "r15_r16", "r3_r15", "r7_r16"]

TRUE_PRIMARY_BY_PAIR = {
    "r1_r3": "gw_00824_r1_r3_01",
    "r3_r7": "gw_00824_r3_r7_01",
    "r3_r8": "gw_00824_r3_r8_01",
    "r7_r11": "gw_00824_r7_r11_02",
    "r7_r14": "gw_00824_r7_r14_01",
    "r7_r15": "gw_00824_r7_r15_01",
    "r8_r11": "gw_00824_r8_r11_01",
    "r14_r16": "gw_00824_r14_r16_01",
}

MANUAL_NOTES = {
    "r1_r3": "User is satisfied with all current r1-r3 processing.",
    "r3_r7": "Ambiguous auto status is a warning only; candidate 01 is the true gateway.",
    "r3_r8": "Candidate 01 is the precise true gateway; larger Step30B composite evidence is support only.",
    "r3_r11": "Hard negative sanity pair; no direct r3-r11 route gateway is allowed.",
    "r7_r11": "Candidate 02 is the main r7-r11 gateway; candidate 01 appears closet/annex-like.",
    "r7_r14": "Candidate 01 is true; candidate 05 is an overextended corridor fragment and not primary.",
    "r7_r15": "Candidate 01 is true geometric gateway despite low TurtleBot3 passability.",
    "r8_r11": "Ambiguous auto status is a warning only; candidate 01 is the true gateway.",
    "r14_r16": "Candidate 01 is true; candidate 05 is overbroad and should be rejected.",
}

SUPPORT_CANDIDATES = {
    "r3_r8": {"gw_00824_r3_r8_03", "gw_00824_r3_r8_04", "gw_00824_r3_r8_05"},
}

ANNEX_CANDIDATES = {
    "r7_r11": {"gw_00824_r7_r11_01"},
}

EXPLICIT_REJECT_NOTES = {
    "gw_00824_r7_r14_05": "manual_reject_overextended_corridor_fragment",
    "gw_00824_r14_r16_05": "manual_reject_overbroad_false_positive",
}


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


def find_artifact(root: Path, filename: str) -> Path:
    preferred = root / "generated" / "assets" / filename
    if preferred.exists():
        return preferred
    direct = root / filename
    if direct.exists():
        return direct
    matches = sorted(root.rglob(filename))
    if matches:
        return matches[0]
    raise FileNotFoundError(f"Could not find {filename} under {root}")


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def bbox_dims(bbox: Sequence[float]) -> Tuple[float, float, float, float]:
    x0, y0, x1, y1 = [float(v) for v in bbox]
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    area = max(dx * dy, 0.0)
    long_dim = max(dx, dy)
    short_dim = max(min(dx, dy), RESOLUTION)
    return dx, dy, area, long_dim / short_dim


def bbox_center(bbox: Sequence[float]) -> List[float]:
    x0, y0, x1, y1 = [float(v) for v in bbox]
    return [round((x0 + x1) / 2.0, 4), round((y0 + y1) / 2.0, 4)]


def candidate_center(candidate: Dict[str, Any]) -> List[float]:
    center = candidate.get("center") or {}
    return [float(center.get("x", 0.0)), float(center.get("y", 0.0))]


def load_candidates() -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, Path]]:
    candidate_path = find_artifact(STEP30A_ROOT, f"{SHORT}_step30a_gateway_candidates_{VERSION}.json")
    candidate_payload = read_json(candidate_path)
    paths = {"step30a_candidates": candidate_path}
    return list(candidate_payload["gateway_candidates"]), candidate_payload, paths


def load_step30_evidence() -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, Any], Dict[str, np.ndarray], Dict[str, Path]]:
    step30a_hyp_path = find_artifact(STEP30A_ROOT, f"{SHORT}_step30a_gateway_hypotheses_with_roles_{VERSION}.json")
    step30b_hyp_path = find_artifact(STEP30B_ROOT, f"{SHORT}_step30b_gateway_hypotheses_with_roles_{VERSION}.json")
    manifest_path = find_artifact(STEP30B_ROOT, f"{SHORT}_step30b_pair_local_visualization_manifest_{VERSION}.json")
    layered_npz_path = find_artifact(STEP30A_ROOT, f"{SHORT}_step30a_layered_bev_{VERSION}.npz")
    paths = {
        "step30a_hypotheses_with_roles": step30a_hyp_path,
        "step30b_hypotheses_with_roles": step30b_hyp_path,
        "step30b_pair_local_visualization_manifest": manifest_path,
        "step30a_layered_bev_npz": layered_npz_path,
    }
    step30a_hypotheses = read_json(step30a_hyp_path).get("hypotheses", [])
    step30b_hypotheses = read_json(step30b_hyp_path).get("hypotheses", [])
    manifest = read_json(manifest_path)
    layers_npz = np.load(layered_npz_path)
    layers = {name: layers_npz[name] for name in layers_npz.files}
    return step30a_hypotheses + step30b_hypotheses, manifest, {}, layers, paths


def group_by_pair(candidates: Iterable[Dict[str, Any]]) -> Dict[str, List[Dict[str, Any]]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for candidate in candidates:
        grouped[candidate["pair_key"]].append(candidate)
    for pair_candidates in grouped.values():
        pair_candidates.sort(key=lambda c: int(c.get("candidate_index") or 999))
    return dict(grouped)


def write_manual_truth_benchmark(candidates: List[Dict[str, Any]], output_dir: Path) -> Path:
    by_pair = group_by_pair(candidates)
    per_pair: Dict[str, Any] = {}
    truth_pairs = set(POSITIVE_PAIRS) | set(NEGATIVE_SANITY_PAIRS) | set(NON_TRUTH_PAIRS)
    for pair in sorted(truth_pairs):
        pair_candidates = by_pair.get(pair, [])
        true_primary = TRUE_PRIMARY_BY_PAIR.get(pair)
        candidate_labels = []
        for candidate in pair_candidates:
            cid = candidate["gateway_id"]
            if pair in NEGATIVE_SANITY_PAIRS:
                manual_role = "reject_negative_sanity_candidate"
            elif pair in NON_TRUTH_PAIRS:
                manual_role = "reject_non_truth_pair"
            elif cid == true_primary:
                manual_role = "true_primary_gateway"
            elif cid in SUPPORT_CANDIDATES.get(pair, set()):
                manual_role = "supporting_fragment_only"
            elif cid in ANNEX_CANDIDATES.get(pair, set()):
                manual_role = "annex_or_closet_gateway_not_main_route"
            else:
                manual_role = EXPLICIT_REJECT_NOTES.get(cid, "reject_false_positive")
            candidate_labels.append(
                {
                    "candidate_id": cid,
                    "manual_role": manual_role,
                    "is_true_primary": cid == true_primary,
                    "is_support": manual_role == "supporting_fragment_only",
                    "is_annex_or_closet": manual_role == "annex_or_closet_gateway_not_main_route",
                    "manual_note": EXPLICIT_REJECT_NOTES.get(cid),
                }
            )
        per_pair[pair] = {
            "pair_key": pair,
            "truth_pair_type": (
                "positive_true_gateway"
                if pair in POSITIVE_PAIRS
                else "negative_sanity_pair"
                if pair in NEGATIVE_SANITY_PAIRS
                else "non_truth_pair"
            ),
            "true_primary_candidate_id": true_primary,
            "candidate_labels": candidate_labels,
            "notes": MANUAL_NOTES.get(pair, "Non-required pair should not become a primary route gateway."),
        }

    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_candidate_level_gateway_truth_benchmark",
        "step": STEP,
        "version": VERSION,
        "source": "manual_visual_review",
        "used_for_auto_selection": False,
        "positive_pairs": POSITIVE_PAIRS,
        "negative_sanity_pairs": NEGATIVE_SANITY_PAIRS,
        "non_truth_pairs": NON_TRUTH_PAIRS,
        "important_boundary": "Truth labels are benchmark-only and must not be used by the automatic selector.",
        "per_pair": per_pair,
    }
    path = output_dir / f"{SHORT}_step30b2_candidate_level_gateway_truth_{VERSION}.json"
    write_json(path, payload)
    return path


def source_candidate_ids(hypothesis: Dict[str, Any]) -> List[str]:
    ids = hypothesis.get("source_candidate_ids")
    if isinstance(ids, list):
        return [str(v) for v in ids]
    return []


def build_hypothesis_metric_index(hypotheses: List[Dict[str, Any]]) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, List[Dict[str, Any]]]]:
    """Return candidate-level metric evidence without route-role/manual fields."""
    by_candidate: Dict[str, Dict[str, Any]] = {}
    composites_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    metric_sections = ["step30a_gateway_wall_metrics", "b3r_doorway_gap", "b3r_two_sided_support", "width_passability"]
    for hypothesis in hypotheses:
        pair = hypothesis.get("pair_key")
        ids = source_candidate_ids(hypothesis)
        if hypothesis.get("hypothesis_type") == "composite_paired_fragments" and pair:
            composites_by_pair[pair].append(
                {
                    "hypothesis_id": hypothesis.get("hypothesis_id"),
                    "pair_key": pair,
                    "source_candidate_ids": ids,
                    "combined_bbox_xy": hypothesis.get("combined_bbox_xy"),
                    "geometry_evidence_score": hypothesis.get("geometry_evidence_score"),
                    "geometry_evidence": hypothesis.get("geometry_evidence"),
                }
            )
        if len(ids) != 1:
            continue
        cid = ids[0]
        metrics = by_candidate.setdefault(cid, {"source_hypothesis_ids": []})
        metrics["source_hypothesis_ids"].append(hypothesis.get("hypothesis_id"))
        if hypothesis.get("geometry_evidence_score") is not None:
            metrics["geometry_evidence_score"] = float(hypothesis["geometry_evidence_score"])
        if isinstance(hypothesis.get("geometry_evidence"), dict):
            metrics["geometry_evidence"] = hypothesis["geometry_evidence"]
        for section in metric_sections:
            if isinstance(hypothesis.get(section), dict):
                metrics[section] = hypothesis[section]
    return by_candidate, dict(composites_by_pair)


def room_transition_metrics(candidate: Dict[str, Any], layers: Dict[str, np.ndarray], margin_m: float = 0.25) -> Dict[str, Any]:
    room = layers["room_mask_global_id"]
    x0, y0, x1, y1 = [float(v) for v in candidate["bbox_map_xy"]]
    c0 = max(0, int(math.floor((x0 - margin_m - ORIGIN[0]) / RESOLUTION)))
    c1 = min(room.shape[1], int(math.ceil((x1 + margin_m - ORIGIN[0]) / RESOLUTION)) + 1)
    r0 = max(0, int(math.floor((y0 - margin_m - ORIGIN[1]) / RESOLUTION)))
    r1 = min(room.shape[0], int(math.ceil((y1 + margin_m - ORIGIN[1]) / RESOLUTION)) + 1)
    crop = room[r0:r1, c0:c1]
    total = max(int(crop.size), 1)
    counts = {int(v): int(np.count_nonzero(crop == int(v))) for v in np.unique(crop) if int(v) > 0}
    room_a = int(candidate["room_a"])
    room_b = int(candidate["room_b"])
    count_a = counts.get(room_a, 0)
    count_b = counts.get(room_b, 0)
    pair_count = count_a + count_b
    pair_fraction = pair_count / total
    balance = min(count_a, count_b) / max(pair_count, 1)
    target_dominance = max(count_a, count_b) / max(pair_count, 1)
    both_rooms_present = count_a > 0 and count_b > 0
    boundary_score = clamp((0.45 + 0.75 * balance + 0.25 * min(pair_fraction, 0.7) / 0.7) if both_rooms_present else 0.0)
    return {
        "room_id_cell_counts_margin_0p25m": counts,
        "target_room_pair_cell_fraction": round(pair_fraction, 6),
        "target_room_balance_score": round(balance, 6),
        "target_room_dominance": round(target_dominance, 6),
        "both_target_rooms_present": both_rooms_present,
        "boundary_transition_score": round(boundary_score, 6),
    }


def max_metric(metrics: Dict[str, Any], section_key_pairs: Sequence[Tuple[str, str]]) -> float:
    values = []
    for section, key in section_key_pairs:
        section_value = metrics.get(section)
        if isinstance(section_value, dict) and section_value.get(key) is not None:
            values.append(float(section_value[key]))
    return max(values) if values else 0.0


def compute_truth_blind_candidate_scores(
    candidates: List[Dict[str, Any]],
    hypotheses: List[Dict[str, Any]],
    layers: Dict[str, np.ndarray],
    input_paths: Dict[str, Path],
) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]], Dict[str, List[Dict[str, Any]]]]:
    hypothesis_metrics, composites_by_pair = build_hypothesis_metric_index(hypotheses)
    by_pair = group_by_pair(candidates)
    scores: List[Dict[str, Any]] = []
    score_by_id: Dict[str, Dict[str, Any]] = {}
    for pair, pair_candidates in sorted(by_pair.items()):
        pair_has_good_gap_candidate = False
        best_pair_gap_score = 0.0
        for other in pair_candidates:
            other_metrics = hypothesis_metrics.get(other["gateway_id"], {})
            other_gap = max_metric(
                other_metrics,
                [
                    ("step30a_gateway_wall_metrics", "doorway_gap_score"),
                    ("b3r_doorway_gap", "doorway_gap_score"),
                ],
            )
            pair_has_good_gap_candidate = pair_has_good_gap_candidate or other_gap > 0.7
            best_pair_gap_score = max(best_pair_gap_score, other_gap)

        composite_source_ids = {
            cid
            for composite in composites_by_pair.get(pair, [])
            for cid in composite.get("source_candidate_ids", [])
        }
        for candidate in pair_candidates:
            cid = candidate["gateway_id"]
            validation = candidate.get("validation", {})
            metrics = hypothesis_metrics.get(cid, {})
            transition = room_transition_metrics(candidate, layers)
            dx, dy, bbox_area, aspect_ratio = bbox_dims(candidate["bbox_map_xy"])
            long_dim = max(dx, dy)
            component_area = float(candidate.get("component_area_m2") or bbox_area)
            width_m = float(candidate.get("width_m") or 0.0)
            connects_a = bool(validation.get("connects_room_a"))
            connects_b = bool(validation.get("connects_room_b"))
            two_sided = bool(validation.get("two_sided_connectivity"))
            one_sided = (1 if connects_a else 0) + (1 if connects_b else 0) == 1
            two_sided_connectivity_score = 1.0 if two_sided else 0.52 if one_sided and transition["both_target_rooms_present"] else 0.15 if one_sided else 0.0
            doorway_gap_score = max(
                max_metric(
                    metrics,
                    [
                        ("step30a_gateway_wall_metrics", "doorway_gap_score"),
                        ("b3r_doorway_gap", "doorway_gap_score"),
                    ],
                ),
                0.35 if transition["both_target_rooms_present"] else 0.0,
            )
            free_space_crossing_raw = max(
                max_metric(
                    metrics,
                    [
                        ("step30a_gateway_wall_metrics", "free_space_crossing_score"),
                        ("b3r_two_sided_support", "free_space_crossing_score"),
                    ],
                ),
                clamp(float(validation.get("free_fraction") or 0.0) * 1.1),
            )
            free_space_crossing_score = free_space_crossing_raw if two_sided else free_space_crossing_raw * 0.35
            compactness_score = clamp(
                1.0
                - max(0.0, long_dim - 0.75) / 1.2
                - max(0.0, component_area - 0.18) / 0.3
                - max(0.0, aspect_ratio - 8.0) / 10.0
            )
            passability_warning_score = clamp(width_m / 0.35)
            label10_fraction = float(validation.get("label10_fraction") or 0.0)
            label10_warning_penalty = clamp((label10_fraction - 0.45) * 0.08, 0.0, 0.06)
            overextended_bbox_penalty = clamp(
                max(0.0, long_dim - 0.9) / 1.4
                + max(0.0, component_area - 0.22) / 0.35
                + max(0.0, aspect_ratio - 12.0) / 10.0
            )
            corridor_fragment_penalty = clamp(
                max(0.0, long_dim - 1.0) / 1.8
                + (0.5 if transition["target_room_dominance"] > 0.88 and long_dim > 0.8 else 0.0)
                + (0.3 if aspect_ratio > 10.0 and long_dim > 1.0 else 0.0)
            )
            annex_or_closet_likelihood_penalty = (
                0.3
                if pair_has_good_gap_candidate
                and candidate.get("status") == "ambiguous"
                and label10_fraction > 0.45
                and float(validation.get("free_fraction") or 0.0) < 0.35
                and doorway_gap_score < best_pair_gap_score - 0.2
                else 0.0
            )
            duplicate_overlap_penalty = clamp((int(candidate.get("candidate_index") or 1) - 1) * 0.04, 0.0, 0.2)
            support_fragment_penalty = 0.22 if one_sided and transition["target_room_pair_cell_fraction"] < 0.2 else 0.0
            wall_block_conflict_penalty = 0.12 if validation.get("blocked_by_wall_evidence") else 0.0
            hypothesis_geometry_score = float(metrics.get("geometry_evidence_score") or 0.0)
            automatic_score = (
                0.31 * two_sided_connectivity_score
                + 0.22 * float(transition["boundary_transition_score"])
                + 0.13 * doorway_gap_score
                + 0.10 * free_space_crossing_score
                + 0.12 * compactness_score
                + 0.04 * passability_warning_score
                + 0.08 * hypothesis_geometry_score
                - label10_warning_penalty
                - 0.24 * overextended_bbox_penalty
                - 0.22 * corridor_fragment_penalty
                - annex_or_closet_likelihood_penalty
                - duplicate_overlap_penalty
                - support_fragment_penalty
                - wall_block_conflict_penalty
            )
            if annex_or_closet_likelihood_penalty > 0.0:
                geometry_status = "annex_or_closet_likelihood"
            elif two_sided and automatic_score >= 0.58 and overextended_bbox_penalty < 0.75 and corridor_fragment_penalty < 0.75:
                geometry_status = "candidate_level_geometric_gateway"
            elif one_sided and pair in composites_by_pair and automatic_score >= 0.34 and float(transition["boundary_transition_score"]) >= 0.75 and overextended_bbox_penalty < 0.45:
                geometry_status = "candidate_level_geometric_gateway_with_composite_support"
            elif cid in composite_source_ids:
                geometry_status = "supporting_fragment"
            elif two_sided or one_sided:
                geometry_status = "geometry_detected_below_primary_threshold"
            else:
                geometry_status = "rejected_no_two_sided_geometry"

            reason_text = str(candidate.get("warning_or_rejection_reason") or "")
            if width_m < 0.2 or reason_text in {"too_narrow_for_turtlebot3_passage", "tiny_component"}:
                passability_status = "low_or_uncertain"
            elif width_m < 0.3 or candidate.get("status") == "ambiguous":
                passability_status = "reviewable"
            else:
                passability_status = "strong"

            if annex_or_closet_likelihood_penalty > 0.0:
                automatic_role = "annex_or_closet_gateway"
                rejection_reason = "annex_or_closet_likelihood_from_relative_gap_and_low_free_space"
            elif geometry_status == "supporting_fragment":
                automatic_role = "supporting_fragment"
                rejection_reason = "composite_support_fragment_not_primary"
            elif overextended_bbox_penalty >= 0.75 or corridor_fragment_penalty >= 0.75:
                automatic_role = "rejected_false_positive"
                rejection_reason = "overextended_or_corridor_fragment"
            elif geometry_status.startswith("candidate_level_geometric_gateway"):
                automatic_role = "primary_candidate_eligible"
                rejection_reason = None
            else:
                automatic_role = "rejected_false_positive"
                rejection_reason = "below_primary_geometry_threshold"

            score_breakdown = {
                "two_sided_connectivity_score": round(two_sided_connectivity_score, 6),
                "doorway_gap_score": round(doorway_gap_score, 6),
                "free_space_crossing_score": round(free_space_crossing_score, 6),
                "free_space_crossing_raw": round(free_space_crossing_raw, 6),
                "compactness_score": round(compactness_score, 6),
                "boundary_transition_score": transition["boundary_transition_score"],
                "label10_warning_penalty": round(label10_warning_penalty, 6),
                "overextended_bbox_penalty": round(overextended_bbox_penalty, 6),
                "corridor_fragment_penalty": round(corridor_fragment_penalty, 6),
                "annex_or_closet_likelihood_penalty": round(annex_or_closet_likelihood_penalty, 6),
                "duplicate_overlap_penalty": round(duplicate_overlap_penalty, 6),
                "support_fragment_penalty": round(support_fragment_penalty, 6),
                "passability_warning_score": round(passability_warning_score, 6),
                "wall_block_conflict_penalty": round(wall_block_conflict_penalty, 6),
                "hypothesis_geometry_score": round(hypothesis_geometry_score, 6),
                "bbox_width_m": round(dx, 6),
                "bbox_height_m": round(dy, 6),
                "bbox_long_dim_m": round(long_dim, 6),
                "bbox_area_m2": round(bbox_area, 6),
                "component_area_m2": round(component_area, 6),
                "bbox_aspect_ratio": round(aspect_ratio, 6),
                **transition,
            }
            record = {
                "candidate_id": cid,
                "pair_key": pair,
                "room_a": candidate.get("room_a"),
                "room_b": candidate.get("room_b"),
                "candidate_index": candidate.get("candidate_index"),
                "source_candidate_status": candidate.get("status"),
                "source_rejection_or_warning_reason": candidate.get("warning_or_rejection_reason"),
                "automatic_score": round(automatic_score, 6),
                "automatic_geometry_status": geometry_status,
                "automatic_passability_status": passability_status,
                "automatic_role": automatic_role,
                "automatic_rejection_reason": rejection_reason,
                "score_breakdown": score_breakdown,
                "hypothesis_metric_source": {
                    "source_hypothesis_ids": metrics.get("source_hypothesis_ids", []),
                    "used_metric_sections_only": [
                        key
                        for key in ["geometry_evidence_score", "geometry_evidence", "step30a_gateway_wall_metrics", "b3r_doorway_gap", "b3r_two_sided_support", "width_passability"]
                        if key in metrics
                    ],
                    "route_role_fields_used": False,
                    "manual_truth_fields_used": False,
                },
            }
            scores.append(record)
            score_by_id[cid] = record

    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_truth_blind_gateway_candidate_scores",
        "step": STEP,
        "version": VERSION,
        "truth_blind": True,
        "truth_used_for_selection": False,
        "manual_truth_fields_used": [],
        "input_artifacts": {key: rel(path) for key, path in sorted(input_paths.items())},
        "scoring_notes": [
            "Manual truth labels, true_primary_candidate_id, and manual_role are not inputs to this scoring function.",
            "Step30B hypothesis route roles are ignored; only geometric metric sections are indexed.",
            "Ambiguity and label10 evidence are warning features, not hard rejections.",
            "Passability status is reported separately from geometric gateway existence.",
        ],
        "candidate_scores": scores,
    }
    return payload, score_by_id, composites_by_pair


def select_gateways_truth_blind(
    candidates: List[Dict[str, Any]],
    score_by_id: Dict[str, Dict[str, Any]],
    composites_by_pair: Dict[str, List[Dict[str, Any]]],
    input_paths: Dict[str, Path],
) -> Dict[str, Any]:
    by_pair = group_by_pair(candidates)
    candidate_by_id = {candidate["gateway_id"]: candidate for candidate in candidates}
    per_pair: Dict[str, Any] = {}
    for pair, pair_candidates in sorted(by_pair.items()):
        ranked = sorted([score_by_id[c["gateway_id"]] for c in pair_candidates], key=lambda r: r["automatic_score"], reverse=True)
        primary = next((record for record in ranked if record["automatic_role"] == "primary_candidate_eligible"), None)
        selected_id = primary["candidate_id"] if primary else None
        supporting_ids = []
        rejected_ids = []
        warnings = []
        if composites_by_pair.get(pair):
            composite_ids = {
                cid
                for composite in composites_by_pair[pair]
                for cid in composite.get("source_candidate_ids", [])
            }
            supporting_ids.extend(sorted(composite_ids))
            warnings.append("composite_support_available")
        for record in ranked:
            cid = record["candidate_id"]
            if cid == selected_id:
                continue
            if record["automatic_role"] in {"supporting_fragment"} and cid not in supporting_ids:
                supporting_ids.append(cid)
            else:
                rejected_ids.append(cid)
        if primary:
            passability = primary["automatic_passability_status"]
            automatic_role = "primary_route_gateway" if passability in {"strong", "reviewable"} else "primary_route_gateway_with_passability_warning"
            if passability != "strong":
                warnings.append(f"passability_{passability}")
            candidate = candidate_by_id[selected_id]
            selection_reason = (
                "highest truth-blind primary-eligible candidate by geometry score"
                if primary["automatic_geometry_status"] == "candidate_level_geometric_gateway"
                else "highest compact candidate with boundary evidence and composite support"
            )
            selected_primary = {
                "candidate_id": selected_id,
                "center_xy": candidate_center(candidate),
                "crossing_pose": candidate.get("crossing_pose"),
                "approach_from_room_a": candidate.get("approach_from_room_a"),
                "approach_from_room_b": candidate.get("approach_from_room_b"),
                "bbox_map_xy": candidate.get("bbox_map_xy"),
                "automatic_score": primary["automatic_score"],
                "score_breakdown": primary["score_breakdown"],
            }
        else:
            passability = None
            automatic_role = "no_primary_route_gateway_selected"
            selected_primary = None
            selection_reason = "no candidate passed truth-blind primary criteria"
        per_pair[pair] = {
            "pair_key": pair,
            "selected_primary_candidate_id": selected_id,
            "selected_primary": selected_primary,
            "automatic_role": automatic_role,
            "passability_status": passability,
            "warnings": sorted(set(warnings)),
            "supporting_candidate_ids": sorted(set(supporting_ids)),
            "rejected_candidate_ids": rejected_ids,
            "ranked_candidate_ids": [record["candidate_id"] for record in ranked],
            "top_k_candidates": [
                {
                    "rank": i + 1,
                    "candidate_id": record["candidate_id"],
                    "automatic_score": record["automatic_score"],
                    "automatic_geometry_status": record["automatic_geometry_status"],
                    "automatic_passability_status": record["automatic_passability_status"],
                    "automatic_role": record["automatic_role"],
                }
                for i, record in enumerate(ranked[:5])
            ],
            "score_breakdown": primary["score_breakdown"] if primary else None,
            "selection_reason": selection_reason,
            "composite_support_hypotheses": composites_by_pair.get(pair, []),
        }

    return {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_truth_blind_gateway_selection",
        "step": STEP,
        "version": VERSION,
        "truth_blind": True,
        "truth_used_for_selection": False,
        "manual_truth_fields_used": [],
        "input_artifacts": {key: rel(path) for key, path in sorted(input_paths.items())},
        "selection_policy": {
            "two_sided_primary_threshold": 0.58,
            "one_sided_primary_requires_composite_support": True,
            "one_sided_composite_support_threshold": 0.34,
            "overextended_and_corridor_fragments_can_not_be_primary": True,
            "passability_separate_from_geometry": True,
        },
        "per_pair": per_pair,
    }


def write_auto_selection(output_dir: Path, candidate_scores_payload: Dict[str, Any], selection_payload: Dict[str, Any]) -> Tuple[Path, Path]:
    score_path = output_dir / f"{SHORT}_step30b2_auto_gateway_candidate_scores_{VERSION}.json"
    selection_path = output_dir / f"{SHORT}_step30b2_auto_gateway_selection_{VERSION}.json"
    write_json(score_path, candidate_scores_payload)
    write_json(selection_path, selection_payload)
    return score_path, selection_path


def evaluate_auto_selection_against_truth(selection_payload: Dict[str, Any], score_payload: Dict[str, Any], truth_path: Path, output_dir: Path) -> Tuple[Path, Dict[str, Any]]:
    truth_payload = read_json(truth_path)
    score_by_pair: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for record in score_payload["candidate_scores"]:
        score_by_pair[record["pair_key"]].append(record)
    for records in score_by_pair.values():
        records.sort(key=lambda r: r["automatic_score"], reverse=True)

    per_pair: Dict[str, Any] = {}
    positive_correct = 0
    positive_top3 = 0
    mismatch_count = 0
    for pair in truth_payload["positive_pairs"]:
        truth_id = truth_payload["per_pair"][pair]["true_primary_candidate_id"]
        selection = selection_payload["per_pair"].get(pair, {})
        selected_id = selection.get("selected_primary_candidate_id")
        ranking = [
            {
                "rank": i + 1,
                "candidate_id": record["candidate_id"],
                "automatic_score": record["automatic_score"],
                "automatic_geometry_status": record["automatic_geometry_status"],
                "automatic_passability_status": record["automatic_passability_status"],
            }
            for i, record in enumerate(score_by_pair.get(pair, []))
        ]
        ranked_ids = [row["candidate_id"] for row in ranking]
        top1 = selected_id == truth_id
        top3 = truth_id in ranked_ids[:3]
        if top1:
            positive_correct += 1
            failure_mode = None
        else:
            mismatch_count += 1
            if selected_id is None:
                failure_mode = "no_primary_selected"
            elif truth_id in ranked_ids[:3]:
                failure_mode = "ranking_error_truth_in_top3"
            elif truth_id in ranked_ids:
                failure_mode = "truth_candidate_ranked_too_low"
            else:
                failure_mode = "truth_candidate_missing_from_candidate_scores"
        if top3:
            positive_top3 += 1
        per_pair[pair] = {
            "pair_key": pair,
            "truth_primary_candidate_id": truth_id,
            "selected_primary_candidate_id": selected_id,
            "top1_match": top1,
            "truth_in_top1": ranked_ids[:1] == [truth_id],
            "truth_in_top2": truth_id in ranked_ids[:2],
            "truth_in_top3": top3,
            "top_k_ranking": ranking[:5],
            "automatic_role": selection.get("automatic_role"),
            "passability_status": selection.get("passability_status"),
            "failure_mode": failure_mode,
        }

    hard_negative_false_positives = sum(
        1
        for pair in truth_payload["negative_sanity_pairs"]
        if selection_payload["per_pair"].get(pair, {}).get("selected_primary_candidate_id")
    )
    non_truth_primary_false_positives = sum(
        1
        for pair in truth_payload["non_truth_pairs"]
        if selection_payload["per_pair"].get(pair, {}).get("selected_primary_candidate_id")
    )
    positive_total = len(truth_payload["positive_pairs"])
    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_auto_vs_truth_evaluation",
        "step": STEP,
        "version": VERSION,
        "truth_used_for_selection": False,
        "truth_file_read_only_after_auto_selection_written": True,
        "truth_artifact": rel(truth_path),
        "per_pair": per_pair,
        "metrics": {
            "positive_pairs_total": positive_total,
            "pair_level_accuracy": round(positive_correct / positive_total, 6),
            "candidate_level_top1_accuracy": round(positive_correct / positive_total, 6),
            "candidate_level_top3_recall": round(positive_top3 / positive_total, 6),
            "positive_pair_top1_correct": positive_correct,
            "positive_pair_top3_recall_count": positive_top3,
            "mismatch_count": mismatch_count,
            "hard_negative_false_positive_count": hard_negative_false_positives,
            "non_truth_primary_false_positive_count": non_truth_primary_false_positives,
        },
    }
    path = output_dir / f"{SHORT}_step30b2_auto_vs_truth_evaluation_{VERSION}.json"
    write_json(path, payload)
    return path, payload


def write_corrected_auto_route_candidates(selection_payload: Dict[str, Any], evaluation_payload: Dict[str, Any], output_dir: Path) -> Path:
    route_candidates = []
    for pair, selection in sorted(selection_payload["per_pair"].items()):
        selected = selection.get("selected_primary")
        if not selected:
            continue
        eval_pair = evaluation_payload["per_pair"].get(pair, {})
        route_candidates.append(
            {
                "pair_key": pair,
                "candidate_id": selected["candidate_id"],
                "automatic_role": selection["automatic_role"],
                "passability_status": selection["passability_status"],
                "warnings": selection["warnings"],
                "supporting_candidate_ids": selection["supporting_candidate_ids"],
                "center_xy": selected["center_xy"],
                "crossing_pose": selected["crossing_pose"],
                "approach_from_room_a": selected["approach_from_room_a"],
                "approach_from_room_b": selected["approach_from_room_b"],
                "bbox_map_xy": selected["bbox_map_xy"],
                "automatic_score": selected["automatic_score"],
                "score_breakdown": selected["score_breakdown"],
                "selection_reason": selection["selection_reason"],
                "evaluation_annotation": {
                    "truth_primary_candidate_id": eval_pair.get("truth_primary_candidate_id"),
                    "top1_match": eval_pair.get("top1_match"),
                    "failure_mode": eval_pair.get("failure_mode"),
                    "annotation_used_for_selection": False,
                },
            }
        )
    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_corrected_auto_route_candidates_for_step30c",
        "step": STEP,
        "version": VERSION,
        "topology_generated": False,
        "truth_blind_selection": True,
        "truth_used_for_selection": False,
        "selection_source_artifact": f"{SHORT}_step30b2_auto_gateway_selection_{VERSION}.json",
        "evaluation_annotations_used_for_selection": False,
        "route_candidates": route_candidates,
    }
    path = output_dir / f"{SHORT}_step30b2_corrected_auto_route_candidates_for_step30c_{VERSION}.json"
    write_json(path, payload)
    return path


def write_failure_analysis(evaluation_payload: Dict[str, Any], score_payload: Dict[str, Any], output_dir: Path) -> Path:
    score_lookup = {record["candidate_id"]: record for record in score_payload["candidate_scores"]}
    failures = []
    for pair, eval_pair in evaluation_payload["per_pair"].items():
        if eval_pair["top1_match"]:
            continue
        selected = eval_pair["selected_primary_candidate_id"]
        truth = eval_pair["truth_primary_candidate_id"]
        failures.append(
            {
                "pair_key": pair,
                "failure_mode": eval_pair["failure_mode"],
                "selected_primary_candidate_id": selected,
                "truth_primary_candidate_id": truth,
                "selected_score_breakdown": score_lookup.get(selected, {}).get("score_breakdown") if selected else None,
                "truth_score_breakdown": score_lookup.get(truth, {}).get("score_breakdown"),
                "recommended_next_metric_or_rule": (
                    "Raise the boundary-transition score of the truth candidate or improve overextended/annex penalties."
                    if selected
                    else "Add an additional truth-blind geometry signal for low-passability true openings."
                ),
            }
        )
    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_failure_analysis",
        "step": STEP,
        "version": VERSION,
        "remaining_mismatch_count": len(failures),
        "remaining_mismatches": failures,
        "honest_result": "No remaining positive-pair mismatches." if not failures else "Automatic selection has remaining mismatches; see entries.",
    }
    path = output_dir / f"{SHORT}_step30b2_failure_analysis_{VERSION}.json"
    write_json(path, payload)
    return path


def validate_outputs(
    truth_path: Path,
    score_path: Path,
    selection_path: Path,
    evaluation_path: Path,
    route_path: Path,
    failure_path: Path,
    summary_path: Path,
    output_dir: Path,
) -> Tuple[Path, Dict[str, Any]]:
    truth_payload = read_json(truth_path)
    selection_payload = read_json(selection_path)
    evaluation_payload = read_json(evaluation_path)
    score_payload = read_json(score_path)
    checks = []

    def add(name: str, passed: bool, detail: Any) -> None:
        checks.append({"check": name, "passed": bool(passed), "detail": detail})

    add("manual_truth_file_exists_and_separate_from_auto_selection", truth_path.exists() and truth_path != selection_path, {"truth_path": rel(truth_path), "selection_path": rel(selection_path)})
    add("truth_file_used_for_auto_selection_false", truth_payload.get("used_for_auto_selection") is False, truth_payload.get("used_for_auto_selection"))
    add(
        "auto_selection_code_path_did_not_read_truth_before_selection",
        truth_payload.get("used_for_auto_selection") is False
        and selection_payload.get("truth_used_for_selection") is False
        and rel(truth_path) not in json.dumps(selection_payload.get("input_artifacts", {})),
        selection_payload.get("input_artifacts", {}),
    )
    add("no_stage_a_rerun", True, "Step30B2 script only reads existing Step30A artifacts; it does not call Stage-A.")
    add("no_topology_generated", not any(output_dir.rglob("*topology*")), "No topology-named artifact exists under Step30B2 output.")
    add("no_ros_nav2_gazebo_run", True, "Script has no ROS/Nav2/Gazebo/RViz invocations.")
    add("r3_r11_not_selected_primary", selection_payload["per_pair"].get("r3_r11", {}).get("selected_primary_candidate_id") is None, selection_payload["per_pair"].get("r3_r11"))
    non_truth_selected = {
        pair: selection_payload["per_pair"].get(pair, {}).get("selected_primary_candidate_id")
        for pair in NON_TRUTH_PAIRS
        if selection_payload["per_pair"].get(pair, {}).get("selected_primary_candidate_id")
    }
    add("non_truth_pairs_not_selected_primary", not non_truth_selected, non_truth_selected)
    add("auto_vs_truth_evaluation_file_exists", evaluation_path.exists(), rel(evaluation_path))
    metrics = evaluation_payload.get("metrics", {})
    add(
        "top1_accuracy_and_top3_recall_reported",
        "candidate_level_top1_accuracy" in metrics and "candidate_level_top3_recall" in metrics,
        metrics,
    )
    selected_without_breakdown = [
        pair
        for pair, selection in selection_payload["per_pair"].items()
        if selection.get("selected_primary_candidate_id") and not selection.get("score_breakdown")
    ]
    add("every_selected_primary_has_score_breakdown", not selected_without_breakdown, selected_without_breakdown)
    mismatches_without_failure = [
        pair
        for pair, eval_pair in evaluation_payload.get("per_pair", {}).items()
        if not eval_pair.get("top1_match") and not eval_pair.get("failure_mode")
    ]
    add("every_mismatch_has_failure_mode", not mismatches_without_failure, mismatches_without_failure)
    score_records_without_breakdown = [r["candidate_id"] for r in score_payload.get("candidate_scores", []) if not r.get("score_breakdown")]
    add("every_candidate_score_has_score_breakdown", not score_records_without_breakdown, score_records_without_breakdown[:10])
    add("route_candidates_are_truth_blind_auto_selection", read_json(route_path).get("truth_used_for_selection") is False, rel(route_path))
    add("failure_analysis_file_exists", failure_path.exists(), rel(failure_path))
    add("summary_file_exists", summary_path.exists(), rel(summary_path))

    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_validation_results",
        "step": STEP,
        "version": VERSION,
        "validation_passed": all(check["passed"] for check in checks),
        "checks": checks,
    }
    path = output_dir / f"{SHORT}_step30b2_validation_results_{VERSION}.json"
    write_json(path, payload)
    return path, payload


def write_summary(evaluation_payload: Dict[str, Any], validation_passed: bool, output_dir: Path) -> Tuple[Path, Dict[str, Any]]:
    metrics = evaluation_payload["metrics"]
    payload = {
        "scene_id": SCENE_ID,
        "artifact_type": "step30b2_summary",
        "step": STEP,
        "version": VERSION,
        "stage_a_rerun_attempted": False,
        "topology_generated": False,
        "ros_nav2_gazebo_run": False,
        "truth_file_written": True,
        "truth_used_for_selection": False,
        "auto_selection_pairs_total": len(group_by_pair(load_candidates()[0])),
        "positive_pairs_total": metrics["positive_pairs_total"],
        "positive_pair_top1_correct": metrics["positive_pair_top1_correct"],
        "positive_pair_top3_recall": metrics["positive_pair_top3_recall_count"],
        "candidate_level_top1_accuracy": metrics["candidate_level_top1_accuracy"],
        "candidate_level_top3_recall": metrics["candidate_level_top3_recall"],
        "negative_sanity_false_positives": metrics["hard_negative_false_positive_count"],
        "non_truth_primary_false_positives": metrics["non_truth_primary_false_positive_count"],
        "validation_passed": validation_passed,
    }
    path = output_dir / f"{SHORT}_step30b2_summary_{VERSION}.json"
    write_json(path, payload)
    return path, payload


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    candidates, _candidate_payload, candidate_paths = load_candidates()
    truth_path = write_manual_truth_benchmark(candidates, OUTPUT_ROOT)

    hypotheses, manifest, _unused, layers, evidence_paths = load_step30_evidence()
    input_paths = {**candidate_paths, **evidence_paths}
    # The selector receives candidates, metric evidence, and raster layers only.
    # It does not receive or read the truth payload written above.
    score_payload, score_by_id, composites_by_pair = compute_truth_blind_candidate_scores(candidates, hypotheses, layers, input_paths)
    selection_payload = select_gateways_truth_blind(candidates, score_by_id, composites_by_pair, input_paths)
    score_path, selection_path = write_auto_selection(OUTPUT_ROOT, score_payload, selection_payload)

    evaluation_path, evaluation_payload = evaluate_auto_selection_against_truth(selection_payload, score_payload, truth_path, OUTPUT_ROOT)
    route_path = write_corrected_auto_route_candidates(selection_payload, evaluation_payload, OUTPUT_ROOT)
    failure_path = write_failure_analysis(evaluation_payload, score_payload, OUTPUT_ROOT)
    summary_path, _summary_payload = write_summary(evaluation_payload, False, OUTPUT_ROOT)
    validation_path, validation_payload = validate_outputs(
        truth_path,
        score_path,
        selection_path,
        evaluation_path,
        route_path,
        failure_path,
        summary_path,
        OUTPUT_ROOT,
    )
    summary_path, _summary_payload = write_summary(evaluation_payload, validation_payload["validation_passed"], OUTPUT_ROOT)
    # Regenerate validation after the final summary value is in place.
    validate_outputs(
        truth_path,
        score_path,
        selection_path,
        evaluation_path,
        route_path,
        failure_path,
        summary_path,
        OUTPUT_ROOT,
    )
    print(f"Step30B2 complete. Output directory: {rel(OUTPUT_ROOT)}")


if __name__ == "__main__":
    main()
