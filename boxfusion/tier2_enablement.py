from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


POLICY_VERSION = "tier2a-enablement-v1"
SUPPORTED_MODES = {"off", "manual", "auto"}
AUTO_OUTSIDE_BOUNDARY_DELTA_MAX = 0.001
AUTO_MAX_CUT_SUPPORT = 0.15
AUTO_MIN_SHORT_LIVED_FRAGMENTS = 2
AUTO_MIN_SMALL_NEWBORNS = 4


def parse_roi(roi_value: Any) -> Optional[Tuple[int, int, int, int]]:
    if roi_value is None:
        return None
    if not isinstance(roi_value, (list, tuple)) or len(roi_value) != 4:
        return None
    x0, y0, x1, y1 = [int(v) for v in roi_value]
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def clip_roi_to_shape(roi: Optional[Tuple[int, int, int, int]], shape) -> Optional[Tuple[int, int, int, int]]:
    if roi is None:
        return None
    h, w = shape
    x0, y0, x1, y1 = roi
    x0 = max(0, min(w, x0))
    x1 = max(0, min(w, x1))
    y0 = max(0, min(h, y0))
    y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def _signal(name: str, detail: str, value: Any = None) -> Dict[str, Any]:
    row: Dict[str, Any] = {"name": name, "detail": detail}
    if value is not None:
        row["value"] = value
    return row


def _resolve_mode(room_cfg: Dict[str, Any]) -> str:
    legacy_enabled = bool(room_cfg.get("tier2_enabled", False))
    mode = str(room_cfg.get("tier2_enablement_mode", "")).strip().lower()
    if mode not in SUPPORTED_MODES:
        return "manual" if legacy_enabled else "off"
    return mode


def _infer_scene_id(config: Optional[Dict[str, Any]], room_cfg: Dict[str, Any]) -> str:
    scene_override = room_cfg.get("tier2_policy_scene_id")
    if scene_override:
        return str(scene_override)

    data_dir = ((config or {}).get("data") or {}).get("datadir")
    if data_dir:
        path = Path(str(data_dir).rstrip("/"))
        if path.name:
            return path.name

    dataset_name = (config or {}).get("dataset")
    if dataset_name:
        return str(dataset_name)
    return "unknown_scene"


def _resolve_artifact_path(room_cfg: Dict[str, Any]) -> Optional[Path]:
    artifact = room_cfg.get("tier2_policy_artifact")
    if not artifact:
        return None
    path = Path(str(artifact)).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    return path


def _base_decision(config: Optional[Dict[str, Any]], room_cfg: Dict[str, Any]) -> Dict[str, Any]:
    mode = _resolve_mode(room_cfg)
    legacy_enabled = bool(room_cfg.get("tier2_enabled", False))
    manual_enable = bool(room_cfg.get("tier2_manual_enable", legacy_enabled))
    selected_roi = parse_roi(room_cfg.get("tier2_central_roi"))
    artifact_path = _resolve_artifact_path(room_cfg)

    return {
        "policy_kind": "tier2a_enablement_decision",
        "policy_version": POLICY_VERSION,
        "scene_id": _infer_scene_id(config, room_cfg),
        "mode": mode,
        "requested_manual_enable": manual_enable,
        "requested_roi": list(selected_roi) if selected_roi is not None else None,
        "selected_roi": list(selected_roi) if selected_roi is not None else None,
        "effective_roi": None,
        "was_enabled": False,
        "auto_enable_checklist_passed": False,
        "positive_signals": [],
        "blocking_signals": [],
        "final_decision_reason": "",
        "confidence": "low",
        "notes": [],
        "recommendation": {
            "enable_recommended": False,
            "recommended_mode": "off",
            "rationale": "",
            "manual_review_items": [],
        },
        "artifact_path": str(artifact_path) if artifact_path is not None else None,
        "artifact_kind": None,
    }


def evaluate_tier2_enablement(config: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    room_cfg = dict((config or {}).get("room_segmentation", config or {}))
    decision = _base_decision(config, room_cfg)
    mode = decision["mode"]
    selected_roi = parse_roi(decision.get("selected_roi"))

    if mode == "off":
        decision["blocking_signals"].append(_signal("mode_off", "Tier 2A is disabled by policy mode."))
        decision["final_decision_reason"] = "enablement_mode_off"
        decision["recommendation"] = {
            "enable_recommended": False,
            "recommended_mode": "off",
            "rationale": "Tier 2A remains disabled unless a scene-specific manual or auto policy is requested.",
            "manual_review_items": [],
        }
        return decision

    if mode == "manual":
        if not decision["requested_manual_enable"]:
            decision["blocking_signals"].append(
                _signal("manual_not_armed", "Manual mode requires tier2_manual_enable=true.")
            )
        else:
            decision["positive_signals"].append(
                _signal("manual_explicit_enable", "Manual mode was explicitly armed in config.")
            )
        if selected_roi is None:
            decision["blocking_signals"].append(
                _signal("missing_manual_roi", "Manual mode requires a valid tier2_central_roi.")
            )
        else:
            decision["positive_signals"].append(
                _signal("manual_roi_present", "Manual mode received an explicit Tier 2A ROI.", list(selected_roi))
            )

        decision["was_enabled"] = bool(decision["requested_manual_enable"] and selected_roi is not None)
        decision["final_decision_reason"] = (
            "manual_enable_with_valid_roi" if decision["was_enabled"] else "manual_mode_requires_explicit_enable_and_valid_roi"
        )
        decision["confidence"] = "high" if decision["was_enabled"] else "medium"
        decision["recommendation"] = {
            "enable_recommended": bool(selected_roi is not None),
            "recommended_mode": "manual" if selected_roi is not None else "off",
            "rationale": (
                "Manual mode is appropriate when a human has already identified a trustworthy local ROI."
                if selected_roi is not None
                else "No valid ROI is configured, so manual enablement is not actionable."
            ),
            "manual_review_items": [
                "Confirm the ROI is tightly localized to the unsupported micro-split hotspot.",
                "Check that the ROI does not cover a doorway or broad room boundary change.",
                "Verify the baseline issue is local and repeatedly reappears in the same region.",
            ] if selected_roi is not None else [],
        }
        return decision

    return _evaluate_auto_mode(decision)


def finalize_tier2_enablement_for_shape(decision: Dict[str, Any], shape) -> Dict[str, Any]:
    finalized = copy.deepcopy(decision)
    requested_roi = parse_roi(finalized.get("selected_roi"))
    clipped_roi = clip_roi_to_shape(requested_roi, shape)
    finalized["effective_roi"] = list(clipped_roi) if clipped_roi is not None else None
    finalized["shape"] = [int(shape[0]), int(shape[1])]

    requested_enablement = bool(
        finalized["mode"] == "manual" and finalized.get("requested_manual_enable")
        or finalized["mode"] == "auto" and finalized.get("auto_enable_checklist_passed")
    )

    if requested_enablement and clipped_roi is None:
        finalized["was_enabled"] = False
        finalized["blocking_signals"].append(
            _signal("roi_not_valid_for_current_shape", "The selected ROI does not intersect the current map shape.")
        )
        finalized["final_decision_reason"] = "selected_roi_invalid_for_current_shape"
    else:
        finalized["was_enabled"] = bool(requested_enablement and clipped_roi is not None)
    return finalized


def _evaluate_auto_mode(decision: Dict[str, Any]) -> Dict[str, Any]:
    artifact_path = decision.get("artifact_path")
    if not artifact_path:
        decision["blocking_signals"].append(
            _signal("missing_policy_artifact", "Auto mode requires tier2_policy_artifact to point to prior analysis output.")
        )
        decision["final_decision_reason"] = "auto_mode_requires_policy_artifact"
        decision["recommendation"] = {
            "enable_recommended": False,
            "recommended_mode": "off",
            "rationale": "No analysis artifact was provided, so auto mode stays off.",
            "manual_review_items": [],
        }
        return decision

    path = Path(str(artifact_path))
    if not path.exists():
        decision["blocking_signals"].append(
            _signal("missing_policy_artifact_file", f"Auto policy artifact was not found: {path}")
        )
        decision["final_decision_reason"] = "auto_policy_artifact_not_found"
        decision["recommendation"] = {
            "enable_recommended": False,
            "recommended_mode": "off",
            "rationale": "The configured analysis artifact path is missing, so auto mode stays off.",
            "manual_review_items": [],
        }
        return decision

    with path.open("r", encoding="utf-8") as f:
        artifact = json.load(f)

    if artifact.get("policy_kind") == "tier2a_enablement_decision":
        return _evaluate_auto_from_prior_decision(decision, artifact, path)
    if isinstance(artifact.get("scenes"), list):
        return _evaluate_auto_from_cross_scene_summary(decision, artifact, path)

    decision["blocking_signals"].append(
        _signal("unsupported_policy_artifact", f"Unsupported auto policy artifact format: {path}")
    )
    decision["final_decision_reason"] = "unsupported_auto_policy_artifact"
    decision["recommendation"] = {
        "enable_recommended": False,
        "recommended_mode": "off",
        "rationale": "Auto mode only understands Tier 2A decision JSON or the cross-scene summary JSON.",
        "manual_review_items": [],
    }
    return decision


def _evaluate_auto_from_prior_decision(decision: Dict[str, Any], artifact: Dict[str, Any], path: Path) -> Dict[str, Any]:
    decision["artifact_kind"] = "tier2a_enablement_decision"
    if str(artifact.get("scene_id")) != str(decision.get("scene_id")):
        decision["blocking_signals"].append(
            _signal(
                "scene_not_found_in_artifact",
                f"Policy artifact scene_id={artifact.get('scene_id')} does not match requested scene_id={decision.get('scene_id')}.",
            )
        )
        decision["final_decision_reason"] = "scene_mismatch_in_policy_artifact"
        return decision

    decision["selected_roi"] = artifact.get("selected_roi") or artifact.get("effective_roi")
    decision["positive_signals"] = list(artifact.get("positive_signals", []))
    decision["blocking_signals"] = list(artifact.get("blocking_signals", []))
    decision["auto_enable_checklist_passed"] = bool(artifact.get("auto_enable_checklist_passed", False))
    decision["was_enabled"] = bool(artifact.get("auto_enable_checklist_passed", False))
    decision["final_decision_reason"] = str(artifact.get("final_decision_reason", "auto_decision_reused"))
    decision["confidence"] = str(artifact.get("confidence", "medium"))
    decision["notes"] = list(artifact.get("notes", []))
    decision["recommendation"] = dict(artifact.get("recommendation", {}))
    decision["notes"].append(f"Auto decision reused prior Tier 2A enablement artifact: {path}")
    return decision


def _evaluate_auto_from_cross_scene_summary(decision: Dict[str, Any], artifact: Dict[str, Any], path: Path) -> Dict[str, Any]:
    decision["artifact_kind"] = "tier2_cross_scene_summary"
    scene_id = str(decision.get("scene_id"))
    scene_metrics = next((row for row in artifact.get("scenes", []) if str(row.get("scene_id")) == scene_id), None)
    if scene_metrics is None:
        decision["blocking_signals"].append(
            _signal("scene_not_found_in_summary", f"No cross-scene summary row found for scene_id={scene_id}.")
        )
        decision["final_decision_reason"] = "scene_not_present_in_cross_scene_summary"
        decision["recommendation"] = {
            "enable_recommended": False,
            "recommended_mode": "off",
            "rationale": "Auto mode stays off when the requested scene has no summary-backed policy record.",
            "manual_review_items": [],
        }
        return decision

    decision["selected_roi"] = scene_metrics.get("selected_roi")
    positives = []
    blocking = []
    notes = []

    roi = parse_roi(scene_metrics.get("selected_roi"))
    if roi is not None:
        positives.append(_signal("roi_candidate_available", "Cross-scene analysis provided a scene-specific ROI.", list(roi)))
    else:
        blocking.append(_signal("missing_roi_candidate", "Cross-scene analysis did not provide a valid ROI."))

    roi_meta = scene_metrics.get("roi_metadata", {})
    peak_value = float(roi_meta.get("peak_value", 0.0) or 0.0)
    hot_pixels = int(roi_meta.get("heatmap_nonzero_pixels", 0) or 0)
    if peak_value > 0.0 and hot_pixels > 0:
        positives.append(
            _signal(
                "tight_local_instability_roi",
                "Baseline heatmap concentrated instability into a localized ROI.",
                {"peak_value": round(peak_value, 6), "heatmap_nonzero_pixels": hot_pixels},
            )
        )
    else:
        blocking.append(_signal("no_local_instability_hotspot", "Baseline artifacts did not expose a strong local instability ROI."))

    short_lived = int(scene_metrics.get("roi_short_lived_fragments_baseline", 0) or 0)
    small_newborn = int(scene_metrics.get("roi_small_newborn_regions_baseline", 0) or 0)
    if short_lived >= AUTO_MIN_SHORT_LIVED_FRAGMENTS or small_newborn >= AUTO_MIN_SMALL_NEWBORNS:
        positives.append(
            _signal(
                "repeated_small_local_fragments",
                "Baseline ROI shows repeated short-lived or small newborn fragments.",
                {"short_lived_fragments": short_lived, "small_newborn_regions": small_newborn},
            )
        )
    else:
        blocking.append(
            _signal(
                "insufficient_repeated_fragment_evidence",
                "Baseline ROI does not show enough repeated local micro-fragment activity.",
                {"short_lived_fragments": short_lived, "small_newborn_regions": small_newborn},
            )
        )

    baseline_repair_demand = int(scene_metrics.get("post_repair_count_baseline", 0) or 0)
    cut_support_mean = (scene_metrics.get("cut_support_repaired_candidates") or {}).get("mean")
    cut_support_mean = float(cut_support_mean) if cut_support_mean is not None else None
    if baseline_repair_demand > 0 and cut_support_mean is not None and cut_support_mean <= AUTO_MAX_CUT_SUPPORT:
        positives.append(
            _signal(
                "unsupported_local_split_evidence",
                "Baseline repair demand is present and the repaired cuts look weakly wall-supported.",
                {"baseline_repair_demand": baseline_repair_demand, "mean_cut_support": round(cut_support_mean, 6)},
            )
        )
    else:
        blocking.append(
            _signal(
                "weak_unsupported_split_evidence",
                "Baseline artifacts do not show a strong unsupported local split signature.",
                {"baseline_repair_demand": baseline_repair_demand, "mean_cut_support": cut_support_mean},
            )
        )

    suppressions_inside = int(scene_metrics.get("cooldown_suppressions_inside_roi", 0) or 0)
    prevented_repairs = int(scene_metrics.get("roi_local_repairs_prevented_earlier_by_cooldown", 0) or 0)
    if suppressions_inside > 0 and prevented_repairs > 0:
        positives.append(
            _signal(
                "local_cooldown_effect_confirmed",
                "Cross-scene replay shows ROI-local suppressions that reduced later repair demand.",
                {"cooldown_suppressions_inside_roi": suppressions_inside, "repairs_prevented": prevented_repairs},
            )
        )
    else:
        blocking.append(
            _signal(
                "cooldown_effect_not_proven",
                "Auto mode requires clear evidence that ROI-local suppression helped in this scene.",
                {"cooldown_suppressions_inside_roi": suppressions_inside, "repairs_prevented": prevented_repairs},
            )
        )

    outside_delta = float(scene_metrics.get("outside_roi_repaired_boundary_delta", 0.0) or 0.0)
    if outside_delta <= AUTO_OUTSIDE_BOUNDARY_DELTA_MAX:
        positives.append(
            _signal(
                "outside_roi_stable",
                "Patched replay stayed stable outside the ROI.",
                {"outside_roi_repaired_boundary_delta": round(outside_delta, 6)},
            )
        )
    else:
        blocking.append(
            _signal(
                "outside_roi_sensitivity",
                "Patched replay changed repaired boundaries outside the ROI beyond the conservative allowance.",
                {"outside_roi_repaired_boundary_delta": round(outside_delta, 6)},
            )
        )

    room_count_regression = bool(scene_metrics.get("room_count_regression_flag", False))
    if not room_count_regression:
        positives.append(_signal("no_room_count_regression", "Cross-scene replay did not flag room-count regression."))
    else:
        blocking.append(
            _signal(
                "room_count_regression",
                "Cross-scene replay flagged room-count or topology sensitivity.",
                scene_metrics.get("room_count_diverged_frames", []),
            )
        )

    remaining = scene_metrics.get("remaining_case_characterization", {})
    dominant_remaining = str(remaining.get("dominant_remaining_case", ""))
    if dominant_remaining != "strong_seed_safe_exemptions":
        positives.append(
            _signal(
                "remaining_cases_not_strong_seed_dominated",
                "Remaining cases are not dominated by strong mature safe exemptions.",
                dominant_remaining,
            )
        )
    else:
        blocking.append(
            _signal(
                "strong_seed_dominated_remaining_cases",
                "Remaining cases are dominated by strong or mature exemptions, which is a conservative auto-enable blocker.",
                dominant_remaining,
            )
        )

    manual_recommended = bool(
        roi is not None
        and peak_value > 0.0
        and (short_lived >= AUTO_MIN_SHORT_LIVED_FRAGMENTS or small_newborn >= AUTO_MIN_SMALL_NEWBORNS)
        and baseline_repair_demand > 0
    )
    auto_passed = len(blocking) == 0

    if room_count_regression and manual_recommended:
        notes.append(
            "Auto mode stayed off because the scene still showed room-count/topology sensitivity, but the local ROI evidence is strong enough to justify a human-reviewed manual trial."
        )
    if not auto_passed and suppressions_inside > 0 and prevented_repairs == 0:
        notes.append(
            "Suppression activity alone is not enough for auto mode; it also requires evidence that later repair demand was reduced."
        )

    decision["positive_signals"] = positives
    decision["blocking_signals"] = blocking
    decision["auto_enable_checklist_passed"] = auto_passed
    decision["was_enabled"] = auto_passed
    decision["confidence"] = "high" if auto_passed else ("medium" if manual_recommended else "low")
    decision["notes"] = notes + [f"Auto policy evaluated from cross-scene summary: {path}"]
    decision["final_decision_reason"] = "auto_enable_checklist_passed" if auto_passed else "auto_enable_checklist_blocked"
    decision["recommendation"] = {
        "enable_recommended": manual_recommended,
        "recommended_mode": "auto" if auto_passed else ("manual" if manual_recommended else "off"),
        "rationale": (
            "All conservative auto-enable checks passed."
            if auto_passed
            else (
                "Manual mode is reasonable because the issue is localized and repeatable, but auto mode remains blocked by safety checks."
                if manual_recommended
                else "Keep Tier 2A off because the scene evidence is incomplete or not clearly favorable."
            )
        ),
        "manual_review_items": _manual_review_items(scene_metrics),
    }
    return decision


def _manual_review_items(scene_metrics: Dict[str, Any]) -> list[str]:
    roi = scene_metrics.get("selected_roi")
    remaining = scene_metrics.get("remaining_case_characterization", {})
    return [
        f"Inspect the candidate ROI overlay and confirm the hotspot is still localized: {roi}.",
        f"Review room-count divergence frames before manual enable: {scene_metrics.get('room_count_diverged_frames', [])}.",
        (
            "Check that cooldown suppressions stay inside the ROI and do not create broad topology drift: "
            f"inside={scene_metrics.get('cooldown_suppressions_inside_roi', 0)}, "
            f"outside={scene_metrics.get('cooldown_suppressions_outside_roi', 0)}."
        ),
        (
            "Review remaining-case composition to confirm the scene is still the intended local failure mode: "
            f"dominant={remaining.get('dominant_remaining_case', 'unknown')}."
        ),
    ]
