#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple


DEFAULT_BASELINE_00843 = Path("tmp/analysis/00843_birth_baseline.json")
DEFAULT_BASELINE_00824 = Path("tmp/analysis/00824_birth_baseline.json")
DEFAULT_OUTPUT = Path("tmp/analysis/00824_vs_00843_birth_threshold_validation.json")

KEY_METRICS: Sequence[Tuple[str, Sequence[str]]] = (
    ("raw_birth_px", ("metrics", "birth_mask_partition", "raw_supported_px")),
    ("raw_seed_px", ("metrics", "seed_support_partition", "raw_supported_px")),
    ("raw_largest_seed_touching_component_px", ("metrics", "connectivity", "raw_largest_seed_touching_component_px")),
    ("raw_birth_fraction", ("metrics", "guard_readout", "raw_supported_birth_fraction")),
    ("raw_seed_fraction", ("metrics", "guard_readout", "raw_supported_seed_fraction")),
)

READOUT_METRICS: Sequence[Tuple[str, Sequence[str]]] = (
    ("birth_area", ("metrics", "birth_mask", "area_px")),
    ("seed_area", ("metrics", "seed_support", "area_px")),
    ("raw_birth", ("metrics", "birth_mask_partition", "raw_supported_px")),
    ("raw_seed", ("metrics", "seed_support_partition", "raw_supported_px")),
    ("raw_largest_seed_touching_component_px", ("metrics", "connectivity", "raw_largest_seed_touching_component_px")),
    ("raw_birth_fraction", ("metrics", "guard_readout", "raw_supported_birth_fraction")),
    ("raw_seed_fraction", ("metrics", "guard_readout", "raw_supported_seed_fraction")),
    ("blur_needed_for_seed_min_25", ("metrics", "guard_readout", "blur_needed_for_seed_min_25")),
    ("close_needed_for_seed_min_25", ("metrics", "guard_readout", "close_needed_for_seed_min_25")),
    ("blur_needed_for_any_seed_coverage", ("metrics", "guard_readout", "blur_needed_for_any_seed_coverage")),
    ("close_needed_for_any_seed_coverage", ("metrics", "guard_readout", "close_needed_for_any_seed_coverage")),
)


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _dig(payload: Dict[str, Any], path: Sequence[str]) -> Any:
    value: Any = payload
    for key in path:
        value = value[key]
    return value


def _row_readout(row: Dict[str, Any]) -> Dict[str, Any]:
    out = {
        "event_id": row["event_id"],
        "frame_idx": int(row["frame_idx"]),
        "global_id": int(row["global_id"]),
        "later_matched_frames": list(row["later_matched_frames"]),
        "later_observed_frames": list(row["later_observed_frames"]),
        "later_retained_missing_frames": list(row["later_retained_missing_frames"]),
        "later_dropped_frames": list(row["later_dropped_frames"]),
    }
    for label, path in READOUT_METRICS:
        out[label] = _dig(row, path)
    return out


def _threshold_hits(
    rows_00824: Sequence[Dict[str, Any]],
    target_00843: Dict[str, Any],
) -> Dict[str, Any]:
    hits_by_metric: Dict[str, Dict[str, Any]] = {}
    genuine_hit_any_metric: List[str] = []
    for label, path in KEY_METRICS:
        threshold = _dig(target_00843, path)
        hits = []
        for row in rows_00824:
            value = _dig(row, path)
            if float(value) <= float(threshold):
                hits.append(
                    {
                        "event_id": row["event_id"],
                        "frame_idx": int(row["frame_idx"]),
                        "global_id": int(row["global_id"]),
                        "value": value,
                        "threshold": threshold,
                        "later_matched_run_count": int(len(row["later_matched_frames"])),
                        "later_observed_run_count": int(len(row["later_observed_frames"])),
                    }
                )
                if row["event_id"] not in genuine_hit_any_metric:
                    genuine_hit_any_metric.append(row["event_id"])
        hits_by_metric[label] = {
            "threshold_value": threshold,
            "hit_count": int(len(hits)),
            "hits": hits,
        }
    return {
        "hits_by_metric": hits_by_metric,
        "unique_hit_event_ids_any_metric": genuine_hit_any_metric,
        "unique_hit_event_count_any_metric": int(len(genuine_hit_any_metric)),
    }


def _combined_weaker_or_equal_events(
    rows_00824: Sequence[Dict[str, Any]],
    target_00843: Dict[str, Any],
) -> List[Dict[str, Any]]:
    weaker = []
    for row in rows_00824:
        metric_deltas = {}
        all_weaker_or_equal = True
        for label, path in KEY_METRICS:
            value = float(_dig(row, path))
            threshold = float(_dig(target_00843, path))
            metric_deltas[label] = round(value - threshold, 4)
            if value > threshold:
                all_weaker_or_equal = False
        if all_weaker_or_equal:
            weaker.append(
                {
                    "event_id": row["event_id"],
                    "frame_idx": int(row["frame_idx"]),
                    "global_id": int(row["global_id"]),
                    "metric_deltas_vs_00843_room_6_800": metric_deltas,
                    "later_matched_run_count": int(len(row["later_matched_frames"])),
                    "later_observed_run_count": int(len(row["later_observed_frames"])),
                }
            )
    return weaker


def build_report(
    baseline_00843_path: Path,
    baseline_00824_path: Path,
) -> Dict[str, Any]:
    report_00843 = _load_json(baseline_00843_path)
    report_00824 = _load_json(baseline_00824_path)

    target_id = report_00843["analysis_scope"]["target_event_id"]
    target_row = next(row for row in report_00843["measured_events"] if row["event_id"] == target_id)
    rows_00824 = list(report_00824["measured_events"])
    threshold_hits = _threshold_hits(rows_00824=rows_00824, target_00843=target_row)
    weaker_or_equal_all_key_metrics = _combined_weaker_or_equal_events(rows_00824=rows_00824, target_00843=target_row)

    metric_minima = {}
    for label, path in KEY_METRICS:
        weakest_row = min(rows_00824, key=lambda row: float(_dig(row, path)))
        metric_minima[label] = {
            "weakest_event_id": weakest_row["event_id"],
            "weakest_value": _dig(weakest_row, path),
            "target_00843_value": _dig(target_row, path),
            "00824_is_weaker_than_target": bool(float(_dig(weakest_row, path)) < float(_dig(target_row, path))),
        }

    unique_metric_hits = int(threshold_hits["unique_hit_event_count_any_metric"])
    if unique_metric_hits >= 3:
        classification = "strongly supports no-fix conclusion"
    elif unique_metric_hits >= 1:
        classification = "mildly supports no-fix conclusion"
    elif weaker_or_equal_all_key_metrics:
        classification = "mixed / inconclusive"
    else:
        classification = "mixed / inconclusive"

    prospective_threshold_unsafe = bool(unique_metric_hits > 0 or len(weaker_or_equal_all_key_metrics) > 0)
    if not prospective_threshold_unsafe:
        classification = "mixed / inconclusive"

    return {
        "baseline_set_00824": {
            "sequence_id": report_00824["sequence_id"],
            "included_event_count": int(len(report_00824["measured_events"])),
            "selection_definition": report_00824["candidate_selection"]["definition"],
            "included_event_ids": [row["event_id"] for row in report_00824["measured_events"]],
        },
        "reference_00843_room_6_800": _row_readout(target_row),
        "comparison_summary": {
            "metric_minima_00824_vs_00843": metric_minima,
            "threshold_hits": threshold_hits,
            "events_weaker_or_equal_on_all_key_metrics": weaker_or_equal_all_key_metrics,
        },
        "answers": {
            "genuine_or_stable_00824_births_as_weak_or_weaker_on_key_support_metrics": bool(
                prospective_threshold_unsafe
            ),
            "would_threshold_targeting_00843_room_6_800_create_false_positive_risk_on_00824": bool(
                prospective_threshold_unsafe
            ),
            "00824_effect_on_current_no_fix_conclusion": (
                "strengthens"
                if prospective_threshold_unsafe
                else "does_not_strengthen"
            ),
        },
        "cross_sequence_conclusion": {
            "classification": classification,
            "prospective_birth_strength_threshold_unsafe": bool(prospective_threshold_unsafe),
            "justification": (
                "At least one 00824 included birth lands at or below the 00843 room_6@800 readout on a key support metric, "
                "so a guard tuned to catch room_6@800 would carry false-positive risk on another sequence."
                if prospective_threshold_unsafe
                else "No 00824 included birth landed below the 00843 room_6@800 readout on the tracked support metrics."
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare the 00824 birth baseline against 00843 room_6@800.")
    parser.add_argument("--baseline-00843", default=str(DEFAULT_BASELINE_00843))
    parser.add_argument("--baseline-00824", default=str(DEFAULT_BASELINE_00824))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    args = parser.parse_args()

    report = build_report(
        baseline_00843_path=Path(args.baseline_00843),
        baseline_00824_path=Path(args.baseline_00824),
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
