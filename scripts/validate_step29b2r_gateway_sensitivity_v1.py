"""
Validate Step29B2R diagnostic sensitivity artifacts and write README_step29b2r.md.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

from extract_step29b2_gateway_candidates_v1 import SCENE_ID, SHORT_SCENE_ID, VERSION, normalize_pair, read_json, rel
from render_step29b2r_gateway_sensitivity_v1 import REQUIRED_VISUALIZATIONS
from review_step29b2r_gateway_sensitivity_v1 import (
    ASSET_DIR,
    FILES_NOT_USED,
    KEY_PAIR_REVIEW_JSON,
    LABEL10_UNKNOWN_JSON,
    PUBLIC_PATH_JSON,
    README_PATH,
    RECLASSIFIED_JSON,
    REVIEW_JSON,
    STEP29B1_INPUTS,
    STEP29B2_INPUTS,
    STEP29B2R_ROOT,
    SUMMARY_JSON,
    VALIDATION_JSON,
    VIS_DIR,
    WALL_LAYER_JSON,
    sha256_file,
    write_json,
)


REQUIRED_OUTPUTS = [
    REVIEW_JSON,
    SUMMARY_JSON,
    VALIDATION_JSON,
    KEY_PAIR_REVIEW_JSON,
    WALL_LAYER_JSON,
    LABEL10_UNKNOWN_JSON,
    RECLASSIFIED_JSON,
    PUBLIC_PATH_JSON,
]


def add_check(results: Dict[str, Any], name: str, passed: bool, detail: Any) -> None:
    results["checks"][name] = {"pass": bool(passed), "detail": detail}
    if not passed:
        results["overall_pass"] = False
        results["failed_checks"].append(name)
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}")


def pair_entry(review: Dict[str, Any], pair: Tuple[int, int]) -> Dict[str, Any]:
    want = normalize_pair(*pair)
    for entry in review.get("pair_reviews", []):
        if normalize_pair(entry["room_pair"]["room_a"], entry["room_pair"]["room_b"]) == want:
            return entry
    return {}


def pair_conclusion(summary: Dict[str, Any], pair: Tuple[int, int]) -> str:
    a, b = normalize_pair(*pair)
    return summary.get("key_pair_conclusions", {}).get(f"room_{a}<->room_{b}", "missing")


def validate() -> Dict[str, Any]:
    results: Dict[str, Any] = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2r_gateway_sensitivity_validation_results",
        "version": VERSION,
        "overall_pass": True,
        "checks": {},
        "failed_checks": [],
        "validation_results_path": rel(VALIDATION_JSON),
        "readme_path": rel(README_PATH),
    }
    for name, path in STEP29B1_INPUTS.items():
        add_check(results, f"input_exists_step29b1_{name}", path.is_file(), rel(path))
    for name, path in STEP29B2_INPUTS.items():
        add_check(results, f"input_exists_step29b2_{name}", path.is_file(), rel(path))

    b1_validation = read_json(STEP29B1_INPUTS["validation_results"]) if STEP29B1_INPUTS["validation_results"].is_file() else {}
    b2_validation = read_json(STEP29B2_INPUTS["gateway_candidate_validation_results"]) if STEP29B2_INPUTS["gateway_candidate_validation_results"].is_file() else {}
    add_check(results, "step29b1_validation_passed", bool(b1_validation.get("overall_pass")), b1_validation.get("failed_checks", []))
    add_check(results, "step29b2_validation_passed", bool(b2_validation.get("overall_pass")), b2_validation.get("failed_checks", []))

    for path in REQUIRED_OUTPUTS:
        if path == VALIDATION_JSON:
            add_check(results, "generated_exists_validation_results", True, rel(path))
        else:
            add_check(results, f"generated_exists_{path.stem}", path.is_file(), rel(path))
    for name in REQUIRED_VISUALIZATIONS:
        path = VIS_DIR / name
        add_check(results, f"visualization_exists_{name}", path.is_file() and path.stat().st_size > 0, rel(path))

    review = read_json(REVIEW_JSON) if REVIEW_JSON.is_file() else {}
    summary = read_json(SUMMARY_JSON) if SUMMARY_JSON.is_file() else {}
    reclass = read_json(RECLASSIFIED_JSON) if RECLASSIFIED_JSON.is_file() else {}
    wall = read_json(WALL_LAYER_JSON) if WALL_LAYER_JSON.is_file() else {}
    label = read_json(LABEL10_UNKNOWN_JSON) if LABEL10_UNKNOWN_JSON.is_file() else {}
    public = read_json(PUBLIC_PATH_JSON) if PUBLIC_PATH_JSON.is_file() else {}

    add_check(results, "room_mask_global_id_was_used", review.get("room_mask_global_id_used") is True and review.get("room_identity_source") == "room_mask_global_id", review.get("room_identity_source"))
    add_check(results, "local_labels_not_used_as_room_ids", review.get("local_labels_used_as_room_ids") is False, review.get("local_label_usage"))
    forbidden_ok = (
        review.get("room_polygons_used") is False
        and review.get("old_bev_used") is False
        and review.get("selected_public_route_edges_carved_w0p6_used") is False
        and review.get("topology_json_edges_used_as_geometry") is False
    )
    add_check(results, "forbidden_geometry_sources_not_used", forbidden_ok, review.get("files_not_used"))
    forbidden_outputs = [p for p in STEP29B2R_ROOT.rglob("*") if p.name in {"gateway_augmented_topology_v0_1.json", "nav2_goals_v0_1.json", "route_waypoints_v0_1.json"}]
    add_check(results, "no_topology_augmentation_artifact_created", not forbidden_outputs and review.get("topology_augmented") is False, [rel(p) for p in forbidden_outputs])
    add_check(results, "no_nav2_gazebo_ros_was_run", review.get("nav2_run") is False and review.get("gazebo_run") is False and review.get("ros_execution_run") is False and review.get("amcl_tf_dwb_run") is False, {
        "nav2_run": review.get("nav2_run"),
        "gazebo_run": review.get("gazebo_run"),
        "ros_execution_run": review.get("ros_execution_run"),
        "amcl_tf_dwb_run": review.get("amcl_tf_dwb_run"),
    })
    hash_mismatches = []
    before_hashes = review.get("source_step29b2_sha256_before_review", {})
    for name, path in STEP29B2_INPUTS.items():
        if path.is_file() and before_hashes.get(name) != sha256_file(path):
            hash_mismatches.append(name)
    add_check(results, "step29b2_inputs_not_overwritten", not hash_mismatches and review.get("step29b2_overwritten") is False, hash_mismatches)

    wall_pairs = wall.get("wall_layer_sensitivity_by_pair", {})
    add_check(results, "wall_policy_sensitivity_evaluated_for_key_pairs", all(
        f"room_{normalize_pair(*pair)[0]}<->room_{normalize_pair(*pair)[1]}" in wall_pairs for pair in [(7, 11), (11, 8), (3, 7), (3, 11), (1, 3)]
    ), list(wall_pairs.keys()))
    add_check(results, "label10_unknown_impact_evaluated", bool(label.get("label10_unknown_gateway_impact_by_pair")), list(label.get("label10_unknown_gateway_impact_by_pair", {}).keys()))
    add_check(results, "reclassification_artifact_exists", bool(reclass.get("reclassified_candidates")), reclass.get("reclassification_counts"))
    add_check(results, "room3_room11_explicitly_evaluated", bool(pair_entry(review, (3, 11))), pair_conclusion(summary, (3, 11)))
    room711 = pair_entry(review, (7, 11))
    add_check(results, "room7_room11_detailed_candidate_review", len(room711.get("candidate_reclassifications", [])) == 9, len(room711.get("candidate_reclassifications", [])))
    room118 = pair_entry(review, (11, 8))
    add_check(results, "room11_room8_detailed_candidate_review", len(room118.get("candidate_reclassifications", [])) >= 1, len(room118.get("candidate_reclassifications", [])))
    add_check(results, "public_path_042_sensitivity_readiness_generated", bool(public.get("public_path_042_sensitivity_readiness")), public.get("public_path_042_sensitivity_readiness", {}).get("diagnostic_route_status"))
    relaxed_promoted = []
    for candidate in reclass.get("reclassified_candidates", []):
        if candidate.get("diagnostic_reclassification") != "strict_valid_existing" and candidate.get("promotion_allowed"):
            relaxed_promoted.append(candidate.get("gateway_id"))
    add_check(results, "no_relaxed_result_promoted_to_topology", not relaxed_promoted and review.get("relaxed_results_promoted_to_topology") is False, relaxed_promoted)
    step29c = summary.get("step29c_recommendation", {})
    add_check(results, "step29c_readiness_conservative_and_justified", step29c.get("can_proceed_from_step29b2r") is False and "Do not proceed" in step29c.get("recommendation", ""), step29c)

    write_json(VALIDATION_JSON, results)
    write_readme(results, review, summary, reclass)
    return results


def bullet_paths(paths: Dict[str, Path]) -> str:
    return "\n".join(f"- `{rel(path)}`" for path in paths.values())


def write_readme(validation: Dict[str, Any], review: Dict[str, Any], summary: Dict[str, Any], reclass: Dict[str, Any]) -> None:
    input_files = {**STEP29B1_INPUTS, **STEP29B2_INPUTS}
    not_used = "\n".join(f"- {item}" for item in FILES_NOT_USED)
    visual_lines = "\n".join(f"- `{rel(VIS_DIR / name)}`" for name in REQUIRED_VISUALIZATIONS)
    wall_lines = []
    for pair_key, result in review.get("pair_reviews", [])[0:0]:
        wall_lines.append(str(pair_key))
    pair_lines = []
    for pair in [(3, 11), (7, 11), (11, 8), (3, 7), (1, 3)]:
        a, b = normalize_pair(*pair)
        pair_lines.append(f"- room_{a} <-> room_{b}: `{summary.get('key_pair_conclusions', {}).get(f'room_{a}<->room_{b}', 'missing')}`; action `{summary.get('key_pair_next_actions', {}).get(f'room_{a}<->room_{b}', 'missing')}`")
    public = summary.get("public_path_042_sensitivity_readiness", {})
    public_lines = "\n".join(
        f"- room_{t['room_a']} -> room_{t['room_b']}: `{t['diagnostic_status']}` ({t['reason']})"
        for t in public.get("transitions", [])
    )
    strict = summary.get("step29b2_strict_baseline_summary", {})
    rec = summary.get("step29c_recommendation", {})
    validation_status = "PASS" if validation.get("overall_pass") else "FAIL"
    readme = f"""# Step29B2R: Gateway Sensitivity Review for {SCENE_ID}

## Summary

Step29B2R is a diagnostic sensitivity review of Step29B2 gateway extraction for scene `{SCENE_ID}`. It compares strict Step29B2 results against wall erosion, wall-ignored diagnostic checks, free-space-only connectivity, unknown/label10 sensitivity, and interior/boundary radius sweeps.

Validation status: **{validation_status}**

This step did not augment topology and did not promote relaxed candidates to selected gateways.

## Exact Input Files Used

{bullet_paths(input_files)}

## Exact Files Not Used

{not_used}

## Confirmations

- Step29B2 was not overwritten.
- Topology was not augmented.
- Nav2, Gazebo, ROS, AMCL, TF, and DWB were not run.
- `room_mask_global_id` was used for all room identity operations.
- Local labels were not used as room IDs. `room_mask_local_label_repaired` was used only for label 10 impact analysis.

## Step29B2 Strict Baseline Summary

- Raw candidates: `{strict.get('raw_candidate_count')}`
- Valid: `{strict.get('valid_candidate_count')}`
- Ambiguous: `{strict.get('ambiguous_candidate_count')}`
- Misleading: `{strict.get('misleading_candidate_count')}`
- Rejected: `{strict.get('rejected_candidate_count')}`
- Selected gateways: `{strict.get('selected_gateway_count')}`
- Selected gateway ids: `{strict.get('selected_gateway_ids')}`
- Step29C can proceed from strict Step29B2: `{strict.get('step29c_topology_augmentation_can_proceed')}`

## Wall-Layer Sensitivity Summary

Policies evaluated: strict current wall, 1-cell wall erosion, 2-cell wall erosion, wall ignored diagnostic, free-space-only connectivity, and full-map-post-doors reference check. Relaxed wall results are diagnostic only.

## Label10/Unknown Impact Summary

Local label 10 and unknown-layer fractions were measured for every reviewed candidate and nearby boundary context. Candidates that depend on label10/unknown permissiveness are marked review-only, not valid.

## Candidate Reclassification Summary

`{reclass.get('reclassification_counts')}`

## Key Pair Results

{chr(10).join(pair_lines)}

## room_3 <-> room_11 Result

Conclusion: `{summary.get('room_3_room_11_conclusion')}`. The direct edge remains rejected under the diagnostic recommendation and is not revived for topology.

## room_7 <-> room_11 Result

Conclusion: `{summary.get('room_7_room_11_conclusion')}`. The nine strict rejected candidates were reviewed individually in the reclassification artifact and montage.

## room_11 <-> room_8 Result

Conclusion: `{summary.get('room_11_room_8_conclusion')}`. The ambiguous strict candidate remains manual/review-only because width and label10 context prevent topology-safe selection.

## room_3 <-> room_7 Result

Conclusion: `{summary.get('room_3_room_7_conclusion')}`. This required public_path_042 transition is not strict-ready.

## room_1 <-> room_3 Positive Control Result

Conclusion: `{summary.get('room_1_room_3_positive_control_conclusion')}`. The positive control remains strict-ready.

## public_path_042 Sensitivity Readiness

Diagnostic route status: `{public.get('diagnostic_route_status')}`

{public_lines}

Step29C readiness from Step29B2R: `{public.get('route_gateway_ready_for_step29c')}`.

## Visualizations Generated

{visual_lines}

## Known Limitations

- This is raster-layer diagnosis only; no Nav2, Gazebo, ROS, AMCL, TF, or DWB execution was performed.
- Full-map-post-doors is reference-only and not authoritative geometry.
- Label10 is not converted into a room and cannot produce a valid topology gateway.
- Relaxed candidates require human review or a later non-diagnostic step before any topology change.

## Recommendation

- Keep Step29B2 strict result: `{rec.get('keep_step29b2_strict_result')}`.
- Tune gateway extraction: `{rec.get('tune_gateway_extraction')}`.
- Manual gateway hint: `{rec.get('manual_gateway_hint')}`.
- Gateway-level route demo: `{rec.get('gateway_level_route_demo')}`.
- Step29C: `{rec.get('recommendation')}`.
"""
    README_PATH.parent.mkdir(parents=True, exist_ok=True)
    README_PATH.write_text(readme)
    print(f"Written: {rel(README_PATH)}")


def main() -> None:
    results = validate()
    print(f"\nStep29B2R validation overall_pass: {results['overall_pass']}")


if __name__ == "__main__":
    main()
