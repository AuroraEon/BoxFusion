"""
Validate Step29B2 gateway candidate artifacts and write README_step29b2.md.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np

from extract_step29b2_gateway_candidates_v1 import (
    ASSET_DIR,
    EXPECTED_KEY_GLOBAL_ROOM_IDS,
    EXTRACTION_SUMMARY_JSON,
    GATEWAY_CANDIDATES_JSON,
    GATEWAY_GRAPH_JSON,
    KEY_ROOM_PAIRS,
    LABEL10_WALL_AUDIT_JSON,
    PUBLIC_PATH_TRANSITIONS,
    README_PATH,
    REJECTED_ROOM_EDGES_JSON,
    REPO_ROOT,
    ROOM_PAIR_ADJACENCY_JSON,
    SCENE_ID,
    SHORT_SCENE_ID,
    SOURCE_LAYERS,
    STATUS_VALUES,
    STEP29B1_LAYERED_JSON,
    STEP29B1_LAYERED_NPZ,
    STEP29B1_MAPPING_JSON,
    STEP29B1_SUMMARY_JSON,
    STEP29B1_VALIDATION_JSON,
    STEP29B2_ROOT,
    VALIDATION_RESULTS_JSON,
    VERSION,
    VIS_DIR,
    normalize_pair,
    read_json,
    rel,
    write_json,
)
from render_step29b2_gateway_debug_v1 import REQUIRED_VISUALIZATIONS


REQUIRED_CANDIDATE_FIELDS = [
    "gateway_id",
    "room_a",
    "room_b",
    "candidate_index",
    "status",
    "selected_for_topology",
    "primary_for_room_pair",
    "center",
    "crossing_pose",
    "approach_from_room_a",
    "approach_from_room_b",
    "width_m",
    "clearance_m",
    "component_area_m2",
    "component_cell_count",
    "bbox_cells",
    "bbox_map_xy",
    "boundary_dilation_radius_m",
    "source_layers",
    "confidence",
    "validation",
    "warning_or_rejection_reason",
]

REQUIRED_POSE_FIELDS = ["x", "y", "yaw"]
REQUIRED_VALIDATION_FIELDS = [
    "connects_room_a",
    "connects_room_b",
    "two_sided_connectivity",
    "blocked_by_wall_evidence",
    "unknown_fraction",
    "free_fraction",
    "wall_overlap_fraction",
    "label10_fraction",
    "min_width_cells",
    "width_m",
    "status",
]


def add_check(results: Dict[str, Any], name: str, passed: bool, detail: Any) -> None:
    results["checks"][name] = {"pass": bool(passed), "detail": detail}
    if not passed:
        results["overall_pass"] = False
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}")


def pair_key(room_a: int, room_b: int) -> Tuple[int, int]:
    return normalize_pair(int(room_a), int(room_b))


def has_pair(entries: Sequence[Dict[str, Any]], room_a: int, room_b: int) -> bool:
    want = pair_key(room_a, room_b)
    return any(pair_key(e["room_a"], e["room_b"]) == want for e in entries)


def pose_ok(payload: Dict[str, Any]) -> bool:
    return all(field in payload and isinstance(payload[field], (int, float)) for field in REQUIRED_POSE_FIELDS)


def candidate_schema_errors(candidate: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    for field in REQUIRED_CANDIDATE_FIELDS:
        if field not in candidate:
            errors.append(f"missing {field}")
    for pose_field in ["center", "crossing_pose", "approach_from_room_a", "approach_from_room_b"]:
        if pose_field in candidate and not pose_ok(candidate[pose_field]):
            errors.append(f"bad pose {pose_field}")
    if candidate.get("status") not in STATUS_VALUES:
        errors.append("bad status")
    if candidate.get("source_layers") != SOURCE_LAYERS:
        errors.append("source_layers mismatch")
    validation = candidate.get("validation", {})
    for field in REQUIRED_VALIDATION_FIELDS:
        if field not in validation:
            errors.append(f"missing validation.{field}")
    if validation.get("status") != candidate.get("status"):
        errors.append("validation.status mismatch")
    if len(candidate.get("bbox_cells", [])) != 4:
        errors.append("bbox_cells length")
    if len(candidate.get("bbox_map_xy", [])) != 4:
        errors.append("bbox_map_xy length")
    return errors


def write_readme(
    validation_results: Dict[str, Any],
    summary: Dict[str, Any],
    graph: Dict[str, Any],
    adjacency: Dict[str, Any],
    rejected: Dict[str, Any],
    audit: Dict[str, Any],
) -> None:
    input_files = [
        STEP29B1_LAYERED_JSON,
        STEP29B1_LAYERED_NPZ,
        STEP29B1_SUMMARY_JSON,
        STEP29B1_VALIDATION_JSON,
        STEP29B1_MAPPING_JSON,
    ]
    not_used = graph.get("extraction_policy", {}).get("forbidden_geometry_sources_not_used", [])
    room_pairs = adjacency.get("room_pair_adjacency_candidates", [])
    def pair_result(a: int, b: int) -> str:
        pair = pair_key(a, b)
        entry = next((e for e in room_pairs if pair_key(e["room_a"], e["room_b"]) == pair), None)
        if not entry:
            return "not evaluated"
        selected = entry.get("selected_gateway_ids", [])
        return (
            f"raw={entry.get('raw_candidate_count')}, valid={entry.get('valid_candidate_count')}, "
            f"ambiguous={entry.get('ambiguous_candidate_count')}, misleading={entry.get('misleading_candidate_count')}, "
            f"rejected={entry.get('rejected_candidate_count')}, selected={selected or 'none'}"
        )

    visual_lines = "\n".join(f"- `{rel(VIS_DIR / name)}`" for name in REQUIRED_VISUALIZATIONS)
    input_lines = "\n".join(f"- `{rel(path)}`" for path in input_files)
    not_used_lines = "\n".join(f"- {item}" for item in not_used)
    pair_lines = "\n".join(
        f"- room_{e['room_a']} <-> room_{e['room_b']}: raw={e['raw_candidate_count']}, valid={e['valid_candidate_count']}, ambiguous={e['ambiguous_candidate_count']}, misleading={e['misleading_candidate_count']}, rejected={e['rejected_candidate_count']}, selected={e['selected_gateway_ids'] or 'none'}"
        for e in room_pairs
    )
    output_paths = [
        GATEWAY_CANDIDATES_JSON,
        GATEWAY_GRAPH_JSON,
        EXTRACTION_SUMMARY_JSON,
        VALIDATION_RESULTS_JSON,
        ROOM_PAIR_ADJACENCY_JSON,
        REJECTED_ROOM_EDGES_JSON,
        LABEL10_WALL_AUDIT_JSON,
    ]
    output_lines = "\n".join(f"- `{rel(path)}`" for path in output_paths)
    public = summary.get("public_path_042_gateway_readiness", {})
    public_lines = "\n".join(
        f"- room_{t['room_a']} -> room_{t['room_b']}: selected={t['selected_gateway_exists']}, gateway={t['selected_gateway_id']}, status={t['status']}, reason={t['reason_if_missing']}"
        for t in public.get("transitions", [])
    )
    can_step29c = "YES" if summary.get("step29c_topology_augmentation_can_proceed") else "NO"
    validation_status = "PASS" if validation_results.get("overall_pass") else "FAIL"
    readme = f"""# Step29B2: Gateway Candidates from Layered BEV for {SCENE_ID}

## Summary

Step29B2 extracted and classified room-boundary gateway candidates from the validated Step29B1 layered BEV. It did not augment topology, generate Nav2 goals, run Nav2, run Gazebo, or use ROS execution artifacts.

Validation status: **{validation_status}**

Step29C topology augmentation can proceed: **{can_step29c}**

## Exact Input Files Used

{input_lines}

## Exact Files Not Used

{not_used_lines}

Room polygons were not used.

Old BEV artifacts were not used.

Topology JSON was not used as geometry.

Nav2, Gazebo, ROS, AMCL, TF, and DWB were not run.

## Room Identity Confirmation

`room_mask_global_id` was used as the authoritative room mask for every room-pair operation.

Local marker labels were not used as room IDs. `room_mask_local_label_repaired` was read only for the mandatory label 10 audit.

## Label 10 Audit Summary

- Cell count: `{audit['label10_audit'].get('cell_count')}`
- Structural wall overlap: `{audit['label10_audit'].get('overlap_with_structural_wall')}`
- Unknown overlap: `{audit['label10_audit'].get('overlap_with_unknown_layer')}`
- Free-space overlap: `{audit['label10_audit'].get('overlap_with_free_space')}`
- Touches key rooms: `{audit['label10_audit'].get('touches_key_global_rooms')}`
- Interpretation: `{audit['label10_audit'].get('interpretation')}`
- Step29B2 usage: `{audit['label10_audit'].get('step29b2_usage')}`

## Wall-Room Overlap Audit Summary

- Overlap cell count: `{audit['wall_room_overlap_audit'].get('overlap_cell_count')}`
- Overlap by room: `{audit['wall_room_overlap_audit'].get('overlap_by_global_room_id')}`
- Key pair concentration: `{audit['wall_room_overlap_audit'].get('near_key_pair_boundary_overlap_counts')}`
- Policy: `{audit['wall_room_overlap_audit'].get('policy')}`

## Gateway Extraction Algorithm Summary

For each evaluated room pair, Step29B2 eroded room interiors, built near-boundary bands at 0.15m, 0.25m, 0.40m, and 0.60m, constrained candidate cells to explored non-unknown non-wall evidence, connected components, and validated two-sided connectivity back to both room interiors without using forbidden geometry sources.

## Candidate Classification Policy

Statuses are exactly `valid`, `ambiguous`, `misleading`, and `rejected`. Valid candidates require two-sided non-wall connectivity, free-space support, plausible TurtleBot3-class width, and low dependence on unknown, wall-room-overlap, or label-10 context. Ambiguous and misleading candidates are retained for review. Selected topology candidates are marked in the graph only; no topology augmentation artifact is created.

## Room Pairs Evaluated

{pair_lines}

## Candidate Counts

- Raw candidates: `{summary.get('raw_candidate_count')}`
- Valid candidates: `{summary.get('valid_candidate_count')}`
- Ambiguous candidates: `{summary.get('ambiguous_candidate_count')}`
- Misleading candidates: `{summary.get('misleading_candidate_count')}`
- Rejected candidates: `{summary.get('rejected_candidate_count')}`
- Selected gateway count: `{summary.get('selected_gateway_count')}`

## Key Pair Results

- room_3 <-> room_11: {pair_result(3, 11)}
- room_11 <-> room_7: {pair_result(11, 7)}
- room_3 <-> room_7: {pair_result(3, 7)}
- room_11 <-> room_8: {pair_result(11, 8)}

## Public Path 042 Gateway Readiness

Route: `{public.get('route')}`

{public_lines}

Route gateway ready: `{public.get('route_gateway_ready')}`

## Output Artifact Paths

{output_lines}

## Visualization Paths

{visual_lines}

## Validation Summary

- Overall pass: `{validation_results.get('overall_pass')}`
- Failed checks: `{validation_results.get('failed_checks')}`

## Known Limitations

{chr(10).join(f"- {item}" for item in graph.get('known_limitations', []))}

## Step29C Readiness

Step29C can proceed: **{can_step29c}**.

No `gateway_augmented_topology_v0_1.json` artifact was created by Step29B2.
"""
    README_PATH.parent.mkdir(parents=True, exist_ok=True)
    README_PATH.write_text(readme)
    print(f"Written: {rel(README_PATH)}")


def validate() -> Dict[str, Any]:
    results: Dict[str, Any] = {
        "scene_id": SCENE_ID,
        "artifact_type": "step29b2_gateway_candidate_validation_results",
        "version": VERSION,
        "overall_pass": True,
        "checks": {},
    }

    input_paths = {
        "step29b1_layered_json": STEP29B1_LAYERED_JSON,
        "step29b1_layered_npz": STEP29B1_LAYERED_NPZ,
        "step29b1_validation": STEP29B1_VALIDATION_JSON,
        "step29b1_summary": STEP29B1_SUMMARY_JSON,
        "step29b1_mapping": STEP29B1_MAPPING_JSON,
    }
    for name, path in input_paths.items():
        add_check(results, f"input_exists_{name}", path.is_file(), rel(path))

    generated_paths = {
        "gateway_candidates": GATEWAY_CANDIDATES_JSON,
        "gateway_graph": GATEWAY_GRAPH_JSON,
        "extraction_summary": EXTRACTION_SUMMARY_JSON,
        "room_pair_adjacency": ROOM_PAIR_ADJACENCY_JSON,
        "rejected_edges": REJECTED_ROOM_EDGES_JSON,
        "label10_wall_audit": LABEL10_WALL_AUDIT_JSON,
    }
    for name, path in generated_paths.items():
        add_check(results, f"generated_exists_{name}", path.is_file(), rel(path))

    if not all(path.is_file() for path in list(input_paths.values()) + list(generated_paths.values())):
        results["failed_checks"] = [name for name, check in results["checks"].items() if not check["pass"]]
        write_json(VALIDATION_RESULTS_JSON, results)
        return results

    b1_validation = read_json(STEP29B1_VALIDATION_JSON)
    candidates_payload = read_json(GATEWAY_CANDIDATES_JSON)
    graph = read_json(GATEWAY_GRAPH_JSON)
    summary = read_json(EXTRACTION_SUMMARY_JSON)
    adjacency = read_json(ROOM_PAIR_ADJACENCY_JSON)
    rejected = read_json(REJECTED_ROOM_EDGES_JSON)
    audit = read_json(LABEL10_WALL_AUDIT_JSON)
    data = np.load(STEP29B1_LAYERED_NPZ)

    add_check(results, "step29b1_validation_passed", bool(b1_validation.get("overall_pass")), b1_validation.get("overall_pass"))
    add_check(results, "room_mask_global_id_exists", "room_mask_global_id" in data.files, sorted(data.files))
    global_room = data["room_mask_global_id"].astype(np.int32)
    present = sorted(int(v) for v in np.unique(global_room) if int(v) > 0)
    add_check(results, "key_room_ids_present", all(room_id in present for room_id in EXPECTED_KEY_GLOBAL_ROOM_IDS), present)
    compatible = all(data[layer].shape == global_room.shape for layer in ["structural_wall", "free_space", "outside_boundary", "unknown_layer"])
    add_check(results, "core_layers_shape_compatible", compatible, {layer: list(data[layer].shape) for layer in ["room_mask_global_id", "structural_wall", "free_space", "outside_boundary", "unknown_layer"]})
    policy = graph.get("extraction_policy", {})
    add_check(results, "local_labels_not_used_as_room_ids", policy.get("local_labels_used_as_room_ids") is False and policy.get("room_identity_source") == "room_mask_global_id", policy)
    add_check(results, "label10_audit_exists", "label10_audit" in audit, rel(LABEL10_WALL_AUDIT_JSON))
    add_check(results, "wall_room_overlap_audit_exists", "wall_room_overlap_audit" in audit, rel(LABEL10_WALL_AUDIT_JSON))
    candidates = candidates_payload.get("gateway_candidates", [])
    add_check(results, "candidate_list_exists", isinstance(candidates, list), f"{len(candidates) if isinstance(candidates, list) else 'not_list'} candidates")

    pair_entries = adjacency.get("room_pair_adjacency_candidates", [])
    for room_a, room_b in [(3, 11), (11, 7), (3, 7), (11, 8)]:
        add_check(results, f"explicit_pair_room{room_a}_room{room_b}_evaluated", has_pair(pair_entries, room_a, room_b), "room pair adjacency candidates")
    public = graph.get("public_path_042_gateway_readiness", {})
    add_check(results, "public_path_042_readiness_evaluated", public.get("route") == [1, 3, 7, 11, 8] and len(public.get("transitions", [])) == 4, public)

    schema_errors = {c.get("gateway_id", f"candidate_{idx}"): candidate_schema_errors(c) for idx, c in enumerate(candidates)}
    schema_errors = {key: value for key, value in schema_errors.items() if value}
    add_check(results, "every_candidate_has_required_geometry_fields", not schema_errors, schema_errors)
    add_check(results, "every_candidate_status_valid", all(c.get("status") in STATUS_VALUES for c in candidates), sorted({c.get("status") for c in candidates}))
    selected_invalid = [c.get("gateway_id") for c in candidates if c.get("selected_for_topology") and c.get("status") != "valid"]
    add_check(results, "selected_for_topology_only_valid", not selected_invalid, selected_invalid)
    augmented = list(STEP29B2_ROOT.rglob("*gateway_augmented_topology*"))
    add_check(results, "no_gateway_augmented_topology_artifact_created", not augmented, [rel(path) for path in augmented])
    add_check(results, "no_nav2_gazebo_ros_execution_run", not policy.get("nav2_run") and not policy.get("gazebo_run") and not policy.get("ros_execution_run") and not policy.get("amcl_tf_dwb_run"), policy)
    forbidden_ok = (
        policy.get("room_polygons_used") is False
        and policy.get("old_bev_used") is False
        and policy.get("topology_json_used_as_geometry") is False
    )
    add_check(results, "forbidden_geometry_sources_not_used", forbidden_ok, policy)
    missing_visuals = [rel(VIS_DIR / name) for name in REQUIRED_VISUALIZATIONS if not (VIS_DIR / name).is_file()]
    add_check(results, "required_visualizations_exist", not missing_visuals, missing_visuals)
    add_check(results, "rejected_edges_file_exists", REJECTED_ROOM_EDGES_JSON.is_file(), rel(REJECTED_ROOM_EDGES_JSON))

    pair_3_11 = [c for c in candidates if pair_key(c["room_a"], c["room_b"]) == pair_key(3, 11)]
    pair_3_11_selected = [c for c in pair_3_11 if c.get("selected_for_topology")]
    rejected_entries = rejected.get("rejected_room_edges", [])
    add_check(
        results,
        "room3_room11_rejected_edge_present_if_no_valid_gateway",
        bool(pair_3_11_selected) or has_pair(rejected_entries, 3, 11),
        {"selected": [c["gateway_id"] for c in pair_3_11_selected], "rejected_edges": rejected_entries},
    )
    pair_11_7 = [c for c in candidates if pair_key(c["room_a"], c["room_b"]) == pair_key(11, 7)]
    primary_11_7 = [c for c in pair_11_7 if c.get("primary_for_room_pair")]
    add_check(
        results,
        "room11_room7_at_most_one_primary_if_multiple_raw",
        len(pair_11_7) <= 1 or len(primary_11_7) <= 1,
        {"raw": len(pair_11_7), "primary": [c["gateway_id"] for c in primary_11_7]},
    )

    results["failed_checks"] = [name for name, check in results["checks"].items() if not check["pass"]]
    results["candidate_counts"] = {
        "raw": len(candidates),
        "valid": sum(1 for c in candidates if c.get("status") == "valid"),
        "ambiguous": sum(1 for c in candidates if c.get("status") == "ambiguous"),
        "misleading": sum(1 for c in candidates if c.get("status") == "misleading"),
        "rejected": sum(1 for c in candidates if c.get("status") == "rejected"),
        "selected": sum(1 for c in candidates if c.get("selected_for_topology")),
    }
    results["public_path_042_gateway_readiness"] = public
    results["label10_audit_result"] = audit.get("label10_audit", {})
    results["wall_room_overlap_audit_result"] = audit.get("wall_room_overlap_audit", {})
    write_json(VALIDATION_RESULTS_JSON, results)
    write_readme(results, summary, graph, adjacency, rejected, audit)
    return results


def main() -> None:
    results = validate()
    print(f"Step29B2 validation overall pass: {results.get('overall_pass')}")
    if results.get("failed_checks"):
        print(f"Failed checks: {results['failed_checks']}")


if __name__ == "__main__":
    main()
