"""Schema checks for RSLG-SLAM route plan dry-run previews."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

from .common import PROJECT_NAME, load_json, save_json


PASS_CLASSIFICATION = "route_plan_schema_validation_passed"
FAIL_CLASSIFICATION = "route_plan_schema_validation_failed"
ARTIFACT_LAYER = "Layer 3: Navigation Interface Layer"

SUPPORTED_ROUTE_KINDS = {"cross_floor_room", "cross_floor_object"}
ROOM_SEGMENT_TYPES = ["floor_1_room_segment", "vertical_transition_segment", "floor_2_room_segment"]
OBJECT_SEGMENT_TYPES = ROOM_SEGMENT_TYPES + ["object_approach_segment"]
REQUIRED_ROOMS = ["room_2", "room_3", "room_7", "room_13", "room_14"]
CONNECTOR_IDS = {"vt_1", "vc_vt_1"}
TRANSITION_EDGE = "vt_1_centerline_e001"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"

OBJECT_EXPECTED = {
    "query": "curtain in room_14 on floor_2",
    "object_id": "obj_175",
    "object_label": "curtain",
    "target_floor": "floor_2",
    "target_room": "room_14",
    "approach_candidate_id": "generated_ring_037",
    "object_centroid_navigation_used": False,
    "direct_object_centroid_goal_used": False,
    "approach_geometry_regenerated": False,
    "approach_feasibility_revalidated": False,
}

REQUIRED_FALSE_FLAGS = [
    "world_model_rerun",
    "runtime_launched",
    "old_scripts_called",
    "business_logic_migrated",
    "real_astar_route_generated",
    "astar_waypoints_generated",
    "executable_route_generated",
    "runtime_input_package_generated",
    "runtime_artifacts_generated",
    "map_pixels_generated",
    "connector_geometry_regenerated",
    "object_approach_geometry_regenerated",
    "object_centroid_navigation_used",
    "direct_object_centroid_goal_used",
]

FORBIDDEN_TRUE_FLAGS = [
    "final_route_plan_generated",
    "is_final_route_plan",
    "real_astar_route_generated",
    "astar_waypoints_generated",
    "executable_route_generated",
    "runtime_input_package_generated",
    "runtime_validation_completed",
    "gazebo_rviz_nav2_validation_completed",
    "object_navigation_runtime_completed",
    "amcl_success",
    "physical_stair_climbing_completed",
    "real_robot_stair_climbing_completed",
    "final_stable_map_generated",
    "pgm_generated",
    "map_server_yaml_generated",
    "external_gt_map_used",
    "simulator_navmesh_used",
]

FORBIDDEN_CLAIM_PHRASES = [
    "final route plan generated",
    "real A* route generated",
    "A* waypoints generated",
    "executable route generated",
    "runtime input package generated",
    "runtime validation completed",
    "Gazebo/RViz/Nav2 validation completed",
    "object navigation runtime completed",
    "AMCL success",
    "physical stair climbing completed",
    "real robot stair climbing completed",
    "final stable map generated",
    "PGM/YAML generated",
    "external GT map used",
    "simulator navmesh used",
]


def _normalize_text(value: Any) -> str:
    text = str(value).lower()
    for char in "-_/.,;:()[]{}\"'":
        text = text.replace(char, " ")
    return " ".join(text.split())


def _walk(value: Any, path: str = "$") -> Iterator[tuple[str, Any]]:
    yield path, value
    if isinstance(value, Mapping):
        for key, item in value.items():
            yield from _walk(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _walk(item, f"{path}[{index}]")


def _string_values(value: Any) -> Iterator[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for item in value.values():
            yield from _string_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from _string_values(item)
    elif value is not None and not isinstance(value, bool):
        yield str(value)


def _phrase_is_negated(text: str, phrase: str) -> bool:
    words = text.split()
    phrase_words = phrase.split()
    for index in range(0, len(words) - len(phrase_words) + 1):
        if words[index : index + len(phrase_words)] != phrase_words:
            continue
        prefix = " ".join(words[max(0, index - 8) : index])
        if any(marker in prefix for marker in ["not", "no", "without", "never", "must not", "do not", "does not"]):
            return True
    return False


def _payload(preview: Mapping[str, Any]) -> Mapping[str, Any]:
    payload = preview.get("route_plan_preview_payload", {})
    return payload if isinstance(payload, Mapping) else {}


def _segment_types(preview: Mapping[str, Any]) -> list[str]:
    payload = _payload(preview)
    segment_list = payload.get("segment_preview_list", [])
    if not isinstance(segment_list, list):
        return []
    result = []
    for item in segment_list:
        if isinstance(item, Mapping) and item.get("segment_type"):
            result.append(str(item["segment_type"]))
    return result


def _validate_forbidden_claims(preview: Mapping[str, Any], errors: list[str], source: Path) -> None:
    for path, value in _walk(preview):
        leaf = path.rsplit(".", 1)[-1]
        if leaf in FORBIDDEN_TRUE_FLAGS and value is True:
            errors.append(f"{source.as_posix()}: forbidden positive claim flag {leaf} is true at {path}")
        if isinstance(value, str):
            text = _normalize_text(value)
            for phrase in FORBIDDEN_CLAIM_PHRASES:
                normalized = _normalize_text(phrase)
                if normalized in text and not _phrase_is_negated(text, normalized):
                    errors.append(f"{source.as_posix()}: forbidden positive claim phrase {phrase!r} at {path}")


def _validate_common(preview: Mapping[str, Any], source: Path, errors: list[str]) -> None:
    for key, expected in {
        "artifact_layer": ARTIFACT_LAYER,
        "schema_name": "rslg_route_plan_preview",
        "schema_version": "0.1",
        "is_route_plan_preview": True,
        "is_final_route_plan": False,
        "generated_from_candidate_route_contract": True,
        "generated_from_final_route_contract": False,
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
    }.items():
        if preview.get(key) != expected:
            errors.append(f"{source.as_posix()}: {key} must be {expected!r}")
    if preview.get("route_kind") not in SUPPORTED_ROUTE_KINDS:
        errors.append(f"{source.as_posix()}: route_kind must be one of {sorted(SUPPORTED_ROUTE_KINDS)}")
    for flag in REQUIRED_FALSE_FLAGS:
        if preview.get(flag) is not False:
            errors.append(f"{source.as_posix()}: safety flag {flag} must be false")


def _validate_segment_scope(preview: Mapping[str, Any], source: Path, errors: list[str]) -> bool:
    route_kind = preview.get("route_kind")
    segment_types = _segment_types(preview)
    expected = OBJECT_SEGMENT_TYPES if route_kind == "cross_floor_object" else ROOM_SEGMENT_TYPES
    if segment_types != expected:
        errors.append(f"{source.as_posix()}: segment types must be exactly {expected!r}, got {segment_types!r}")
    if route_kind == "cross_floor_room" and "object_approach_segment" in segment_types:
        errors.append(f"{source.as_posix()}: cross_floor_room must not include object_approach_segment")
    return segment_types == expected


def _validate_cross_floor_truth(preview: Mapping[str, Any], source: Path, errors: list[str]) -> None:
    raw_text = json.dumps(preview, sort_keys=True)
    tokens = set(re.findall(r"[a-z]+(?:_[a-z0-9]+)+", raw_text.lower()))
    if not tokens.intersection(CONNECTOR_IDS):
        errors.append(f"{source.as_posix()}: connector must include vt_1 or vc_vt_1")
    for token in [TRANSITION_EDGE, NON_TRANSITION_EDGE, "floor_1", "floor_2", *REQUIRED_ROOMS]:
        if token not in raw_text:
            errors.append(f"{source.as_posix()}: missing cross-floor truth token {token!r}")
    payload = _payload(preview)
    floor_transition = payload.get("floor_transition", {})
    if not isinstance(floor_transition, Mapping) or floor_transition.get("source_floor") != "floor_1" or floor_transition.get("target_floor") != "floor_2":
        errors.append(f"{source.as_posix()}: floor transition floor_1 -> floor_2 must be present")


def _validate_object_truth(preview: Mapping[str, Any], source: Path, errors: list[str]) -> bool:
    payload = _payload(preview)
    object_safety = payload.get("object_safety", {})
    object_errors_before = len(errors)
    for key, expected in OBJECT_EXPECTED.items():
        if key in {
            "object_centroid_navigation_used",
            "direct_object_centroid_goal_used",
            "approach_geometry_regenerated",
            "approach_feasibility_revalidated",
        }:
            actual = payload.get(key)
            if actual is None and isinstance(object_safety, Mapping):
                actual = object_safety.get(key)
        else:
            actual = payload.get(key)
        if actual != expected:
            errors.append(f"{source.as_posix()}: route_plan_preview_payload.{key} must be {expected!r}")
    return len(errors) == object_errors_before


def _validate_preview_file(path: Path) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    try:
        data = load_json(path)
        if not isinstance(data, Mapping):
            raise ValueError("route plan preview JSON must be an object")
    except Exception as exc:
        message = f"{path.as_posix()}: {exc}"
        return {
            "input_json": path.as_posix(),
            "ok": False,
            "route_kind": None,
            "error_count": 1,
            "warning_count": 0,
            "segment_scope_checked": False,
            "cross_floor_truth_checked": False,
            "object_truth_checked": False,
            "claim_boundary_checked": False,
            "errors": [message],
            "warnings": [],
        }

    _validate_common(data, path, errors)
    segment_scope_ok = _validate_segment_scope(data, path, errors)
    _validate_cross_floor_truth(data, path, errors)
    object_truth_checked = False
    if data.get("route_kind") == "cross_floor_object":
        object_truth_checked = _validate_object_truth(data, path, errors)
    _validate_forbidden_claims(data, errors, path)

    return {
        "input_json": path.as_posix(),
        "classification": data.get("classification"),
        "scene_id": data.get("scene_id"),
        "route_kind": data.get("route_kind"),
        "artifact_layer": data.get("artifact_layer"),
        "segment_types": _segment_types(data),
        "segment_scope_checked": segment_scope_ok,
        "cross_floor_truth_checked": True,
        "object_truth_checked": object_truth_checked,
        "claim_boundary_checked": True,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "errors": errors,
        "warnings": warnings,
    }


def validate_route_plan_previews(input_jsons: Iterable[str | Path]) -> dict[str, Any]:
    paths = [Path(path).expanduser().resolve() for path in input_jsons]
    errors: list[str] = []
    warnings: list[str] = []
    validated_previews = []
    for path in paths:
        item = _validate_preview_file(path)
        validated_previews.append(item)
        errors.extend(item["errors"])
        warnings.extend(item["warnings"])

    route_kinds = {item.get("route_kind") for item in validated_previews}
    missing_route_kinds = sorted(SUPPORTED_ROUTE_KINDS - route_kinds)
    if paths and missing_route_kinds:
        errors.append(f"route plan schema validation input set missing route kinds {missing_route_kinds}")

    return {
        "project_name": PROJECT_NAME,
        "classification": PASS_CLASSIFICATION if not errors else FAIL_CLASSIFICATION,
        "error_count": len(errors),
        "warning_count": len(warnings),
        "artifact_layer": ARTIFACT_LAYER,
        "schema_validation_scope": "route_plan_preview_files",
        "stage_outputs_required": False,
        "historical_stage_outputs_required": False,
        "world_model_rerun": False,
        "runtime_launched": False,
        "old_scripts_called": False,
        "business_logic_migrated": False,
        "final_route_plans_generated": False,
        "real_astar_route_generated": False,
        "astar_waypoints_generated": False,
        "executable_route_generated": False,
        "runtime_input_package_generated": False,
        "runtime_artifacts_generated": False,
        "map_pixels_generated": False,
        "pgm_generated": False,
        "map_server_yaml_generated": False,
        "connector_geometry_regenerated": False,
        "object_approach_geometry_regenerated": False,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "validated_route_plan_previews": validated_previews,
        "route_kind_segment_scope": {
            "cross_floor_room": {
                "required_segment_types": ROOM_SEGMENT_TYPES,
                "forbidden_segment_types": ["object_approach_segment"],
                "segment_scope_checked": "cross_floor_room" in route_kinds,
            },
            "cross_floor_object": {
                "required_segment_types": OBJECT_SEGMENT_TYPES,
                "forbidden_segment_types": [],
                "segment_scope_checked": "cross_floor_object" in route_kinds,
            },
        },
        "claim_boundary_summary": {
            "preview_only": True,
            "final_route_plan_generated": False,
            "real_astar_route_generated": False,
            "astar_waypoints_generated": False,
            "executable_route_generated": False,
            "runtime_input_package_generated": False,
            "runtime_validation_completed": False,
            "gazebo_rviz_nav2_validation_completed": False,
            "object_navigation_runtime_completed": False,
            "amcl_success": False,
            "physical_stair_climbing_completed": False,
            "real_robot_stair_climbing_completed": False,
            "final_stable_map_generated": False,
            "pgm_generated": False,
            "map_server_yaml_generated": False,
            "external_gt_map_used": False,
            "simulator_navmesh_used": False,
        },
        "errors": errors,
        "warnings": warnings,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate RSLG-SLAM route plan preview JSON files.")
    parser.add_argument("--input-json", action="append", default=[], help="Route plan preview JSON. May be repeated.")
    parser.add_argument("--output-json", required=True, help="Write the schema validation report JSON.")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if not args.input_json:
        report = {
            "project_name": PROJECT_NAME,
            "classification": FAIL_CLASSIFICATION,
            "error_count": 1,
            "warning_count": 0,
            "artifact_layer": ARTIFACT_LAYER,
            "stage_outputs_required": False,
            "historical_stage_outputs_required": False,
            "world_model_rerun": False,
            "runtime_launched": False,
            "old_scripts_called": False,
            "business_logic_migrated": False,
            "errors": ["Pass at least one --input-json."],
            "warnings": [],
        }
    else:
        report = validate_route_plan_previews(args.input_json)
    output_path = Path(args.output_json).expanduser().resolve()
    save_json(output_path, report)
    print(
        json.dumps(
            {
                "classification": report["classification"],
                "error_count": report["error_count"],
                "warning_count": report["warning_count"],
                "output_json": output_path.as_posix(),
            },
            indent=2,
        )
    )
    return 0 if report["classification"] == PASS_CLASSIFICATION else 1


if __name__ == "__main__":
    raise SystemExit(main())
