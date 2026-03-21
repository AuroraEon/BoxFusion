from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from stage_a_minimal_vln_demo import build_minimal_vln_demo_result


DEFAULT_MANIFEST_PATH = Path(__file__).resolve().with_name("minimal_vln_demo_v0_1_cases.json")


def load_manifest(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def resolve_sequence_alias(manifest: Dict[str, Any], sequence_arg: str) -> str:
    normalized = str(sequence_arg or manifest.get("default_sequence", "main")).strip().lower()
    sequences = dict(manifest.get("sequences") or {})
    alias_map = {alias.lower(): alias for alias in sequences}
    full_id_map = {
        str(sequence.get("sequence_id", "")).lower(): alias
        for alias, sequence in sequences.items()
    }
    short_id_map = {
        str(sequence.get("sequence_id", "")).split("-", 1)[0].lower(): alias
        for alias, sequence in sequences.items()
    }

    if normalized in alias_map:
        return alias_map[normalized]
    if normalized in full_id_map:
        return full_id_map[normalized]
    if normalized in short_id_map:
        return short_id_map[normalized]

    raise SystemExit(
        "Unknown sequence selector {!r}. Choose one of: {}.".format(
            sequence_arg,
            ", ".join(sorted({*alias_map.keys(), *short_id_map.keys()})),
        )
    )


def select_cases(sequence_payload: Dict[str, Any], case_id: Optional[str]) -> List[Dict[str, Any]]:
    cases = list(sequence_payload.get("cases") or [])
    if case_id is None:
        return cases

    for case in cases:
        if case.get("case_id") == case_id:
            return [case]
    raise SystemExit(
        "Unknown case_id {!r} for sequence {!r}. Available cases: {}.".format(
            case_id,
            sequence_payload.get("alias"),
            ", ".join(case.get("case_id", "<unknown>") for case in cases),
        )
    )


def _family_template_line(family: str) -> str:
    family_to_line = {
        "explicit_room_target": "Instruction matched the explicit room-target template.",
        "room_to_room": "Instruction matched the room-to-room template.",
        "object_label_target": "Instruction matched the object-label template.",
        "anchor_id_target": "Instruction matched the anchor-id template.",
    }
    return family_to_line.get(family, f"Instruction matched the {family} template.")


def build_teacher_summary_lines(
    case: Dict[str, Any],
    actual: Dict[str, Any],
    matched_expected_behavior: bool,
) -> List[str]:
    lines = [_family_template_line(str(case.get("intended_template_family") or ""))]
    target_reference = dict(case.get("target_reference") or {})
    actual_start = actual.get("resolved_start_room_id")
    actual_goal = actual.get("resolved_goal_room_id")
    room_sequence = list(actual.get("room_sequence") or [])
    family = str(case.get("intended_template_family") or "")

    if family in {"explicit_room_target", "room_to_room"} and actual_start and actual_goal:
        lines.append(f"Start room {actual_start} and target room {actual_goal} were resolved directly.")
    elif family == "object_label_target" and actual_goal:
        lines.append(
            "Object label {!r} grounded to {}.".format(
                target_reference.get("object_label"),
                actual_goal,
            )
        )
    elif family == "anchor_id_target" and actual_goal:
        lines.append(
            "Anchor {} grounded to {}.".format(
                target_reference.get("anchor_id"),
                actual_goal,
            )
        )

    if room_sequence:
        lines.append(f"Computed room-level route: {' -> '.join(room_sequence)}.")

    lines.append(str(case.get("why_useful") or "").strip())
    if matched_expected_behavior:
        lines.append("Observed output matched the validated v0.1 expectation.")
    else:
        lines.append("Observed output differed from the validated v0.1 expectation; see checks below.")
    return [line for line in lines if line]


def build_expectation_checks(
    case: Dict[str, Any],
    actual: Dict[str, Any],
    runtime_policy: str,
) -> Dict[str, Any]:
    expected_policy = str(case.get("expected_route_policy") or "balanced")
    checks: Dict[str, Any] = {
        "result_category_match": actual.get("result_category") == case.get("expected_status"),
        "grounded_target_type_match": actual.get("grounded_target_type") == case.get("expected_grounded_target_type"),
        "resolved_goal_room_match": actual.get("resolved_goal_room_id") == case.get("expected_resolved_goal_room_id"),
        "resolved_start_room_match": actual.get("resolved_start_room_id") == case.get("expected_resolved_start_room_id"),
        "policy_match": runtime_policy == expected_policy,
    }

    if runtime_policy == expected_policy:
        checks["route_room_sequence_match"] = list(actual.get("room_sequence") or []) == list(
            case.get("expected_route_room_sequence") or []
        )
    else:
        checks["route_room_sequence_match"] = None
        checks["validation_scope_note"] = (
            "Route sequence validation is only strict when the runtime policy matches the "
            f"validated policy {expected_policy!r}."
        )

    checks["matched_expected_behavior"] = all(
        value is True for key, value in checks.items() if key not in {"validation_scope_note"}
    )
    return checks


def execute_case(case: Dict[str, Any], runtime_policy: str) -> Dict[str, Any]:
    stack_report = build_minimal_vln_demo_result(
        Path(case["topology_json"]),
        instruction=str(case["instruction"]),
        start_room_id=case.get("start_room_id"),
        route_policy=runtime_policy,
    )
    demo_summary = dict(stack_report.get("demo_summary") or {})
    route_summary = dict(demo_summary.get("route_summary") or {})
    actual = {
        "runtime_policy": runtime_policy,
        "result_category": demo_summary.get("result_category"),
        "matched_template": demo_summary.get("matched_template"),
        "instruction_family": demo_summary.get("instruction_family"),
        "instruction_family_display_name": demo_summary.get("instruction_family_display_name"),
        "supported": demo_summary.get("supported"),
        "supported_on_current_export": demo_summary.get("supported_on_current_export"),
        "grounded_target_type": (demo_summary.get("grounded_target_summary") or {}).get("grounded_target_type"),
        "requested_start_room_id": (demo_summary.get("start_room_summary") or {}).get("requested_start_room_id"),
        "resolved_start_room_id": (demo_summary.get("start_room_summary") or {}).get("resolved_start_room_id"),
        "resolved_goal_room_id": (demo_summary.get("resolved_goal_room_summary") or {}).get("resolved_goal_room_id"),
        "route_found": route_summary.get("found"),
        "room_sequence": list(route_summary.get("room_sequence") or []),
        "hop_count": route_summary.get("hop_count"),
        "relation_sequence": list(route_summary.get("relation_sequence") or []),
        "route_confidence": route_summary.get("route_confidence"),
        "total_cost": route_summary.get("total_cost"),
        "route_steps": list(route_summary.get("steps") or []),
        "teacher_explanation": demo_summary.get("teacher_explanation"),
        "one_line_summary": demo_summary.get("one_line_summary"),
    }
    expectation_checks = build_expectation_checks(case, actual, runtime_policy)
    teacher_summary_lines = build_teacher_summary_lines(
        case=case,
        actual=actual,
        matched_expected_behavior=bool(expectation_checks.get("matched_expected_behavior")),
    )
    return {
        "case_id": case.get("case_id"),
        "case_name": case.get("case_name"),
        "display_label": case.get("display_label"),
        "instruction": case.get("instruction"),
        "topology_json": case.get("topology_json"),
        "target_reference": case.get("target_reference"),
        "expected": {
            "expected_status": case.get("expected_status"),
            "expected_grounded_target_type": case.get("expected_grounded_target_type"),
            "expected_resolved_start_room_id": case.get("expected_resolved_start_room_id"),
            "expected_resolved_goal_room_id": case.get("expected_resolved_goal_room_id"),
            "expected_route_room_sequence": list(case.get("expected_route_room_sequence") or []),
            "expected_route_policy": case.get("expected_route_policy"),
        },
        "actual": actual,
        "expectation_checks": expectation_checks,
        "matched_expected_behavior": expectation_checks.get("matched_expected_behavior"),
        "teacher_summary_lines": teacher_summary_lines,
        "demo_value": case.get("why_useful"),
        "stack_report": stack_report,
    }


def build_report(
    manifest_path: Path,
    manifest: Dict[str, Any],
    sequence_alias: str,
    cases: List[Dict[str, Any]],
    runtime_policy: str,
) -> Dict[str, Any]:
    sequence_payload = dict((manifest.get("sequences") or {}).get(sequence_alias) or {})
    results = [execute_case(case, runtime_policy=runtime_policy) for case in cases]
    matched_count = sum(1 for item in results if item.get("matched_expected_behavior"))
    runtime_pass_count = sum(1 for item in results if (item.get("actual") or {}).get("result_category") == "PASS")

    validation_scope_note = None
    if runtime_policy != str(sequence_payload.get("route_policy") or manifest.get("default_policy") or "balanced"):
        validation_scope_note = (
            "This package was validated against the balanced policy. Alternate policies still run through the same "
            "stack, but route-sequence checks may differ."
        )

    return {
        "package_id": manifest.get("package_id"),
        "package_name": manifest.get("package_name"),
        "version": manifest.get("version"),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "manifest_path": str(manifest_path),
        "selected_sequence_alias": sequence_alias,
        "sequence": sequence_payload,
        "runtime_policy": runtime_policy,
        "validation_scope_note": validation_scope_note,
        "semantic_room_name_demo_status": manifest.get("semantic_room_name_demo_status"),
        "summary": {
            "total_cases": len(results),
            "cases_matching_expectations": matched_count,
            "runtime_passed_cases": runtime_pass_count,
            "runtime_failed_cases": len(results) - runtime_pass_count,
        },
        "results": results,
    }


def render_text_report(report: Dict[str, Any], pretty: bool = False) -> str:
    sequence = dict(report.get("sequence") or {})
    summary = dict(report.get("summary") or {})
    lines = [
        f"{report.get('package_name')} ({report.get('version')})",
        f"Sequence: {sequence.get('display_label')} ({sequence.get('sequence_id')})",
        f"Topology: {sequence.get('topology_json')}",
        f"Policy: {report.get('runtime_policy')}",
        (
            "Validated matches: "
            f"{summary.get('cases_matching_expectations', 0)}/{summary.get('total_cases', 0)}"
        ),
        (
            "Runtime PASS count: "
            f"{summary.get('runtime_passed_cases', 0)}/{summary.get('total_cases', 0)}"
        ),
    ]

    semantic_status = dict(report.get("semantic_room_name_demo_status") or {})
    if semantic_status:
        lines.append(
            "Note: semantic room-name targets remain excluded from v0.1 demos because "
            + str(semantic_status.get("reason"))
        )
    if report.get("validation_scope_note"):
        lines.append(f"Validation note: {report.get('validation_scope_note')}")

    for item in report.get("results", []):
        actual = dict(item.get("actual") or {})
        expected = dict(item.get("expected") or {})
        lines.append("")
        lines.append(f"[{item.get('case_id')}] {item.get('display_label')}")
        lines.append(f"Instruction: {item.get('instruction')}")
        lines.append(
            "Observed: "
            f"{actual.get('result_category')} | goal={actual.get('resolved_goal_room_id')} | "
            f"route={' -> '.join(actual.get('room_sequence') or [])}"
        )
        for summary_line in item.get("teacher_summary_lines") or []:
            lines.append(f"- {summary_line}")
        if pretty:
            lines.append(
                "Expected: "
                f"status={expected.get('expected_status')} | goal={expected.get('expected_resolved_goal_room_id')} | "
                f"route={' -> '.join(expected.get('expected_route_room_sequence') or [])}"
            )
            lines.append(
                "Checks: "
                + ", ".join(
                    f"{key}={value}"
                    for key, value in (item.get("expectation_checks") or {}).items()
                    if key != "validation_scope_note"
                )
            )
            validation_scope_note = (item.get("expectation_checks") or {}).get("validation_scope_note")
            if validation_scope_note:
                lines.append(f"Checks note: {validation_scope_note}")
            route_steps = list(actual.get("route_steps") or [])
            if route_steps:
                lines.append("Route steps:")
                for step in route_steps:
                    lines.append(f"  - {step}")
    return "\n".join(lines) + "\n"


def render_markdown_report(report: Dict[str, Any]) -> str:
    sequence = dict(report.get("sequence") or {})
    summary = dict(report.get("summary") or {})
    lines = [
        f"# {report.get('package_name')} ({report.get('version')})",
        "",
        f"- sequence: `{sequence.get('display_label')} ({sequence.get('sequence_id')})`",
        f"- topology: `{sequence.get('topology_json')}`",
        f"- policy: `{report.get('runtime_policy')}`",
        f"- validated matches: `{summary.get('cases_matching_expectations', 0)}/{summary.get('total_cases', 0)}`",
        f"- runtime PASS count: `{summary.get('runtime_passed_cases', 0)}/{summary.get('total_cases', 0)}`",
        "",
    ]
    semantic_status = dict(report.get("semantic_room_name_demo_status") or {})
    if semantic_status:
        lines.extend(
            [
                "## Demo Boundary",
                "",
                f"- semantic room-name targets: `{semantic_status.get('status')}`",
                f"- reason: {semantic_status.get('reason')}",
                "",
            ]
        )

    for item in report.get("results", []):
        actual = dict(item.get("actual") or {})
        lines.extend(
            [
                f"## {item.get('case_id')}: {item.get('display_label')}",
                "",
                f"- instruction: `{item.get('instruction')}`",
                f"- observed: `{actual.get('result_category')}`",
                f"- goal_room: `{actual.get('resolved_goal_room_id')}`",
                f"- room_sequence: `{' -> '.join(actual.get('room_sequence') or [])}`",
            ]
        )
        for summary_line in item.get("teacher_summary_lines") or []:
            lines.append(f"- {summary_line}")
        lines.append("")
    return "\n".join(lines)


def maybe_write_report(report: Dict[str, Any], output_path: Optional[str], json_mode: bool) -> None:
    if not output_path:
        return

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if json_mode or path.suffix.lower() == ".json":
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return
    if path.suffix.lower() == ".md":
        path.write_text(render_markdown_report(report), encoding="utf-8")
        return
    path.write_text(render_text_report(report, pretty=True), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the formal Minimal VLN Demo v0.1 case package through the existing Stage A demo wrapper."
    )
    parser.add_argument(
        "--manifest",
        default=str(DEFAULT_MANIFEST_PATH),
        help="Path to the Minimal VLN Demo v0.1 manifest JSON.",
    )
    parser.add_argument(
        "--sequence",
        default="main",
        help="Sequence selector: main | backup | 00843 | 00847 | full sequence id.",
    )
    selection_group = parser.add_mutually_exclusive_group()
    selection_group.add_argument("--case", default=None, help="Optional case_id to run.")
    selection_group.add_argument(
        "--all-cases",
        action="store_true",
        help="Run all packaged cases for the selected sequence.",
    )
    parser.add_argument(
        "--policy",
        default="balanced",
        help="Route policy passed through to the existing minimal VLN demo wrapper.",
    )
    parser.add_argument("--json", action="store_true", help="Print the full JSON report.")
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="Include expectation checks and step-by-step route details in the text report.",
    )
    parser.add_argument(
        "--report-out",
        default=None,
        help="Optional path to save the report as .json, .md, or plain text.",
    )
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    manifest = load_manifest(manifest_path)
    sequence_alias = resolve_sequence_alias(manifest, args.sequence)
    sequence_payload = dict((manifest.get("sequences") or {}).get(sequence_alias) or {})
    selected_cases = select_cases(sequence_payload, case_id=args.case)

    report = build_report(
        manifest_path=manifest_path,
        manifest=manifest,
        sequence_alias=sequence_alias,
        cases=selected_cases,
        runtime_policy=str(args.policy),
    )

    maybe_write_report(report, args.report_out, json_mode=bool(args.json))

    if args.json:
        print(json.dumps(report, indent=2))
        return
    print(render_text_report(report, pretty=bool(args.pretty)), end="")


if __name__ == "__main__":
    main()
