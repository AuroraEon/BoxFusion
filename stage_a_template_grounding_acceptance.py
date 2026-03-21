import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from boxfusion.stage_a_template_grounding_utils import (
    aggregate_acceptance_results,
    build_acceptance_case_result,
    build_default_acceptance_cases,
    build_export_metadata_summary,
    choose_default_start_room_id,
    infer_instruction_family,
    load_instruction_cases,
    render_acceptance_console_summary,
    render_acceptance_markdown,
)
from boxfusion.template_grounding import TemplateGrounder


def build_template_grounding_acceptance_report(
    topology_json: Path,
    *,
    start_room_id: Optional[str] = None,
    route_policy: str = "balanced",
    instruction_args: Optional[List[str]] = None,
    instruction_file: Optional[str] = None,
    include_defaults: bool = True,
    include_export_probes: bool = True,
    sample_limit: int = 5,
) -> Dict[str, Any]:
    topology_json = Path(topology_json)
    grounder = TemplateGrounder.from_json(topology_json)
    topology = grounder.query_api.topology
    export_summary = build_export_metadata_summary(topology, sample_limit=sample_limit)
    effective_start_room_id, auto_selected_start_room_id = choose_default_start_room_id(
        topology,
        preferred_start_room_id=start_room_id,
    )

    cases: List[Dict[str, Any]] = []
    if include_defaults:
        cases.extend(
            build_default_acceptance_cases(
                topology,
                start_room_id=effective_start_room_id,
                export_summary=export_summary,
                include_export_probes=include_export_probes,
            )
        )

    if instruction_file:
        cases.extend(load_instruction_cases(Path(instruction_file), default_start_room_id=effective_start_room_id))

    for instruction in instruction_args or []:
        family = infer_instruction_family(instruction)
        cases.append(
            {
                "instruction": instruction,
                "family": family,
                "start_room_id": effective_start_room_id if family in {
                    "explicit_room_target",
                    "semantic_room_target",
                    "object_label_target",
                    "anchor_id_target",
                } else None,
                "source": "cli_arg",
            }
        )

    case_results = []
    for index, case in enumerate(cases, start=1):
        result = grounder.ground(
            instruction=str(case["instruction"]),
            start_room_id=case.get("start_room_id"),
            route_policy=route_policy,
        )
        case_results.append(
            build_acceptance_case_result(
                case_id=f"case_{index:03d}",
                case=dict(case, route_policy=route_policy),
                result=result,
                export_summary=export_summary,
            )
        )

    return aggregate_acceptance_results(
        topology_json=topology_json,
        sequence_id=topology.sequence_id,
        route_policy=route_policy,
        effective_start_room_id=effective_start_room_id,
        auto_selected_start_room_id=auto_selected_start_room_id,
        export_summary=export_summary,
        case_results=case_results,
    )


def _print_text_report(report: Dict[str, Any]) -> None:
    export_summary = dict(report.get("export_summary") or {})
    inspection = dict(export_summary.get("inspection") or {})
    room_type_summary = dict(export_summary.get("room_type_summary") or {})

    print(f"topology_json: {report.get('topology_json')}")
    print(f"sequence_id: {report.get('sequence_id')}")
    print(f"route_policy: {report.get('route_policy')}")
    print(
        "counts: "
        f"rooms={inspection.get('room_count', 0)} "
        f"objects={inspection.get('object_count', 0)} "
        f"anchors={inspection.get('anchor_count', 0)} "
        f"object_labels={inspection.get('object_label_count', 0)}"
    )
    print(
        "room_type_coverage: "
        f"{room_type_summary.get('usable_room_type_count', 0)}/{room_type_summary.get('room_count', 0)} "
        f"usable ({room_type_summary.get('usable_room_type_coverage', 0.0)})"
    )
    print(f"effective_start_room_id: {report.get('effective_start_room_id')}")
    print(f"auto_selected_start_room_id: {report.get('auto_selected_start_room_id')}")

    diagnostics = list(export_summary.get("diagnostics") or [])
    if diagnostics:
        print("diagnostics:")
        for item in diagnostics:
            print(f"  - {item}")

    print(render_acceptance_console_summary(report))
    print("by_family:")
    for family, family_report in (report.get("by_family") or {}).items():
        print(
            f"  - {family_report.get('family_display_name')}: "
            f"{family_report.get('passed_cases', 0)}/{family_report.get('total_cases', 0)} passed "
            f"(available_on_current_export={family_report.get('available_on_current_export')})"
        )
        if family_report.get("family_note"):
            print(f"    note: {family_report.get('family_note')}")

    print("cases:")
    for item in report.get("cases", []):
        print(
            f"  - {item.get('case_id')}: {item.get('result_category')} "
            f"family={item.get('family')} start={item.get('start_room_id')} "
            f"instruction={item.get('instruction')!r} "
            f"goal={item.get('resolved_goal_room_id')} "
            f"reason={item.get('failure_reason')}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run Stage A real-export acceptance checks for Template Grounding v0.1."
    )
    parser.add_argument("--topology-json", required=True, help="Path to logs/topology_v0_1.json")
    parser.add_argument(
        "--start-room",
        default=None,
        help="Optional start room context. If omitted, the runner auto-selects the first room in the export when available.",
    )
    parser.add_argument(
        "--route-policy",
        default="balanced",
        help="Route policy preset: strict | balanced | exploratory",
    )
    parser.add_argument(
        "--instruction",
        action="append",
        default=[],
        help="Additional instruction to evaluate. May be repeated.",
    )
    parser.add_argument(
        "--instruction-file",
        default=None,
        help="Optional path to a small JSON/YAML/TXT instruction file.",
    )
    parser.add_argument(
        "--no-defaults",
        action="store_true",
        help="Disable the built-in default instruction set.",
    )
    parser.add_argument(
        "--no-export-probes",
        action="store_true",
        help="Disable export-aware auto-generated probe cases.",
    )
    parser.add_argument("--sample-limit", type=int, default=5, help="Number of sample metadata items to keep.")
    parser.add_argument("--report-json-out", default=None, help="Optional path to save the JSON report.")
    parser.add_argument("--report-md-out", default=None, help="Optional path to save the markdown report.")
    parser.add_argument("--json", action="store_true", help="Print the full report JSON to stdout.")
    parser.add_argument(
        "--markdown",
        action="store_true",
        help="Print the markdown report to stdout instead of the text summary.",
    )
    args = parser.parse_args()

    report = build_template_grounding_acceptance_report(
        Path(args.topology_json),
        start_room_id=args.start_room,
        route_policy=args.route_policy,
        instruction_args=list(args.instruction or []),
        instruction_file=args.instruction_file,
        include_defaults=not args.no_defaults,
        include_export_probes=not args.no_export_probes,
        sample_limit=args.sample_limit,
    )

    if args.report_json_out:
        output_path = Path(args.report_json_out)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

    markdown_report = render_acceptance_markdown(report)
    if args.report_md_out:
        output_path = Path(args.report_md_out)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(markdown_report, encoding="utf-8")

    if args.json:
        print(json.dumps(report, indent=2))
        return
    if args.markdown:
        print(markdown_report, end="")
        return
    _print_text_report(report)


if __name__ == "__main__":
    main()
