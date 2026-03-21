import argparse
import json
from pathlib import Path
from typing import Any, Dict, Optional

from boxfusion.stage_a_template_grounding_utils import (
    build_export_metadata_summary,
    build_minimal_vln_demo_payload,
)
from boxfusion.template_grounding import TemplateGrounder


def build_minimal_vln_demo_result(
    topology_json: Path,
    *,
    instruction: str,
    start_room_id: Optional[str] = None,
    route_policy: str = "balanced",
    sample_limit: int = 5,
) -> Dict[str, Any]:
    topology_json = Path(topology_json)
    grounder = TemplateGrounder.from_json(topology_json)
    export_summary = build_export_metadata_summary(grounder.query_api.topology, sample_limit=sample_limit)
    result = grounder.ground(
        instruction=instruction,
        start_room_id=start_room_id,
        route_policy=route_policy,
    )
    return {
        "topology_json": str(topology_json),
        "sequence_id": grounder.query_api.topology.sequence_id,
        "demo_summary": build_minimal_vln_demo_payload(
            instruction=instruction,
            start_room_id=start_room_id,
            route_policy=route_policy,
            result=result,
            export_summary=export_summary,
        ),
        "export_summary": export_summary,
    }


def _print_text_summary(report: Dict[str, Any]) -> None:
    demo_summary = dict(report.get("demo_summary") or {})
    route_summary = dict(demo_summary.get("route_summary") or {})
    grounded_target_summary = dict(demo_summary.get("grounded_target_summary") or {})
    start_room_summary = dict(demo_summary.get("start_room_summary") or {})
    goal_room_summary = dict(demo_summary.get("resolved_goal_room_summary") or {})

    print(f"instruction: {demo_summary.get('instruction')!r}")
    print(f"supported: {demo_summary.get('supported')}")
    print(f"supported_on_current_export: {demo_summary.get('supported_on_current_export')}")
    print(f"result_category: {demo_summary.get('result_category')}")
    print(f"matched_template: {demo_summary.get('matched_template')}")
    print(f"instruction_family: {demo_summary.get('instruction_family_display_name')}")
    print(f"start_room: requested={start_room_summary.get('requested_start_room_id')} resolved={start_room_summary.get('resolved_start_room_id')}")
    print(f"goal_room: {goal_room_summary.get('resolved_goal_room_id')}")
    print(f"grounded_target: {grounded_target_summary.get('summary')}")
    print(
        "route: "
        f"found={route_summary.get('found')} "
        f"hop_count={route_summary.get('hop_count')} "
        f"room_sequence={route_summary.get('room_sequence')} "
        f"relations={route_summary.get('relation_sequence')} "
        f"confidence={route_summary.get('route_confidence')} "
        f"cost={route_summary.get('total_cost')}"
    )
    if route_summary.get("steps"):
        print("route_steps:")
        for item in route_summary.get("steps", []):
            print(f"  - {item}")
    print(f"teacher_explanation: {demo_summary.get('teacher_explanation')}")
    print(f"one_line_summary: {demo_summary.get('one_line_summary')}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the Stage A minimal VLN demo wrapper over Template Grounding v0.1."
    )
    parser.add_argument("--topology-json", required=True, help="Path to logs/topology_v0_1.json")
    parser.add_argument("--instruction", required=True, help="Restricted-form navigation instruction.")
    parser.add_argument(
        "--start-room",
        default=None,
        help="Optional start room context for templates that do not specify the source room.",
    )
    parser.add_argument(
        "--route-policy",
        default="balanced",
        help="Route policy preset: strict | balanced | exploratory",
    )
    parser.add_argument("--sample-limit", type=int, default=5, help="Number of sample metadata items to keep.")
    parser.add_argument("--json", action="store_true", help="Print full JSON output.")
    parser.add_argument(
        "--markdown",
        action="store_true",
        help="Print a concise markdown summary instead of the text summary.",
    )
    args = parser.parse_args()

    report = build_minimal_vln_demo_result(
        Path(args.topology_json),
        instruction=args.instruction,
        start_room_id=args.start_room,
        route_policy=args.route_policy,
        sample_limit=args.sample_limit,
    )

    if args.json:
        print(json.dumps(report, indent=2))
        return

    if args.markdown:
        demo_summary = dict(report.get("demo_summary") or {})
        route_summary = dict(demo_summary.get("route_summary") or {})
        steps = route_summary.get("steps") or []
        print("# Minimal VLN Demo")
        print("")
        print(f"- instruction: `{demo_summary.get('instruction')}`")
        print(f"- result_category: `{demo_summary.get('result_category')}`")
        print(f"- matched_template: `{demo_summary.get('matched_template')}`")
        print(f"- goal_room: `{(demo_summary.get('resolved_goal_room_summary') or {}).get('resolved_goal_room_id')}`")
        print(f"- room_sequence: `{route_summary.get('room_sequence')}`")
        print(f"- teacher_explanation: {demo_summary.get('teacher_explanation')}")
        if steps:
            print("")
            print("## Route Steps")
            for item in steps:
                print(f"- {item}")
        return

    _print_text_summary(report)


if __name__ == "__main__":
    main()
