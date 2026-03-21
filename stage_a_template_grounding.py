import argparse
import json
from pathlib import Path

from boxfusion.template_grounding import TemplateGrounder


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run minimal rule-based Template Grounding v0.1 on a topology_v0_1 JSON export."
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
    args = parser.parse_args()

    grounder = TemplateGrounder.from_json(Path(args.topology_json))
    result = grounder.ground(
        instruction=args.instruction,
        start_room_id=args.start_room,
        route_policy=args.route_policy,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
