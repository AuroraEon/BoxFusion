import argparse
import json
from pathlib import Path

from boxfusion.room_topology import RoomTopology


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Search room-level abstract routes from a topology_v0_1 JSON export."
    )
    parser.add_argument("--topology-json", required=True, help="Path to logs/topology_v0_1.json")
    parser.add_argument("--start", required=True, help="Structured start room id, e.g. room_1")
    parser.add_argument("--goal", required=True, help="Structured goal room id, e.g. room_3")
    parser.add_argument(
        "--allowed-relations",
        nargs="*",
        default=None,
        help="Optional subset of relation types to use, e.g. transition adjacent",
    )
    parser.add_argument("--min-conf", type=float, default=0.0, help="Minimum edge confidence to keep")
    parser.add_argument("--method", default="shortest", help="Routing method. Currently only 'shortest' is supported.")
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=1,
        help="Return up to N candidate abstract routes instead of only the best route.",
    )
    parser.add_argument(
        "--explain",
        action="store_true",
        help="Attach per-hop relation explanations for the selected route.",
    )
    args = parser.parse_args()

    topology = RoomTopology.from_json(Path(args.topology_json))
    if int(args.max_candidates) > 1:
        result = topology.find_candidate_room_paths(
            args.start,
            args.goal,
            allowed_relations=args.allowed_relations,
            min_conf=args.min_conf,
            method=args.method,
            max_paths=args.max_candidates,
        )
    elif args.explain:
        result = topology.explain_room_path(
            args.start,
            args.goal,
            allowed_relations=args.allowed_relations,
            min_conf=args.min_conf,
            method=args.method,
        )
    else:
        result = topology.find_room_path(
            args.start,
            args.goal,
            allowed_relations=args.allowed_relations,
            min_conf=args.min_conf,
            method=args.method,
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
