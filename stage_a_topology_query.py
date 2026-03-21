import argparse
import json
from pathlib import Path

from boxfusion.query_api import RoomTopologyQueryAPI


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run structured room / anchor / object queries from a topology_v0_1 JSON export."
    )
    parser.add_argument("--topology-json", required=True, help="Path to logs/topology_v0_1.json")
    parser.add_argument("--start", required=True, help="Structured start room id, e.g. room_1")
    parser.add_argument("--goal-room", default=None, help="Structured goal room id, e.g. room_3")
    parser.add_argument("--anchor-id", default=None, help="Structured anchor id, e.g. anchor_obj_201")
    parser.add_argument("--object-id", default=None, help="Structured object id, e.g. obj_201")
    parser.add_argument("--object-label", default=None, help="Structured object label, e.g. sofa")
    parser.add_argument(
        "--route-policy",
        default="balanced",
        help="Route policy preset: strict | balanced | exploratory",
    )
    parser.add_argument(
        "--resolve-only",
        action="store_true",
        help="Only resolve the target into a room without running route search.",
    )
    args = parser.parse_args()

    query_api = RoomTopologyQueryAPI.from_json(Path(args.topology_json))
    target_mode_count = sum(
        [
            bool(args.goal_room),
            bool(args.anchor_id),
            bool(args.object_id or args.object_label),
        ]
    )
    if target_mode_count != 1:
        raise SystemExit("Choose exactly one target mode: --goal-room, --anchor-id, or --object-id/--object-label.")

    if args.goal_room:
        if args.resolve_only:
            result = query_api.resolve_room_target(args.goal_room)
        else:
            result = query_api.query_route(args.start, args.goal_room, route_policy=args.route_policy)
    elif args.anchor_id:
        if args.resolve_only:
            result = query_api.resolve_anchor_room(args.anchor_id)
        else:
            result = query_api.query_route_to_anchor(args.start, args.anchor_id, route_policy=args.route_policy)
    else:
        if args.resolve_only:
            result = query_api.resolve_object_room(object_id=args.object_id, object_label=args.object_label)
        else:
            result = query_api.query_route_to_object(
                args.start,
                object_id=args.object_id,
                object_label=args.object_label,
                route_policy=args.route_policy,
            )

    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
