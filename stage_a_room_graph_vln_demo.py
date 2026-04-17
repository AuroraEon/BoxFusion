from __future__ import annotations

import argparse
import json
from pathlib import Path

from boxfusion.room_graph_vln_demo import (
    RoomGraphVLNDemo,
    build_room_graph_vln_demo,
    write_room_graph_vln_demo,
)


def _default_html_out(scene_root: Path, sequence_id: str, query_slug: str) -> Path:
    safe_slug = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in query_slug.lower()).strip("_")
    safe_slug = safe_slug[:80] or "query"
    return scene_root / "final" / f"{sequence_id}_room_graph_vln_{safe_slug}.html"


def _default_json_out(html_out: Path) -> Path:
    return html_out.with_suffix(".json")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render a lightweight room-graph VLN visualization demo from committed/public BoxFusion exports."
    )
    parser.add_argument("--scene-root", default=None, help="Scene root containing logs/summary.json and related exports.")
    parser.add_argument("--summary-json", default=None, help="Optional logs/summary.json path.")
    parser.add_argument("--topology-json", default=None, help="Optional logs/topology_v0_1.json path.")
    parser.add_argument(
        "--committed-room-world-model-json",
        default=None,
        help="Optional logs/committed_room_world_model_v0_1.json path.",
    )
    parser.add_argument("--start-room", default=None, help="Start room id or room name if available.")
    parser.add_argument("--goal-room", default=None, help="Explicit goal room id or room name if available.")
    parser.add_argument(
        "--semantic-target",
        default=None,
        help="Object/semantic target resolved against committed room summaries, restricted to public topology rooms.",
    )
    parser.add_argument("--route-policy", default="balanced", help="Route policy preset from RoomTopologyQueryAPI.")
    parser.add_argument("--title", default=None, help="Optional custom page title.")
    parser.add_argument("--html-out", default=None, help="Output HTML path.")
    parser.add_argument("--json-out", default=None, help="Optional JSON summary path.")
    parser.add_argument("--print-json", action="store_true", help="Print the JSON summary to stdout.")
    args = parser.parse_args()

    if bool(args.goal_room) == bool(args.semantic_target):
        raise SystemExit("Provide exactly one of --goal-room or --semantic-target.")

    demo = RoomGraphVLNDemo.from_inputs(
        scene_root=None if args.scene_root is None else Path(args.scene_root),
        summary_json=None if args.summary_json is None else Path(args.summary_json),
        topology_json=None if args.topology_json is None else Path(args.topology_json),
        committed_room_world_model_json=(
            None if args.committed_room_world_model_json is None else Path(args.committed_room_world_model_json)
        ),
    )
    demo_result = build_room_graph_vln_demo(
        scene_root=None if args.scene_root is None else Path(args.scene_root),
        summary_json=None if args.summary_json is None else Path(args.summary_json),
        topology_json=None if args.topology_json is None else Path(args.topology_json),
        committed_room_world_model_json=(
            None if args.committed_room_world_model_json is None else Path(args.committed_room_world_model_json)
        ),
        start_room=args.start_room,
        goal_room=args.goal_room,
        semantic_target=args.semantic_target,
        route_policy=args.route_policy,
        title=args.title,
    )

    sequence_id = str(demo_result.get("sequence_id") or demo.sequence_id or "sequence")
    query_slug = args.goal_room if args.goal_room is not None else args.semantic_target
    scene_root = demo.artifact_paths.scene_root or demo.artifact_paths.topology_json.parent.parent
    html_out = (
        Path(args.html_out)
        if args.html_out is not None
        else _default_html_out(scene_root, sequence_id, query_slug or "query")
    )
    json_out = None if args.json_out is None and not args.print_json else (
        _default_json_out(html_out) if args.json_out is None else Path(args.json_out)
    )
    outputs = write_room_graph_vln_demo(
        demo_result,
        demo=demo,
        html_out=html_out,
        json_out=json_out,
    )

    print(f"sequence_id: {sequence_id}")
    print(f"ok: {demo_result.get('ok')}")
    print(f"start_room: {(demo_result.get('start_resolution') or {}).get('resolved_room_id')}")
    print(f"goal_room: {(demo_result.get('goal_resolution') or {}).get('resolved_room_id')}")
    print(f"room_sequence: {(demo_result.get('route') or {}).get('room_sequence')}")
    print(f"next_hop: {(demo_result.get('next_hop') or {}).get('room_id')}")
    for key, value in outputs.items():
        print(f"{key}: {value}")

    if args.print_json:
        print(json.dumps(demo_result, indent=2))


if __name__ == "__main__":
    main()
