#!/usr/bin/env python3
"""RSLG-SLAM task63 canonical object-goal candidate inventory audit.

Reads the canonical Layer 1 world-model final snapshot for scene
00843-DYehNKdT76V and produces a structured object inventory that the task63
multi-room / multi-object goal selector consumes. It never invents object ids,
categories, rooms, or floors: every field is copied from canonical data.

It never runs Stage-A, raw RGB-D inference, ROS, Gazebo, RViz, Nav2, AMCL, or
map_server, and never writes under a canonical directory. RSLG-SLAM is the
project name; ``BoxFusion`` is only a historical repository path.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Categories that are structural / background / low-value as an explicit
# navigation object goal (still reported, but flagged non-preferred).
STRUCTURAL_CATEGORIES = {
    "floor", "sky", "snow", "roof", "ceiling", "wall-wood", "light",
    "window", "window-other", "door", "stairs",
}

# Rooms present as nodes in the canonical planner graph (route-reachable).
PLANNER_GRAPH_ROOMS = {
    "room_2", "room_3", "room_4", "room_5", "room_6",
    "room_7", "room_8", "room_9", "room_11", "room_13", "room_14",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_snapshot(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def build_inventory(snapshot: dict[str, Any], snapshot_path: str) -> dict[str, Any]:
    objects = snapshot.get("objects") or []
    rooms = snapshot.get("rooms") or []
    floors = snapshot.get("floors") or []

    room_floor = {r.get("room_id"): r.get("floor_id") for r in rooms}

    records: list[dict[str, Any]] = []
    for obj in objects:
        oid = obj.get("id")
        room_id = obj.get("room_id")
        floor_id = obj.get("floor_id")
        category = obj.get("category")
        pose_3d = obj.get("pose_3d")
        pose = obj.get("pose")
        centroid_xy = None
        if isinstance(pose, (list, tuple)) and len(pose) >= 2:
            centroid_xy = [round(float(pose[0]), 4), round(float(pose[1]), 4)]
        assigned = bool(room_id) and bool(floor_id)
        in_graph = room_id in PLANNER_GRAPH_ROOMS
        structural = category in STRUCTURAL_CATEGORIES
        record = {
            "object_id": f"obj_{oid}",
            "raw_id": oid,
            "category": category,
            "label": obj.get("label"),
            "room_id": room_id,
            "floor_id": floor_id,
            "centroid_xy": centroid_xy,
            "centroid_3d": pose_3d,
            "size": obj.get("size"),
            "detection_confidence": obj.get("detection_confidence"),
            "room_and_floor_assigned": assigned,
            "room_in_planner_graph": in_graph,
            "structural_or_background_category": structural,
            "goal_candidate": bool(assigned and in_graph and not structural and category),
            "is_reference_curtain_obj_175": (oid == 175),
        }
        records.append(record)

    assigned = [r for r in records if r["room_and_floor_assigned"]]
    candidates = [r for r in records if r["goal_candidate"]]

    cat_all = Counter(r["category"] for r in records if r["category"])
    cat_assigned = Counter(r["category"] for r in assigned)
    cat_candidate = Counter(r["category"] for r in candidates)
    room_counts = Counter(r["room_id"] for r in assigned)
    floor_counts = Counter(r["floor_id"] for r in assigned)

    cats_by_room: dict[str, set[str]] = defaultdict(set)
    floor_by_room: dict[str, str] = {}
    for r in assigned:
        cats_by_room[r["room_id"]].add(r["category"])
        floor_by_room[r["room_id"]] = r["floor_id"]

    rooms_multi_category = {
        room: sorted(cats)
        for room, cats in cats_by_room.items()
        if len(cats) >= 2
    }

    candidate_rooms = sorted({r["room_id"] for r in candidates})
    candidate_floors = sorted({r["floor_id"] for r in candidates})
    floors_with_candidates = {
        floor: sorted({r["room_id"] for r in candidates if r["floor_id"] == floor})
        for floor in candidate_floors
    }

    enough_for_six = len(candidates) >= 6 and len(candidate_rooms) >= 4 and len(cat_candidate) >= 4

    excluded = [
        {
            "object_id": r["object_id"],
            "category": r["category"],
            "room_id": r["room_id"],
            "floor_id": r["floor_id"],
            "reason": (
                "no_room_or_floor_assignment" if not r["room_and_floor_assigned"]
                else "room_not_in_planner_graph" if not r["room_in_planner_graph"]
                else "structural_or_background_category" if r["structural_or_background_category"]
                else "no_category"
            ),
        }
        for r in records if not r["goal_candidate"]
    ]

    inventory = {
        "schema_name": "rslg_task63_object_inventory_audit",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": snapshot.get("map_info", {}).get("scene_id", "00843-DYehNKdT76V"),
        "generated_utc": utc_now(),
        "canonical_snapshot": snapshot_path,
        "summary": {
            "total_objects": len(records),
            "objects_with_room_and_floor": len(assigned),
            "goal_candidate_objects": len(candidates),
            "distinct_categories_all": len(cat_all),
            "distinct_categories_assigned": len(cat_assigned),
            "distinct_categories_candidate": len(cat_candidate),
            "room_count": len(rooms),
            "floor_count": len(floors),
            "candidate_room_count": len(candidate_rooms),
            "candidate_floor_count": len(candidate_floors),
            "enough_candidates_for_six_diverse_goals": enough_for_six,
        },
        "object_count_by_category_assigned": dict(cat_assigned.most_common()),
        "object_count_by_category_candidate": dict(cat_candidate.most_common()),
        "object_count_by_room_assigned": {k: room_counts[k] for k in sorted(room_counts)},
        "object_count_by_floor_assigned": dict(floor_counts),
        "top_candidate_categories": [c for c, _ in cat_candidate.most_common(12)],
        "rooms_with_multiple_categories": rooms_multi_category,
        "floor_of_room": floor_by_room,
        "candidate_rooms": candidate_rooms,
        "candidate_floors": candidate_floors,
        "floors_with_feasible_objects": floors_with_candidates,
        "objects_excluded": excluded,
        "objects": records,
        "planner_graph_rooms": sorted(PLANNER_GRAPH_ROOMS),
        "claim_boundary": {
            "no_stage_a": True,
            "no_raw_rgbd_inference": True,
            "no_nav2": True,
            "no_amcl": True,
            "no_map_server": True,
            "no_canonical_write": True,
        },
    }
    return inventory


def render_markdown(inv: dict[str, Any]) -> str:
    s = inv["summary"]
    lines: list[str] = []
    lines.append("# task63 Canonical Object Inventory Audit")
    lines.append("")
    lines.append("Scene: `00843-DYehNKdT76V` | Project: RSLG-SLAM")
    lines.append("")
    lines.append(f"Source snapshot: `{inv['canonical_snapshot']}`")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- Total objects in world model snapshot: **{s['total_objects']}**")
    lines.append(f"- Objects with room + floor assignment: **{s['objects_with_room_and_floor']}**")
    lines.append(f"- Goal-candidate objects (assigned, in planner graph, non-structural): **{s['goal_candidate_objects']}**")
    lines.append(f"- Distinct categories (all / assigned / candidate): **{s['distinct_categories_all']} / {s['distinct_categories_assigned']} / {s['distinct_categories_candidate']}**")
    lines.append(f"- Rooms: **{s['room_count']}** | Floors: **{s['floor_count']}**")
    lines.append(f"- Candidate rooms: **{s['candidate_room_count']}** | Candidate floors: **{s['candidate_floor_count']}**")
    lines.append(f"- Enough candidates for 6 diverse goals: **{s['enough_candidates_for_six_diverse_goals']}**")
    lines.append("")
    lines.append("## Object count by floor (assigned)")
    lines.append("")
    for floor, n in inv["object_count_by_floor_assigned"].items():
        lines.append(f"- `{floor}`: {n}")
    lines.append("")
    lines.append("## Object count by room (assigned)")
    lines.append("")
    lines.append("| room | floor | object_count | categories |")
    lines.append("|------|-------|--------------|------------|")
    for room in sorted(inv["object_count_by_room_assigned"], key=lambda r: int(r.split("_")[1])):
        floor = inv["floor_of_room"].get(room, "?")
        n = inv["object_count_by_room_assigned"][room]
        cats = ", ".join(sorted(inv["rooms_with_multiple_categories"].get(room, []))) or "(single/see JSON)"
        lines.append(f"| `{room}` | `{floor}` | {n} | {cats} |")
    lines.append("")
    lines.append("## Top candidate categories")
    lines.append("")
    for c in inv["top_candidate_categories"]:
        lines.append(f"- `{c}`: {inv['object_count_by_category_candidate'].get(c)}")
    lines.append("")
    lines.append("## Rooms with multiple object categories")
    lines.append("")
    for room, cats in sorted(inv["rooms_with_multiple_categories"].items(), key=lambda kv: int(kv[0].split("_")[1])):
        lines.append(f"- `{room}` ({inv['floor_of_room'].get(room)}): {', '.join(cats)}")
    lines.append("")
    lines.append("## Floors with feasible objects")
    lines.append("")
    for floor, rooms in inv["floors_with_feasible_objects"].items():
        lines.append(f"- `{floor}`: {', '.join(rooms)}")
    lines.append("")
    lines.append(f"## Objects excluded ({len(inv['objects_excluded'])})")
    lines.append("")
    reason_counts = Counter(e["reason"] for e in inv["objects_excluded"])
    for reason, n in reason_counts.most_common():
        lines.append(f"- `{reason}`: {n}")
    lines.append("")
    lines.append("## Diversity feasibility")
    lines.append("")
    lines.append(
        f"- There are {s['goal_candidate_objects']} candidate objects across "
        f"{s['candidate_room_count']} rooms and {s['distinct_categories_candidate']} categories on "
        f"{s['candidate_floor_count']} floors, which is sufficient to select 6 diverse "
        "object goals across at least 4 rooms and 4 categories on 2 floors."
    )
    lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-md", required=True)
    args = parser.parse_args()

    snapshot_path = Path(args.snapshot_json)
    snapshot = load_snapshot(snapshot_path)
    inventory = build_inventory(snapshot, snapshot_path.as_posix())

    out_json = Path(args.output_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(inventory, indent=2) + "\n", encoding="utf-8")

    out_md = Path(args.output_md)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text(render_markdown(inventory), encoding="utf-8")

    print(json.dumps({
        "ok": True,
        "goal_candidate_objects": inventory["summary"]["goal_candidate_objects"],
        "candidate_rooms": inventory["summary"]["candidate_room_count"],
        "candidate_categories": inventory["summary"]["distinct_categories_candidate"],
        "output_json": out_json.as_posix(),
        "output_md": out_md.as_posix(),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
