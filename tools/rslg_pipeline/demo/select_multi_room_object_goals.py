#!/usr/bin/env python3
"""RSLG-SLAM task63 multi-room / multi-object goal selector.

Consumes the canonical object inventory audit (produced by
``audit_object_goal_candidates.py``) and selects a diverse set of object goals
according to the task63 selection policy: multiple rooms, multiple categories,
both floors, same-floor and cross-floor routes, curtain/obj_175 as a reference
case only. It never invents object ids; every selected object is resolved from
the canonical inventory. It performs no ROS/Gazebo/Nav2/Stage-A work.

RSLG-SLAM is the project name; ``BoxFusion`` is only a historical repository
path.
"""
from __future__ import annotations

import argparse
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Ordered preferred goal plan. Each entry names a category+room to select from
# the canonical inventory plus the runtime intent. object_id is resolved from
# the inventory, never invented. The first entry is the curtain/obj_175
# reference case (reuse existing validated task56c route).
PREFERRED_PLAN: list[dict[str, Any]] = [
    {
        "category": "curtain", "room_id": "room_14", "floor_id": "floor_2",
        "start_room_id": "room_2", "start_floor_id": "floor_1",
        "route_type": "scripted_stair_transition", "route_gen_query_type": "reference_reuse",
        "object_goal_query_type": "cross_floor_object",
        "is_reference": True, "force_object_id": "obj_175",
        "reason": "Reference case: previously validated cross-floor curtain/obj_175 route (reused, not primary evidence).",
    },
    {
        "category": "bed", "room_id": "room_11", "floor_id": "floor_2",
        "start_room_id": "room_7", "start_floor_id": "floor_2",
        "route_type": "same_floor_pid", "route_gen_query_type": "room_gateway",
        "object_goal_query_type": "object_in_room",
        "is_reference": False,
        "reason": "Same-floor floor_2 object goal in a non-curtain room (bed in room_11).",
    },
    {
        "category": "toilet", "room_id": "room_8", "floor_id": "floor_2",
        "start_room_id": "room_7", "start_floor_id": "floor_2",
        "route_type": "same_floor_pid", "route_gen_query_type": "room_gateway",
        "object_goal_query_type": "object_in_room",
        "is_reference": False,
        "reason": "Same-floor floor_2 object goal, bathroom fixture (toilet) in room_8.",
    },
    {
        "category": "Picture/Frame", "room_id": "room_9", "floor_id": "floor_2",
        "start_room_id": "room_7", "start_floor_id": "floor_2",
        "route_type": "same_floor_pid", "route_gen_query_type": "room_gateway",
        "object_goal_query_type": "object_in_room",
        "is_reference": False,
        "reason": "Same-floor floor_2 object goal, wall object (Picture/Frame) in room_9 (multi-hop).",
    },
    {
        "category": "couch", "room_id": "room_3", "floor_id": "floor_1",
        "start_room_id": "room_2", "start_floor_id": "floor_1",
        "route_type": "same_floor_pid", "route_gen_query_type": "room_gateway",
        "object_goal_query_type": "object_in_room",
        "is_reference": False,
        "reason": "Same-floor floor_1 object goal (couch in room_3) to demonstrate floor_1 generalization.",
    },
    {
        "category": "toilet", "room_id": "room_13", "floor_id": "floor_2",
        "start_room_id": "room_2", "start_floor_id": "floor_1",
        "route_type": "scripted_stair_transition", "route_gen_query_type": "cross_floor_room",
        "object_goal_query_type": "cross_floor_object",
        "is_reference": False,
        "reason": "Cross-floor scripted-stair object goal to a non-curtain object (toilet in room_13).",
    },
    {
        "category": "toilet", "room_id": "room_4", "floor_id": "floor_1",
        "start_room_id": "room_2", "start_floor_id": "floor_1",
        "route_type": "same_floor_pid", "route_gen_query_type": "room_gateway",
        "object_goal_query_type": "object_in_room",
        "is_reference": False,
        "reason": "Second same-floor floor_1 object goal (toilet in room_4) for route-length variety.",
    },
]

FORBIDDEN_RUNTIME_GOAL = "generated_ring_037"
NON_TRANSITION_EDGE = "vt_1_centerline_e003"
VALID_TRANSITION_EDGE = "vt_1_centerline_e001"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sanitize(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_").lower()


def resolve_object(inventory: dict[str, Any], category: str, room_id: str,
                   force_object_id: str | None) -> dict[str, Any] | None:
    objects = inventory["objects"]
    if force_object_id is not None:
        for o in objects:
            if o["object_id"] == force_object_id:
                return o
        return None
    matches = [
        o for o in objects
        if o["category"] == category and o["room_id"] == room_id and o["goal_candidate"]
    ]
    if not matches:
        return None
    matches.sort(key=lambda o: o["raw_id"])
    return matches[0]


def select(inventory: dict[str, Any]) -> dict[str, Any]:
    selected: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []

    for plan in PREFERRED_PLAN:
        obj = resolve_object(
            inventory, plan["category"], plan["room_id"], plan.get("force_object_id")
        )
        if obj is None:
            skipped.append({
                "category": plan["category"], "room_id": plan["room_id"],
                "reason": "no_matching_canonical_object_in_inventory",
            })
            continue

        cat_slug = sanitize(obj["category"])
        gen_id = f"task63_00843_{obj['object_id']}_{cat_slug}_{plan['room_id']}"
        cross_floor = plan["start_floor_id"] != obj["floor_id"]

        selected.append({
            "generated_query_id": gen_id,
            "object_id": obj["object_id"],
            "category": obj["category"],
            "room_id": obj["room_id"],
            "floor_id": obj["floor_id"],
            "object_centroid_xy": obj["centroid_xy"],
            "object_centroid_3d": obj["centroid_3d"],
            "object_size": obj["size"],
            "start_room_id": plan["start_room_id"],
            "start_floor_id": plan["start_floor_id"],
            "expected_route_type": plan["route_type"],
            "route_gen_query_type": plan["route_gen_query_type"],
            "object_goal_query_type": plan["object_goal_query_type"],
            "expected_final_room_id": obj["room_id"],
            "expected_final_floor_id": obj["floor_id"],
            "is_cross_floor": cross_floor,
            "is_reference_case": plan["is_reference"],
            "reuse_existing_route": plan["route_gen_query_type"] == "reference_reuse",
            "differs_from_obj_175": obj["object_id"] != "obj_175",
            "room_differs_from_room_14": obj["room_id"] != "room_14",
            "category_differs_from_curtain": obj["category"] != "curtain",
            "forbidden_runtime_goals": [FORBIDDEN_RUNTIME_GOAL],
            "forbidden_transition_edges": [NON_TRANSITION_EDGE],
            "valid_transition_edge": VALID_TRANSITION_EDGE if cross_floor else None,
            "selection_reason": plan["reason"],
        })

    categories = sorted({g["category"] for g in selected})
    rooms = sorted({g["room_id"] for g in selected})
    floors = sorted({g["floor_id"] for g in selected})
    same_floor = [g for g in selected if not g["is_cross_floor"]]
    cross_floor = [g for g in selected if g["is_cross_floor"]]

    return {
        "schema_name": "rslg_task63_selected_object_goals",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": inventory.get("scene_id", "00843-DYehNKdT76V"),
        "generated_utc": utc_now(),
        "inventory_source": inventory.get("canonical_snapshot"),
        "policy_summary": {
            "min_goals_target": 6,
            "selected_count": len(selected),
            "distinct_categories": len(categories),
            "distinct_rooms": len(rooms),
            "distinct_floors": len(floors),
            "same_floor_count": len(same_floor),
            "cross_floor_count": len(cross_floor),
            "not_obj_175_count": sum(1 for g in selected if g["differs_from_obj_175"]),
            "not_curtain_count": sum(1 for g in selected if g["category_differs_from_curtain"]),
            "not_room_14_count": sum(1 for g in selected if g["room_differs_from_room_14"]),
            "categories": categories,
            "rooms": rooms,
            "floors": floors,
        },
        "guards": {
            "forbidden_runtime_goal": FORBIDDEN_RUNTIME_GOAL,
            "forbidden_transition_edge": NON_TRANSITION_EDGE,
            "valid_transition_edge": VALID_TRANSITION_EDGE,
            "physical_stair_climbing_claim": False,
        },
        "selected_goals": selected,
        "skipped": skipped,
    }


def render_markdown(sel: dict[str, Any]) -> str:
    p = sel["policy_summary"]
    lines: list[str] = []
    lines.append("# task63 Selected Multi-Room / Multi-Object Goals")
    lines.append("")
    lines.append(f"Scene: `{sel['scene_id']}` | Project: RSLG-SLAM")
    lines.append("")
    lines.append("## Diversity summary")
    lines.append("")
    lines.append(f"- Selected goals: **{p['selected_count']}**")
    lines.append(f"- Distinct categories: **{p['distinct_categories']}** ({', '.join(p['categories'])})")
    lines.append(f"- Distinct rooms: **{p['distinct_rooms']}** ({', '.join(p['rooms'])})")
    lines.append(f"- Distinct floors: **{p['distinct_floors']}** ({', '.join(p['floors'])})")
    lines.append(f"- Same-floor routes: **{p['same_floor_count']}** | Cross-floor routes: **{p['cross_floor_count']}**")
    lines.append(f"- Goals not obj_175: **{p['not_obj_175_count']}** | not curtain: **{p['not_curtain_count']}** | not room_14: **{p['not_room_14_count']}**")
    lines.append("")
    lines.append("## Selected goals")
    lines.append("")
    lines.append("| generated_query_id | object | category | room | floor | start | route_type | reference |")
    lines.append("|--------------------|--------|----------|------|-------|-------|------------|-----------|")
    for g in sel["selected_goals"]:
        lines.append(
            f"| `{g['generated_query_id']}` | `{g['object_id']}` | {g['category']} | "
            f"`{g['room_id']}` | `{g['floor_id']}` | `{g['start_room_id']}`/`{g['start_floor_id']}` | "
            f"{g['expected_route_type']} | {'yes' if g['is_reference_case'] else 'no'} |"
        )
    lines.append("")
    lines.append("## Per-goal rationale")
    lines.append("")
    for g in sel["selected_goals"]:
        lines.append(f"### `{g['generated_query_id']}`")
        lines.append("")
        lines.append(f"- object_id: `{g['object_id']}` | category: `{g['category']}`")
        lines.append(f"- room: `{g['room_id']}` | floor: `{g['floor_id']}` | centroid_xy: {g['object_centroid_xy']}")
        lines.append(f"- start: `{g['start_room_id']}` / `{g['start_floor_id']}`")
        lines.append(f"- expected route type: `{g['expected_route_type']}` (route-gen query type: `{g['route_gen_query_type']}`)")
        lines.append(f"- cross-floor: {g['is_cross_floor']} | reference case: {g['is_reference_case']}")
        lines.append(f"- differs from obj_175: {g['differs_from_obj_175']} | not curtain: {g['category_differs_from_curtain']} | not room_14: {g['room_differs_from_room_14']}")
        lines.append(f"- reason: {g['selection_reason']}")
        lines.append("")
    if sel["skipped"]:
        lines.append("## Skipped plan entries")
        lines.append("")
        for s in sel["skipped"]:
            lines.append(f"- {s['category']} in {s['room_id']}: {s['reason']}")
        lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-md", required=True)
    args = parser.parse_args()

    inventory = json.loads(Path(args.inventory_json).read_text(encoding="utf-8"))
    sel = select(inventory)

    Path(args.output_json).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output_json).write_text(json.dumps(sel, indent=2) + "\n", encoding="utf-8")
    Path(args.output_md).write_text(render_markdown(sel), encoding="utf-8")

    print(json.dumps({"ok": True, **sel["policy_summary"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
