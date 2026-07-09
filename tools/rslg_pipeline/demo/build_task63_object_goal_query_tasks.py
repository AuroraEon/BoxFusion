#!/usr/bin/env python3
"""RSLG-SLAM task63 task-local QueryTask generator.

Reads ``06_selected_object_goals.json`` and emits, for each selected goal:

1. An *object-goal* QueryTask (schema ``rslg_query_task``) that carries the
   object identity (id/category/room/floor) and the RSLG-SLAM guardrails. The
   blocked legacy candidate ``generated_ring_037`` must never be selected and is
   recorded only as forbidden/rejected evidence; the non-transition edge
   ``vt_1_centerline_e003`` is not used as a transition edge. These object-goal
   *requests* are the
   validated task-local artifacts written under ``generated_query_tasks/``.

2. A *route-generation* QueryTask (room-target) written under
   ``generalization_pack/route_gen_query_tasks/``. Because the canonical object
   approach resolver is single-object (obj_175 -> generated_ring_002), object
   query types would mis-attribute obj_175's approach to other objects. So for
   non-reference goals the RouteResult is produced by routing to the *room that
   contains the object* via the generic room-route planner (``room_gateway`` for
   same-floor, ``cross_floor_room`` for cross-floor). The object centroid is the
   in-room semantic target.

All generated QueryTasks are task-local. This tool never edits
``configs/rslg_queryset_v0/`` and never runs ROS/Gazebo/Nav2/Stage-A.
RSLG-SLAM is the project name; ``BoxFusion`` is only a historical repository path.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.planning import query_task as qt


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_object_goal_query(goal: dict[str, Any], scene_id: str) -> dict[str, Any]:
    qtype = goal["object_goal_query_type"]
    cross_floor = goal["is_cross_floor"]
    if cross_floor:
        text = (
            f"Find the {goal['category']} object ({goal['object_id']}) in {goal['room_id']} "
            f"on {goal['floor_id']}, starting from {goal['start_room_id']} on {goal['start_floor_id']}."
        )
    else:
        text = (
            f"Go to the {goal['category']} object ({goal['object_id']}) in {goal['room_id']} "
            f"on {goal['floor_id']}."
        )
    forbidden_edges = [goal["forbidden_transition_edges"][0]] if goal["forbidden_transition_edges"] else []
    task = qt.new_query_task(
        query_id=goal["generated_query_id"],
        query_text=text,
        query_type=qtype,
        scene_id=scene_id,
        start={
            "start_pose": None,
            "start_room_id": goal["start_room_id"],
            "start_floor_id": goal["start_floor_id"],
        },
        target={
            "target_type": "object",
            "target_object_id": goal["object_id"],
            "target_object_category": goal["category"],
            "target_room_id": goal["room_id"],
            "target_floor_id": goal["floor_id"],
            "target_gateway_id": None,
        },
        expected={
            "expected_object_id": goal["object_id"],
            "expected_room_id": goal["room_id"],
            "expected_floor_id": goal["floor_id"],
            "expected_room_sequence": [],
            "expected_gateway_sequence": [],
            "expected_vertical_connector": "vt_1" if cross_floor else None,
            "expected_approach_candidate_id": None,
            "forbidden_runtime_goals": list(goal["forbidden_runtime_goals"]),
            "forbidden_transition_edges": forbidden_edges,
        },
        support={
            "requires_object": True,
            "requires_room": True,
            "requires_gateway": cross_floor,
            "requires_floor": True,
            "requires_route_contract": True,
            "dualmap_native_support": "object_proxy",
        },
        evaluation_modes={
            "planner_static": True,
            "runtime_static": True,
            "rviz_demo": True,
            "gazebo_smoke": goal["expected_route_type"] == "same_floor_pid",
        },
    )
    return task


def build_route_gen_query(goal: dict[str, Any], scene_id: str) -> dict[str, Any] | None:
    if goal["route_gen_query_type"] == "reference_reuse":
        return None
    qtype = goal["route_gen_query_type"]  # room_gateway | cross_floor_room
    cross_floor = goal["is_cross_floor"]
    text = (
        f"Route to {goal['room_id']} on {goal['floor_id']} (room containing "
        f"{goal['category']} {goal['object_id']}), starting from {goal['start_room_id']} "
        f"on {goal['start_floor_id']}."
    )
    forbidden_edges = ["vt_1_centerline_e003"] if cross_floor else []
    task = qt.new_query_task(
        query_id=f"{goal['generated_query_id']}_routegen",
        query_text=text,
        query_type=qtype,
        scene_id=scene_id,
        start={
            "start_pose": None,
            "start_room_id": goal["start_room_id"],
            "start_floor_id": goal["start_floor_id"],
        },
        target={
            "target_type": "room",
            "target_object_id": None,
            "target_object_category": None,
            "target_room_id": goal["room_id"],
            "target_floor_id": goal["floor_id"],
            "target_gateway_id": None,
        },
        expected={
            "expected_object_id": None,
            "expected_room_id": goal["room_id"],
            "expected_floor_id": goal["floor_id"],
            "expected_room_sequence": [],
            "expected_gateway_sequence": [],
            "expected_vertical_connector": "vt_1" if cross_floor else None,
            "expected_approach_candidate_id": None,
            "forbidden_runtime_goals": [],
            "forbidden_transition_edges": forbidden_edges,
        },
        support={
            "requires_object": False,
            "requires_room": True,
            "requires_gateway": True,
            "requires_floor": True,
            "requires_route_contract": True,
            "dualmap_native_support": "native",
        },
        evaluation_modes={
            "planner_static": True,
            "runtime_static": True,
            "rviz_demo": True,
            "gazebo_smoke": False,
        },
    )
    return task


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selected-json", required=True)
    parser.add_argument("--object-goal-dir", required=True,
                        help="Task-local generated_query_tasks/ directory.")
    parser.add_argument("--route-gen-dir", required=True,
                        help="generalization_pack/route_gen_query_tasks/ directory.")
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--summary-md", required=True)
    args = parser.parse_args()

    sel = json.loads(Path(args.selected_json).read_text(encoding="utf-8"))
    scene_id = sel.get("scene_id", "00843-DYehNKdT76V")
    goal_dir = Path(args.object_goal_dir)
    route_gen_dir = Path(args.route_gen_dir)
    goal_dir.mkdir(parents=True, exist_ok=True)
    route_gen_dir.mkdir(parents=True, exist_ok=True)

    records: list[dict[str, Any]] = []
    all_valid = True
    for goal in sel["selected_goals"]:
        og = build_object_goal_query(goal, scene_id)
        og_ok, og_errors = qt.validate(og)
        all_valid = all_valid and og_ok
        og_path = goal_dir / f"{goal['generated_query_id']}.json"
        og_path.write_text(qt.dump(og), encoding="utf-8")

        rg = build_route_gen_query(goal, scene_id)
        rg_path = None
        rg_ok = None
        rg_errors: list[str] = []
        if rg is not None:
            rg_ok, rg_errors = qt.validate(rg)
            all_valid = all_valid and rg_ok
            rg_path = route_gen_dir / f"{rg['query_id']}.json"
            rg_path.write_text(qt.dump(rg), encoding="utf-8")

        records.append({
            "generated_query_id": goal["generated_query_id"],
            "object_id": goal["object_id"],
            "category": goal["category"],
            "room_id": goal["room_id"],
            "floor_id": goal["floor_id"],
            "object_goal_query_type": goal["object_goal_query_type"],
            "object_goal_query_path": og_path.as_posix(),
            "object_goal_query_valid": og_ok,
            "object_goal_query_errors": og_errors,
            "route_gen_query_type": goal["route_gen_query_type"],
            "route_gen_query_path": rg_path.as_posix() if rg_path else None,
            "route_gen_query_valid": rg_ok,
            "route_gen_query_errors": rg_errors,
            "reuse_existing_route": goal["reuse_existing_route"],
        })

    summary = {
        "schema_name": "rslg_task63_generated_query_task_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": scene_id,
        "generated_utc": utc_now(),
        "object_goal_query_task_dir": goal_dir.as_posix(),
        "route_gen_query_task_dir": route_gen_dir.as_posix(),
        "generated_object_goal_query_count": len(records),
        "generated_route_gen_query_count": sum(1 for r in records if r["route_gen_query_path"]),
        "all_generated_query_tasks_valid": all_valid,
        "note": (
            "Object-goal QueryTasks carry the object identity and are the validated "
            "task-local requests. Route-generation QueryTasks are room-targeted and "
            "drive RouteResult generation via the generic room-route planner to avoid "
            "mis-attributing obj_175's single-object approach to other objects."
        ),
        "records": records,
    }
    Path(args.summary_json).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    lines = ["# task63 Generated QueryTask Summary", "",
             f"Scene: `{scene_id}` | Project: RSLG-SLAM", "",
             f"- Object-goal QueryTasks: **{summary['generated_object_goal_query_count']}** (under `generated_query_tasks/`)",
             f"- Route-generation QueryTasks: **{summary['generated_route_gen_query_count']}** (under `generalization_pack/route_gen_query_tasks/`)",
             f"- All generated QueryTasks valid: **{all_valid}**", "",
             summary["note"], "",
             "| generated_query_id | object_goal_type | valid | route_gen_type | route_gen_valid |",
             "|--------------------|------------------|-------|----------------|-----------------|"]
    for r in records:
        lines.append(
            f"| `{r['generated_query_id']}` | {r['object_goal_query_type']} | "
            f"{r['object_goal_query_valid']} | {r['route_gen_query_type']} | {r['route_gen_query_valid']} |"
        )
    lines.append("")
    Path(args.summary_md).write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps({
        "ok": all_valid,
        "object_goal_queries": summary["generated_object_goal_query_count"],
        "route_gen_queries": summary["generated_route_gen_query_count"],
        "all_valid": all_valid,
    }, indent=2))
    return 0 if all_valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
