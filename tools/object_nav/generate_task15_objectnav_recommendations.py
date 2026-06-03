#!/usr/bin/env python3
"""Generate different-room object-nav demo recommendations from 00843 artifacts."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
OBJECT_NAV_DIR = Path(__file__).resolve().parent
if str(OBJECT_NAV_DIR) not in sys.path:
    sys.path.insert(0, str(OBJECT_NAV_DIR))

from object_nav_common import load_index, normalize_label  # noqa: E402


SCENE_ID = "00843-DYehNKdT76V"
CLEAN_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "clean_rerun"
TASKS_ROOT = ROOT / "stage_outputs/stage1_generalization" / SCENE_ID / "tasks"
TASK14A = TASKS_ROOT / "task14a_object_nav_experiment_adapter"
TASK14C = TASKS_ROOT / "task14c_multi_query_objectnav_runtime_validation"
TASK14D = TASKS_ROOT / "task14d_multi_object_object_facing_nav_validation"
ROUTE = CLEAN_ROOT / "routes/room_routes/floor_2_room11_to_room14/executable_route_waypoints_v0_1.json"
NOISY = {"floor", "sky", "snow", "ceiling", "wall_wood", "roof", "oyster"}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def rel(path: Path | None) -> str | None:
    if path is None:
        return None
    return str(path.resolve().relative_to(ROOT))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def candidate_report(object_id: str) -> tuple[Path | None, dict[str, Any]]:
    for source in (TASK14D, TASK14C):
        path = source / "approach_candidate_reports" / f"object_approach_report_{object_id}.json"
        if path.exists():
            return path, read_json(path, {})
    return None, {}


def previously_validated_ids() -> set[str]:
    summary = read_json(TASK14D / "per_query_runtime_results.json", {})
    return {
        row["object_id"]
        for row in summary.get("results", [])
        if row.get("object_facing_approach_success") is True
    }


def build_rows(index: dict[str, Any]) -> list[dict[str, Any]]:
    route_data = read_json(ROUTE, {})
    route_rooms = route_data.get("room_sequence") or []
    validated = previously_validated_ids()
    rows: list[dict[str, Any]] = []
    for obj in index.get("objects", []):
        object_id = str(obj.get("object_id"))
        label = str(obj.get("label") or "")
        norm = normalize_label(label)
        floor_id = obj.get("floor_id")
        room_id = obj.get("room_id")
        report_path, report = candidate_report(object_id)
        candidate = report.get("recommended_candidate") or {}
        approach_available = bool(candidate and candidate.get("world_xy"))
        yaw_proxy_available = bool(candidate.get("visible_proxy_xy"))
        target_route_available = bool(
            ROUTE.exists() and floor_id == "floor_2" and room_id == "room_14"
        )
        uncertain = obj.get("floor_assignment_status") != "stable"
        noisy = norm in NOISY
        expected_work: list[str] = []
        if object_id in validated:
            category = "immediately runnable with existing route and approach candidate"
            if uncertain:
                expected_work.append("retain floor-assignment uncertainty in reporting despite existing runtime evidence")
        elif noisy:
            category = "not recommended"
            expected_work.append("choose a semantically useful object label")
        elif uncertain:
            category = "floor assignment uncertain"
            expected_work.append("resolve or explicitly accept uncertain floor assignment")
        elif not target_route_available:
            category = "route asset missing"
            expected_work.append(f"build an executable destination route to {room_id}")
        elif not approach_available:
            category = "approach candidate missing"
            expected_work.append("generate and validate an object approach candidate")
        elif not yaw_proxy_available:
            category = "yaw proxy missing"
            expected_work.append("provide a facing target proxy for yaw alignment")
        else:
            category = "immediately runnable with existing route and approach candidate"
        if not approach_available:
            expected_work.append("generate and validate an approach candidate")
        if approach_available and not yaw_proxy_available:
            expected_work.append("derive a visible proxy or supported object target for facing")
        rows.append(
            {
                "query_string": f"{label.lower()} in {room_id} on {floor_id}",
                "object_id": object_id,
                "label": label,
                "room_id": room_id,
                "floor_id": floor_id,
                "classification": category,
                "route_availability": {
                    "available_for_destination_runtime": target_route_available,
                    "existing_route_id": route_data.get("route_id"),
                    "existing_room_sequence": route_rooms,
                    "reason": "existing executable route terminates in room_14"
                    if target_route_available
                    else "no existing executable destination route for this object room",
                },
                "approach_candidate_availability": approach_available,
                "approach_candidate_id": candidate.get("candidate_id"),
                "approach_report": rel(report_path),
                "yaw_proxy_availability": yaw_proxy_available,
                "floor_assignment_status": obj.get("floor_assignment_status"),
                "warnings": obj.get("warnings") or [],
                "expected_missing_work_before_runtime": list(dict.fromkeys(expected_work)),
                "task14d_runtime_success_already_observed": object_id in validated,
            }
        )
    return rows


def markdown(payload: dict[str, Any]) -> str:
    rows = payload["recommendations"]
    non_room14 = [r for r in rows if r["room_id"] != "room_14" and r["classification"] != "not recommended"]
    runnable = [r for r in rows if r["classification"].startswith("immediately")]
    table_rows = runnable + non_room14
    lines = [
        "# task15 Different-Room Object-Nav Recommendations",
        "",
        "This scan uses the `00843` clean rerun committed/public object index and existing object-nav outputs only. It does not add route or approach assets.",
        "",
        f"- Objects scanned: `{payload['objects_scanned']}`",
        f"- Immediately runnable objects: `{payload['classification_counts'].get('immediately runnable with existing route and approach candidate', 0)}`",
        f"- Immediately runnable non-room14 objects: `{payload['immediately_runnable_non_room14_count']}`",
        "- Existing executable destination route: `floor_2_room11_to_room14` only.",
        "",
        "## Candidate Table",
        "",
        "| query | object_id | label | room | floor | classification | route | approach | yaw proxy | missing work |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in table_rows:
        work = "; ".join(row["expected_missing_work_before_runtime"]) or "none"
        lines.append(
            f"| {row['query_string']} | {row['object_id']} | {row['label']} | {row['room_id']} | "
            f"{row['floor_id']} | {row['classification']} | "
            f"{row['route_availability']['available_for_destination_runtime']} | "
            f"{row['approach_candidate_availability']} | {row['yaw_proxy_availability']} | {work} |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "- No non-room14 object is immediately runnable from the current destination-route and approach assets.",
        "- Objects outside `room_14` first require destination route assets; most also require object approach and yaw-proxy assets.",
        "- `obj_177` remains reportable as already runtime-tested despite its recorded floor-assignment uncertainty.",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--index-json", type=Path, default=TASK14A / "object_candidate_index_v0_1.json")
    args = parser.parse_args()
    index = load_index(args.index_json)
    rows = build_rows(index)
    counts = Counter(r["classification"] for r in rows)
    payload = {
        "artifact_type": "task15_different_room_objectnav_recommendations",
        "created_utc": now_iso(),
        "project_name": "RSLG-SLAM",
        "stage_a_rerun": False,
        "reference_00824_modified": False,
        "active_clean_rerun_root": rel(CLEAN_ROOT),
        "source_object_index": rel(args.index_json),
        "source_route": rel(ROUTE),
        "objects_scanned": len(rows),
        "classification_counts": dict(sorted(counts.items())),
        "immediately_runnable_non_room14_count": sum(
            1
            for r in rows
            if r["room_id"] != "room_14"
            and r["classification"] == "immediately runnable with existing route and approach candidate"
        ),
        "recommendations": rows,
    }
    write_json(args.output_dir / "different_room_objectnav_recommendations.json", payload)
    (args.output_dir / "different_room_objectnav_recommendations.md").write_text(markdown(payload), encoding="utf-8")
    print(json.dumps(payload["classification_counts"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
