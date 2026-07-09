#!/usr/bin/env python3
"""RSLG-SLAM unified runtime demo helper (task62).

This helper backs ``run_rslg_runtime_demo.sh``. It owns registry parsing,
route listing, path printing, and pure-static dry-run validation. It never
launches Gazebo/RViz, never runs Nav2/AMCL/map_server, never runs Stage-A or
raw RGB-D inference, and never modifies canonical artifacts, QueryTask configs,
or prior task outputs. Live Gazebo/RViz process launching is handled by the
shell wrapper, which reuses the stable task58/task60 tools.

Subcommands (selected via flags):
  --list                       Print supported demo routes.
  --print-paths [--query-id]   Print repo root, registry, tool, and route paths.
  --resolve --query-id --field Print a single resolved registry value (shell use).
  --dry-run --query-id         Pure-static dry-run validation for one route.
  --dry-run-all                Dry-run validation for every registered route.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Optional

_THIS = Path(__file__).resolve()
sys.path.insert(0, str(_THIS.parent))
import classify_demo_route as classifier  # noqa: E402


def find_repo_root(start: Optional[Path] = None) -> Path:
    cur = (start or _THIS).resolve()
    for candidate in [cur, *cur.parents]:
        if (candidate / "tools" / "rslg_pipeline").is_dir() and (candidate / "stage_outputs").is_dir():
            return candidate
    raise RuntimeError(f"Could not locate RSLG-SLAM repo root from {cur}")


REPO_ROOT = find_repo_root()
DEFAULT_REGISTRY = REPO_ROOT / "tools/rslg_pipeline/demo/rslg_demo_route_registry_v0_1.json"
DEFAULT_TASK62_DIR = (
    REPO_ROOT
    / "stage_outputs/rslg_slam/00843-DYehNKdT76V/tasks/task62_demo_package_cleanup_and_route_generalization"
)


def load_registry(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def route_by_id(registry: dict[str, Any], query_id: str) -> Optional[dict[str, Any]]:
    for route in registry.get("routes", []):
        if route.get("query_id") == query_id:
            return route
    return None


def _abs(rel: Optional[str]) -> Optional[Path]:
    if not rel:
        return None
    return REPO_ROOT / rel


def _exists(rel: Optional[str]) -> bool:
    p = _abs(rel)
    return bool(p and p.is_file())


def cmd_list(registry: dict[str, Any]) -> int:
    conv = registry.get("conventions", {})
    print(f"RSLG-SLAM demo route registry: {registry.get('registry_id')}")
    print(f"scene_id: {registry.get('scene_id')}  profile_id: {conv.get('profile_id')}")
    print("")
    header = f"{'query_id':<44} {'mode':<28} {'floor_filter':<12} live_modes"
    print(header)
    print("-" * len(header))
    for route in registry.get("routes", []):
        qid = route.get("query_id")
        mode = route.get("mode")
        floor = route.get("floor_id_filter") or "-"
        live = ",".join(route.get("supported_live_modes", []))
        print(f"{qid:<44} {mode:<28} {floor:<12} {live}")
    print("")
    print("Recommended commands:")
    for route in registry.get("routes", []):
        qid = route.get("query_id")
        mode = route.get("mode")
        dur = route.get("recommended_duration_sec")
        if mode == "same_floor_pid":
            print(
                f"  # {qid} (same-floor PID, floor_2, ~{dur}s)\n"
                f"  tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --query-id {qid} --dry-run"
            )
        elif mode == "scripted_stair_transition":
            print(
                f"  # {qid} (scripted stair transition, ~{dur}s)\n"
                f"  tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --query-id {qid} --dry-run"
            )
        else:
            print(
                f"  # {qid} ({mode})\n"
                f"  tools/rslg_pipeline/demo/run_rslg_runtime_demo.sh --query-id {qid} --dry-run"
            )
    return 0


def cmd_print_paths(registry: dict[str, Any], query_id: Optional[str], registry_path: Path) -> int:
    conv = registry.get("conventions", {})
    print(f"REPO_ROOT={REPO_ROOT}")
    print(f"REGISTRY={registry_path}")
    print(f"TASK62_OUTPUT_DIR={DEFAULT_TASK62_DIR}")
    print(f"PROFILE_JSON={_abs(conv.get('profile_json'))}")
    print(f"SAME_FLOOR_PID_TOOL={_abs(conv.get('same_floor_pid_tool'))}")
    print(f"SAME_FLOOR_RVIZ_TOOL={_abs(conv.get('same_floor_rviz_tool'))}")
    print(f"SCRIPTED_STAIR_TOOL={_abs(conv.get('scripted_stair_tool'))}")
    print(f"SAME_FLOOR_WORLD={_abs(conv.get('same_floor_world'))}")
    print(f"SCRIPTED_STAIR_WORLD={_abs(conv.get('scripted_stair_world'))}")
    print(f"SAME_FLOOR_RVIZ_CONFIG={_abs(conv.get('same_floor_rviz_config'))}")
    print(f"SCRIPTED_STAIR_RVIZ_CONFIG={_abs(conv.get('scripted_stair_rviz_config'))}")
    if query_id:
        route = route_by_id(registry, query_id)
        if not route:
            print(f"# unknown query_id: {query_id}", file=sys.stderr)
            return 2
        print("")
        print(f"QUERY_ID={query_id}")
        print(f"MODE={route.get('mode')}")
        print(f"FLOOR_ID_FILTER={route.get('floor_id_filter')}")
        print(f"QUERY_JSON={_abs(route.get('query_json'))}")
        print(f"ROUTE_RESULT_JSON={_abs(route.get('route_result_json'))}")
        print(f"PID_RUNTIME_INPUT_JSON={_abs(route.get('pid_runtime_input_json'))}")
        print(f"WORLD={_abs(route.get('world'))}")
        print(f"RVIZ_CONFIG={_abs(route.get('rviz_config'))}")
        print(f"TOOL={_abs(route.get('tool'))}")
        print(f"RECOMMENDED_DURATION_SEC={route.get('recommended_duration_sec')}")
    return 0


def cmd_resolve(registry: dict[str, Any], query_id: str, field: Optional[str]) -> int:
    route = route_by_id(registry, query_id)
    if not route:
        print(f"# unknown query_id: {query_id}", file=sys.stderr)
        return 2
    conv = registry.get("conventions", {})
    resolved = {
        "QUERY_ID": query_id,
        "MODE": route.get("mode"),
        "FLOOR_ID_FILTER": route.get("floor_id_filter") or "",
        "PROFILE_ID": conv.get("profile_id"),
        "WORLD": str(_abs(route.get("world")) or ""),
        "RVIZ_CONFIG": str(_abs(route.get("rviz_config")) or ""),
        "TOOL": str(_abs(route.get("tool")) or ""),
        "RVIZ_TOOL": str(_abs(route.get("rviz_tool")) or ""),
        "RECOMMENDED_DURATION_SEC": str(route.get("recommended_duration_sec") or ""),
        "SUPPORTED_LIVE_MODES": ",".join(route.get("supported_live_modes", [])),
    }
    if field:
        if field not in resolved:
            print(f"# unknown field: {field}", file=sys.stderr)
            return 2
        print(resolved[field])
        return 0
    for key, value in resolved.items():
        print(f"{key}={value}")
    return 0


def dry_run_route(registry: dict[str, Any], route: dict[str, Any]) -> dict[str, Any]:
    query_id = route.get("query_id")
    mode = route.get("mode")
    conv = registry.get("conventions", {})

    route_result = classifier._load_json(str(_abs(route.get("route_result_json"))))
    pid_input = classifier._load_json(str(_abs(route.get("pid_runtime_input_json"))))
    query_json = classifier._load_json(str(_abs(route.get("query_json"))))

    classification = classifier.classify(
        query_id=query_id,
        route_result=route_result,
        pid_input=pid_input,
        query_json=query_json,
    )

    checks: dict[str, bool] = {
        "query_json_exists": _exists(route.get("query_json")),
        "route_result_exists": _exists(route.get("route_result_json")),
        "pid_runtime_input_exists": _exists(route.get("pid_runtime_input_json")),
        "profile_exists": _exists(conv.get("profile_json")),
    }

    floor_breakdown = classification.get("floor_breakdown", {})
    floors = classification.get("floors_in_waypoints", [])
    floor_sequence = list(floor_breakdown.keys())

    if mode == "same_floor_pid":
        floor_filter = route.get("floor_id_filter")
        checks["world_exists"] = _exists(route.get("world"))
        checks["rviz_config_exists"] = _exists(route.get("rviz_config"))
        checks["tool_exists"] = _exists(route.get("tool"))
        checks["floor_filter_has_waypoints"] = bool(
            floor_filter and floor_breakdown.get(floor_filter, 0) > 0
        )
        waypoint_count = floor_breakdown.get(floor_filter, 0)
    elif mode == "scripted_stair_transition":
        checks["world_exists"] = _exists(route.get("world"))
        checks["rviz_config_exists"] = _exists(route.get("rviz_config"))
        checks["tool_exists"] = _exists(route.get("tool"))
        checks["has_floor_1_segment"] = floor_breakdown.get("floor_1", 0) > 0
        checks["has_floor_2_segment"] = floor_breakdown.get("floor_2", 0) > 0
        checks["has_connector_handoff"] = classification.get("connector_handoff_count", 0) >= 1
        checks["transition_edge_is_valid"] = (
            classifier.VALID_TRANSITION_EDGE in classification.get("transition_edges", [])
        )
        waypoint_count = classification.get("runtime_waypoint_count")
    elif mode == "connector_scripted_transition":
        checks["world_exists"] = _exists(route.get("world"))
        checks["rviz_config_exists"] = _exists(route.get("rviz_config"))
        checks["transition_edge_is_valid"] = (
            classifier.VALID_TRANSITION_EDGE in classification.get("transition_edges", [])
        )
        checks["has_connector_handoff"] = classification.get("connector_handoff_count", 0) >= 1
        waypoint_count = classification.get("runtime_waypoint_count")
    else:  # dry_run_only
        checks["guard_route_result_available"] = _exists(route.get("route_result_json"))
        waypoint_count = classification.get("runtime_waypoint_count")

    guard = classification.get("guard", {})
    guard_ok = (
        not guard.get("forbidden_runtime_goal_selected", False)
        and not guard.get("forbidden_transition_edge_used_as_transition", False)
        and guard.get("physical_stair_climbing_claim", True) is False
    )
    checks["guard_forbidden_goal_not_selected"] = not guard.get(
        "forbidden_runtime_goal_selected", False
    )
    checks["guard_forbidden_edge_not_transition"] = not guard.get(
        "forbidden_transition_edge_used_as_transition", False
    )
    checks["guard_physical_stair_claim_false"] = guard.get("physical_stair_climbing_claim") is False

    dry_run_status = "passed" if all(checks.values()) else "failed"

    return {
        "query_id": query_id,
        "registered_mode": mode,
        "natural_route_mode": classification.get("natural_route_mode"),
        "dry_run_status": dry_run_status,
        "route_result_exists": checks["route_result_exists"],
        "runtime_input_exists": checks["pid_runtime_input_exists"],
        "floor_sequence": floor_sequence,
        "floor_breakdown": floor_breakdown,
        "floors_in_waypoints": floors,
        "waypoint_or_segment_count": waypoint_count,
        "route_segment_types": classification.get("route_segment_types"),
        "transition_edge_used": classification.get("transition_edges"),
        "forbidden_transition_edge_used_as_transition": guard.get(
            "forbidden_transition_edge_used_as_transition", False
        ),
        "forbidden_runtime_goal_selected": guard.get("forbidden_runtime_goal_selected", False),
        "forbidden_runtime_goal_in_rejected": classification.get("forbidden_goal_in_rejected"),
        "selected_runtime_goal": classification.get("selected_runtime_goal"),
        "physical_stair_climbing_claim": guard.get("physical_stair_climbing_claim", False),
        "guard_ok": guard_ok,
        "checks": checks,
        "recommended_execution_mode": mode,
        "attempt_headless_execution": bool(
            "headless" in route.get("supported_live_modes", []) and dry_run_status == "passed"
        ),
        "claim_boundary": classification.get("claim_boundary"),
    }


def cmd_dry_run(registry: dict[str, Any], query_id: str, output_dir: Optional[Path]) -> int:
    route = route_by_id(registry, query_id)
    if not route:
        print(f"# unknown query_id: {query_id}", file=sys.stderr)
        return 2
    result = dry_run_route(registry, route)
    text = json.dumps(result, indent=2)
    print(text)
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / f"dry_run_{query_id}.json").write_text(text + "\n", encoding="utf-8")
    return 0 if result["dry_run_status"] == "passed" else 1


def cmd_dry_run_all(registry: dict[str, Any], output_dir: Optional[Path]) -> int:
    results = [dry_run_route(registry, route) for route in registry.get("routes", [])]
    summary = {
        "registry_id": registry.get("registry_id"),
        "route_count": len(results),
        "passed": sum(1 for r in results if r["dry_run_status"] == "passed"),
        "failed": sum(1 for r in results if r["dry_run_status"] == "failed"),
        "guard_all_ok": all(r["guard_ok"] for r in results),
        "any_forbidden_goal_selected": any(r["forbidden_runtime_goal_selected"] for r in results),
        "any_forbidden_edge_transition": any(
            r["forbidden_transition_edge_used_as_transition"] for r in results
        ),
        "any_physical_stair_claim": any(r["physical_stair_climbing_claim"] for r in results),
        "routes": results,
    }
    text = json.dumps(summary, indent=2)
    print(text)
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "dry_run_all.json").write_text(text + "\n", encoding="utf-8")
    return 0 if summary["failed"] == 0 else 1


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--print-paths", action="store_true")
    parser.add_argument("--resolve", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-all", action="store_true")
    parser.add_argument("--query-id", default=None)
    parser.add_argument("--field", default=None)
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args(argv)

    registry_path = Path(args.registry)
    registry = load_registry(registry_path)
    output_dir = Path(args.output_dir) if args.output_dir else None

    if args.list:
        return cmd_list(registry)
    if args.print_paths:
        return cmd_print_paths(registry, args.query_id, registry_path)
    if args.resolve:
        if not args.query_id:
            print("--resolve requires --query-id", file=sys.stderr)
            return 64
        return cmd_resolve(registry, args.query_id, args.field)
    if args.dry_run_all:
        return cmd_dry_run_all(registry, output_dir)
    if args.dry_run:
        if not args.query_id:
            print("--dry-run requires --query-id (or use --dry-run-all)", file=sys.stderr)
            return 64
        return cmd_dry_run(registry, args.query_id, output_dir)

    print("No action specified. Use --list, --print-paths, --dry-run, or --dry-run-all.", file=sys.stderr)
    return 64


if __name__ == "__main__":
    sys.exit(main())
