"""RSLGQueryTask: input unit for the RSLG-SLAM static Layer 3 planner.

This module owns the ``rslg_query_task`` schema constants, a construction
helper, JSON loading, and a self-contained semantic validator. A QueryTask is
the input to :mod:`tools.rslg_pipeline.plan_query_static`; an
``RSLGRouteResult`` is its output. RSLG-SLAM is the project name; ``BoxFusion``
is only a historical repository path.

The QueryTask carries the language/task query, the start/target descriptors, the
expected structured route truth, guardrails (forbidden runtime goals, forbidden
transition edges), DualMap support classification, and evaluation-mode flags.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    BLOCKED_LEGACY_APPROACH_IDS,
    NON_TRANSITION_EDGE,
    PROJECT_NAME,
)

SCHEMA_NAME = "rslg_query_task"
SCHEMA_VERSION = "0.1"
SCHEMA_PATH = Path(__file__).resolve().parents[1] / "schemas" / "query_task_schema.json"

QUERY_TYPES = (
    "object_to_path",
    "object_in_room",
    "room_gateway",
    "floor_connector",
    "cross_floor_room",
    "cross_floor_object",
)

TARGET_TYPES = ("object", "room", "gateway", "floor_connector")

DUALMAP_SUPPORT_VALUES = (
    "native",
    "object_proxy",
    "unsupported",
    "not_evaluated",
)

# Query types that target a concrete object and must therefore forbid the
# blocked legacy approach candidate as a runtime goal.
OBJECT_CENTRIC_QUERY_TYPES = (
    "object_to_path",
    "object_in_room",
    "cross_floor_object",
)

# Query types that cross floors and must forbid the non-transition edge.
CROSS_FLOOR_QUERY_TYPES = (
    "floor_connector",
    "cross_floor_room",
    "cross_floor_object",
)


def new_query_task(
    *,
    query_id: str,
    query_text: str,
    query_type: str,
    scene_id: str,
    start: dict[str, Any] | None = None,
    target: dict[str, Any] | None = None,
    expected: dict[str, Any] | None = None,
    support: dict[str, Any] | None = None,
    evaluation_modes: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Assemble a schema-consistent ``RSLGQueryTask`` dictionary."""

    return {
        "schema_name": SCHEMA_NAME,
        "schema_version": SCHEMA_VERSION,
        "project_name": PROJECT_NAME,
        "query_id": query_id,
        "query_text": query_text,
        "query_type": query_type,
        "scene_id": scene_id,
        "start": start
        or {"start_pose": None, "start_room_id": None, "start_floor_id": None},
        "target": target
        or {
            "target_type": None,
            "target_object_id": None,
            "target_object_category": None,
            "target_room_id": None,
            "target_floor_id": None,
            "target_gateway_id": None,
        },
        "expected": expected
        or {
            "expected_object_id": None,
            "expected_room_id": None,
            "expected_floor_id": None,
            "expected_room_sequence": [],
            "expected_gateway_sequence": [],
            "expected_vertical_connector": None,
            "expected_approach_candidate_id": None,
            "forbidden_runtime_goals": [],
            "forbidden_transition_edges": [],
        },
        "support": support
        or {
            "requires_object": None,
            "requires_room": None,
            "requires_gateway": None,
            "requires_floor": None,
            "requires_route_contract": None,
            "dualmap_native_support": "not_evaluated",
        },
        "evaluation_modes": evaluation_modes
        or {
            "planner_static": True,
            "runtime_static": False,
            "rviz_demo": False,
            "gazebo_smoke": False,
        },
    }


def _expected(data: dict[str, Any]) -> dict[str, Any]:
    expected = data.get("expected")
    return expected if isinstance(expected, dict) else {}


def _support(data: dict[str, Any]) -> dict[str, Any]:
    support = data.get("support")
    return support if isinstance(support, dict) else {}


def validate(data: dict[str, Any]) -> tuple[bool, list[str]]:
    """Validate an ``RSLGQueryTask`` payload against RSLG-SLAM rules."""

    errors: list[str] = []

    if data.get("schema_name") != SCHEMA_NAME:
        errors.append(f"schema_name must be {SCHEMA_NAME!r}")
    if not data.get("schema_version"):
        errors.append("schema_version is required")

    for key in ("query_id", "query_text", "query_type", "scene_id"):
        if not data.get(key):
            errors.append(f"{key} is required")

    query_type = data.get("query_type")
    if query_type and query_type not in QUERY_TYPES:
        errors.append(f"query_type {query_type!r} not in {QUERY_TYPES}")

    target = data.get("target") or {}
    target_type = target.get("target_type")
    if target_type and target_type not in TARGET_TYPES:
        errors.append(f"target.target_type {target_type!r} not in {TARGET_TYPES}")

    support = _support(data)
    dualmap = support.get("dualmap_native_support")
    if dualmap is not None and dualmap not in DUALMAP_SUPPORT_VALUES:
        errors.append(
            f"support.dualmap_native_support {dualmap!r} not in {DUALMAP_SUPPORT_VALUES}"
        )

    expected = _expected(data)
    forbidden_goals = expected.get("forbidden_runtime_goals") or []
    forbidden_edges = expected.get("forbidden_transition_edges") or []

    # Object-centric queries must forbid every blocked legacy approach id as a
    # runtime goal (guardrail against generated_ring_037 becoming a goal).
    if query_type in OBJECT_CENTRIC_QUERY_TYPES:
        for blocked_id in BLOCKED_LEGACY_APPROACH_IDS:
            if blocked_id not in forbidden_goals:
                errors.append(
                    f"expected.forbidden_runtime_goals must include blocked legacy "
                    f"approach {blocked_id!r} for {query_type!r} queries"
                )
        approach = expected.get("expected_approach_candidate_id")
        if approach and approach in BLOCKED_LEGACY_APPROACH_IDS:
            errors.append(
                f"expected.expected_approach_candidate_id must not be a blocked "
                f"legacy approach id ({approach!r})"
            )

    # Cross-floor queries must forbid the non-transition edge as a transition.
    if query_type in CROSS_FLOOR_QUERY_TYPES:
        if NON_TRANSITION_EDGE not in forbidden_edges:
            errors.append(
                f"expected.forbidden_transition_edges must include the non-transition "
                f"edge {NON_TRANSITION_EDGE!r} for {query_type!r} queries"
            )

    return not errors, errors


def load(path: Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_schema() -> dict[str, Any]:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def dump(data: dict[str, Any]) -> str:
    return json.dumps(data, indent=2, sort_keys=True) + "\n"
