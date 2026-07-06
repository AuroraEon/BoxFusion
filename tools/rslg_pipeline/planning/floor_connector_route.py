"""Vertical connector / floor transition route helper for RSLG-SLAM.

Resolves the vertical connector sequence for a cross-floor route and produces a
connector/handoff route segment. Vertical movement is treated as a **semantic
handoff**, never as physical stair climbing. RSLG-SLAM is the project name;
``BoxFusion`` is only a historical repository path.

Guarantees:

* ``vt_1_centerline_e001`` is the transition edge.
* ``vt_1_centerline_e003`` is the non-transition edge and is never used for a
  transition.
* No Gazebo/RViz/ROS logic here.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import (
    NON_TRANSITION_EDGE,
    TRUE_TRANSITION_EDGE,
)


class TransitionEdgeGuardError(RuntimeError):
    """Raised if a connector would use the non-transition edge as a transition."""


def _connectors(connector_arts: dict[str, Any]) -> list[dict[str, Any]]:
    data = (
        connector_arts.get("artifacts", {})
        .get("vertical_connectors", {})
        .get("data")
    )
    if isinstance(data, dict):
        return data.get("connectors") or []
    return []


def resolve_connector(
    connector_arts: dict[str, Any],
    *,
    from_floor: str | None = None,
    to_floor: str | None = None,
    connector_id: str | None = None,
) -> dict[str, Any]:
    """Resolve the vertical connector for a floor transition.

    Selection prefers an explicit ``connector_id`` match, then a
    ``from_floor``/``to_floor`` match, then the single current connector. The
    transition edge is validated against the non-transition-edge guard.
    """

    connectors = _connectors(connector_arts)
    selected: dict[str, Any] | None = None
    reason = "unresolved"

    for connector in connectors:
        if connector_id and connector.get("connector_id") == connector_id:
            selected, reason = connector, "matched_connector_id"
            break
    if selected is None:
        for connector in connectors:
            if (
                from_floor
                and to_floor
                and connector.get("source_floor") == from_floor
                and connector.get("target_floor") == to_floor
            ):
                selected, reason = connector, "matched_floor_transition"
                break
    if selected is None and len(connectors) == 1:
        selected, reason = connectors[0], "single_scene_connector"

    if selected is None:
        # Centralized project-truth fallback for the current 00843 connector.
        return {
            "status": "fallback",
            "used_fallback": True,
            "connector_id": connector_id or "vt_1",
            "connector_id_alias": "vc_vt_1",
            "from_floor": from_floor or "floor_1",
            "to_floor": to_floor or "floor_2",
            "source_room": "room_3",
            "target_room": "room_7",
            "transition_edge": TRUE_TRANSITION_EDGE,
            "non_transition_edge": NON_TRANSITION_EDGE,
            "resolution_reason": "project_truth_fallback",
            "physical_stair_climbing_claimed": False,
        }

    transition_edge = selected.get("transition_edge") or TRUE_TRANSITION_EDGE
    if transition_edge == NON_TRANSITION_EDGE:
        raise TransitionEdgeGuardError(
            f"connector {selected.get('connector_id')!r} reports non-transition edge "
            f"{NON_TRANSITION_EDGE!r} as its transition edge"
        )

    return {
        "status": "ok",
        "used_fallback": False,
        "connector_id": selected.get("connector_id"),
        "connector_id_alias": selected.get("connector_id_alias"),
        "from_floor": selected.get("source_floor"),
        "to_floor": selected.get("target_floor"),
        "source_room": selected.get("source_room"),
        "target_room": selected.get("target_room"),
        "transition_edge": transition_edge,
        "non_transition_edge": selected.get("non_transition_edge") or NON_TRANSITION_EDGE,
        "endpoint_geometry": selected.get("layer1_endpoint_geometry"),
        "resolution_reason": reason,
        "physical_stair_climbing_claimed": False,
    }
