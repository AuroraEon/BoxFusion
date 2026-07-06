"""Canonical artifact loading utilities for the RSLG-SLAM static planner.

This module loads the current 00843 canonical Layer 2 / Layer 3 artifacts that
the generic static planner core consumes. RSLG-SLAM is the project name;
``BoxFusion`` is only a historical repository path.

The loaders are read-only. They never write under canonical and never launch
ROS/Gazebo/RViz/Nav2/AMCL/map_server. When an expected artifact path is missing,
each loader returns a structured status (``status="missing"`` with a ``missing``
list) instead of crashing, so the planner can degrade gracefully and record the
gap in provenance.

Public functions:

* :func:`load_scene_context` — resolve canonical roots and per-floor map paths.
* :func:`load_object_artifacts` — object query resolution / approach artifacts.
* :func:`load_topology_artifacts` — planner graph + cross-floor topology.
* :func:`load_connector_artifacts` — vertical connectors.
* :func:`load_stable_map_artifacts` — stable occupancy map metadata / yaml refs.
* :func:`load_current_route_comparison` — canonical real routes (comparison /
  provenance only, never the generated metric path).

Per-floor metric-planning loaders (used by ``metric_path_stitcher``):

* :func:`load_planner_graph_artifacts` — planner graph nodes/edges/bindings.
* :func:`load_route_anchor_points` — room representative points per floor.
* :func:`load_room_anchor_or_representative_point` — one room anchor.
* :func:`load_connector_endpoint_geometry` — connector entry/exit points.
* :func:`load_floor_stable_map_artifacts` — a floor's stable-map yaml/metadata.
* :func:`load_floor_occupancy_planner_inputs` — a floor's occupancy A* inputs.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:  # pragma: no cover - direct script execution
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.rslg_pipeline.project_truth import MAIN_SCENE_ID

REPO_ROOT = Path(__file__).resolve().parents[3]

L2 = "layer2_formal_artifacts"
L3 = "layer3_navigation_interface"
L4 = "layer4_runtime_validation"

# Occupancy A* inputs mirroring the canonical conservative real-route planner
# (route_planner_graph_v0_1 / cross_floor_room real route planner_parameters).
DEFAULT_OCCUPANCY_PLANNER_PARAMS = {
    "inflation_radius_m": 0.2,
    "waypoint_spacing_m": 0.2,
    "connectivity": 8,
    "maximum_anchor_snap_m": 1.0,
}


def _rel(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _artifact(path: Path, role: str, *, load: bool = True) -> dict[str, Any]:
    """Return a structured artifact record with existence + optional payload."""

    exists = path.is_file()
    record: dict[str, Any] = {
        "role": role,
        "path": _rel(path),
        "abs_path": path.as_posix(),
        "exists": exists,
    }
    if load and exists:
        record["data"] = _read_json(path)
    return record


def _status(records: dict[str, dict[str, Any]]) -> dict[str, Any]:
    missing = [r["role"] for r in records.values() if not r["exists"]]
    return {
        "status": "ok" if not missing else "missing",
        "missing": missing,
        "artifacts": records,
    }


def load_scene_context(canonical_root: str | Path) -> dict[str, Any]:
    """Resolve canonical roots, scene id, and per-floor stable map paths."""

    root = Path(canonical_root)
    l2 = root / L2
    l3 = root / L3
    l4 = root / L4
    floors = ["floor_1", "floor_2"]
    stable_map_yaml = {
        floor: l2 / "stable_maps" / floor / f"{floor}_stable_occupancy_map_v0_1.yaml"
        for floor in floors
    }
    return {
        "scene_id": MAIN_SCENE_ID,
        "canonical_root": root,
        "canonical_root_rel": _rel(root),
        "canonical_root_exists": root.is_dir(),
        "layer2_root": l2,
        "layer3_root": l3,
        "layer4_root": l4,
        "floors": floors,
        "stable_map_yaml": stable_map_yaml,
    }


def load_object_artifacts(context: dict[str, Any]) -> dict[str, Any]:
    """Load object query resolution and approach artifacts for the target object."""

    l2 = context["layer2_root"]
    obj = l2 / "object_interfaces"
    records = {
        "object_query_resolution": _artifact(
            obj / "object_query_resolution_v0_1.json", "object_query_resolution"
        ),
        "object_approach_selected": _artifact(
            obj / "object_approach_selected_v0_2.json", "object_approach_selected"
        ),
        "object_approach_candidates_recovery": _artifact(
            obj / "object_approach_candidates_recovery_v0_1.json",
            "object_approach_candidates_recovery",
        ),
        "object_interface_package": _artifact(
            obj / "object_interface_package_v0_2.json", "object_interface_package"
        ),
    }
    return _status(records)


def load_topology_artifacts(context: dict[str, Any]) -> dict[str, Any]:
    """Load the planner graph and cross-floor topology artifacts."""

    l2 = context["layer2_root"]
    records = {
        "route_planner_graph": _artifact(
            l2 / "planner_graph" / "route_planner_graph_v0_1.json", "route_planner_graph"
        ),
        "cross_floor_topology": _artifact(
            l2 / "topology" / "cross_floor_topology_v0_1.json", "cross_floor_topology"
        ),
    }
    return _status(records)


def load_connector_artifacts(context: dict[str, Any]) -> dict[str, Any]:
    """Load the vertical connector artifacts."""

    l2 = context["layer2_root"]
    records = {
        "vertical_connectors": _artifact(
            l2 / "vertical_connectors" / "vertical_connectors_v0_1.json",
            "vertical_connectors",
        ),
    }
    return _status(records)


def load_stable_map_artifacts(context: dict[str, Any]) -> dict[str, Any]:
    """Load stable occupancy map metadata + yaml references per floor.

    The heavy pgm/npz payloads are referenced by path, not read here, to keep
    the loader lightweight.
    """

    l2 = context["layer2_root"]
    records: dict[str, dict[str, Any]] = {
        "stable_occupancy_map_package": _artifact(
            l2 / "stable_maps" / "stable_occupancy_map_package_v0_1.json",
            "stable_occupancy_map_package",
        ),
    }
    for floor in context["floors"]:
        base = l2 / "stable_maps" / floor
        records[f"{floor}_metadata"] = _artifact(
            base / f"{floor}_stable_occupancy_map_metadata_v0_1.json",
            f"{floor}_stable_map_metadata",
        )
        # yaml is referenced (not JSON), so only record existence, not payload.
        records[f"{floor}_map_yaml"] = _artifact(
            base / f"{floor}_stable_occupancy_map_v0_1.yaml",
            f"{floor}_stable_map_yaml",
            load=False,
        )
    return _status(records)


def load_current_route_comparison(context: dict[str, Any]) -> dict[str, Any]:
    """Load canonical real routes used **only** as comparison / provenance.

    These are not the planner identity and are never copied into
    ``metric_path``. The planner stitches its own metric path dynamically
    (:mod:`tools.rslg_pipeline.planning.metric_path_stitcher`); these real-route
    files are loaded solely so a route result can record a
    ``canonical_comparison`` length alongside its dynamically stitched length.
    """

    l3 = context["layer3_root"]
    real = l3 / "real_routes"
    records = {
        "cross_floor_object_real_route": _artifact(
            real / "cross_floor_object_real_astar_route_conservative_canonical_v0_2.json",
            "cross_floor_object_real_route",
        ),
        "cross_floor_room_real_route": _artifact(
            real / "cross_floor_room_real_astar_route_conservative_canonical_v0_1.json",
            "cross_floor_room_real_route",
        ),
    }
    return _status(records)


def load_planner_graph_artifacts(context: dict[str, Any]) -> dict[str, Any]:
    """Load the planner graph with nodes indexed by id and floor map bindings.

    Returns a structured record with ``status`` (``ok``/``missing``), the raw
    ``artifact`` record, an index of ``nodes`` by ``node_id``, and
    ``floor_map_bindings``. This exposes the per-room ``planner_anchor_xy``
    representative points the stitcher plans A* segments between.
    """

    l2 = context["layer2_root"]
    record = _artifact(
        l2 / "planner_graph" / "route_planner_graph_v0_1.json", "route_planner_graph"
    )
    data = record.get("data") if record.get("exists") else None
    nodes: dict[str, dict[str, Any]] = {}
    bindings: dict[str, str] = {}
    invariants: dict[str, Any] = {}
    if isinstance(data, dict):
        for node in data.get("nodes") or []:
            node_id = node.get("node_id")
            if node_id:
                nodes[node_id] = node
        bindings = data.get("floor_map_bindings") or {}
        invariants = data.get("vertical_transition_invariants") or {}
    return {
        "status": "ok" if record.get("exists") else "missing",
        "missing": [] if record.get("exists") else ["route_planner_graph"],
        "artifact": record,
        "nodes": nodes,
        "floor_map_bindings": bindings,
        "vertical_transition_invariants": invariants,
    }


def load_route_anchor_points(context: dict[str, Any]) -> dict[str, Any]:
    """Return room representative points keyed by room id.

    Each entry is ``{"anchor_xy": [x, y], "floor_id": ..., "source": ...}``
    derived from the planner graph's ``planner_anchor_xy`` (the canonical Layer 2
    room polygon centroid). These are pure geometry anchors, not a canonical
    route path.
    """

    graph = load_planner_graph_artifacts(context)
    anchors: dict[str, dict[str, Any]] = {}
    for node_id, node in graph["nodes"].items():
        if node.get("node_type") != "room":
            continue
        anchor = node.get("planner_anchor_xy")
        if isinstance(anchor, (list, tuple)) and len(anchor) >= 2:
            anchors[node_id] = {
                "anchor_xy": [float(anchor[0]), float(anchor[1])],
                "floor_id": node.get("floor_id"),
                "source": "route_planner_graph_v0_1.planner_anchor_xy",
            }
    return {
        "status": "ok" if anchors else "missing",
        "missing": [] if anchors else ["planner_anchor_xy"],
        "anchors": anchors,
    }


def load_room_anchor_or_representative_point(
    context: dict[str, Any], room_id: str, floor_id: str | None = None
) -> dict[str, Any]:
    """Return one room's representative point with a structured missing status."""

    anchors = load_route_anchor_points(context)["anchors"]
    record = anchors.get(room_id)
    if record is None:
        return {
            "status": "missing",
            "room_id": room_id,
            "floor_id": floor_id,
            "anchor_xy": None,
            "reason": f"no planner_anchor_xy for room {room_id!r}",
        }
    return {
        "status": "ok",
        "room_id": room_id,
        "floor_id": floor_id or record.get("floor_id"),
        "anchor_xy": record["anchor_xy"],
        "source": record["source"],
    }


def load_connector_endpoint_geometry(
    context: dict[str, Any], connector_id: str | None = None
) -> dict[str, Any]:
    """Return a vertical connector's floor entry/exit points and transition edge.

    Reads ``layer1_endpoint_geometry`` from the canonical vertical connectors
    artifact. Returns a structured missing status if the connector or its
    endpoint geometry is absent, so the stitcher can centralize a documented
    fallback rather than crashing.
    """

    connector_arts = load_connector_artifacts(context)
    data = (
        connector_arts.get("artifacts", {})
        .get("vertical_connectors", {})
        .get("data")
    )
    connectors = data.get("connectors") if isinstance(data, dict) else None
    if not connectors:
        return {
            "status": "missing",
            "connector_id": connector_id,
            "reason": "no vertical connectors artifact / connectors list",
        }

    selected = None
    for connector in connectors:
        if connector_id is None or connector.get("connector_id") == connector_id:
            selected = connector
            break
    if selected is None:
        return {
            "status": "missing",
            "connector_id": connector_id,
            "reason": f"connector {connector_id!r} not found",
        }

    geometry = selected.get("layer1_endpoint_geometry") or {}
    entry = geometry.get("source_position_xy")
    exit_ = geometry.get("target_position_xy")
    if not (isinstance(entry, (list, tuple)) and isinstance(exit_, (list, tuple))):
        return {
            "status": "missing",
            "connector_id": selected.get("connector_id"),
            "reason": "connector endpoint geometry incomplete",
            "from_floor": selected.get("source_floor"),
            "to_floor": selected.get("target_floor"),
            "transition_edge": selected.get("transition_edge"),
        }
    return {
        "status": "ok",
        "connector_id": selected.get("connector_id"),
        "connector_id_alias": selected.get("connector_id_alias"),
        "from_floor": selected.get("source_floor"),
        "to_floor": selected.get("target_floor"),
        "source_room": selected.get("source_room"),
        "target_room": selected.get("target_room"),
        "transition_edge": selected.get("transition_edge"),
        "non_transition_edge": selected.get("non_transition_edge"),
        "entry_xy": [float(entry[0]), float(entry[1])],
        "exit_xy": [float(exit_[0]), float(exit_[1])],
        "z_span_m": geometry.get("z_span_m"),
        "source": "vertical_connectors_v0_1.layer1_endpoint_geometry",
    }


def load_floor_stable_map_artifacts(
    context: dict[str, Any], floor_id: str
) -> dict[str, Any]:
    """Return a floor's stable-map yaml path and metadata with missing status."""

    l2 = context["layer2_root"]
    base = l2 / "stable_maps" / floor_id
    yaml_path = base / f"{floor_id}_stable_occupancy_map_v0_1.yaml"
    metadata = _artifact(
        base / f"{floor_id}_stable_occupancy_map_metadata_v0_1.json",
        f"{floor_id}_stable_map_metadata",
    )
    yaml_record = _artifact(yaml_path, f"{floor_id}_stable_map_yaml", load=False)
    missing = [r["role"] for r in (metadata, yaml_record) if not r["exists"]]
    return {
        "status": "ok" if not missing else "missing",
        "missing": missing,
        "floor_id": floor_id,
        "map_yaml_path": yaml_path.as_posix(),
        "map_yaml_rel": _rel(yaml_path),
        "map_yaml_exists": yaml_record["exists"],
        "metadata": metadata,
    }


def load_floor_occupancy_planner_inputs(
    context: dict[str, Any], floor_id: str
) -> dict[str, Any]:
    """Return the occupancy-A* inputs for a floor (map yaml + planner params).

    The heavy pgm payload is loaded by ``occupancy_planner.OccupancyPlanner``
    from the returned ``map_yaml_path``; this loader only resolves the path and
    the A* parameters, keeping itself lightweight.
    """

    floor_map = load_floor_stable_map_artifacts(context, floor_id)
    return {
        "status": floor_map["status"],
        "missing": floor_map["missing"],
        "floor_id": floor_id,
        "map_yaml_path": floor_map["map_yaml_path"],
        "map_yaml_rel": floor_map["map_yaml_rel"],
        "map_yaml_exists": floor_map["map_yaml_exists"],
        "planner_params": dict(DEFAULT_OCCUPANCY_PLANNER_PARAMS),
    }


def rel(path: str | Path) -> str:
    """Public helper: repo-relative POSIX path for provenance records."""

    return _rel(Path(path))
