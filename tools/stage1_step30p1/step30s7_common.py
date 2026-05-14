#!/usr/bin/env python3
"""Shared Step30S7 request-aware route/map helpers."""

from __future__ import annotations

import heapq
import json
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CONFIG_REL = Path("config/step30s7_gateway_projection_config_v0_1.json")
OLD_STEP30S5_PROFILES = {"step30s5_room15_diagnostic_patch", "step30s5_room15_interior"}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def room_num(room_id: str) -> int:
    return int(str(room_id).split("_")[1])


def room_name(room_id_or_num: str | int) -> str:
    if isinstance(room_id_or_num, int):
        return f"room_{room_id_or_num}"
    text = str(room_id_or_num)
    return text if text.startswith("room_") else f"room_{int(text)}"


def pair_key(a: str | int, b: str | int) -> str:
    ai = room_num(room_name(a))
    bi = room_num(room_name(b))
    lo, hi = sorted((ai, bi))
    return f"r{lo}_r{hi}"


def ensure_step30s7_config(stage_output: Path) -> Path:
    """Create the scene config once from gateway artifacts if absent."""
    path = stage_output / CONFIG_REL
    if path.exists():
        return path
    selection_path = stage_output / "stage1_process/gateway_extraction/assets/00824_step30b2_auto_gateway_selection_v0_1.json"
    truth_path = stage_output / "gateway/gateway_truth_blind_selection_v0_1.json"
    selection = read_json(selection_path)
    truth = read_json(truth_path) if truth_path.exists() else {}
    selected_pairs: dict[str, str] = {}
    for key, item in (selection.get("per_pair") or {}).items():
        gateway_id = item.get("selected_primary_candidate_id")
        if gateway_id:
            selected_pairs[key] = gateway_id
    forbidden_pairs = sorted(
        set(truth.get("negative_sanity_pairs") or [])
        | set(truth.get("non_truth_pairs") or [])
        | {key for key in (selection.get("per_pair") or {}) if key not in selected_pairs}
    )
    config = {
        "artifact_type": "step30s7_gateway_projection_config",
        "version": "v0_1",
        "created_utc": now_iso(),
        "scene_id": selection.get("scene_id") or truth.get("scene_id"),
        "selected_gateway_pairs": selected_pairs,
        "forbidden_gateway_pairs": forbidden_pairs,
        "gateway_hypotheses_path": "stage1_process/gateway_extraction/assets/00824_step30b_gateway_hypotheses_with_roles_v0_1.json",
        "room_mask_path": "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy",
        "layered_bev_path": "stage1_process/room_segmentation/assets/00824_step30a_layered_bev_v0_1.npz",
        "h8r2_map_yaml": "maps/h8r2_gateway_preserving_nav_map.yaml",
        "h8r2_masks_npz": "maps/h8r2_gateway_preserving_masks.npz",
        "note": "Scene-specific gateway selection is externalized here so active route/projection code does not hard-code route_room_ids, selected gateway ids, or forbidden pairs.",
    }
    write_json(path, config)
    return path


def load_config(stage_output: Path) -> dict[str, Any]:
    return read_json(ensure_step30s7_config(stage_output))


def selected_edges(config: dict[str, Any]) -> dict[tuple[str, str], str]:
    edges: dict[tuple[str, str], str] = {}
    for key, gateway_id in (config.get("selected_gateway_pairs") or {}).items():
        left, right = key.split("_")
        a = room_name(left[1:])
        b = room_name(right[1:])
        edges[(a, b)] = gateway_id
    return edges


def shortest_room_route(config: dict[str, Any], start: str, goal: str, through: list[str]) -> list[str]:
    graph: dict[str, list[str]] = {}
    for a, b in selected_edges(config):
        graph.setdefault(a, []).append(b)
        graph.setdefault(b, []).append(a)

    def bfs(src: str, dst: str, avoid: set[str] | None = None) -> list[str]:
        """BFS preferring rooms not in *avoid* to reduce backtracking."""
        queue = deque([[src]])
        seen = {src}
        while queue:
            path = queue.popleft()
            if path[-1] == dst:
                return path
            neighbours = sorted(graph.get(path[-1], []), key=lambda r: (r in (avoid or set()), room_num(r)))
            for nxt in neighbours:
                if nxt in seen:
                    continue
                seen.add(nxt)
                queue.append(path + [nxt])
        raise ValueError(f"no topology route from {src} to {dst}")

    route = [start]
    current = start
    for waypoint in [*through, goal]:
        visited = set(route)
        leg = bfs(current, waypoint, avoid=visited)
        route.extend(leg[1:])
        current = waypoint
    return route


def required_room_reasons(start: str, goal: str, through: list[str], terminal: str, route: list[str]) -> dict[str, list[str]]:
    reasons: dict[str, set[str]] = {}
    for room, reason in [(start, "start"), (goal, "goal"), (terminal, "terminal")]:
        reasons.setdefault(room, set()).add(reason)
    for room in through:
        reasons.setdefault(room, set()).add("through")
    for room in route:
        reasons.setdefault(room, set()).add("intermediate_route" if room not in {start, goal, terminal, *through} else "resolved_route")
    return {room: sorted(values) for room, values in sorted(reasons.items(), key=lambda item: room_num(item[0]))}


def parse_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in {b"\n", b""}:
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    if tokens[0] != b"P5":
        raise ValueError(f"unsupported PGM magic in {path}")
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError("16-bit PGM is not supported")
    while data[idx:idx + 1].isspace():
        idx += 1
    return np.frombuffer(data[idx:idx + width * height], dtype=np.uint8).reshape((height, width)).copy()


def write_pgm(path: Path, grid_world: np.ndarray) -> None:
    image_grid = np.flipud(grid_world).astype(np.uint8)
    header = f"P5\n{image_grid.shape[1]} {image_grid.shape[0]}\n255\n".encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + image_grid.tobytes())


def parse_yaml_map(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def load_nav_map(map_yaml: Path) -> tuple[np.ndarray, float, tuple[float, float], dict[str, Any]]:
    meta = parse_yaml_map(map_yaml)
    grid = np.flipud(parse_pgm(map_yaml.parent / meta["image"]))
    origin = tuple(float(x.strip()) for x in meta["origin"].strip("[]").split(",")[:2])
    return grid, float(meta["resolution"]), origin, meta  # type: ignore[return-value]


def write_yaml_for_pgm(path: Path, image_name: str, source_meta: dict[str, Any]) -> None:
    lines = [
        f"image: {image_name}",
        f"resolution: {source_meta.get('resolution', '0.05')}",
        f"origin: {source_meta.get('origin', '[-50.0, -50.0, 0.0]')}",
        f"negate: {source_meta.get('negate', '0')}",
        f"occupied_thresh: {source_meta.get('occupied_thresh', '0.65')}",
        f"free_thresh: {source_meta.get('free_thresh', '0.196')}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def xy_to_rc(x: float, y: float, resolution: float, origin: tuple[float, float]) -> tuple[int, int]:
    return int(round((y - origin[1]) / resolution)), int(round((x - origin[0]) / resolution))


def rc_to_xy(row: int, col: int, resolution: float, origin: tuple[float, float]) -> tuple[float, float]:
    return origin[0] + col * resolution, origin[1] + row * resolution


def nearest_free_index(grid: np.ndarray, x: float, y: float, resolution: float, origin: tuple[float, float]) -> tuple[int, int]:
    start = xy_to_rc(x, y, resolution, origin)
    queue = deque([start])
    seen = {start}
    while queue:
        row, col = queue.popleft()
        if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and int(grid[row, col]) >= 250:
            return row, col
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nxt = (row + dr, col + dc)
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
    raise RuntimeError("no free cell found")


def astar(grid: np.ndarray, start_rc: tuple[int, int], goal_rc: tuple[int, int], wall_clearance_cells: int = 5) -> list[tuple[int, int]]:
    if start_rc == goal_rc:
        return [start_rc]
    # Precompute wall distance for clearance-aware cost
    free_mask = (grid >= 250).astype(np.uint8)
    dist_from_wall = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
    neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    heap: list[tuple[float, tuple[int, int]]] = [(0.0, start_rc)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start_rc: 0.0}
    while heap:
        _, current = heapq.heappop(heap)
        if current == goal_rc:
            out = [current]
            while out[-1] in came:
                out.append(came[out[-1]])
            return list(reversed(out))
        row, col = current
        for dr, dc, step in neighbors:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]) or int(grid[nr, nc]) < 250:
                continue
            # Add wall proximity penalty to keep path away from walls
            wall_dist = float(dist_from_wall[nr, nc])
            wall_penalty = max(0.0, (wall_clearance_cells - wall_dist) * 0.5) if wall_dist < wall_clearance_cells else 0.0
            new_cost = cost[current] + step + wall_penalty
            nxt = (nr, nc)
            if new_cost >= cost.get(nxt, 1e18):
                continue
            cost[nxt] = new_cost
            came[nxt] = current
            heapq.heappush(heap, (new_cost + math.hypot(goal_rc[0] - nr, goal_rc[1] - nc), nxt))
    raise RuntimeError("A* found no path")


def simplify_path(path: list[tuple[int, int]], resolution: float, spacing_m: float = 0.35) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    step = max(1, int(round(spacing_m / resolution)))
    return [path[0], *[path[i] for i in range(step, len(path) - 1, step)], path[-1]]


GATEWAY_REGISTRY_REL = Path("gateway/gateway_registry_v0_1.json")


def load_gateway_lookup(stage_output: Path, config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Load gateway data keyed by gateway_id.

    Primary source: unified gateway registry (gateway_registry_v0_1.json).
    Fallback: hypotheses + auto-selection files (legacy path).
    """
    registry_path = stage_output / GATEWAY_REGISTRY_REL
    selected_pairs = config.get("selected_gateway_pairs") or {}
    selected_ids = set(selected_pairs.values())

    out: dict[str, dict[str, Any]] = {}

    # --- Primary: unified registry ---
    if registry_path.exists():
        registry = read_json(registry_path)
        by_id = registry.get("gateways_by_id") or {}
        for gateway_id in selected_ids:
            entry = by_id.get(gateway_id)
            if entry and entry.get("representative_crossing_pose"):
                out[gateway_id] = entry
        if set(out.keys()) >= selected_ids:
            return out

    # --- Fallback: hypotheses file ---
    hyp_path = stage_output / config.get("gateway_hypotheses_path", "")
    if hyp_path.exists():
        hypotheses = read_json(hyp_path).get("hypotheses") or []
        for item in hypotheses:
            for gid in item.get("source_candidate_ids") or []:
                if gid in selected_ids and gid not in out:
                    out[gid] = item

    # --- Fallback: auto-selection file ---
    auto_path = stage_output / "stage1_process/gateway_extraction/assets/00824_step30b2_auto_gateway_selection_v0_1.json"
    if auto_path.exists():
        auto_data = read_json(auto_path)
        for pair_key, gateway_id in selected_pairs.items():
            if gateway_id in out:
                continue
            auto_entry = (auto_data.get("per_pair") or {}).get(pair_key, {})
            primary = auto_entry.get("selected_primary")
            if primary and primary.get("crossing_pose"):
                parts = pair_key.split("_")
                room_a_num = int(parts[0][1:])
                room_b_num = int(parts[1][1:])
                out[gateway_id] = {
                    "gateway_id": gateway_id,
                    "pair_key": pair_key,
                    "room_a": room_a_num,
                    "room_b": room_b_num,
                    "representative_crossing_pose": primary["crossing_pose"],
                    "representative_approach_from_room_a": primary.get("approach_from_room_a"),
                    "representative_approach_from_room_b": primary.get("approach_from_room_b"),
                    "pose_source": "auto_gateway_selection_fallback",
                }

    # --- Preflight check ---
    missing = selected_ids - set(out.keys())
    if missing:
        report = {gid: {"pair_key": pk for pk, gid2 in selected_pairs.items() if gid2 == gid} for gid in missing}
        raise RuntimeError(
            f"Gateway registry preflight failed: {len(missing)} gateway(s) missing from lookup.\n"
            f"Missing: {sorted(missing)}\n"
            f"Details: {json.dumps(report, indent=2)}\n"
            f"Registry path: {registry_path}\n"
            f"Ensure gateway_registry_v0_1.json contains all selected gateways."
        )

    return out


def load_rooms(stage_output: Path) -> dict[str, dict[str, Any]]:
    data = read_json(stage_output / "stage1_committed_public/topology_v0_1.json")
    return {room["id"]: room for room in data.get("rooms", [])}


def derive_interior_target(
    stage_output: Path,
    room_id: str,
    grid: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    gateway_points: list[dict[str, Any]],
    min_wall_distance_m: float = 0.35,
    preferred_gateway_distance_m: float = 0.80,
) -> dict[str, Any]:
    rooms = load_rooms(stage_output)
    center = rooms[room_id]["center"]
    mask = np.load(stage_output / "stage1_process/room_segmentation/assets/00824_step30a_global_room_mask_v0_1.npy")
    room_free = (mask == room_num(room_id)) & (grid >= 250)
    if not room_free.any():
        raise RuntimeError(f"{room_id} has no free cells in the selected Nav2 map")
    distance_to_nonfree = cv2.distanceTransform((grid >= 250).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    gateway_xy = [(float(g["x"]), float(g["y"])) for g in gateway_points]
    best: tuple[float, int, int, float, float] | None = None
    for row, col in np.argwhere(room_free):
        wx, wy = rc_to_xy(int(row), int(col), resolution, origin)
        dist_gate = min((math.hypot(wx - gx, wy - gy) for gx, gy in gateway_xy), default=999.0)
        wall_dist = float(distance_to_nonfree[row, col])
        score = min(dist_gate, preferred_gateway_distance_m) + 2.0 * min(wall_dist, min_wall_distance_m) - 0.03 * math.hypot(wx - float(center[0]), wy - float(center[1]))
        if best is None or score > best[0]:
            best = (score, int(row), int(col), dist_gate, wall_dist)
    if best is None:
        raise RuntimeError(f"could not select an interior target for {room_id}")
    _, row, col, dist_gate, wall_dist = best
    x, y = rc_to_xy(row, col, resolution, origin)
    return {
        "room_id": room_id,
        "x": round(x, 4),
        "y": round(y, 4),
        "yaw": 0.0,
        "distance_to_nearest_gateway_m": round(dist_gate, 6),
        "distance_to_occupied_or_unknown_m": round(wall_dist, 6),
        "meets_preferred_gateway_distance": dist_gate >= preferred_gateway_distance_m,
        "meets_wall_distance_threshold": wall_dist >= min_wall_distance_m,
        "derivation_method": "room_mask_free_cell_maximizing_gateway_clearance_and_wall_distance",
    }
