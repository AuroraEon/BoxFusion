#!/usr/bin/env python3
"""Shared helpers for parameterized Stage1 runtime scripts."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def resolve_path(path: str | Path | None, base: Path = REPO_ROOT) -> Path | None:
    if path is None or str(path) == "":
        return None
    p = Path(path)
    return p if p.is_absolute() else (base / p).resolve()


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
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def room_num(room: str) -> int:
    return int(str(room).split("_")[1])


def pair_key(a: str, b: str) -> str:
    return "__".join(sorted([a, b], key=room_num))


def parse_rooms(value: str | list[str] | None) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        raw = value
    else:
        raw = []
        for chunk in str(value).split(","):
            raw.extend(part for part in chunk.split() if part)
    return [item.strip() for item in raw if item and item.strip()]


def default_runtime_profile(stage_output_dir: Path, floor_id: str) -> Path:
    return stage_output_dir / "runtime" / "profiles" / f"{floor_id}_nav2" / "runtime_profile.json"


def load_runtime_profile(path: Path | None) -> dict[str, Any]:
    if path and path.exists():
        payload = read_json(path)
        for key, value in list(payload.items()):
            if isinstance(value, str) and (value.startswith("/") or value.startswith("stage_outputs/")):
                payload[key] = resolve_path(value).as_posix() if resolve_path(value) else value
        return payload
    return {}


def derive_paths(args: Any) -> dict[str, Path | str | None]:
    stage_output = resolve_path(args.stage_output_dir)
    if stage_output is None:
        raise ValueError("--stage-output-dir is required")
    floor_id = args.floor_id
    profile_path = resolve_path(getattr(args, "runtime_profile", None)) or default_runtime_profile(stage_output, floor_id)
    profile = load_runtime_profile(profile_path)
    map_yaml = resolve_path(getattr(args, "map_yaml", None) or profile.get("map_yaml")) or (
        stage_output / "maps" / floor_id / f"stage1_{floor_id}_stable_occupancy_map.yaml"
    )
    route_query = resolve_path(getattr(args, "route_query_json", None)) or None
    waypoints = resolve_path(getattr(args, "waypoints_json", None) or profile.get("waypoints")) or None
    if route_query is None and waypoints is not None:
        route_query = waypoints.parent / "route_query_result_v0_1.json"
    route_dir = route_query.parent if route_query else (
        stage_output / "routes" / "room_routes" / f"{floor_id}_{getattr(args, 'start_room', 'start')}_to_{getattr(args, 'goal_room', 'goal')}"
    )
    return {
        "stage_output": stage_output,
        "stage_a_output": resolve_path(getattr(args, "stage_a_output_dir", None)),
        "floor_id": floor_id,
        "scene_id": getattr(args, "scene_id", profile.get("scene_id")),
        "runtime_profile": profile_path,
        "map_yaml": map_yaml,
        "route_query_json": route_query or route_dir / "route_query_result_v0_1.json",
        "waypoints_json": waypoints or route_dir / "semantic_route_waypoints_v0_1.json",
        "topology_json": stage_output / "committed_public" / "topology_v0_1.json",
        "gateway_registry_json": stage_output / "process" / "floors" / floor_id / "gateway" / "assets" / "gateway_registry_v0_1.json",
        "room_mask_npy": stage_output / "process" / "floors" / floor_id / "room_segmentation" / "assets" / "room_mask_global_id_v0_1.npy",
        "layer_meta_json": stage_output / "process" / "floors" / floor_id / "room_segmentation" / "assets" / "layered_bev_metadata_v0_1.json",
        "free_space_png": stage_output / "process" / "floors" / floor_id / "room_segmentation" / "assets" / "free_space.png",
        "gateway_wall_png": stage_output / "process" / "floors" / floor_id / "room_segmentation" / "assets" / "gateway_wall_preclose.png",
        "nav2_params": resolve_path(profile.get("nav2_params")) if profile.get("nav2_params") else stage_output / "runtime" / "nav2" / f"{str(getattr(args, 'scene_id', profile.get('scene_id', 'scene'))).split('-')[0]}_{floor_id.replace('_', '')}_nav2_params.yaml",
        "nav2_launch": resolve_path(profile.get("nav2_launch")) if profile.get("nav2_launch") else None,
        "gazebo_world": resolve_path(profile.get("gazebo_world")) if profile.get("gazebo_world") else None,
        "rviz_config": resolve_path(profile.get("rviz_config")) if profile.get("rviz_config") else None,
        "overlay_topic": getattr(args, "overlay_topic", None) or profile.get("overlay_topic"),
        "profile": profile,
    }


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


def parse_yaml_map(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip().strip('"')
    return meta


def load_nav_map(map_yaml: Path) -> tuple[np.ndarray, float, tuple[float, float], dict[str, Any]]:
    meta = parse_yaml_map(map_yaml)
    image = Path(meta["image"])
    image_path = image if image.is_absolute() else map_yaml.parent / image
    grid = np.flipud(parse_pgm(image_path))
    origin = tuple(float(x.strip()) for x in meta.get("origin", "[-50.0, -50.0, 0.0]").strip("[]").split(",")[:2])
    return grid, float(meta.get("resolution", 0.05)), origin, meta  # type: ignore[return-value]


def map_value(grid: np.ndarray, x: float, y: float, resolution: float, origin: tuple[float, float]) -> int | None:
    col = int(round((x - origin[0]) / resolution))
    row = int(round((y - origin[1]) / resolution))
    if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]:
        return int(grid[row, col])
    return None


def room_lookup(topology_json: Path, floor_id: str) -> dict[str, dict[str, Any]]:
    topology = read_json(topology_json)
    return {
        room["id"]: room
        for room in topology.get("rooms", [])
        if room.get("floor_id") == floor_id
    }


def route_id(floor_id: str, start_room: str, goal_room: str) -> str:
    return f"{floor_id}_{start_room.replace('_', '')}_to_{goal_room.replace('_', '')}"


def yaw_between(a: dict[str, Any], b: dict[str, Any]) -> float | None:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    if math.hypot(dx, dy) <= 1e-9:
        return None
    return math.atan2(dy, dx)
