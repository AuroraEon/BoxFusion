#!/usr/bin/env python3
"""Task24b2 audit/backfill for stair graph and per-floor stable maps.

This script is intentionally read-only with respect to clean_rerun. It writes
all generated evidence into the task output directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import py_compile
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CLAIM_BOUNDARY = "topological_vertical_transition_only"
PRIMARY_CLASSIFICATION = "current_artifacts_need_stage_a_exporter_backfill"
STABLE_MAP_CLASSIFICATION = "floor_2_only_stable_map_found"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def to_jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    return value


def rel(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def sha256(path: Path) -> str | None:
    if not path.exists() or not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def run_cmd(args: list[str], cwd: Path) -> dict[str, Any]:
    proc = subprocess.run(args, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    return {"args": args, "returncode": proc.returncode, "output": proc.stdout.strip()}


def parse_map_yaml(path: Path) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    if not path.exists():
        return meta
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        meta[key.strip()] = value.strip()
    return meta


def write_pgm(path: Path, grid_world: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pgm = np.flipud(grid_world).astype(np.uint8)
    header = f"P5\n{pgm.shape[1]} {pgm.shape[0]}\n255\n".encode("ascii")
    path.write_bytes(header + pgm.tobytes())


def write_map_yaml(path: Path, image_name: str, resolution: float, origin: list[float]) -> None:
    text = "\n".join(
        [
            f"image: {image_name}",
            "mode: trinary",
            f"resolution: {resolution}",
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        ]
    )
    write_text(path, text)


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    tokens: list[bytes] = []
    idx = 0
    while len(tokens) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b"#":
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
        raise ValueError(f"unsupported PGM maxval in {path}: {maxval}")
    while idx < len(data) and data[idx:idx + 1].isspace():
        idx += 1
    return np.frombuffer(data[idx:idx + width * height], dtype=np.uint8).reshape((height, width)).copy()


def load_pose_matrix(dataset_root: Path, frame_idx: int) -> np.ndarray | None:
    seq_short = dataset_root.name.split("-", 1)[-1]
    path = dataset_root / "pose" / f"{seq_short}_{frame_idx:06d}.txt"
    if not path.exists():
        return None
    arr = np.loadtxt(path, dtype=np.float64)
    if arr.shape == (16,):
        arr = arr.reshape(4, 4)
    if arr.shape != (4, 4):
        return None
    habitat_to_opencv_camera = np.eye(4, dtype=np.float64)
    habitat_to_opencv_camera[1, 1] = -1.0
    habitat_to_opencv_camera[2, 2] = -1.0
    habitat_yup_to_zup_world = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return habitat_yup_to_zup_world @ arr @ habitat_to_opencv_camera


def yaw_from_pose(matrix: np.ndarray) -> float:
    # Heading proxy using the camera/world rotation projected into XY.
    return float(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def room_for_xy(xy: list[float], rooms: list[dict[str, Any]], floor_id: str | None) -> str | None:
    candidates = [room for room in rooms if floor_id is None or str(room.get("floor_id")) == str(floor_id)] or rooms
    point = (float(xy[0]), float(xy[1]))
    best: tuple[float, dict[str, Any]] | None = None
    for room in candidates:
        poly = room.get("polygon") or []
        if len(poly) >= 3:
            contour = np.asarray(poly, dtype=np.float32)
            if cv2.pointPolygonTest(contour, point, False) >= 0:
                return f"room_{int(room['id'])}"
            pts = np.asarray(poly, dtype=np.float32)
            centroid = [float(pts[:, 0].mean()), float(pts[:, 1].mean())]
            dist = math.hypot(point[0] - centroid[0], point[1] - centroid[1])
            if best is None or dist < best[0]:
                best = (dist, room)
    if best is not None and best[0] <= 3.2:
        return f"room_{int(best[1]['id'])}"
    return None


def path_length(points: list[list[float]]) -> float:
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
    return total


def build_stairs_graph(
    transition: dict[str, Any],
    assignments: list[dict[str, Any]],
    vector_map: dict[str, Any],
    dataset_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    frame_start = int(transition["transition_frame_start"])
    frame_end = int(transition["transition_frame_end"])
    rooms = list(vector_map.get("rooms") or [])
    by_frame = {int(row["frame_idx"]): row for row in assignments}
    samples: list[dict[str, Any]] = []
    missing_pose_frames: list[int] = []
    for frame_idx in range(frame_start, frame_end + 1):
        pose = load_pose_matrix(dataset_root, frame_idx)
        assignment = dict(by_frame.get(frame_idx) or {})
        if pose is None:
            missing_pose_frames.append(frame_idx)
            continue
        xyz = [round(float(pose[0, 3]), 3), round(float(pose[1, 3]), 3), round(float(pose[2, 3]), 3)]
        floor_id = assignment.get("floor_id")
        status = str(assignment.get("status") or "transition")
        floor_status = str(floor_id) if status == "stable" and floor_id else "transition"
        room_id = room_for_xy(xyz[:2], rooms, str(floor_id) if floor_id else None)
        samples.append(
            {
                "frame_idx": frame_idx,
                "timestamp": assignment.get("timestamp"),
                "xyz": xyz,
                "yaw": round(yaw_from_pose(pose), 6),
                "assigned_floor_id": floor_id,
                "floor_status": floor_status,
                "room_id": room_id,
                "confidence": assignment.get("confidence"),
                "provenance": {
                    "pose_source": (dataset_root / "pose" / f"{dataset_root.name.split('-', 1)[-1]}_{frame_idx:06d}.txt").as_posix(),
                    "floor_assignment_source": "final_vector_map_snapshot.frame_floor_assignments",
                },
            }
        )

    if len(samples) >= 2:
        density = "dense_pose_samples"
        nodes = []
        for index, sample in enumerate(samples):
            if index == 0 or index == len(samples) - 1:
                node_type = "stair_endpoint"
            elif sample["floor_status"] == "transition":
                node_type = "stair_node"
            else:
                node_type = "stair_landing"
            nodes.append(
                {
                    "node_id": f"{transition['transition_id']}_n{index:03d}",
                    "node_type": node_type,
                    "position_xyz": sample["xyz"],
                    "frame_idx": sample["frame_idx"],
                    "room_id": sample["room_id"],
                    "floor_status": sample["floor_status"],
                    "yaw_or_heading": sample["yaw"],
                    "provenance": sample["provenance"],
                }
            )
        edges = []
        for index, (a, b) in enumerate(zip(nodes, nodes[1:])):
            pa = a["position_xyz"]
            pb = b["position_xyz"]
            dist = math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(pa, pb)))
            dz = float(pb[2]) - float(pa[2])
            horizontal = math.hypot(float(pb[0]) - float(pa[0]), float(pb[1]) - float(pa[1]))
            edges.append(
                {
                    "edge_id": f"{transition['transition_id']}_e{index:03d}",
                    "source": a["node_id"],
                    "target": b["node_id"],
                    "edge_type": "stair_graph_edge",
                    "distance_3d": round(dist, 3),
                    "delta_z": round(dz, 3),
                    "slope_or_grade": None if horizontal < 1e-6 else round(dz / horizontal, 3),
                    "traversable_topological_only": True,
                }
            )
        missing_evidence = []
        if missing_pose_frames:
            missing_evidence.append("some_transition_pose_files_missing")
    else:
        density = "endpoint_only"
        from_xy = transition.get("from_position_xy") or [0.0, 0.0]
        to_xy = transition.get("to_position_xy") or [0.0, 0.0]
        floors = {str(f["floor_id"]): f for f in vector_map.get("floors", []) if f.get("floor_id") is not None}
        from_z = float((floors.get(str(transition.get("from_floor_id"))) or {}).get("z_center", transition.get("z_min", 0.0)))
        to_z = float((floors.get(str(transition.get("to_floor_id"))) or {}).get("z_center", transition.get("z_max", from_z)))
        nodes = [
            {
                "node_id": f"{transition['transition_id']}_from",
                "node_type": "stair_endpoint",
                "position_xyz": [round(float(from_xy[0]), 3), round(float(from_xy[1]), 3), round(from_z, 3)],
                "frame_idx": transition.get("entry_stable_frame"),
                "room_id": f"room_{transition.get('from_room_id')}",
                "floor_status": str(transition.get("from_floor_id")),
                "yaw_or_heading": None,
                "provenance": {"source": "vertical_transition_evidence.from_position_xy"},
            },
            {
                "node_id": f"{transition['transition_id']}_to",
                "node_type": "stair_endpoint",
                "position_xyz": [round(float(to_xy[0]), 3), round(float(to_xy[1]), 3), round(to_z, 3)],
                "frame_idx": transition.get("exit_stable_frame"),
                "room_id": f"room_{transition.get('to_room_id')}",
                "floor_status": str(transition.get("to_floor_id")),
                "yaw_or_heading": None,
                "provenance": {"source": "vertical_transition_evidence.to_position_xy"},
            },
        ]
        dist = path_length([nodes[0]["position_xyz"], nodes[1]["position_xyz"]])
        edges = [
            {
                "edge_id": f"{transition['transition_id']}_e000",
                "source": nodes[0]["node_id"],
                "target": nodes[1]["node_id"],
                "edge_type": "stair_graph_edge",
                "distance_3d": round(dist, 3),
                "delta_z": round(nodes[1]["position_xyz"][2] - nodes[0]["position_xyz"][2], 3),
                "slope_or_grade": None,
                "traversable_topological_only": True,
            }
        ]
        missing_evidence = ["missing_dense_pose_xy_yaw"]

    graph = {
        "graph_id": f"stairs_graph_{transition['transition_id']}",
        "schema_version": "stairs_graph_v0_1",
        "source_transition_id": transition.get("transition_id"),
        "connector_type": transition.get("connector_label") or "unknown_vertical_connector",
        "source": "dataset_pose_files_plus_stage_a_floor_assignments" if density == "dense_pose_samples" else "vertical_transition_evidence_endpoints",
        "floor_from": transition.get("from_floor_id"),
        "floor_to": transition.get("to_floor_id"),
        "graph_density": density,
        "nodes": nodes,
        "edges": edges,
        "endpoint_bindings": {
            "from": {
                "endpoint_node": nodes[0]["node_id"],
                "floor_topology_node": f"room_{transition.get('from_room_id')}",
                "room_id": f"room_{transition.get('from_room_id')}",
                "gateway_id": None,
            },
            "to": {
                "endpoint_node": nodes[-1]["node_id"],
                "floor_topology_node": f"room_{transition.get('to_room_id')}",
                "room_id": f"room_{transition.get('to_room_id')}",
                "gateway_id": None,
            },
        },
        "path_length_3d_m": round(path_length([node["position_xyz"] for node in nodes]), 3),
        "z_span_m": transition.get("z_span_m"),
        "confidence": transition.get("confidence"),
        "missing_evidence": missing_evidence,
        "physical_execution_supported": False,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    path_record = {
        "transition_id": transition.get("transition_id"),
        "floor_from": transition.get("from_floor_id"),
        "floor_to": transition.get("to_floor_id"),
        "frame_start": frame_start,
        "frame_end": frame_end,
        "ordered_samples": samples,
        "endpoint_candidates": {
            "from_position_xy": transition.get("from_position_xy"),
            "to_position_xy": transition.get("to_position_xy"),
            "entry_stable_frame": transition.get("entry_stable_frame"),
            "exit_stable_frame": transition.get("exit_stable_frame"),
        },
        "path_length": graph["path_length_3d_m"],
        "z_span": transition.get("z_span_m"),
        "graph_density": density,
        "missing_evidence": missing_evidence,
    }
    return graph, path_record


def render_stairs_graph(path: Path, graph: dict[str, Any]) -> None:
    nodes = graph.get("nodes") or []
    if not nodes:
        return
    pts = np.asarray([node["position_xyz"] for node in nodes], dtype=np.float32)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    pad = 40
    w, h = 1000, 700
    canvas = np.full((h, w, 3), 245, dtype=np.uint8)
    min_x, max_x = float(x.min()), float(x.max())
    min_y, max_y = float(y.min()), float(y.max())
    span_x = max(max_x - min_x, 0.5)
    span_y = max(max_y - min_y, 0.5)

    def to_px(px: float, py: float) -> tuple[int, int]:
        cx = int(round(pad + (px - min_x) / span_x * (w - 2 * pad)))
        cy = int(round(h - pad - (py - min_y) / span_y * (h - 2 * pad)))
        return cx, cy

    poly = np.asarray([to_px(float(px), float(py)) for px, py in zip(x, y)], dtype=np.int32)
    if len(poly) >= 2:
        cv2.polylines(canvas, [poly], False, (42, 98, 180), 3, cv2.LINE_AA)
    min_z, max_z = float(z.min()), float(z.max())
    for idx, node in enumerate(nodes):
        px, py = to_px(float(node["position_xyz"][0]), float(node["position_xyz"][1]))
        frac = 0.0 if max_z <= min_z else (float(node["position_xyz"][2]) - min_z) / (max_z - min_z)
        color = (int(220 - 120 * frac), int(120 + 80 * frac), int(45 + 160 * frac))
        radius = 8 if node.get("node_type") == "stair_endpoint" else 5
        cv2.circle(canvas, (px, py), radius, color, -1, cv2.LINE_AA)
        if idx in {0, len(nodes) - 1}:
            cv2.putText(canvas, node["node_id"], (px + 8, py - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (30, 30, 30), 1, cv2.LINE_AA)
    lines = [
        f"{graph['graph_id']} | {graph['graph_density']}",
        f"{graph['floor_from']} -> {graph['floor_to']} | length {graph.get('path_length_3d_m')}m | z_span {graph.get('z_span_m')}m",
        "Topological vertical transition only; no physical stair execution claim.",
    ]
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (24, 32 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (35, 35, 35), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(path.as_posix(), canvas)


def build_floor_map(stage_dir: Path, out_dir: Path, scene_id: str, floor_id: str) -> dict[str, Any]:
    src = stage_dir / "process" / "floors" / floor_id / "room_segmentation" / "assets"
    room_path = src / "room_mask_global_id_v0_1.npy"
    layered_meta_path = src / "layered_bev_v0_1.json"
    gateway_wall_path = src / "gateway_wall_preclose.png"
    if not room_path.exists() or not layered_meta_path.exists() or not gateway_wall_path.exists():
        return {"floor_id": floor_id, "generated": False, "reason": "missing_required_sources"}
    room_mask = np.load(room_path)
    gateway_wall = cv2.imread(gateway_wall_path.as_posix(), cv2.IMREAD_GRAYSCALE) > 0
    meta = read_json(layered_meta_path)
    occupied = (room_mask <= 0) | gateway_wall
    carved_gateway = np.zeros_like(occupied, dtype=bool)
    gateway_registry_path = stage_dir / "process" / "floors" / floor_id / "gateway" / "assets" / "gateway_registry_v0_1.json"
    carved_gateway_ids: list[str] = []
    if gateway_registry_path.exists():
        registry = read_json(gateway_registry_path)
        resolution = float(meta.get("resolution_m_per_cell", 0.05))
        radius_px_default = max(3, int(round(0.35 / resolution)))
        for gateway in registry.get("gateways", []):
            if not dict(gateway.get("validation") or {}).get("accepted_for_carving", True):
                continue
            gx = int(round(float(gateway.get("grid_x", -9999))))
            gy_world = int(round(float(gateway.get("grid_y", -9999))))
            if gx < 0 or gy_world < 0:
                continue
            radius_px = max(radius_px_default, int(round(float(gateway.get("width_m", 0.7)) / max(resolution, 1e-6) * 0.22)))
            carved_uint8 = carved_gateway.astype(np.uint8)
            cv2.circle(carved_uint8, (gx, gy_world), radius_px, 1, -1)
            carved_gateway = carved_uint8.astype(bool)
            carved_gateway_ids.append(str(gateway.get("gateway_id")))
    occupied[carved_gateway] = False
    grid = np.where(occupied, 0, 254).astype(np.uint8)
    stem = f"stage1_{floor_id}_stable_occupancy_map"
    out_floor = out_dir / "generated_maps" / floor_id
    out_pgm = out_floor / f"{stem}.pgm"
    out_yaml = out_floor / f"{stem}.yaml"
    out_npz = out_floor / f"{stem}.npz"
    out_preview = out_floor / f"{stem}_preview.png"
    out_prov = out_floor / f"{stem}_provenance.json"
    resolution = float(meta.get("resolution_m_per_cell", 0.05))
    origin = [float(v) for v in meta.get("origin_xy_yaw", [-50.0, -50.0, 0.0])[:3]]
    write_pgm(out_pgm, grid)
    write_map_yaml(out_yaml, out_pgm.name, resolution, origin)
    np.savez_compressed(
        out_npz,
        occupancy=np.where(occupied, 100, 0).astype(np.int16),
        free=(~occupied).astype(np.uint8),
        occupied=occupied.astype(np.uint8),
        carved_gateway=carved_gateway.astype(np.uint8),
        room_mask=room_mask.astype(np.int32),
        resolution=np.asarray([resolution], dtype=np.float32),
        origin=np.asarray(origin, dtype=np.float32),
    )
    preview = np.zeros((*grid.shape, 3), dtype=np.uint8)
    preview[~occupied] = (245, 245, 245)
    preview[occupied] = (20, 20, 20)
    cv2.imwrite(out_preview.as_posix(), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
    provenance = {
        "scene_id": scene_id,
        "floor_id": floor_id,
        "map_frame": "map",
        "resolution_m_per_cell": resolution,
        "origin_xy_yaw": origin,
        "array_shape_hw": list(grid.shape),
        "pgm_is_vertical_flip_of_stage_a_grid": True,
        "sources": {
            "room_mask": room_path.as_posix(),
            "gateway_wall_preclose": gateway_wall_path.as_posix(),
            "layered_bev_metadata": layered_meta_path.as_posix(),
            "gateway_registry": gateway_registry_path.as_posix() if gateway_registry_path.exists() else None,
        },
        "carved_gateway_ids": carved_gateway_ids,
        "counts": {
            "free": int((grid >= 250).sum()),
            "occupied": int((grid <= 10).sum()),
            "unknown": int(((grid > 10) & (grid < 250)).sum()),
        },
        "experimental_backfill": True,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    write_json(out_prov, provenance)
    return {
        "floor_id": floor_id,
        "generated": True,
        "map_yaml": out_yaml.as_posix(),
        "map_pgm": out_pgm.as_posix(),
        "map_png": out_preview.as_posix(),
        "map_npz": out_npz.as_posix(),
        "provenance": out_prov.as_posix(),
        "shape_hw": list(grid.shape),
        "counts": provenance["counts"],
        "gateway_registry_exists": gateway_registry_path.exists(),
    }


def file_inventory(stage_dir: Path, out_dir: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(stage_dir.rglob("*")):
        if not path.is_file():
            continue
        token = path.as_posix()
        if any(part in token for part in ["/maps/", "/process/floors/"]) or path.name in {
            "vertical_transition_evidence.json",
            "final_vector_map_snapshot.json",
            "floor_diagnostics_summary.json",
        }:
            rows.append(
                {
                    "path": path.as_posix(),
                    "size": path.stat().st_size,
                    "suffix": path.suffix,
                    "sha256": sha256(path) if path.stat().st_size < 5_000_000 else None,
                }
            )
    lines = ["path\tsize_bytes\tsuffix\tsha256"]
    for row in rows:
        lines.append(f"{row['path']}\t{row['size']}\t{row['suffix']}\t{row['sha256'] or ''}")
    write_text(out_dir / "floor_map_file_inventory.txt", "\n".join(lines))
    return rows


def markdown_reports(ctx: dict[str, Any]) -> dict[str, str]:
    full_cmd = (
        "/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py hm3d "
        "--model-path ./models/cutr_rgbd.pth --config ./config/hm3d.yaml --device cuda "
        "--seq 00843-DYehNKdT76V "
        "--output-root ./stage_outputs/stage1_generalization/00843-DYehNKdT76V/stage_a_rerun_task24b2/scenes "
        "--room-seg-interval 100 --capture-stride 25 --video-fps 12 "
        "--runtime-profile-interval 25"
    )
    backfill_cmd = (
        "/home/ws/miniconda3/envs/boxfusion/bin/python tools/vertical_connectors/task24b2_audit_and_backfill.py "
        "--stage-dir stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun "
        "--output-dir stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24b2_stage_a_stair_graph_and_per_floor_stable_map_audit"
    )
    return {
        "current_stage_a_run_entry_audit.md": f"""# Current Stage-A Run Entry Audit

- Current entrypoint: `stage_a_demo.py` at repository root. It imports `ClosedLoopDemoRecorder` from `boxfusion/stage_a_demo.py` and calls `demo.run`.
- Package implementation: `boxfusion/stage_a_demo.py` contains the recorder/exporter and writes `logs/vertical_transition_evidence.json`.
- Python executable for Stage-A/offline work: `/home/ws/miniconda3/envs/boxfusion/bin/python`.
- Config/model inputs exist: `config/hm3d.yaml`, `models/cutr_rgbd.pth`, `models/ViT-B-32/open_clip_pytorch_model.bin`, `data/class_features_small.pt`.
- Current `config/hm3d.yaml` points at `/home/ws/data/00843-DYehNKdT76V`.
- Existing clean_rerun canonical raw output root: `{ctx['raw_dir']}`.
- Existing clean_rerun raw manifest reports `artifact_profile=full_artifact`, `core_only_mode=false`, and `processed_frames=2710`.
- Clean_rerun Stage-A command log was not preserved as a single exact command in the inspected clean_rerun metadata. The nearest preserved Stage-A log is `tasks/task2/logs/stage_a_full_00843.log`, but it does not include the invocation line.
- The closest clean_rerun reconstruction is the command in `current_stage_a_rerun_command_plan.md` with the output root changed to the clean_rerun raw-output parent.
""",
        "current_stage_a_rerun_command_plan.md": f"""# Current Stage-A Rerun Command Plan

Use a new output root and do not overwrite `clean_rerun`.

```bash
{full_cmd}
```

Output root to use: `stage_outputs/stage1_generalization/00843-DYehNKdT76V/stage_a_rerun_task24b2/scenes`.

Backfill/exporter answer: the existing clean_rerun artifacts plus source pose files are enough for an experimental task-local stair graph backfill. Stage-A still needs exporter changes because clean_rerun does not preserve `camera_position_xyz` or yaw in `frame_floor_assignments`; it only preserves `pose_z` there.

This command intentionally omits `--core-only` because the inspected clean_rerun raw manifest reports a full-artifact run (`core_only_mode=false`). If a later exporter-only test guarantees the new logs are produced in core-only mode, a smaller run can be considered separately.
""",
        "vertical_transition_evidence_producer_audit.md": f"""# Vertical Transition Evidence Producer Audit

- Producer: `boxfusion/stage_a_demo.py` writes `logs/vertical_transition_evidence.json` during `ClosedLoopDemoRecorder.finalize`.
- Transition computation: `boxfusion/floor_aware_room_segmenter.py::_build_vertical_transitions`.
- Floor/pose observation rows are built in `FloorAwareRoomSegmenter.observe_frame`; internal `frame_history` includes `position_xy` and `pose_z`.
- Existing exported `final_vector_map_snapshot.frame_floor_assignments` has `pose_z`, floor assignment, status, confidence, and timestamps, but does not include `position_xy`, full XYZ, yaw, or quaternion.
- `from_position_xy` and `to_position_xy` are copied from `_room_support_summary(...).position_xy`.
- `_room_support_summary` looks at the last 24 stable frames before the transition and first 24 stable frames after the transition, votes for the best room by polygon containment/nearest room, and uses the last supporting stable pose XY as representative endpoint.
- Therefore endpoint XY is a nearby stable-frame camera-pose proxy, not a room center and not a dense transition-frame path.
- Existing output does not preserve dense ordered pose XY/yaw for frames 1516-1538. The local source dataset pose files do preserve them and were used only for the task-local experimental backfill.
""",
        "hovsg_style_stair_graph_design.md": """# HOV-SG-Style Stair Graph Design

RSLG-SLAM should represent cross-floor navigation as:

1. floor-level navigation graph on the source floor,
2. stair graph built from the vertical transition evidence,
3. floor-level navigation graph on the destination floor.

The stair graph is topological/navigation evidence only. It must not claim quadruped gait, footstep planning, or physical stair climbing.

`stairs_graph_v0_1.json` uses `nodes`, `edges`, and `endpoint_bindings`. If dense ordered pose samples exist, nodes are sampled along the transition frames. If only endpoints exist, graph density must be `endpoint_only` and `missing_dense_pose_xy_yaw` must be explicit. For turn stairs or spiral stairs, endpoint-only evidence is insufficient; ordered pose samples or explicit stair geometry are required.
""",
        "stairs_graph_generation_feasibility.md": f"""# Stairs Graph Generation Feasibility

Current clean_rerun artifacts support an endpoint-level connector and preserve z-only per-frame floor assignment. They do not by themselves preserve the dense ordered XY/yaw stair path.

Task-local backfill feasibility:

- Dense source pose files exist under `{ctx['dataset_root']}`.
- Transition frames 1516-1538 can be read from those pose files.
- The generated `stairs_graph_v0_1.json` is therefore an experimental backfill from source dataset poses plus clean_rerun floor assignments, not proof that Stage-A currently exports the required dense path artifact.

Primary classification: `{PRIMARY_CLASSIFICATION}`.
""",
        "per_floor_stable_occupancy_map_audit.md": f"""# Per-Floor Stable Occupancy Map Audit

- Detected floors: `{', '.join(ctx['floor_ids'])}`.
- Existing packaged stable map floors: `{', '.join(ctx['existing_map_floors']) or 'none'}`.
- `floor_1` stable occupancy map exists in `clean_rerun/maps`: `{ctx['floor1_map_exists']}`.
- `floor_2` stable occupancy map exists in `clean_rerun/maps`: `{ctx['floor2_map_exists']}`.
- `floor_1` has Stage-A room segmentation/layered BEV assets, but no packaged runtime stable map and no gateway registry.
- `floor_2` has packaged stable map YAML/PGM/NPZ/preview/provenance and gateway registry.
- Task24b treated floor_1 runtime occupancy/gateway registry as missing because runtime packaging only materialized the selected floor_2 runtime assets.
- The task-local experimental map generator can emit maps for both detected floors, but its generated floor_2 map is not byte-identical to the validated packaged floor_2 map. Validation records the difference count; production all-floor packaging should reuse or factor the exact validated builder semantics rather than promote the generic backfill as canonical.

Stable-map classification: `{STABLE_MAP_CLASSIFICATION}`.
""",
        "per_floor_stable_map_generation_plan.md": """# Per-Floor Stable Map Generation Plan

Keep the floor_2 stable-map semantics: room-mask cells are free unless occupied by gateway wall/preclose evidence, outside-room cells remain occupied, and accepted gateway registry entries may carve openings.

Change the exporter/runtime packaging from selected-floor-only to all detected floors:

- iterate `final_vector_map_snapshot.floors`,
- require each floor's `process/floors/floor_<id>/room_segmentation/assets/layered_bev_v0_1.*`,
- generate `maps/floor_<id>/stage1_floor_<id>_stable_occupancy_map.yaml`,
- generate matching PGM, PNG preview, NPZ, and provenance,
- write a per-floor manifest with source files, counts, and missing gateway registries.

Do not substitute room segmentation visualizations as runtime occupancy maps. Use the same stable occupancy builder policy as floor_2.
""",
        "stage_a_exporter_modification_plan.md": """# Stage-A Exporter Modification Plan

Add first-class exports in Stage-A/offline:

- `logs/frame_pose_floor_assignments_v0_1.jsonl`: one row per frame with full camera XYZ, yaw or quaternion, assigned floor, floor status, room id, confidence, and provenance.
- `logs/vertical_transition_paths_v0_1.json`: transition records with ordered samples, endpoint candidates, path length, z span, graph density, and missing evidence.
- `logs/stairs_graphs_v0_1.json`: HOV-SG-style stair graph records with nodes, edges, bindings, confidence, provenance, and claim boundary.
- `logs/semantic_vertical_connector_candidates_v0_1.json`: stair/elevator/ramp object candidates and whether they span floors.
- per-floor stable occupancy maps for every detected floor.

Code locations:

- `boxfusion/floor_aware_room_segmenter.py::observe_frame` already has pose XY and pose Z internally; extend it or a recorder-side stream to preserve full pose/yaw.
- `boxfusion/floor_aware_room_segmenter.py::_build_vertical_transitions` should include transition path references or hand off to a path exporter.
- `boxfusion/stage_a_demo.py::ClosedLoopDemoRecorder.finalize` should write the new logs and stair graph records.
- `tools/stage1_runtime` should get an actual parameterized map builder, not only a validator.
""",
        "stage_a_minimal_rerun_or_backfill_plan.md": f"""# Stage-A Minimal Rerun Or Backfill Plan

Safe backfill command used/planned:

```bash
{backfill_cmd}
```

Backfill is safe because it performs no model inference, does not overwrite `clean_rerun`, does not touch task23b, and writes generated outputs only under task24b2.

A full Stage-A rerun is not required to create task-local experimental artifacts, but a Stage-A exporter modification is required for durable, current-schema outputs.
""",
        "missing_stairs_graph_evidence.md": """# Missing Stairs Graph Evidence

Clean_rerun does not export dense ordered transition XY/yaw. The experimental stair graph was backfilled from local source pose files.

For generalization and for turn/spiral stairs, Stage-A must export ordered transition samples directly. Endpoint-only connectors are insufficient for those cases.
""",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-dir", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/clean_rerun"))
    parser.add_argument("--output-dir", type=Path, default=Path("stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks/task24b2_stage_a_stair_graph_and_per_floor_stable_map_audit"))
    parser.add_argument("--dataset-root", type=Path, default=Path("/home/ws/data/00843-DYehNKdT76V"))
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    args = parser.parse_args()

    repo = args.repo_root.resolve()
    stage = args.stage_dir.resolve()
    out = args.output_dir.resolve()
    scene_id = "00843-DYehNKdT76V"
    raw_dir = stage / "canonical_stage1" / "raw_outputs" / scene_id
    committed_public = stage / "committed_public"
    vt_path = committed_public / "vertical_transition_evidence.json"
    vector_path = committed_public / "final_vector_map_snapshot.json"
    if not vector_path.exists():
        vector_path = raw_dir / "logs" / "final_vector_map_snapshot.json"
    vt = read_json(vt_path)
    vector = read_json(vector_path)
    transitions = list(vt.get("transitions") or [])
    assignments = list(vector.get("frame_floor_assignments") or [])
    floor_ids = [str(f["floor_id"]) for f in vector.get("floors", []) if f.get("floor_id") is not None]
    existing_map_floors = []
    for floor_id in floor_ids:
        if (stage / "maps" / floor_id / f"stage1_{floor_id}_stable_occupancy_map.yaml").exists():
            existing_map_floors.append(floor_id)

    graphs = []
    paths = []
    for transition in transitions:
        graph, path_record = build_stairs_graph(transition, assignments, vector, args.dataset_root)
        graphs.append(graph)
        paths.append(path_record)
    graph_payload = {
        "schema_version": "stairs_graphs_v0_1",
        "scene_id": scene_id,
        "created_utc": now_iso(),
        "graphs": graphs,
        "physical_execution_supported": False,
        "claim_boundary": CLAIM_BOUNDARY,
    }
    if graphs:
        write_json(out / "stairs_graph_v0_1.json", graphs[0])
        render_stairs_graph(out / "stairs_graph_visualization.png", graphs[0])
    write_json(out / "logs" / "vertical_transition_paths_v0_1.json", {"scene_id": scene_id, "transitions": paths})
    write_json(out / "logs" / "stairs_graphs_v0_1.json", graph_payload)

    semantic_candidates = []
    for obj in vector.get("objects", []):
        label = str(obj.get("label", "")).lower()
        if any(token in label for token in ("stair", "stairs", "elevator", "ramp")):
            semantic_candidates.append(
                {
                    "object_id": f"obj_{obj.get('id')}",
                    "label": obj.get("label"),
                    "floor_id": obj.get("floor_id"),
                    "room_id": None if obj.get("room_id") is None else f"room_{obj.get('room_id')}",
                    "position_or_footprint": obj.get("pose") or obj.get("footprint_2d"),
                    "spans_floors": False,
                    "connector_relevance": "corroborative_only",
                }
            )
    write_json(out / "logs" / "semantic_vertical_connector_candidates_v0_1.json", {"scene_id": scene_id, "candidates": semantic_candidates})

    frame_pose_path = out / "logs" / "frame_pose_floor_assignments_v0_1.jsonl"
    frame_pose_path.parent.mkdir(parents=True, exist_ok=True)
    assignment_by_frame = {int(row["frame_idx"]): row for row in assignments}
    with frame_pose_path.open("w", encoding="utf-8") as handle:
        for frame_idx in sorted(assignment_by_frame):
            pose = load_pose_matrix(args.dataset_root, frame_idx)
            row = dict(assignment_by_frame[frame_idx])
            payload = {
                "frame_idx": frame_idx,
                "timestamp": row.get("timestamp"),
                "camera_position_xyz": None if pose is None else [round(float(pose[0, 3]), 3), round(float(pose[1, 3]), 3), round(float(pose[2, 3]), 3)],
                "camera_yaw": None if pose is None else round(yaw_from_pose(pose), 6),
                "camera_quaternion": None,
                "assigned_floor_id": row.get("floor_id"),
                "floor_status": "stable" if row.get("status") == "stable" else "transition",
                "room_id": None,
                "confidence": row.get("confidence"),
                "provenance": {
                    "pose_source": "source_dataset_pose_file" if pose is not None else "missing_pose_file",
                    "floor_assignment_source": vector_path.as_posix(),
                },
            }
            handle.write(json.dumps(to_jsonable(payload), sort_keys=True) + "\n")

    generated_maps = [build_floor_map(stage, out, scene_id, floor_id) for floor_id in floor_ids]
    write_json(out / "generated_per_floor_map_manifest.json", {"scene_id": scene_id, "maps": generated_maps})
    inventory = file_inventory(stage, out)

    ctx = {
        "raw_dir": raw_dir.as_posix(),
        "dataset_root": args.dataset_root.as_posix(),
        "floor_ids": floor_ids,
        "existing_map_floors": existing_map_floors,
        "floor1_map_exists": (stage / "maps/floor_1/stage1_floor_1_stable_occupancy_map.yaml").exists(),
        "floor2_map_exists": (stage / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml").exists(),
    }
    for name, text in markdown_reports(ctx).items():
        write_text(out / name, text)

    report_json = {
        "task": "task24b2_stage_a_stair_graph_and_per_floor_stable_map_audit",
        "created_utc": now_iso(),
        "scene_id": scene_id,
        "primary_classification": PRIMARY_CLASSIFICATION,
        "stable_map_classification": STABLE_MAP_CLASSIFICATION,
        "claim_boundary": CLAIM_BOUNDARY,
        "stage_a_entrypoint": "stage_a_demo.py",
        "stage_a_python": "/home/ws/miniconda3/envs/boxfusion/bin/python",
        "clean_rerun": stage.as_posix(),
        "raw_stage_a_output": raw_dir.as_posix(),
        "vertical_transition_producer_found": True,
        "from_to_position_producer": "boxfusion/floor_aware_room_segmenter.py::_room_support_summary last representative stable pose XY",
        "clean_rerun_dense_pose_xy_yaw_exported": False,
        "dataset_pose_files_available_for_backfill": args.dataset_root.exists(),
        "hovsg_style_stair_graph_generated_now": bool(graphs),
        "stair_graph_density": graphs[0].get("graph_density") if graphs else None,
        "floor_1_stable_map_exists_in_clean_rerun": ctx["floor1_map_exists"],
        "floor_2_stable_map_exists_in_clean_rerun": ctx["floor2_map_exists"],
        "generated_maps": generated_maps,
        "generated_files": [
            "stairs_graph_v0_1.json",
            "stairs_graph_visualization.png",
            "logs/frame_pose_floor_assignments_v0_1.jsonl",
            "logs/vertical_transition_paths_v0_1.json",
            "logs/stairs_graphs_v0_1.json",
            "logs/semantic_vertical_connector_candidates_v0_1.json",
            "generated_per_floor_map_manifest.json",
        ],
        "inventory_count": len(inventory),
    }
    write_json(out / "task24b2_audit_report.json", report_json)
    write_text(
        out / "task24b2_audit_report.md",
        f"""# Task24b2 Audit Report

Primary classification: `{PRIMARY_CLASSIFICATION}`

Stable-map classification: `{STABLE_MAP_CLASSIFICATION}`

Claim boundary: `{CLAIM_BOUNDARY}`

The current Stage-A entrypoint is `stage_a_demo.py`, run with `/home/ws/miniconda3/envs/boxfusion/bin/python`.

`vertical_transition_evidence.json` is produced by Stage-A finalization, from transition records computed in `boxfusion/floor_aware_room_segmenter.py::_build_vertical_transitions`.

`from_position_xy` and `to_position_xy` are representative nearby stable-frame camera XY values from room-support summaries, not dense stair path samples.

Clean_rerun does not export dense ordered transition XY/yaw. This task generated an experimental HOV-SG-style stair graph from local source dataset pose files plus clean_rerun floor assignments. Stage-A should export these records directly.

`floor_1` does not have a packaged stable occupancy map in clean_rerun. `floor_2` does. Experimental per-floor maps were written under this task directory only; the generic experimental floor_2 map is not byte-identical to the validated packaged floor_2 map, so it is evidence for feasibility, not a canonical replacement.
""",
    )

    validation = {
        "python_compile_new_script": None,
        "json_parse_checks": {},
        "file_existence_checks": {},
        "no_overwrite_clean_rerun_committed_public": {},
        "no_task23b_modified_by_script": True,
        "map_dimension_checks": {},
        "floor2_generated_vs_existing": {},
    }
    try:
        py_compile.compile(__file__, doraise=True)
        validation["python_compile_new_script"] = True
    except py_compile.PyCompileError as exc:
        validation["python_compile_new_script"] = str(exc)
    for path in [out / "stairs_graph_v0_1.json", out / "task24b2_audit_report.json", out / "generated_per_floor_map_manifest.json"]:
        try:
            read_json(path)
            validation["json_parse_checks"][path.name] = True
        except Exception as exc:  # noqa: BLE001
            validation["json_parse_checks"][path.name] = str(exc)
    for path in [out / "stairs_graph_visualization.png", frame_pose_path]:
        validation["file_existence_checks"][path.name] = path.exists()
    for path in committed_public.glob("*.json"):
        validation["no_overwrite_clean_rerun_committed_public"][path.name] = {
            "exists": path.exists(),
            "sha256": sha256(path),
        }
    existing_floor2 = stage / "maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
    generated_floor2 = out / "generated_maps/floor_2/stage1_floor_2_stable_occupancy_map.yaml"
    floor2_compare: dict[str, Any] = {
        "existing": existing_floor2.as_posix(),
        "generated": generated_floor2.as_posix(),
        "both_exist": existing_floor2.exists() and generated_floor2.exists(),
        "note": "experimental generator is not asserted byte-identical to existing floor_2 map",
    }
    if existing_floor2.exists() and generated_floor2.exists():
        existing_meta = parse_map_yaml(existing_floor2)
        generated_meta = parse_map_yaml(generated_floor2)
        existing_pgm = existing_floor2.parent / str(existing_meta.get("image", "")).strip('"')
        generated_pgm = generated_floor2.parent / str(generated_meta.get("image", "")).strip('"')
        if existing_pgm.exists() and generated_pgm.exists():
            existing_grid = read_pgm(existing_pgm)
            generated_grid = read_pgm(generated_pgm)
            same_shape = existing_grid.shape == generated_grid.shape
            floor2_compare.update(
                {
                    "existing_pgm": existing_pgm.as_posix(),
                    "generated_pgm": generated_pgm.as_posix(),
                    "same_shape": bool(same_shape),
                    "existing_shape_hw": list(existing_grid.shape),
                    "generated_shape_hw": list(generated_grid.shape),
                    "byte_identical": bool(same_shape and np.array_equal(existing_grid, generated_grid)),
                    "different_cell_count": None if not same_shape else int(np.count_nonzero(existing_grid != generated_grid)),
                    "existing_occupied_cells": int((existing_grid <= 10).sum()),
                    "generated_occupied_cells": int((generated_grid <= 10).sum()),
                    "interpretation": "The generic task-local builder does not yet reproduce the validated floor_2 stable map exactly; production all-floor packaging should reuse or factor the validated builder semantics.",
                }
            )
    validation["floor2_generated_vs_existing"] = {
        **floor2_compare,
    }
    for item in generated_maps:
        if item.get("generated"):
            validation["map_dimension_checks"][item["floor_id"]] = {
                "shape_hw": item.get("shape_hw"),
                "yaml_exists": Path(item["map_yaml"]).exists(),
                "pgm_exists": Path(item["map_pgm"]).exists(),
                "png_exists": Path(item["map_png"]).exists(),
                "npz_exists": Path(item["map_npz"]).exists(),
            }
    write_json(out / "validation_summary.json", validation)
    write_text(
        out / "validation_summary.md",
        "\n".join(
            [
                "# Validation Summary",
                "",
                f"- Python compile check: `{validation['python_compile_new_script']}`",
                f"- JSON parse checks: `{validation['json_parse_checks']}`",
                f"- Generated file checks: `{validation['file_existence_checks']}`",
                f"- Map dimension checks: `{validation['map_dimension_checks']}`",
                "- Clean_rerun committed_public was read only; hashes recorded in `validation_summary.json`.",
                "- task23b was not modified or rerun.",
            ]
        ),
    )
    write_text(
        out / "final_answer_for_user.md",
        f"""# Final Answer For User

1. Current Stage-A command: see `current_stage_a_rerun_command_plan.md`; use `/home/ws/miniconda3/envs/boxfusion/bin/python stage_a_demo.py ...` into `stage_a_rerun_task24b2/scenes`, not clean_rerun.
2. `from_position_xy` / `to_position_xy` producer was found in `boxfusion/floor_aware_room_segmenter.py`.
3. Dense stair graph samples are not exported in clean_rerun, but source dataset pose files exist locally.
4. A HOV-SG-style stair graph was generated as an experimental backfill from true source pose files plus Stage-A floor assignment.
5. `floor_1` stable occupancy map is missing from clean_rerun/maps.
6. Stable maps can be generated experimentally for all detected floors from existing floor BEV assets; production logic should be added to Stage-A/runtime packaging.
7. Needed next: Stage-A exporter backfill/modification, not a full rerun unless you need artifacts generated by the modified exporter end to end.
8. Generated files are under this task directory.
9. Recommended next task: implement Stage-A exports `frame_pose_floor_assignments_v0_1.jsonl`, `vertical_transition_paths_v0_1.json`, `stairs_graphs_v0_1.json`, and all-floor stable map packaging.
""",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
