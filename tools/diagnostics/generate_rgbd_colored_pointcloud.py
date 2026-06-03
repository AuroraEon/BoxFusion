#!/usr/bin/env python3
"""Safely estimate or generate an RGB-D colored point cloud.

The default is intentionally non-writing: it selects motion keyframes, estimates
raw/fused sizes, and writes only a manifest. Use --write to emit PLY shards.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image

try:
    import yaml
except ModuleNotFoundError:  # Keep --help and basic config parsing usable in lean envs.
    yaml = None


BINARY_POINT_BYTES = 15  # float32 x/y/z + uint8 r/g/b
ASCII_POINT_BYTES_ESTIMATE = 48
GB = 1024 ** 3


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale: float


@dataclass(frozen=True)
class FrameRecord:
    stem: str
    rgb_path: Path
    depth_path: Path
    pose_path: Path
    pose: np.ndarray


@dataclass(frozen=True)
class ConventionSelection:
    pose_convention: str
    camera_convention: str
    transform_description: str
    scores: dict


def _habitat_to_opencv_camera_transform() -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[1, 1] = -1.0
    transform[2, 2] = -1.0
    return transform


def _yup_to_zup_world_transform() -> np.ndarray:
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _camera_convention_description(name: str) -> str:
    if name == "opencv":
        return "OpenCV camera coordinates: +x right, +y down, +z forward; pose is already in the Stage-A Z-up world frame."
    if name in {"opengl_habitat", "habitat"}:
        return (
            "Habitat/OpenGL camera coordinates in Habitat Y-up world. Stage-A conversion is "
            "opencv_zup_c2w = T_Yup2Zup @ pose_txt_c2w @ C_habitat2opencv, where "
            "C_habitat2opencv flips camera y and z, and T_Yup2Zup maps Habitat +Y to world +Z."
        )
    raise ValueError(f"Unsupported camera convention: {name}")


def _pose_to_opencv_zup_c2w(raw_pose: np.ndarray, *, pose_convention: str, camera_convention: str) -> np.ndarray:
    if pose_convention == "camera_to_world":
        c2w = raw_pose
    elif pose_convention == "world_to_camera":
        c2w = np.linalg.inv(raw_pose)
    else:
        raise ValueError(f"Unsupported pose convention: {pose_convention}")

    if camera_convention == "opencv":
        return np.asarray(c2w, dtype=np.float64)
    if camera_convention in {"opengl_habitat", "habitat"}:
        return _yup_to_zup_world_transform() @ c2w @ _habitat_to_opencv_camera_transform()
    raise ValueError(f"Unsupported camera convention: {camera_convention}")


def _load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        text = handle.read()
    if yaml is not None:
        return yaml.safe_load(text) or {}

    config: dict = {}
    current_section: Optional[str] = None
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith((" ", "\t")) and line.endswith(":"):
            current_section = line[:-1].strip()
            config.setdefault(current_section, {})
            continue
        if current_section is None or ":" not in line:
            continue
        key, value = line.strip().split(":", 1)
        value = value.strip()
        try:
            parsed: object = float(value)
            if parsed.is_integer():
                parsed = int(parsed)
        except ValueError:
            parsed = value
        config.setdefault(current_section, {})[key.strip()] = parsed
    return config


def _intrinsics_from_config(config: dict) -> CameraIntrinsics:
    cam = config.get("cam") or {}
    required = ["W", "H", "fx", "fy", "cx", "cy", "png_depth_scale"]
    missing = [key for key in required if key not in cam]
    if missing:
        raise ValueError(f"Config cam section is missing required keys: {missing}")
    return CameraIntrinsics(
        width=int(cam["W"]),
        height=int(cam["H"]),
        fx=float(cam["fx"]),
        fy=float(cam["fy"]),
        cx=float(cam["cx"]),
        cy=float(cam["cy"]),
        depth_scale=float(cam["png_depth_scale"]),
    )


def _read_pose(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64)
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"Pose has non-finite values: {path}")
    return matrix


def _discover_frames(scene_root: Path) -> List[FrameRecord]:
    rgb_dir = scene_root / "rgb"
    depth_dir = scene_root / "depth"
    pose_dir = scene_root / "pose"
    if not rgb_dir.is_dir() or not depth_dir.is_dir() or not pose_dir.is_dir():
        raise FileNotFoundError(f"Expected rgb/depth/pose directories under {scene_root}")

    rgb_paths = {path.stem: path for path in sorted(rgb_dir.glob("*.png"))}
    depth_paths = {path.stem: path for path in sorted(depth_dir.glob("*.png"))}
    pose_paths = {path.stem: path for path in sorted(pose_dir.glob("*.txt"))}
    stems = set(rgb_paths) | set(depth_paths) | set(pose_paths)
    mismatched = {
        stem: {
            "rgb": stem in rgb_paths,
            "depth": stem in depth_paths,
            "pose": stem in pose_paths,
        }
        for stem in sorted(stems)
        if not (stem in rgb_paths and stem in depth_paths and stem in pose_paths)
    }
    if mismatched:
        preview = dict(list(mismatched.items())[:20])
        raise ValueError(
            "RGB/depth/pose frame stems must match exactly. "
            f"First mismatches: {json.dumps(preview, sort_keys=True)}"
        )

    records: List[FrameRecord] = []
    for stem in sorted(stems):
        rgb_path = rgb_paths[stem]
        depth_path = depth_paths[stem]
        pose_path = pose_paths[stem]
        records.append(
            FrameRecord(
                stem=stem,
                rgb_path=rgb_path,
                depth_path=depth_path,
                pose_path=pose_path,
                pose=_read_pose(pose_path),
            )
        )
    if not records:
        raise FileNotFoundError(f"No matched rgb/depth/pose frames found under {scene_root}")
    return records


def _rotation_angle_deg(previous: np.ndarray, current: np.ndarray) -> float:
    relative = previous[:3, :3].T @ current[:3, :3]
    cos_theta = float((np.trace(relative) - 1.0) * 0.5)
    cos_theta = min(1.0, max(-1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def _select_keyframes(
    frames: Sequence[FrameRecord],
    *,
    translation_m: float,
    rotation_deg: float,
) -> Tuple[List[int], Dict[str, object]]:
    selected = [0]
    last_pose = frames[0].pose
    for idx in range(1, len(frames) - 1):
        pose = frames[idx].pose
        translation = float(np.linalg.norm(pose[:3, 3] - last_pose[:3, 3]))
        rotation = _rotation_angle_deg(last_pose, pose)
        if translation >= translation_m or rotation >= rotation_deg:
            selected.append(idx)
            last_pose = pose
    if len(frames) > 1 and selected[-1] != len(frames) - 1:
        selected.append(len(frames) - 1)
    return selected, {
        "type": "motion_threshold",
        "keyframe_translation_m": translation_m,
        "keyframe_rotation_deg": rotation_deg,
        "always_keep": ["first", "last"],
    }


def _valid_depth_mask(depth: np.ndarray, intr: CameraIntrinsics, depth_min: Optional[float], depth_max: float) -> np.ndarray:
    depth_m = depth.astype(np.float32) / np.float32(intr.depth_scale)
    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m <= np.float32(depth_max))
    if depth_min is not None:
        valid &= depth_m >= np.float32(depth_min)
    return valid


def _load_depth(path: Path, intr: CameraIntrinsics) -> np.ndarray:
    depth = np.asarray(Image.open(path))
    if depth.shape[:2] != (intr.height, intr.width):
        raise ValueError(f"Depth size mismatch for {path}: got {depth.shape[:2]}, expected {(intr.height, intr.width)}")
    return depth


def _load_rgb(path: Path, intr: CameraIntrinsics, *, allow_resize: bool) -> np.ndarray:
    image = Image.open(path).convert("RGB")
    if image.size != (intr.width, intr.height):
        if not allow_resize:
            raise ValueError(
                f"RGB size mismatch for {path}: got {(image.size[1], image.size[0])}, "
                f"expected {(intr.height, intr.width)}. Pass --allow-rgb-resize to resize explicitly."
            )
        image = image.resize((intr.width, intr.height), Image.BILINEAR)
    return np.asarray(image, dtype=np.uint8)


def _iter_valid_world_points(
    frame: FrameRecord,
    intr: CameraIntrinsics,
    *,
    c2w: np.ndarray,
    depth_min: Optional[float],
    depth_max: float,
    include_rgb: bool,
    chunk_rows: int,
    allow_rgb_resize: bool,
) -> Iterable[Tuple[np.ndarray, Optional[np.ndarray]]]:
    depth = _load_depth(frame.depth_path, intr).astype(np.float32) / np.float32(intr.depth_scale)
    rgb = _load_rgb(frame.rgb_path, intr, allow_resize=allow_rgb_resize) if include_rgb else None
    rotation = c2w[:3, :3].astype(np.float32)
    translation = c2w[:3, 3].astype(np.float32)
    xs = np.arange(intr.width, dtype=np.float32)
    for row0 in range(0, intr.height, chunk_rows):
        row1 = min(intr.height, row0 + chunk_rows)
        z = depth[row0:row1]
        valid = np.isfinite(z) & (z > 0.0) & (z <= np.float32(depth_max))
        if depth_min is not None:
            valid &= z >= np.float32(depth_min)
        if not np.any(valid):
            continue
        ys = np.arange(row0, row1, dtype=np.float32)[:, None]
        x_cam = ((xs[None, :] - np.float32(intr.cx)) / np.float32(intr.fx)) * z
        y_cam = ((ys - np.float32(intr.cy)) / np.float32(intr.fy)) * z
        camera_points = np.stack((x_cam, y_cam, z), axis=-1)[valid]
        world_points = camera_points @ rotation.T + translation
        colors = None if rgb is None else rgb[row0:row1][valid]
        yield world_points.astype(np.float32, copy=False), colors


def _count_raw_valid_points(
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
    *,
    depth_min: Optional[float],
    depth_max: float,
) -> int:
    total = 0
    for idx in selected_indices:
        depth = _load_depth(frames[idx].depth_path, intr)
        total += int(np.count_nonzero(_valid_depth_mask(depth, intr, depth_min, depth_max)))
    return total


def _estimate_fused_points(
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
    *,
    pose_convention: str,
    camera_convention: str,
    voxel_size: float,
    depth_min: Optional[float],
    depth_max: float,
    estimation_pixel_stride: int,
    key_cap: int,
    chunk_rows: int,
    raw_count: int,
) -> Tuple[Optional[int], List[str]]:
    warnings: List[str] = []
    keys = set()
    sampled_valid = 0
    stride = max(1, int(estimation_pixel_stride))
    for idx in selected_indices:
        frame = frames[idx]
        depth = _load_depth(frame.depth_path, intr).astype(np.float32) / np.float32(intr.depth_scale)
        depth = depth[::stride, ::stride]
        z = depth
        valid = np.isfinite(z) & (z > 0.0) & (z <= np.float32(depth_max))
        if depth_min is not None:
            valid &= z >= np.float32(depth_min)
        if not np.any(valid):
            continue
        ys = np.arange(0, intr.height, stride, dtype=np.float32)[: z.shape[0]][:, None]
        xs = np.arange(0, intr.width, stride, dtype=np.float32)[: z.shape[1]]
        x_cam = ((xs[None, :] - np.float32(intr.cx)) / np.float32(intr.fx)) * z
        y_cam = ((ys - np.float32(intr.cy)) / np.float32(intr.fy)) * z
        camera_points = np.stack((x_cam, y_cam, z), axis=-1)[valid]
        c2w = _pose_to_opencv_zup_c2w(frame.pose, pose_convention=pose_convention, camera_convention=camera_convention)
        rotation = c2w[:3, :3].astype(np.float32)
        translation = c2w[:3, 3].astype(np.float32)
        points = camera_points @ rotation.T + translation
        sampled_valid += int(points.shape[0])
        voxel = np.floor(points / np.float32(voxel_size)).astype(np.int64)
        for key in map(tuple, voxel):
            keys.add(key)
        if len(keys) > key_cap:
            warnings.append(
                f"Fused estimate exceeded estimation voxel key cap ({key_cap}); fused count omitted."
            )
            return None, warnings
    if stride > 1:
        warnings.append(
            "Fused point count is approximate because estimation used pixel stride "
            f"{stride}; reported count is a conservative scaled estimate and write mode computes exact voxel fusion."
        )
    if sampled_valid == 0:
        return 0, warnings
    if stride <= 1:
        return len(keys), warnings
    # Unique voxels do not scale linearly with pixel stride. Scale the sampled
    # occupancy by the sampled pixel area and cap by raw valid points so the
    # write gate errs on the side of refusing a too-large output.
    return int(min(int(raw_count), int(len(keys) * stride * stride))), warnings


def _ply_header(point_count: int, *, binary: bool) -> bytes:
    fmt = "binary_little_endian" if binary else "ascii"
    lines = [
        "ply",
        f"format {fmt} 1.0",
        f"element vertex {point_count}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
        "",
    ]
    return "\n".join(lines).encode("ascii")


def _write_binary_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    with path.open("wb") as handle:
        handle.write(_ply_header(int(points.shape[0]), binary=True))
        payload = np.empty(
            int(points.shape[0]),
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")],
        )
        payload["x"] = points[:, 0]
        payload["y"] = points[:, 1]
        payload["z"] = points[:, 2]
        payload["r"] = colors[:, 0]
        payload["g"] = colors[:, 1]
        payload["b"] = colors[:, 2]
        payload.tofile(handle)


def _write_raw_shards(
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
    output_dir: Path,
    *,
    pose_convention: str,
    camera_convention: str,
    max_points_per_shard: int,
    depth_min: Optional[float],
    depth_max: float,
    chunk_rows: int,
    allow_rgb_resize: bool,
) -> Tuple[List[dict], int]:
    output_files: List[dict] = []
    shard_points: List[np.ndarray] = []
    shard_colors: List[np.ndarray] = []
    shard_count = 0
    shard_index = 0
    total_points = 0

    def flush() -> None:
        nonlocal shard_count, shard_index, shard_points, shard_colors
        if shard_count <= 0:
            return
        points = np.concatenate(shard_points, axis=0)
        colors = np.concatenate(shard_colors, axis=0)
        path = output_dir / f"rgbd_raw_points_shard_{shard_index:04d}.ply"
        _write_binary_ply(path, points, colors)
        output_files.append({"path": str(path), "points": int(points.shape[0]), "bytes": path.stat().st_size})
        shard_index += 1
        shard_count = 0
        shard_points = []
        shard_colors = []

    for idx in selected_indices:
        c2w = _pose_to_opencv_zup_c2w(frames[idx].pose, pose_convention=pose_convention, camera_convention=camera_convention)
        for points, colors in _iter_valid_world_points(
            frames[idx],
            intr,
            c2w=c2w,
            depth_min=depth_min,
            depth_max=depth_max,
            include_rgb=True,
            chunk_rows=chunk_rows,
            allow_rgb_resize=allow_rgb_resize,
        ):
            assert colors is not None
            start = 0
            while start < points.shape[0]:
                remaining = max_points_per_shard - shard_count
                end = min(points.shape[0], start + remaining)
                shard_points.append(points[start:end].copy())
                shard_colors.append(colors[start:end].copy())
                added = end - start
                shard_count += added
                total_points += added
                start = end
                if shard_count >= max_points_per_shard:
                    flush()
    flush()
    return output_files, total_points


def _write_voxel_fused_shards(
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
    output_dir: Path,
    *,
    pose_convention: str,
    camera_convention: str,
    voxel_size: float,
    max_points_per_shard: int,
    depth_min: Optional[float],
    depth_max: float,
    chunk_rows: int,
    allow_rgb_resize: bool,
) -> Tuple[List[dict], int]:
    sums: Dict[Tuple[int, int, int], np.ndarray] = {}
    counts: Dict[Tuple[int, int, int], int] = {}
    color_sums: Dict[Tuple[int, int, int], np.ndarray] = {}

    for idx in selected_indices:
        c2w = _pose_to_opencv_zup_c2w(frames[idx].pose, pose_convention=pose_convention, camera_convention=camera_convention)
        for points, colors in _iter_valid_world_points(
            frames[idx],
            intr,
            c2w=c2w,
            depth_min=depth_min,
            depth_max=depth_max,
            include_rgb=True,
            chunk_rows=chunk_rows,
            allow_rgb_resize=allow_rgb_resize,
        ):
            assert colors is not None
            voxels = np.floor(points / np.float32(voxel_size)).astype(np.int64)
            for point, color, voxel in zip(points, colors, voxels):
                key = (int(voxel[0]), int(voxel[1]), int(voxel[2]))
                if key in counts:
                    sums[key] += point
                    color_sums[key] += color.astype(np.float64)
                    counts[key] += 1
                else:
                    sums[key] = point.astype(np.float64)
                    color_sums[key] = color.astype(np.float64)
                    counts[key] = 1

    output_files: List[dict] = []
    keys = list(counts.keys())
    total = len(keys)
    for shard_index, start in enumerate(range(0, total, max_points_per_shard)):
        shard_keys = keys[start : start + max_points_per_shard]
        points = np.empty((len(shard_keys), 3), dtype=np.float32)
        colors = np.empty((len(shard_keys), 3), dtype=np.uint8)
        for row, key in enumerate(shard_keys):
            count = float(counts[key])
            points[row] = (sums[key] / count).astype(np.float32)
            colors[row] = np.clip(np.rint(color_sums[key] / count), 0, 255).astype(np.uint8)
        path = output_dir / f"rgbd_voxel_fused_{voxel_size:.3f}m_shard_{shard_index:04d}.ply"
        _write_binary_ply(path, points, colors)
        output_files.append({"path": str(path), "points": int(points.shape[0]), "bytes": path.stat().st_size})
    return output_files, total


def _load_floor_bands(path: Optional[Path]) -> list[dict]:
    if not path:
        return []
    if not path.exists():
        return []
    payload = read_json(path)
    return list(payload.get("per_floor") or payload.get("floors") or [])


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_room_bounds(path: Optional[Path]) -> tuple[float, float, float, float] | None:
    if not path or not path.exists():
        return None
    payload = read_json(path)
    points: list[tuple[float, float]] = []
    for room in payload.get("rooms") or []:
        poly = room.get("polygon") or room.get("region_polygon") or []
        for xy in poly:
            if len(xy) >= 2:
                points.append((float(xy[0]), float(xy[1])))
        label = room.get("label_xy")
        if label and len(label) >= 2:
            points.append((float(label[0]), float(label[1])))
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    margin = 3.0
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def _camera_points_sample(
    frame: FrameRecord,
    intr: CameraIntrinsics,
    *,
    depth_min: Optional[float],
    depth_max: float,
    pixel_stride: int,
    include_rgb: bool,
    allow_rgb_resize: bool,
) -> tuple[np.ndarray, np.ndarray]:
    depth_raw = _load_depth(frame.depth_path, intr)
    depth = depth_raw.astype(np.float32) / np.float32(intr.depth_scale)
    stride = max(1, int(pixel_stride))
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.0) & (z <= np.float32(depth_max))
    if depth_min is not None:
        valid &= z >= np.float32(depth_min)
    ys = np.arange(0, intr.height, stride, dtype=np.float32)[: z.shape[0]][:, None]
    xs = np.arange(0, intr.width, stride, dtype=np.float32)[: z.shape[1]]
    x_cam = ((xs[None, :] - np.float32(intr.cx)) / np.float32(intr.fx)) * z
    y_cam = ((ys - np.float32(intr.cy)) / np.float32(intr.fy)) * z
    points = np.stack((x_cam, y_cam, z), axis=-1)[valid].astype(np.float32, copy=False)
    if include_rgb:
        rgb = _load_rgb(frame.rgb_path, intr, allow_resize=allow_rgb_resize)[::stride, ::stride][valid]
    else:
        rgb = np.full((points.shape[0], 3), 210, dtype=np.uint8)
    return points, rgb


def _frame_alignment_stats(frame: FrameRecord, intr: CameraIntrinsics, *, depth_min: Optional[float], depth_max: float, allow_rgb_resize: bool) -> dict:
    depth = _load_depth(frame.depth_path, intr)
    image = Image.open(frame.rgb_path)
    if image.size != (intr.width, intr.height) and not allow_rgb_resize:
        raise ValueError(
            f"RGB size mismatch for {frame.rgb_path}: got {(image.size[1], image.size[0])}, "
            f"expected {(intr.height, intr.width)}. Pass --allow-rgb-resize to resize explicitly."
        )
    depth_m = depth.astype(np.float32) / np.float32(intr.depth_scale)
    valid = _valid_depth_mask(depth, intr, depth_min, depth_max)
    finite = depth_m[np.isfinite(depth_m)]
    return {
        "stem": frame.stem,
        "rgb_size_hw": [int(image.size[1]), int(image.size[0])],
        "depth_size_hw": [int(depth.shape[0]), int(depth.shape[1])],
        "depth_dtype": str(depth.dtype),
        "depth_scale": float(intr.depth_scale),
        "depth_raw_min": int(np.min(depth)) if depth.size else None,
        "depth_raw_max": int(np.max(depth)) if depth.size else None,
        "depth_m_min": float(np.min(finite)) if finite.size else None,
        "depth_m_max": float(np.max(finite)) if finite.size else None,
        "invalid_depth_ratio": float(1.0 - (np.count_nonzero(valid) / max(1, valid.size))),
        "allow_rgb_resize": bool(allow_rgb_resize),
    }


def _score_convention(
    frames: Sequence[FrameRecord],
    indices: Sequence[int],
    intr: CameraIntrinsics,
    *,
    pose_convention: str,
    camera_convention: str,
    floor_bands: Sequence[dict],
    room_bounds: tuple[float, float, float, float] | None,
    depth_min: Optional[float],
    depth_max: float,
    pixel_stride: int,
    voxel_size: float,
    allow_rgb_resize: bool,
) -> dict:
    poses = [_pose_to_opencv_zup_c2w(frames[idx].pose, pose_convention=pose_convention, camera_convention=camera_convention) for idx in indices]
    translations = np.asarray([pose[:3, 3] for pose in poses], dtype=np.float64)
    points_all: list[np.ndarray] = []
    for frame, c2w in zip([frames[idx] for idx in indices], poses):
        cam, _ = _camera_points_sample(
            frame,
            intr,
            depth_min=depth_min,
            depth_max=depth_max,
            pixel_stride=pixel_stride,
            include_rgb=False,
            allow_rgb_resize=allow_rgb_resize,
        )
        if cam.size == 0:
            continue
        world = cam @ c2w[:3, :3].T.astype(np.float32) + c2w[:3, 3].astype(np.float32)
        if world.shape[0] > 8000:
            world = world[:: max(1, world.shape[0] // 8000)]
        points_all.append(world)
    points = np.concatenate(points_all, axis=0) if points_all else np.empty((0, 3), dtype=np.float32)

    floor_pose_fraction = 0.0
    point_height_fraction = 0.0
    if floor_bands:
        def in_any_band(z: np.ndarray, margin: float) -> np.ndarray:
            mask = np.zeros_like(z, dtype=bool)
            for band in floor_bands:
                mask |= (z >= float(band["z_min"]) - margin) & (z <= float(band["z_max"]) + margin)
            return mask

        floor_pose_fraction = float(np.mean(in_any_band(translations[:, 2], 0.35))) if translations.size else 0.0
        point_height_fraction = float(np.mean(in_any_band(points[:, 2], 1.25))) if points.size else 0.0

    bev_bounds_fraction = 0.0
    if room_bounds and points.size:
        xmin, xmax, ymin, ymax = room_bounds
        bev_bounds_fraction = float(np.mean((points[:, 0] >= xmin) & (points[:, 0] <= xmax) & (points[:, 1] >= ymin) & (points[:, 1] <= ymax)))

    vertical_range = float(np.ptp(points[:, 2])) if points.size else 999.0
    trajectory_vertical_range = float(np.ptp(translations[:, 2])) if translations.size else 999.0
    voxel_overlap_score = 0.0
    if points.size:
        voxels = np.floor(points / np.float32(voxel_size)).astype(np.int64)
        unique = len({tuple(v) for v in voxels})
        voxel_overlap_score = float(1.0 - min(1.0, unique / max(1, points.shape[0])))

    score = (
        3.0 * floor_pose_fraction
        + 2.0 * point_height_fraction
        + 1.5 * bev_bounds_fraction
        + 0.75 * voxel_overlap_score
        + max(0.0, 1.0 - trajectory_vertical_range / 8.0)
        + max(0.0, 1.0 - abs(vertical_range - 6.0) / 18.0)
    )
    return {
        "pose_convention": pose_convention,
        "camera_convention": camera_convention,
        "score": round(float(score), 6),
        "floor_pose_fraction": round(floor_pose_fraction, 6),
        "point_height_fraction": round(point_height_fraction, 6),
        "bev_bounds_fraction": round(bev_bounds_fraction, 6),
        "voxel_overlap_score": round(voxel_overlap_score, 6),
        "point_count_scored": int(points.shape[0]),
        "trajectory_translation_z_min": round(float(np.min(translations[:, 2])), 6) if translations.size else None,
        "trajectory_translation_z_max": round(float(np.max(translations[:, 2])), 6) if translations.size else None,
        "trajectory_vertical_range": round(trajectory_vertical_range, 6),
        "point_z_min": round(float(np.min(points[:, 2])), 6) if points.size else None,
        "point_z_max": round(float(np.max(points[:, 2])), 6) if points.size else None,
        "point_vertical_range": round(vertical_range, 6),
    }


def _select_convention(
    args: argparse.Namespace,
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
) -> ConventionSelection:
    floor_bands = _load_floor_bands(args.floor_diagnostics_json)
    room_bounds = _load_room_bounds(args.stage1_floorplan_report)
    diag_indices = list(selected_indices[: max(1, min(len(selected_indices), args.diagnostic_frame_count))])
    pose_candidates = ["camera_to_world", "world_to_camera"] if args.pose_convention == "auto" else [args.pose_convention]
    camera_candidates = ["opencv", "opengl_habitat", "habitat"] if args.camera_convention == "auto" else [args.camera_convention]
    scores = [
        _score_convention(
            frames,
            diag_indices,
            intr,
            pose_convention=pose_convention,
            camera_convention=camera_convention,
            floor_bands=floor_bands,
            room_bounds=room_bounds,
            depth_min=args.depth_min,
            depth_max=args.depth_max,
            pixel_stride=args.diagnostic_pixel_stride,
            voxel_size=args.voxel_size,
            allow_rgb_resize=args.allow_rgb_resize,
        )
        for pose_convention in pose_candidates
        for camera_convention in camera_candidates
    ]
    best = max(scores, key=lambda row: row["score"])
    return ConventionSelection(
        pose_convention=str(best["pose_convention"]),
        camera_convention=str(best["camera_convention"]),
        transform_description=_camera_convention_description(str(best["camera_convention"])),
        scores={
            "selected": best,
            "candidates": scores,
            "floor_diagnostics_json": str(args.floor_diagnostics_json.resolve()) if args.floor_diagnostics_json else None,
            "stage1_floorplan_report": str(args.stage1_floorplan_report.resolve()) if args.stage1_floorplan_report else None,
            "diagnostic_frame_indices": [int(i) for i in diag_indices],
            "auto_selection_used": args.pose_convention == "auto" or args.camera_convention == "auto",
        },
    )


def _write_topdown_png(path: Path, points: np.ndarray, poses: np.ndarray | None = None) -> None:
    if points.size == 0:
        Image.new("RGB", (640, 640), (20, 20, 20)).save(path)
        return
    xy = points[:, :2]
    xmin, ymin = np.percentile(xy, 1, axis=0)
    xmax, ymax = np.percentile(xy, 99, axis=0)
    if poses is not None and poses.size:
        xmin = min(xmin, float(np.min(poses[:, 0])))
        xmax = max(xmax, float(np.max(poses[:, 0])))
        ymin = min(ymin, float(np.min(poses[:, 1])))
        ymax = max(ymax, float(np.max(poses[:, 1])))
    span = max(float(xmax - xmin), float(ymax - ymin), 1.0)
    cx, cy = (float(xmin + xmax) * 0.5, float(ymin + ymax) * 0.5)
    xmin, xmax = cx - span * 0.55, cx + span * 0.55
    ymin, ymax = cy - span * 0.55, cy + span * 0.55
    width = height = 900
    img = np.full((height, width, 3), 245, dtype=np.uint8)
    cols = np.clip(((xy[:, 0] - xmin) / (xmax - xmin) * (width - 1)).astype(np.int32), 0, width - 1)
    rows = np.clip((height - 1 - (xy[:, 1] - ymin) / (ymax - ymin) * (height - 1)).astype(np.int32), 0, height - 1)
    img[rows, cols] = (35, 95, 180)
    if poses is not None and poses.size:
        pc = np.clip(((poses[:, 0] - xmin) / (xmax - xmin) * (width - 1)).astype(np.int32), 0, width - 1)
        pr = np.clip((height - 1 - (poses[:, 1] - ymin) / (ymax - ymin) * (height - 1)).astype(np.int32), 0, height - 1)
        for r, c in zip(pr, pc):
            img[max(0, r - 3): min(height, r + 4), max(0, c - 3): min(width, c + 4)] = (220, 45, 45)
    Image.fromarray(img, mode="RGB").save(path)


def _write_diagnostic_outputs(
    args: argparse.Namespace,
    frames: Sequence[FrameRecord],
    selected_indices: Sequence[int],
    intr: CameraIntrinsics,
    selection: ConventionSelection,
) -> dict:
    diagnostics: dict[str, Any] = {}
    diag_indices = list(selected_indices[: max(1, min(len(selected_indices), args.diagnostic_frame_count))])
    frame_idx = int(diag_indices[0])
    frame = frames[frame_idx]
    cam_points, colors = _camera_points_sample(
        frame,
        intr,
        depth_min=args.depth_min,
        depth_max=args.depth_max,
        pixel_stride=args.diagnostic_pixel_stride,
        include_rgb=True,
        allow_rgb_resize=args.allow_rgb_resize,
    )
    c2w = _pose_to_opencv_zup_c2w(frame.pose, pose_convention=selection.pose_convention, camera_convention=selection.camera_convention)
    world_points = cam_points @ c2w[:3, :3].T.astype(np.float32) + c2w[:3, 3].astype(np.float32)
    if args.write:
        camera_path = args.output_dir / f"single_frame_debug_{frame.stem}_camera_frame.ply"
        world_path = args.output_dir / f"single_frame_debug_{frame.stem}.ply"
        _write_binary_ply(camera_path, cam_points, colors)
        _write_binary_ply(world_path, world_points, colors)
        diagnostics["single_frame_camera_frame_ply"] = str(camera_path)
        diagnostics["single_frame_world_frame_ply"] = str(world_path)

    multi_points: list[np.ndarray] = []
    multi_colors: list[np.ndarray] = []
    pose_rows: list[np.ndarray] = []
    for idx in diag_indices:
        f = frames[idx]
        c2w_i = _pose_to_opencv_zup_c2w(f.pose, pose_convention=selection.pose_convention, camera_convention=selection.camera_convention)
        cam_i, col_i = _camera_points_sample(
            f,
            intr,
            depth_min=args.depth_min,
            depth_max=args.depth_max,
            pixel_stride=args.diagnostic_pixel_stride,
            include_rgb=True,
            allow_rgb_resize=args.allow_rgb_resize,
        )
        pts = cam_i @ c2w_i[:3, :3].T.astype(np.float32) + c2w_i[:3, 3].astype(np.float32)
        multi_points.append(pts)
        multi_colors.append(col_i)
        pose_rows.append(c2w_i[:3, 3])
    fused_points = np.concatenate(multi_points, axis=0) if multi_points else np.empty((0, 3), dtype=np.float32)
    fused_colors = np.concatenate(multi_colors, axis=0) if multi_colors else np.empty((0, 3), dtype=np.uint8)
    poses = np.asarray(pose_rows, dtype=np.float32) if pose_rows else np.empty((0, 3), dtype=np.float32)
    if fused_points.shape[0] > args.diagnostic_max_points:
        stride = max(1, fused_points.shape[0] // args.diagnostic_max_points)
        fused_points = fused_points[::stride]
        fused_colors = fused_colors[::stride]
    topdown = args.output_dir / "topdown_debug_auto_selected.png"
    trajectory_png = args.output_dir / "camera_trajectory_debug.png"
    _write_topdown_png(topdown, fused_points, poses)
    _write_topdown_png(trajectory_png, poses if poses.size else fused_points, poses)
    diagnostics["topdown_debug_auto_selected_png"] = str(topdown)
    diagnostics["camera_trajectory_debug_png"] = str(trajectory_png)
    if args.write:
        multi_path = args.output_dir / "multiframe_debug_auto_selected.ply"
        _write_binary_ply(multi_path, fused_points, fused_colors)
        diagnostics["multiframe_debug_auto_selected_ply"] = str(multi_path)
    diagnostics["diagnostic_frame_stems"] = [frames[idx].stem for idx in diag_indices]
    diagnostics["diagnostic_point_count"] = int(fused_points.shape[0])
    return diagnostics


def _write_pose_report(output_dir: Path, selection: ConventionSelection, diagnostics: dict[str, Any], alignment: list[dict]) -> None:
    payload = {
        "artifact_type": "rgbd_pose_convention_report",
        "selected_pose_convention": selection.pose_convention,
        "selected_camera_convention": selection.camera_convention,
        "transform_description": selection.transform_description,
        "scores": selection.scores,
        "alignment_checks": alignment,
        "diagnostic_outputs": diagnostics,
        "coherent": bool(selection.scores["selected"]["floor_pose_fraction"] >= 0.8 and selection.scores["selected"]["point_height_fraction"] >= 0.5),
    }
    json_path = output_dir / "pose_convention_report.json"
    md_path = output_dir / "pose_convention_report.md"
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = [
        "# RGB-D Pose Convention Report",
        "",
        f"- selected pose convention: `{selection.pose_convention}`",
        f"- selected camera convention: `{selection.camera_convention}`",
        f"- coherent: `{payload['coherent']}`",
        f"- selected score: `{selection.scores['selected']['score']}`",
        f"- floor pose fraction: `{selection.scores['selected']['floor_pose_fraction']}`",
        f"- point height fraction: `{selection.scores['selected']['point_height_fraction']}`",
        f"- BEV bounds fraction: `{selection.scores['selected']['bev_bounds_fraction']}`",
        "",
        selection.transform_description,
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_manifest(
    *,
    args: argparse.Namespace,
    scene_id: str,
    intr: CameraIntrinsics,
    frame_count: int,
    selected_indices: Sequence[int],
    selection_policy: Dict[str, object],
    raw_count: int,
    fused_count: Optional[int],
    estimated_output_gb: float,
    output_files: Sequence[dict],
    write_executed: bool,
    warnings: Sequence[str],
    convention_selection: ConventionSelection,
    diagnostic_outputs: dict,
) -> dict:
    return {
        "scene_id": scene_id,
        "config_path": str(Path(args.config).resolve()),
        "input_frame_count": int(frame_count),
        "selected_keyframe_count": int(len(selected_indices)),
        "selected_keyframe_indices": [int(i) for i in selected_indices],
        "frame_selection_policy": selection_policy,
        "depth_scale": intr.depth_scale,
        "pose_convention": convention_selection.pose_convention,
        "camera_convention": convention_selection.camera_convention,
        "camera_convention_definition": convention_selection.transform_description,
        "pose_convention_report": str(Path(args.output_dir).resolve() / "pose_convention_report.json"),
        "diagnostic_outputs": diagnostic_outputs,
        "intrinsics": {
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
        },
        "depth_filters": {
            "depth_min": args.depth_min,
            "depth_max": args.depth_max,
        },
        "mode": args.mode,
        "voxel_size": args.voxel_size if args.mode in {"voxel", "fused"} else None,
        "estimated_raw_point_count": int(raw_count),
        "estimated_fused_point_count": None if fused_count is None else int(fused_count),
        "estimated_output_size_gb": estimated_output_gb,
        "target_size_gb": args.target_size_gb,
        "max_output_gb": args.max_output_gb,
        "max_points_per_shard": args.max_points_per_shard,
        "output_files": list(output_files),
        "write_executed": bool(write_executed),
        "raw_mode_used": bool(args.mode == "raw"),
        "large_output_allowed": bool(args.allow_large_output),
        "memory_policy": {
            "raw_points_loaded_all_at_once": False,
            "frame_processing": "incremental",
            "raw_mode_sharding": True,
            "voxel_mode_storage": "one aggregate per occupied world-space voxel",
            "estimation_pixel_stride": args.estimation_pixel_stride,
            "chunk_rows": args.chunk_rows,
        },
        "warnings": list(warnings),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", required=True, type=Path, help="HM3D scene root with rgb/depth/pose directories")
    parser.add_argument("--config", required=True, type=Path, help="YAML config containing camera intrinsics/depth scale")
    parser.add_argument("--output-dir", required=True, type=Path, help="Directory for pointcloud_manifest.json and optional PLY shards")
    parser.add_argument("--mode", choices=["voxel", "fused", "raw", "pixel-stride"], default="voxel")
    parser.add_argument("--pose-convention", choices=["auto", "camera_to_world", "world_to_camera"], default="auto")
    parser.add_argument("--camera-convention", choices=["auto", "opencv", "opengl_habitat", "habitat"], default="auto")
    parser.add_argument("--floor-diagnostics-json", type=Path)
    parser.add_argument("--stage1-floorplan-report", type=Path)
    parser.add_argument("--allow-rgb-resize", action="store_true", help="Explicitly allow RGB resizing to the configured intrinsics.")
    parser.add_argument("--write", action="store_true", help="Actually write PLY shards. Default is dry-run manifest only.")
    parser.add_argument("--dry-run", action="store_true", help="Estimate only. This is the default unless --write is passed.")
    parser.add_argument("--preview-manifest-only", action="store_true", help="Alias for dry-run manifest-only behavior.")
    parser.add_argument("--allow-large-output", action="store_true", help="Allow writes above --max-output-gb.")
    parser.add_argument("--max-output-gb", type=float, default=2.0)
    parser.add_argument("--target-size-gb", type=float, default=None, help="Advisory target recorded in the manifest.")
    parser.add_argument("--max-points-per-shard", type=int, default=2_000_000)
    parser.add_argument("--voxel-size", type=float, default=0.03)
    parser.add_argument("--depth-max", type=float, default=8.0)
    parser.add_argument("--depth-min", type=float, default=None)
    parser.add_argument("--keyframe-translation-m", type=float, default=0.15)
    parser.add_argument("--keyframe-rotation-deg", type=float, default=8.0)
    parser.add_argument("--estimation-pixel-stride", type=int, default=8)
    parser.add_argument("--estimation-voxel-key-cap", type=int, default=8_000_000)
    parser.add_argument("--pixel-stride", type=int, default=8, help="Debug-only mode; not geometry-preserving.")
    parser.add_argument("--chunk-rows", type=int, default=48)
    parser.add_argument("--diagnostic-frame-count", type=int, default=6)
    parser.add_argument("--diagnostic-pixel-stride", type=int, default=16)
    parser.add_argument("--diagnostic-max-points", type=int, default=250_000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_output_gb <= 0:
        raise ValueError("--max-output-gb must be positive")
    if args.max_points_per_shard <= 0:
        raise ValueError("--max-points-per-shard must be positive")
    if args.voxel_size <= 0:
        raise ValueError("--voxel-size must be positive")
    if args.depth_max <= 0:
        raise ValueError("--depth-max must be positive")
    if args.depth_min is not None and args.depth_min < 0:
        raise ValueError("--depth-min must be non-negative")
    if args.diagnostic_frame_count <= 0:
        raise ValueError("--diagnostic-frame-count must be positive")
    if args.diagnostic_pixel_stride <= 0:
        raise ValueError("--diagnostic-pixel-stride must be positive")
    if args.mode == "raw" and (not args.write or not args.allow_large_output):
        raise SystemExit("--mode raw requires both --write and --allow-large-output")

    dry_run = (not args.write) or args.dry_run or args.preview_manifest_only
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = _load_config(args.config)
    intr = _intrinsics_from_config(config)
    frames = _discover_frames(args.scene_root)
    selected_indices, selection_policy = _select_keyframes(
        frames,
        translation_m=args.keyframe_translation_m,
        rotation_deg=args.keyframe_rotation_deg,
    )
    warnings: List[str] = []
    alignment_checks = [
        _frame_alignment_stats(
            frames[idx],
            intr,
            depth_min=args.depth_min,
            depth_max=args.depth_max,
            allow_rgb_resize=args.allow_rgb_resize,
        )
        for idx in selected_indices[: max(1, min(len(selected_indices), args.diagnostic_frame_count))]
    ]
    convention_selection = _select_convention(args, frames, selected_indices, intr)
    diagnostic_outputs = _write_diagnostic_outputs(args, frames, selected_indices, intr, convention_selection)
    _write_pose_report(args.output_dir, convention_selection, diagnostic_outputs, alignment_checks)
    if args.mode == "pixel-stride":
        warnings.append("pixel-stride mode is for quick/debug output only and is not geometry-preserving.")

    raw_count = _count_raw_valid_points(
        frames,
        selected_indices,
        intr,
        depth_min=args.depth_min,
        depth_max=args.depth_max,
    )

    fused_count: Optional[int] = None
    if args.mode in {"voxel", "fused"}:
        fused_count, estimate_warnings = _estimate_fused_points(
            frames,
            selected_indices,
            intr,
            pose_convention=convention_selection.pose_convention,
            camera_convention=convention_selection.camera_convention,
            voxel_size=args.voxel_size,
            depth_min=args.depth_min,
            depth_max=args.depth_max,
            estimation_pixel_stride=max(1, args.estimation_pixel_stride),
            key_cap=args.estimation_voxel_key_cap,
            chunk_rows=args.chunk_rows,
            raw_count=raw_count,
        )
        warnings.extend(estimate_warnings)
        estimate_points = fused_count if fused_count is not None else raw_count
    elif args.mode == "pixel-stride":
        estimate_points = int(math.ceil(raw_count / float(max(1, args.pixel_stride))))
    else:
        estimate_points = raw_count

    estimated_bytes_per_point = BINARY_POINT_BYTES if args.write else ASCII_POINT_BYTES_ESTIMATE
    estimated_output_gb = float(estimate_points * estimated_bytes_per_point) / float(GB)
    output_files: List[dict] = []
    write_executed = False

    if args.write and estimated_output_gb > args.max_output_gb and not args.allow_large_output:
        warnings.append(
            f"Refused to write: estimated output {estimated_output_gb:.3f} GB exceeds "
            f"--max-output-gb {args.max_output_gb:.3f}. Pass --allow-large-output to override."
        )
    elif args.write and not dry_run:
        if args.mode in {"voxel", "fused"}:
            output_files, actual_count = _write_voxel_fused_shards(
                frames,
                selected_indices,
                intr,
                args.output_dir,
                pose_convention=convention_selection.pose_convention,
                camera_convention=convention_selection.camera_convention,
                voxel_size=args.voxel_size,
                max_points_per_shard=args.max_points_per_shard,
                depth_min=args.depth_min,
                depth_max=args.depth_max,
                chunk_rows=args.chunk_rows,
                allow_rgb_resize=args.allow_rgb_resize,
            )
            fused_count = actual_count
            estimate_points = actual_count
        elif args.mode == "raw":
            output_files, actual_count = _write_raw_shards(
                frames,
                selected_indices,
                intr,
                args.output_dir,
                pose_convention=convention_selection.pose_convention,
                camera_convention=convention_selection.camera_convention,
                max_points_per_shard=args.max_points_per_shard,
                depth_min=args.depth_min,
                depth_max=args.depth_max,
                chunk_rows=args.chunk_rows,
                allow_rgb_resize=args.allow_rgb_resize,
            )
            estimate_points = actual_count
        else:
            warnings.append("Write skipped: pixel-stride mode is estimate-only in this safe diagnostic tool.")
        write_executed = bool(output_files)
        if output_files:
            estimated_output_gb = sum(int(item.get("bytes", 0)) for item in output_files) / float(GB)
    else:
        warnings.append("Dry-run/manifest-only mode: no PLY files were written.")

    if not output_files and args.write and args.mode in {"voxel", "fused"} and estimated_output_gb <= args.max_output_gb:
        # This only happens if dry-run flags overrode --write.
        warnings.append("Write flag was present, but dry-run/preview mode prevented file output.")

    manifest = _build_manifest(
        args=args,
        scene_id=args.scene_root.name,
        intr=intr,
        frame_count=len(frames),
        selected_indices=selected_indices,
        selection_policy=selection_policy,
        raw_count=raw_count,
        fused_count=fused_count,
        estimated_output_gb=estimated_output_gb,
        output_files=output_files,
        write_executed=write_executed,
        warnings=warnings,
        convention_selection=convention_selection,
        diagnostic_outputs=diagnostic_outputs,
    )
    manifest_path = args.output_dir / "pointcloud_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps({
        "manifest": str(manifest_path),
        "write_executed": write_executed,
        "mode": args.mode,
        "input_frame_count": len(frames),
        "selected_keyframe_count": len(selected_indices),
        "estimated_raw_point_count": raw_count,
        "estimated_fused_point_count": fused_count,
        "estimated_output_size_gb": round(estimated_output_gb, 6),
        "output_files": output_files,
        "selected_pose_convention": convention_selection.pose_convention,
        "selected_camera_convention": convention_selection.camera_convention,
        "pose_convention_report": str(args.output_dir / "pose_convention_report.json"),
        "warnings": warnings,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
