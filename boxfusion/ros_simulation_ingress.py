from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import shlex
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from boxfusion.artifact_contract import ARTIFACT_PROFILE_CORE_ONLY
from boxfusion.backend_eval_scaffold import write_scene_manifest
from boxfusion.ros_publication_diagnostics_server import BoxFusionRosPublicationDiagnosticsBackend
from boxfusion.ros_query_server import BoxFusionRosQueryServerBackend
from boxfusion.runtime_export_coordinator import BoxFusionRuntimeExportCoordinator, ExportRefreshResult

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.utilities import remove_ros_args
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:  # pragma: no cover - exercised only in ROS-enabled environments
    rclpy = None
    Duration = None
    Node = None
    Time = None
    remove_ros_args = None
    PoseStamped = None
    Odometry = None
    CameraInfo = None
    Image = None
    Buffer = None
    TransformException = None
    TransformListener = None


DEFAULT_RGB_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/depth/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/color/camera_info"
DEFAULT_POSE_TOPIC = "/boxfusion/sim/pose"
DEFAULT_ODOM_TOPIC = ""
DEFAULT_OUTPUT_ROOT = Path("ros2_sim_ingress_runs")
DEFAULT_SEQUENCE_NAME = "gazebo_sim_sequence"
DEFAULT_SNAPSHOT_PERIOD_SEC = 1.0
DEFAULT_MAX_SYNC_SKEW_SEC = 0.15
DEFAULT_TARGET_SNAPSHOT_COUNT = 0
DEFAULT_TF_TARGET_FRAME = "map"
DEFAULT_TF_SOURCE_FRAME = ""
DEFAULT_TF_LOOKUP_TIMEOUT_SEC = 0.15
BACKEND_MODE_STUB = "stub"
BACKEND_MODE_REAL = "real"
BACKEND_MODES = (BACKEND_MODE_STUB, BACKEND_MODE_REAL)
REAL_BACKEND_HANDOFF_HM3D = "hm3d"
REAL_BACKEND_HANDOFF_CA1M = "ca1m"
REAL_BACKEND_HANDOFF_FORMATS = (REAL_BACKEND_HANDOFF_HM3D, REAL_BACKEND_HANDOFF_CA1M)
DEFAULT_REAL_BACKEND_HANDOFF_FORMAT = REAL_BACKEND_HANDOFF_HM3D
DEFAULT_REAL_BACKEND_OUTPUT_SUBDIR = "stage_a_real_backend_sequences"
DEFAULT_REAL_BACKEND_INPUT_SUBDIR = "stage_a_backend_inputs"
DEFAULT_REAL_BACKEND_SCRIPT = Path("stage_a_demo.py")
DEFAULT_REAL_BACKEND_MODEL_PATH = Path("./models/cutr_rgbd.pth")
DEFAULT_REAL_BACKEND_CLIP_PATH = Path("./models/ViT-B-32/open_clip_pytorch_model.bin")
DEFAULT_REAL_BACKEND_TEXT_FEATURES = Path("./data/class_features_small.pt")
DEFAULT_REAL_BACKEND_DEVICE = "cpu"
DEFAULT_REAL_BACKEND_TIMEOUT_SEC = 0.0


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _stamp_to_sec(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None:
        return None
    return float(sec) + float(nanosec or 0) * 1.0e-9


def _message_stamp_sec(message: Any) -> Optional[float]:
    return _stamp_to_sec(getattr(getattr(message, "header", None), "stamp", None))


def _message_frame_id(message: Any) -> Optional[str]:
    return _clean_optional_text(getattr(getattr(message, "header", None), "frame_id", None))


def _target_snapshot_reached(*, target_snapshot_count: int, recorded_snapshot_count: int) -> bool:
    return int(target_snapshot_count) > 0 and int(recorded_snapshot_count) >= int(target_snapshot_count)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line:
                rows.append(dict(json.loads(line)))
    return rows


def _relative_or_absolute(path: Optional[Path], root: Path) -> Optional[str]:
    if path is None:
        return None
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def _normalize_backend_mode(value: Any) -> str:
    mode = str(value or BACKEND_MODE_STUB).strip().lower()
    if mode not in BACKEND_MODES:
        raise ValueError(f"Unsupported simulation ingress backend mode: {mode!r}. Expected one of {BACKEND_MODES}.")
    return mode


def _normalize_real_backend_handoff_format(value: Any) -> str:
    handoff_format = str(value or DEFAULT_REAL_BACKEND_HANDOFF_FORMAT).strip().lower()
    if handoff_format not in REAL_BACKEND_HANDOFF_FORMATS:
        raise ValueError(
            "Unsupported simulation ingress real backend handoff format: "
            f"{handoff_format!r}. Expected one of {REAL_BACKEND_HANDOFF_FORMATS}."
        )
    return handoff_format


def _tail_text(value: Optional[str], *, max_chars: int = 4000) -> str:
    text = str(value or "")
    if len(text) <= max_chars:
        return text
    return text[-max_chars:]


def _quaternion_xyzw_to_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if norm <= 1.0e-12:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _pose_record_to_matrix(pose: Dict[str, Any]) -> List[List[float]]:
    qx, qy, qz, qw = [float(value) for value in list(pose.get("orientation_xyzw") or [0.0, 0.0, 0.0, 1.0])[:4]]
    rot = _quaternion_xyzw_to_matrix(qx, qy, qz, qw)
    return [
        [rot[0][0], rot[0][1], rot[0][2], float(pose.get("x", 0.0) or 0.0)],
        [rot[1][0], rot[1][1], rot[1][2], float(pose.get("y", 0.0) or 0.0)],
        [rot[2][0], rot[2][1], rot[2][2], float(pose.get("z", 0.0) or 0.0)],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _hm3d_habitat_pose_from_opencv_zup_pose(pose_matrix: Any) -> Any:
    import numpy as np

    opencv_zup_pose = np.asarray(pose_matrix, dtype=np.float32).reshape(4, 4)
    habitat_to_opencv_camera = np.eye(4, dtype=np.float32)
    habitat_to_opencv_camera[1, 1] = -1.0
    habitat_to_opencv_camera[2, 2] = -1.0
    habitat_yup_to_zup_world = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    return np.linalg.inv(habitat_yup_to_zup_world) @ opencv_zup_pose @ np.linalg.inv(habitat_to_opencv_camera)


def _simulation_pose_from_transform(
    transform_message: Any,
    *,
    source: str = "tf",
) -> SimulationPoseInput:
    transform = getattr(transform_message, "transform", transform_message)
    translation = getattr(transform, "translation", None)
    rotation = getattr(transform, "rotation", None)
    return SimulationPoseInput(
        x=float(getattr(translation, "x", 0.0) or 0.0),
        y=float(getattr(translation, "y", 0.0) or 0.0),
        z=float(getattr(translation, "z", 0.0) or 0.0),
        qx=float(getattr(rotation, "x", 0.0) or 0.0),
        qy=float(getattr(rotation, "y", 0.0) or 0.0),
        qz=float(getattr(rotation, "z", 0.0) or 0.0),
        qw=float(getattr(rotation, "w", 1.0) or 1.0),
        stamp_sec=_message_stamp_sec(transform_message),
        frame_id=_message_frame_id(transform_message),
        source=source,
    )


def _dependency_available(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


@dataclass
class SimulationImageInput:
    width: int
    height: int
    encoding: str
    step: int
    stamp_sec: Optional[float] = None
    frame_id: Optional[str] = None
    data: bytes = b""

    @classmethod
    def from_ros_image(cls, message: Any) -> "SimulationImageInput":
        return cls(
            width=int(getattr(message, "width", 0) or 0),
            height=int(getattr(message, "height", 0) or 0),
            encoding=str(getattr(message, "encoding", "") or ""),
            step=int(getattr(message, "step", 0) or 0),
            stamp_sec=_message_stamp_sec(message),
            frame_id=_message_frame_id(message),
            data=bytes(getattr(message, "data", b"") or b""),
        )


@dataclass
class SimulationCameraInfoInput:
    width: int
    height: int
    k: Sequence[float]
    d: Sequence[float]
    distortion_model: str = ""
    stamp_sec: Optional[float] = None
    frame_id: Optional[str] = None

    @classmethod
    def from_ros_camera_info(cls, message: Any) -> "SimulationCameraInfoInput":
        raw_k = getattr(message, "k", None)
        raw_d = getattr(message, "d", None)
        return cls(
            width=int(getattr(message, "width", 0) or 0),
            height=int(getattr(message, "height", 0) or 0),
            k=[] if raw_k is None else [float(value) for value in list(raw_k)],
            d=[] if raw_d is None else [float(value) for value in list(raw_d)],
            distortion_model=str(getattr(message, "distortion_model", "") or ""),
            stamp_sec=_message_stamp_sec(message),
            frame_id=_message_frame_id(message),
        )


@dataclass
class SimulationPoseInput:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_sec: Optional[float] = None
    frame_id: Optional[str] = None
    source: str = "pose"

    @classmethod
    def from_ros_pose_stamped(cls, message: Any, *, source: str = "pose") -> "SimulationPoseInput":
        pose = getattr(message, "pose", None)
        return cls.from_pose_like(pose, stamp_sec=_message_stamp_sec(message), frame_id=_message_frame_id(message), source=source)

    @classmethod
    def from_ros_odometry(cls, message: Any, *, source: str = "odom") -> "SimulationPoseInput":
        pose = getattr(getattr(message, "pose", None), "pose", None)
        return cls.from_pose_like(pose, stamp_sec=_message_stamp_sec(message), frame_id=_message_frame_id(message), source=source)

    @classmethod
    def from_pose_like(
        cls,
        pose: Any,
        *,
        stamp_sec: Optional[float] = None,
        frame_id: Optional[str] = None,
        source: str = "pose",
    ) -> "SimulationPoseInput":
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        return cls(
            x=float(getattr(position, "x", 0.0) or 0.0),
            y=float(getattr(position, "y", 0.0) or 0.0),
            z=float(getattr(position, "z", 0.0) or 0.0),
            qx=float(getattr(orientation, "x", 0.0) or 0.0),
            qy=float(getattr(orientation, "y", 0.0) or 0.0),
            qz=float(getattr(orientation, "z", 0.0) or 0.0),
            qw=float(getattr(orientation, "w", 1.0) or 1.0),
            stamp_sec=stamp_sec,
            frame_id=frame_id,
            source=source,
        )


class StageASimulationSnapshotWriter:
    """Write a narrow Stage-A-compatible scene root from periodic simulated RGB-D samples."""

    def __init__(
        self,
        *,
        output_root: Path = DEFAULT_OUTPUT_ROOT,
        sequence_name: str = DEFAULT_SEQUENCE_NAME,
        coordination_root: Optional[Path] = None,
        runtime_artifact_mode: str = "benchmark",
        store_image_payloads: bool = True,
        backend_mode: str = BACKEND_MODE_STUB,
        fallback_to_stub_on_real_backend_failure: bool = False,
        real_backend_handoff_format: str = DEFAULT_REAL_BACKEND_HANDOFF_FORMAT,
        real_backend_input_root: Optional[Path] = None,
        real_backend_output_root: Optional[Path] = None,
        real_backend_python: str = sys.executable,
        real_backend_script: Path = DEFAULT_REAL_BACKEND_SCRIPT,
        real_backend_model_path: Path = DEFAULT_REAL_BACKEND_MODEL_PATH,
        real_backend_clip_path: Path = DEFAULT_REAL_BACKEND_CLIP_PATH,
        real_backend_text_features: Path = DEFAULT_REAL_BACKEND_TEXT_FEATURES,
        real_backend_device: str = DEFAULT_REAL_BACKEND_DEVICE,
        real_backend_timeout_sec: float = DEFAULT_REAL_BACKEND_TIMEOUT_SEC,
        real_backend_command: Optional[str] = None,
        real_backend_extra_args: Optional[Sequence[str]] = None,
    ) -> None:
        self.output_root = Path(output_root).resolve()
        self.sequence_name = _clean_optional_text(sequence_name) or DEFAULT_SEQUENCE_NAME
        self.scene_root = self.output_root / "stage_a_sequences" / self.sequence_name
        self.coordination_root = Path(coordination_root).resolve() if coordination_root is not None else self.output_root / "coordination"
        self.runtime_artifact_mode = str(runtime_artifact_mode)
        self.store_image_payloads = bool(store_image_payloads)
        self.backend_mode = _normalize_backend_mode(backend_mode)
        self.fallback_to_stub_on_real_backend_failure = bool(fallback_to_stub_on_real_backend_failure)
        self.real_backend_handoff_format = _normalize_real_backend_handoff_format(real_backend_handoff_format)
        self.real_backend_input_root = (
            Path(real_backend_input_root).resolve()
            if real_backend_input_root is not None
            else self.output_root / DEFAULT_REAL_BACKEND_INPUT_SUBDIR / self.sequence_name
        )
        self.real_backend_output_root = (
            Path(real_backend_output_root).resolve()
            if real_backend_output_root is not None
            else self.output_root / DEFAULT_REAL_BACKEND_OUTPUT_SUBDIR
        )
        self.real_backend_scene_root = self.real_backend_output_root / self.sequence_name
        self.real_backend_python = str(real_backend_python or sys.executable)
        self.real_backend_script = Path(real_backend_script)
        self.real_backend_model_path = Path(real_backend_model_path)
        self.real_backend_clip_path = Path(real_backend_clip_path)
        self.real_backend_text_features = Path(real_backend_text_features)
        self.real_backend_device = str(real_backend_device or DEFAULT_REAL_BACKEND_DEVICE)
        self.real_backend_timeout_sec = float(real_backend_timeout_sec or 0.0)
        self.real_backend_command = _clean_optional_text(real_backend_command)
        self.real_backend_extra_args = [str(arg) for arg in list(real_backend_extra_args or [])]
        self.logs_dir = self.scene_root / "logs"
        self.frames_dir = self.scene_root / "frames"
        self.frame_log_path = self.logs_dir / "simulation_ingress_frames.jsonl"
        self.samples: List[Dict[str, Any]] = _load_jsonl(self.frame_log_path)
        self.latest_backend_handoff_result: Optional[Dict[str, Any]] = None

    def _next_frame_idx(self) -> int:
        if not self.samples:
            return 0
        return max(int(row.get("frame_idx", -1)) for row in self.samples) + 1

    def _persist_image_payload(self, *, frame_idx: int, suffix: str, image: SimulationImageInput) -> Optional[Path]:
        if not self.store_image_payloads:
            return None
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        path = self.frames_dir / f"frame_{frame_idx:06d}_{suffix}.bin"
        path.write_bytes(bytes(image.data or b""))
        return path

    def _image_record(self, *, frame_idx: int, suffix: str, image: SimulationImageInput) -> Dict[str, Any]:
        payload_path = self._persist_image_payload(frame_idx=frame_idx, suffix=suffix, image=image)
        return {
            "width": int(image.width),
            "height": int(image.height),
            "encoding": str(image.encoding),
            "step": int(image.step),
            "stamp_sec": image.stamp_sec,
            "frame_id": image.frame_id,
            "data_bytes": int(len(image.data or b"")),
            "payload_path": _relative_or_absolute(payload_path, self.scene_root),
        }

    def record_sample(
        self,
        *,
        rgb: SimulationImageInput,
        depth: SimulationImageInput,
        pose: SimulationPoseInput,
        camera_info: Optional[SimulationCameraInfoInput] = None,
        refresh_coordinator: bool = True,
    ) -> Tuple[Dict[str, Any], Optional[ExportRefreshResult]]:
        frame_idx = self._next_frame_idx()
        sample = {
            "frame_idx": int(frame_idx),
            "captured_at_utc": _utc_now_iso(),
            "rgb": self._image_record(frame_idx=frame_idx, suffix="rgb", image=rgb),
            "depth": self._image_record(frame_idx=frame_idx, suffix="depth", image=depth),
            "camera_info": None
            if camera_info is None
            else {
                "width": int(camera_info.width),
                "height": int(camera_info.height),
                "k": [float(value) for value in camera_info.k],
                "d": [float(value) for value in camera_info.d],
                "distortion_model": str(camera_info.distortion_model),
                "stamp_sec": camera_info.stamp_sec,
                "frame_id": camera_info.frame_id,
            },
            "pose": {
                "x": float(pose.x),
                "y": float(pose.y),
                "z": float(pose.z),
                "orientation_xyzw": [float(pose.qx), float(pose.qy), float(pose.qz), float(pose.qw)],
                "stamp_sec": pose.stamp_sec,
                "frame_id": pose.frame_id,
                "source": str(pose.source),
            },
        }
        self.samples.append(sample)
        self.write_capture_sequence_log()
        refresh_result = self.refresh_current_backend() if refresh_coordinator else None
        return sample, refresh_result

    def write_capture_sequence_log(self) -> Path:
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        _write_jsonl(self.frame_log_path, self.samples)
        _write_json(
            self.logs_dir / "simulation_ingress_capture_summary.json",
            {
                "artifact_kind": "simulation_ingress_capture_sequence",
                "sequence_id": self.sequence_name,
                "backend_mode": self.backend_mode,
                "real_backend_handoff_format": self.real_backend_handoff_format,
                "sample_count": len(self.samples),
                "frames_jsonl": str(self.frame_log_path.relative_to(self.scene_root)),
                "real_backend_input_root": str(self.real_backend_input_root),
                "real_backend_scene_root": str(self.real_backend_scene_root),
            },
        )
        return self.frame_log_path

    def _pose_bounds(self) -> Dict[str, float]:
        poses = [dict(row.get("pose") or {}) for row in self.samples]
        xs = [float(pose.get("x", 0.0) or 0.0) for pose in poses] or [0.0]
        ys = [float(pose.get("y", 0.0) or 0.0) for pose in poses] or [0.0]
        zs = [float(pose.get("z", 0.0) or 0.0) for pose in poses] or [0.0]
        return {
            "min_x": min(xs),
            "max_x": max(xs),
            "min_y": min(ys),
            "max_y": max(ys),
            "min_z": min(zs),
            "max_z": max(zs),
        }

    def _build_room_polygon(self) -> List[List[float]]:
        bounds = self._pose_bounds()
        margin = 0.75
        min_x = bounds["min_x"] - margin
        max_x = bounds["max_x"] + margin
        min_y = bounds["min_y"] - margin
        max_y = bounds["max_y"] + margin
        if abs(max_x - min_x) < 0.5:
            max_x = min_x + 1.0
        if abs(max_y - min_y) < 0.5:
            max_y = min_y + 1.0
        return [
            [round(min_x, 3), round(min_y, 3)],
            [round(max_x, 3), round(min_y, 3)],
            [round(max_x, 3), round(max_y, 3)],
            [round(min_x, 3), round(max_y, 3)],
        ]

    def _build_topology_payload(self) -> Dict[str, Any]:
        polygon = self._build_room_polygon()
        center = [
            round(sum(point[0] for point in polygon) / float(len(polygon)), 3),
            round(sum(point[1] for point in polygon) / float(len(polygon)), 3),
        ]
        area = round(abs((polygon[1][0] - polygon[0][0]) * (polygon[2][1] - polygon[1][1])), 3)
        return {
            "version": "0.1",
            "sequence_id": self.sequence_name,
            "floors": [
                {
                    "floor_id": "floor_1",
                    "floor_index": 0,
                    "display_floor_id": "floor_1",
                    "display_order": 1,
                    "z_min": round(self._pose_bounds()["min_z"] - 0.25, 3),
                    "z_max": round(self._pose_bounds()["max_z"] + 0.25, 3),
                    "z_center": round((self._pose_bounds()["min_z"] + self._pose_bounds()["max_z"]) / 2.0, 3),
                    "confidence": 1.0,
                    "status": "confirmed",
                }
            ],
            "rooms": [
                {
                    "id": "room_1",
                    "room_uuid": 1,
                    "room_type": "unknown",
                    "floor_id": "floor_1",
                    "floor_index": 0,
                    "display_floor_id": "floor_1",
                    "display_order": 1,
                    "floor_assignment_confidence": 1.0,
                    "status": "confirmed",
                    "center": center,
                    "polygon": polygon,
                    "area_m2": area,
                }
            ],
            "edges": [],
            "indices": {
                "object_to_room": {},
                "anchor_to_room": {},
                "room_to_objects": {"room_1": []},
                "room_to_anchors": {"room_1": []},
            },
            "entities": {
                "objects": [],
                "anchors": [],
            },
            "evidences": [],
            "metadata": {
                "artifact_kind": "stage_a_simulation_ingress_snapshot",
                "source": "ros2_simulation_ingress_adapter",
                "topology_source": "single_committed_pose_envelope",
                "public_topology_meaning": "committed/published only",
                "gazebo_specific_semantics": False,
            },
            "query_examples": {
                "floor_count": 1,
                "room_count": 1,
                "edge_count": 0,
                "anchor_count": 0,
                "object_count": 0,
                "sample_room_ids": ["room_1"],
                "capabilities": {
                    "anchor_lookup_available": False,
                    "object_id_lookup_available": False,
                    "object_label_lookup_available": False,
                },
            },
        }

    def _build_final_vector_map_payload(self) -> Dict[str, Any]:
        topology = self._build_topology_payload()
        return {
            "artifact_kind": "simulation_ingress_final_vector_map_snapshot",
            "sequence_id": self.sequence_name,
            "generated_at_utc": _utc_now_iso(),
            "floors": list(topology["floors"]),
            "rooms": [{"id": 1, "floor_id": "floor_1", "polygon": topology["rooms"][0]["polygon"]}],
            "edges": [],
            "objects": [],
            "anchors": [],
            "simulated_rgbd_pose_sequence": {
                "frame_count": len(self.samples),
                "frames_jsonl": str(self.frame_log_path.relative_to(self.scene_root)),
            },
        }

    def _build_lifecycle_payload(self) -> Dict[str, Any]:
        frame_idx = max(0, len(self.samples) - 1)
        timestamp = None
        if self.samples:
            timestamp = dict(self.samples[-1].get("pose") or {}).get("stamp_sec")
        return {
            "version": "0.1",
            "artifact_kind": "online_topology_lifecycle_history",
            "artifact_surface": "lifecycle",
            "debug_only": True,
            "public_default": False,
            "non_public": True,
            "sequence_id": self.sequence_name,
            "frame_idx": frame_idx,
            "timestamp": timestamp,
            "active_room_id": "room_1",
            "active_floor_id": "floor_1",
            "active_floor_status": "stable",
            "summary": {
                "room_count": 1,
                "refresh_count": len(self.samples),
                "dirty_room_count": 0,
                "candidate_complete_room_count": 1,
                "committed_room_count": 1,
                "blocked_commit_room_count": 0,
                "trigger_count": len(self.samples),
                "trigger_counts": {"simulation_ingress_snapshot": len(self.samples)},
                "public_topology_export_succeeded": True,
            },
            "dirty_rooms": [],
            "candidate_complete_rooms": ["room_1"],
            "committed_rooms": ["room_1"],
            "blocked_commit_reasons": {},
            "rooms": [
                {
                    "room_id": "room_1",
                    "lifecycle_state": "committed",
                    "dirty": False,
                    "dirty_reasons": [],
                    "last_updated_frame_idx": frame_idx,
                    "last_updated_timestamp": timestamp,
                    "first_seen_frame_idx": 0,
                    "last_observed_frame_idx": frame_idx,
                    "last_departed_frame_idx": None,
                    "export_observation_count": len(self.samples),
                    "floor_id": "floor_1",
                    "floor_status": "confirmed",
                    "present_in_latest_export": True,
                    "room_signature_stability_count": len(self.samples),
                    "gateway_signature_stability_count": len(self.samples),
                    "containment_stability_count": len(self.samples),
                    "candidate_complete": True,
                    "candidate_readiness_score": max(2, len(self.samples)),
                    "candidate_complete_reasons": [
                        "floor_assignment_stable",
                        "simulation_ingress_snapshot_available",
                    ],
                    "candidate_block_reasons": [],
                    "commit_block_reasons": [],
                    "stable_refresh_opportunities_since_structural_delta": len(self.samples),
                    "last_structural_delta_frame_idx": 0,
                    "vertical_transition_statuses": [],
                    "vertical_transition_ids": [],
                    "tracking_summary": {"simulation_ingress_frame_count": len(self.samples)},
                    "trigger_counts": {"simulation_ingress_snapshot": len(self.samples)},
                    "recent_triggers": [
                        {
                            "frame_idx": frame_idx,
                            "timestamp": timestamp,
                            "room_id": "room_1",
                            "trigger": "simulation_ingress_snapshot",
                        }
                    ],
                }
            ],
            "refresh_history": [
                {
                    "frame_idx": frame_idx,
                    "timestamp": timestamp,
                    "reason": "periodic_simulation_ingress_snapshot",
                    "public_topology_export_succeeded": True,
                }
            ],
        }

    def _build_summary_payload(self) -> Dict[str, Any]:
        return {
            "sequence_id": self.sequence_name,
            "output_mode": "core_only",
            "artifact_profile": ARTIFACT_PROFILE_CORE_ONLY,
            "core_only_mode": True,
            "optional_demo_artifacts_enabled": False,
            "processed_frames": len(self.samples),
            "snapshot_count": len(self.samples),
            "segmentation_cycle_count": len(self.samples),
            "final_floor_count": 1,
            "final_room_count": 1,
            "final_object_count": 0,
            "final_anchor_count": 0,
            "final_vertical_transition_count": 0,
            "final_vector_map_path": str(self.logs_dir / "final_vector_map_snapshot.json"),
            "simulation_ingress": {
                "adapter": "ros2_simulation_ingress",
                "frames_jsonl": str(self.frame_log_path),
                "source_topics": {
                    "rgb": DEFAULT_RGB_TOPIC,
                    "depth": DEFAULT_DEPTH_TOPIC,
                    "camera_info": DEFAULT_CAMERA_INFO_TOPIC,
                    "pose_default": DEFAULT_POSE_TOPIC,
                },
                "synchronization": "latest-cache sampled by periodic timer",
            },
            "runtime_growth_summary": {"sample_count": len(self.samples)},
        }

    def _camera_intrinsics(self) -> Tuple[List[List[float]], int, int]:
        for sample in reversed(self.samples):
            camera_info = dict(sample.get("camera_info") or {})
            k_values = [float(value) for value in list(camera_info.get("k") or [])]
            if len(k_values) >= 9:
                width = int(camera_info.get("width") or dict(sample.get("rgb") or {}).get("width") or 1)
                height = int(camera_info.get("height") or dict(sample.get("rgb") or {}).get("height") or 1)
                return (
                    [
                        [k_values[0], k_values[1], k_values[2]],
                        [k_values[3], k_values[4], k_values[5]],
                        [k_values[6], k_values[7], k_values[8]],
                    ],
                    width,
                    height,
                )
        rgb = dict(self.samples[-1].get("rgb") or {}) if self.samples else {}
        width = max(1, int(rgb.get("width") or 1))
        height = max(1, int(rgb.get("height") or 1))
        focal = float(max(width, height))
        return (
            [
                [focal, 0.0, float(width - 1) / 2.0],
                [0.0, focal, float(height - 1) / 2.0],
                [0.0, 0.0, 1.0],
            ],
            width,
            height,
        )

    def _read_sample_payload_bytes(self, record: Dict[str, Any]) -> bytes:
        relative_path = _clean_optional_text(record.get("payload_path"))
        if relative_path is None:
            return b""
        path = Path(relative_path)
        if not path.is_absolute():
            path = self.scene_root / path
        if not path.exists():
            return b""
        return path.read_bytes()

    def _rgb_array_for_backend(self, record: Dict[str, Any]) -> Any:
        import numpy as np

        width = max(1, int(record.get("width") or 1))
        height = max(1, int(record.get("height") or 1))
        encoding = str(record.get("encoding") or "rgb8").lower()
        data = self._read_sample_payload_bytes(record)
        if encoding in {"rgb8", "bgr8"}:
            expected = height * width * 3
            if len(data) >= expected:
                array = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, width, 3)).copy()
            else:
                array = np.zeros((height, width, 3), dtype=np.uint8)
            if encoding == "bgr8":
                array = array[:, :, ::-1]
            return array
        if encoding in {"rgba8", "bgra8"}:
            expected = height * width * 4
            if len(data) >= expected:
                array = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, width, 4)).copy()[:, :, :3]
            else:
                array = np.zeros((height, width, 3), dtype=np.uint8)
            if encoding == "bgra8":
                array = array[:, :, ::-1]
            return array
        if encoding in {"mono8", "8uc1"}:
            expected = height * width
            if len(data) >= expected:
                mono = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, width)).copy()
            else:
                mono = np.zeros((height, width), dtype=np.uint8)
            return np.repeat(mono[:, :, None], 3, axis=2)
        return np.zeros((height, width, 3), dtype=np.uint8)

    def _depth_array_for_backend(self, record: Dict[str, Any]) -> Any:
        import numpy as np

        width = max(1, int(record.get("width") or 1))
        height = max(1, int(record.get("height") or 1))
        encoding = str(record.get("encoding") or "32fc1").lower()
        data = self._read_sample_payload_bytes(record)
        if encoding in {"32fc1", "32fc"}:
            expected = height * width * 4
            if len(data) >= expected:
                meters = np.frombuffer(data[:expected], dtype=np.float32).reshape((height, width)).copy()
            else:
                meters = np.zeros((height, width), dtype=np.float32)
            meters = np.nan_to_num(meters, nan=0.0, posinf=0.0, neginf=0.0)
            return np.clip(meters * 1000.0, 0.0, 65535.0).astype(np.uint16)
        if encoding in {"16uc1", "mono16"}:
            expected = height * width * 2
            if len(data) >= expected:
                return np.frombuffer(data[:expected], dtype=np.uint16).reshape((height, width)).copy()
            return np.zeros((height, width), dtype=np.uint16)
        if encoding in {"8uc1", "mono8"}:
            expected = height * width
            if len(data) >= expected:
                mono = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, width)).copy()
            else:
                mono = np.zeros((height, width), dtype=np.uint8)
            return mono.astype(np.uint16)
        return np.zeros((height, width), dtype=np.uint16)

    def _build_real_backend_config(
        self,
        *,
        handoff_format: str,
        intrinsics: Sequence[Sequence[float]],
        width: int,
        height: int,
    ) -> Dict[str, Any]:
        fx = float(intrinsics[0][0])
        fy = float(intrinsics[1][1])
        cx = float(intrinsics[0][2])
        cy = float(intrinsics[1][2])
        dataset = "hm3d" if handoff_format == REAL_BACKEND_HANDOFF_HM3D else "CA1M"
        return {
            "dataset": dataset,
            "data": {
                "datadir": str(self.real_backend_input_root),
                "start": 0,
                "output_dir": "./results",
                "gap": 1,
            },
            "cam": {
                "H": int(height),
                "W": int(width),
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "png_depth_scale": 1000.0,
            },
            "detection": {
                "score_thresh": 0.4,
                "uv_bound": True,
                "uv_bound_value": 0.9,
                "floor_mask": True,
                "floor_ratio": 15,
                "scale_box": 1.5,
            },
            "association": {
                "small_threshold": 0.2,
                "rotation_gap": 30,
                "translation_gap": 0.8,
            },
            "room_segmentation": {
                "tracking_iou_threshold": 0.3,
                "tracking_missed_cycles": 2,
                "tier2_enablement_mode": "off",
                "tier2_manual_enable": False,
                "tier2_central_roi": [0, 0, max(1, int(width)), max(1, int(height))],
                "tier2_small_region_area_ratio_thresh": 0.2,
                "tier2_newborn_age_thresh": 2,
                "tier2_cut_support_thresh": 0.25,
                "tier2_dominant_neighbor_ratio_thresh": 0.6,
                "tier2_cooldown_ttl": 300,
                "tier2_cooldown_seed_strength_thresh": 0.65,
                "tier2_cooldown_bbox_margin_px": 12,
                "tier2_cooldown_persistent_seed_frames": 2,
                "tier2_region_match_coverage_thresh": 0.6,
                "tier2_region_match_area_ratio_max": 2.5,
            },
            "box_fusion": {
                "use": True,
                "iters": 20,
                "pst_path": "./data/pst_1024_0.tiff",
                "pst_size": 1024,
                "random_opt": {
                    "center_init_size": 0.1,
                    "center_scaling_coefficient": 0.1,
                    "shape_init_size": 0.5,
                    "shape_scaling_coefficient": 0.5,
                },
                "check_valid": False,
                "nms_threshold": 0.1,
                "small_size": 0.5,
            },
            "vis": {
                "rerun": False,
                "show_class": False,
                "show_label": False,
                "trajectory": True,
            },
            "eval": False,
        }

    def materialize_real_backend_input_sequence(self) -> Dict[str, Any]:
        if not self.samples:
            raise RuntimeError("No simulation RGB-D + pose samples have been captured.")
        import cv2
        import numpy as np
        import yaml

        self.write_capture_sequence_log()
        rgb_dir = self.real_backend_input_root / "rgb"
        depth_dir = self.real_backend_input_root / "depth"
        pose_dir = self.real_backend_input_root / "pose"
        shutil.rmtree(rgb_dir, ignore_errors=True)
        shutil.rmtree(depth_dir, ignore_errors=True)
        shutil.rmtree(pose_dir, ignore_errors=True)
        rgb_dir.mkdir(parents=True, exist_ok=True)
        depth_dir.mkdir(parents=True, exist_ok=True)
        if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D:
            pose_dir.mkdir(parents=True, exist_ok=True)
        poses = []
        for idx, sample in enumerate(self.samples):
            rgb_array = self._rgb_array_for_backend(dict(sample.get("rgb") or {}))
            depth_array = self._depth_array_for_backend(dict(sample.get("depth") or {}))
            cv2.imwrite(str(rgb_dir / f"{idx}.png"), cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(depth_dir / f"{idx}.png"), depth_array)
            opencv_zup_pose = _pose_record_to_matrix(dict(sample.get("pose") or {}))
            poses.append(opencv_zup_pose)
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D:
                habitat_pose = _hm3d_habitat_pose_from_opencv_zup_pose(opencv_zup_pose)
                np.savetxt(pose_dir / f"{idx}.txt", habitat_pose, fmt="%.9g")
        intrinsics, width, height = self._camera_intrinsics()
        if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_CA1M:
            np.savetxt(self.real_backend_input_root / "K_rgb.txt", np.asarray(intrinsics, dtype=np.float32))
            np.savetxt(self.real_backend_input_root / "K_depth.txt", np.asarray(intrinsics, dtype=np.float32))
            np.save(self.real_backend_input_root / "all_poses.npy", np.asarray(poses, dtype=np.float32))
        config_name = (
            "boxfusion_hm3d_config.yaml"
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D
            else "boxfusion_ca1m_config.yaml"
        )
        config_path = self.real_backend_input_root / config_name
        config_path.write_text(
            yaml.safe_dump(
                self._build_real_backend_config(
                    handoff_format=self.real_backend_handoff_format,
                    intrinsics=intrinsics,
                    width=width,
                    height=height,
                ),
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        manifest = {
            "artifact_kind": "simulation_ingress_real_backend_input_sequence",
            "sequence_id": self.sequence_name,
            "handoff_format": self.real_backend_handoff_format,
            "producer_dataset_arg": "hm3d"
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D
            else "CA1M",
            "input_root": str(self.real_backend_input_root),
            "sample_count": len(self.samples),
            "rgb_dir": str(rgb_dir),
            "depth_dir": str(depth_dir),
            "pose_dir": str(pose_dir) if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D else None,
            "poses_npy": str(self.real_backend_input_root / "all_poses.npy")
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_CA1M
            else None,
            "k_rgb_txt": str(self.real_backend_input_root / "K_rgb.txt")
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_CA1M
            else None,
            "k_depth_txt": str(self.real_backend_input_root / "K_depth.txt")
            if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_CA1M
            else None,
            "config_path": str(config_path),
            "intrinsics": {
                "width": int(width),
                "height": int(height),
                "fx": float(intrinsics[0][0]),
                "fy": float(intrinsics[1][1]),
                "cx": float(intrinsics[0][2]),
                "cy": float(intrinsics[1][2]),
                "source": "latest CameraInfo K when available, otherwise image-size fallback",
            },
            "hm3d_pose_frame_conversion": None
            if self.real_backend_handoff_format != REAL_BACKEND_HANDOFF_HM3D
            else {
                "source_pose_contract": "simulation c2w pose treated as OpenCV camera axes in Z-up world, matching the prior CA1M bridge assumption",
                "written_pose_contract": "HM3D/Habitat c2w pose text, OpenGL camera axes in Y-up world",
                "stage_a_loader_conversion": "opencv_zup_pose = T_Yup2Zup @ pose_txt @ C_habitat2opencv",
                "materialization_inverse": "pose_txt = inv(T_Yup2Zup) @ simulation_opencv_zup_pose @ inv(C_habitat2opencv)",
            },
            "source_frames_jsonl": str(self.frame_log_path),
        }
        _write_json(self.real_backend_input_root / "simulation_ingress_backend_input_manifest.json", manifest)
        return manifest

    def _format_real_backend_command(self, config_path: Path) -> List[str]:
        if self.real_backend_command is not None:
            formatted = self.real_backend_command.format(
                input_root=str(self.real_backend_input_root),
                config_path=str(config_path),
                output_root=str(self.real_backend_output_root),
                scene_root=str(self.real_backend_scene_root),
                sequence_name=self.sequence_name,
                handoff_format=self.real_backend_handoff_format,
                dataset_path="hm3d" if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D else "CA1M",
            )
            return shlex.split(formatted)
        dataset_path = "hm3d" if self.real_backend_handoff_format == REAL_BACKEND_HANDOFF_HM3D else "CA1M"
        command = [
            self.real_backend_python,
            str(self.real_backend_script),
            dataset_path,
            "--model-path",
            str(self.real_backend_model_path),
            "--config",
            str(config_path),
            "--clip-path",
            str(self.real_backend_clip_path),
            "--clip-model-name",
            "ViT-B-32",
            "--text-features",
            str(self.real_backend_text_features),
            "--device",
            self.real_backend_device,
            "--max-frames",
            str(len(self.samples)),
            "--keyframe-gap",
            "1",
            "--room-seg-interval",
            "1",
            "--capture-stride",
            "1",
            "--runtime-profile-interval",
            "1",
            "--runtime-artifact-mode",
            self.runtime_artifact_mode,
            "--core-only",
            "--quiet",
            "--output-root",
            str(self.real_backend_output_root),
        ]
        command.extend(self.real_backend_extra_args)
        return command

    def run_real_backend_producer(self) -> Dict[str, Any]:
        handoff_input = self.materialize_real_backend_input_sequence()
        command = self._format_real_backend_command(Path(handoff_input["config_path"]))
        started_at = _utc_now_iso()
        result = subprocess.run(
            command,
            cwd=str(Path.cwd()),
            text=True,
            capture_output=True,
            timeout=None if self.real_backend_timeout_sec <= 0.0 else self.real_backend_timeout_sec,
        )
        completed_at = _utc_now_iso()
        handoff_result = {
            "artifact_kind": "simulation_ingress_real_backend_handoff_result",
            "sequence_id": self.sequence_name,
            "backend_mode": self.backend_mode,
            "real_backend_handoff_format": self.real_backend_handoff_format,
            "producer_command": " ".join(shlex.quote(part) for part in command),
            "producer_returncode": int(result.returncode),
            "producer_started_at_utc": started_at,
            "producer_completed_at_utc": completed_at,
            "producer_stdout_tail": _tail_text(result.stdout),
            "producer_stderr_tail": _tail_text(result.stderr),
            "input_manifest": handoff_input,
            "backend_scene_root": str(self.real_backend_scene_root),
            "backend_scene_root_exists": self.real_backend_scene_root.exists(),
            "backend_topology_exists": (self.real_backend_scene_root / "logs" / "topology_v0_1.json").exists(),
            "backend_lifecycle_exists": (self.real_backend_scene_root / "logs" / "online_topology_lifecycle_v0_1.json").exists(),
            "status": "succeeded" if result.returncode == 0 else "failed",
        }
        self.latest_backend_handoff_result = handoff_result
        _write_json(self.logs_dir / "simulation_ingress_backend_handoff.json", handoff_result)
        if self.real_backend_scene_root.exists():
            _write_json(self.real_backend_scene_root / "logs" / "simulation_ingress_backend_handoff.json", handoff_result)
        if result.returncode != 0:
            raise RuntimeError(
                "Real Stage A backend producer failed with return code "
                f"{result.returncode}; see {self.logs_dir / 'simulation_ingress_backend_handoff.json'}"
            )
        if not (self.real_backend_scene_root / "logs" / "topology_v0_1.json").exists():
            handoff_result["status"] = "failed"
            handoff_result["failure_reason"] = "producer_returned_zero_but_topology_missing"
            self.latest_backend_handoff_result = handoff_result
            _write_json(self.logs_dir / "simulation_ingress_backend_handoff.json", handoff_result)
            raise RuntimeError(f"Real Stage A backend producer did not produce topology: {self.real_backend_scene_root}")
        return handoff_result

    def write_stage_a_scene_root(self) -> Path:
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.write_capture_sequence_log()
        topology = self._build_topology_payload()
        _write_json(self.logs_dir / "topology_v0_1.json", topology)
        _write_json(
            self.logs_dir / "topology_query_report.json",
            {
                "floor_count": 1,
                "room_count": 1,
                "object_count": 0,
                "anchor_count": 0,
                "edge_count": 0,
                "source": "ros2_simulation_ingress_adapter",
            },
        )
        _write_json(self.logs_dir / "vertical_transition_evidence.json", {"summary": {"count": 0}, "transitions": []})
        _write_json(
            self.logs_dir / "floor_diagnostics_summary.json",
            {
                "sequence_id": self.sequence_name,
                "per_floor": [{"floor_id": "floor_1", "status": "confirmed", "source": "simulation_pose"}],
            },
        )
        _write_json(self.logs_dir / "runtime_growth_profile.json", {"records": [], "summary": {"sample_count": len(self.samples)}})
        (self.logs_dir / "runtime_growth_profile.csv").write_text("frame_idx,total_step_sec\n", encoding="utf-8")
        _write_json(self.logs_dir / "online_topology_lifecycle_v0_1.json", self._build_lifecycle_payload())
        _write_json(self.logs_dir / "working_topology_v0_1.json", {"artifact_kind": "working_topology_debug", "rooms": []})
        _write_json(
            self.logs_dir / "working_vs_committed_topology_report_v0_1.json",
            {"artifact_kind": "working_vs_committed_topology_report_debug", "deferred": True},
        )
        _write_json(
            self.logs_dir / "working_vs_committed_topology_timeline_v0_1.json",
            {"artifact_kind": "working_vs_committed_topology_timeline_debug", "rows": []},
        )
        (self.logs_dir / "working_vs_committed_topology_timeline_v0_1.md").write_text(
            "# Simulation Ingress Working/Committed Timeline\n\nDeferred for this thin ingress step.\n",
            encoding="utf-8",
        )
        _write_json(self.logs_dir / "final_vector_map_snapshot.json", self._build_final_vector_map_payload())
        _write_json(self.logs_dir / "summary.json", self._build_summary_payload())
        write_scene_manifest(self.scene_root, sequence_name=self.sequence_name)
        return self.scene_root

    def refresh_coordinator(self, *, source_scene_root: Optional[Path] = None) -> ExportRefreshResult:
        selected_scene_root = Path(source_scene_root).resolve() if source_scene_root is not None else self.scene_root
        coordinator = BoxFusionRuntimeExportCoordinator(
            source_scene_root=selected_scene_root,
            coordination_root=self.coordination_root,
            sequence_name=self.sequence_name,
            runtime_artifact_mode=self.runtime_artifact_mode,
            enable_sidecar_shadow_export=False,
        )
        return coordinator.refresh_once()

    def refresh_current_backend(self) -> ExportRefreshResult:
        if self.backend_mode == BACKEND_MODE_STUB:
            self.write_stage_a_scene_root()
            return self.refresh_coordinator(source_scene_root=self.scene_root)
        try:
            self.run_real_backend_producer()
            return self.refresh_coordinator(source_scene_root=self.real_backend_scene_root)
        except Exception:
            if not self.fallback_to_stub_on_real_backend_failure:
                raise
            fallback_note = dict(self.latest_backend_handoff_result or {})
            fallback_note["fallback_to_stub_on_real_backend_failure"] = True
            fallback_note["fallback_scene_root"] = str(self.scene_root)
            fallback_note["fallback_reason"] = "real_backend_producer_failed"
            self.latest_backend_handoff_result = fallback_note
            _write_json(self.logs_dir / "simulation_ingress_backend_handoff.json", fallback_note)
            self.write_stage_a_scene_root()
            return self.refresh_coordinator(source_scene_root=self.scene_root)


if Node is not None:  # pragma: no branch - definition-only split

    class BoxFusionSimulationIngressNode(Node):  # pragma: no cover - requires ROS runtime
        def __init__(self) -> None:
            super().__init__("boxfusion_simulation_ingress_node")
            self.declare_parameter("rgb_topic", DEFAULT_RGB_TOPIC)
            self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
            self.declare_parameter("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)
            self.declare_parameter("pose_topic", DEFAULT_POSE_TOPIC)
            self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
            self.declare_parameter("pose_source", "pose")
            self.declare_parameter("tf_target_frame", DEFAULT_TF_TARGET_FRAME)
            self.declare_parameter("tf_source_frame", DEFAULT_TF_SOURCE_FRAME)
            self.declare_parameter("tf_lookup_timeout_sec", DEFAULT_TF_LOOKUP_TIMEOUT_SEC)
            self.declare_parameter("output_root", str(DEFAULT_OUTPUT_ROOT))
            self.declare_parameter("coordination_root", str(DEFAULT_OUTPUT_ROOT / "coordination"))
            self.declare_parameter("sequence_name", DEFAULT_SEQUENCE_NAME)
            self.declare_parameter("snapshot_period_sec", DEFAULT_SNAPSHOT_PERIOD_SEC)
            self.declare_parameter("target_snapshot_count", DEFAULT_TARGET_SNAPSHOT_COUNT)
            self.declare_parameter("max_sync_skew_sec", DEFAULT_MAX_SYNC_SKEW_SEC)
            self.declare_parameter("store_image_payloads", True)
            self.declare_parameter("refresh_coordinator", True)
            self.declare_parameter("runtime_artifact_mode", "benchmark")
            self.declare_parameter("backend_mode", BACKEND_MODE_STUB)
            self.declare_parameter("fallback_to_stub_on_real_backend_failure", False)
            self.declare_parameter("real_backend_handoff_format", DEFAULT_REAL_BACKEND_HANDOFF_FORMAT)
            self.declare_parameter("real_backend_input_root", "")
            self.declare_parameter("real_backend_output_root", str(DEFAULT_OUTPUT_ROOT / DEFAULT_REAL_BACKEND_OUTPUT_SUBDIR))
            self.declare_parameter("real_backend_python", sys.executable)
            self.declare_parameter("real_backend_script", str(DEFAULT_REAL_BACKEND_SCRIPT))
            self.declare_parameter("real_backend_model_path", str(DEFAULT_REAL_BACKEND_MODEL_PATH))
            self.declare_parameter("real_backend_clip_path", str(DEFAULT_REAL_BACKEND_CLIP_PATH))
            self.declare_parameter("real_backend_text_features", str(DEFAULT_REAL_BACKEND_TEXT_FEATURES))
            self.declare_parameter("real_backend_device", DEFAULT_REAL_BACKEND_DEVICE)
            self.declare_parameter("real_backend_timeout_sec", DEFAULT_REAL_BACKEND_TIMEOUT_SEC)

            self.max_sync_skew_sec = float(self.get_parameter("max_sync_skew_sec").value)
            self.refresh_enabled = bool(self.get_parameter("refresh_coordinator").value)
            self.target_snapshot_count = max(0, int(self.get_parameter("target_snapshot_count").value))
            self.pose_source = str(self.get_parameter("pose_source").value or "pose").strip().lower()
            self.tf_target_frame = _clean_optional_text(self.get_parameter("tf_target_frame").value) or DEFAULT_TF_TARGET_FRAME
            self.tf_source_frame = _clean_optional_text(self.get_parameter("tf_source_frame").value)
            self.tf_lookup_timeout_sec = max(0.0, float(self.get_parameter("tf_lookup_timeout_sec").value))
            real_backend_input_root = _clean_optional_text(self.get_parameter("real_backend_input_root").value)
            self.writer = StageASimulationSnapshotWriter(
                output_root=Path(str(self.get_parameter("output_root").value)),
                coordination_root=Path(str(self.get_parameter("coordination_root").value)),
                sequence_name=str(self.get_parameter("sequence_name").value),
                runtime_artifact_mode=str(self.get_parameter("runtime_artifact_mode").value),
                store_image_payloads=bool(self.get_parameter("store_image_payloads").value),
                backend_mode=str(self.get_parameter("backend_mode").value),
                fallback_to_stub_on_real_backend_failure=bool(
                    self.get_parameter("fallback_to_stub_on_real_backend_failure").value
                ),
                real_backend_handoff_format=str(self.get_parameter("real_backend_handoff_format").value),
                real_backend_input_root=None if real_backend_input_root is None else Path(real_backend_input_root),
                real_backend_output_root=Path(str(self.get_parameter("real_backend_output_root").value)),
                real_backend_python=str(self.get_parameter("real_backend_python").value),
                real_backend_script=Path(str(self.get_parameter("real_backend_script").value)),
                real_backend_model_path=Path(str(self.get_parameter("real_backend_model_path").value)),
                real_backend_clip_path=Path(str(self.get_parameter("real_backend_clip_path").value)),
                real_backend_text_features=Path(str(self.get_parameter("real_backend_text_features").value)),
                real_backend_device=str(self.get_parameter("real_backend_device").value),
                real_backend_timeout_sec=float(self.get_parameter("real_backend_timeout_sec").value),
            )

            self.latest_rgb: Optional[Any] = None
            self.latest_depth: Optional[Any] = None
            self.latest_camera_info: Optional[Any] = None
            self.latest_pose: Optional[SimulationPoseInput] = None
            self.pending_pose: Optional[SimulationPoseInput] = None
            self.last_recorded_key: Optional[Tuple[Optional[float], Optional[float], Optional[float]]] = None
            self.shutdown_requested = False
            self.shutdown_timer = None
            self.tf_buffer: Optional[Any] = None
            self.tf_listener: Optional[Any] = None

            self.create_subscription(Image, str(self.get_parameter("rgb_topic").value), self._handle_rgb, 10)
            self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self._handle_depth, 10)
            camera_info_topic = _clean_optional_text(self.get_parameter("camera_info_topic").value)
            if camera_info_topic:
                self.create_subscription(CameraInfo, camera_info_topic, self._handle_camera_info, 10)
            if self.pose_source == "odom":
                odom_topic = _clean_optional_text(self.get_parameter("odom_topic").value) or "/odom"
                self.create_subscription(Odometry, odom_topic, self._handle_odom, 10)
            elif self.pose_source == "tf":
                if Buffer is None or TransformListener is None:
                    raise ImportError("tf2_ros is not installed; pose_source=tf is unavailable in this environment.")
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            else:
                self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self._handle_pose, 10)

            period = max(0.05, float(self.get_parameter("snapshot_period_sec").value))
            self.timer = self.create_timer(period, self._try_record_snapshot)
            self.get_logger().info(
                f"BoxFusion simulation ingress writing scene_root={self.writer.scene_root} "
                f"coordination_root={self.writer.coordination_root} backend_mode={self.writer.backend_mode} "
                f"real_backend_handoff_format={self.writer.real_backend_handoff_format} "
                f"pose_source={self.pose_source} target_snapshot_count={self.target_snapshot_count}"
            )

        def _handle_rgb(self, message: Any) -> None:
            self.latest_rgb = message

        def _handle_depth(self, message: Any) -> None:
            self.latest_depth = message

        def _handle_camera_info(self, message: Any) -> None:
            self.latest_camera_info = message

        def _handle_pose(self, message: Any) -> None:
            self.latest_pose = SimulationPoseInput.from_ros_pose_stamped(message, source="pose")

        def _handle_odom(self, message: Any) -> None:
            self.latest_pose = SimulationPoseInput.from_ros_odometry(message, source="odom")

        def _lookup_stamp_sec(self) -> Optional[float]:
            for message in (self.latest_rgb, self.latest_depth, self.latest_camera_info):
                stamp_sec = _message_stamp_sec(message)
                if stamp_sec is not None:
                    return stamp_sec
            return None

        def _resolve_tf_source_frame(self) -> Optional[str]:
            for candidate in (
                self.tf_source_frame,
                _message_frame_id(self.latest_rgb),
                _message_frame_id(self.latest_camera_info),
                _message_frame_id(self.latest_depth),
            ):
                text = _clean_optional_text(candidate)
                if text is not None:
                    return text
            return None

        def _lookup_tf_pose(self) -> Optional[SimulationPoseInput]:
            if self.tf_buffer is None or Duration is None or Time is None:
                return None
            source_frame = self._resolve_tf_source_frame()
            if source_frame is None:
                return None
            stamp_sec = self._lookup_stamp_sec()
            lookup_time = Time() if stamp_sec is None else Time(seconds=float(stamp_sec))
            timeout = Duration(seconds=float(self.tf_lookup_timeout_sec))
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.tf_target_frame,
                    source_frame,
                    lookup_time,
                    timeout=timeout,
                )
            except TransformException:
                return None
            pose = _simulation_pose_from_transform(transform, source="tf")
            pose.frame_id = self.tf_target_frame
            pose.stamp_sec = pose.stamp_sec if pose.stamp_sec is not None else stamp_sec
            return pose

        def _resolve_pose(self) -> Optional[SimulationPoseInput]:
            if self.pose_source == "tf":
                return self._lookup_tf_pose()
            return self.latest_pose

        def _sync_ready(self) -> bool:
            if self.latest_rgb is None or self.latest_depth is None:
                return False
            pose = self._resolve_pose()
            if pose is None:
                return False
            stamps = [
                _message_stamp_sec(self.latest_rgb),
                _message_stamp_sec(self.latest_depth),
                pose.stamp_sec,
            ]
            known = [stamp for stamp in stamps if stamp is not None]
            if len(known) >= 2 and max(known) - min(known) > self.max_sync_skew_sec:
                return False
            key = (stamps[0], stamps[1], stamps[2])
            if key == self.last_recorded_key:
                return False
            self.pending_pose = pose
            self.last_recorded_key = key
            return True

        def _request_shutdown_if_target_reached(self) -> None:
            if self.shutdown_requested:
                return
            if not _target_snapshot_reached(
                target_snapshot_count=self.target_snapshot_count,
                recorded_snapshot_count=len(self.writer.samples),
            ):
                return
            self.shutdown_requested = True
            self.timer.cancel()
            self.get_logger().info(
                f"Reached target_snapshot_count={self.target_snapshot_count}; stopping simulation ingress capture."
            )
            self.shutdown_timer = self.create_timer(0.05, self._shutdown_ros_once)

        def _shutdown_ros_once(self) -> None:
            if self.shutdown_timer is not None:
                self.shutdown_timer.cancel()
                self.shutdown_timer = None
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()

        def _try_record_snapshot(self) -> None:
            if not self._sync_ready():
                return
            camera_info = (
                None
                if self.latest_camera_info is None
                else SimulationCameraInfoInput.from_ros_camera_info(self.latest_camera_info)
            )
            pose = self.pending_pose or self._resolve_pose()
            self.pending_pose = None
            if pose is None:
                return
            next_snapshot_count = len(self.writer.samples) + 1
            refresh_now = bool(self.refresh_enabled)
            if refresh_now and self.target_snapshot_count > 0:
                refresh_now = _target_snapshot_reached(
                    target_snapshot_count=self.target_snapshot_count,
                    recorded_snapshot_count=next_snapshot_count,
                )
            sample, refresh_result = self.writer.record_sample(
                rgb=SimulationImageInput.from_ros_image(self.latest_rgb),
                depth=SimulationImageInput.from_ros_image(self.latest_depth),
                camera_info=camera_info,
                pose=pose,
                refresh_coordinator=refresh_now,
            )
            latest = None if refresh_result is None else str(refresh_result.latest_scene_root)
            self.get_logger().info(
                f"Recorded simulation ingress frame {sample['frame_idx']} "
                f"scene_root={self.writer.scene_root} latest={latest}"
            )
            self._request_shutdown_if_target_reached()

else:

    class BoxFusionSimulationIngressNode:  # pragma: no cover - import-time fallback
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise ImportError("rclpy is not installed; the ROS simulation ingress node cannot be created in this environment.")


def build_validation_report(
    *,
    output_root: Path,
    sequence_name: str,
    coordination_root: Optional[Path] = None,
    reset_output: bool = False,
    backend_mode: str = BACKEND_MODE_STUB,
    fallback_to_stub_on_real_backend_failure: bool = False,
    real_backend_handoff_format: str = DEFAULT_REAL_BACKEND_HANDOFF_FORMAT,
    real_backend_output_root: Optional[Path] = None,
    real_backend_python: str = sys.executable,
    real_backend_script: Path = DEFAULT_REAL_BACKEND_SCRIPT,
    real_backend_model_path: Path = DEFAULT_REAL_BACKEND_MODEL_PATH,
    real_backend_clip_path: Path = DEFAULT_REAL_BACKEND_CLIP_PATH,
    real_backend_text_features: Path = DEFAULT_REAL_BACKEND_TEXT_FEATURES,
    real_backend_device: str = DEFAULT_REAL_BACKEND_DEVICE,
    real_backend_timeout_sec: float = DEFAULT_REAL_BACKEND_TIMEOUT_SEC,
    real_backend_command: Optional[str] = None,
) -> Dict[str, Any]:
    backend_mode = _normalize_backend_mode(backend_mode)
    real_backend_handoff_format = _normalize_real_backend_handoff_format(real_backend_handoff_format)
    if reset_output:
        output_root = Path(output_root)
        target_scene_root = output_root / "stage_a_sequences" / sequence_name
        target_input_root = output_root / DEFAULT_REAL_BACKEND_INPUT_SUBDIR / sequence_name
        target_backend_root = (
            Path(real_backend_output_root) / sequence_name
            if real_backend_output_root is not None
            else output_root / DEFAULT_REAL_BACKEND_OUTPUT_SUBDIR / sequence_name
        )
        target_coordination_root = Path(coordination_root) if coordination_root is not None else output_root / "coordination"
        shutil.rmtree(target_scene_root, ignore_errors=True)
        shutil.rmtree(target_input_root, ignore_errors=True)
        shutil.rmtree(target_backend_root, ignore_errors=True)
        shutil.rmtree(target_coordination_root, ignore_errors=True)
    writer = StageASimulationSnapshotWriter(
        output_root=output_root,
        coordination_root=coordination_root,
        sequence_name=sequence_name,
        store_image_payloads=True,
        backend_mode=backend_mode,
        fallback_to_stub_on_real_backend_failure=fallback_to_stub_on_real_backend_failure,
        real_backend_handoff_format=real_backend_handoff_format,
        real_backend_output_root=real_backend_output_root,
        real_backend_python=real_backend_python,
        real_backend_script=real_backend_script,
        real_backend_model_path=real_backend_model_path,
        real_backend_clip_path=real_backend_clip_path,
        real_backend_text_features=real_backend_text_features,
        real_backend_device=real_backend_device,
        real_backend_timeout_sec=real_backend_timeout_sec,
        real_backend_command=real_backend_command,
    )
    environment_payload = {
        "python_executable": sys.executable,
        "python_version": sys.version.split()[0],
        "cwd": str(Path.cwd()),
        "CUDA_HOME": os.environ.get("CUDA_HOME"),
        "path_has_cuda_bin": "/usr/local/cuda/bin" in str(os.environ.get("PATH") or "").split(":"),
    }
    dependency_payload = {
        "torch_available": _dependency_available("torch"),
        "open_clip_available": _dependency_available("open_clip"),
        "cv2_available": _dependency_available("cv2"),
        "yaml_available": _dependency_available("yaml"),
        "numpy_available": _dependency_available("numpy"),
        "PIL_available": _dependency_available("PIL"),
        "stage_a_script_exists": Path(real_backend_script).exists(),
        "model_path_exists": Path(real_backend_model_path).exists(),
        "clip_path_exists": Path(real_backend_clip_path).exists(),
        "text_features_exists": Path(real_backend_text_features).exists(),
    }
    samples = [
        (0.0, 0.0, 0.0),
        (0.4, 0.0, 0.0),
        (0.8, 0.2, 0.0),
    ]
    refresh_result: Optional[ExportRefreshResult] = None
    producer_error: Optional[str] = None
    for idx, (x, y, z) in enumerate(samples):
        stamp_sec = float(idx) * 0.5
        _, refresh_result = writer.record_sample(
            rgb=SimulationImageInput(
                width=64,
                height=64,
                encoding="rgb8",
                step=192,
                stamp_sec=stamp_sec,
                frame_id="camera_color_optical_frame",
                data=bytes(([idx * 40, 20, 80] * 64 * 64)),
            ),
            depth=SimulationImageInput(
                width=64,
                height=64,
                encoding="32FC1",
                step=256,
                stamp_sec=stamp_sec,
                frame_id="camera_depth_optical_frame",
                data=(bytes([0, 0, 128, 63]) * 64 * 64),
            ),
            camera_info=SimulationCameraInfoInput(
                width=64,
                height=64,
                k=[60.0, 0.0, 32.0, 0.0, 60.0, 32.0, 0.0, 0.0, 1.0],
                d=[],
                distortion_model="plumb_bob",
                stamp_sec=stamp_sec,
                frame_id="camera_color_optical_frame",
            ),
            pose=SimulationPoseInput(
                x=x,
                y=y,
                z=z,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                stamp_sec=stamp_sec,
                frame_id="map",
                source="stub_pose_topic",
            ),
            refresh_coordinator=False,
        )
    try:
        refresh_result = writer.refresh_current_backend()
    except Exception as exc:
        producer_error = f"{type(exc).__name__}: {exc}"
        refresh_result = None
    if refresh_result is None:
        handoff_result = dict(writer.latest_backend_handoff_result or {})
        return {
            "validation_kind": f"ros2_simulation_ingress_{backend_mode}",
            "backend_mode": backend_mode,
            "real_backend_handoff_format": real_backend_handoff_format,
            "environment": environment_payload,
            "dependency_check": dependency_payload,
            "real_backend_export_produced": False,
            "producer_error": producer_error,
            "gazebo_check": {
                "gz_available": shutil.which("gz") is not None,
                "gazebo_available": shutil.which("gazebo") is not None,
                "used_gazebo_topics": False,
                "reason": "Gazebo and ROS 2 are not required for this local writer-level validation path.",
            },
            "ros2_check": {
                "ros2_cli_available": shutil.which("ros2") is not None,
                "rclpy_available": rclpy is not None,
                "used_ros2_topic_stub": False,
                "reason": "The validation uses the same adapter writer directly when rclpy is unavailable.",
            },
            "ingress": {
                "scene_root": str(writer.scene_root),
                "frames_jsonl": str(writer.frame_log_path),
                "sample_count": len(writer.samples),
                "real_backend_input_root": str(writer.real_backend_input_root),
                "real_backend_scene_root": str(writer.real_backend_scene_root),
            },
            "backend_handoff": handoff_result,
            "coordinator": {
                "latest_pointer_exists": False,
                "latest_scene_root": None,
                "latest_manifest_path": None,
                "latest_export_metadata_path": None,
            },
            "query_server_check": {"consumed_result": False, "reason": "No coordinator refresh was produced."},
            "publication_diagnostics_check": {"consumed_result": False, "reason": "No coordinator refresh was produced."},
        }

    query_backend = BoxFusionRosQueryServerBackend.from_bundle_path(refresh_result.latest_scene_root)
    diagnostics_backend = BoxFusionRosPublicationDiagnosticsBackend.from_bundle_path(refresh_result.latest_scene_root)
    topology_payload = dict(query_backend.get_topology().payload.get("payload") or {})
    world_payload = dict(query_backend.get_world_snapshot().payload.get("payload") or {})
    diagnostics_payload = dict(diagnostics_backend.get_publication_diagnostics().payload.get("payload") or {})
    latest_export_payload = json.loads(refresh_result.latest_export_metadata_path.read_text(encoding="utf-8"))
    return {
        "validation_kind": f"ros2_simulation_ingress_{backend_mode}",
        "backend_mode": backend_mode,
        "real_backend_handoff_format": real_backend_handoff_format,
        "environment": environment_payload,
        "dependency_check": dependency_payload,
        "real_backend_export_produced": backend_mode == BACKEND_MODE_REAL and refresh_result.source_scene_root == writer.real_backend_scene_root,
        "stub_fallback_used": backend_mode == BACKEND_MODE_REAL and refresh_result.source_scene_root == writer.scene_root,
        "gazebo_check": {
            "gz_available": shutil.which("gz") is not None,
            "gazebo_available": shutil.which("gazebo") is not None,
            "used_gazebo_topics": False,
            "reason": "Gazebo and ROS 2 are not required for this local writer-level validation path.",
        },
        "ros2_check": {
            "ros2_cli_available": shutil.which("ros2") is not None,
            "rclpy_available": rclpy is not None,
            "used_ros2_topic_stub": False,
            "reason": "The validation uses the same adapter writer directly when rclpy is unavailable.",
        },
        "ingress": {
            "scene_root": str(writer.scene_root),
            "frames_jsonl": str(writer.frame_log_path),
            "sample_count": len(writer.samples),
            "real_backend_input_root": str(writer.real_backend_input_root),
            "real_backend_scene_root": str(writer.real_backend_scene_root),
        },
        "backend_handoff": dict(writer.latest_backend_handoff_result or {}),
        "coordinator": {
            "latest_scene_root": str(refresh_result.latest_scene_root),
            "latest_manifest_path": str(refresh_result.latest_manifest_path),
            "latest_export_metadata_path": str(refresh_result.latest_export_metadata_path),
            "latest_pointer_exists": refresh_result.latest_scene_root.exists(),
            "latest_export_version": latest_export_payload.get("version"),
            "consumer_inputs": latest_export_payload.get("consumer_inputs"),
        },
        "query_server_check": {
            "consumed_result": True,
            "topology_path": topology_payload.get("topology_path"),
            "room_count": len(list(dict(topology_payload.get("topology") or {}).get("rooms") or [])),
            "world_snapshot_available": bool(world_payload.get("snapshot_available")),
        },
        "publication_diagnostics_check": {
            "consumed_result": True,
            "room_count": len(list(diagnostics_payload.get("rooms") or [])),
            "published_room_count": dict(diagnostics_payload.get("summary") or {}).get("published_room_count"),
            "public_topology_definition": diagnostics_payload.get("public_topology_definition"),
            "non_published_rooms_remain_internal": diagnostics_payload.get("non_published_rooms_remain_internal"),
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run or validate the thin ROS 2 simulation ingress adapter.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Workspace-local root for simulation ingress outputs.")
    parser.add_argument("--coordination-root", default=None, help="Optional coordinator root. Defaults under output-root/coordination.")
    parser.add_argument("--sequence-name", default=DEFAULT_SEQUENCE_NAME)
    parser.add_argument(
        "--backend-mode",
        choices=BACKEND_MODES,
        default=BACKEND_MODE_STUB,
        help="stub writes the existing minimal committed bundle; real runs the Stage A producer over the captured RGB-D + pose sequence.",
    )
    parser.add_argument(
        "--fallback-to-stub-on-real-backend-failure",
        action="store_true",
        help="In real mode, refresh the previous minimal committed stub if the Stage A producer fails.",
    )
    parser.add_argument(
        "--real-backend-handoff-format",
        choices=REAL_BACKEND_HANDOFF_FORMATS,
        default=DEFAULT_REAL_BACKEND_HANDOFF_FORMAT,
        help="Real producer input contract: hm3d is the preferred bridge; ca1m keeps the validated rollback bridge.",
    )
    parser.add_argument("--real-backend-output-root", default=None, help="Output root for real Stage A producer scene roots.")
    parser.add_argument("--real-backend-python", default=sys.executable, help="Python executable used for the real Stage A producer.")
    parser.add_argument("--real-backend-script", default=str(DEFAULT_REAL_BACKEND_SCRIPT), help="Path to stage_a_demo.py.")
    parser.add_argument("--real-backend-model-path", default=str(DEFAULT_REAL_BACKEND_MODEL_PATH))
    parser.add_argument("--real-backend-clip-path", default=str(DEFAULT_REAL_BACKEND_CLIP_PATH))
    parser.add_argument("--real-backend-text-features", default=str(DEFAULT_REAL_BACKEND_TEXT_FEATURES))
    parser.add_argument("--real-backend-device", default=DEFAULT_REAL_BACKEND_DEVICE)
    parser.add_argument(
        "--real-backend-timeout-sec",
        type=float,
        default=DEFAULT_REAL_BACKEND_TIMEOUT_SEC,
        help="Optional producer timeout. 0 disables the timeout.",
    )
    parser.add_argument("--validate-stub", action="store_true", help="Run the local RGB-D + pose stub validation and exit.")
    parser.add_argument("--validate-real-backend", action="store_true", help="Run the local RGB-D + pose validation through backend-mode=real and exit.")
    parser.add_argument(
        "--reset-output",
        action="store_true",
        help="For validation, clear only the selected sequence/input/backend roots and coordination root before writing samples.",
    )
    parser.add_argument("--json-out", default=None, help="Optional JSON report output path for validation.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    if argv is not None:
        args = build_arg_parser().parse_args(list(argv))
    else:
        raw_args = list(remove_ros_args(args=None)) if remove_ros_args is not None else None
        args = build_arg_parser().parse_args(None if raw_args is None else raw_args[1:])
    if args.validate_stub or args.validate_real_backend:
        validation_backend_mode = BACKEND_MODE_REAL if args.validate_real_backend else args.backend_mode
        report = build_validation_report(
            output_root=Path(args.output_root),
            coordination_root=None if args.coordination_root is None else Path(args.coordination_root),
            sequence_name=args.sequence_name,
            reset_output=bool(args.reset_output),
            backend_mode=validation_backend_mode,
            fallback_to_stub_on_real_backend_failure=bool(args.fallback_to_stub_on_real_backend_failure),
            real_backend_handoff_format=args.real_backend_handoff_format,
            real_backend_output_root=None if args.real_backend_output_root is None else Path(args.real_backend_output_root),
            real_backend_python=args.real_backend_python,
            real_backend_script=Path(args.real_backend_script),
            real_backend_model_path=Path(args.real_backend_model_path),
            real_backend_clip_path=Path(args.real_backend_clip_path),
            real_backend_text_features=Path(args.real_backend_text_features),
            real_backend_device=args.real_backend_device,
            real_backend_timeout_sec=float(args.real_backend_timeout_sec),
        )
        text = json.dumps(report, indent=2, sort_keys=True)
        if args.json_out:
            out_path = Path(args.json_out)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(text + "\n", encoding="utf-8")
        print(text)
        return 0
    if rclpy is None:
        raise ImportError("rclpy is not installed; pass --validate-stub for the non-ROS validation path.")
    rclpy.init(args=None)
    node = BoxFusionSimulationIngressNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())
