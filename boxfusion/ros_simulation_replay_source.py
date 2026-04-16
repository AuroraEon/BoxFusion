from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # pragma: no cover - exercised only in ROS-enabled environments
    rclpy = None
    PoseStamped = None
    Node = None
    remove_ros_args = None
    CameraInfo = None
    Image = None


DEFAULT_CAPTURE_LOG = ""
DEFAULT_RGB_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/depth/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/color/camera_info"
DEFAULT_POSE_TOPIC = "/boxfusion/sim/pose"
DEFAULT_FRAME_PERIOD_SEC = 1.0
DEFAULT_SHUTDOWN_GRACE_SEC = 1.5
DEFAULT_MAX_FRAMES = 0


def _clean_optional_text(value: Any) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if line:
                rows.append(dict(json.loads(line)))
    return rows


def _infer_scene_root(capture_log: Path) -> Path:
    if capture_log.parent.name == "logs":
        return capture_log.parent.parent.resolve()
    return capture_log.parent.resolve()


@dataclass
class ReplayFrame:
    frame_idx: int
    rgb: Dict[str, Any]
    depth: Dict[str, Any]
    camera_info: Optional[Dict[str, Any]]
    pose: Dict[str, Any]


@dataclass
class SimulationCaptureSequence:
    capture_log_path: Path
    scene_root: Path
    frames: List[ReplayFrame]

    def resolve_payload_path(self, payload_path: Any) -> Path:
        text = _clean_optional_text(payload_path)
        if text is None:
            raise FileNotFoundError(f"Capture payload path is missing in {self.capture_log_path}")
        path = Path(text)
        if not path.is_absolute():
            path = (self.scene_root / path).resolve()
        if not path.exists():
            raise FileNotFoundError(f"Capture payload does not exist: {path}")
        return path


def load_simulation_capture_sequence(capture_log_path: Path) -> SimulationCaptureSequence:
    resolved_log = Path(capture_log_path).resolve()
    if not resolved_log.exists():
        raise FileNotFoundError(f"Capture log does not exist: {resolved_log}")
    scene_root = _infer_scene_root(resolved_log)
    raw_rows = _load_jsonl(resolved_log)
    frames = [
        ReplayFrame(
            frame_idx=int(row.get("frame_idx", idx)),
            rgb=dict(row.get("rgb") or {}),
            depth=dict(row.get("depth") or {}),
            camera_info=None if row.get("camera_info") is None else dict(row.get("camera_info") or {}),
            pose=dict(row.get("pose") or {}),
        )
        for idx, row in enumerate(raw_rows)
    ]
    frames.sort(key=lambda frame: frame.frame_idx)
    return SimulationCaptureSequence(
        capture_log_path=resolved_log,
        scene_root=scene_root,
        frames=frames,
    )


if Node is not None:  # pragma: no branch - definition-only split

    class BoxFusionSimulationReplaySourceNode(Node):  # pragma: no cover - requires ROS runtime
        def __init__(
            self,
            *,
            capture_log: Optional[Path] = None,
            rgb_topic: str = DEFAULT_RGB_TOPIC,
            depth_topic: str = DEFAULT_DEPTH_TOPIC,
            camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC,
            pose_topic: str = DEFAULT_POSE_TOPIC,
            frame_period_sec: float = DEFAULT_FRAME_PERIOD_SEC,
            shutdown_grace_sec: float = DEFAULT_SHUTDOWN_GRACE_SEC,
            max_frames: int = DEFAULT_MAX_FRAMES,
            node_name: str = "boxfusion_simulation_replay_source_node",
        ) -> None:
            super().__init__(node_name)
            self.declare_parameter("capture_log", DEFAULT_CAPTURE_LOG if capture_log is None else str(capture_log))
            self.declare_parameter("rgb_topic", rgb_topic)
            self.declare_parameter("depth_topic", depth_topic)
            self.declare_parameter("camera_info_topic", camera_info_topic)
            self.declare_parameter("pose_topic", pose_topic)
            self.declare_parameter("frame_period_sec", float(frame_period_sec))
            self.declare_parameter("shutdown_grace_sec", float(shutdown_grace_sec))
            self.declare_parameter("max_frames", int(max_frames))

            configured_capture_log = _clean_optional_text(self.get_parameter("capture_log").value)
            if configured_capture_log is None:
                raise ValueError("capture_log is required for the simulation replay source.")
            self.sequence = load_simulation_capture_sequence(Path(configured_capture_log))

            configured_max_frames = max(0, int(self.get_parameter("max_frames").value))
            if configured_max_frames > 0:
                self.frames = list(self.sequence.frames[:configured_max_frames])
            else:
                self.frames = list(self.sequence.frames)
            if not self.frames:
                raise ValueError(f"No replay frames were found in capture log: {self.sequence.capture_log_path}")

            self.frame_period_sec = max(0.05, float(self.get_parameter("frame_period_sec").value))
            self.shutdown_grace_sec = max(0.05, float(self.get_parameter("shutdown_grace_sec").value))

            self.rgb_publisher = self.create_publisher(Image, str(self.get_parameter("rgb_topic").value), 10)
            self.depth_publisher = self.create_publisher(Image, str(self.get_parameter("depth_topic").value), 10)

            configured_camera_info_topic = _clean_optional_text(self.get_parameter("camera_info_topic").value)
            self.camera_info_publisher = (
                None if configured_camera_info_topic is None else self.create_publisher(CameraInfo, configured_camera_info_topic, 10)
            )
            self.pose_publisher = self.create_publisher(PoseStamped, str(self.get_parameter("pose_topic").value), 10)

            self._frame_cursor = 0
            self._shutdown_timer = None
            self._timer = self.create_timer(self.frame_period_sec, self._publish_next_frame)

            self.get_logger().info(
                f"Simulation replay source loaded {len(self.frames)} frame(s) from {self.sequence.capture_log_path} "
                f"scene_root={self.sequence.scene_root}"
            )

        def _build_image_message(self, record: Dict[str, Any], *, stamp: Any) -> Image:
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = _clean_optional_text(record.get("frame_id")) or ""
            message.width = int(record.get("width") or 0)
            message.height = int(record.get("height") or 0)
            message.encoding = str(record.get("encoding") or "")
            message.step = int(record.get("step") or 0)
            message.data = self.sequence.resolve_payload_path(record.get("payload_path")).read_bytes()
            return message

        def _build_camera_info_message(self, record: Dict[str, Any], *, stamp: Any) -> CameraInfo:
            message = CameraInfo()
            message.header.stamp = stamp
            message.header.frame_id = _clean_optional_text(record.get("frame_id")) or ""
            message.width = int(record.get("width") or 0)
            message.height = int(record.get("height") or 0)
            message.k = [float(value) for value in list(record.get("k") or [])[:9]]
            if len(message.k) < 9:
                message.k = (message.k + [0.0] * 9)[:9]
            message.d = [float(value) for value in list(record.get("d") or [])]
            message.distortion_model = str(record.get("distortion_model") or "")
            return message

        def _build_pose_message(self, record: Dict[str, Any], *, stamp: Any) -> PoseStamped:
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = _clean_optional_text(record.get("frame_id")) or "map"
            message.pose.position.x = float(record.get("x", 0.0) or 0.0)
            message.pose.position.y = float(record.get("y", 0.0) or 0.0)
            message.pose.position.z = float(record.get("z", 0.0) or 0.0)
            orientation = list(record.get("orientation_xyzw") or [])
            padded = (orientation + [0.0, 0.0, 0.0, 1.0])[:4]
            message.pose.orientation.x = float(padded[0] or 0.0)
            message.pose.orientation.y = float(padded[1] or 0.0)
            message.pose.orientation.z = float(padded[2] or 0.0)
            message.pose.orientation.w = float(padded[3] or 1.0)
            return message

        def _schedule_shutdown(self) -> None:
            if self._shutdown_timer is not None:
                return
            self.get_logger().info(
                f"Published all replay frames; waiting {self.shutdown_grace_sec:.2f}s before shutdown."
            )
            self._shutdown_timer = self.create_timer(self.shutdown_grace_sec, self._shutdown_once)

        def _shutdown_once(self) -> None:
            if self._shutdown_timer is not None:
                self._shutdown_timer.cancel()
            self.get_logger().info("Simulation replay source finished.")
            if rclpy.ok():
                rclpy.shutdown()

        def _publish_next_frame(self) -> None:
            if self._frame_cursor >= len(self.frames):
                self._timer.cancel()
                self._schedule_shutdown()
                return
            frame = self.frames[self._frame_cursor]
            stamp = self.get_clock().now().to_msg()

            self.rgb_publisher.publish(self._build_image_message(frame.rgb, stamp=stamp))
            self.depth_publisher.publish(self._build_image_message(frame.depth, stamp=stamp))
            if self.camera_info_publisher is not None and frame.camera_info is not None:
                self.camera_info_publisher.publish(self._build_camera_info_message(frame.camera_info, stamp=stamp))
            self.pose_publisher.publish(self._build_pose_message(frame.pose, stamp=stamp))

            self.get_logger().info(
                f"Published replay frame_idx={frame.frame_idx} "
                f"rgb={frame.rgb.get('encoding')} depth={frame.depth.get('encoding')} "
                f"pose_frame={frame.pose.get('frame_id')}"
            )
            self._frame_cursor += 1

else:

    class BoxFusionSimulationReplaySourceNode:  # pragma: no cover - import-time fallback
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise ImportError("rclpy is not installed; the ROS simulation replay source node cannot be created.")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay a saved simulation_ingress capture log back onto ROS 2 RGB-D + pose topics."
    )
    parser.add_argument("--capture-log", default=None, help="Path to logs/simulation_ingress_frames.jsonl.")
    parser.add_argument("--rgb-topic", default=DEFAULT_RGB_TOPIC)
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    parser.add_argument("--frame-period-sec", type=float, default=DEFAULT_FRAME_PERIOD_SEC)
    parser.add_argument("--shutdown-grace-sec", type=float, default=DEFAULT_SHUTDOWN_GRACE_SEC)
    parser.add_argument("--max-frames", type=int, default=DEFAULT_MAX_FRAMES)
    parser.add_argument("--node-name", default="boxfusion_simulation_replay_source_node")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    if rclpy is None:
        raise ImportError("rclpy is not installed; install ROS 2 Python bindings to run the replay source node.")
    ros_args = None if argv is None else list(argv)
    raw_cli_args = list(remove_ros_args(args=ros_args)) if remove_ros_args is not None else list(argv or [])
    cli_args = raw_cli_args[1:] if argv is None else raw_cli_args
    args = build_arg_parser().parse_args(cli_args)

    rclpy.init(args=ros_args)
    node = BoxFusionSimulationReplaySourceNode(
        capture_log=None if args.capture_log is None else Path(args.capture_log),
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        pose_topic=args.pose_topic,
        frame_period_sec=float(args.frame_period_sec),
        shutdown_grace_sec=float(args.shutdown_grace_sec),
        max_frames=int(args.max_frames),
        node_name=args.node_name,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI wrapper
    raise SystemExit(main())
