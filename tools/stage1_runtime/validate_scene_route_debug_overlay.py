#!/usr/bin/env python3
"""Headless subscriber validation for the route-debug RViz MarkerArray."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import MarkerArray
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"

from scene_runtime_common import read_json, write_json


def marker_qos() -> Any:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MarkerCapture(Node):  # pragma: no cover - exercised by ROS validation
    def __init__(self, topic: str) -> None:
        super().__init__("boxfusion_scene_route_debug_overlay_validator")
        self.received: MarkerArray | None = None
        self.create_subscription(MarkerArray, topic, self.on_markers, marker_qos())

    def on_markers(self, msg: MarkerArray) -> None:
        self.received = msg


def analyze_markers(msg: MarkerArray | None, executable_waypoint_count: int) -> dict[str, Any]:
    markers = list(msg.markers) if msg is not None else []
    namespaces = sorted({marker.ns for marker in markers})
    namespace_counter = Counter(marker.ns for marker in markers)
    text = "\n".join(str(getattr(marker, "text", "")) for marker in markers if getattr(marker, "text", ""))
    executable_line_points = [
        len(marker.points)
        for marker in markers
        if marker.ns.endswith("/executable_route_line")
    ]
    marker_colors = {
        marker.ns: {
            "r": round(float(marker.color.r), 3),
            "g": round(float(marker.color.g), 3),
            "b": round(float(marker.color.b), 3),
            "a": round(float(marker.color.a), 3),
        }
        for marker in markers
        if float(marker.color.a) > 0.0
    }
    segment_03_colors = [marker_colors[marker.ns] for marker in markers if "/segment_03_" in marker.ns and marker.ns in marker_colors]
    segment_04_colors = [marker_colors[marker.ns] for marker in markers if "/segment_04_" in marker.ns and marker.ns in marker_colors]
    colors_present = {
        "sparse_anchor_line": any(ns.endswith("/sparse_anchor_line") for ns in marker_colors),
        "executable_route_line": any(ns.endswith("/executable_route_line") for ns in marker_colors),
        "executed_trajectory": any(ns.endswith("/executed_trajectory") for ns in marker_colors),
        "phase_trajectory": any("/phase_trajectory/" in ns for ns in marker_colors),
        "sent_controller_path": any("/sent_controller_path/" in ns for ns in marker_colors),
        "gateway_006_marker": any(ns.endswith("/gateway_006_marker") for ns in marker_colors),
        "gateway_006_halo": any(ns.endswith("/gateway_006_halo") for ns in marker_colors),
        "segment_03": bool(segment_03_colors),
        "segment_04": bool(segment_04_colors),
        "segment_03_04_different": bool(segment_03_colors and segment_04_colors and segment_03_colors[0] != segment_04_colors[0]),
    }
    result = {
        "marker_array_received": msg is not None,
        "marker_count": len(markers),
        "marker_namespaces": namespaces,
        "namespace_counts": dict(sorted(namespace_counter.items())),
        "marker_colors": marker_colors,
        "colors_present": colors_present,
        "contains_sparse_anchor_markers": any("sparse_anchor_points" in ns or "sparse_anchor_line" in ns for ns in namespaces),
        "contains_executable_route_markers": any("executable_route_line" in ns for ns in namespaces),
        "contains_segment_markers": any("/segment_00_" in ns for ns in namespaces) and any("/segment_05_" in ns for ns in namespaces),
        "contains_gateway_markers": any("gateway_markers" in ns or "gateway_006_marker" in ns for ns in namespaces),
        "contains_label_markers": any("labels" in ns for ns in namespaces),
        "contains_executed_trajectory_markers": any(ns.endswith("/executed_trajectory") for ns in namespaces),
        "contains_phase_trajectory_markers": any("/phase_trajectory/" in ns for ns in namespaces),
        "contains_slice0_phase_marker": any(ns.endswith("/phase_trajectory/slice0") for ns in namespaces),
        "contains_room13_dwell_post_phase_marker": any(ns.endswith("/phase_trajectory/room13_dwell_post") for ns in namespaces),
        "contains_slice1_phase_marker": any(ns.endswith("/phase_trajectory/slice1") for ns in namespaces),
        "contains_sent_controller_path_markers": any("/sent_controller_path/" in ns for ns in namespaces),
        "contains_handoff_path_marker": any("slice1a_handoff_staging_prefix_path" in ns for ns in namespaces),
        "contains_key_waypoint_labels": all(f"wp{idx}" in text for idx in (33, 39, 51, 58)),
        "contains_clean_status_label": all(key in text for key in ("clean_runtime_success", "sparse_fallback_used", "wall_crossing_validation_passed")),
        "contains_gateway_004_label": "gateway_004" in text or "00843_floor2_gateway_004" in text,
        "contains_gateway_006_label": "gateway_006" in text or "00843_floor2_gateway_006" in text,
        "contains_room13_label": "room13" in text or "room_13" in text,
        "contains_seg3_label": "seg3" in text,
        "contains_seg4_label": "seg4" in text,
        "contains_segment_3_marker": any("/segment_03_" in ns for ns in namespaces),
        "contains_segment_4_marker": any("/segment_04_" in ns for ns in namespaces),
        "executable_route_line_point_counts": executable_line_points,
        "executable_route_marker_has_expected_count": executable_waypoint_count in executable_line_points,
    }
    result["passed"] = all([
        result["marker_array_received"],
        result["marker_count"] > 0,
        result["contains_sparse_anchor_markers"],
        result["contains_executable_route_markers"],
        result["contains_segment_markers"],
        result["contains_gateway_markers"],
        result["contains_label_markers"],
        result["contains_gateway_004_label"],
        result["contains_gateway_006_label"],
        result["contains_room13_label"],
        result["contains_seg3_label"],
        result["contains_seg4_label"],
        result["colors_present"]["segment_03_04_different"],
        result["colors_present"]["gateway_006_marker"],
        result["executable_route_marker_has_expected_count"],
    ])
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--floor-id", required=True)
    parser.add_argument("--stage-output-dir", type=Path, required=True)
    parser.add_argument("--runtime-profile", type=Path)
    parser.add_argument("--semantic-waypoints-json", type=Path, required=True)
    parser.add_argument("--executable-waypoints-json", type=Path, required=True)
    parser.add_argument("--diagnostics-json", type=Path, required=True)
    parser.add_argument("--trajectory-json", type=Path)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--trajectory-trace-jsonl", type=Path)
    parser.add_argument("--sent-controller-paths-dir", type=Path)
    parser.add_argument("--route-execution-json", type=Path)
    parser.add_argument("--require-task-run-markers", action="store_true")
    parser.add_argument("--marker-topic", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if IMPORT_ERROR:
        payload = {"passed": False, "rclpy_import_error": IMPORT_ERROR}
        write_json(args.output_json, payload)
        print(json.dumps(payload, sort_keys=True))
        return 2

    semantic = read_json(args.semantic_waypoints_json).get("waypoints", [])
    executable = read_json(args.executable_waypoints_json).get("waypoints", [])
    diagnostics = read_json(args.diagnostics_json)
    source_counts = Counter(wp.get("source") for wp in executable)
    artifact_checks = {
        "semantic_waypoint_count": len(semantic),
        "executable_waypoint_count": len(executable),
        "source_counts": dict(sorted(source_counts.items())),
        "diagnostics_segment_count": len(diagnostics.get("segments", [])),
        "gateway_006_in_executable": any(wp.get("gateway_id") == "00843_floor2_gateway_006" for wp in executable),
        "segment_3_in_diagnostics": any(seg.get("segment_index") == 3 for seg in diagnostics.get("segments", [])),
        "segment_4_in_diagnostics": any(seg.get("segment_index") == 4 for seg in diagnostics.get("segments", [])),
    }

    publisher = Path(__file__).with_name("publish_scene_route_debug_overlay.py")
    command = [
        sys.executable,
        publisher.as_posix(),
        "--scene-id", args.scene_id,
        "--floor-id", args.floor_id,
        "--stage-output-dir", args.stage_output_dir.as_posix(),
        "--semantic-waypoints-json", args.semantic_waypoints_json.as_posix(),
        "--executable-waypoints-json", args.executable_waypoints_json.as_posix(),
        "--diagnostics-json", args.diagnostics_json.as_posix(),
        "--marker-topic", args.marker_topic,
        "--duration-sec", str(max(args.timeout_sec - 1.0, 3.0)),
        "--rate-hz", "2.0",
    ]
    if args.runtime_profile:
        command.extend(["--runtime-profile", args.runtime_profile.as_posix()])
    if args.trajectory_json:
        command.extend(["--trajectory-json", args.trajectory_json.as_posix()])
    if args.run_dir:
        command.extend(["--run-dir", args.run_dir.as_posix()])
    if args.trajectory_trace_jsonl:
        command.extend(["--trajectory-trace-jsonl", args.trajectory_trace_jsonl.as_posix()])
    if args.sent_controller_paths_dir:
        command.extend(["--sent-controller-paths-dir", args.sent_controller_paths_dir.as_posix()])
    if args.route_execution_json:
        command.extend(["--route-execution-json", args.route_execution_json.as_posix()])
    if args.require_task_run_markers:
        command.extend([
            "--show-sent-controller-paths",
            "--show-handoff-paths",
            "--show-success-trajectory",
            "--show-key-waypoints",
            "--show-phase-trajectory",
        ])

    rclpy.init(args=None)
    node = MarkerCapture(args.marker_topic)
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.received is not None:
                break
    finally:
        if proc.poll() is None:
            proc.terminate()
        try:
            stdout, stderr = proc.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, stderr = proc.communicate(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    marker_checks = analyze_markers(node.received, len(executable))
    task_run_marker_checks = {
        "required": bool(args.require_task_run_markers),
        "passed": True,
    }
    if args.require_task_run_markers:
        required_keys = [
            "contains_executed_trajectory_markers",
            "contains_phase_trajectory_markers",
            "contains_slice0_phase_marker",
            "contains_room13_dwell_post_phase_marker",
            "contains_slice1_phase_marker",
            "contains_sent_controller_path_markers",
            "contains_handoff_path_marker",
            "contains_key_waypoint_labels",
            "contains_clean_status_label",
        ]
        task_run_marker_checks["required_keys"] = required_keys
        task_run_marker_checks["passed"] = all(bool(marker_checks.get(key)) for key in required_keys)
    artifact_passed = all([
        artifact_checks["semantic_waypoint_count"] == 7,
        artifact_checks["executable_waypoint_count"] == 59,
        artifact_checks["source_counts"].get("bridge") == 52,
        artifact_checks["source_counts"].get("gateway") == 3,
        artifact_checks["source_counts"].get("room_center") == 4,
        artifact_checks["gateway_006_in_executable"],
        artifact_checks["segment_3_in_diagnostics"],
        artifact_checks["segment_4_in_diagnostics"],
    ])
    payload = {
        "artifact_checks": artifact_checks,
        "artifact_validation_passed": artifact_passed,
        "marker_checks": marker_checks,
        "task_run_marker_checks": task_run_marker_checks,
        "marker_topic": args.marker_topic,
        "publisher_command": command,
        "publisher_return_code": proc.returncode,
        "publisher_stdout_tail": stdout[-4000:],
        "publisher_stderr_tail": stderr[-4000:],
        "passed": bool(artifact_passed and marker_checks["passed"] and task_run_marker_checks["passed"]),
    }
    write_json(args.output_json, payload)
    print(json.dumps(payload, sort_keys=True))
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
