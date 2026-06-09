#!/usr/bin/env python3
"""RSLG-SLAM task24i object-level cross-floor visual tracking player."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from statistics import mean
from typing import Any

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from build_cross_floor_object_approach_route import (
    APPROACH_CANDIDATE_ID,
    APPROACH_NODE_ID,
    CLAIM_BOUNDARY,
    DEFAULT_OUTPUT_DIR,
    OBJECT_ID,
    OBJECT_LABEL,
    QUERY,
    ROOM_ID,
)
from cross_floor_visual_tracking_player import (
    DEFAULT_FALLBACK_CONTRACT,
    DEFAULT_PROFILE,
    DEFAULT_WORLD,
    TrackingConfig,
    TrackingNode,
    angle_wrap,
    clamp,
    color,
    dist3,
    interpolate_planned_yaw,
    load_planned_route,
    make_sphere_marker,
    make_text_marker,
    nearest_path_index,
    planned_distance_metrics,
    prepare_gazebo_world,
    prepare_visual_urdf,
    process_snapshot,
    simulate_tracking,
    validate_checkpoints,
    write_csv,
    write_json,
    write_text,
    yaw_between,
    z_and_stair_validation,
)
from cross_floor_visual_traversal_player import now_iso

try:
    from rclpy.qos import DurabilityPolicy
except ImportError:
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy


REPO_ROOT = Path("/home/ws/workspace/BoxFusion")
TASK_ROOT = REPO_ROOT / "stage_outputs/stage1_generalization/00843-DYehNKdT76V/tasks"
DEFAULT_PLANNED_OBJECT_ROUTE = DEFAULT_OUTPUT_DIR / "planned_object_level_3d_route_v0_1.json"

ENTITY_NAME = "rslg_cross_floor_object_quadruped_proxy_tracking"
BASE_FRAME = "rslg_cross_floor_object_tracking_base_link"
PLANNED_TOPIC = "/rslg/cross_floor_object_tracking_planned_route"
EXECUTED_TOPIC = "/rslg/cross_floor_object_tracking_executed_trajectory"
ERROR_TOPIC = "/rslg/cross_floor_object_tracking_error_markers"
CHECKPOINT_TOPIC = "/rslg/cross_floor_object_tracking_checkpoints"
TARGET_TOPIC = "/rslg/cross_floor_object_tracking_target"
CLAIM_TOPIC = "/rslg/cross_floor_object_tracking_claim_boundary"
ODOM_TOPIC = "/rslg/cross_floor_object_tracking/odom"

CLAIM_TEXT = "\n".join(
    [
        "task24i visual_kinematic_proxy_only",
        "object_level_smoke_test_only",
        "occupancy_aware_same_floor_tracking=true",
        "stair_connector_2p5d_tracking=true",
        "topological_vertical_transition_only",
        "physical_stair_climbing_supported=false",
        "object_centroid_navigation_used=false",
        "not Nav2 / not AMCL / not full object-navigation benchmark",
    ]
)


def object_target_from_route(route_raw: dict[str, Any]) -> dict[str, Any]:
    target = route_raw.get("object_target") or {}
    centroid = target.get("centroid_xy") or [None, None]
    z = float(target.get("z", 4.056))
    return {
        "object_id": target.get("object_id", OBJECT_ID),
        "label": target.get("label", OBJECT_LABEL),
        "room_id": target.get("room_id", ROOM_ID),
        "floor_id": target.get("floor_id", "floor_2"),
        "x": centroid[0],
        "y": centroid[1],
        "z": z,
    }


def approach_pose_from_route(route_raw: dict[str, Any]) -> dict[str, Any]:
    approach = route_raw.get("object_approach_candidate") or {}
    pose = dict(approach.get("approach_pose") or {})
    return {
        "approach_candidate_id": approach.get("approach_candidate_id", APPROACH_CANDIDATE_ID),
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", route_raw.get("object_target", {}).get("z", 4.056))),
        "yaw": float(pose.get("yaw", route_raw.get("object_approach_candidate", {}).get("target_facing_yaw_rad", 0.0))),
        "target_facing_yaw_rad": float(approach.get("target_facing_yaw_rad", pose.get("yaw", 0.0))),
    }


def simulate_object_tracking(route: Any, cfg: TrackingConfig, target_yaw: float, yaw_tolerance_rad: float) -> list[dict[str, Any]]:
    samples = simulate_tracking(route, cfg)
    if not samples:
        return samples
    dt = 1.0 / max(cfg.publish_rate_hz, 1.0)
    state = dict(samples[-1])
    elapsed = float(state.get("t_sec", 0.0))
    sample_index = int(state.get("sample_index", 0)) + 1
    max_steps = int(math.ceil(8.0 / dt))
    for _ in range(max_steps):
        yaw_error = angle_wrap(target_yaw - float(state["yaw"]))
        if abs(yaw_error) <= yaw_tolerance_rad:
            break
        omega = clamp(1.8 * yaw_error, -cfg.angular_speed_max_radps, cfg.angular_speed_max_radps)
        state["yaw"] = angle_wrap(float(state["yaw"]) + omega * dt)
        elapsed += dt
        sample = {
            **state,
            "sample_index": sample_index,
            "t_sec": round(elapsed, 6),
            "planned_nearest_index": len(route.points) - 1,
            "planned_target_index": len(route.points) - 1,
            "planned_nearest_s_m": round(route.cumulative_s[-1], 6),
            "lookahead_distance_m": 0.0,
            "cmd_linear_mps": 0.0,
            "cmd_angular_radps": float(omega),
            "cmd_vertical_mps": 0.0,
            "pose_source": "integrated_actual_pose_from_object_facing_yaw_repair",
            "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
            "phase": "object_facing_yaw_repair",
        }
        samples.append(sample)
        state = dict(sample)
        sample_index += 1
    return samples


class ObjectTrackingNode(TrackingNode):
    def __init__(self, args: argparse.Namespace, route: Any, cfg: TrackingConfig) -> None:
        self.object_target = object_target_from_route(route.raw)
        self.approach_pose = approach_pose_from_route(route.raw)
        super().__init__(args, route, cfg)
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(MarkerArray, args.target_marker_topic, qos)
        self.target_markers = self._target_markers()

    def _claim_markers(self) -> MarkerArray:
        msg = MarkerArray()
        xs = [p["x"] for p in self.route.points]
        ys = [p["y"] for p in self.route.points]
        zs = [p["z"] for p in self.route.points]
        pose = {"x": min(xs) - 0.7, "y": min(ys) - 0.7, "z": max(zs) + 0.9}
        msg.markers.append(make_text_marker(self.args.frame_id, "object_tracking_claim_boundary", 1, pose, CLAIM_TEXT, color(0.02, 0.02, 0.02, 1.0), 0.20))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _target_markers(self) -> MarkerArray:
        msg = MarkerArray()
        marker_id = 1
        approach = dict(self.approach_pose)
        msg.markers.append(make_sphere_marker(self.args.frame_id, "obj175_approach_generated_ring_037", marker_id, approach, color(0.85, 0.10, 0.78, 1.0), 0.32))
        marker_id += 1
        label_pose = dict(approach)
        label_pose["z"] = float(label_pose["z"]) + 0.42
        msg.markers.append(make_text_marker(self.args.frame_id, "obj175_approach_label", marker_id, label_pose, "obj_175 approach generated_ring_037", color(0.02, 0.02, 0.02, 1.0), 0.18))
        marker_id += 1
        if self.object_target.get("x") is not None and self.object_target.get("y") is not None:
            target = {
                "x": float(self.object_target["x"]),
                "y": float(self.object_target["y"]),
                "z": float(self.object_target["z"]),
            }
            msg.markers.append(make_sphere_marker(self.args.frame_id, "obj175_target_marker", marker_id, target, color(0.95, 0.05, 0.05, 1.0), 0.24))
            marker_id += 1
            target_label = dict(target)
            target_label["z"] += 0.36
            msg.markers.append(make_text_marker(self.args.frame_id, "obj175_target_label", marker_id, target_label, "obj_175 curtain", color(0.02, 0.02, 0.02, 1.0), 0.18))
        for marker in msg.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish_pose(self, pose: dict[str, Any]) -> None:
        super()._publish_pose(pose)
        self.target_pub.publish(self.target_markers)

    def run(self) -> dict[str, Any]:
        service_found = False
        if not self.args.dry_run:
            service_found = self._discover_service(self.args.service_timeout_sec)
            if service_found:
                self._query_entity()
        planned_executed = simulate_object_tracking(
            self.route,
            self.cfg,
            self.approach_pose["target_facing_yaw_rad"],
            self.args.object_yaw_tolerance_rad * 0.45,
        )
        start_wall = time.monotonic()
        prev_wall = start_wall
        for sample in planned_executed:
            if self.shutdown_requested or not rclpy.ok():
                self._record("shutdown_requested_during_tracking", sample_index=sample.get("sample_index"))
                break
            ok = self._update_gazebo(sample) if service_found else False
            sample["gazebo_set_entity_state_success"] = bool(ok)
            sample["entity_name"] = self.args.entity_name
            sample["frame_id"] = self.args.frame_id
            self.executed.append(sample)
            self._publish_pose(sample)
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.args.dry_run:
                now = time.monotonic()
                sleep = max(0.0, (1.0 / max(self.cfg.publish_rate_hz, 1.0)) - (now - prev_wall))
                sleep *= max(0.0, self.args.realtime_scale)
                if sleep > 0.0:
                    time.sleep(sleep)
                prev_wall = time.monotonic()
        hold_until = time.monotonic() + max(0.0, self.args.hold_final_sec)
        while not self.args.dry_run and rclpy.ok() and not self.shutdown_requested and self.executed and time.monotonic() < hold_until:
            self._publish_pose(self.executed[-1])
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self._record(
            "object_tracking_complete",
            service_found=service_found,
            service_name=self.service_name,
            executed_sample_count=len(self.executed),
            set_entity_success_count=self.set_entity_success_count,
            set_entity_failure_count=self.set_entity_failure_count,
            elapsed_wall_sec=round(time.monotonic() - start_wall, 3),
        )
        return {
            "service_found": service_found,
            "service_name": self.service_name,
            "get_entity_state_service_name": self.get_state_service_name,
            "entity_exists": self.entity_exists,
            "entity_state_message": self.entity_state_message,
            "set_entity_success_count": self.set_entity_success_count,
            "set_entity_failure_count": self.set_entity_failure_count,
            "executed_sample_count": len(self.executed),
            "shutdown_requested": self.shutdown_requested,
            "service_events": self.service_events,
        }


def approach_validation(route: Any, executed: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    approach = approach_pose_from_route(route.raw)
    final = executed[-1] if executed else {}
    final_distance = dist3(final, approach) if final else float("inf")
    yaw_error = abs(angle_wrap(float(final.get("yaw", 0.0)) - float(approach["target_facing_yaw_rad"]))) if final else float("inf")
    target = object_target_from_route(route.raw)
    object_segment = next(
        (
            segment
            for segment in reversed(route.raw.get("segments") or [])
            if segment.get("target_node_id") == APPROACH_NODE_ID or segment.get("approach_candidate_id") == APPROACH_CANDIDATE_ID
        ),
        {},
    )
    wall_check = object_segment.get("wall_obstacle_crossing_check_result") or {}
    terminal_check = object_segment.get("terminal_exact_candidate_append_check") or {}
    return {
        "artifact_type": "task24i_object_level_approach_validation",
        "created_utc": now_iso(),
        "query": QUERY,
        "object_id": OBJECT_ID,
        "approach_candidate_id": APPROACH_CANDIDATE_ID,
        "route_reaches_approach_pose": bool(final_distance <= args.final_approach_tolerance_m),
        "final_distance_to_approach_pose_m": None if not math.isfinite(final_distance) else round(final_distance, 6),
        "final_approach_tolerance_m": args.final_approach_tolerance_m,
        "final_yaw_error_rad": None if not math.isfinite(yaw_error) else round(yaw_error, 6),
        "yaw_tolerance_rad": args.object_yaw_tolerance_rad,
        "object_facing_yaw_available": True,
        "object_facing_yaw_passed": bool(yaw_error <= args.object_yaw_tolerance_rad),
        "target_facing_yaw_rad": approach["target_facing_yaw_rad"],
        "object_target_marker": target,
        "object_centroid_navigation_used": False,
        "generated_approach_candidate_used": True,
        "floor_2_approach_route_source": route.raw.get("route_checks", {}).get("floor_2_approach_route_source"),
        "no_wall_crossing_if_map_based_validation_available": bool(
            wall_check.get("wall_obstacle_crossing_check_passed")
            and terminal_check.get("wall_obstacle_crossing_check_passed", True)
        ),
        "object_segment_map_validation": {
            "planner_type": object_segment.get("planner_type"),
            "map_yaml_path": object_segment.get("map_yaml_path"),
            "wall_obstacle_crossing_check_result": wall_check,
            "terminal_exact_candidate_append_check": terminal_check,
        },
    }


def direct_playback_detection(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        "artifact_type": "task24i_object_level_direct_playback_detection",
        "created_utc": now_iso(),
        "direct_planned_pose_playback": False,
        "task24i_failed_direct_playback_detected": bool(metrics.get("task24h_failed_direct_playback_detected")),
        "mean_cross_track_error_m": metrics.get("mean_cross_track_error_m"),
        "max_cross_track_error_m": metrics.get("max_cross_track_error_m"),
        "threshold": metrics.get("direct_playback_detection_threshold"),
        "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
    }


def gazebo_validation(args: argparse.Namespace, runtime: dict[str, Any]) -> dict[str, Any]:
    return {
        "artifact_type": "task24i_object_level_gazebo_entity_state_validation",
        "created_utc": now_iso(),
        "entity_name": args.entity_name,
        "frame_id": args.frame_id,
        "set_entity_state_service_found": bool(runtime.get("service_found")),
        "set_entity_state_service_used": runtime.get("service_name"),
        "get_entity_state_service_used": runtime.get("get_entity_state_service_name"),
        "gazebo_visual_entity_exists": bool(runtime.get("entity_exists")),
        "entity_state_message": runtime.get("entity_state_message"),
        "set_entity_success_count": runtime.get("set_entity_success_count", 0),
        "set_entity_failure_count": runtime.get("set_entity_failure_count", 0),
        "gazebo_set_entity_state_service_used_successfully_if_available": bool(runtime.get("set_entity_success_count", 0) > 0),
        "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
        "set_entity_state_used_only_as_visual_display": True,
        "dry_run": bool(args.dry_run),
    }


def rviz_manifest(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "artifact_type": "task24i_object_level_rviz_topic_manifest",
        "created_utc": now_iso(),
        "topics": {
            "planned_route": args.planned_marker_topic,
            "executed_trajectory": args.executed_marker_topic,
            "tracking_error_markers": args.error_marker_topic,
            "checkpoints": args.checkpoint_marker_topic,
            "target": args.target_marker_topic,
            "claim_boundary": args.claim_marker_topic,
            "odom": args.odom_topic,
        },
        "rviz_auto_started": bool(args.rviz_auto_started),
        "automated_topic_publisher_available": True,
        "manual_gui_validation_required": not bool(args.rviz_auto_started),
        "manual_gui_validation_classification": "manual_gui_validation_required" if not args.rviz_auto_started else "rviz_auto_started",
    }


def classify(report: dict[str, Any]) -> str:
    if not report["object_query_resolution"]["validation"]["object_id_matches_expected"]:
        return "task24i_blocked_by_missing_object_resolution_or_candidate"
    if not report["object_approach_validation"]["route_reaches_approach_pose"]:
        return "task24i_blocked_by_object_approach_route_infeasible"
    if report["direct_playback_detection"]["task24i_failed_direct_playback_detected"]:
        return "task24i_blocked_by_tracking_direct_playback_detected"
    checkpoint_ok = bool(report["checkpoint_validation"]["all_semantic_checkpoints_passed"])
    object_checkpoint_ok = any(
        row.get("node_id") == APPROACH_NODE_ID and row.get("passed")
        for row in report["checkpoint_validation"]["checkpoints"]
    )
    stair_ok = bool(
        report["object_level_stair_transition_validation"]["transition_edge_is_vt_1_centerline_e001"]
        and report["object_level_stair_transition_validation"]["transition_edge_is_not_vt_1_centerline_e003"]
        and report["object_level_stair_transition_validation"]["z_rises_from_floor_1_to_floor_2"]
    )
    approach_ok = bool(
        report["object_approach_validation"]["route_reaches_approach_pose"]
        and report["object_approach_validation"]["object_facing_yaw_passed"]
    )
    gazebo_ok = bool(
        report["gazebo_entity_state_validation"]["set_entity_state_service_found"]
        and report["gazebo_entity_state_validation"]["gazebo_visual_entity_exists"]
        and report["gazebo_entity_state_validation"]["set_entity_success_count"] > 0
    )
    route_tracking_ok = checkpoint_ok and object_checkpoint_ok and stair_ok and approach_ok
    if route_tracking_ok and gazebo_ok:
        return "task24i_cross_floor_object_level_tracking_smoke_passed"
    if route_tracking_ok:
        return "task24i_route_ready_but_gazebo_runtime_blocked"
    return "task24i_blocked_by_object_approach_route_infeasible"


def write_outputs(args: argparse.Namespace, route: Any, cfg: TrackingConfig, executed: list[dict[str, Any]], runtime: dict[str, Any]) -> dict[str, Any]:
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    metrics = planned_distance_metrics(route, executed, cfg)
    metrics.update(
        {
            "artifact_type": "task24i_object_level_tracking_error_metrics",
            "tracking_error_metrics_nonzero_within_tolerance": bool(
                metrics.get("mean_cross_track_error_m") is not None
                and metrics["mean_cross_track_error_m"] > cfg.direct_playback_mean_threshold_m
                and metrics.get("max_cross_track_error_m") is not None
                and metrics["max_cross_track_error_m"] <= cfg.success_path_tolerance_m
            ),
        }
    )
    checkpoints = validate_checkpoints(route, executed, cfg)
    checkpoints.update(
        {
            "artifact_type": "task24i_object_level_checkpoint_validation",
            "object_approach_checkpoint_passed": any(
                row.get("node_id") == APPROACH_NODE_ID and row.get("passed")
                for row in checkpoints.get("checkpoints", [])
            ),
            "required_semantic_checkpoints": [
                "room_2",
                "room_3",
                "vt_1 connector",
                "room_7",
                "room_13",
                "room_14",
                APPROACH_NODE_ID,
            ],
        }
    )
    _zval, stair = z_and_stair_validation(route, executed)
    stair_validation = {
        "artifact_type": "task24i_object_level_stair_transition_validation",
        "created_utc": now_iso(),
        "transition_edge": route.source_summary.get("transition_edge"),
        "transition_edge_id": route.source_summary.get("transition_edge_id"),
        "transition_edge_is_vt_1_centerline_e001": route.source_summary.get("transition_edge_id") == "vt_1_centerline_e001",
        "transition_edge_is_not_vt_1_centerline_e003": route.source_summary.get("transition_edge_id") != "vt_1_centerline_e003",
        "z_rises_from_floor_1_to_floor_2": bool(stair.get("z_monotonic_non_decreasing_on_stair_samples") and stair.get("executed_stair_sample_count", 0) > 0),
        "stair_connector_tracking_mode": "stair_connector_2p5d_tracking",
        "stair_connector_tracking_summary": stair,
        "physical_stair_climbing_supported": False,
    }
    approach = approach_validation(route, executed, args)
    direct = direct_playback_detection(metrics)
    gazebo = gazebo_validation(args, runtime)
    rviz = rviz_manifest(args)
    ps = process_snapshot()

    report = {
        "artifact_type": "task24i_report",
        "created_utc": now_iso(),
        "classification": "pending",
        "project_name": "RSLG-SLAM",
        "query": QUERY,
        "object_query_resolution": route.raw.get("object_query_resolution"),
        "object_approach_candidate": route.raw.get("object_approach_candidate"),
        "route_source": {
            "base_route": route.raw.get("base_route_source"),
            "object_route": str(args.planned_route_json),
            "floor_2_approach_route_source": approach["floor_2_approach_route_source"],
        },
        "tracking_result": {
            "executed_sample_count": len(executed),
            "object_facing_yaw_repair_samples": sum(1 for row in executed if row.get("phase") == "object_facing_yaw_repair"),
            "direct_planned_pose_playback": False,
            "set_entity_state_input_source": "integrated_actual_pose_not_planned_pose_sample",
        },
        "tracking_error_metrics": metrics,
        "direct_playback_detection": direct,
        "checkpoint_validation": checkpoints,
        "object_approach_validation": approach,
        "object_level_stair_transition_validation": stair_validation,
        "gazebo_entity_state_validation": gazebo,
        "rviz_topic_manifest": rviz,
        "process_snapshot_summary": {
            "command": 'ps -ef | grep -E "nav2|amcl|bt_navigator|controller_server|planner_server|behavior_server|waypoint_follower" | grep -v grep || true',
            "no_nav2_amcl_process_detected": ps["no_nav2_process_detected"],
            "nav2_matching_lines": ps["nav2_matching_lines"],
        },
        "claim_boundary": CLAIM_BOUNDARY,
        "files_created_or_changed": [
            "tools/vertical_connectors/build_cross_floor_object_approach_route.py",
            "tools/vertical_connectors/cross_floor_object_tracking_player.py",
            "tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh",
            "tools/vertical_connectors/rviz_room2_to_obj175_cross_floor_object_tracking.rviz",
            "tools/vertical_connectors/README_room2_to_obj175_cross_floor_object_tracking_demo.md",
            str(out),
        ],
        "task25_interface_requirements_exposed": [
            "object query resolution must provide object_id, label, room_id, floor_id",
            "wall-like objects should provide standoff approach candidates, not centroid goals",
            "cross-floor route contract needs an appendable same-floor object approach segment",
            "tracking validation should separate approach position tolerance from object-facing yaw tolerance",
            "GUI evidence should publish target, approach, checkpoint, planned, executed, and claim boundary markers",
        ],
    }
    report["classification"] = classify(report)

    write_csv(out / "executed_object_level_tracking_trajectory.csv", executed)
    write_json(out / "executed_object_level_tracking_trajectory.json", {"artifact_type": "task24i_executed_object_level_tracking_trajectory", "created_utc": now_iso(), "samples": executed})
    write_json(out / "object_level_tracking_error_metrics.json", metrics)
    write_json(out / "object_level_direct_playback_detection.json", direct)
    write_json(out / "object_level_checkpoint_validation.json", checkpoints)
    write_json(out / "object_level_approach_validation.json", approach)
    write_json(out / "object_level_stair_transition_validation.json", stair_validation)
    write_json(out / "object_level_gazebo_entity_state_validation.json", gazebo)
    write_json(out / "object_level_rviz_topic_manifest.json", rviz)
    write_json(out / "object_level_claim_boundary.json", CLAIM_BOUNDARY)
    write_text(out / "set_entity_state_service_log.txt", "\n".join(json.dumps(item, sort_keys=True) for item in runtime.get("service_events", [])) + "\n")
    write_text(out / "process_snapshot.txt", ps["snapshot"])
    write_json(out / "task24i_report.json", report)
    write_text(out / "manual_gui_validation_instructions.md", manual_instructions())
    write_text(out / "task24i_report.md", report_markdown(report))
    write_text(out / "final_answer_for_user.md", final_answer_markdown(report))
    return report


def report_markdown(report: dict[str, Any]) -> str:
    approach = report["object_approach_validation"]
    candidate_pose = report["object_approach_candidate"]["approach_pose"]
    metrics = report["tracking_error_metrics"]
    stair = report["object_level_stair_transition_validation"]
    return f"""# task24i Cross-Floor Object-Level Tracking Smoke

Classification: `{report['classification']}`

## Object Resolution

- Query: `{QUERY}`
- Object: `obj_175` / `curtain`
- Target room/floor: `room_14` / `floor_2`
- Approach candidate: `generated_ring_037`
- Approach pose: `x={candidate_pose.get('x')}`, `y={candidate_pose.get('y')}`, `z={candidate_pose.get('z')}`, `yaw={candidate_pose.get('yaw')}`

## Route And Tracking

- Base route: `room_2 -> room_3 -> vt_1 / vc_vt_1 -> room_7 -> room_13 -> room_14`
- Object append source: `{report['route_source']['floor_2_approach_route_source']}`
- Final approach distance m: `{approach['final_distance_to_approach_pose_m']}`
- Final yaw error rad: `{approach['final_yaw_error_rad']}`
- Direct planned-pose playback: `False`
- SetEntityState source: `integrated_actual_pose_not_planned_pose_sample`
- Mean cross-track error m: `{metrics['mean_cross_track_error_m']}`
- Max cross-track error m: `{metrics['max_cross_track_error_m']}`

## Stair Boundary

- Transition edge: `{stair['transition_edge_id']}`
- Not vt_1_centerline_e003: `{stair['transition_edge_is_not_vt_1_centerline_e003']}`
- z rises floor_1 to floor_2: `{stair['z_rises_from_floor_1_to_floor_2']}`
- Tracking mode: `stair_connector_2p5d_tracking`

## Gazebo And RViz

- Gazebo entity: `{report['gazebo_entity_state_validation']['entity_name']}`
- SetEntityState successes: `{report['gazebo_entity_state_validation']['set_entity_success_count']}`
- RViz manual validation required: `{report['rviz_topic_manifest']['manual_gui_validation_required']}`

## Claim Boundary

`visual_kinematic_proxy_only`, `object_level_smoke_test_only`, `physical_stair_climbing_supported=false`, `object_centroid_navigation_used=false`, not Nav2 execution, not AMCL/localization, and not a full object-navigation benchmark.
"""


def manual_instructions() -> str:
    return """# task24i Manual GUI Validation Instructions

Run:

```bash
cd /home/ws/workspace/BoxFusion
tools/vertical_connectors/run_00843_room2_to_obj175_cross_floor_object_tracking_demo.sh --manual-demo
```

Expected RViz topics:

- /rslg/cross_floor_object_tracking_planned_route
- /rslg/cross_floor_object_tracking_executed_trajectory
- /rslg/cross_floor_object_tracking_checkpoints
- /rslg/cross_floor_object_tracking_target
- /rslg/cross_floor_object_tracking_claim_boundary

Confirm the route reaches the generated_ring_037 approach marker, not the obj_175 centroid marker. This remains visual_kinematic_proxy_only and does not claim physical stair climbing, gait, footstep planning, contact-based stair climbing, Nav2 execution, AMCL/localization, or full object-navigation benchmark coverage.
"""


def final_answer_markdown(report: dict[str, Any]) -> str:
    approach = report["object_approach_validation"]
    return f"""Classification: `{report['classification']}`

Resolved `{QUERY}` to `obj_175` / `curtain` in `room_14` on `floor_2`, using approach candidate `generated_ring_037`.

Route source: `{report['route_source']['floor_2_approach_route_source']}` appended to the existing room_2 -> room_14 cross-floor chain. Final approach distance is `{approach['final_distance_to_approach_pose_m']}` m; final yaw error is `{approach['final_yaw_error_rad']}` rad. Direct playback detected: `{report['direct_playback_detection']['task24i_failed_direct_playback_detected']}`.

Stair transition: `{report['object_level_stair_transition_validation']['transition_edge_id']}`, not `vt_1_centerline_e003`, with `stair_connector_2p5d_tracking`.
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planned-route-json", type=Path, default=DEFAULT_PLANNED_OBJECT_ROUTE)
    parser.add_argument("--fallback-route-contract", type=Path, default=DEFAULT_FALLBACK_CONTRACT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--entity-name", default=ENTITY_NAME)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--base-frame", default=BASE_FRAME)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--linear-speed-max", type=float, default=0.25)
    parser.add_argument("--angular-speed-max", type=float, default=0.55)
    parser.add_argument("--vertical-speed-limit", type=float, default=0.12)
    parser.add_argument("--lookahead-distance", type=float, default=0.35)
    parser.add_argument("--goal-tolerance", type=float, default=0.24)
    parser.add_argument("--checkpoint-tolerance-m", type=float, default=0.45)
    parser.add_argument("--final-tolerance-m", type=float, default=0.45)
    parser.add_argument("--final-approach-tolerance-m", type=float, default=0.45)
    parser.add_argument("--object-yaw-tolerance-rad", type=float, default=0.70)
    parser.add_argument("--max-run-time-sec", type=float, default=220.0)
    parser.add_argument("--z-visual-scale", type=float, default=1.0)
    parser.add_argument("--pitch-along-slope", action="store_true")
    parser.add_argument("--service-timeout-sec", type=float, default=25.0)
    parser.add_argument("--hold-final-sec", type=float, default=5.0)
    parser.add_argument("--realtime-scale", type=float, default=1.0)
    parser.add_argument("--planned-marker-topic", default=PLANNED_TOPIC)
    parser.add_argument("--executed-marker-topic", default=EXECUTED_TOPIC)
    parser.add_argument("--error-marker-topic", default=ERROR_TOPIC)
    parser.add_argument("--checkpoint-marker-topic", default=CHECKPOINT_TOPIC)
    parser.add_argument("--target-marker-topic", default=TARGET_TOPIC)
    parser.add_argument("--claim-marker-topic", default=CLAIM_TOPIC)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rviz-auto-started", action="store_true")
    parser.add_argument("--command-label", default="")
    parser.add_argument("--robot-profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--prepare-urdf", type=Path)
    parser.add_argument("--prepare-world", type=Path)
    parser.add_argument("--source-world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--print-start-pose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = TrackingConfig(
        publish_rate_hz=args.publish_rate,
        linear_speed_max_mps=args.linear_speed_max,
        angular_speed_max_radps=args.angular_speed_max,
        vertical_speed_limit_mps=args.vertical_speed_limit,
        lookahead_distance_m=args.lookahead_distance,
        goal_tolerance_m=args.goal_tolerance,
        checkpoint_tolerance_m=args.checkpoint_tolerance_m,
        final_tolerance_m=args.final_tolerance_m,
        max_run_time_sec=args.max_run_time_sec,
        z_visual_scale=args.z_visual_scale,
        pitch_along_slope=args.pitch_along_slope,
    )
    try:
        route = load_planned_route(args.planned_route_json, args.fallback_route_contract, args.z_visual_scale)
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        report = {
            "artifact_type": "task24i_report",
            "created_utc": now_iso(),
            "classification": "task24i_blocked_by_object_approach_route_infeasible",
            "planned_route_json": str(args.planned_route_json),
            "error": str(exc),
            "claim_boundary": CLAIM_BOUNDARY,
        }
        write_json(args.output_dir / "task24i_report.json", report)
        write_text(args.output_dir / "task24i_report.md", f"# task24i Report\n\nClassification: `{report['classification']}`\n\nError: {exc}\n")
        return 2
    if args.print_start_pose:
        yaw = interpolate_planned_yaw(route, 0, 0.0) + cfg.initial_yaw_lag_rad
        start = route.points[0]
        print(f"{start['x']} {start['y']} {start['z']} {yaw}")
        return 0
    if args.prepare_urdf:
        prepare_visual_urdf(args.robot_profile, args.prepare_urdf, args.entity_name)
    if args.prepare_world:
        prepare_gazebo_world(args.source_world, args.prepare_world)
    if args.prepare_only:
        return 0

    rclpy.init(args=None)
    node = ObjectTrackingNode(args, route, cfg)

    def request_shutdown(_signum: int, _frame: Any) -> None:
        node.shutdown_requested = True

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    runtime: dict[str, Any] = {}
    try:
        runtime.update(node.run())
        executed = node.executed
    finally:
        try:
            runtime["service_events"] = node.service_events
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    report = write_outputs(args, route, cfg, executed, runtime)
    print(json.dumps({"classification": report["classification"], "output_dir": str(args.output_dir)}, sort_keys=True))
    return 0 if report["classification"] in {"task24i_cross_floor_object_level_tracking_smoke_passed", "task24i_route_ready_but_gazebo_runtime_blocked"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
