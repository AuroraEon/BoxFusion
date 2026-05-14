#!/usr/bin/env python3
"""Run a minimal Nav2 movement smoke test for Stage1 Step30P1."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


os.environ.setdefault("ROS_DOMAIN_ID", "84")
os.environ["PATH"] = "/usr/bin:/usr/local/bin:" + os.environ.get("PATH", "")

IMPORT_ERROR: str | None = None
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as exc:  # pragma: no cover
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STAGE_OUTPUT = REPO_ROOT / "stage_outputs/stage1_00824_step30p1"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def yaw_to_quat(yaw: float) -> dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def yaw_from_quat(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def status_name(status: int | None) -> str | None:
    if status is None or IMPORT_ERROR:
        return None
    return {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }.get(int(status), str(status))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Step30S2 Navigation Smoke Report",
        "",
        f"Created: `{payload.get('created_utc')}`",
        f"Smoke passed: `{payload.get('smoke_passed')}`",
        f"Robot moved distance: `{payload.get('robot_moved_distance_m')}` m",
        f"cmd_vel messages observed: `{payload.get('cmd_vel_message_count')}`",
        f"NavigateToPose status: `{payload.get('navigate', {}).get('status')}`",
    ]
    if payload.get("failure_layer"):
        lines.append(f"Failure layer: `{payload.get('failure_layer')}`")
    if payload.get("failure_reason"):
        lines.append(f"Failure reason: `{payload.get('failure_reason')}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class SmokeNode(Node):  # pragma: no cover - ROS runtime only
    def __init__(self) -> None:
        super().__init__("boxfusion_stage1_step30p1_smoke_nav")
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.feedback_count = 0
        self.cmd_vel_count = 0
        self.latest_odom: dict[str, Any] | None = None
        self.samples: list[dict[str, Any]] = []
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 20)

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = {"x": float(p.x), "y": float(p.y), "z": float(p.z), "yaw": yaw_from_quat(q), "frame_id": msg.header.frame_id or "odom"}

    def cmd_vel_cb(self, _msg: Twist) -> None:
        self.cmd_vel_count += 1

    def current_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.2))
            t = transform.transform.translation
            q = transform.transform.rotation
            return {"x": float(t.x), "y": float(t.y), "z": float(t.z), "yaw": yaw_from_quat(q), "frame_id": "map", "source": "/tf map->base_footprint"}
        except TransformException:
            if self.latest_odom:
                pose = dict(self.latest_odom)
                pose["frame_id"] = "map"
                pose["source"] = "/odom under static map->odom"
                return pose
            return None

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = yaw_to_quat(yaw)
        pose.pose.orientation.x = q["x"]
        pose.pose.orientation.y = q["y"]
        pose.pose.orientation.z = q["z"]
        pose.pose.orientation.w = q["w"]
        return pose

    def compute_plan(self, target: PoseStamped, timeout_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "path_pose_count": 0, "failure_reason": None}
        goal = ComputePathToPose.Goal()
        goal.pose = target
        goal.planner_id = "GridBased"
        send_future = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "ComputePathToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "ComputePathToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_future, timeout_sec=timeout_sec)
        wrapped = get_future.result()
        if wrapped is None:
            result["failure_reason"] = "ComputePathToPose result timeout"
            return result
        result["status"] = status_name(int(wrapped.status))
        result["success"] = int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED and len(wrapped.result.path.poses) > 0
        result["path_pose_count"] = len(wrapped.result.path.poses)
        if not result["success"]:
            result["failure_reason"] = f"ComputePathToPose returned {result['status']}"
        return result

    def navigate(self, target: PoseStamped, timeout_sec: float, sample_period_sec: float) -> dict[str, Any]:
        result = {"attempted": True, "goal_accepted": False, "success": False, "status": None, "failure_reason": None, "feedback_count": 0}
        goal = NavigateToPose.Goal()
        goal.pose = target
        send_future = self.nav_client.send_goal_async(goal, feedback_callback=lambda _msg: setattr(self, "feedback_count", self.feedback_count + 1))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None:
            result["failure_reason"] = "NavigateToPose send_goal timeout"
            return result
        if not handle.accepted:
            result["status"] = "rejected"
            result["failure_reason"] = "NavigateToPose goal rejected"
            return result
        result["goal_accepted"] = True
        get_future = handle.get_result_async()
        deadline = time.time() + timeout_sec
        next_sample = 0.0
        while rclpy.ok() and time.time() < deadline and not get_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() >= next_sample:
                pose = self.current_pose()
                if pose:
                    pose["time_wall_sec"] = time.time()
                    pose["sample_index"] = len(self.samples)
                    self.samples.append(pose)
                next_sample = time.time() + sample_period_sec
        if not get_future.done():
            try:
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
            except Exception:
                pass
            result["status"] = "timeout"
            result["failure_reason"] = f"NavigateToPose timed out after {timeout_sec}s"
            result["feedback_count"] = self.feedback_count
            return result
        wrapped = get_future.result()
        status = int(wrapped.status) if wrapped is not None else None
        result["status"] = status_name(status)
        result["success"] = status == GoalStatus.STATUS_SUCCEEDED
        if not result["success"]:
            result["failure_reason"] = f"NavigateToPose returned {result['status']}"
        result["feedback_count"] = self.feedback_count
        return result


def dist(a: dict[str, Any] | None, b: dict[str, Any] | None) -> float | None:
    if not a or not b:
        return None
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def run_smoke(args: argparse.Namespace, stage_output: Path) -> dict[str, Any]:
    if IMPORT_ERROR:
        return {"artifact_type": "step30s2_navigation_smoke_report", "created_utc": now_iso(), "smoke_passed": False, "failure_layer": "rclpy", "failure_reason": IMPORT_ERROR}
    rclpy.init(args=None)
    node = SmokeNode()
    try:
        deadline = time.time() + args.pose_timeout_sec
        start_pose = None
        while rclpy.ok() and time.time() < deadline and start_pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            start_pose = node.current_pose()
        if start_pose is None:
            return {"artifact_type": "step30s2_navigation_smoke_report", "created_utc": now_iso(), "smoke_passed": False, "failure_layer": "TF/odom", "failure_reason": "could not observe robot pose"}
        yaw = float(start_pose.get("yaw", -2.221151))
        target_x = float(args.goal_x) if args.goal_x is not None else float(start_pose["x"]) + math.cos(yaw) * args.distance_m
        target_y = float(args.goal_y) if args.goal_y is not None else float(start_pose["y"]) + math.sin(yaw) * args.distance_m
        target_yaw = float(args.goal_yaw) if args.goal_yaw is not None else yaw
        target = node.make_pose(target_x, target_y, target_yaw)
        plan_ready = node.plan_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        nav_ready = node.nav_client.wait_for_server(timeout_sec=args.server_timeout_sec)
        plan = {"attempted": False, "skipped_reason": "planner unavailable"}
        if plan_ready:
            plan = node.compute_plan(target, args.plan_timeout_sec)
        if not nav_ready:
            return {"artifact_type": "step30s2_navigation_smoke_report", "created_utc": now_iso(), "smoke_passed": False, "failure_layer": "Nav2 action", "failure_reason": "/navigate_to_pose unavailable", "start_pose": start_pose, "plan": plan}
        navigate = node.navigate(target, args.goal_timeout_sec, args.sample_period_sec)
        final_pose = node.current_pose()
        moved = dist(start_pose, final_pose)
        target_dist = dist(final_pose, {"x": target_x, "y": target_y})
        smoke_passed = bool(navigate.get("success") or (target_dist is not None and target_dist <= args.goal_tolerance_m))
        smoke_passed = smoke_passed and moved is not None and moved >= args.min_movement_m and node.cmd_vel_count > 0
        failure_layer = None
        failure_reason = None
        if not smoke_passed:
            if not plan.get("success") and plan.get("attempted"):
                failure_layer = "planner"
            elif not navigate.get("success"):
                failure_layer = "controller"
            elif not node.cmd_vel_count:
                failure_layer = "cmd_vel"
            else:
                failure_layer = "trajectory"
            failure_reason = navigate.get("failure_reason") or plan.get("failure_reason") or "movement/cmd_vel acceptance criteria not met"
        return {
            "artifact_type": "step30s2_navigation_smoke_report",
            "version": "v0_1",
            "created_utc": now_iso(),
            "stage_output_dir": stage_output.as_posix(),
            "localization_mode": "static_map_to_odom",
            "test_type": "short NavigateToPose movement",
            "direct_cmd_vel_published_by_harness": False,
            "start_pose": start_pose,
            "target_pose": {"x": target_x, "y": target_y, "yaw": target_yaw},
            "final_pose": final_pose,
            "robot_moved_distance_m": round(moved, 6) if moved is not None else None,
            "distance_to_target_m": round(target_dist, 6) if target_dist is not None else None,
            "cmd_vel_message_count": node.cmd_vel_count,
            "plan": plan,
            "navigate": navigate,
            "trajectory_sample_count": len(node.samples),
            "trajectory_samples": node.samples,
            "smoke_passed": smoke_passed,
            "failure_layer": failure_layer,
            "failure_reason": failure_reason,
        }
    finally:
        try:
            node.nav_client.destroy()
            node.plan_client.destroy()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage-output-dir", type=Path, default=DEFAULT_STAGE_OUTPUT)
    parser.add_argument("--distance-m", type=float, default=0.25)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-yaw", type=float)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.25)
    parser.add_argument("--min-movement-m", type=float, default=0.03)
    parser.add_argument("--pose-timeout-sec", type=float, default=12.0)
    parser.add_argument("--server-timeout-sec", type=float, default=20.0)
    parser.add_argument("--plan-timeout-sec", type=float, default=20.0)
    parser.add_argument("--goal-timeout-sec", type=float, default=60.0)
    parser.add_argument("--sample-period-sec", type=float, default=0.2)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args()
    stage_output = args.stage_output_dir.resolve()
    out_dir = stage_output / "post_restructure_validation"
    output_json = args.output_json or out_dir / "step30s2_navigation_smoke_report_v0_1.json"
    output_md = args.output_md or out_dir / "step30s2_navigation_smoke_report_v0_1.md"
    trajectory_json = out_dir / "step30s2_navigation_smoke_trajectory_v0_1.json"
    payload = run_smoke(args, stage_output)
    write_json(output_json, payload)
    write_md(output_md, payload)
    write_json(trajectory_json, {"artifact_type": "step30s2_navigation_smoke_trajectory", "created_utc": now_iso(), "samples": payload.get("trajectory_samples", [])})
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload.get("smoke_passed") else 1


if __name__ == "__main__":
    sys.exit(main())
