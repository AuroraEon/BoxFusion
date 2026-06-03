"""Approach stop/settle/remeasure/creep and yaw-facing alignment."""

from __future__ import annotations

import math
import time
from typing import Any

from .schemas import ControllerParams, angle_wrap, clamp, distance, now_iso


def run_approach_and_yaw(
    node,
    controller,
    candidate: dict[str, Any],
    proxy: dict[str, Any],
    planner,
    params: ControllerParams,
    dense_route_waypoints: list[dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Execute approach (settle/creep) and yaw alignment. Returns (approach_result, yaw_result)."""
    import rclpy

    current = node.pose()
    cand_dist = distance(current, candidate["world_xy"]) if current else None

    approach_result: dict[str, Any] = {
        "approach_position_tolerance_reached": False,
        "approach_controller_stop_tolerance_m": params.approach_controller_stop_tolerance_m,
        "formal_approach_position_tolerance_m": params.approach_position_tolerance_m,
        "final_creep_attempted": False,
        "final_creep_attempt_count": 0,
        "final_creep_distances_m": [],
        "controller_reported_distance_m": None,
        "post_settle_distance_m": None,
        "final_remeasured_distance_m": None,
        "final_pose_after_settle": None,
        "final_pose_after_creep": None,
    }

    if cand_dist is not None and cand_dist <= params.approach_position_tolerance_m:
        approach_result["approach_position_tolerance_reached"] = True
        approach_result["controller_reported_distance_m"] = round(cand_dist, 6)
        approach_result["post_settle_distance_m"] = round(cand_dist, 6)
        approach_result["final_remeasured_distance_m"] = round(cand_dist, 6)
        approach_result["final_distance_to_approach_candidate_m"] = round(cand_dist, 6)
        approach_result["final_pose_after_settle"] = current
    elif current:
        try:
            pts, _ = planner.plan_segment((current["x"], current["y"]), candidate["world_xy"], "approach")
            for i, p in enumerate(pts):
                p["waypoint_index"] = i

            appr_follow = controller.follow_pure_pursuit(
                node, pts, "approach_position", params.approach_controller_stop_tolerance_m,
                dense_route_waypoints, candidate["world_xy"])
            approach_result["controller_result"] = appr_follow
            approach_result["controller_reported_distance_m"] = appr_follow.get("final_distance_m")

            # Step 1: Stop and settle
            node.stop()
            time.sleep(params.approach_settle_sec)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)

            # Step 2: Re-read pose and recompute distance
            post_settle_pose = node.pose()
            approach_result["final_pose_after_settle"] = post_settle_pose
            post_settle_dist = distance(post_settle_pose, candidate["world_xy"]) if post_settle_pose else None
            approach_result["post_settle_distance_m"] = round(post_settle_dist, 6) if post_settle_dist is not None else None

            # Step 3: Check against formal tolerance
            if post_settle_dist is not None and post_settle_dist <= params.approach_position_tolerance_m:
                approach_result["approach_position_tolerance_reached"] = True
                approach_result["final_remeasured_distance_m"] = round(post_settle_dist, 6)
                approach_result["final_distance_to_approach_candidate_m"] = round(post_settle_dist, 6)
            elif post_settle_dist is not None and post_settle_dist <= params.approach_creep_boundary_upper_m:
                # Step 4: Boundary band creep
                approach_result["final_creep_attempted"] = True
                creep_distances: list[float] = []
                creep_pose = post_settle_pose
                for creep_i in range(params.approach_creep_max_attempts):
                    approach_result["final_creep_attempt_count"] = creep_i + 1
                    heading_to_cand = math.atan2(
                        candidate["world_xy"][1] - float(creep_pose["y"]),
                        candidate["world_xy"][0] - float(creep_pose["x"]),
                    )
                    heading_err = angle_wrap(heading_to_cand - float(creep_pose["yaw"]))
                    creep_linear = 0.06
                    creep_angular = clamp(0.6 * heading_err, -0.20, 0.20)
                    creep_step_duration = params.approach_creep_step_m / max(creep_linear, 0.01)
                    creep_deadline = time.monotonic() + creep_step_duration + 0.5
                    while time.monotonic() < creep_deadline and controller.remaining() > 0:
                        rclpy.spin_once(node, timeout_sec=0.03)
                        node.command(creep_linear, creep_angular)
                        time.sleep(0.10)
                        cp = node.pose()
                        if cp:
                            cd = distance(cp, candidate["world_xy"])
                            if cd <= params.approach_position_tolerance_m:
                                break
                    node.stop()
                    time.sleep(0.3)
                    for _ in range(5):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    creep_pose = node.pose()
                    if creep_pose:
                        cd = distance(creep_pose, candidate["world_xy"])
                        creep_distances.append(round(cd, 6))
                        if cd <= params.approach_position_tolerance_m:
                            approach_result["approach_position_tolerance_reached"] = True
                            approach_result["final_remeasured_distance_m"] = round(cd, 6)
                            approach_result["final_distance_to_approach_candidate_m"] = round(cd, 6)
                            approach_result["final_pose_after_creep"] = creep_pose
                            break
                    else:
                        creep_distances.append(None)
                approach_result["final_creep_distances_m"] = creep_distances
                if not approach_result["approach_position_tolerance_reached"]:
                    final_cd = creep_distances[-1] if creep_distances else post_settle_dist
                    approach_result["final_remeasured_distance_m"] = final_cd
                    approach_result["final_distance_to_approach_candidate_m"] = final_cd
                    approach_result["final_pose_after_creep"] = creep_pose
                    approach_result["failure_reason"] = "approach not reached after bounded creep"
            else:
                approach_result["final_remeasured_distance_m"] = round(post_settle_dist, 6) if post_settle_dist is not None else None
                approach_result["final_distance_to_approach_candidate_m"] = round(post_settle_dist, 6) if post_settle_dist is not None else None
                approach_result["failure_reason"] = "approach not reached"
        except Exception as exc:
            approach_result["failure_reason"] = str(exc)

    approach_result["approach_position_reached"] = approach_result["approach_position_tolerance_reached"]

    # Yaw alignment (only after approach success)
    yaw_result: dict[str, Any] = {"attempted": False, "yaw_alignment_success": False, "failure_reason": None}
    if approach_result["approach_position_tolerance_reached"]:
        yaw_result = _align_yaw(node, controller, proxy, params)

    return approach_result, yaw_result


def _align_yaw(node, controller, proxy: dict[str, Any], params: ControllerParams) -> dict[str, Any]:
    """Align robot yaw to face the object proxy."""
    import rclpy

    start = node.pose()
    result: dict[str, Any] = {"attempted": True, "yaw_alignment_success": False, "failure_reason": None, "start_pose": start}
    if start is None:
        result["failure_reason"] = "pose unavailable"
        return result

    deadline = time.monotonic() + min(params.yaw_timeout_sec, max(0.0, controller.remaining()))
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.04)
        pose = node.pose()
        if pose is None:
            result["failure_reason"] = "pose lost during yaw"
            break
        target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(pose["y"]),
                                proxy["yaw_proxy_xy"][0] - float(pose["x"]))
        err = angle_wrap(target_yaw - float(pose["yaw"]))
        drift = distance(start, pose)
        if drift > params.xy_drift_limit_m:
            result["failure_reason"] = "xy drift exceeded"
            break
        if abs(err) <= params.yaw_internal_tolerance_rad:
            node.stop()
            result["yaw_alignment_success"] = True
            break
        raw_ang = clamp(params.yaw_kp * err, -params.max_yaw_angular_speed, params.max_yaw_angular_speed)
        applied = params.angular_smoothing_alpha * raw_ang + (1.0 - params.angular_smoothing_alpha) * controller.prev_angular
        applied = clamp(applied, -params.max_yaw_angular_speed, params.max_yaw_angular_speed)
        node.command(0.0, applied)
        controller.prev_angular = applied

        t_sec = round(time.monotonic() - controller.start_time, 6)
        controller.telemetry.append({
            "t_sec": t_sec, "phase": "yaw_alignment",
            "x": round(float(pose["x"]), 6), "y": round(float(pose["y"]), 6), "yaw": round(float(pose["yaw"]), 6),
            "heading_error": round(err, 6), "raw_angular_z": round(raw_ang, 6), "applied_angular_z": round(applied, 6),
            "linear_x": 0.0, "rotate_in_place": True, "zero_linear_reason": "yaw_alignment",
        })
        controller.trajectory.append({
            "t_sec": t_sec, "phase": "yaw_alignment",
            "x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose["yaw"]),
            "cmd_vel_linear_x": 0.0, "cmd_vel_angular_z": round(applied, 6),
        })
        controller.commands.append({"t_sec": t_sec, "phase": "yaw_alignment", "linear_x": 0.0, "angular_z": round(applied, 6)})
        time.sleep(0.125)

    node.stop()
    final = node.pose()
    if final and not result.get("yaw_alignment_success"):
        target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(final["y"]),
                                proxy["yaw_proxy_xy"][0] - float(final["x"]))
        final_err = abs(angle_wrap(target_yaw - float(final["yaw"])))
        drift = distance(start, final)
        if final_err <= params.yaw_report_tolerance_rad and drift <= params.xy_drift_limit_m:
            result["yaw_alignment_success"] = True
        result["final_yaw_error_rad"] = round(final_err, 6)
        result["xy_drift_m"] = round(drift, 6)
    elif final and result.get("yaw_alignment_success"):
        target_yaw = math.atan2(proxy["yaw_proxy_xy"][1] - float(final["y"]),
                                proxy["yaw_proxy_xy"][0] - float(final["x"]))
        result["final_yaw_error_rad"] = round(abs(angle_wrap(target_yaw - float(final["yaw"]))), 6)
        result["xy_drift_m"] = round(distance(start, final), 6)
    result["final_pose"] = final
    if not result["yaw_alignment_success"] and result["failure_reason"] is None:
        result["failure_reason"] = "yaw tolerance not reached"
    return result
