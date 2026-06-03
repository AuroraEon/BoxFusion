"""Curvature-based pure pursuit controller with dynamic lookahead."""

from __future__ import annotations

import math
import time
from typing import Any

from .schemas import ControllerParams, angle_wrap, clamp, distance


def project_onto_polyline(px: float, py: float, points: list[dict[str, Any]]) -> tuple[float, float, float, int]:
    """Returns (proj_x, proj_y, arc_length_to_projection, segment_index)."""
    best_dist = float("inf")
    best_proj = (float(points[0]["x"]), float(points[0]["y"]))
    best_arc = 0.0
    best_seg = 0
    arc = 0.0
    for i in range(len(points) - 1):
        ax, ay = float(points[i]["x"]), float(points[i]["y"])
        bx, by = float(points[i + 1]["x"]), float(points[i + 1]["y"])
        dx, dy = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-9:
            arc += seg_len
            continue
        t = clamp(((px - ax) * dx + (py - ay) * dy) / (seg_len * seg_len), 0.0, 1.0)
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        d = math.hypot(px - proj_x, py - proj_y)
        if d < best_dist:
            best_dist = d
            best_proj = (proj_x, proj_y)
            best_arc = arc + t * seg_len
            best_seg = i
        arc += seg_len
    return best_proj[0], best_proj[1], best_arc, best_seg


def polyline_total_length(points: list[dict[str, Any]]) -> float:
    return sum(math.hypot(float(points[i + 1]["x"]) - float(points[i]["x"]),
                          float(points[i + 1]["y"]) - float(points[i]["y"]))
               for i in range(len(points) - 1))


def advance_along_polyline(points: list[dict[str, Any]], arc_start: float, advance: float) -> tuple[float, float]:
    """Get XY position at arc_start + advance along polyline."""
    target_arc = arc_start + advance
    arc = 0.0
    for i in range(len(points) - 1):
        ax, ay = float(points[i]["x"]), float(points[i]["y"])
        bx, by = float(points[i + 1]["x"]), float(points[i + 1]["y"])
        seg_len = math.hypot(bx - ax, by - ay)
        if arc + seg_len >= target_arc:
            t = (target_arc - arc) / seg_len if seg_len > 1e-9 else 0.0
            return ax + t * (bx - ax), ay + t * (by - ay)
        arc += seg_len
    return float(points[-1]["x"]), float(points[-1]["y"])


def compute_upcoming_turn_angle(points: list[dict[str, Any]], seg_idx: int, lookahead_segs: int = 4) -> float:
    """Estimate the upcoming turn angle by looking at path direction changes."""
    max_turn = 0.0
    end = min(seg_idx + lookahead_segs, len(points) - 2)
    for i in range(max(seg_idx, 0), end):
        ax, ay = float(points[i]["x"]), float(points[i]["y"])
        bx, by = float(points[i + 1]["x"]), float(points[i + 1]["y"])
        if i + 2 <= len(points) - 1:
            cx, cy = float(points[i + 2]["x"]), float(points[i + 2]["y"])
        else:
            break
        v1 = (bx - ax, by - ay)
        v2 = (cx - bx, cy - by)
        len1, len2 = math.hypot(*v1), math.hypot(*v2)
        if len1 < 1e-6 or len2 < 1e-6:
            continue
        cos_a = clamp((v1[0] * v2[0] + v1[1] * v2[1]) / (len1 * len2), -1.0, 1.0)
        turn = math.acos(cos_a)
        max_turn = max(max_turn, turn)
    return max_turn


def dynamic_lookahead(seg_idx: int, points: list[dict[str, Any]], goal_dist: float, params: ControllerParams) -> tuple[float, float, str]:
    """Compute dynamic lookahead based on upcoming curvature and proximity."""
    upcoming_turn = compute_upcoming_turn_angle(points, seg_idx)
    if goal_dist < 0.8:
        ld = params.lookahead_min + 0.15
        return ld, upcoming_turn, "near_goal"
    if upcoming_turn > 0.6:
        ld = params.lookahead_min + (params.lookahead_base - params.lookahead_min) * max(0.0, 1.0 - upcoming_turn / 1.5)
        return ld, upcoming_turn, "high_curvature"
    if upcoming_turn > 0.25:
        ld = params.lookahead_base
        return ld, upcoming_turn, "moderate_curvature"
    ld = min(params.lookahead_max, params.lookahead_base + (params.lookahead_max - params.lookahead_base) * (1.0 - upcoming_turn / 0.25))
    return ld, upcoming_turn, "straight"


class CurvatureController:
    """Curvature-based pure pursuit controller with angular smoothing and rate limiting."""

    def __init__(self, params: ControllerParams) -> None:
        self.params = params
        self.prev_angular: float = 0.0
        self.rotate_in_place: bool = False
        self.last_stamp: int | None = None
        self.pose_changed_count: int = 0
        self.pose_repeated_count: int = 0
        self.angular_saturation_count: int = 0
        self.angular_sign_switch_count: int = 0
        self.zero_linear_count: int = 0
        self.total_steps: int = 0
        self.prev_angular_sign: int = 0
        self.start_time: float = time.monotonic()
        # Telemetry
        self.telemetry: list[dict[str, Any]] = []
        self.trajectory: list[dict[str, Any]] = []
        self.commands: list[dict[str, Any]] = []

    def remaining(self) -> float:
        return self.params.timeout_sec - (time.monotonic() - self.start_time)

    def follow_pure_pursuit(self, node, points: list[dict[str, Any]], phase: str,
                            tolerance: float, dense_route_waypoints: list[dict[str, Any]],
                            candidate_world_xy: list[float]) -> dict[str, Any]:
        """Follow a polyline using curvature-based pure pursuit. Returns result dict."""
        import rclpy

        params = self.params
        result: dict[str, Any] = {"phase": phase, "attempted": True, "controller_success": False, "failure_reason": None}

        while rclpy.ok() and self.remaining() > 0:
            rclpy.spin_once(node, timeout_sec=0.03)
            pose = node.pose()
            if pose is None:
                result["failure_reason"] = "pose feedback unavailable"
                break

            stamp = pose.get("stamp_ns")
            pose_is_new = stamp != self.last_stamp
            if pose_is_new:
                self.last_stamp = stamp
                self.pose_changed_count += 1
            else:
                self.pose_repeated_count += 1
                time.sleep(0.02)
                continue

            self.total_steps += 1
            px, py = float(pose["x"]), float(pose["y"])
            pyaw = float(pose["yaw"])

            proj_x, proj_y, progress, seg_idx = project_onto_polyline(px, py, points)
            cross_track_error = math.hypot(px - proj_x, py - proj_y)

            if cross_track_error > params.path_deviation_limit_m:
                result["failure_reason"] = f"path deviation {cross_track_error:.3f} m exceeds {params.path_deviation_limit_m:.3f} m"
                break

            goal_dist = distance(pose, points[-1])
            if goal_dist <= tolerance:
                node.stop()
                result.update({"controller_success": True, "final_pose": pose, "final_distance_m": round(goal_dist, 6)})
                return result

            lookahead_d, upcoming_turn, lookahead_reason = dynamic_lookahead(seg_idx, points, goal_dist, params)
            lk_x, lk_y = advance_along_polyline(points, progress, lookahead_d)

            # Transform to robot frame
            dx_world = lk_x - px
            dy_world = lk_y - py
            cos_yaw = math.cos(pyaw)
            sin_yaw = math.sin(pyaw)
            robot_lk_x = cos_yaw * dx_world + sin_yaw * dy_world
            robot_lk_y = -sin_yaw * dx_world + cos_yaw * dy_world

            heading_error = angle_wrap(math.atan2(lk_y - py, lk_x - px) - pyaw)

            # Curvature
            lookahead_dist_sq = robot_lk_x ** 2 + robot_lk_y ** 2
            curvature = 2.0 * robot_lk_y / lookahead_dist_sq if lookahead_dist_sq >= 1e-6 else 0.0

            # Hysteresis rotate-in-place
            if not self.rotate_in_place and abs(heading_error) > params.rotate_enter_rad:
                self.rotate_in_place = True
            elif self.rotate_in_place and abs(heading_error) < params.rotate_exit_rad:
                self.rotate_in_place = False

            zero_linear_reason = "none"
            speed_scale_heading = 1.0
            speed_scale_curvature = 1.0

            if self.rotate_in_place:
                zero_linear_reason = "extreme_heading_rotate"
                linear = 0.0
                self.zero_linear_count += 1
                omega_raw = clamp(params.heading_kp * heading_error, -params.max_angular_speed, params.max_angular_speed)
            else:
                speed_scale_heading = max(0.15, math.cos(min(abs(heading_error), 1.2)))
                abs_curvature = abs(curvature)
                speed_scale_curvature = max(0.25, 1.0 - abs_curvature * 0.4)
                base_speed = min(params.max_linear_speed, max(0.04, goal_dist * 0.4))
                linear = base_speed * speed_scale_heading * speed_scale_curvature
                omega_raw = clamp(linear * curvature, -params.max_angular_speed, params.max_angular_speed)

            # Angular smoothing + rate limit
            applied_angular = params.angular_smoothing_alpha * omega_raw + (1.0 - params.angular_smoothing_alpha) * self.prev_angular
            delta = applied_angular - self.prev_angular
            if abs(delta) > params.angular_rate_limit:
                applied_angular = self.prev_angular + math.copysign(params.angular_rate_limit, delta)
            applied_angular = clamp(applied_angular, -params.max_angular_speed, params.max_angular_speed)

            saturated = abs(applied_angular) >= params.max_angular_speed * 0.95
            if saturated:
                self.angular_saturation_count += 1

            cur_sign = 1 if applied_angular > 0.01 else (-1 if applied_angular < -0.01 else 0)
            if cur_sign != 0 and self.prev_angular_sign != 0 and cur_sign != self.prev_angular_sign:
                self.angular_sign_switch_count += 1
            if cur_sign != 0:
                self.prev_angular_sign = cur_sign

            node.command(linear, applied_angular)
            self.prev_angular = applied_angular

            # Dense route nearest distance
            dense_nearest = min((distance(pose, dp) for dp in dense_route_waypoints), default=None) if dense_route_waypoints else None
            dist_to_approach = distance(pose, candidate_world_xy)

            t_sec = round(time.monotonic() - self.start_time, 6)
            step_record = {
                "t_sec": t_sec, "phase": phase,
                "x": round(px, 6), "y": round(py, 6), "yaw": round(pyaw, 6),
                "proj_x": round(proj_x, 6), "proj_y": round(proj_y, 6),
                "robot_frame_lookahead_x": round(robot_lk_x, 6),
                "robot_frame_lookahead_y": round(robot_lk_y, 6),
                "lookahead_x": round(lk_x, 6), "lookahead_y": round(lk_y, 6),
                "lookahead_distance_used": round(lookahead_d, 4),
                "upcoming_turn_angle_rad": round(upcoming_turn, 4),
                "lookahead_policy_reason": lookahead_reason,
                "cross_track_error": round(cross_track_error, 6),
                "heading_error": round(heading_error, 6),
                "curvature": round(curvature, 6),
                "omega_raw_curvature": round(omega_raw, 6),
                "omega_applied": round(applied_angular, 6),
                "linear_x": round(linear, 6),
                "speed_scale_heading": round(speed_scale_heading, 4),
                "speed_scale_curvature": round(speed_scale_curvature, 4),
                "angular_saturation": saturated,
                "rotate_in_place": self.rotate_in_place,
                "zero_linear_reason": zero_linear_reason,
                "control_path_progress_m": round(progress, 6),
                "dense_route_nearest_m": round(dense_nearest, 6) if dense_nearest is not None else None,
                "pose_stamp_changed": True,
                "distance_to_goal": round(goal_dist, 6),
                "distance_to_approach": round(dist_to_approach, 6),
            }
            self.telemetry.append(step_record)
            self.trajectory.append({"t_sec": t_sec, "phase": phase, "x": px, "y": py, "yaw": pyaw,
                                    "cmd_vel_linear_x": round(linear, 6), "cmd_vel_angular_z": round(applied_angular, 6)})
            self.commands.append({"t_sec": t_sec, "phase": phase, "linear_x": round(linear, 6), "angular_z": round(applied_angular, 6)})

            time.sleep(0.10)

        node.stop()
        pose = node.pose()
        result.update({"final_pose": pose, "final_distance_m": round(distance(pose, points[-1]), 6) if pose else None})
        if result["failure_reason"] is None:
            result["failure_reason"] = "timed out"
        return result
