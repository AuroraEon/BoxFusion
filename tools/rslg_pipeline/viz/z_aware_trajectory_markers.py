#!/usr/bin/env python3
"""Z-aware trajectory / robot MarkerArray builders for RSLG-SLAM visualization.

These helpers build ``visualization_msgs`` markers for the z-aware vertical-transition
*visualization overlay* used by the base-level route follower. They lift the executed
trajectory and robot marker to a route-derived z so RViz shows a visible rise from
floor_1 z=0.0 to floor_2 z=1.6 across the vertical connector vt_1_centerline_e001.

This is a visualization overlay only. It does not implement or imply physical stair
climbing, quadruped gait control, or Unitree Go2 real-robot control. The Gazebo
physics robot stays base-level (x/y). Imported by the ROS follower (runs under
/usr/bin/python3); not intended to run under the offline python.
"""

from __future__ import annotations

import math
from typing import Any, Optional

from geometry_msgs.msg import Point
from rclpy.duration import Duration
from visualization_msgs.msg import Marker

Z_AWARE_TRAJECTORY_NS = "rslg_z_aware_executed_trajectory"
Z_AWARE_ROBOT_NS = "rslg_z_aware_robot_pose"
Z_AWARE_ROBOT_BODY_NS = "rslg_z_aware_robot_body"
Z_AWARE_WAYPOINT_NS = "rslg_z_aware_current_waypoint"
Z_AWARE_PLANNED_NS = "rslg_z_aware_planned_route"
Z_AWARE_TRANSITION_NS = "rslg_z_aware_transition_band"


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _common(marker: Marker, *, frame_id: str, stamp: Any, ns: str, marker_id: int, marker_type: int, lifetime_sec: float = 0.0) -> None:
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = Duration(seconds=lifetime_sec).to_msg()


def build_z_aware_planned_route_marker(z_aware_waypoints: list[dict[str, Any]], *, frame_id: str, stamp: Any) -> Marker:
    """A LINE_STRIP through all z-aware planned waypoints (rises at the connector)."""
    marker = Marker()
    _common(marker, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_PLANNED_NS, marker_id=10, marker_type=Marker.LINE_STRIP)
    marker.scale.x = 0.05
    marker.color.r = 0.55
    marker.color.g = 0.35
    marker.color.b = 0.95
    marker.color.a = 0.85
    for wp in z_aware_waypoints:
        marker.points.append(_point(wp["x"], wp["y"], wp["z"]))
    return marker


def build_z_aware_transition_band_marker(z_aware_waypoints: list[dict[str, Any]], *, frame_id: str, stamp: Any) -> Optional[Marker]:
    """A thick LINE_STRIP over the vertical_transition phase highlighting the rise."""
    transition = [wp for wp in z_aware_waypoints if wp.get("route_phase") == "vertical_transition"]
    if not transition:
        return None
    # Extend the band by one waypoint on each side for a clearer floor_1->floor_2 rise.
    first_idx = transition[0]["index"]
    last_idx = transition[-1]["index"]
    lo = max(0, first_idx - 1)
    hi = min(len(z_aware_waypoints) - 1, last_idx + 1)
    marker = Marker()
    _common(marker, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_TRANSITION_NS, marker_id=11, marker_type=Marker.LINE_STRIP)
    marker.scale.x = 0.11
    marker.color.r = 0.98
    marker.color.g = 0.55
    marker.color.b = 0.10
    marker.color.a = 0.95
    for wp in z_aware_waypoints[lo : hi + 1]:
        marker.points.append(_point(wp["x"], wp["y"], wp["z"]))
    return marker


def build_z_aware_executed_trajectory_marker(z_points: list[tuple[float, float, float]], *, frame_id: str, stamp: Any) -> Marker:
    """A LINE_STRIP of executed (x, y, visual_z) samples — the rising visual path."""
    marker = Marker()
    _common(marker, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_TRAJECTORY_NS, marker_id=1, marker_type=Marker.LINE_STRIP)
    marker.scale.x = 0.07
    marker.color.r = 0.05
    marker.color.g = 0.85
    marker.color.b = 1.0
    marker.color.a = 1.0
    for x, y, z in z_points[-1500:]:
        marker.points.append(_point(x, y, z))
    return marker


def build_z_aware_robot_markers(x: float, y: float, z: float, yaw: float, *, frame_id: str, stamp: Any) -> list[Marker]:
    """A lifted robot body cube + heading arrow at the z-aware visual z."""
    qx, qy, qz, qw = quaternion_from_yaw(yaw)

    body = Marker()
    _common(body, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_ROBOT_BODY_NS, marker_id=2, marker_type=Marker.CUBE)
    body.pose.position.x = float(x)
    body.pose.position.y = float(y)
    body.pose.position.z = float(z) + 0.16
    body.pose.orientation.x = qx
    body.pose.orientation.y = qy
    body.pose.orientation.z = qz
    body.pose.orientation.w = qw
    body.scale.x = 0.5
    body.scale.y = 0.24
    body.scale.z = 0.2
    body.color.r = 0.12
    body.color.g = 0.14
    body.color.b = 0.18
    body.color.a = 0.95

    arrow = Marker()
    _common(arrow, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_ROBOT_NS, marker_id=3, marker_type=Marker.ARROW)
    arrow.pose.position.x = float(x)
    arrow.pose.position.y = float(y)
    arrow.pose.position.z = float(z) + 0.30
    arrow.pose.orientation.x = qx
    arrow.pose.orientation.y = qy
    arrow.pose.orientation.z = qz
    arrow.pose.orientation.w = qw
    arrow.scale.x = 0.55
    arrow.scale.y = 0.08
    arrow.scale.z = 0.08
    arrow.color.r = 0.02
    arrow.color.g = 0.9
    arrow.color.b = 1.0
    arrow.color.a = 1.0
    return [body, arrow]


def build_z_aware_current_waypoint_marker(waypoint: dict[str, Any], *, frame_id: str, stamp: Any) -> Marker:
    """A sphere at the current z-aware target waypoint (route-truth z)."""
    marker = Marker()
    _common(marker, frame_id=frame_id, stamp=stamp, ns=Z_AWARE_WAYPOINT_NS, marker_id=4, marker_type=Marker.SPHERE)
    marker.pose.position.x = float(waypoint["x"])
    marker.pose.position.y = float(waypoint["y"])
    marker.pose.position.z = float(waypoint["z"]) + 0.12
    marker.scale.x = 0.22
    marker.scale.y = 0.22
    marker.scale.z = 0.22
    marker.color.r = 0.95
    marker.color.g = 0.85
    marker.color.b = 0.1
    marker.color.a = 0.9
    return marker
