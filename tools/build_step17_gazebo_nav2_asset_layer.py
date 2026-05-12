#!/usr/bin/env python3
"""Build Step 17 Gazebo/Nav2 preparation artifacts.

The generated outputs are downstream-only. They read committed/public scene
artifacts plus the Step 16 navigation projection and do not modify Stage-A
runtime behavior, topology construction, route planning, schemas, or exports.
"""

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = REPO_ROOT / "runtime_stage1_frozen_evidence" / "step17_gazebo_nav2_asset_layer"
DOC_MD = REPO_ROOT / "docs" / "step17_gazebo_nav2_asset_layer.md"
DOC_CSV = REPO_ROOT / "docs" / "step17_gazebo_nav2_asset_layer.csv"

SCENES = {
    "00829-QaLdnwvtxbs": {
        "root": REPO_ROOT
        / "runtime_stage1_frozen_evidence/step4_regenerated_missing_scenes/scenes/00829-QaLdnwvtxbs",
        "role": "primary_sanity_scene",
        "reason": "smallest route, 2 rooms, 2 topology edges, first Gazebo sanity target",
    },
    "00843-DYehNKdT76V": {
        "root": REPO_ROOT
        / "runtime_stage1_frozen_evidence/final_freeze_verification/scenes/00843-DYehNKdT76V",
        "role": "primary_paper_scene",
        "reason": "main retained scene, moderate size, second validation target",
    },
}

COMMITTED_ARTIFACT_NAMES = (
    "topology_v0_1.json",
    "topology_query_report.json",
    "committed_room_world_model_v0_1.json",
    "committed_room_world_snapshot_v0_1.json",
)

SAFE_AUDIT_COMMANDS = [
    "pwd",
    "uname -a",
    "cat /etc/os-release || true",
    "which ros2 || true",
    "which gazebo || true",
    "which gzserver || true",
    "which gzclient || true",
    "which rviz2 || true",
    "printenv ROS_DISTRO || true",
    "printenv AMENT_PREFIX_PATH || true",
    'ros2 pkg list | rg "gazebo|nav2|turtlebot|slam_toolbox|map_server|amcl|robot_state_publisher|joint_state_publisher|tf2_ros|geometry_msgs|nav_msgs|visualization_msgs" || true',
    "apt-cache policy gazebo || true",
    "apt-cache policy ros-foxy-gazebo-ros-pkgs || true",
    "apt-cache policy ros-foxy-navigation2 || true",
    "apt-cache policy ros-foxy-nav2-bringup || true",
    "apt-cache policy ros-foxy-turtlebot3-gazebo || true",
]


def rel(path: Path) -> str:
    return str(path.relative_to(REPO_ROOT))


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def room_id(value: Any) -> str:
    text = str(value)
    if text.startswith("room_"):
        return text
    if text.isdigit():
        return f"room_{text}"
    return text


def xy_from(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        if "x" in value and "y" in value:
            return [float(value["x"]), float(value["y"])]
        if "xy" in value:
            return xy_from(value["xy"])
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
    return None


def floor_z_lookup(topo: dict[str, Any]) -> dict[str, float]:
    floors = topo.get("floors") or []
    lookup: dict[str, float] = {}
    for index, floor in enumerate(floors):
        floor_id = str(floor.get("floor_id") or floor.get("id") or f"floor_{index + 1}")
        z = floor.get("z_center")
        if z is None:
            z = float(index) * 3.2
        lookup[floor_id] = float(z)
    return lookup


def polygon_points(room: dict[str, Any]) -> list[list[float]]:
    polygon = room.get("polygon") or room.get("footprint_polygon_xy") or []
    points: list[list[float]] = []
    for vertex in polygon:
        xy = xy_from(vertex)
        if xy is not None:
            points.append(xy)
    return points


def room_floor(room: dict[str, Any]) -> str:
    return str(room.get("floor_id") or "floor_1")


def point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    if len(polygon) < 3:
        return False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point[0], point[1]
        xj, yj = polygon[j][0], polygon[j][1]
        intersects = (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        if intersects:
            inside = not inside
        j = i
    return inside


def command_capture(command: str) -> dict[str, Any]:
    proc = subprocess.run(
        ["bash", "-lc", command],
        cwd=REPO_ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return {
        "command": command,
        "returncode": proc.returncode,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
    }


def env_audit() -> dict[str, Any]:
    commands = [command_capture(command) for command in SAFE_AUDIT_COMMANDS]
    ros_packages = next(
        (item["stdout"].splitlines() for item in commands if item["command"].startswith("ros2 pkg list")),
        [],
    )
    actual_nav2_packages = [
        line
        for line in ros_packages
        if line.startswith("nav2") or line in {"amcl", "map_server", "nav2_map_server", "nav2_amcl"}
    ]
    actual_gazebo_packages = [line for line in ros_packages if "gazebo" in line]
    return {
        "audit_kind": "safe_environment_feasibility_audit",
        "did_not_run_privileged_install": True,
        "commands": commands,
        "summary": {
            "ros2": shutil.which("ros2"),
            "rviz2": shutil.which("rviz2"),
            "gazebo": shutil.which("gazebo"),
            "gzserver": shutil.which("gzserver"),
            "gzclient": shutil.which("gzclient"),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "matching_ros_packages": [line for line in ros_packages if line.strip()],
            "actual_nav2_packages": actual_nav2_packages,
            "actual_gazebo_packages": actual_gazebo_packages,
            "gazebo_executable_available": shutil.which("gazebo") is not None,
            "nav2_package_hint_available": bool(actual_nav2_packages),
        },
    }


def map_bounds(rooms: list[dict[str, Any]], margin: float) -> tuple[float, float, float, float]:
    points: list[list[float]] = []
    for room in rooms:
        points.extend(polygon_points(room))
        center = xy_from(room.get("center"))
        if center is not None:
            points.append(center)
    if not points:
        return (-5.0, -5.0, 5.0, 5.0)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)


def write_pgm(path: Path, pixels: list[list[int]]) -> None:
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    lines = ["P2", "# BoxFusion Step 17 approximate map; not validated for navigation", f"{width} {height}", "255"]
    for row in pixels:
        lines.append(" ".join(str(value) for value in row))
    write_text(path, "\n".join(lines) + "\n")


def generate_floor_map(scene_id: str, scene_out: Path, floor_id: str, rooms: list[dict[str, Any]]) -> dict[str, Any]:
    resolution = 0.05
    margin = 1.0
    x_min, y_min, x_max, y_max = map_bounds(rooms, margin)
    width = max(20, int(math.ceil((x_max - x_min) / resolution)))
    height = max(20, int(math.ceil((y_max - y_min) / resolution)))

    polygons = [polygon_points(room) for room in rooms if len(polygon_points(room)) >= 3]
    pixels: list[list[int]] = []
    free_count = 0
    occupied_count = 0
    for row in range(height):
        y = y_max - (row + 0.5) * resolution
        pixel_row: list[int] = []
        for col in range(width):
            x = x_min + (col + 0.5) * resolution
            free = any(point_in_polygon(x, y, polygon) for polygon in polygons)
            if free:
                pixel_row.append(254)
                free_count += 1
            else:
                pixel_row.append(0)
                occupied_count += 1
        pixels.append(pixel_row)

    safe_floor = floor_id.replace("/", "_")
    pgm_path = scene_out / "maps" / f"{safe_floor}_approx_nav_map.pgm"
    yaml_path = scene_out / "maps" / f"{safe_floor}_approx_nav_map.yaml"
    write_pgm(pgm_path, pixels)
    yaml = f"""# BoxFusion Step 17 approximate occupancy map.
# Generated from committed/public room polygons only.
# This is not a validated collision-free Nav2 map.
image: {pgm_path.name}
resolution: {resolution}
origin: [{x_min:.3f}, {y_min:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
"""
    write_text(yaml_path, yaml)
    return {
        "scene_id": scene_id,
        "floor_id": floor_id,
        "map_yaml": rel(yaml_path),
        "map_pgm": rel(pgm_path),
        "resolution_m_per_pixel": resolution,
        "origin_xy_yaw": [round(x_min, 3), round(y_min, 3), 0.0],
        "width_px": width,
        "height_px": height,
        "free_pixels": free_count,
        "occupied_pixels": occupied_count,
        "map_status": "approximate_unvalidated_from_room_polygons",
    }


def yaw_between(a: list[float], b: list[float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def sdf_box_visual(name: str, pose: list[float], size: list[float], rgba: str) -> str:
    return f"""
      <model name="{name}">
        <static>true</static>
        <pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} 0 0 {pose[5]:.3f}</pose>
        <link name="link">
          <visual name="visual">
            <geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>
            <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
          </visual>
        </link>
      </model>"""


def sdf_cylinder_visual(name: str, xy: list[float], z: float, radius: float, length: float, rgba: str) -> str:
    return f"""
      <model name="{name}">
        <static>true</static>
        <pose>{xy[0]:.3f} {xy[1]:.3f} {z:.3f} 0 0 0</pose>
        <link name="link">
          <visual name="visual">
            <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
            <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
          </visual>
        </link>
      </model>"""


def generate_marker_world(scene_id: str, scene_out: Path, topo: dict[str, Any], snapshot: dict[str, Any]) -> Path:
    floor_z = floor_z_lookup(topo)
    models: list[str] = []
    all_points: list[list[float]] = []

    for room in topo.get("rooms", []):
        rid = room_id(room.get("id"))
        fid = room_floor(room)
        z = floor_z.get(fid, 0.0)
        polygon = polygon_points(room)
        center = xy_from(room.get("center"))
        if center is not None:
            all_points.append(center)
            models.append(sdf_cylinder_visual(f"{rid}_center_marker", center, z + 0.08, 0.16, 0.16, "0.1 0.35 0.95 0.75"))
        for point in polygon:
            all_points.append(point)
        if len(polygon) >= 2:
            for index, start in enumerate(polygon):
                end = polygon[(index + 1) % len(polygon)]
                mid = [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0]
                length = math.hypot(end[0] - start[0], end[1] - start[1])
                yaw = yaw_between(start, end)
                models.append(
                    sdf_box_visual(
                        f"{rid}_polygon_edge_{index}",
                        [mid[0], mid[1], z + 0.025, 0.0, 0.0, yaw],
                        [length, 0.035, 0.05],
                        "0.1 0.1 0.1 0.55",
                    )
                )

    for index, gateway in enumerate(snapshot.get("gateways", [])):
        xy = xy_from(gateway.get("pos_world"))
        if xy is None:
            continue
        fid = str(gateway.get("floor_id") or "floor_1")
        z = floor_z.get(fid, 0.0)
        all_points.append(xy)
        models.append(sdf_cylinder_visual(f"gateway_marker_{index}", xy, z + 0.1, 0.12, 0.2, "0.0 0.8 0.25 0.85"))

    if all_points:
        x_min, y_min, x_max, y_max = map_bounds([{"polygon": all_points}], 2.0)
        ground_x = max(5.0, x_max - x_min)
        ground_y = max(5.0, y_max - y_min)
        ground_pose = [(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, -0.02, 0.0, 0.0, 0.0]
    else:
        ground_x = ground_y = 10.0
        ground_pose = [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]

    models.insert(
        0,
        sdf_box_visual(
            "boxfusion_marker_ground",
            ground_pose,
            [ground_x, ground_y, 0.02],
            "0.85 0.85 0.82 1.0",
        ),
    )

    world = f"""<?xml version="1.0" ?>
<!-- BoxFusion Step 17 marker-only world.
     Generated from committed/public artifacts. Visual scaffold only.
     It contains no validated collision world and proves no robot navigation. -->
<sdf version="1.6">
  <world name="boxfusion_{scene_id}_marker_world">
    <include><uri>model://sun</uri></include>
    {''.join(models)}
  </world>
</sdf>
"""
    path = scene_out / "worlds" / f"{scene_id}_marker_world.sdf"
    write_text(path, world)
    return path


def yaml_scalar(value: Any) -> str:
    if isinstance(value, str):
        return f'"{value}"'
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    return str(value)


def write_waypoint_yaml(scene_out: Path, scene_id: str, nav_topology: dict[str, Any], nav_waypoints: dict[str, Any]) -> Path:
    path = scene_out / "waypoints" / "artifact_route_waypoints.yaml"
    lines = [
        "# BoxFusion Step 17 topological waypoints.",
        "# These are not collision-free local planner paths.",
        f"scene_id: {scene_id}",
        "frame_id: boxfusion_map",
        "source_policy: committed_public_artifacts_only",
        "robot_enabled_edges: []",
        "requires_planner_validation: true",
        "sample_route:",
    ]
    route = nav_waypoints.get("sample_route") or {}
    for key in ("room_sequence", "used_relation_types", "route_confidence", "total_cost"):
        lines.append(f"  {key}: {json.dumps(route.get(key))}")
    lines.append("sample_route_waypoints:")
    for waypoint in nav_waypoints.get("sample_route_waypoints") or []:
        lines.append("  -")
        for key in ("type", "source", "room_id", "floor_id", "connects", "xy", "yaw", "width_m"):
            if key in waypoint:
                lines.append(f"    {key}: {json.dumps(waypoint[key])}")
    lines.append("candidate_edges:")
    for edge in nav_topology.get("robot_candidate_edges") or []:
        lines.append("  -")
        for key in ("edge_id", "source", "target", "relation_type", "source_floor_id", "target_floor_id", "edge_evidence_type", "navigation_policy"):
            lines.append(f"    {key}: {yaml_scalar(edge.get(key))}")
        lines.append("    not_collision_free: true")
        lines.append("    requires_planner_validation: true")
    write_text(path, "\n".join(lines) + "\n")
    return path


def nav2_params_template(default_map: str | None) -> str:
    map_yaml = default_map or "<path-to-generated-map-yaml>"
    return f"""# BoxFusion Step 17 Nav2 parameter template for ROS Foxy.
# It is a starting point only; no Nav2 run has been validated.
use_sim_time: true

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "{map_yaml}"

amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    global_frame_id: "map"
    robot_model_type: "nav2_amcl::DifferentialMotionModel"

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: false

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

bt_navigator:
  ros__parameters:
    use_sim_time: true

recoveries_server:
  ros__parameters:
    use_sim_time: true

waypoint_follower:
  ros__parameters:
    use_sim_time: true
"""


def package_files(package_root: Path) -> list[Path]:
    paths: list[Path] = []
    paths.append(package_root / "package.xml")
    write_text(
        paths[-1],
        """<?xml version="1.0"?>
<package format="3">
  <name>boxfusion_gazebo_nav2_demo</name>
  <version>0.0.1</version>
  <description>Step 17 downstream Gazebo/Nav2 asset skeleton for BoxFusion committed artifacts.</description>
  <maintainer email="boxfusion@example.invalid">BoxFusion</maintainer>
  <license>Research artifact</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
""",
    )
    paths.append(package_root / "setup.py")
    write_text(
        paths[-1],
        """from setuptools import setup
from glob import glob

package_name = "boxfusion_gazebo_nav2_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BoxFusion",
    maintainer_email="boxfusion@example.invalid",
    description="Step 17 downstream Gazebo/Nav2 skeleton for BoxFusion artifacts.",
    license="Research artifact",
)
""",
    )
    paths.append(package_root / "resource" / "boxfusion_gazebo_nav2_demo")
    write_text(paths[-1], "")
    paths.append(package_root / "boxfusion_gazebo_nav2_demo" / "__init__.py")
    write_text(paths[-1], "")
    paths.append(package_root / "launch" / "gazebo_marker_world.launch.py")
    write_text(
        paths[-1],
        """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    world = LaunchConfiguration("world")
    gazebo_launch = PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("world", description="Generated Step 17 marker-only SDF world path."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch), launch_arguments={"world": world}.items()),
    ])
""",
    )
    paths.append(package_root / "launch" / "nav2_map_only.launch.py")
    write_text(
        paths[-1],
        """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Generated approximate map YAML path."),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}],
        ),
    ])
""",
    )
    paths.append(package_root / "launch" / "nav2_bringup_template.launch.py")
    write_text(
        paths[-1],
        """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    bringup_launch = PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Generated approximate map YAML path."),
        DeclareLaunchArgument("params", description="Step 17 Nav2 params template path."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={"map": map_yaml, "params_file": params, "use_sim_time": "true"}.items(),
        ),
    ])
""",
    )
    paths.append(package_root / "params" / "nav2_params_template.yaml")
    write_text(paths[-1], nav2_params_template(None))
    return paths


def install_script() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail

echo "BoxFusion Step 17 optional ROS Foxy Gazebo/Nav2 dependency installer"
echo "This script is intentionally interactive. It will ask before sudo apt install."
echo

if [[ "${ROS_DISTRO:-}" != "foxy" ]]; then
  echo "Warning: ROS_DISTRO is '${ROS_DISTRO:-unset}', expected 'foxy'."
fi

packages=(
  gazebo
  ros-foxy-gazebo-ros-pkgs
  ros-foxy-navigation2
  ros-foxy-nav2-bringup
  ros-foxy-turtlebot3-gazebo
  ros-foxy-turtlebot3-description
  ros-foxy-robot-state-publisher
  ros-foxy-joint-state-publisher
  ros-foxy-tf2-ros
)

printf 'Packages proposed for installation:\\n'
printf '  %s\\n' "${packages[@]}"
echo
read -r -p "Run sudo apt update and sudo apt install for these packages? [y/N] " answer
case "$answer" in
  y|Y|yes|YES)
    sudo apt update
    sudo apt install -y "${packages[@]}"
    ;;
  *)
    echo "No privileged installation was run."
    ;;
esac
"""


def runbook_text(env: dict[str, Any], map_rows: list[dict[str, Any]], package_root: Path) -> str:
    gazebo_state = "available" if env["summary"]["gazebo_executable_available"] else "missing"
    nav_hint = "Nav2 packages found" if env["summary"]["nav2_package_hint_available"] else "Nav2 packages not installed"
    first_map = next((row["map_yaml"] for row in map_rows if row["scene_id"] == "00829-QaLdnwvtxbs"), "<map-yaml>")
    first_map_from_ws = first_map.replace("runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/", "../")
    return f"""# Step 17 Gazebo/Nav2 Preparation Runbook

This package prepares the first concrete Gazebo/Nav2 execution path from BoxFusion committed/public artifacts. It does not prove robot navigation.

## Current Feasibility

- ROS distro: `{env["summary"].get("ros_distro")}`
- `ros2`: `{env["summary"].get("ros2")}`
- `rviz2`: `{env["summary"].get("rviz2")}`
- Gazebo executables: `{gazebo_state}`
- Nav2 package status: `{nav_hint}`
- Privileged install executed by Step 17: `false`

## Source Boundary

Generated assets come from:

- `logs/topology_v0_1.json`
- `logs/topology_query_report.json`
- `logs/committed_room_world_model_v0_1.json`
- `logs/committed_room_world_snapshot_v0_1.json`
- Step 16 downstream navigation projection files

They do not use working topology, sidecar shadow, BEV, or modified runtime state as authoritative input.

## Optional Dependency Setup

Inspect the proposed installer before running it:

```bash
sed -n '1,220p' runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scripts/install_foxy_gazebo_nav2_deps.sh
```

Run it only when you are ready to authorize `sudo apt install`:

```bash
bash runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/scripts/install_foxy_gazebo_nav2_deps.sh
```

## Build The Skeleton Package

The ROS package skeleton is staged under:

`{rel(package_root)}`

Build from the generated workspace after dependencies are installed:

```bash
cd runtime_stage1_frozen_evidence/step17_gazebo_nav2_asset_layer/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
```

## First Sanity Scene

Use `00829-QaLdnwvtxbs` first. It is the smallest route and keeps the first Gazebo/Nav2 test focused.

Marker-only Gazebo world:

```bash
ros2 launch boxfusion_gazebo_nav2_demo gazebo_marker_world.launch.py world:=$PWD/../scenes/00829-QaLdnwvtxbs/worlds/00829-QaLdnwvtxbs_marker_world.sdf
```

Map-server-only smoke test:

```bash
ros2 launch boxfusion_gazebo_nav2_demo nav2_map_only.launch.py map:=$PWD/{first_map_from_ws}
```

Nav2 bringup template:

```bash
ros2 launch boxfusion_gazebo_nav2_demo nav2_bringup_template.launch.py \\
  map:=$PWD/{first_map_from_ws} \\
  params:=$PWD/src/boxfusion_gazebo_nav2_demo/params/nav2_params_template.yaml
```

## What To Record Later

For a real validation run, record:

- exact scene id, floor id, generated map YAML, world path, robot model, and Nav2 params
- whether Gazebo launched successfully
- whether Nav2 lifecycle nodes activated successfully
- planned path success/failure for each `robot_candidate_needs_validation` edge
- any collision, recovery, localization, or controller failure

Only after a real planner/Gazebo/Nav2 run should any edge be promoted beyond `robot_candidate_needs_validation`.

## Non-Claims

The generated PGM/YAML maps are approximate room-polygon rasterizations. The SDF world is marker-only. These assets do not prove collision-free navigation, local obstacle avoidance, full robot navigation, physical traversability, Gazebo execution, or Nav2 success.
"""


def docs_text(env: dict[str, Any], map_rows: list[dict[str, Any]], scene_rows: list[dict[str, Any]], generated_files: list[str]) -> str:
    env_summary = env["summary"]
    scene_table = "\n".join(
        f"| `{row['scene_id']}` | {row['role']} | {row['floors']} | {row['candidate_edges']} | {row['robot_enabled_edges']} | `{row['primary_world']}` |"
        for row in scene_rows
    )
    map_table = "\n".join(
        f"| `{row['scene_id']}` | `{row['floor_id']}` | {row['width_px']}x{row['height_px']} | `{row['map_yaml']}` | {row['map_status']} |"
        for row in map_rows
    )
    file_bullets = "\n".join(f"- `{path}`" for path in generated_files[:80])
    return f"""# Step 17 Gazebo/Nav2 Asset Layer

Step 17 prepares the first concrete Gazebo/Nav2 execution path from BoxFusion committed/public artifacts. It creates maps, waypoints, marker worlds, launch templates, an optional dependency installer, and a runbook. It does not execute Gazebo or Nav2 in this environment.

## Environment Feasibility

Safe audit results:

- `ros2`: `{env_summary.get('ros2')}`
- `rviz2`: `{env_summary.get('rviz2')}`
- `gazebo`: `{env_summary.get('gazebo')}`
- `gzserver`: `{env_summary.get('gzserver')}`
- `gzclient`: `{env_summary.get('gzclient')}`
- `ROS_DISTRO`: `{env_summary.get('ros_distro')}`
- Matching ROS package probe: `{', '.join(env_summary.get('matching_ros_packages') or [])}`

Gazebo executables are not available on PATH, and Nav2/Gazebo packages are not installed locally. Apt candidates exist for ROS Foxy packages in the configured ROS repository. Step 17 did not run `sudo apt install`.

## Generated Scenes

| scene_id | role | floors | candidate edges | robot-enabled edges | marker world |
| --- | --- | ---: | ---: | ---: | --- |
{scene_table}

## Approximate Maps

These maps are rasterized from committed room polygons. They are not validated occupancy maps.

| scene_id | floor_id | size | map YAML | status |
| --- | --- | ---: | --- | --- |
{map_table}

## Asset Contract

- `robot_enabled_edges` remains empty.
- Same-floor candidate edges remain `robot_candidate_needs_validation`.
- Weak, possible, and unsupported multi-floor edges are excluded from robot execution.
- The SDF world is marker-only and not a validated collision world.
- The Nav2 parameter and launch files are templates for later execution.
- Generated artifacts are downstream-only and leave Stage-A behavior, topology, route planning, and committed exports untouched.

## Output Root

`{rel(OUTPUT_ROOT)}`

## Key Files

{file_bullets}

## Reproduction

```bash
python3 tools/build_step17_gazebo_nav2_asset_layer.py
```

## Claim Boundary

Supported after Step 17: a concrete, reproducible Gazebo/Nav2 preparation layer exists for the 00829 sanity scene and 00843 paper scene.

Not supported after Step 17: Gazebo execution, Nav2 execution, collision-free local planning, local obstacle avoidance, full robot navigation, physical traversability, or promotion of any edge to `robot_enabled`.
"""


def main() -> None:
    env = env_audit()
    generated_files: list[str] = []
    scene_rows: list[dict[str, Any]] = []
    map_rows: list[dict[str, Any]] = []
    waypoint_rows: list[dict[str, Any]] = []

    write_json(OUTPUT_ROOT / "tables" / "environment_feasibility_audit.json", env)
    generated_files.append(rel(OUTPUT_ROOT / "tables" / "environment_feasibility_audit.json"))

    env_rows = [
        {
            "command": item["command"],
            "returncode": item["returncode"],
            "stdout_first_line": item["stdout"].splitlines()[0] if item["stdout"].splitlines() else "",
            "stderr_first_line": item["stderr"].splitlines()[0] if item["stderr"].splitlines() else "",
        }
        for item in env["commands"]
    ]
    write_csv(
        OUTPUT_ROOT / "tables" / "environment_feasibility_audit.csv",
        env_rows,
        ["command", "returncode", "stdout_first_line", "stderr_first_line"],
    )
    generated_files.append(rel(OUTPUT_ROOT / "tables" / "environment_feasibility_audit.csv"))

    for scene_id, info in SCENES.items():
        scene_root = Path(info["root"])
        logs = scene_root / "logs"
        topo = load_json(logs / "topology_v0_1.json")
        snapshot = load_json(logs / "committed_room_world_snapshot_v0_1.json")
        nav_topology = load_json(
            REPO_ROOT
            / "runtime_stage1_frozen_evidence/step16_navigation_projection_contract/scenes"
            / scene_id
            / "navigation_topology_v0_1.json"
        )
        nav_waypoints = load_json(
            REPO_ROOT
            / "runtime_stage1_frozen_evidence/step16_navigation_projection_contract/scenes"
            / scene_id
            / "navigation_waypoints_v0_1.json"
        )
        scene_out = OUTPUT_ROOT / "scenes" / scene_id

        rooms_by_floor: dict[str, list[dict[str, Any]]] = {}
        for room in topo.get("rooms", []):
            rooms_by_floor.setdefault(room_floor(room), []).append(room)

        scene_map_rows = [
            generate_floor_map(scene_id, scene_out, floor_id, rooms)
            for floor_id, rooms in sorted(rooms_by_floor.items())
        ]
        map_rows.extend(scene_map_rows)
        generated_files.extend(row["map_yaml"] for row in scene_map_rows)
        generated_files.extend(row["map_pgm"] for row in scene_map_rows)

        world_path = generate_marker_world(scene_id, scene_out, topo, snapshot)
        generated_files.append(rel(world_path))

        waypoint_yaml = write_waypoint_yaml(scene_out, scene_id, nav_topology, nav_waypoints)
        write_json(scene_out / "waypoints" / "artifact_route_waypoints.json", nav_waypoints)
        generated_files.append(rel(waypoint_yaml))
        generated_files.append(rel(scene_out / "waypoints" / "artifact_route_waypoints.json"))

        edge_rows = []
        for edge in nav_topology.get("robot_candidate_edges") or []:
            row = {
                "scene_id": scene_id,
                "edge_id": edge.get("edge_id"),
                "source": edge.get("source"),
                "target": edge.get("target"),
                "relation_type": edge.get("relation_type"),
                "source_floor_id": edge.get("source_floor_id"),
                "target_floor_id": edge.get("target_floor_id"),
                "edge_evidence_type": edge.get("edge_evidence_type"),
                "navigation_policy": edge.get("navigation_policy"),
                "not_collision_free": "yes",
                "requires_planner_validation": "yes",
                "gateway_xy": json.dumps((edge.get("gateway_waypoint") or {}).get("xy")),
            }
            edge_rows.append(row)
            waypoint_rows.append(row)
        write_csv(
            scene_out / "waypoints" / "candidate_edges_for_validation.csv",
            edge_rows,
            [
                "scene_id",
                "edge_id",
                "source",
                "target",
                "relation_type",
                "source_floor_id",
                "target_floor_id",
                "edge_evidence_type",
                "navigation_policy",
                "not_collision_free",
                "requires_planner_validation",
                "gateway_xy",
            ],
        )
        generated_files.append(rel(scene_out / "waypoints" / "candidate_edges_for_validation.csv"))

        params_path = scene_out / "nav2" / "nav2_params_template.yaml"
        write_text(params_path, nav2_params_template(scene_map_rows[0]["map_yaml"] if scene_map_rows else None))
        generated_files.append(rel(params_path))

        scene_manifest = {
            "scene_id": scene_id,
            "role": info["role"],
            "reason": info["reason"],
            "source_committed_artifact_paths": [rel(logs / name) for name in COMMITTED_ARTIFACT_NAMES],
            "step16_projection_paths": [
                rel(
                    REPO_ROOT
                    / "runtime_stage1_frozen_evidence/step16_navigation_projection_contract/scenes"
                    / scene_id
                    / "navigation_topology_v0_1.json"
                ),
                rel(
                    REPO_ROOT
                    / "runtime_stage1_frozen_evidence/step16_navigation_projection_contract/scenes"
                    / scene_id
                    / "navigation_waypoints_v0_1.json"
                ),
            ],
            "robot_enabled_edges": [],
            "robot_candidate_edges": len(nav_topology.get("robot_candidate_edges") or []),
            "disabled_edges": len(nav_topology.get("disabled_edges") or []),
            "unsupported_multifloor_edges": len(nav_topology.get("unsupported_multifloor_edges") or []),
            "map_rows": scene_map_rows,
            "marker_world": rel(world_path),
            "waypoint_yaml": rel(waypoint_yaml),
            "nav2_params_template": rel(params_path),
            "claim_boundary": "prepared_assets_only_no_gazebo_or_nav2_success_claim",
        }
        write_json(scene_out / "scene_asset_manifest.json", scene_manifest)
        generated_files.append(rel(scene_out / "scene_asset_manifest.json"))

        scene_rows.append(
            {
                "scene_id": scene_id,
                "role": info["role"],
                "reason": info["reason"],
                "floors": len(rooms_by_floor),
                "candidate_edges": len(nav_topology.get("robot_candidate_edges") or []),
                "robot_enabled_edges": 0,
                "primary_world": rel(world_path),
            }
        )

    write_csv(
        OUTPUT_ROOT / "tables" / "generated_map_inventory.csv",
        map_rows,
        [
            "scene_id",
            "floor_id",
            "map_yaml",
            "map_pgm",
            "resolution_m_per_pixel",
            "origin_xy_yaw",
            "width_px",
            "height_px",
            "free_pixels",
            "occupied_pixels",
            "map_status",
        ],
    )
    generated_files.append(rel(OUTPUT_ROOT / "tables" / "generated_map_inventory.csv"))

    write_csv(
        OUTPUT_ROOT / "tables" / "candidate_edge_waypoint_inventory.csv",
        waypoint_rows,
        [
            "scene_id",
            "edge_id",
            "source",
            "target",
            "relation_type",
            "source_floor_id",
            "target_floor_id",
            "edge_evidence_type",
            "navigation_policy",
            "not_collision_free",
            "requires_planner_validation",
            "gateway_xy",
        ],
    )
    generated_files.append(rel(OUTPUT_ROOT / "tables" / "candidate_edge_waypoint_inventory.csv"))

    write_csv(
        OUTPUT_ROOT / "tables" / "scene_asset_summary.csv",
        scene_rows,
        ["scene_id", "role", "reason", "floors", "candidate_edges", "robot_enabled_edges", "primary_world"],
    )
    generated_files.append(rel(OUTPUT_ROOT / "tables" / "scene_asset_summary.csv"))

    package_root = OUTPUT_ROOT / "ros2_ws" / "src" / "boxfusion_gazebo_nav2_demo"
    for path in package_files(package_root):
        generated_files.append(rel(path))

    installer_path = OUTPUT_ROOT / "scripts" / "install_foxy_gazebo_nav2_deps.sh"
    write_text(installer_path, install_script())
    generated_files.append(rel(installer_path))

    runbook_path = OUTPUT_ROOT / "runbook.md"
    write_text(runbook_path, runbook_text(env, map_rows, package_root))
    generated_files.append(rel(runbook_path))

    write_text(DOC_MD, docs_text(env, map_rows, scene_rows, generated_files))
    generated_files.append(rel(DOC_MD))
    write_csv(
        DOC_CSV,
        [
            {"category": "environment", "item": "ros2", "status": env["summary"].get("ros2") or "missing", "claim_boundary": "feasibility_audit_only"},
            {"category": "environment", "item": "gazebo", "status": env["summary"].get("gazebo") or "missing", "claim_boundary": "not_executed"},
            {"category": "environment", "item": "nav2", "status": "not_installed_locally", "claim_boundary": "not_executed"},
            {"category": "asset", "item": "00829 maps/world/waypoints", "status": "generated", "claim_boundary": "approximate_unvalidated"},
            {"category": "asset", "item": "00843 maps/world/waypoints", "status": "generated", "claim_boundary": "approximate_unvalidated"},
            {"category": "claim", "item": "robot_enabled_edges", "status": "0", "claim_boundary": "no_promotion_without_real_validation"},
        ],
        ["category", "item", "status", "claim_boundary"],
    )
    generated_files.append(rel(DOC_CSV))

    manifest_path = OUTPUT_ROOT / "manifest" / "step17_gazebo_nav2_asset_manifest.json"
    generated_files.append(rel(manifest_path))
    manifest = {
        "step": 17,
        "artifact_kind": "gazebo_nav2_asset_layer",
        "output_root": rel(OUTPUT_ROOT),
        "source_policy": "committed_public_artifacts_and_step16_downstream_projection_only",
        "does_not_modify_stage_a_runtime": True,
        "does_not_modify_topology_construction": True,
        "does_not_modify_committed_exports": True,
        "does_not_promote_robot_enabled_edges": True,
        "gazebo_executed": False,
        "nav2_executed": False,
        "collision_free_navigation_claimed": False,
        "generated_files": generated_files,
    }
    write_json(manifest_path, manifest)

    print(f"Wrote Step 17 artifacts under {rel(OUTPUT_ROOT)}")
    print(f"Wrote docs: {rel(DOC_MD)} and {rel(DOC_CSV)}")


if __name__ == "__main__":
    main()
