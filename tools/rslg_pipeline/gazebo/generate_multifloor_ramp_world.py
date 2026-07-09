#!/usr/bin/env python3
"""Generate a Gazebo SDF world for task59 multi-floor ramp validation."""

from __future__ import annotations

import argparse
import html
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task59_multifloor_ramp_gazebo_validation"
DEFAULT_TASK_DIR = (
    REPO_ROOT
    / "stage_outputs"
    / "rslg_slam"
    / SCENE_ID
    / "tasks"
    / TASK_NAME
)
DEFAULT_RUNTIME_INPUT = (
    DEFAULT_TASK_DIR
    / "multifloor_ramp_pack"
    / "runtime_inputs"
    / "00843_cross_floor_object_curtain_room14_multifloor_ramp_runtime_input.json"
)
DEFAULT_WORLD_OUTPUT = DEFAULT_TASK_DIR / "multifloor_ramp_pack" / "worlds" / "rslg_multifloor_ramp_turtlebot3_burger.generated.world"
CHECKED_IN_WORLD = REPO_ROOT / "tools" / "rslg_pipeline" / "gazebo" / "worlds" / "rslg_multifloor_ramp_turtlebot3_burger.world"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> Any:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding="utf-8"))


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def as_float(value: Any, default: float = 0.0) -> float:
    if value in {None, ""}:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def point_tuple(raw: dict[str, Any]) -> tuple[float, float, float]:
    return as_float(raw.get("x")), as_float(raw.get("y")), as_float(raw.get("z"))


def route_extents(waypoints: list[dict[str, Any]], floor_id: str) -> tuple[float, float, float, float]:
    points = [wp for wp in waypoints if wp.get("floor_id") == floor_id or (floor_id == "floor_1" and as_float(wp.get("z")) <= 0.05)]
    if not points:
        points = waypoints
    xs = [as_float(wp.get("x")) for wp in points]
    ys = [as_float(wp.get("y")) for wp in points]
    return min(xs), max(xs), min(ys), max(ys)


def expanded_box(extents: tuple[float, float, float, float], margin: float, min_size: tuple[float, float]) -> dict[str, float]:
    min_x, max_x, min_y, max_y = extents
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin
    size_x = max(max_x - min_x, min_size[0])
    size_y = max(max_y - min_y, min_size[1])
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    return {"center_x": center_x, "center_y": center_y, "size_x": size_x, "size_y": size_y}


def floor_box_xml(name: str, box: dict[str, float], top_z: float, color: tuple[float, float, float, float]) -> str:
    thickness = 0.08
    pose_z = top_z - thickness * 0.5
    r, g, b, a = color
    return f"""    <model name=\"{html.escape(name)}\">
      <static>true</static>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box><size>{box['size_x']:.6f} {box['size_y']:.6f} {thickness:.6f}</size></box>
          </geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
          </surface>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box><size>{box['size_x']:.6f} {box['size_y']:.6f} {thickness:.6f}</size></box>
          </geometry>
          <material>
            <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          </material>
        </visual>
      </link>
      <pose>{box['center_x']:.6f} {box['center_y']:.6f} {pose_z:.6f} 0 0 0</pose>
    </model>
"""


def ramp_slab_xml(index: int, a: tuple[float, float, float], b: tuple[float, float, float]) -> str:
    ax, ay, az = a
    bx, by, bz = b
    dx = bx - ax
    dy = by - ay
    dz = bz - az
    run = math.hypot(dx, dy)
    length = math.sqrt(run * run + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, run)
    thickness = 0.08
    width = 1.05
    pose_x = (ax + bx) * 0.5
    pose_y = (ay + by) * 0.5
    pose_z = (az + bz) * 0.5 - math.cos(abs(pitch)) * thickness * 0.5
    return f"""    <model name=\"rslg_ramp_surrogate_slab_{index:02d}\">
      <static>true</static>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box><size>{length:.6f} {width:.6f} {thickness:.6f}</size></box>
          </geometry>
          <surface>
            <friction><ode><mu>1.4</mu><mu2>1.4</mu2></ode></friction>
          </surface>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box><size>{length:.6f} {width:.6f} {thickness:.6f}</size></box>
          </geometry>
          <material>
            <ambient>0.34 0.60 0.92 1</ambient>
            <diffuse>0.34 0.60 0.92 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>
    </model>
"""


def turn_platform_xml(index: int, point: tuple[float, float, float]) -> str:
    x, y, z = point
    thickness = 0.06
    size = 1.15
    pose_z = z - thickness * 0.5
    return f"""    <model name=\"rslg_ramp_turn_platform_{index:02d}\">
      <static>true</static>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box><size>{size:.6f} {size:.6f} {thickness:.6f}</size></box>
          </geometry>
          <surface>
            <friction><ode><mu>1.4</mu><mu2>1.4</mu2></ode></friction>
          </surface>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box><size>{size:.6f} {size:.6f} {thickness:.6f}</size></box>
          </geometry>
          <material>
            <ambient>0.30 0.72 0.70 1</ambient>
            <diffuse>0.30 0.72 0.70 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>{x:.6f} {y:.6f} {pose_z:.6f} 0 0 0</pose>
    </model>
"""


def marker_post_xml(name: str, x: float, y: float, z: float, color: tuple[float, float, float, float]) -> str:
    r, g, b, a = color
    return f"""    <model name=\"{html.escape(name)}\">
      <static>true</static>
      <link name=\"link\">
        <visual name=\"visual\">
          <geometry><cylinder><radius>0.08</radius><length>0.45</length></cylinder></geometry>
          <material>
            <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          </material>
        </visual>
      </link>
      <pose>{x:.6f} {y:.6f} {z + 0.225:.6f} 0 0 0</pose>
    </model>
"""


def build_world(runtime_input: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    waypoints = runtime_input.get("waypoints") or runtime_input.get("runtime_waypoints") or []
    control_points = [point_tuple(point) for point in (runtime_input.get("ramp_geometry") or {}).get("control_points") or []]
    if len(control_points) < 2:
        raise ValueError("runtime input does not contain ramp_geometry.control_points")

    floor_1_box = expanded_box(route_extents(waypoints, "floor_1"), margin=1.2, min_size=(11.5, 7.5))
    floor_2_box = expanded_box(route_extents(waypoints, "floor_2"), margin=1.2, min_size=(9.5, 7.5))
    first = waypoints[0]
    yaw = as_float(first.get("yaw"))
    floor_1_z = as_float(runtime_input.get("floor_1_z"), 0.0)
    floor_2_z = as_float(runtime_input.get("floor_2_z"), 1.6)
    xml_parts = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.6">',
        '  <world name="rslg_multifloor_ramp_turtlebot3_burger">',
        "    <gravity>0 0 -9.8</gravity>",
        '    <atmosphere type="adiabatic"/>',
        "",
        "    <include>",
        "      <uri>model://sun</uri>",
        "    </include>",
        "",
        floor_box_xml("rslg_floor_1_surface_z0", floor_1_box, floor_1_z, (0.74, 0.76, 0.78, 1.0)),
        floor_box_xml("rslg_floor_2_surface_z1_6", floor_2_box, floor_2_z, (0.68, 0.76, 0.62, 1.0)),
    ]
    for index in range(len(control_points) - 1):
        xml_parts.append(ramp_slab_xml(index, control_points[index], control_points[index + 1]))
    for index, point in enumerate(control_points[1:-1], start=1):
        xml_parts.append(turn_platform_xml(index, point))
    xml_parts.append(
        marker_post_xml(
            "rslg_vt_1_centerline_e001_ramp_start_marker",
            control_points[0][0],
            control_points[0][1],
            control_points[0][2],
            (0.55, 0.35, 1.0, 1.0),
        )
    )
    xml_parts.append(
        marker_post_xml(
            "rslg_generated_ring_002_final_approach_marker",
            as_float(waypoints[-1].get("x")),
            as_float(waypoints[-1].get("y")),
            as_float(waypoints[-1].get("z")),
            (0.1, 0.9, 0.35, 1.0),
        )
    )
    xml_parts.extend(
        [
            "    <include>",
            "      <uri>model://turtlebot3_burger</uri>",
            f"      <pose>{as_float(first.get('x')):.6f} {as_float(first.get('y')):.6f} {floor_1_z + 0.01:.6f} 0 0 {yaw:.6f}</pose>",
            "    </include>",
            "  </world>",
            "</sdf>",
            "",
        ]
    )
    summary = {
        "world_name": "rslg_multifloor_ramp_turtlebot3_burger",
        "spawn_pose": {"x": as_float(first.get("x")), "y": as_float(first.get("y")), "z": floor_1_z + 0.01, "yaw": yaw},
        "floor_1_box": floor_1_box,
        "floor_2_box": floor_2_box,
        "ramp_control_points": [
            {"x": point[0], "y": point[1], "z": point[2]} for point in control_points
        ],
        "ramp_segment_count": len(control_points) - 1,
        "turn_platform_count": max(0, len(control_points) - 2),
        "model_paths_required": [
            "/opt/ros/foxy/share/turtlebot3_gazebo/models",
            "/usr/share/gazebo-11/models",
            "tools/rslg_pipeline/gazebo/models",
        ],
        "online_model_dependency": False,
    }
    return "\n".join(xml_parts), summary


def world_design_markdown(runtime_input: dict[str, Any], world_path: Path, summary: dict[str, Any]) -> str:
    geometry = runtime_input["ramp_geometry"]
    return "\n".join(
        [
            "# Task59 Multifloor Ramp World Design",
            "",
            f"- Generated world: `{world_path}`",
            f"- Checked-in stable world: `{CHECKED_IN_WORLD}`",
            "- World type: simplified two-floor Gazebo world with a switchback ramp surrogate.",
            f"- Floor 1 top surface z: `{runtime_input['floor_1_z']}`",
            f"- Floor 2 top surface z: `{runtime_input['floor_2_z']}`",
            f"- Ramp connector semantic identity: `{runtime_input['ramp_connector_edge_id']}`",
            f"- Forbidden non-transition edge: `{runtime_input['forbidden_transition_edge_ids']}`",
            f"- Ramp equivalent slope: `{runtime_input['ramp_equivalent_slope_degrees']}` degrees",
            f"- Max ramp segment slope: `{runtime_input['ramp_slope_degrees']}` degrees",
            f"- Ramp segment count: `{summary['ramp_segment_count']}`",
            f"- Turn platform count: `{summary['turn_platform_count']}`",
            f"- Robot spawn pose: `{summary['spawn_pose']}`",
            f"- Floor 1 deck: `{summary['floor_1_box']}`",
            f"- Floor 2 deck: `{summary['floor_2_box']}`",
            "",
            "## Ramp Control Points",
            "",
            "```json",
            json.dumps(geometry["control_points"], indent=2),
            "```",
            "",
            "The ramp is a Gazebo simulation surrogate for the already-planned RSLG-SLAM connector. "
            "It is not physical stair climbing, not a real-robot deployment claim, and not a global "
            "collision-free guarantee.",
            "",
            "The world uses local model paths only: `/opt/ros/foxy/share/turtlebot3_gazebo/models`, "
            "`/usr/share/gazebo-11/models`, and `tools/rslg_pipeline/gazebo/models`.",
            "",
        ]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-input-json", type=Path, default=DEFAULT_RUNTIME_INPUT)
    parser.add_argument("--world-output", type=Path, default=DEFAULT_WORLD_OUTPUT)
    parser.add_argument("--task-dir", type=Path, default=DEFAULT_TASK_DIR)
    parser.add_argument("--write-design-md", action="store_true")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    runtime_input = read_json(args.runtime_input_json)
    world, summary = build_world(runtime_input)
    write_text(args.world_output, world)
    if args.write_design_md:
        write_text(args.task_dir / "04_multifloor_ramp_world_design.md", world_design_markdown(runtime_input, args.world_output, summary))
    print(json.dumps({"ok": True, "world_output": str(args.world_output), **summary}, indent=2, sort_keys=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
