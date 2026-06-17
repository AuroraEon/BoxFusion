#!/usr/bin/env python3
"""Run the task41 canonical cross-floor object route in controlled simulation."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_ID = "00843-DYehNKdT76V"
TASK_NAME = "task41_authorized_object_route_runtime_validation"
CANONICAL_ROOT = (
    REPO_ROOT / "stage_outputs/rslg_slam" / SCENE_ID / "canonical"
)
LAYER4 = CANONICAL_ROOT / "layer4_runtime_validation"
TASK_DIR = (
    REPO_ROOT / "stage_outputs/rslg_slam" / SCENE_ID / "tasks" / TASK_NAME
)
RUNTIME_INPUT = LAYER4 / "runtime_inputs/selected_route_runtime_input_v0_1.json"
RUNTIME_PACKAGE = LAYER4 / "runtime_inputs/runtime_input_package_v0_1.json"
MAP_INPUTS = LAYER4 / "runtime_inputs/map_server_inputs_v0_1.json"
OBJECT_RUNTIME_INPUT = (
    LAYER4 / "object_readiness/object_route_runtime_input_v0_1.json"
)
OBJECT_ROUTE = (
    CANONICAL_ROOT
    / "layer3_navigation_interface/real_routes/"
    / "cross_floor_object_real_astar_route_conservative_canonical_v0_2.json"
)
LAUNCHER = REPO_ROOT / "tools/object_nav/launch_lightweight_gazebo_turtlebot3.sh"
PROFILE = TASK_DIR / "runtime_configs/task41_lightweight_runtime_profile_v0_1.json"
RUNTIME_WORLD = (
    REPO_ROOT
    / "tools/rslg_pipeline/runtime_assets/task39_empty_runtime_support_world.sdf"
)
FLOOR_BINDINGS = TASK_DIR / "runtime_configs/task41_floor_segment_bindings_v0_1.json"
HANDOFF_PLAN = TASK_DIR / "runtime_configs/task41_topological_handoff_plan_v0_1.json"
GENERATED_RUNTIME_INPUT = (
    TASK_DIR / "runtime_inputs/task41_object_route_runtime_input_v0_1.json"
)
LOG_ROOT = TASK_DIR / "logs"
STDIO_ROOT = LOG_ROOT / "runtime_stdout_stderr"
TRAJECTORY_PATH = TASK_DIR / "trajectories/object_executed_trajectory_v0_1.json"
TRAJECTORY_SUMMARY_PATH = (
    TASK_DIR / "trajectories/object_executed_trajectory_summary_v0_1.json"
)
RUNTIME_RESULT_PATH = TASK_DIR / "task41_object_route_runtime_result_v0_1.json"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def rel(path: Path | str) -> str:
    path = Path(path)
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def run_capture(
    command: list[str],
    *,
    timeout: float = 30.0,
    env: dict[str, str] | None = None,
) -> dict[str, Any]:
    started = now_iso()
    try:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
        return {
            "command": command,
            "started_utc": started,
            "finished_utc": now_iso(),
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": command,
            "started_utc": started,
            "finished_utc": now_iso(),
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
            "timed_out": True,
        }


def angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def distance(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.hypot(
        float(right["x"]) - float(left["x"]),
        float(right["y"]) - float(left["y"]),
    )


def route_length(points: list[dict[str, Any]]) -> float:
    return sum(distance(left, right) for left, right in zip(points, points[1:]))


def project_onto_polyline(
    px: float, py: float, points: list[dict[str, Any]]
) -> tuple[float, float, float, int]:
    best_distance = float("inf")
    best_projection = (float(points[0]["x"]), float(points[0]["y"]))
    best_arc = 0.0
    best_segment = 0
    arc = 0.0
    for index in range(len(points) - 1):
        ax, ay = float(points[index]["x"]), float(points[index]["y"])
        bx, by = float(points[index + 1]["x"]), float(points[index + 1]["y"])
        dx, dy = bx - ax, by - ay
        segment_length = math.hypot(dx, dy)
        if segment_length < 1e-9:
            continue
        ratio = clamp(
            ((px - ax) * dx + (py - ay) * dy)
            / (segment_length * segment_length),
            0.0,
            1.0,
        )
        projection = (ax + ratio * dx, ay + ratio * dy)
        candidate_distance = math.hypot(px - projection[0], py - projection[1])
        if candidate_distance < best_distance:
            best_distance = candidate_distance
            best_projection = projection
            best_arc = arc + ratio * segment_length
            best_segment = index
        arc += segment_length
    return best_projection[0], best_projection[1], best_arc, best_segment


def advance_along_polyline(
    points: list[dict[str, Any]], arc_start: float, advance: float
) -> tuple[float, float]:
    target_arc = arc_start + advance
    arc = 0.0
    for index in range(len(points) - 1):
        ax, ay = float(points[index]["x"]), float(points[index]["y"])
        bx, by = float(points[index + 1]["x"]), float(points[index + 1]["y"])
        segment_length = math.hypot(bx - ax, by - ay)
        if arc + segment_length >= target_arc:
            ratio = (
                (target_arc - arc) / segment_length
                if segment_length > 1e-9
                else 0.0
            )
            return ax + ratio * (bx - ax), ay + ratio * (by - ay)
        arc += segment_length
    return float(points[-1]["x"]), float(points[-1]["y"])


def upcoming_turn_angle(
    points: list[dict[str, Any]],
    start_segment_index: int,
    lookahead_segments: int = 3,
) -> float:
    max_angle = 0.0
    first_corner = max(1, start_segment_index + 1)
    last_corner = min(len(points) - 1, first_corner + lookahead_segments)
    for corner_index in range(first_corner, last_corner):
        incoming = (
            float(points[corner_index]["x"])
            - float(points[corner_index - 1]["x"]),
            float(points[corner_index]["y"])
            - float(points[corner_index - 1]["y"]),
        )
        outgoing = (
            float(points[corner_index + 1]["x"])
            - float(points[corner_index]["x"]),
            float(points[corner_index + 1]["y"])
            - float(points[corner_index]["y"]),
        )
        incoming_norm = math.hypot(*incoming)
        outgoing_norm = math.hypot(*outgoing)
        if incoming_norm <= 1.0e-9 or outgoing_norm <= 1.0e-9:
            continue
        dot = (
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
        ) / (incoming_norm * outgoing_norm)
        max_angle = max(max_angle, math.acos(clamp(dot, -1.0, 1.0)))
    return max_angle


def parse_map_yaml(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        result[key.strip()] = value.strip().strip('"')
    return result


def parse_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index : index + 1].isspace():
            index += 1
        if data[index : index + 1] == b"#":
            while index < len(data) and data[index : index + 1] != b"\n":
                index += 1
            continue
        start = index
        while index < len(data) and not data[index : index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    if tokens[0] != b"P5" or int(tokens[3]) > 255:
        raise ValueError(f"unsupported PGM encoding: {path}")
    width, height = int(tokens[1]), int(tokens[2])
    while index < len(data) and data[index : index + 1].isspace():
        index += 1
    pixels = data[index : index + width * height]
    if len(pixels) != width * height:
        raise ValueError(f"truncated PGM: {path}")
    return width, height, pixels


class MapSampler:
    def __init__(self, yaml_path: Path) -> None:
        self.yaml_path = yaml_path
        self.meta = parse_map_yaml(yaml_path)
        image = Path(self.meta["image"])
        self.image_path = image if image.is_absolute() else yaml_path.parent / image
        self.width, self.height, self.pixels = parse_pgm(self.image_path)
        self.resolution = float(self.meta["resolution"])
        origin = self.meta["origin"].strip("[]").split(",")
        self.origin = (float(origin[0]), float(origin[1]))

    def sample(self, x: float, y: float) -> dict[str, Any]:
        col = int(round((x - self.origin[0]) / self.resolution))
        grid_row = int(round((y - self.origin[1]) / self.resolution))
        image_row = self.height - 1 - grid_row
        in_bounds = 0 <= col < self.width and 0 <= image_row < self.height
        value = self.pixels[image_row * self.width + col] if in_bounds else None
        return {
            "grid_col": col,
            "grid_row": grid_row,
            "in_bounds": in_bounds,
            "occupancy_pixel": value,
            "free_by_canonical_planner_convention": bool(
                in_bounds and value is not None and value >= 250
            ),
        }


def yaw_between_points(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.atan2(float(right["y"]) - float(left["y"]), float(right["x"]) - float(left["x"]))


def build_floor_sequences(
    object_route: dict[str, Any],
    object_runtime_input: dict[str, Any],
) -> dict[str, list[dict[str, Any]]]:
    sequences: dict[str, list[dict[str, Any]]] = {"floor_1": [], "floor_2": []}
    waypoint_index = {"floor_1": 0, "floor_2": 0}
    for segment in object_route.get("same_floor_segments", []):
        floor_id = segment.get("floor_id")
        if floor_id not in sequences:
            continue
        segment_points = [dict(point) for point in segment.get("waypoints", [])]
        if not segment_points:
            raise ValueError(f"empty waypoint segment: {segment.get('segment_id')}")
        if sequences[floor_id]:
            previous = sequences[floor_id][-1]
            first = segment_points[0]
            if distance(previous, first) <= 1.0e-6:
                segment_points = segment_points[1:]
        for point in segment_points:
            sequences[floor_id].append(
                {
                    "waypoint_index": waypoint_index[floor_id],
                    "floor_id": floor_id,
                    "segment_id": segment["segment_id"],
                    "x": float(point["x"]),
                    "y": float(point["y"]),
                    "yaw": 0.0,
                    "source": "canonical_layer3_cross_floor_object_route",
                }
            )
            waypoint_index[floor_id] += 1

    approach_goal = object_runtime_input["approach_goal"]
    approach_yaw = float(approach_goal["yaw"])
    for floor_id, points in sequences.items():
        if not points:
            raise ValueError(f"missing route sequence: {floor_id}")
        for index, point in enumerate(points):
            if index + 1 < len(points):
                point["yaw"] = round(yaw_between_points(point, points[index + 1]), 6)
            elif floor_id == "floor_2":
                point["yaw"] = round(approach_yaw, 6)
            elif len(points) > 1:
                point["yaw"] = points[index - 1]["yaw"]
    return sequences


def build_runtime_input(
    object_route: dict[str, Any],
    object_runtime_input: dict[str, Any],
) -> dict[str, Any]:
    floor_sequences = build_floor_sequences(object_route, object_runtime_input)
    return {
        "schema_name": "rslg_task41_object_route_runtime_input",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "route_id": "cross_floor_object",
        "route_kind": "cross_floor_object",
        "selected_profile": "conservative_canonical",
        "source_route_artifact": rel(OBJECT_ROUTE),
        "source_object_runtime_input": rel(OBJECT_RUNTIME_INPUT),
        "target_object": {
            "query": object_runtime_input.get("query", "curtain in room_14 on floor_2"),
            "object_id": object_runtime_input["object_id"],
            "object_label": object_runtime_input.get("object_label", "curtain"),
            "target_floor": "floor_2",
            "target_room": "room_14",
        },
        "approach_candidate": {
            "candidate_id": object_runtime_input["approach_candidate_id"],
            "goal": object_runtime_input["approach_goal"],
            "runtime_goal_is_object_centroid": False,
        },
        "vertical_transition": {
            "connector_id": "vt_1",
            "connector_id_alias": "vc_vt_1",
            "source_floor": "floor_1",
            "target_floor": "floor_2",
            "transition_edge": "vt_1_centerline_e001",
            "non_transition_edge": "vt_1_centerline_e003",
            "handling": "topological_handoff_between_floor_specific_executors",
            "physical_stair_motion_command_generated": False,
        },
        "validated_route_chain": [
            "room_2 on floor_1",
            "room_3 on floor_1",
            "vt_1 / vc_vt_1 connector",
            "room_7 on floor_2",
            "room_13 on floor_2",
            "room_14 on floor_2",
            "generated_ring_002 approach candidate for obj_175",
        ],
        "floor_sequences": floor_sequences,
        "object_centroid_navigation_used": False,
        "direct_object_centroid_goal_used": False,
        "manual_target_pose_used": False,
        "generated_ring_037_used": False,
        "navigation_thr0p25_candidate_selected": False,
    }


def validate_inputs() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    for path in (
        RUNTIME_PACKAGE,
        MAP_INPUTS,
        OBJECT_RUNTIME_INPUT,
        OBJECT_ROUTE,
        LAUNCHER,
        RUNTIME_WORLD,
    ):
        if not path.exists() or path.stat().st_size == 0:
            raise FileNotFoundError(path)
    object_runtime_input = read_json(OBJECT_RUNTIME_INPUT)
    object_route = read_json(OBJECT_ROUTE)
    route = build_runtime_input(object_route, object_runtime_input)
    write_json(GENERATED_RUNTIME_INPUT, route)
    package = read_json(RUNTIME_PACKAGE)
    maps = read_json(MAP_INPUTS)
    if route.get("route_id") != "cross_floor_object":
        raise ValueError("task41 accepts only route_id=cross_floor_object")
    if route.get("selected_profile") != "conservative_canonical":
        raise ValueError("task41 accepts only profile=conservative_canonical")
    if object_route.get("route_id") != "cross_floor_object":
        raise ValueError("object route artifact must be route_id=cross_floor_object")
    if object_route.get("profile_id") != "conservative_canonical":
        raise ValueError("object route artifact must use conservative_canonical")
    if object_runtime_input.get("approach_candidate_id") != "generated_ring_002":
        raise ValueError("task41 requires generated_ring_002")
    if object_runtime_input.get("approach_candidate_id") == "generated_ring_037":
        raise ValueError("generated_ring_037 must not be used")
    for key in (
        "object_centroid_navigation_used",
        "direct_object_centroid_goal_used",
        "manual_target_pose_used",
    ):
        if object_runtime_input.get(key):
            raise ValueError(f"forbidden object target flag is set: {key}")
    connector = route.get("vertical_transition") or {}
    if connector.get("transition_edge") != "vt_1_centerline_e001":
        raise ValueError("transition edge truth mismatch")
    if connector.get("non_transition_edge") != "vt_1_centerline_e003":
        raise ValueError("non-transition edge truth mismatch")
    for floor_id in ("floor_1", "floor_2"):
        map_yaml = REPO_ROOT / maps["maps"][floor_id]["packaged_yaml"]
        sampler = MapSampler(map_yaml)
        if not sampler.image_path.exists():
            raise FileNotFoundError(sampler.image_path)
        if not route["floor_sequences"].get(floor_id):
            raise ValueError(f"missing route sequence: {floor_id}")
    return route, package, maps


def binding_payloads(
    route: dict[str, Any], maps: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    floor_bindings = {
        "schema_name": "rslg_task41_floor_segment_bindings",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "adapter": rel(Path(__file__)),
        "selected_route": "cross_floor_object",
        "selected_profile": "conservative_canonical",
        "segments": {},
    }
    room_sequences = {
        "floor_1": ["room_2", "room_3", "vt_1_floor_1_entry"],
        "floor_2": [
            "vt_1_floor_2_exit",
            "room_7",
            "room_13",
            "room_14",
            "generated_ring_002",
        ],
    }
    for floor_id in ("floor_1", "floor_2"):
        waypoints = route["floor_sequences"][floor_id]
        floor_bindings["segments"][floor_id] = {
            "map_yaml": maps["maps"][floor_id]["packaged_yaml"],
            "waypoint_count": len(waypoints),
            "room_sequence": room_sequences[floor_id],
            "start_pose": {
                "x": waypoints[0]["x"],
                "y": waypoints[0]["y"],
                "yaw": waypoints[0]["yaw"],
            },
            "terminal_pose": {
                "x": waypoints[-1]["x"],
                "y": waypoints[-1]["y"],
                "yaw": waypoints[-1]["yaw"],
            },
            "waypoints": waypoints,
        }
    handoff = {
        "schema_name": "rslg_task41_topological_handoff_plan",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "connector_id": "vt_1",
        "connector_id_alias": "vc_vt_1",
        "source_floor": "floor_1",
        "target_floor": "floor_2",
        "transition_edge": "vt_1_centerline_e001",
        "non_transition_edge": "vt_1_centerline_e003",
        "handoff_actions": [
            "stop floor_1 velocity execution at the connector entry",
            "load the canonical floor_2 map through nav2_map_server",
            "reset the controlled simulation robot to the canonical floor_2 connector exit",
            "resume the canonical floor_2 route",
        ],
        "physical_stair_climbing_claimed": False,
        "manual_connector_geometry_added": False,
    }
    profile = {
        "schema_name": "rslg_task41_lightweight_runtime_profile",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "gazebo_world": rel(RUNTIME_WORLD),
        "gazebo_world_role": (
            "runtime robot dynamics support only; not a world-model, map, "
            "navmesh, geometry, or collision-validity source"
        ),
        "spawn_pose": floor_bindings["segments"]["floor_1"]["start_pose"],
        "map_frame": "map",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "cmd_vel_topic": "/cmd_vel",
        "pose_source": "/tf map->base_footprint",
        "localization_mode": "static_map_to_odom_controlled_simulation_alignment",
        "amcl_used": False,
        "ros_domain_id": 84,
        "turtlebot3_model": "burger",
    }
    return floor_bindings, handoff, profile


def prepare_bindings() -> dict[str, Any]:
    route, package, maps = validate_inputs()
    floor_bindings, handoff, profile = binding_payloads(route, maps)
    write_json(FLOOR_BINDINGS, floor_bindings)
    write_json(HANDOFF_PLAN, handoff)
    write_json(PROFILE, profile)
    return {
        "status": "compatible_with_narrow_adapter",
        "runtime_input_package": rel(RUNTIME_PACKAGE),
        "selected_route_runtime_input": rel(GENERATED_RUNTIME_INPUT),
        "floor_bindings": rel(FLOOR_BINDINGS),
        "handoff_plan": rel(HANDOFF_PLAN),
        "runtime_profile": rel(PROFILE),
        "floor_1_waypoint_count": len(route["floor_sequences"]["floor_1"]),
        "floor_2_waypoint_count": len(route["floor_sequences"]["floor_2"]),
        "approach_candidate_id": route["approach_candidate"]["candidate_id"],
        "approach_goal": route["approach_candidate"]["goal"],
        "object_route_attempted": True,
        "navigation_thr0p25_candidate_selected": False,
        "package_runtime_authorization_before_task41": package.get(
            "runtime_authorization_detected"
        ),
    }


def yaw_quaternion(yaw: float) -> dict[str, float]:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def reset_robot(
    target: dict[str, Any], env: dict[str, str], timeout: float = 20.0
) -> dict[str, Any]:
    services = run_capture(["ros2", "service", "list"], timeout=10, env=env)
    names = set(services.get("stdout", "").splitlines())
    quaternion = yaw_quaternion(float(target.get("yaw", 0.0)))
    request = (
        "{state: {"
        "name: 'turtlebot3_burger', "
        f"pose: {{position: {{x: {float(target['x'])}, y: {float(target['y'])}, z: 0.08}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {quaternion['z']}, w: {quaternion['w']}}}}}, "
        "twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, "
        "angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: 'world'}}"
    )
    for service in ("/set_entity_state", "/gazebo/set_entity_state"):
        if service not in names:
            continue
        result = run_capture(
            [
                "ros2",
                "service",
                "call",
                service,
                "gazebo_msgs/srv/SetEntityState",
                request,
            ],
            timeout=timeout,
            env=env,
        )
        result["service"] = service
        normalized_stdout = result.get("stdout", "").lower()
        result["success"] = bool(
            result.get("returncode") == 0
            and (
                "success: true" in normalized_stdout
                or "success=true" in normalized_stdout
            )
        )
        result["stdout"] = result.get("stdout", "")[-2000:]
        result["stderr"] = result.get("stderr", "")[-2000:]
        return result
    return {
        "success": False,
        "failure_reason": "Gazebo SetEntityState service unavailable",
        "available_services": sorted(names),
    }


def lifecycle_map_server(
    map_yaml: Path, env: dict[str, str], log_handle: Any
) -> tuple[subprocess.Popen[str], list[dict[str, Any]]]:
    command = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_server",
        "--ros-args",
        "-r",
        "__node:=map_server",
        "-p",
        f"yaml_filename:={map_yaml}",
        "-p",
        "use_sim_time:=true",
    ]
    process = subprocess.Popen(
        command,
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    operations: list[dict[str, Any]] = [
        {"operation": "start_map_server", "command": command, "pid": process.pid}
    ]
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        nodes = run_capture(["ros2", "node", "list"], timeout=5, env=env)
        if "/map_server" in nodes.get("stdout", "").splitlines():
            break
        time.sleep(0.5)
    for transition in ("configure", "activate"):
        result = run_capture(
            ["ros2", "lifecycle", "set", "/map_server", transition],
            timeout=15,
            env=env,
        )
        result["operation"] = f"map_server_{transition}"
        operations.append(result)
        if result.get("returncode") != 0:
            raise RuntimeError(f"map_server lifecycle {transition} failed")
    topic_info = run_capture(
        ["ros2", "topic", "info", "/map", "-v"], timeout=15, env=env
    )
    topic_info["operation"] = "map_topic_endpoint_probe"
    operations.append(topic_info)
    return process, operations


def switch_map(map_yaml: Path, env: dict[str, str]) -> dict[str, Any]:
    request = "{map_url: '" + map_yaml.as_posix() + "'}"
    result = run_capture(
        [
            "ros2",
            "service",
            "call",
            "/map_server/load_map",
            "nav2_msgs/srv/LoadMap",
            request,
        ],
        timeout=30,
        env=env,
    )
    result["map_yaml"] = rel(map_yaml)
    result["success"] = bool(
        result.get("returncode") == 0 and "result=0" in result.get("stdout", "")
    )
    result["response_summary"] = (
        "LoadMap returned RESULT_SUCCESS"
        if result["success"]
        else "LoadMap did not return RESULT_SUCCESS"
    )
    result["stdout"] = result.get("stdout", "")[-2000:]
    result["stderr"] = result.get("stderr", "")[-2000:]
    return result


def execute_runtime(args: argparse.Namespace) -> dict[str, Any]:
    if os.environ.get("RSLG_TASK38_ALLOW_RUNTIME") != "1":
        raise PermissionError("RSLG_TASK38_ALLOW_RUNTIME=1 is required")
    if os.environ.get("RSLG_TASK41_ALLOW_RUNTIME") != "1":
        raise PermissionError("RSLG_TASK41_ALLOW_RUNTIME=1 is required")
    if Path(sys.executable).resolve() != Path("/usr/bin/python3").resolve():
        raise RuntimeError("runtime execution must use /usr/bin/python3")

    binding_summary = prepare_bindings()
    route = read_json(GENERATED_RUNTIME_INPUT)
    maps = read_json(MAP_INPUTS)
    floor_bindings = read_json(FLOOR_BINDINGS)["segments"]
    env = dict(os.environ)
    env.update(
        {
            "ROS_DOMAIN_ID": "84",
            "TURTLEBOT3_MODEL": "burger",
            "RSLG_TASK38_ALLOW_RUNTIME": "1",
            "RSLG_TASK41_ALLOW_RUNTIME": "1",
        }
    )
    STDIO_ROOT.mkdir(parents=True, exist_ok=True)
    launch_log = STDIO_ROOT / "task41_gazebo_bringup.log"
    map_log = STDIO_ROOT / "task41_map_server.log"
    adapter_log = STDIO_ROOT / "task41_runtime_adapter.log"
    command_records: list[dict[str, Any]] = []
    trajectory: list[dict[str, Any]] = []
    started = now_iso()
    result: dict[str, Any] = {
        "schema_name": "rslg_task41_runtime_result",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "started_utc": started,
        "runtime_authorized": True,
        "runtime_attempted": True,
        "adapter": rel(Path(__file__)),
        "selected_route": "cross_floor_object",
        "selected_profile": "conservative_canonical",
        "binding_summary": binding_summary,
        "floor_1_segment_status": "not_started",
        "floor_transition_handoff_status": "not_started",
        "floor_2_segment_status": "not_started",
        "route_completion_status": "not_completed",
        "object_route_attempted": True,
        "object_runtime_goal_candidate_id": "generated_ring_002",
        "blocked_candidate_generated_ring_037_used": False,
        "object_centroid_navigation_used": False,
        "manual_target_pose_used": False,
        "object_executable_approach_claimed": False,
        "object_visual_confirmation_claimed": False,
        "full_object_navigation_benchmark_claimed": False,
        "AMCL_success_claimed": False,
        "real_robot_claimed": False,
        "physical_stair_climbing_claimed": False,
        "collision_free_guarantee_claimed": False,
        "runtime_components_launched": [],
        "exact_blockers": [],
    }

    launcher_command = [
        str(LAUNCHER),
        "--stage-output-dir",
        str(LAYER4),
        "--floor-id",
        "floor_1",
        "--map-yaml",
        str(REPO_ROOT / maps["maps"]["floor_1"]["packaged_yaml"]),
        "--runtime-profile",
        str(PROFILE),
        "--ros-domain-id",
        "84",
        "--log-dir",
        str(STDIO_ROOT / "task41_gazebo"),
        "--headless",
    ]
    stop_command = launcher_command[:-1] + ["--stop"]
    map_process: subprocess.Popen[str] | None = None

    try:
        launch = run_capture(launcher_command, timeout=120, env=env)
        command_records.append(launch)
        write_text(
            launch_log,
            (launch.get("stdout") or "") + "\n" + (launch.get("stderr") or ""),
        )
        if launch.get("returncode") != 0:
            raise RuntimeError("controlled Gazebo/TurtleBot3 bringup failed")
        result["runtime_components_launched"].extend(
            ["Gazebo headless", "TurtleBot3 Burger", "robot_state_publisher", "static map -> odom TF"]
        )

        with map_log.open("w", encoding="utf-8") as map_handle:
            floor1_map = REPO_ROOT / maps["maps"]["floor_1"]["packaged_yaml"]
            map_process, map_operations = lifecycle_map_server(
                floor1_map, env, map_handle
            )
            command_records.extend(map_operations)
            result["runtime_components_launched"].append("nav2_map_server")

            ros_nodes_before = run_capture(
                ["ros2", "node", "list"], timeout=15, env=env
            )
            ros_topics_before = run_capture(
                ["ros2", "topic", "list", "-t"], timeout=15, env=env
            )
            command_records.extend([ros_nodes_before, ros_topics_before])
            write_text(LOG_ROOT / "ros_node_list.txt", ros_nodes_before["stdout"])
            write_text(LOG_ROOT / "ros_topic_list.txt", ros_topics_before["stdout"])

            import rclpy
            from gazebo_msgs.msg import ModelStates
            from geometry_msgs.msg import Point, Twist
            from nav_msgs.msg import Odometry
            from rclpy.duration import Duration
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from std_msgs.msg import ColorRGBA
            from tf2_ros import Buffer, TransformException, TransformListener
            from visualization_msgs.msg import Marker, MarkerArray

            def yaw_from_quaternion(quaternion: Any) -> float:
                return math.atan2(
                    2.0
                    * (
                        quaternion.w * quaternion.z
                        + quaternion.x * quaternion.y
                    ),
                    1.0
                    - 2.0
                    * (
                        quaternion.y * quaternion.y
                        + quaternion.z * quaternion.z
                    ),
                )

            class RuntimeNode(Node):
                def __init__(self) -> None:
                    super().__init__("rslg_task41_cross_floor_runtime_adapter")
                    self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
                    self.tf_listener = TransformListener(self.tf_buffer, self)
                    self.odom: dict[str, Any] | None = None
                    self.gazebo: dict[str, Any] | None = None
                    self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
                    self.create_subscription(Odometry, "/odom", self.on_odom, 20)
                    self.create_subscription(
                        ModelStates, "/gazebo/model_states", self.on_models, 10
                    )
                    qos = QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST,
                    )
                    self.route_pub = self.create_publisher(
                        MarkerArray, "/rslg/layer4/selected_route", qos
                    )
                    self.anchor_pub = self.create_publisher(
                        MarkerArray, "/rslg/layer4/anchors", qos
                    )
                    self.connector_pub = self.create_publisher(
                        MarkerArray, "/rslg/layer4/connector", qos
                    )
                    self.readiness_pub = self.create_publisher(
                        MarkerArray, "/rslg/layer4/readiness", qos
                    )
                    self.last_overlay = 0.0

                def on_odom(self, message: Any) -> None:
                    pose = message.pose.pose
                    self.odom = {
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "yaw": yaw_from_quaternion(pose.orientation),
                        "source": "/odom",
                        "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec),
                    }

                def on_models(self, message: Any) -> None:
                    for index, name in enumerate(message.name):
                        if name == "turtlebot3_burger":
                            pose = message.pose[index]
                            self.gazebo = {
                                "x": float(pose.position.x),
                                "y": float(pose.position.y),
                                "yaw": yaw_from_quaternion(pose.orientation),
                                "source": "/gazebo/model_states",
                                "stamp_ns": time.monotonic_ns(),
                            }
                            return

                def pose(self) -> dict[str, Any] | None:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            "map",
                            "base_footprint",
                            rclpy.time.Time(),
                            timeout=Duration(seconds=0.15),
                        )
                        return {
                            "x": float(transform.transform.translation.x),
                            "y": float(transform.transform.translation.y),
                            "yaw": yaw_from_quaternion(
                                transform.transform.rotation
                            ),
                            "source": "/tf map->base_footprint",
                            "stamp_ns": int(transform.header.stamp.sec)
                            * 1_000_000_000
                            + int(transform.header.stamp.nanosec),
                        }
                    except TransformException:
                        return self.gazebo or self.odom

                def command(self, linear: float, angular: float) -> None:
                    message = Twist()
                    message.linear.x = float(linear)
                    message.angular.z = float(angular)
                    self.cmd.publish(message)

                def stop(self) -> None:
                    for _ in range(5):
                        self.command(0.0, 0.0)
                        rclpy.spin_once(self, timeout_sec=0.03)

                @staticmethod
                def color(red: float, green: float, blue: float, alpha: float = 1.0) -> Any:
                    return ColorRGBA(r=red, g=green, b=blue, a=alpha)

                @staticmethod
                def point(x: float, y: float, z: float) -> Any:
                    return Point(x=float(x), y=float(y), z=float(z))

                def line_marker(
                    self,
                    marker_id: int,
                    namespace: str,
                    points: list[dict[str, Any]],
                    color: Any,
                    z: float,
                ) -> Any:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = namespace
                    marker.id = marker_id
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.07
                    marker.color = color
                    marker.points = [
                        self.point(point["x"], point["y"], z) for point in points
                    ]
                    return marker

                def sphere_marker(
                    self,
                    marker_id: int,
                    namespace: str,
                    point: dict[str, Any],
                    color: Any,
                    z: float,
                    scale: float = 0.22,
                ) -> Any:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = namespace
                    marker.id = marker_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position = self.point(point["x"], point["y"], z)
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = marker.scale.y = marker.scale.z = scale
                    marker.color = color
                    return marker

                def publish_overlays(self, active_floor: str, force: bool = False) -> None:
                    if not force and time.monotonic() - self.last_overlay < 1.0:
                        return
                    self.last_overlay = time.monotonic()
                    floor1 = floor_bindings["floor_1"]["waypoints"]
                    floor2 = floor_bindings["floor_2"]["waypoints"]
                    route_markers = MarkerArray()
                    route_markers.markers = [
                        self.line_marker(
                            1, "task41/floor_1_route", floor1, self.color(0.1, 0.55, 1.0), 0.12
                        ),
                        self.line_marker(
                            2, "task41/floor_2_route", floor2, self.color(0.7, 0.25, 0.95), 0.18
                        ),
                    ]
                    if trajectory:
                        for index, floor_id in enumerate(("floor_1", "floor_2"), start=10):
                            points = [
                                sample
                                for sample in trajectory
                                if sample.get("floor_id") == floor_id
                                and sample.get("event") != "topological_handoff"
                            ]
                            if len(points) > 1:
                                route_markers.markers.append(
                                    self.line_marker(
                                        index,
                                        f"task41/{floor_id}_executed",
                                        points,
                                        self.color(0.1, 0.9, 0.25),
                                        0.28,
                                    )
                                )
                    anchors = MarkerArray()
                    anchors.markers = [
                        self.sphere_marker(
                            1, "task41/start", floor1[0], self.color(0.1, 0.9, 0.2), 0.35
                        ),
                        self.sphere_marker(
                            2, "task41/floor_1_terminal", floor1[-1], self.color(1.0, 0.65, 0.0), 0.35
                        ),
                        self.sphere_marker(
                            3, "task41/floor_2_start", floor2[0], self.color(1.0, 0.65, 0.0), 0.35
                        ),
                        self.sphere_marker(
                            4, "task41/generated_ring_002_object_approach", floor2[-1], self.color(0.95, 0.1, 0.15), 0.35
                        ),
                    ]
                    connector = MarkerArray()
                    connector.markers = [
                        self.line_marker(
                            1,
                            "task41/vt_1_centerline_e001_topological_handoff",
                            [floor1[-1], floor2[0]],
                            self.color(1.0, 0.45, 0.0),
                            0.5,
                        )
                    ]
                    readiness = MarkerArray()
                    readiness.markers = [
                        self.sphere_marker(
                            1,
                            f"task41/active_{active_floor}",
                            floor_bindings[active_floor]["waypoints"][0],
                            self.color(1.0, 1.0, 0.0),
                            0.65,
                            scale=0.12,
                        )
                    ]
                    self.route_pub.publish(route_markers)
                    self.anchor_pub.publish(anchors)
                    self.connector_pub.publish(connector)
                    self.readiness_pub.publish(readiness)

            rclpy.init(args=None)
            node = RuntimeNode()
            result["runtime_components_launched"].extend(
                [
                    "task41 direct velocity route executor",
                    "task41 MarkerArray overlay publisher",
                    "task41 TF/odom trajectory logger",
                ]
            )
            start_monotonic = time.monotonic()

            def observe(timeout: float = 15.0) -> dict[str, Any] | None:
                deadline = time.monotonic() + timeout
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    pose = node.pose()
                    node.publish_overlays("floor_1")
                    if pose:
                        return pose
                return None

            def append_sample(
                pose: dict[str, Any],
                floor_id: str,
                phase: str,
                linear: float,
                angular: float,
                target_index: int | None,
            ) -> None:
                trajectory.append(
                    {
                        "t_sec": round(time.monotonic() - start_monotonic, 6),
                        "floor_id": floor_id,
                        "phase": phase,
                        "x": round(float(pose["x"]), 6),
                        "y": round(float(pose["y"]), 6),
                        "yaw": round(float(pose["yaw"]), 6),
                        "pose_source": pose["source"],
                        "target_waypoint_index": target_index,
                        "cmd_vel_linear_x": round(linear, 6),
                        "cmd_vel_angular_z": round(angular, 6),
                    }
                )

            def follow_floor(floor_id: str) -> dict[str, Any]:
                floor_points = floor_bindings[floor_id]["waypoints"]
                started_floor = time.monotonic()
                segment_groups: list[tuple[str, list[dict[str, Any]]]] = []
                for point in floor_points:
                    segment_id = str(point["segment_id"])
                    if not segment_groups or segment_groups[-1][0] != segment_id:
                        segment_groups.append((segment_id, []))
                    segment_groups[-1][1].append(point)
                report: dict[str, Any] = {
                    "floor_id": floor_id,
                    "attempted": True,
                    "waypoint_count": len(floor_points),
                    "route_length_m": round(route_length(floor_points), 6),
                    "controller": (
                        "sequential_segment_adaptive_polyline_lookahead"
                    ),
                    "controller_success": False,
                    "start_pose": node.pose(),
                    "route_segment_results": [],
                }

                previous_terminal: dict[str, Any] | None = None
                for segment_id, canonical_points in segment_groups:
                    control_points = list(canonical_points)
                    if previous_terminal is not None:
                        control_points.insert(0, previous_terminal)
                    segment_started = time.monotonic()
                    last_stamp: int | None = None
                    previous_angular = 0.0
                    rotate_in_place = False
                    highest_progress_arc = 0.0
                    segment_report: dict[str, Any] = {
                        "segment_id": segment_id,
                        "attempted": True,
                        "canonical_waypoint_count": len(canonical_points),
                        "control_point_count": len(control_points),
                        "controller_success": False,
                    }
                    while (
                        rclpy.ok()
                        and time.monotonic() - started_floor
                        < args.floor_timeout_sec
                    ):
                        rclpy.spin_once(node, timeout_sec=0.03)
                        node.publish_overlays(floor_id)
                        pose = node.pose()
                        if pose is None:
                            segment_report["failure_reason"] = (
                                "pose feedback unavailable"
                            )
                            break
                        stamp = pose.get("stamp_ns")
                        if stamp == last_stamp:
                            time.sleep(0.02)
                            continue
                        last_stamp = stamp
                        px, py = float(pose["x"]), float(pose["y"])
                        (
                            projection_x,
                            projection_y,
                            progress_arc,
                            local_segment_index,
                        ) = project_onto_polyline(px, py, control_points)
                        highest_progress_arc = max(
                            highest_progress_arc, progress_arc
                        )
                        cross_track_error = math.hypot(
                            px - projection_x, py - projection_y
                        )
                        if cross_track_error > args.path_deviation_limit_m:
                            segment_report["failure_reason"] = (
                                f"path deviation {cross_track_error:.3f} m "
                                f"exceeds {args.path_deviation_limit_m:.3f} m"
                            )
                            break
                        final_distance = distance(
                            pose, canonical_points[-1]
                        )
                        if final_distance <= args.goal_tolerance_m:
                            node.stop()
                            append_sample(
                                pose,
                                floor_id,
                                segment_id,
                                0.0,
                                0.0,
                                int(canonical_points[-1]["waypoint_index"]),
                            )
                            segment_report.update(
                                {
                                    "controller_success": True,
                                    "duration_sec": round(
                                        time.monotonic() - segment_started, 6
                                    ),
                                    "final_pose": pose,
                                    "final_distance_m": round(
                                        final_distance, 6
                                    ),
                                    "highest_progress_m": round(
                                        highest_progress_arc, 6
                                    ),
                                }
                            )
                            break
                        turn_angle = upcoming_turn_angle(
                            control_points,
                            local_segment_index,
                        )
                        active_lookahead = args.lookahead_distance_m
                        if turn_angle > math.radians(26.0):
                            active_lookahead = min(active_lookahead, 0.35)
                        lookahead_x, lookahead_y = advance_along_polyline(
                            control_points,
                            progress_arc,
                            active_lookahead,
                        )
                        heading = math.atan2(
                            lookahead_y - py, lookahead_x - px
                        )
                        error = angle_wrap(
                            heading - float(pose["yaw"])
                        )
                        if not rotate_in_place and abs(error) > 0.85:
                            rotate_in_place = True
                        elif rotate_in_place and abs(error) < 0.55:
                            rotate_in_place = False
                        raw_angular = clamp(
                            args.heading_kp * error,
                            -args.max_angular_speed,
                            args.max_angular_speed,
                        )
                        angular = clamp(
                            0.35 * raw_angular
                            + 0.65 * previous_angular,
                            -args.max_angular_speed,
                            args.max_angular_speed,
                        )
                        previous_angular = angular
                        if rotate_in_place:
                            linear = 0.0
                        else:
                            linear = min(
                                args.max_linear_speed,
                                max(0.035, final_distance * 0.4),
                            )
                            linear *= max(0.0, math.cos(error))
                            if turn_angle > math.radians(26.0):
                                linear *= 0.55
                        node.command(linear, angular)
                        target_local_index = min(
                            local_segment_index + 1,
                            len(control_points) - 1,
                        )
                        append_sample(
                            pose,
                            floor_id,
                            segment_id,
                            linear,
                            angular,
                            int(
                                control_points[target_local_index][
                                    "waypoint_index"
                                ]
                            ),
                        )
                        time.sleep(0.125)
                    node.stop()
                    if not segment_report["controller_success"]:
                        final_pose = node.pose()
                        segment_report.update(
                            {
                                "duration_sec": round(
                                    time.monotonic() - segment_started, 6
                                ),
                                "final_pose": final_pose,
                                "final_distance_m": (
                                    round(
                                        distance(
                                            final_pose,
                                            canonical_points[-1],
                                        ),
                                        6,
                                    )
                                    if final_pose
                                    else None
                                ),
                                "highest_progress_m": round(
                                    highest_progress_arc, 6
                                ),
                                "failure_reason": segment_report.get(
                                    "failure_reason"
                                )
                                or "floor or segment execution timed out",
                            }
                        )
                    report["route_segment_results"].append(segment_report)
                    if not segment_report["controller_success"]:
                        report.update(
                            {
                                "failure_reason": (
                                    f"{segment_id}: "
                                    + str(
                                        segment_report.get(
                                            "failure_reason"
                                        )
                                    )
                                ),
                                "final_pose": segment_report.get(
                                    "final_pose"
                                ),
                                "final_distance_m": segment_report.get(
                                    "final_distance_m"
                                ),
                            }
                        )
                        return report
                    previous_terminal = canonical_points[-1]

                final_pose = node.pose()
                report.update(
                    {
                        "controller_success": True,
                        "final_pose": final_pose,
                        "final_distance_m": (
                            round(
                                distance(final_pose, floor_points[-1]), 6
                            )
                            if final_pose
                            else None
                        ),
                    }
                )
                return report

            def align_to_yaw(
                floor_id: str,
                target_yaw: float,
                tolerance_rad: float,
                timeout_sec: float,
            ) -> dict[str, Any]:
                started_align = time.monotonic()
                last_pose: dict[str, Any] | None = None
                while rclpy.ok() and time.monotonic() - started_align < timeout_sec:
                    rclpy.spin_once(node, timeout_sec=0.03)
                    node.publish_overlays(floor_id)
                    pose = node.pose()
                    if pose is None:
                        time.sleep(0.02)
                        continue
                    last_pose = pose
                    yaw_error = angle_wrap(target_yaw - float(pose["yaw"]))
                    if abs(yaw_error) <= tolerance_rad:
                        node.stop()
                        append_sample(
                            pose,
                            floor_id,
                            "object_goal_yaw_alignment",
                            0.0,
                            0.0,
                            None,
                        )
                        return {
                            "attempted": True,
                            "success": True,
                            "target_yaw_rad": round(target_yaw, 6),
                            "yaw_tolerance_rad": tolerance_rad,
                            "final_yaw_error_rad": round(yaw_error, 6),
                            "final_pose": pose,
                            "duration_sec": round(time.monotonic() - started_align, 6),
                        }
                    angular = clamp(
                        args.heading_kp * yaw_error,
                        -args.max_angular_speed,
                        args.max_angular_speed,
                    )
                    node.command(0.0, angular)
                    append_sample(
                        pose,
                        floor_id,
                        "object_goal_yaw_alignment",
                        0.0,
                        angular,
                        None,
                    )
                    time.sleep(0.1)
                node.stop()
                final_error = (
                    angle_wrap(target_yaw - float(last_pose["yaw"]))
                    if last_pose
                    else None
                )
                return {
                    "attempted": True,
                    "success": False,
                    "target_yaw_rad": round(target_yaw, 6),
                    "yaw_tolerance_rad": tolerance_rad,
                    "final_yaw_error_rad": (
                        round(final_error, 6) if final_error is not None else None
                    ),
                    "final_pose": last_pose,
                    "duration_sec": round(time.monotonic() - started_align, 6),
                    "failure_reason": "object approach yaw alignment timed out",
                }

            initial = observe()
            if initial is None:
                raise RuntimeError("no controlled simulation pose feedback")
            node.publish_overlays("floor_1", force=True)

            floor1 = follow_floor("floor_1")
            result["floor_1"] = floor1
            result["floor_1_segment_status"] = (
                "completed" if floor1["controller_success"] else "failed"
            )
            if not floor1["controller_success"]:
                raise RuntimeError(
                    "floor_1 route execution failed: "
                    + str(floor1.get("failure_reason"))
                )

            floor2_map = REPO_ROOT / maps["maps"]["floor_2"]["packaged_yaml"]
            map_switch = switch_map(floor2_map, env)
            command_records.append(map_switch)
            floor2_start = floor_bindings["floor_2"]["start_pose"]
            reset = reset_robot(floor2_start, env)
            command_records.append(reset)
            result["floor_transition_handoff"] = {
                "transition_edge": "vt_1_centerline_e001",
                "non_transition_edge": "vt_1_centerline_e003",
                "map_switch": map_switch,
                "simulation_pose_reset": reset,
                "physical_stair_climbing_claimed": False,
            }
            if not map_switch.get("success") or not reset.get("success"):
                result["floor_transition_handoff_status"] = "failed"
                raise RuntimeError("topological handoff map switch or pose reset failed")
            trajectory.append(
                {
                    "t_sec": round(time.monotonic() - start_monotonic, 6),
                    "floor_id": "floor_transition",
                    "phase": "topological_handoff",
                    "event": "topological_handoff",
                    "transition_edge": "vt_1_centerline_e001",
                    "source_floor": "floor_1",
                    "target_floor": "floor_2",
                    "physical_stair_climbing_claimed": False,
                }
            )
            convergence_deadline = time.monotonic() + 15.0
            converged_pose: dict[str, Any] | None = None
            while rclpy.ok() and time.monotonic() < convergence_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                candidate_pose = node.pose()
                node.publish_overlays("floor_2")
                if (
                    candidate_pose is not None
                    and distance(candidate_pose, floor2_start) <= 0.5
                ):
                    converged_pose = candidate_pose
                    break
            result["floor_transition_handoff"]["post_reset_pose_convergence"] = {
                "required": True,
                "tolerance_m": 0.5,
                "success": converged_pose is not None,
                "observed_pose": converged_pose,
            }
            if converged_pose is None:
                result["floor_transition_handoff_status"] = "failed"
                raise RuntimeError(
                    "TF did not converge to the floor_2 connector exit after reset"
                )
            result["floor_transition_handoff_status"] = "completed"
            node.publish_overlays("floor_2", force=True)

            floor2 = follow_floor("floor_2")
            result["floor_2"] = floor2
            result["floor_2_segment_status"] = (
                "completed" if floor2["controller_success"] else "failed"
            )
            if not floor2["controller_success"]:
                raise RuntimeError(
                    "floor_2 route execution failed: "
                    + str(floor2.get("failure_reason"))
                )
            approach_goal = route["approach_candidate"]["goal"]
            object_goal = {
                "x": float(approach_goal["world_xy"][0]),
                "y": float(approach_goal["world_xy"][1]),
                "yaw": float(approach_goal["yaw"]),
            }
            final_pose_before_yaw = node.pose()
            object_goal_distance = (
                distance(final_pose_before_yaw, object_goal)
                if final_pose_before_yaw
                else None
            )
            if object_goal_distance is None or object_goal_distance > args.goal_tolerance_m:
                raise RuntimeError(
                    "object approach endpoint not reached within runtime tolerance"
                )
            yaw_alignment = align_to_yaw(
                "floor_2",
                object_goal["yaw"],
                args.object_yaw_tolerance_rad,
                args.object_yaw_timeout_sec,
            )
            result["object_approach_goal"] = {
                "candidate_id": "generated_ring_002",
                "world_xy": approach_goal["world_xy"],
                "yaw": approach_goal["yaw"],
                "yaw_policy": approach_goal.get("yaw_policy"),
                "visible_object_footprint_proxy_xy": approach_goal.get(
                    "visible_object_footprint_proxy_xy"
                ),
                "distance_to_goal_before_yaw_alignment_m": round(
                    object_goal_distance, 6
                ),
                "endpoint_reached_within_runtime_tolerance": True,
                "yaw_alignment": yaw_alignment,
                "object_centroid_navigation_used": False,
                "manual_target_pose_used": False,
                "generated_ring_037_used": False,
            }
            if not yaw_alignment["success"]:
                raise RuntimeError(
                    "object approach yaw alignment failed: "
                    + str(yaw_alignment.get("failure_reason"))
                )
            result["route_completion_status"] = "completed"
            result["object_route_runtime_validation_status"] = (
                "generated_ring_002_reached_and_yaw_aligned_within_runtime_tolerance"
            )
            result["final_room_status"] = "room_14_reached_before_object_approach"
            result["object_executable_approach_claimed"] = True
            node.publish_overlays("floor_2", force=True)
            time.sleep(1.0)
            node.stop()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    except Exception as exc:
        blocker = f"{type(exc).__name__}: {exc}"
        result["exact_blockers"].append(blocker)
        result["route_completion_status"] = "failed"
    finally:
        write_json(
            TRAJECTORY_PATH,
            {
                "schema_name": "rslg_task41_executed_trajectory",
                "schema_version": "0.1",
                "project_name": "RSLG-SLAM",
                "scene_id": SCENE_ID,
                "artifact_layer": "Layer 4: Runtime Validation Layer",
                "selected_route": "cross_floor_object",
                "selected_profile": "conservative_canonical",
                "samples": trajectory,
                "object_route_attempted": True,
                "approach_candidate_id": "generated_ring_002",
                "object_centroid_navigation_used": False,
                "manual_target_pose_used": False,
                "physical_stair_climbing_claimed": False,
            },
        )
        summary = summarize_trajectory(trajectory, maps)
        write_json(TRAJECTORY_SUMMARY_PATH, summary)
        result["trajectory_available"] = bool(trajectory)
        result["trajectory_file"] = rel(TRAJECTORY_PATH) if trajectory else None
        result["trajectory_summary_file"] = rel(TRAJECTORY_SUMMARY_PATH)
        result["runtime_logs"] = [
            rel(launch_log),
            rel(map_log),
            rel(adapter_log),
            rel(LOG_ROOT / "ros_node_list.txt"),
            rel(LOG_ROOT / "ros_topic_list.txt"),
        ]
        result["finished_utc"] = now_iso()
        result["command_records"] = command_records
        write_json(RUNTIME_RESULT_PATH, result)
        write_text(
            adapter_log,
            json.dumps(
                {
                    "started_utc": started,
                    "finished_utc": result["finished_utc"],
                    "route_completion_status": result["route_completion_status"],
                    "exact_blockers": result["exact_blockers"],
                    "trajectory_samples": len(trajectory),
                },
                indent=2,
            ),
        )
        if map_process is not None:
            try:
                os.killpg(map_process.pid, signal.SIGTERM)
                map_process.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(map_process.pid, signal.SIGKILL)
                except Exception:
                    pass
        stop = run_capture(stop_command, timeout=30, env=env)
        command_records.append(stop)
        write_json(RUNTIME_RESULT_PATH, result)
    return result


def summarize_trajectory(
    trajectory: list[dict[str, Any]], maps: dict[str, Any]
) -> dict[str, Any]:
    floor_reports: dict[str, Any] = {}
    total_length = 0.0
    for floor_id in ("floor_1", "floor_2"):
        samples = [
            sample
            for sample in trajectory
            if sample.get("floor_id") == floor_id and sample.get("x") is not None
        ]
        sampler = MapSampler(
            REPO_ROOT / maps["maps"][floor_id]["packaged_yaml"]
        )
        sampled = [
            {
                "sample_index": index,
                **sampler.sample(float(sample["x"]), float(sample["y"])),
            }
            for index, sample in enumerate(samples)
        ]
        invalid = [
            sample
            for sample in sampled
            if not sample["free_by_canonical_planner_convention"]
        ]
        length = route_length(samples) if len(samples) > 1 else 0.0
        total_length += length
        floor_reports[floor_id] = {
            "sample_count": len(samples),
            "trajectory_length_m": round(length, 6),
            "canonical_map_yaml": rel(sampler.yaml_path),
            "occupied_unknown_or_out_of_bounds_sample_count": len(invalid),
            "wall_crossing_sample_validation_passed": not invalid,
            "invalid_samples": invalid[:100],
        }
    return {
        "schema_name": "rslg_task41_executed_trajectory_summary",
        "schema_version": "0.1",
        "project_name": "RSLG-SLAM",
        "scene_id": SCENE_ID,
        "artifact_layer": "Layer 4: Runtime Validation Layer",
        "selected_route": "cross_floor_object",
        "selected_profile": "conservative_canonical",
        "trajectory_available": bool(trajectory),
        "total_sample_count": len(trajectory),
        "floor_reports": floor_reports,
        "total_floor_trajectory_length_m": round(total_length, 6),
        "topological_handoff_event_count": len(
            [
                sample
                for sample in trajectory
                if sample.get("event") == "topological_handoff"
            ]
        ),
        "full_collision_free_guarantee_claimed": False,
        "object_route_attempted": True,
        "approach_candidate_id": "generated_ring_002",
        "object_centroid_navigation_used": False,
        "manual_target_pose_used": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--prepare-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--max-linear-speed", type=float, default=0.12)
    parser.add_argument("--max-angular-speed", type=float, default=0.38)
    parser.add_argument("--heading-kp", type=float, default=0.80)
    parser.add_argument("--forward-heading-limit-rad", type=float, default=0.48)
    parser.add_argument("--lookahead-distance-m", type=float, default=0.60)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.28)
    parser.add_argument("--object-yaw-tolerance-rad", type=float, default=0.20)
    parser.add_argument("--object-yaw-timeout-sec", type=float, default=20.0)
    parser.add_argument("--path-deviation-limit-m", type=float, default=0.80)
    parser.add_argument("--floor-timeout-sec", type=float, default=210.0)
    args = parser.parse_args()
    if args.prepare_only:
        print(json.dumps(prepare_bindings(), indent=2, sort_keys=True))
        return 0
    result = execute_runtime(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("route_completion_status") == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
