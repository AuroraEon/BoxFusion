"""Robot profile loader for the lightweight object-navigation runner."""

from __future__ import annotations

from pathlib import Path
from typing import Any


REQUIRED_FIELDS = [
    "robot_name",
    "robot_type",
    "base_frame",
    "pose_source",
    "odom_frame",
    "map_frame",
    "cmd_topic",
    "cmd_msg_type",
    "command_model",
    "max_linear_speed",
    "max_angular_speed",
    "footprint_radius_m",
    "footprint_polygon",
    "inflation_radius_m",
    "min_clearance_m",
    "control_rate_hz",
    "yaw_drift_limit_m",
    "settle_time_sec",
    "supports_cmd_vel",
    "requires_gait_controller",
    "spawn_launch",
    "required_env",
    "known_limitations",
]


def load_robot_profile(path: str | Path) -> dict[str, Any]:
    """Load and validate a robot profile YAML file."""
    profile_path = Path(path).expanduser()
    if not profile_path.exists():
        raise FileNotFoundError(f"robot profile not found: {profile_path}")
    try:
        import yaml
    except Exception as exc:
        raise RuntimeError(
            "PyYAML is required to load robot profile YAML files; "
            "install the project requirements or provide PyYAML."
        ) from exc

    with profile_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"robot profile must be a YAML mapping: {profile_path}")
    profile = validate_robot_profile(data)
    profile["profile_path"] = str(profile_path)
    return profile


def validate_robot_profile(profile: dict[str, Any]) -> dict[str, Any]:
    """Validate required profile fields and return a shallow copy."""
    missing = [field for field in REQUIRED_FIELDS if field not in profile]
    if missing:
        raise ValueError(f"robot profile missing required fields: {', '.join(missing)}")
    if profile["pose_source"] != "tf":
        raise ValueError("lightweight robot adapters support only pose_source=tf")
    if profile["cmd_msg_type"] != "geometry_msgs/msg/Twist":
        raise ValueError("lightweight robot adapters support only geometry_msgs/msg/Twist commands")
    if not isinstance(profile.get("required_env"), dict):
        raise ValueError("robot profile required_env must be a mapping")
    if not isinstance(profile.get("known_limitations"), list):
        raise ValueError("robot profile known_limitations must be a list")
    return dict(profile)
