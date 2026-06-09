"""Profile-driven robot adapter factory for the lightweight runner."""

from __future__ import annotations

from typing import Any

from .base_adapter import BaseRobotAdapter
from .quadruped_kinematic_proxy_adapter import QuadrupedKinematicProxyAdapter
from .turtlebot3_cmdvel_adapter import TurtleBot3CmdVelAdapter


ADAPTER_TYPES = {
    "turtlebot3_cmd_vel": TurtleBot3CmdVelAdapter,
    "quadruped_kinematic_proxy": QuadrupedKinematicProxyAdapter,
}


def adapter_type_for_profile(profile: dict[str, Any]) -> str:
    """Return the explicit adapter type, preserving the task20 default."""
    return str(profile.get("adapter_type") or "turtlebot3_cmd_vel")


def create_robot_adapter(profile: dict[str, Any], **kwargs: Any) -> BaseRobotAdapter:
    """Construct the adapter selected by the robot profile."""
    adapter_type = adapter_type_for_profile(profile)
    try:
        adapter_class = ADAPTER_TYPES[adapter_type]
    except KeyError as exc:
        raise ValueError(
            f"unsupported robot adapter_type={adapter_type!r}; "
            f"available: {', '.join(sorted(ADAPTER_TYPES))}"
        ) from exc
    return adapter_class(profile, **kwargs)
