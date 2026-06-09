"""Base robot adapter interface for lightweight object-navigation runtime."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class BaseRobotAdapter(ABC):
    """Interface owned by robot-specific runtime adapters."""

    def __init__(self, profile: dict[str, Any]) -> None:
        self.profile = profile

    @abstractmethod
    def start_bringup(self) -> dict[str, Any]:
        """Start simulator/robot bringup and write runtime command artifacts."""

    @abstractmethod
    def stop_bringup(self) -> dict[str, Any]:
        """Stop simulator/robot bringup."""

    @abstractmethod
    def wait_until_ready(self) -> dict[str, Any]:
        """Wait for and report runtime dataplane readiness."""

    @abstractmethod
    def get_pose(self) -> dict[str, Any] | None:
        """Return the current robot pose in the configured map frame."""

    @abstractmethod
    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        """Send a body/base velocity command."""

    @abstractmethod
    def stop(self) -> None:
        """Send stop commands appropriate for the robot."""

    @abstractmethod
    def collect_runtime_graph(self) -> dict[str, Any]:
        """Collect ROS graph evidence for runtime reporting."""

    @abstractmethod
    def validate_no_nav2(self) -> dict[str, Any]:
        """Report whether prohibited Nav2 actions/nodes/processes are absent."""

    @abstractmethod
    def report_dataplane_status(self) -> dict[str, Any]:
        """Report command, pose, and readiness dataplane status."""
