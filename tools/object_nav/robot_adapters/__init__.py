"""Robot profile and adapter layer for the lightweight object-nav runner."""

from .profile_loader import load_robot_profile, validate_robot_profile
from .adapter_factory import adapter_type_for_profile, create_robot_adapter
from .quadruped_kinematic_proxy_adapter import QuadrupedKinematicProxyAdapter
from .turtlebot3_cmdvel_adapter import TurtleBot3CmdVelAdapter

__all__ = [
    "QuadrupedKinematicProxyAdapter",
    "TurtleBot3CmdVelAdapter",
    "adapter_type_for_profile",
    "create_robot_adapter",
    "load_robot_profile",
    "validate_robot_profile",
]
