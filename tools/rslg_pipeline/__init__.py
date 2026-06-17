"""Canonical RSLG-SLAM pipeline tool namespace.

This package was introduced as a lightweight skeleton. It intentionally does
not migrate or invoke historical Stage-A, ROS, Gazebo, RViz, Nav2, or AMCL
runtime logic.
"""

__all__ = [
    "artifact_registry",
    "common",
    "validate_artifacts",
]
