from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    artifact_path_arg = DeclareLaunchArgument(
        "artifact_path",
        default_value="",
        description="Path to a scene root, manifest.json, logs/summary.json, logs/online_topology_lifecycle_v0_1.json, or logs/topology_v0_1.json.",
    )
    service_prefix_arg = DeclareLaunchArgument(
        "service_prefix",
        default_value="/boxfusion/debug/publication",
        description="ROS service namespace prefix for debug publication diagnostics.",
    )
    stability_refresh_threshold_arg = DeclareLaunchArgument(
        "stability_refresh_threshold",
        default_value="2",
        description="Threshold used for deriving leave-like publication diagnostics from lifecycle payloads.",
    )

    diagnostics_node = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_publication_diagnostics_node",
        name="boxfusion_publication_diagnostics_node",
        output="screen",
        parameters=[
            {
                "artifact_path": LaunchConfiguration("artifact_path"),
                "service_prefix": LaunchConfiguration("service_prefix"),
                "stability_refresh_threshold": LaunchConfiguration("stability_refresh_threshold"),
            }
        ],
    )

    return LaunchDescription(
        [
            artifact_path_arg,
            service_prefix_arg,
            stability_refresh_threshold_arg,
            diagnostics_node,
        ]
    )
