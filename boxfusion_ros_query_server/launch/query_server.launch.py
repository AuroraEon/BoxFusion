from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    artifact_path_arg = DeclareLaunchArgument(
        "artifact_path",
        default_value="",
        description="Path to a scene root, manifest.json, logs/summary.json, or logs/topology_v0_1.json.",
    )
    service_prefix_arg = DeclareLaunchArgument(
        "service_prefix",
        default_value="/boxfusion/query",
        description="ROS service namespace prefix.",
    )

    query_server_node = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_query_server_node",
        name="boxfusion_query_server_node",
        output="screen",
        parameters=[
            {
                "artifact_path": LaunchConfiguration("artifact_path"),
                "service_prefix": LaunchConfiguration("service_prefix"),
            }
        ],
    )

    return LaunchDescription(
        [
            artifact_path_arg,
            service_prefix_arg,
            query_server_node,
        ]
    )
