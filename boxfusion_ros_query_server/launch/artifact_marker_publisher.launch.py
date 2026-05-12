from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    scene_root_arg = DeclareLaunchArgument(
        "scene_root",
        description="Scene root containing the four committed/public BoxFusion artifacts under logs/.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="boxfusion_map",
        description="Artifact visualization frame id.",
    )
    marker_topic_prefix_arg = DeclareLaunchArgument(
        "marker_topic_prefix",
        default_value="/boxfusion/markers",
        description="Topic prefix for MarkerArray outputs.",
    )
    status_topic_arg = DeclareLaunchArgument(
        "status_topic",
        default_value="/boxfusion/artifacts/status",
        description="JSON status topic for the loaded committed/public artifact bundle.",
    )
    publish_period_sec_arg = DeclareLaunchArgument(
        "publish_period_sec",
        default_value="2.0",
        description="Marker republish period in seconds.",
    )
    include_debug_arg = DeclareLaunchArgument(
        "include_debug",
        default_value="false",
        description="Publish optional debug/audit marker group when available.",
    )

    marker_node = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_artifact_marker_publisher",
        name="boxfusion_artifact_marker_publisher",
        output="screen",
        parameters=[
            {
                "scene_root": LaunchConfiguration("scene_root"),
                "frame_id": LaunchConfiguration("frame_id"),
                "marker_topic_prefix": LaunchConfiguration("marker_topic_prefix"),
                "status_topic": LaunchConfiguration("status_topic"),
                "publish_period_sec": ParameterValue(LaunchConfiguration("publish_period_sec"), value_type=float),
                "include_debug": ParameterValue(LaunchConfiguration("include_debug"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            scene_root_arg,
            frame_id_arg,
            marker_topic_prefix_arg,
            status_topic_arg,
            publish_period_sec_arg,
            include_debug_arg,
            marker_node,
        ]
    )
