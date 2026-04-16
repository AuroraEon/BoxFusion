from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    capture_log_arg = DeclareLaunchArgument(
        "capture_log",
        description="Path to logs/simulation_ingress_frames.jsonl used as the replay source.",
    )
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/color/image_raw",
        description="RGB image topic to publish.",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/depth/image_raw",
        description="Depth image topic to publish.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="CameraInfo topic to publish.",
    )
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/boxfusion/sim/pose",
        description="PoseStamped topic to publish.",
    )
    frame_period_sec_arg = DeclareLaunchArgument(
        "frame_period_sec",
        default_value="1.0",
        description="Wall-clock period between replayed frame publications.",
    )
    shutdown_grace_sec_arg = DeclareLaunchArgument(
        "shutdown_grace_sec",
        default_value="1.5",
        description="How long to keep the last frame live before the replay node exits.",
    )
    max_frames_arg = DeclareLaunchArgument(
        "max_frames",
        default_value="0",
        description="Optional frame cap; 0 replays the whole capture log.",
    )

    replay = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_simulation_replay_source_node",
        name="boxfusion_simulation_replay_source_node",
        output="screen",
        parameters=[
            {
                "capture_log": LaunchConfiguration("capture_log"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "frame_period_sec": ParameterValue(LaunchConfiguration("frame_period_sec"), value_type=float),
                "shutdown_grace_sec": ParameterValue(LaunchConfiguration("shutdown_grace_sec"), value_type=float),
                "max_frames": ParameterValue(LaunchConfiguration("max_frames"), value_type=int),
            }
        ],
    )

    return LaunchDescription(
        [
            capture_log_arg,
            rgb_topic_arg,
            depth_topic_arg,
            camera_info_topic_arg,
            pose_topic_arg,
            frame_period_sec_arg,
            shutdown_grace_sec_arg,
            max_frames_arg,
            replay,
        ]
    )
