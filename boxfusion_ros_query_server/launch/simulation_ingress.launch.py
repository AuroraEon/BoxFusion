from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/color/image_raw",
        description="Simulated RGB image topic.",
    )
    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/depth/image_raw",
        description="Simulated depth image topic.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="Optional simulated camera info topic.",
    )
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/boxfusion/sim/pose",
        description="PoseStamped topic used when pose_source:=pose.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="",
        description="Odometry topic used when pose_source:=odom.",
    )
    pose_source_arg = DeclareLaunchArgument(
        "pose_source",
        default_value="pose",
        description="Pose source selector: pose, odom, or tf.",
    )
    tf_target_frame_arg = DeclareLaunchArgument(
        "tf_target_frame",
        default_value="map",
        description="Target/world frame used when pose_source:=tf.",
    )
    tf_source_frame_arg = DeclareLaunchArgument(
        "tf_source_frame",
        default_value="",
        description="Optional source/camera frame used when pose_source:=tf. Empty falls back to the latest image frame id.",
    )
    tf_lookup_timeout_sec_arg = DeclareLaunchArgument(
        "tf_lookup_timeout_sec",
        default_value="0.15",
        description="Maximum TF lookup wait in seconds when pose_source:=tf.",
    )
    output_root_arg = DeclareLaunchArgument(
        "output_root",
        default_value="ros2_sim_ingress_runs",
        description="Workspace-local output root for simulation ingress scene roots.",
    )
    coordination_root_arg = DeclareLaunchArgument(
        "coordination_root",
        default_value="ros2_sim_ingress_runs/coordination",
        description="Workspace-local coordinator latest-pointer root.",
    )
    sequence_name_arg = DeclareLaunchArgument(
        "sequence_name",
        default_value="gazebo_sim_sequence",
        description="Sequence name used for the Stage-A-compatible simulation scene root.",
    )
    snapshot_period_sec_arg = DeclareLaunchArgument(
        "snapshot_period_sec",
        default_value="1.0",
        description="Periodic snapshot cadence in seconds.",
    )
    target_snapshot_count_arg = DeclareLaunchArgument(
        "target_snapshot_count",
        default_value="0",
        description="Optional bounded capture count. 0 keeps periodic capture running until externally stopped.",
    )
    max_sync_skew_sec_arg = DeclareLaunchArgument(
        "max_sync_skew_sec",
        default_value="0.15",
        description="Maximum allowed timestamp skew across latest RGB, depth, and pose samples.",
    )
    backend_mode_arg = DeclareLaunchArgument(
        "backend_mode",
        default_value="stub",
        description="Simulation backend mode: stub or real.",
    )
    fallback_to_stub_arg = DeclareLaunchArgument(
        "fallback_to_stub_on_real_backend_failure",
        default_value="false",
        description="In real mode, refresh the minimal stub if the Stage A producer fails.",
    )
    real_backend_handoff_format_arg = DeclareLaunchArgument(
        "real_backend_handoff_format",
        default_value="hm3d",
        description="Real producer input contract: hm3d or ca1m.",
    )
    real_backend_output_root_arg = DeclareLaunchArgument(
        "real_backend_output_root",
        default_value="ros2_sim_ingress_runs/stage_a_real_backend_sequences",
        description="Workspace-local output root for real Stage A producer scene roots.",
    )
    real_backend_python_arg = DeclareLaunchArgument(
        "real_backend_python",
        default_value="python3",
        description="Python executable used to run the real Stage A producer.",
    )
    real_backend_script_arg = DeclareLaunchArgument(
        "real_backend_script",
        default_value="stage_a_demo.py",
        description="Path to the existing Stage A producer script.",
    )
    real_backend_model_path_arg = DeclareLaunchArgument(
        "real_backend_model_path",
        default_value="./models/cutr_rgbd.pth",
        description="BoxFusion/Cubify checkpoint for real backend mode.",
    )
    real_backend_clip_path_arg = DeclareLaunchArgument(
        "real_backend_clip_path",
        default_value="./models/ViT-B-32/open_clip_pytorch_model.bin",
        description="CLIP checkpoint for real backend mode.",
    )
    real_backend_text_features_arg = DeclareLaunchArgument(
        "real_backend_text_features",
        default_value="./data/class_features_small.pt",
        description="Text feature tensor for real backend mode.",
    )
    real_backend_device_arg = DeclareLaunchArgument(
        "real_backend_device",
        default_value="cpu",
        description="Device passed to the real Stage A producer.",
    )
    real_backend_timeout_sec_arg = DeclareLaunchArgument(
        "real_backend_timeout_sec",
        default_value="0.0",
        description="Optional real producer timeout in seconds; 0 disables the timeout.",
    )

    ingress = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_simulation_ingress_node",
        name="boxfusion_simulation_ingress_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "pose_source": LaunchConfiguration("pose_source"),
                "tf_target_frame": LaunchConfiguration("tf_target_frame"),
                "tf_source_frame": LaunchConfiguration("tf_source_frame"),
                "tf_lookup_timeout_sec": ParameterValue(LaunchConfiguration("tf_lookup_timeout_sec"), value_type=float),
                "output_root": LaunchConfiguration("output_root"),
                "coordination_root": LaunchConfiguration("coordination_root"),
                "sequence_name": LaunchConfiguration("sequence_name"),
                "snapshot_period_sec": ParameterValue(LaunchConfiguration("snapshot_period_sec"), value_type=float),
                "target_snapshot_count": ParameterValue(LaunchConfiguration("target_snapshot_count"), value_type=int),
                "max_sync_skew_sec": ParameterValue(LaunchConfiguration("max_sync_skew_sec"), value_type=float),
                "backend_mode": LaunchConfiguration("backend_mode"),
                "fallback_to_stub_on_real_backend_failure": ParameterValue(
                    LaunchConfiguration("fallback_to_stub_on_real_backend_failure"),
                    value_type=bool,
                ),
                "real_backend_handoff_format": LaunchConfiguration("real_backend_handoff_format"),
                "real_backend_output_root": LaunchConfiguration("real_backend_output_root"),
                "real_backend_python": LaunchConfiguration("real_backend_python"),
                "real_backend_script": LaunchConfiguration("real_backend_script"),
                "real_backend_model_path": LaunchConfiguration("real_backend_model_path"),
                "real_backend_clip_path": LaunchConfiguration("real_backend_clip_path"),
                "real_backend_text_features": LaunchConfiguration("real_backend_text_features"),
                "real_backend_device": LaunchConfiguration("real_backend_device"),
                "real_backend_timeout_sec": ParameterValue(LaunchConfiguration("real_backend_timeout_sec"), value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            rgb_topic_arg,
            depth_topic_arg,
            camera_info_topic_arg,
            pose_topic_arg,
            odom_topic_arg,
            pose_source_arg,
            tf_target_frame_arg,
            tf_source_frame_arg,
            tf_lookup_timeout_sec_arg,
            output_root_arg,
            coordination_root_arg,
            sequence_name_arg,
            snapshot_period_sec_arg,
            target_snapshot_count_arg,
            max_sync_skew_sec_arg,
            backend_mode_arg,
            fallback_to_stub_arg,
            real_backend_handoff_format_arg,
            real_backend_output_root_arg,
            real_backend_python_arg,
            real_backend_script_arg,
            real_backend_model_path_arg,
            real_backend_clip_path_arg,
            real_backend_text_features_arg,
            real_backend_device_arg,
            real_backend_timeout_sec_arg,
            ingress,
        ]
    )
