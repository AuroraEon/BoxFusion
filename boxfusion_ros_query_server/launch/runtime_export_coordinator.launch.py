from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    source_scene_root_arg = DeclareLaunchArgument(
        "source_scene_root",
        description="Path to the Stage A scene root that already contains or will contain manifest/log artifacts.",
    )
    coordination_root_arg = DeclareLaunchArgument(
        "coordination_root",
        description="Directory that will host the stable latest pointer and refresh metadata.",
    )
    dataset_root_arg = DeclareLaunchArgument(
        "dataset_root",
        default_value="",
        description="Optional dataset root used when refreshing manifest.json.",
    )
    sequence_name_arg = DeclareLaunchArgument(
        "sequence_name",
        default_value="",
        description="Optional sequence name override for manifest refresh.",
    )
    producer_command_arg = DeclareLaunchArgument(
        "producer_command",
        default_value="",
        description="Optional command to run before each refresh, for example a stage_a_demo.py invocation.",
    )
    producer_cwd_arg = DeclareLaunchArgument(
        "producer_cwd",
        default_value="",
        description="Optional working directory for producer_command.",
    )
    refresh_interval_sec_arg = DeclareLaunchArgument(
        "refresh_interval_sec",
        default_value="30.0",
        description="Sleep duration between refreshes in periodic mode.",
    )
    runtime_artifact_mode_arg = DeclareLaunchArgument(
        "runtime_artifact_mode",
        default_value="benchmark",
        description="benchmark, debug, or service. Service records artifact-only work as skipped/deferred.",
    )
    enable_sidecar_shadow_export_arg = DeclareLaunchArgument(
        "enable_sidecar_shadow_export",
        default_value="false",
        description="When true, write a shadow sidecar metadata stub from the runtime snapshot.",
    )
    sidecar_output_dir_arg = DeclareLaunchArgument(
        "sidecar_output_dir",
        default_value="",
        description="Optional sidecar shadow output directory.",
    )
    coordinator = Node(
        package="boxfusion_ros_query_server",
        executable="boxfusion_runtime_export_coordinator",
        name="boxfusion_runtime_export_coordinator",
        output="screen",
        arguments=[
            "--source-scene-root",
            LaunchConfiguration("source_scene_root"),
            "--coordination-root",
            LaunchConfiguration("coordination_root"),
            "--dataset-root",
            LaunchConfiguration("dataset_root"),
            "--sequence-name",
            LaunchConfiguration("sequence_name"),
            "--producer-command",
            LaunchConfiguration("producer_command"),
            "--producer-cwd",
            LaunchConfiguration("producer_cwd"),
            "--refresh-interval-sec",
            LaunchConfiguration("refresh_interval_sec"),
            "--runtime-artifact-mode",
            LaunchConfiguration("runtime_artifact_mode"),
            "--sidecar-output-dir",
            LaunchConfiguration("sidecar_output_dir"),
            "--enable-sidecar-shadow-export",
            LaunchConfiguration("enable_sidecar_shadow_export"),
        ],
    )

    return LaunchDescription(
        [
            source_scene_root_arg,
            coordination_root_arg,
            dataset_root_arg,
            sequence_name_arg,
            producer_command_arg,
            producer_cwd_arg,
            refresh_interval_sec_arg,
            runtime_artifact_mode_arg,
            enable_sidecar_shadow_export_arg,
            sidecar_output_dir_arg,
            coordinator,
        ]
    )
