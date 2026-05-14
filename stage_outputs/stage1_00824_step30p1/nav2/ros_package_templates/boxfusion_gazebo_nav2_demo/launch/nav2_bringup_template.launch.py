from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    bringup_launch = PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Generated approximate map YAML path."),
        DeclareLaunchArgument("params", description="Step 17 Nav2 params template path."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={"map": map_yaml, "params_file": params, "use_sim_time": "true"}.items(),
        ),
    ])
