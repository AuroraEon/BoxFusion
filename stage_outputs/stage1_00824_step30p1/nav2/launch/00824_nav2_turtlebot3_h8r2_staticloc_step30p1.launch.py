"""Step30P1 H8R2 Nav2 bringup without AMCL.

This launch file expects map->odom to be supplied by
scripts/launch_step30p1_staticloc_tf.sh. It deliberately does not launch AMCL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bt_xml = LaunchConfiguration("default_bt_xml_filename")

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "recoveries_server",
        "bt_navigator",
        "waypoint_follower",
    ]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "default_bt_xml_filename": bt_xml,
            "autostart": autostart,
            "map_subscribe_transient_local": "true",
        },
        convert_types=True,
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "map",
            default_value="/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/maps/h8r2_gateway_preserving_nav_map.yaml",
            description="Step30P1 H8R2 map YAML path.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/ws/workspace/BoxFusion/stage_outputs/stage1_00824_step30p1/nav2/config/00824_nav2_turtlebot3_step30h7_narrow_gateway.yaml",
            description="Step30H7 narrow-gateway Nav2 params reused by Step30P1.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "default_bt_xml_filename",
            default_value="/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml",
            description="Foxy NavigateToPose behavior tree XML.",
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen", parameters=[configured_params], remappings=remappings),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[configured_params], remappings=remappings),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[configured_params], remappings=remappings),
        Node(package="nav2_recoveries", executable="recoveries_server", name="recoveries_server", output="screen", parameters=[configured_params], remappings=remappings),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[configured_params], remappings=remappings),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=[configured_params], remappings=remappings),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"autostart": autostart}, {"node_names": lifecycle_nodes}],
        ),
    ])
