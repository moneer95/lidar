# Outdoor / open-area: Nav2 + SLAM Toolbox (sync) while mapping.
#
# Prereqs (run in other terminals first):
#   1) turtlebot3_bringup + OpenCR + /odom + /tf
#   2) LiDAR publishing /scan; TF so laser frame attaches to base (base_link/base_scan)
#   3) TURTLEBOT3_MODEL=burger
#
# Install deps once (see scripts/install_tb3_nav2_deps.sh).
#
# Usage:
#   ros2 launch tb3_outdoor_nav2 outdoor_slam_nav2.launch.py
#
# Inverted drive (swap forward/back + turn sense), same idea as tb3_gap_nav invert_drive:
#   ros2 launch tb3_outdoor_nav2 outdoor_slam_nav2.launch.py invert_cmd_vel:=true
#
# Optional RViz (second terminal):
#   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    nav2_bringup = get_package_share_directory("nav2_bringup")
    tb3_outdoor_share = get_package_share_directory("tb3_outdoor_nav2")

    tb3_nav = None
    try:
        tb3_nav = get_package_share_directory("turtlebot3_navigation2")
    except PackageNotFoundError:
        # Allow running without turtlebot3_navigation2 installed by falling back to nav2 defaults.
        tb3_nav = None

    if tb3_nav is not None:
        default_map = os.path.join(tb3_nav, "map", "map.yaml")
        default_params = os.path.join(tb3_nav, "param", "humble", "burger.yaml")
    else:
        default_map = os.path.join(tb3_outdoor_share, "config", "map.yaml")
        default_params = os.path.join(nav2_bringup, "params", "nav2_params.yaml")
    bringup_py = os.path.join(nav2_bringup, "launch", "bringup_launch.py")

    invert_cmd_vel = LaunchConfiguration("invert_cmd_vel")
    # Accept common CLI spellings like `invert_cmd_vel:=true` / `false`.
    invert_cmd_vel_is_true = PythonExpression(
        ["'", invert_cmd_vel, "' in ['true','True','1','yes','on']"]
    )

    bringup_kwargs = {
        "slam": LaunchConfiguration("slam"),
        "map": LaunchConfiguration("map"),
        "params_file": LaunchConfiguration("params_file"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    def _bringup():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_py),
            launch_arguments=bringup_kwargs.items(),
        )

    return LaunchDescription(
        [
            # slam_toolbox defaults to `base_frame: base_footprint`, but this robot often
            # publishes only `base_link`. Create a static TF so SLAM/Nav2 frames exist.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_base_footprint",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "base_footprint",
                ],
                output="screen",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                # Use Python-style booleans since Nav2 launch files use IfCondition(slam/use_sim_time).
                default_value="False",
                description="False on real TurtleBot3 + Jetson",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="True",
                description="Run slam_toolbox + Nav2 mapping stack",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Map YAML (localization off when slam:=true; still passed to bringup)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="TurtleBot3 Nav2 params (DWB, costmaps, etc.)",
            ),
            DeclareLaunchArgument(
                "invert_cmd_vel",
                default_value="false",
                description="If true: Nav2 publishes to /cmd_vel_raw; tb3_cmd_vel_invert negates and publishes /cmd_vel",
            ),
            GroupAction(
                condition=UnlessCondition(invert_cmd_vel_is_true),
                actions=[_bringup()],
            ),
            GroupAction(
                condition=IfCondition(invert_cmd_vel_is_true),
                actions=[
                    SetRemap(src="/cmd_vel", dst="/cmd_vel_raw"),
                    _bringup(),
                ],
            ),
            Node(
                package="tb3_tools",
                executable="tb3_cmd_vel_invert",
                condition=IfCondition(invert_cmd_vel_is_true),
                parameters=[
                    {
                        "input_topic": "/cmd_vel_raw",
                        "output_topic": "/cmd_vel",
                        "invert": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
