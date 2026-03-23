"""Start YDLidar G4 with the same parameters as config/ydlidar_g4_params.yaml (installed in share)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("tb3_outdoor_nav2")
    default_params = os.path.join(pkg, "config", "ydlidar_g4_params.yaml")
    ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")
    ydlidar_launch = os.path.join(ydlidar_share, "launch", "ydlidar_launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YDLidar G4 params (angles 60–180°, range 0.12–0.4 m, 12 Hz)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ydlidar_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                }.items(),
            ),
        ]
    )
