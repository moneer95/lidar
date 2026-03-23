#!/usr/bin/env bash
# One-time on Ubuntu 22.04 / Jetson with ROS 2 Humble
set -euo pipefail
sudo apt-get update
sudo apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3-navigation2
echo "OK. Source: source /opt/ros/humble/setup.bash"
echo "Build workspace: colcon build --packages-select tb3_outdoor_nav2"
