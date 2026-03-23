#!/bin/bash
# Clean build/install/log and rebuild lidar_tools.
# Run from the lidar workspace root: ./clean_and_build.sh

set -e
cd "$(dirname "$0")"

# Unset stale paths that might point to missing install
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH

echo "Working directory: $(pwd)"
if [[ "$(pwd)" == *"Trash"* ]]; then
    echo "WARNING: You are inside Trash. Copy this folder to e.g. ~/Desktop/lidar or ~/lidar and run from there."
    exit 1
fi

echo "Removing build, install, log..."
rm -rf build install log

echo "Sourcing ROS2..."
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true

echo "Building..."
colcon build --symlink-install

echo "Done. Source the workspace: source install/setup.bash"
