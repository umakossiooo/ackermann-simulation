#!/bin/bash
# Build script that avoids duplicate package errors
# Run this inside Docker container: docker compose exec ackermann_sim bash
# Then: bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/build_workspace.sh

set -e

COLCON_WS=/root/colcon_ws
cd $COLCON_WS

# Source ROS 2
source /opt/ros/jazzy/setup.bash

echo "============================================================"
echo "Building workspace (avoiding duplicate packages)"
echo "============================================================"
echo

# List of packages to build (from ackermann-vehicle-gzsim-ros2 only)
PACKAGES="saye_msgs saye_description saye_control saye_behaviortree saye_bringup saye_localization ackermann_drl"

echo "Building packages: $PACKAGES"
echo

# Build dependencies first, then ackermann_drl
colcon build \
  --packages-select $PACKAGES \
  --symlink-install

echo
echo "============================================================"
echo "Build completed successfully!"
echo "============================================================"
echo
echo "To use the workspace, source:"
echo "  source /root/colcon_ws/install/setup.bash"

