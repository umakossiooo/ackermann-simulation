#!/bin/bash
# Fix colcon build symlink issue
# Run this inside the Docker container

COLCON_WS=/root/colcon_ws
cd $COLCON_WS

echo "Cleaning build directory for ackermann_drl..."
rm -rf build/ackermann_drl
rm -rf install/ackermann_drl
rm -rf log/latest_build/ackermann_drl

echo "Rebuilding ackermann_drl package..."
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ackermann_drl

echo "Build fix complete!"
