#!/bin/bash
# Build script - builds all packages from ackermann-vehicle-gzsim-ros2
# The duplicate saye_description in osm_city_pipeline is ignored via .colcon_ignore

set -e

COLCON_WS=/root/colcon_ws
cd $COLCON_WS

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build all packages
# Note: colcon should respect .colcon_ignore in osm_city_pipeline/saye_description
echo "Building workspace..."
colcon build --symlink-install "$@"

echo "Build completed successfully!"

