#!/bin/bash
# Phase 1 Launch File Test - Verify launch file parses correctly
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase1_launch_test.sh

set -e

echo "============================================================"
echo "Phase 1 - Launch File Parse Test"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash

# Test launch file parsing
echo "Testing launch file parsing..."
echo

LAUNCH_FILE="$COLCON_WS/install/ackermann_drl/share/ackermann_drl/launch/single_drl_training.launch.py"

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "✗ Launch file not found: $LAUNCH_FILE"
    echo "  Run phase1_build_test.sh first"
    exit 1
fi

echo "Launch file: $LAUNCH_FILE"
echo

# Use ros2 launch --show-args to test parsing (without actually launching)
echo "Testing launch file syntax..."
if ros2 launch ackermann_drl single_drl_training.launch.py --show-args > /dev/null 2>&1; then
    echo "✓ Launch file parses correctly"
else
    echo "✗ Launch file has syntax errors"
    echo
    echo "Attempting to show errors:"
    ros2 launch ackermann_drl single_drl_training.launch.py --show-args 2>&1 || true
    exit 1
fi

echo
echo "Launch file test passed!"

