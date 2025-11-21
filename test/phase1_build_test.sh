#!/bin/bash
# Phase 1 Build Test - Verify colcon build succeeds inside Docker
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase1_build_test.sh

set -e

echo "============================================================"
echo "Phase 1 - Colcon Build Test"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws
cd $COLCON_WS

# Source ROS 2
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
echo "Workspace: $COLCON_WS"
echo

# Build the workspace
colcon build --symlink-install --packages-select ackermann_drl

echo
echo "============================================================"
echo "Build completed successfully!"
echo "============================================================"
echo

# Verify installation
if [ -d "$COLCON_WS/install/ackermann_drl" ]; then
    echo "✓ ackermann_drl package installed"
    echo "  Location: $COLCON_WS/install/ackermann_drl"
else
    echo "✗ ackermann_drl package not found in install directory"
    exit 1
fi

# Verify launch file
if [ -f "$COLCON_WS/install/ackermann_drl/share/ackermann_drl/launch/single_drl_training.launch.py" ]; then
    echo "✓ Launch file installed"
else
    echo "✗ Launch file not found"
    exit 1
fi

# Verify config file
if [ -f "$COLCON_WS/install/ackermann_drl/share/ackermann_drl/config/drl_params.yaml" ]; then
    echo "✓ Config file installed"
else
    echo "✗ Config file not found"
    exit 1
fi

# Verify scripts
if [ -f "$COLCON_WS/install/ackermann_drl/lib/ackermann_drl/train_ppo.py" ]; then
    echo "✓ Training script installed"
else
    echo "✗ Training script not found"
    exit 1
fi

echo
echo "All build checks passed!"

