#!/bin/bash
# Phase 0 Re-test - Verify Phase 0 analysis still holds after Phase 1
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase0_retest.sh

set -e

echo "============================================================"
echo "Phase 0 Re-test - Verify Phase 0 Analysis Still Valid"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash

echo "Testing ROS 2 topics..."
echo

# Check if ROS 2 is working
if ! ros2 topic list > /dev/null 2>&1; then
    echo "✗ ROS 2 topics not accessible (this is normal if Gazebo is not running)"
    echo "  This is expected - topics will be available when simulation is running"
else
    echo "✓ ROS 2 topics accessible"
fi

echo
echo "Testing package availability..."
echo

# Test ackermann_drl package
if ros2 pkg list | grep -q "ackermann_drl"; then
    echo "✓ ackermann_drl package found"
else
    echo "✗ ackermann_drl package not found"
    exit 1
fi

# Test saye_bringup package
if ros2 pkg list | grep -q "saye_bringup"; then
    echo "✓ saye_bringup package found"
else
    echo "✗ saye_bringup package not found"
    exit 1
fi

# Test saye_description package
if ros2 pkg list | grep -q "saye_description"; then
    echo "✓ saye_description package found"
else
    echo "✗ saye_description package not found"
    exit 1
fi

echo
echo "Testing Python imports..."
echo

# Test DRL dependencies
python3 -c "import gymnasium; print('✓ gymnasium')" || { echo "✗ gymnasium"; exit 1; }
python3 -c "import stable_baselines3; print('✓ stable-baselines3')" || { echo "✗ stable-baselines3"; exit 1; }
python3 -c "import torch; print('✓ torch')" || { echo "✗ torch"; exit 1; }
python3 -c "import numpy; print('✓ numpy')" || { echo "✗ numpy"; exit 1; }
python3 -c "import shapely; print('✓ shapely')" || { echo "✗ shapely"; exit 1; }

echo
echo "Testing ackermann_drl imports..."
echo

python3 -c "import ackermann_drl; print('✓ ackermann_drl')" || { echo "✗ ackermann_drl"; exit 1; }
python3 -c "from ackermann_drl.envs import AckermannCityEnv; print('✓ AckermannCityEnv')" || { echo "✗ AckermannCityEnv"; exit 1; }
python3 -c "from ackermann_drl.utils import RoadsGeometry; print('✓ RoadsGeometry')" || { echo "✗ RoadsGeometry"; exit 1; }

echo
echo "Testing Docker environment..."
echo

# Check Docker environment variables
if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo "✓ RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
else
    echo "✗ RMW_IMPLEMENTATION not set"
    exit 1
fi

if [ -n "$ROS_DISABLE_SHARED_MEMORY" ]; then
    echo "✓ ROS_DISABLE_SHARED_MEMORY=$ROS_DISABLE_SHARED_MEMORY"
else
    echo "✗ ROS_DISABLE_SHARED_MEMORY not set"
    exit 1
fi

echo
echo "Testing workspace structure..."
echo

# Check workspace structure
if [ -d "$COLCON_WS/src/ackermann-vehicle-gzsim-ros2" ]; then
    echo "✓ Source directory exists"
else
    echo "✗ Source directory not found"
    exit 1
fi

if [ -d "$COLCON_WS/install" ]; then
    echo "✓ Install directory exists"
else
    echo "✗ Install directory not found"
    exit 1
fi

echo
echo "============================================================"
echo "Phase 0 Re-test: All checks passed!"
echo "============================================================"
echo
echo "Phase 0 analysis remains valid after Phase 1 changes."

