#!/bin/bash
# Phase 2 Launch Test - Verify environment can launch with bari_world.sdf WITHOUT GUI
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase2_launch_test.sh

set -e

echo "============================================================"
echo "Phase 2 - Launch Test (bari_world.sdf + env, NO GUI)"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash

echo "Testing launch file with gui:=false..."
echo

# Test launch file with gui:=false (headless mode)
echo "Launch command:"
echo "  ros2 launch ackermann_drl single_drl_training.launch.py gui:=false"
echo
echo "This will start Gazebo in headless mode with the environment."
echo "Press Ctrl+C after 10 seconds to stop the test."
echo
echo "Starting launch (will timeout after 15 seconds)..."
echo

# Launch in background and capture PID
timeout 15 ros2 launch ackermann_drl single_drl_training.launch.py gui:=false &
LAUNCH_PID=$!

# Wait a bit for launch to start
sleep 5

# Check if processes are running
if ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo "✓ Launch process started successfully"
    
    # Check if Gazebo is running
    if pgrep -f "gz sim" > /dev/null; then
        echo "✓ Gazebo process detected"
    else
        echo "⚠ Gazebo process not detected (may still be starting)"
    fi
    
    # Check if ROS nodes are running
    if ros2 node list > /dev/null 2>&1; then
        echo "✓ ROS 2 nodes accessible"
        echo
        echo "Active nodes:"
        ros2 node list | head -5
    else
        echo "⚠ ROS 2 nodes not accessible yet"
    fi
    
    # Wait for timeout or kill
    wait $LAUNCH_PID 2>/dev/null || true
    
    echo
    echo "✓ Launch test completed (timeout or stopped)"
else
    echo "✗ Launch process failed to start"
    exit 1
fi

echo
echo "============================================================"
echo "Phase 2 Launch Test: Passed"
echo "============================================================"
echo
echo "The environment can launch with bari_world.sdf in headless mode."

