#!/bin/bash
# Test the training command from QUICK_START_TRAINING.md
# Run this inside Docker container with Gazebo running

set -e  # Exit on error

echo "======================================================================"
echo "Testing Training Command"
echo "======================================================================"
echo ""

# Check ROS topics
echo "1. Checking ROS topics..."
TOPICS=$(ros2 topic list 2>/dev/null || echo "")

if echo "$TOPICS" | grep -q "/odom"; then
    echo "   ✓ /odom topic found"
else
    echo "   ✗ /odom topic NOT found - is Gazebo running with play button on?"
    exit 1
fi

if echo "$TOPICS" | grep -q "/scan"; then
    echo "   ✓ /scan topic found"
else
    echo "   ✗ /scan topic NOT found - is Gazebo running with play button on?"
    exit 1
fi

if echo "$TOPICS" | grep -q "/cmd_vel"; then
    echo "   ✓ /cmd_vel topic found"
else
    echo "   ✗ /cmd_vel topic NOT found"
    exit 1
fi

echo ""
echo "2. Testing training command (reduced timesteps for testing)..."
echo "   Using the same parameters as QUICK_START_TRAINING.md"
echo "   but with 500 timesteps instead of 1000000 for quick test"
echo ""

# Set paths
SCRIPT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2"
TRAIN_SCRIPT="$SCRIPT_DIR/ackermann_drl/scripts/train_ppo.py"

# Run training with reduced timesteps for testing
# Using same parameters as QUICK_START_TRAINING.md
python3 "$TRAIN_SCRIPT" \
    --total-timesteps 500 \
    --checkpoint-interval 250 \
    --learning-rate 3e-4 \
    --batch-size 128 \
    --n-steps 2048 \
    --device auto

EXIT_CODE=$?

echo ""
echo "======================================================================"
if [ $EXIT_CODE -eq 0 ]; then
    echo "✓ Training test PASSED!"
    echo "✓ All logs are working correctly"
    echo "✓ No errors detected"
    echo ""
    echo "The training command works! You can now run the full training:"
    echo ""
    echo "python3 $TRAIN_SCRIPT \\"
    echo "    --total-timesteps 1000000 \\"
    echo "    --checkpoint-interval 50000 \\"
    echo "    --learning-rate 3e-4 \\"
    echo "    --batch-size 128 \\"
    echo "    --n-steps 2048 \\"
    echo "    --device auto"
else
    echo "✗ Training test FAILED (exit code: $EXIT_CODE)"
    echo "Check the output above for errors"
    echo ""
    echo "Common fixes:"
    echo "  - Make sure Gazebo is running with play button on"
    echo "  - Check ROS topics: ros2 topic list"
    echo "  - Rebuild package: cd /root/colcon_ws && colcon build --packages-select ackermann_drl && source install/setup.bash"
fi
echo "======================================================================"

exit $EXIT_CODE

