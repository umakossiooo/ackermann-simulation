#!/bin/bash
# Test the training command from QUICK_START_TRAINING.md
# Run this inside Docker container with Gazebo running

set -e  # Exit on error

echo "======================================================================"
echo "Testing Training Command"
echo "======================================================================"
echo ""

# Paths
SCRIPT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2"
TRAIN_SCRIPT="$SCRIPT_DIR/ackermann_drl/scripts/train_ppo.py"

# Check if script exists
if [ ! -f "$TRAIN_SCRIPT" ]; then
    echo "ERROR: Training script not found at $TRAIN_SCRIPT"
    exit 1
fi

# Check ROS topics
echo "1. Checking ROS topics..."
TOPICS=$(ros2 topic list 2>/dev/null || echo "")

if echo "$TOPICS" | grep -q "/odom"; then
    echo "   ✓ /odom topic available"
else
    echo "   ✗ /odom topic NOT found - is Gazebo running with play button on?"
    exit 1
fi

if echo "$TOPICS" | grep -q "/scan"; then
    echo "   ✓ /scan topic available"
else
    echo "   ✗ /scan topic NOT found - is Gazebo running with play button on?"
    exit 1
fi

if echo "$TOPICS" | grep -q "/cmd_vel"; then
    echo "   ✓ /cmd_vel topic available"
else
    echo "   ✗ /cmd_vel topic NOT found"
    exit 1
fi

echo ""
echo "2. Running training test (200 timesteps - shortened for testing)..."
echo "   Using same parameters as QUICK_START_TRAINING.md but fewer timesteps"
echo ""

# Run training with shortened timesteps for testing
# Using same parameters as the guide but only 200 steps to verify it works
python3 "$TRAIN_SCRIPT" \
    --total-timesteps 200 \
    --checkpoint-interval 100 \
    --learning-rate 3e-4 \
    --batch-size 128 \
    --n-steps 2048 \
    --device auto

EXIT_CODE=$?

echo ""
echo "======================================================================"
if [ $EXIT_CODE -eq 0 ]; then
    echo "✓ Training test PASSED!"
    echo "✓ Command works correctly"
    echo "✓ Logs are functioning"
    echo ""
    echo "You can now run the full training command:"
    echo "  python3 $TRAIN_SCRIPT \\"
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
    echo "Common issues:"
    echo "  - Make sure Gazebo is running with play button on"
    echo "  - Check ROS topics: ros2 topic list"
    echo "  - Verify package is built: cd /root/colcon_ws && colcon build --packages-select ackermann_drl"
fi
echo "======================================================================"

exit $EXIT_CODE

