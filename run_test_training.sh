#!/bin/bash
# Quick training test - run this inside Docker container with Gazebo running

echo "======================================================================"
echo "Testing Training with Gazebo Running"
echo "======================================================================"
echo ""

# Check if we're in the right directory
if [ ! -f "ackermann_drl/scripts/train_ppo.py" ]; then
    echo "ERROR: Please run from workspace root or use absolute path"
    echo "Current directory: $(pwd)"
    exit 1
fi

# Set paths
SCRIPT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2"
TRAIN_SCRIPT="$SCRIPT_DIR/ackermann_drl/scripts/train_ppo.py"

# Check ROS topics first
echo "1. Checking ROS topics..."
if ros2 topic list | grep -q "/odom"; then
    echo "   ✓ /odom topic found"
else
    echo "   ✗ /odom topic NOT found - is Gazebo running?"
    exit 1
fi

if ros2 topic list | grep -q "/scan"; then
    echo "   ✓ /scan topic found"
else
    echo "   ✗ /scan topic NOT found - is Gazebo running?"
    exit 1
fi

if ros2 topic list | grep -q "/cmd_vel"; then
    echo "   ✓ /cmd_vel topic found"
else
    echo "   ✗ /cmd_vel topic NOT found"
    exit 1
fi

echo ""
echo "2. Running short training test (100 timesteps)..."
echo "   This will verify:"
echo "   - Environment creation"
echo "   - Reward calculation"
echo "   - Real-time logging"
echo "   - No errors"
echo ""

# Run training with minimal timesteps
python3 "$TRAIN_SCRIPT" \
    --total-timesteps 100 \
    --checkpoint-interval 50 \
    --learning-rate 3e-4 \
    --batch-size 32 \
    --n-steps 128 \
    --n-epochs 4

EXIT_CODE=$?

echo ""
echo "======================================================================"
if [ $EXIT_CODE -eq 0 ]; then
    echo "✓ Training test PASSED!"
    echo "✓ Everything is working correctly"
    echo ""
    echo "You can now run full training with:"
    echo "  python3 $TRAIN_SCRIPT --total-timesteps 10000"
else
    echo "✗ Training test FAILED (exit code: $EXIT_CODE)"
    echo "Check the output above for errors"
fi
echo "======================================================================"

exit $EXIT_CODE

