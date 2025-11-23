#!/bin/bash
# Run training test - execute training and check for errors
# Run this inside Docker container with Gazebo running

set -e  # Exit on error

echo "======================================================================"
echo "Training Test - Running Actual Training"
echo "======================================================================"
echo ""
echo "This will run a short training session (100 steps) to verify everything works"
echo ""

# Paths
SCRIPT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2"
TRAIN_SCRIPT="$SCRIPT_DIR/ackermann_drl/scripts/train_ppo.py"

# Check if script exists
if [ ! -f "$TRAIN_SCRIPT" ]; then
    echo "ERROR: Training script not found at $TRAIN_SCRIPT"
    exit 1
fi

# Check if we're in Docker
if [ ! -d "/root/colcon_ws" ]; then
    echo "WARNING: This script should be run inside Docker container"
    echo "Run: docker compose exec ackermann_sim bash"
    exit 1
fi

echo "Starting training test (100 timesteps)..."
echo ""

# Run training with minimal timesteps
python3 "$TRAIN_SCRIPT" \
    --total-timesteps 100 \
    --checkpoint-interval 50 \
    --learning-rate 3e-4 \
    --batch-size 32 \
    --n-steps 128 \
    --n-epochs 4 \
    2>&1 | tee /tmp/training_test_output.log

TRAIN_EXIT_CODE=${PIPESTATUS[0]}

echo ""
echo "======================================================================"
echo "Training Test Results"
echo "======================================================================"

# Check for common errors in output
ERRORS_FOUND=0

if grep -i "error\|exception\|traceback\|failed" /tmp/training_test_output.log | grep -v "Warning\|INFO"; then
    echo "ERRORS FOUND IN OUTPUT:"
    grep -i "error\|exception\|traceback\|failed" /tmp/training_test_output.log | grep -v "Warning\|INFO" | head -10
    ERRORS_FOUND=1
fi

if [ $TRAIN_EXIT_CODE -ne 0 ]; then
    echo "✗ Training exited with error code: $TRAIN_EXIT_CODE"
    ERRORS_FOUND=1
else
    echo "✓ Training completed successfully (exit code: 0)"
fi

# Check for expected output
if grep -q "Starting training" /tmp/training_test_output.log; then
    echo "✓ Training started"
else
    echo "✗ Training may not have started"
    ERRORS_FOUND=1
fi

if grep -q "REWARD" /tmp/training_test_output.log; then
    echo "✓ Reward logs appearing"
else
    echo "⚠ No reward logs found (may be normal for very short training)"
fi

if grep -q "TOTAL REWARD" /tmp/training_test_output.log; then
    echo "✓ Total reward logs appearing"
else
    echo "⚠ No total reward logs found"
fi

echo ""
if [ $ERRORS_FOUND -eq 0 ]; then
    echo "✓ Training test PASSED"
    echo "✓ System is working correctly"
    exit 0
else
    echo "✗ Training test FAILED"
    echo "Check /tmp/training_test_output.log for details"
    exit 1
fi

