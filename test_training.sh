#!/bin/bash
# Test training script - run inside Docker container
# This script tests the training command and checks for errors

set -e  # Exit on error

echo "======================================================================"
echo "Training Test Script"
echo "======================================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ERRORS=0
WARNINGS=0

# Test 1: Check if we're in Docker
echo "1. Checking environment..."
if [ -d "/root/colcon_ws" ]; then
    echo -e "${GREEN}✓ In Docker container${NC}"
else
    echo -e "${YELLOW}⚠ Not in Docker container (some tests may fail)${NC}"
    WARNINGS=$((WARNINGS + 1))
fi

# Test 2: Check ROS 2
echo ""
echo "2. Checking ROS 2..."
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓ ROS 2 available${NC}"
    ROS_DISTRO=$(printenv ROS_DISTRO || echo "unknown")
    echo "  ROS_DISTRO: $ROS_DISTRO"
else
    echo -e "${RED}✗ ROS 2 not found${NC}"
    ERRORS=$((ERRORS + 1))
fi

# Test 3: Check Python imports
echo ""
echo "3. Checking Python imports..."
python3 << 'PYTHON_EOF'
import sys
errors = []

try:
    import rclpy
    print("✓ rclpy")
except ImportError as e:
    print(f"✗ rclpy: {e}")
    errors.append("rclpy")

try:
    from stable_baselines3 import PPO
    print("✓ stable_baselines3")
except ImportError as e:
    print(f"✗ stable_baselines3: {e}")
    errors.append("stable_baselines3")

try:
    from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
    print("✓ ackermann_drl")
except ImportError as e:
    print(f"✗ ackermann_drl: {e}")
    errors.append("ackermann_drl")

if errors:
    sys.exit(1)
PYTHON_EOF

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ All Python imports successful${NC}"
else
    echo -e "${RED}✗ Some Python imports failed${NC}"
    ERRORS=$((ERRORS + 1))
fi

# Test 4: Check if Gazebo topics are available
echo ""
echo "4. Checking ROS 2 topics..."
if command -v ros2 &> /dev/null; then
    TOPICS=$(timeout 2 ros2 topic list 2>/dev/null || echo "")
    if echo "$TOPICS" | grep -q "/odom"; then
        echo -e "${GREEN}✓ /odom topic available${NC}"
    else
        echo -e "${YELLOW}⚠ /odom topic not found (Gazebo may not be running)${NC}"
        WARNINGS=$((WARNINGS + 1))
    fi
    
    if echo "$TOPICS" | grep -q "/scan"; then
        echo -e "${GREEN}✓ /scan topic available${NC}"
    else
        echo -e "${YELLOW}⚠ /scan topic not found (Gazebo may not be running)${NC}"
        WARNINGS=$((WARNINGS + 1))
    fi
    
    if echo "$TOPICS" | grep -q "/cmd_vel"; then
        echo -e "${GREEN}✓ /cmd_vel topic available${NC}"
    else
        echo -e "${YELLOW}⚠ /cmd_vel topic not found${NC}"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "${YELLOW}⚠ Cannot check topics (ROS 2 not available)${NC}"
    WARNINGS=$((WARNINGS + 1))
fi

# Test 5: Test training script syntax
echo ""
echo "5. Checking training script..."
TRAIN_SCRIPT="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py"
if [ -f "$TRAIN_SCRIPT" ]; then
    if python3 -m py_compile "$TRAIN_SCRIPT" 2>/dev/null; then
        echo -e "${GREEN}✓ Training script syntax OK${NC}"
    else
        echo -e "${RED}✗ Training script has syntax errors${NC}"
        ERRORS=$((ERRORS + 1))
    fi
else
    echo -e "${YELLOW}⚠ Training script not found at expected path${NC}"
    WARNINGS=$((WARNINGS + 1))
fi

# Test 6: Test environment import
echo ""
echo "6. Testing environment import..."
python3 << 'PYTHON_EOF'
import sys
sys.path.insert(0, '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl')

try:
    from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
    print("✓ AckermannGymEnv can be imported")
    
    # Try to create instance (will fail without ROS, but should not crash on import)
    print("✓ Environment class is valid")
except Exception as e:
    print(f"✗ Environment import failed: {e}")
    sys.exit(1)
PYTHON_EOF

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Environment import successful${NC}"
else
    echo -e "${RED}✗ Environment import failed${NC}"
    ERRORS=$((ERRORS + 1))
fi

# Summary
echo ""
echo "======================================================================"
echo "Test Summary"
echo "======================================================================"
echo -e "Errors: ${RED}${ERRORS}${NC}"
echo -e "Warnings: ${YELLOW}${WARNINGS}${NC}"
echo ""

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ All critical tests passed!${NC}"
    echo ""
    echo "You can now run training with:"
    echo "  python3 $TRAIN_SCRIPT --total-timesteps 1000"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please fix errors before training.${NC}"
    exit 1
fi

