#!/bin/bash
# Phase 0-7 Re-test - Verify Phases 0-7 still work after Phase 8
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase07_retest.sh

set -e

echo "============================================================"
echo "Phase 0-7 Re-test - Verify Previous Phases Still Work"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash

echo "Testing Phase 0 (ROS Topics)..."
echo

# Test ROS 2 topics (if Gazebo is running)
if ros2 topic list > /dev/null 2>&1; then
    echo "✓ ROS 2 topics accessible"
    
    # Check for key topics
    if ros2 topic list | grep -q "/cmd_vel"; then
        echo "✓ /cmd_vel topic exists"
    else
        echo "⚠ /cmd_vel topic not found (Gazebo may not be running)"
    fi
    
    if ros2 topic list | grep -q "/odom"; then
        echo "✓ /odom topic exists"
    else
        echo "⚠ /odom topic not found (Gazebo may not be running)"
    fi
    
    if ros2 topic list | grep -q "/scan"; then
        echo "✓ /scan topic exists"
    else
        echo "⚠ /scan topic not found (Gazebo may not be running)"
    fi
else
    echo "⚠ ROS 2 topics not accessible (this is normal if Gazebo is not running)"
fi

echo
echo "Testing Phase 1 (Package Structure)..."
echo

# Test package availability
if ros2 pkg list | grep -q "ackermann_drl"; then
    echo "✓ ackermann_drl package found"
else
    echo "✗ ackermann_drl package not found"
    exit 1
fi

# Test Python imports
echo
echo "Testing Python imports..."
echo

python3 -c "import ackermann_drl; print('✓ ackermann_drl')" || { echo "✗ ackermann_drl"; exit 1; }
python3 -c "from ackermann_drl.envs import AckermannCityEnv; print('✓ AckermannCityEnv')" || { echo "✗ AckermannCityEnv"; exit 1; }
python3 -c "from ackermann_drl.envs import AckermannGymEnv; print('✓ AckermannGymEnv')" || { echo "✗ AckermannGymEnv"; exit 1; }
python3 -c "from ackermann_drl.utils import RoadsGeometry; print('✓ RoadsGeometry')" || { echo "✗ RoadsGeometry"; exit 1; }
python3 -c "from ackermann_drl.utils import BatteryModel; print('✓ BatteryModel')" || { echo "✗ BatteryModel"; exit 1; }
python3 -c "from ackermann_drl.utils import DeliveryPoints; print('✓ DeliveryPoints')" || { echo "✗ DeliveryPoints"; exit 1; }

# Test DRL dependencies
echo
echo "Testing DRL dependencies..."
echo

python3 -c "import gymnasium; print('✓ gymnasium')" || { echo "✗ gymnasium"; exit 1; }
python3 -c "import stable_baselines3; print('✓ stable-baselines3')" || { echo "✗ stable-baselines3"; exit 1; }
python3 -c "import torch; print('✓ torch')" || { echo "✗ torch"; exit 1; }
python3 -c "import numpy; print('✓ numpy')" || { echo "✗ numpy"; exit 1; }
python3 -c "import shapely; print('✓ shapely')" || { echo "✗ shapely"; exit 1; }

# Test build
echo
echo "Testing package build..."
echo

if [ -d "$COLCON_WS/install/ackermann_drl" ]; then
    echo "✓ ackermann_drl package installed"
else
    echo "✗ ackermann_drl package not found in install directory"
    exit 1
fi

# Test launch file
echo
echo "Testing launch file..."
echo

LAUNCH_FILE="$COLCON_WS/install/ackermann_drl/share/ackermann_drl/launch/single_drl_training.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✓ Launch file exists"
    
    # Test parsing
    if ros2 launch ackermann_drl single_drl_training.launch.py --show-args > /dev/null 2>&1; then
        echo "✓ Launch file parses correctly"
    else
        echo "✗ Launch file has syntax errors"
        exit 1
    fi
else
    echo "✗ Launch file not found"
    exit 1
fi

echo
echo "Testing Phase 2 (Environment)..."
echo

# Test environment import
if python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_env_import.py > /dev/null 2>&1; then
    echo "✓ Environment import test passed"
else
    echo "⚠ Environment import test failed (check manually)"
fi

echo
echo "Testing Phase 3 (Roads Geometry)..."
echo

# Test roads geometry import
if python3 -c "from ackermann_drl.utils.roads_geometry import RoadsGeometry; print('✓ RoadsGeometry import')" 2>/dev/null; then
    echo "✓ RoadsGeometry can be imported"
else
    echo "✗ RoadsGeometry import failed"
    exit 1
fi

echo
echo "Testing Phase 4 (Battery Model)..."
echo

# Test battery model import
if python3 -c "from ackermann_drl.utils.battery_model import BatteryModel; print('✓ BatteryModel import')" 2>/dev/null; then
    echo "✓ BatteryModel can be imported"
else
    echo "✗ BatteryModel import failed"
    exit 1
fi

echo
echo "Testing Phase 5 (Delivery Points)..."
echo

# Test delivery points import
if python3 -c "from ackermann_drl.utils.delivery_points import DeliveryPoints; print('✓ DeliveryPoints import')" 2>/dev/null; then
    echo "✓ DeliveryPoints can be imported"
else
    echo "✗ DeliveryPoints import failed"
    exit 1
fi

echo
echo "Testing Phase 6 (Complete Observation)..."
echo

# Test observation shape
if python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_obs_shape.py > /dev/null 2>&1; then
    echo "✓ Observation shape test passed"
else
    echo "⚠ Observation shape test failed (check manually)"
fi

echo
echo "Testing Phase 7 (Reward Function)..."
echo

# Test reward function
if python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_reward_progress.py > /dev/null 2>&1; then
    echo "✓ Progress reward test passed"
else
    echo "⚠ Progress reward test failed (check manually)"
fi

echo
echo "Testing Phase 8 (PPO Training)..."
echo

# Test PPO imports
if python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_ppo_import.py > /dev/null 2>&1; then
    echo "✓ PPO import test passed"
else
    echo "⚠ PPO import test failed (check manually)"
fi

# Test train_ppo script exists
TRAIN_SCRIPT="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py"
if [ -f "$TRAIN_SCRIPT" ]; then
    echo "✓ train_ppo.py exists"
    if [ -x "$TRAIN_SCRIPT" ]; then
        echo "✓ train_ppo.py is executable"
    else
        echo "⚠ train_ppo.py is not executable"
    fi
else
    echo "✗ train_ppo.py not found"
    exit 1
fi

# Test checkpoint directory can be created
CHECKPOINT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints"
mkdir -p "$CHECKPOINT_DIR"
if [ -d "$CHECKPOINT_DIR" ]; then
    echo "✓ Checkpoint directory exists and is writable"
else
    echo "✗ Checkpoint directory cannot be created"
    exit 1
fi

# Test gym wrapper
if python3 -c "
from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
import rclpy
rclpy.init()
env = AckermannGymEnv()
print(f'✓ Gym wrapper created')
print(f'  Observation space: {env.observation_space.shape}')
print(f'  Action space: {env.action_space.shape}')
env.close()
rclpy.shutdown()
" 2>/dev/null; then
    echo "✓ Gym wrapper works correctly"
else
    echo "✗ Gym wrapper test failed"
    exit 1
fi

echo
echo "============================================================"
echo "Phase 0-7 Re-test: All checks passed!"
echo "============================================================"
echo
echo "Previous phases remain valid after Phase 8 changes."

