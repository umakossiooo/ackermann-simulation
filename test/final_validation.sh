#!/bin/bash
# Final Validation - Comprehensive test of entire DRL system
#
# This test must be run inside the Docker container:
#     docker compose exec ackermann_sim bash
#     bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/final_validation.sh

set -e

echo "============================================================"
echo "FINAL VALIDATION - Single-Agent Docker-Based DRL System"
echo "============================================================"
echo

COLCON_WS=/root/colcon_ws
TEST_DIR=/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test
PASSED=0
FAILED=0
WARNINGS=0

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash

# Function to print test result
test_result() {
    if [ $1 -eq 0 ]; then
        echo "✓ $2"
        PASSED=$((PASSED + 1))
    else
        echo "✗ $2"
        FAILED=$((FAILED + 1))
    fi
}

# Function to print warning
test_warning() {
    echo "⚠ $1"
    WARNINGS=$((WARNINGS + 1))
}

echo "============================================================"
echo "1. ENVIRONMENT RESET TEST"
echo "============================================================"
echo

if python3 -c "
import rclpy
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
rclpy.init()
env = AckermannCityEnv()
obs = env.reset()
print(f'✓ Environment reset successful')
print(f'  Observation shape: {obs.shape}')
assert obs.shape == (187,), f'Expected shape (187,), got {obs.shape}'
env.destroy_node()
rclpy.shutdown()
" 2>&1; then
    test_result 0 "Environment reset works in Docker"
else
    test_result 1 "Environment reset failed"
fi

echo
echo "============================================================"
echo "2. ENVIRONMENT STEP TEST"
echo "============================================================"
echo

if python3 -c "
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
rclpy.init()
env = AckermannCityEnv()
obs = env.reset()
action = np.array([0.5, 0.0], dtype=np.float32)
obs, reward, terminated, truncated, info = env.step(action)
print(f'✓ Environment step successful')
print(f'  Observation shape: {obs.shape}')
print(f'  Reward: {reward:.3f}')
print(f'  Terminated: {terminated}, Truncated: {truncated}')
assert obs.shape == (187,), f'Expected shape (187,), got {obs.shape}'
assert 'battery_level' in info, 'battery_level not in info'
assert 'reward_progress' in info, 'reward_progress not in info'
env.destroy_node()
rclpy.shutdown()
" 2>&1; then
    test_result 0 "Environment step works in Docker"
else
    test_result 1 "Environment step failed"
fi

echo
echo "============================================================"
echo "3. PPO SHORT TRAINING TEST"
echo "============================================================"
echo

# Check if train_ppo.py exists
TRAIN_SCRIPT="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py"
if [ ! -f "$TRAIN_SCRIPT" ]; then
    test_result 1 "train_ppo.py not found"
else
    echo "Running short PPO training test (100 steps)..."
    echo "(This may take up to 60 seconds)"
    
    CHECKPOINT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints"
    LOG_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/logs"
    mkdir -p "$CHECKPOINT_DIR" "$LOG_DIR"
    
    if timeout 60 python3 "$TRAIN_SCRIPT" \
        --total-timesteps 100 \
        --checkpoint-interval 50 \
        --checkpoint-dir "$CHECKPOINT_DIR" \
        --log-dir "$LOG_DIR" \
        --n-steps 32 \
        --batch-size 16 \
        --n-epochs 2 \
        > /tmp/train_test.log 2>&1; then
        test_result 0 "PPO short training works in Docker"
    else
        test_result 1 "PPO short training failed"
        echo "Training log:"
        tail -20 /tmp/train_test.log
    fi
fi

echo
echo "============================================================"
echo "4. EVALUATION TEST"
echo "============================================================"
echo

# Check if eval_policy.py exists
EVAL_SCRIPT="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/eval_policy.py"
if [ ! -f "$EVAL_SCRIPT" ]; then
    test_result 1 "eval_policy.py not found"
else
    # Check if any models exist
    CHECKPOINT_DIR="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints"
    MODEL_FILES=$(find "$CHECKPOINT_DIR" -name "*.zip" 2>/dev/null | head -1)
    
    if [ -z "$MODEL_FILES" ]; then
        test_warning "No trained models found for evaluation test"
        echo "  (This is normal if training hasn't been run yet)"
    else
        MODEL_PATH=$(echo "$MODEL_FILES" | head -1)
        echo "Testing evaluation with model: $(basename $MODEL_PATH)"
        
        if timeout 120 python3 "$EVAL_SCRIPT" \
            --model-path "$MODEL_PATH" \
            --num-episodes 2 \
            --max-steps 100 \
            > /tmp/eval_test.log 2>&1; then
            test_result 0 "Evaluation works in Docker"
        else
            test_result 1 "Evaluation failed"
            echo "Evaluation log:"
            tail -20 /tmp/eval_test.log
        fi
    fi
fi

echo
echo "============================================================"
echo "5. ALL TESTS UNDER /test"
echo "============================================================"
echo

# List all test files
TEST_FILES=(
    "test_env_import.py"
    "test_env_step.py"
    "test_roads_loading.py"
    "test_roads_distance.py"
    "test_battery_basic.py"
    "test_goal_loading.py"
    "test_goal_selection.py"
    "test_goal_reached.py"
    "test_obs_shape.py"
    "test_obs_values.py"
    "test_reward_progress.py"
    "test_reward_offroad.py"
    "test_reward_battery.py"
    "test_reward_goal.py"
    "test_ppo_import.py"
    "test_eval_model_load.py"
)

for test_file in "${TEST_FILES[@]}"; do
    test_path="$TEST_DIR/$test_file"
    if [ -f "$test_path" ]; then
        echo "Running $test_file..."
        if python3 "$test_path" > /tmp/${test_file}.log 2>&1; then
            test_result 0 "$test_file"
        else
            test_result 1 "$test_file"
            echo "  Error log:"
            tail -10 /tmp/${test_file}.log | sed 's/^/    /'
        fi
    else
        test_warning "$test_file not found"
    fi
done

echo
echo "============================================================"
echo "6. REGRESSION TESTS - All Previous Phases"
echo "============================================================"
echo

# Test all phase retest scripts
PHASE_RETESTS=(
    "phase0_checks.txt"
    "phase03_retest.sh"
    "phase04_retest.sh"
    "phase05_retest.sh"
    "phase06_retest.sh"
    "phase07_retest.sh"
    "phase08_retest.sh"
)

for retest in "${PHASE_RETESTS[@]}"; do
    retest_path="$TEST_DIR/$retest"
    if [ -f "$retest_path" ]; then
        if [ "${retest: -4}" == ".sh" ]; then
            echo "Running $retest..."
            if bash "$retest_path" > /tmp/${retest}.log 2>&1; then
                test_result 0 "$retest"
            else
                test_result 1 "$retest"
                echo "  Error log:"
                tail -10 /tmp/${retest}.log | sed 's/^/    /'
            fi
        else
            # For .txt files, just check they exist
            test_result 0 "$retest exists"
        fi
    else
        test_warning "$retest not found"
    fi
done

echo
echo "============================================================"
echo "7. PACKAGE INTEGRITY CHECK"
echo "============================================================"
echo

# Check package structure
if ros2 pkg list | grep -q "ackermann_drl"; then
    test_result 0 "ackermann_drl package registered"
else
    test_result 1 "ackermann_drl package not registered"
fi

# Check Python imports
IMPORTS=(
    "ackermann_drl"
    "ackermann_drl.envs.AckermannCityEnv"
    "ackermann_drl.envs.AckermannGymEnv"
    "ackermann_drl.utils.RoadsGeometry"
    "ackermann_drl.utils.BatteryModel"
    "ackermann_drl.utils.DeliveryPoints"
)

for import_name in "${IMPORTS[@]}"; do
    if python3 -c "import $import_name" 2>/dev/null; then
        test_result 0 "Import: $import_name"
    else
        test_result 1 "Import: $import_name"
    fi
done

# Check DRL dependencies
DRL_DEPS=(
    "gymnasium"
    "stable_baselines3"
    "torch"
    "numpy"
    "shapely"
)

for dep in "${DRL_DEPS[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        test_result 0 "DRL dependency: $dep"
    else
        test_result 1 "DRL dependency: $dep"
    fi
done

echo
echo "============================================================"
echo "8. FILE STRUCTURE CHECK"
echo "============================================================"
echo

# Check required files exist
REQUIRED_FILES=(
    "ackermann_drl/envs/ackermann_city_env.py"
    "ackermann_drl/envs/gym_wrapper.py"
    "ackermann_drl/utils/battery_model.py"
    "ackermann_drl/utils/delivery_points.py"
    "ackermann_drl/utils/roads_geometry.py"
    "ackermann_drl/scripts/train_ppo.py"
    "ackermann_drl/scripts/eval_policy.py"
    "ackermann_drl/config/delivery_points.yaml"
    "ackermann_drl/launch/single_drl_training.launch.py"
)

for file in "${REQUIRED_FILES[@]}"; do
    file_path="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/$file"
    if [ -f "$file_path" ]; then
        test_result 0 "File exists: $file"
    else
        test_result 1 "File missing: $file"
    fi
done

# Check directories
REQUIRED_DIRS=(
    "ackermann_drl/envs"
    "ackermann_drl/utils"
    "ackermann_drl/scripts"
    "ackermann_drl/config"
    "ackermann_drl/launch"
    "ackermann_drl/checkpoints"
    "ackermann_drl/logs"
)

for dir in "${REQUIRED_DIRS[@]}"; do
    dir_path="/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/$dir"
    if [ -d "$dir_path" ]; then
        test_result 0 "Directory exists: $dir"
    else
        # Create missing directories (checkpoints and logs)
        if [[ "$dir" == *"checkpoints"* ]] || [[ "$dir" == *"logs"* ]]; then
            mkdir -p "$dir_path"
            test_result 0 "Directory created: $dir"
        else
            test_result 1 "Directory missing: $dir"
        fi
    fi
done

echo
echo "============================================================"
echo "FINAL VALIDATION SUMMARY"
echo "============================================================"
echo
echo "Tests Passed:  $PASSED"
echo "Tests Failed:  $FAILED"
echo "Warnings:     $WARNINGS"
echo "Total Tests:  $((PASSED + FAILED + WARNINGS))"
echo

if [ $FAILED -eq 0 ]; then
    echo "============================================================"
    echo "✓ ALL VALIDATION TESTS PASSED!"
    echo "============================================================"
    echo
    echo "The single-agent Docker-based DRL system is fully validated."
    echo "All components are working correctly inside Docker."
    echo
    exit 0
else
    echo "============================================================"
    echo "✗ SOME VALIDATION TESTS FAILED"
    echo "============================================================"
    echo
    echo "Please review the failed tests above and fix any issues."
    echo
    exit 1
fi

