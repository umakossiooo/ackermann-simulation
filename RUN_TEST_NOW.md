# Run Test Now - Quick Instructions

## You're Inside Docker with Gazebo Running ✅

### Option 1: Python Test (Recommended - Most Comprehensive)
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_training_verify.py
```

This will:
- ✅ Check ROS topics
- ✅ Verify imports
- ✅ Test environment creation
- ✅ Run actual training (100 steps) with your exact command
- ✅ Monitor logs in real-time
- ✅ Report any errors

### Option 2: Bash Test (Quick Check)
```bash
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_training_command.sh
```

### Option 3: Run Training Directly
If tests pass, run the full command:
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 1000000 \
    --checkpoint-interval 50000 \
    --learning-rate 3e-4 \
    --batch-size 128 \
    --n-steps 2048 \
    --device auto
```

## What to Look For

### ✅ Success Indicators:
1. **Topics found** - `/odom`, `/scan`, `/cmd_vel` all available
2. **Imports work** - No "No module named" errors
3. **Environment creates** - "Creating environment..." appears
4. **Reward logs** - `[REWARD] Step X | ...` messages appear
5. **No errors** - No tracebacks or exceptions
6. **Vehicle moves** - Car responds in Gazebo

### ❌ Error Indicators:
- "No module named" → Build package: `cd /root/colcon_ws && colcon build --packages-select ackermann_drl && source install/setup.bash`
- "topic not found" → Check Gazebo is running with play button on
- "rclpy context invalid" → Restart Docker container

## Expected Output

You should see logs like:
```
[REWARD] Step 1 | Progress: +0.1234 (moved 0.12m closer, dist=45.23m)
[REWARD] Step 1 | Battery: 100.0% (HIGH) | Conservation: +0.1000
[REWARD] Step 1 | Time penalty: -0.0100 (per step)
[REWARD] Step 1 | TOTAL REWARD: +0.2134
```

## Ready to Test!

Run the Python test script now:
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_training_verify.py
```

