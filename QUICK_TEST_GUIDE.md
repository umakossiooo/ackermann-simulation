# Quick Test Guide - Testing with Gazebo Running

## Prerequisites
- ✅ Gazebo is running: `ros2 launch saye_bringup saye_spawn.launch.py gui:=true`
- ✅ Play button is hit in Gazebo
- ✅ Docker container is running

## Quick Test Commands

### Option 1: Run System Readiness Test
```bash
# Inside Docker container
docker compose exec ackermann_sim bash

# Run test
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_training.sh
```

### Option 2: Run Short Training Test (100 steps)
```bash
# Inside Docker container
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/run_training_test.sh
```

### Option 3: Run Full Training
```bash
# Inside Docker container
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 1000 \
    --checkpoint-interval 500
```

## What to Look For

### ✅ Success Indicators:
1. **No import errors** - All modules load correctly
2. **Environment creates** - "Creating environment..." appears
3. **Observation space** - Shows "Observation space: Box(...)"
4. **Action space** - Shows "Action space: Box(...)"
5. **PPO agent creates** - "Creating PPO agent..." appears
6. **Training starts** - "Starting training..." appears
7. **Reward logs** - `[REWARD] Step X | ...` messages appear
8. **No errors** - No tracebacks or exceptions

### ❌ Error Indicators:
1. **Import errors** - "No module named ..."
2. **ROS errors** - "rclpy context invalid"
3. **Topic errors** - "Failed to publish/subscribe"
4. **Syntax errors** - Python syntax errors
5. **Attribute errors** - "object has no attribute"

## Common Errors and Fixes

### Error: "No module named 'ackermann_drl'"
**Fix:**
```bash
cd /root/colcon_ws
colcon build --packages-select ackermann_drl
source install/setup.bash
```

### Error: "rclpy context invalid"
**Fix:** Restart ROS 2 context or restart Docker container

### Error: "No topics available"
**Fix:** Ensure Gazebo is running and play button is hit

### Error: "Failed to load roads"
**Fix:** Check if roads file is mounted:
```bash
ls /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/osm_city_pipeline/maps/bari_roads.json
```

## Expected Output

### During Training:
```
Creating environment...
Observation space: Box(187,)
Action space: Box(2,)
Creating PPO agent...
Starting training...

EPISODE 1 STARTED - Resetting environment
[REWARD] Step 1 | Progress: +0.1234 (moved 0.12m closer, dist=45.23m)
[REWARD] Step 1 | Battery: 100.0% (HIGH) | Conservation: +0.1000
[REWARD] Step 1 | Time penalty: -0.0100 (per step)
[REWARD] Step 1 | TOTAL REWARD: +0.2134
...
```

## Testing Checklist

- [ ] System readiness test passes
- [ ] Training script runs without errors
- [ ] Environment resets successfully
- [ ] Rewards are calculated
- [ ] Real-time logs appear
- [ ] Vehicle receives commands (moves in Gazebo)
- [ ] No exceptions or tracebacks

## If Errors Occur

1. **Check the error message** - Look for specific error type
2. **Check logs** - Review terminal output for details
3. **Verify Gazebo** - Ensure it's running and playing
4. **Check ROS topics** - `ros2 topic list`
5. **Restart if needed** - Restart Docker container

## Success!

If training runs without errors and you see:
- Reward logs appearing
- Vehicle moving in Gazebo
- No exceptions

Then everything is working correctly! ✅

