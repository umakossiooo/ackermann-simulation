# Test Training Now - Quick Guide

## Your Gazebo is Running ✅
You have Gazebo launched with the car. Now let's test the training!

## Quick Test (Inside Docker)

### Step 1: Enter Docker Container
```bash
docker compose exec ackermann_sim bash
```

### Step 2: Run Live Test (Verifies Everything Works)
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_training_live.py
```

This will:
- ✅ Check ROS 2 is working
- ✅ Verify topics are available
- ✅ Test environment creation
- ✅ Run 5 test steps
- ✅ Show reward logs in real-time

### Step 3: Run Actual Training
If the test passes, run training:
```bash
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 500 \
    --checkpoint-interval 250
```

## What You Should See

### During Test:
```
[REWARD] Step 1 | Progress: +0.1234 (moved 0.12m closer, dist=45.23m)
[REWARD] Step 1 | Battery: 100.0% (HIGH) | Conservation: +0.1000
[REWARD] Step 1 | Time penalty: -0.0100 (per step)
[REWARD] Step 1 | TOTAL REWARD: +0.2134
```

### During Training:
- Real-time reward logs every step
- Reward breakdown showing all components
- Episode summaries
- No errors or exceptions

## If You See Errors

### Error: "No module named 'ackermann_drl'"
```bash
cd /root/colcon_ws
colcon build --packages-select ackermann_drl
source install/setup.bash
```

### Error: "rclpy context invalid"
- Restart Docker container
- Or restart ROS 2: `rclpy.shutdown()` then `rclpy.init()`

### Error: "No topics available"
- Make sure Gazebo is running
- Make sure play button is hit
- Check: `ros2 topic list`

### Error: "Failed to load roads"
- Check file exists: `ls /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/osm_city_pipeline/maps/bari_roads.json`

## Success Indicators ✅

1. **Test script runs without errors**
2. **Environment creates successfully**
3. **Reward logs appear in real-time**
4. **Vehicle moves in Gazebo**
5. **No exceptions or tracebacks**

## Expected Log Format

Every step you should see:
```
[REWARD] Step X | Progress: +X.XXXX
[REWARD] Step X | Battery: XX.X% (ZONE) | Conservation: +X.XXXX
[REWARD] Step X | Time penalty: -X.XXXX
[REWARD] Step X | TOTAL REWARD: +X.XXXX
```

Plus additional logs for:
- Goal reached
- On-time delivery
- Late delivery
- Off-road
- Collision
- Battery efficiency

## Ready to Test!

Run the test script now and verify everything works! 🚀

