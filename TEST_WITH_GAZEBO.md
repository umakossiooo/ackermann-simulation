# Testing with Gazebo Running

## Prerequisites
- ✅ Gazebo is running (`ros2 launch saye_bringup saye_spawn.launch.py gui:=true`)
- ✅ Play button is hit in Gazebo
- ✅ Docker container is running

## Quick Test Commands

### Test 1: System Readiness (Inside Docker)
```bash
# Enter Docker container
docker compose exec ackermann_sim bash

# Run system readiness test
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test_system_ready.py
```

### Test 2: SMC Controller (With Gazebo Running)
```bash
# Inside Docker container
# Terminal 1: Gazebo should already be running
# Terminal 2: Run SMC controller
ros2 launch ackermann_drl smc_control.launch.py
```

**Expected behavior:**
- Vehicle should start following roads
- Real-time logs showing battery, road distance, steering, velocity
- Vehicle should avoid obstacles
- Metrics logged every 10 seconds

### Test 3: DRL Training (With Gazebo Running)
```bash
# Inside Docker container
# Terminal 1: Gazebo should already be running
# Terminal 2: Start training
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py --total-timesteps 1000
```

**Expected behavior:**
- Environment resets and selects delivery point
- Real-time reward logs every step
- Reward breakdown every 25 steps
- Vehicle learns to navigate

## What to Check

### ✅ SMC Controller Should Show:
```
[REWARD] Step X | Battery: 85.3% (HIGH) | Conservation: +0.1000
[REWARD] Step X | Time penalty: -0.0100 (per step)
[REWARD] Step X | TOTAL REWARD: +0.2345
SMC Control: lateral_error=0.123m, heading_error=5.2°, steering=0.234, velocity=2.00m/s
```

### ✅ DRL Training Should Show:
```
[REWARD] Step X | Progress: +0.5000 (moved 0.50m closer, dist=45.23m)
[REWARD] Step X | Battery: 90.0% (HIGH) | Conservation: +0.1000
[REWARD] Step X | Time penalty: -0.0100 (per step)
[REWARD] Step X | TOTAL REWARD: +0.5900
```

### ✅ Vehicle Behavior:
- Should move forward
- Should follow roads (stay on road centerlines)
- Should avoid obstacles (stop if too close)
- Should respond to commands

## Troubleshooting

### Vehicle Not Moving
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Verify SMC/DRL node is publishing commands
- Check Gazebo is receiving commands

### No Sensor Data
- Check `/odom` topic: `ros2 topic echo /odom --once`
- Check `/scan` topic: `ros2 topic echo /scan --once`
- Verify ROS 2 bridge is working

### Import Errors
- Ensure you're inside Docker container
- Check package is built: `colcon build --packages-select ackermann_drl`
- Source workspace: `source install/setup.bash`

### No Roads/Low Battery
- Check roads file is mounted: `ls /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/osm_city_pipeline/maps/bari_roads.json`
- Battery starts at 100%, should decrease over time
- Check logs for warnings

## Success Criteria

✅ **System is working if:**
1. No import errors
2. Vehicle receives commands (moves in Gazebo)
3. Real-time logs appear
4. Rewards/penalties are calculated
5. Vehicle follows roads (SMC) or learns (DRL)

## Next Steps

Once tests pass:
- For SMC: Vehicle should autonomously follow roads
- For DRL: Training should progress, rewards should increase over time

