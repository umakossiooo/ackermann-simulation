# Quick Start Training - Step by Step

## Yes, you need to run training INSIDE the container, and Gazebo MUST be running in the background!

## Complete Setup (2 Terminals)

### Terminal 1: Start Gazebo (Background)

```bash
# Start Gazebo in background (headless mode)
docker compose exec -d ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   ros2 launch saye_bringup saye_spawn.launch.py world:=bari_world.sdf gui:=false"

# Wait a few seconds for Gazebo to start
sleep 5

# Verify Gazebo is running (check for topics)
docker compose exec ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   ros2 topic list | grep -E '(odom|scan|cmd_vel)'"
```

### Terminal 2: Start Training

```bash
# Enter container
docker compose exec ackermann_sim bash

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

# Verify Gazebo is running (topics should be available)
ros2 topic list
# Should see: /cmd_vel, /odom, /scan

# Start training
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
    --total-timesteps 1000000 \
    --checkpoint-interval 50000 \
    --learning-rate 3e-4 \
    --batch-size 128 \
    --n-steps 2048 \
    --device auto
```

## Alternative: All in One Script

Create a script to start everything:

```bash
#!/bin/bash
# start_training.sh

# Start Gazebo in background
echo "Starting Gazebo..."
docker compose exec -d ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   ros2 launch saye_bringup saye_spawn.launch.py world:=bari_world.sdf gui:=false"

# Wait for Gazebo to initialize
echo "Waiting for Gazebo to start..."
sleep 10

# Verify topics
echo "Verifying ROS topics..."
docker compose exec ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   timeout 5 ros2 topic echo /odom --once" || {
    echo "ERROR: Gazebo not responding. Check if it's running."
    exit 1
}

# Start training
echo "Starting training..."
docker compose exec ackermann_sim bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /root/colcon_ws/install/setup.bash && \
   python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py \
     --total-timesteps 1000000 \
     --checkpoint-interval 50000"
```

## What Needs to Run

### ✅ Required (Background):
- **Gazebo** with bari_world.sdf
- **ROS 2 topics** (/odom, /scan, /cmd_vel)

### ✅ Required (Foreground):
- **Training script** (train_ppo.py)

## Verification Checklist

Before starting training, verify:

```bash
# Inside container
docker compose exec ackermann_sim bash
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

# 1. Check topics exist
ros2 topic list
# Should see: /cmd_vel, /odom, /scan

# 2. Check topics have data
ros2 topic echo /odom --once
# Should show odometry data (not empty)

ros2 topic echo /scan --once
# Should show laser scan data (not empty)

# 3. Check robot is spawned
ros2 topic echo /tf --once
# Should show transform data
```

## Common Issues

### "No topics found"
**Problem:** Gazebo is not running
**Solution:** Start Gazebo first (Terminal 1)

### "Topics exist but no data"
**Problem:** Gazebo is starting but not ready
**Solution:** Wait 10-15 seconds, then check again

### "Environment reset fails"
**Problem:** Topics not receiving data
**Solution:** Verify Gazebo is fully started and robot is spawned

## Summary

**YES, you need:**
1. ✅ Run training command **INSIDE container**
2. ✅ Run Gazebo **IN BACKGROUND** (separate terminal or background process)
3. ✅ Verify ROS topics are available before training

**Training won't work without Gazebo running!**

