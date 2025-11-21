# Quick Topic Check Commands

## Check if Topics Exist

```bash
# Inside Docker container
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

# List all topics
ros2 topic list

# Filter for required topics (note: pipe ros2 topic list INTO grep)
ros2 topic list | grep -E '(odom|scan|cmd_vel)'
```

## Check if Topics Have Data

```bash
# Check topic publishing rate
ros2 topic hz /odom
ros2 topic hz /scan

# Get one message from each topic
ros2 topic echo /odom --once
ros2 topic echo /scan --once

# Check topic info (type, publishers, subscribers)
ros2 topic info /odom
ros2 topic info /scan
ros2 topic info /cmd_vel
```

## Verify System is Ready

```bash
# Check all required topics exist
ros2 topic list | grep -E '(odom|scan|cmd_vel)'
# Should output:
# /cmd_vel
# /odom
# /scan

# Check topics are publishing (should show rate)
ros2 topic hz /odom --window 5
# Should show: average rate: ~5-10 Hz

ros2 topic hz /scan --window 5
# Should show: average rate: ~5-10 Hz
```

## Common Issues

### "No topics found"
- Gazebo is not running
- Solution: Start Gazebo with `ros2 launch saye_bringup saye_spawn.launch.py gui:=true`

### "Topics exist but no data"
- Gazebo is paused
- Solution: Press play in Gazebo GUI

### "grep returns nothing"
- You're using `grep` without input
- Solution: Use `ros2 topic list | grep -E '(odom|scan|cmd_vel)'` (note the pipe `|`)

