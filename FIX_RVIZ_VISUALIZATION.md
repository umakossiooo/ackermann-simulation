# Fix RViz Visualization Issue

## Problem
The robot doesn't appear correctly in RViz because:
1. **`robot_state_publisher` is not running** - No `/robot_description` topic
2. **No TF frames published** - RViz can't find the robot
3. **Missing `odom` frame** - No `odom_to_tf` node when using `saye_spawn.launch.py` alone

## Solution

### Option 1: Launch with RViz (Recommended)
```bash
# Inside Docker container
ros2 launch saye_bringup saye_spawn.launch.py rviz:=true
```

This will:
- Start `robot_state_publisher` (publishes `/robot_description`)
- Launch RViz automatically
- Set up TF frames correctly

### Option 2: Manually Start robot_state_publisher

If you already launched Gazebo without RViz:

```bash
# Inside Docker container
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

# Get URDF content
URDF_FILE=/root/colcon_ws/install/saye_description/share/saye_description/urdf/saye.urdf

# Start robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat $URDF_FILE)" \
    -p frame_prefix:=saye/
```

### Option 3: Add odom_to_tf for Complete TF Tree

If you need the `odom` frame (for navigation/SLAM):

```bash
# Start odom_to_tf node
ros2 run saye_bringup odom_to_tf \
    --ros-args \
    -p base_frame:=saye \
    -p odom_frame:=odom \
    -p use_sim_time:=true
```

## RViz Settings

Once `robot_state_publisher` is running:

1. **Fixed Frame:** Set to `saye/base_link` (if no odom) or `odom` (if odom_to_tf is running)
2. **RobotModel Display:**
   - Description Topic: `/robot_description`
   - TF Prefix: `saye`
   - Visual Enabled: ✓

## Verify It's Working

```bash
# Check robot_description is published
ros2 topic hz /robot_description

# Check TF frames
ros2 run tf2_ros tf2_echo saye saye/base_link

# List all TF frames
ros2 run tf2_tools view_frames
```

## Quick Fix Command

If you're already in the container with Gazebo running:

```bash
# One-liner to start robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat /root/colcon_ws/install/saye_description/share/saye_description/urdf/saye.urdf)" \
    -p frame_prefix:=saye/ &
```

Then refresh RViz (or restart it) and the robot should appear!

