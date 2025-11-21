# RViz Visualization Troubleshooting

## Common Issues and Solutions

### 1. Robot appears as a simple box or wireframe

**Problem:** Mesh files not found or not accessible.

**Check:**
```bash
# Inside Docker container
ros2 run tf2_ros tf2_echo odom saye/base_link
ros2 topic echo /robot_description --once | grep -i mesh
```

**Solution:**
- Verify mesh files exist:
  ```bash
  ls -la /root/colcon_ws/install/saye_description/share/saye_description/models/saye/meshes/
  ```
- Check `GZ_SIM_RESOURCE_PATH` includes the package:
  ```bash
  echo $GZ_SIM_RESOURCE_PATH
  ```

### 2. Robot doesn't appear at all

**Problem:** Robot description not published or TF frames missing.

**Check:**
```bash
# Check robot_description topic
ros2 topic list | grep robot_description
ros2 topic hz /robot_description

# Check TF frames
ros2 run tf2_ros tf2_echo odom saye/base_link
ros2 run tf2_tools view_frames
```

**Solution:**
- Verify `robot_state_publisher` is running:
  ```bash
  ros2 node list | grep robot_state
  ```
- Check fixed frame in RViz matches your TF tree (usually `odom` or `map`)

### 3. Robot appears in wrong position

**Problem:** TF frame mismatch or incorrect fixed frame.

**Check:**
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# This creates frames.pdf - check the frame hierarchy
```

**Solution:**
- In RViz, set **Fixed Frame** to `odom` (or `map` if using SLAM)
- Verify TF prefix: frames should be `saye/base_link`, not `base_link`

### 4. Robot parts missing (wheels, etc.)

**Problem:** URDF parsing issue or missing joint definitions.

**Check:**
```bash
# Validate URDF
check_urdf /root/colcon_ws/install/saye_description/share/saye_description/urdf/saye.urdf
```

**Solution:**
- Check RViz RobotModel display settings:
  - **TF Prefix:** `saye`
  - **Description Topic:** `/robot_description`
  - **Visual Enabled:** ✓
  - **Collision Enabled:** (optional)

### 5. Robot appears but doesn't move

**Problem:** TF not updating or odometry not publishing.

**Check:**
```bash
# Check odometry topic
ros2 topic hz /odom
ros2 topic echo /odom --once

# Check if odom->saye transform exists
ros2 run tf2_ros tf2_echo odom saye
```

**Solution:**
- If using `saye_spawn.launch.py` without SLAM, you may need `odom_to_tf.py`:
  ```bash
  # This should be running automatically, but check:
  ros2 node list | grep odom_to_tf
  ```

## Quick Diagnostic Commands

```bash
# Inside Docker container
source /opt/ros/jazzy/setup.bash
source /root/colcon_ws/install/setup.bash

# 1. Check all topics
ros2 topic list

# 2. Check robot description is published
ros2 topic hz /robot_description

# 3. Check TF tree
ros2 run tf2_tools view_frames

# 4. Check specific transform
ros2 run tf2_ros tf2_echo odom saye/base_link

# 5. List all TF frames
ros2 run tf2_ros tf2_monitor
```

## RViz Settings Checklist

In RViz, verify:

1. **Global Options:**
   - Fixed Frame: `odom` (or `map` if using SLAM)

2. **RobotModel Display:**
   - Enabled: ✓
   - Description Topic: `/robot_description`
   - TF Prefix: `saye`
   - Visual Enabled: ✓
   - Collision Enabled: (optional)

3. **TF Display:**
   - Enabled: ✓
   - Show Names: ✓
   - Show Axes: ✓
   - Frame Timeout: 15

## Expected TF Tree

```
odom
└── saye (from odom_to_tf.py or odometry)
    └── saye/base_link (from robot_state_publisher)
        ├── saye/front_left_wheel_steering_link
        │   └── saye/front_left_wheel_link
        ├── saye/front_right_wheel_steering_link
        │   └── saye/front_right_wheel_link
        ├── saye/rear_left_wheel_link
        └── saye/rear_right_wheel_link
```

If you see the robot but it looks wrong, please describe what you see (e.g., "just a box", "missing wheels", "wrong position", etc.) and I can provide more specific help.

