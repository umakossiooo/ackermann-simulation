# Sliding Mode Control (SMC) Guide

This guide explains how to use the Sliding Mode Control (SMC) controller that replaces the DRL agent for path following.

## Overview

The SMC controller uses classical control theory to follow road polylines:
- **Input**: Robot position, heading, road geometry
- **Output**: Velocity and steering commands
- **Method**: Sliding Mode Control with boundary layer to reduce chattering

## Architecture

### Components

1. **SlidingModeController** (`ackermann_drl/utils/sliding_mode_control.py`)
   - Implements SMC algorithm
   - Computes steering and velocity commands
   - Handles boundary layer for smooth control

2. **RoadsGeometry** (enhanced)
   - Calculates distance to nearest road
   - Computes desired heading from road polylines
   - Determines which side of road robot is on

3. **SMCControlNode** (`ackermann_drl/scripts/smc_control_node.py`)
   - ROS 2 node that runs SMC controller
   - Subscribes to `/odom` and `/scan`
   - Publishes to `/cmd_vel`
   - Handles obstacle avoidance

## Usage

### Method 1: Direct Python Script

```bash
# Inside Docker container
docker compose exec ackermann_sim bash

# Run SMC controller
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/smc_control_node.py
```

### Method 2: Using Launch File

```bash
# Inside Docker container
ros2 launch ackermann_drl smc_control.launch.py \
  lambda_param:=1.0 \
  K_smc:=2.0 \
  boundary_layer:=0.1 \
  desired_velocity:=2.0 \
  max_steering:=1.0
```

### Method 3: With Gazebo Spawn

```bash
# Terminal 1: Start Gazebo
ros2 launch saye_bringup saye_spawn.launch.py gui:=true

# Terminal 2: Start SMC controller
ros2 launch ackermann_drl smc_control.launch.py
```

## Parameters

### SMC Controller Parameters

- **`lambda_param`** (default: 1.0)
  - Weight for heading error in sliding surface
  - Higher = more emphasis on heading correction
  - Range: 0.1 - 5.0

- **`K_smc`** (default: 2.0)
  - Control gain (switching gain)
  - Higher = more aggressive control
  - Range: 0.5 - 10.0

- **`boundary_layer`** (default: 0.1)
  - Thickness of boundary layer (reduces chattering)
  - Larger = smoother but slower response
  - Range: 0.01 - 0.5

- **`desired_velocity`** (default: 2.0 m/s)
  - Target forward velocity
  - Range: 0.5 - 5.0 m/s

- **`max_steering`** (default: 1.0 rad/s)
  - Maximum angular velocity command
  - Range: 0.5 - 2.0 rad/s

### Obstacle Avoidance Parameters

- **`obstacle_threshold`**: 1.5m (stop if obstacle closer)
- **`slowdown_threshold`**: 3.0m (slow down if obstacle closer)

## How It Works

### 1. Sliding Surface

The sliding surface combines lateral and heading errors:

```
s = e_y + λ * e_θ
```

Where:
- `e_y`: Lateral error (distance from road, signed)
- `e_θ`: Heading error (angle difference)
- `λ`: Weight parameter

### 2. Control Law

```
if |s| > boundary_layer:
    steering = -K * sign(s)  # Switching control
else:
    steering = -K * s / boundary_layer  # Linear control (smooth)
```

### 3. Path Following Flow

```
1. Get robot position from /odom
   ↓
2. Find nearest road polyline
   ↓
3. Calculate:
   - Lateral error (signed distance from road)
   - Heading error (desired - current heading)
   ↓
4. Compute sliding surface: s = e_y + λ*e_θ
   ↓
5. Apply SMC control law
   ↓
6. Publish cmd_vel (velocity, steering)
   ↓
7. Repeat at 50 Hz
```

## Tuning Guide

### For Smooth Following (Less Aggressive)

```python
lambda_param = 0.5      # Less emphasis on heading
K_smc = 1.0             # Lower gain
boundary_layer = 0.2    # Larger boundary layer
desired_velocity = 1.5  # Slower speed
```

### For Aggressive Following (Faster Response)

```python
lambda_param = 2.0      # More emphasis on heading
K_smc = 3.0             # Higher gain
boundary_layer = 0.05   # Smaller boundary layer
desired_velocity = 3.0  # Faster speed
```

### For Tight Turns

```python
lambda_param = 1.5      # Balance lateral/heading
K_smc = 2.5             # Moderate gain
boundary_layer = 0.1   # Standard boundary layer
desired_velocity = 1.5  # Slower for turns
```

## Comparison: DRL vs SMC

| Feature | DRL (PPO) | SMC |
|---------|-----------|-----|
| **Learning** | Yes (requires training) | No (model-based) |
| **Adaptability** | High (learns from experience) | Medium (requires tuning) |
| **Obstacle Avoidance** | Learned | Rule-based (LiDAR) |
| **Path Following** | Learned | Mathematical (precise) |
| **Setup Time** | Long (training) | Immediate (ready to use) |
| **Robustness** | Depends on training | High (theoretical guarantees) |
| **Computational** | High (neural network) | Low (simple calculations) |

## Troubleshooting

### Vehicle Not Following Road

1. Check if roads are loaded:
   ```python
   # In Python
   from ackermann_drl.utils.roads_geometry import RoadsGeometry
   roads = RoadsGeometry()
   print(f"Loaded {roads.get_all_roads_count()} roads")
   ```

2. Verify robot position is on a road:
   - Check `/odom` topic
   - Ensure robot spawns on a road centerline

3. Adjust parameters:
   - Increase `K_smc` for more aggressive control
   - Decrease `boundary_layer` for faster response

### Chattering (Oscillations)

- Increase `boundary_layer` (e.g., 0.2)
- Decrease `K_smc` (e.g., 1.5)
- Increase `lambda_param` to emphasize heading

### Too Slow Response

- Decrease `boundary_layer` (e.g., 0.05)
- Increase `K_smc` (e.g., 3.0)
- Check if obstacle avoidance is triggering

### Vehicle Stopping Frequently

- Check LiDAR for obstacles
- Adjust `obstacle_threshold` and `slowdown_threshold`
- Verify road geometry is correct

## Advanced: Custom SMC Controller

You can create a custom SMC controller:

```python
from ackermann_drl.utils.sliding_mode_control import SlidingModeController

# Create controller with custom parameters
smc = SlidingModeController(
    lambda_param=1.5,
    K_smc=2.5,
    boundary_layer=0.15,
    desired_velocity=2.5
)

# Compute control
steering, velocity = smc.compute_control(
    lateral_error=0.2,      # 0.2m to the right
    heading_error=0.1,      # 0.1 rad pointing right
    current_velocity=2.0
)
```

## Integration with Existing System

The SMC controller can be used alongside or instead of DRL:

- **Replace DRL**: Use SMC for all control
- **Hybrid**: Use DRL for high-level planning, SMC for path following
- **Comparison**: Run both and compare performance

## References

- Sliding Mode Control theory
- Ackermann vehicle kinematics
- Path following algorithms
- Road geometry calculations

