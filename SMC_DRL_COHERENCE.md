# SMC and DRL Coherence Documentation

## Overview

This document explains the relationship between the Sliding Mode Control (SMC) controller and the Deep Reinforcement Learning (DRL) environment, and how they interact with the reward/penalty system.

## Key Points

### 1. **Separate Systems - Cannot Run Simultaneously**

**Important**: SMC and DRL are **mutually exclusive** - they cannot run at the same time because:

- Both publish to `/cmd_vel` topic
- Both subscribe to `/odom` and `/scan` topics
- Running both would cause command conflicts

**Usage:**
- **SMC**: Direct control, no training needed
- **DRL**: Learning-based, requires training

### 2. **Reward/Penalty System**

#### DRL Environment (`AckermannCityEnv`)
- **Uses rewards/penalties**: Yes, extensively
- **Purpose**: Train neural network agent
- **Reward components**:
  - Progress reward: `+1.0 × (distance_closer)`
  - Goal reward: `+50.0` when goal reached
  - On-time delivery: `+30.0` if delivered on time
  - Battery conservation: Zone-based (`+0.1` to `-0.05`)
  - Efficiency reward: `+0.2 × (progress/battery_consumed)`
  - Off-road penalty: `-0.05 × distance_offroad`
  - Collision penalty: `-20.0` if collision
  - High-speed penalty: `-0.02 × (velocity - 3.0)` if > 3.0 m/s
  - Aggressive change penalty: `-0.05 × (change - 1.0)` if > 1.0 m/s
  - Time penalty: `-0.01` per step

#### SMC Controller (`SMCControlNode`)
- **Uses rewards/penalties**: **No** (for control), **Yes** (for metrics/evaluation)
- **Purpose**: Direct mathematical control
- **Control method**: Sliding surface `s = e_y + λ·e_θ`
- **Reward tracking**: Added for **comparison/evaluation only**
  - Same reward formula as DRL environment
  - Used to compare SMC vs DRL performance
  - Not used for control decisions

### 3. **Constraint Coherence**

Both systems respect the **same constraints**:

| Constraint | DRL Environment | SMC Controller | Threshold |
|------------|----------------|----------------|-----------|
| **LiDAR Collision** | ✅ Penalty: `-20.0` | ✅ Stops vehicle | `1.5m` |
| **Off-road Collision** | ✅ Penalty: `-20.0` | ✅ Detected | `1.0m` |
| **Off-road Penalty** | ✅ `-0.05 × distance` | ✅ Tracked (metrics) | `> 0.0m` |
| **Obstacle Avoidance** | ✅ Via learning | ✅ Rule-based (LiDAR) | `1.5m` stop, `3.0m` slow |

### 4. **Topic Usage**

#### Shared Topics (Both Systems)

| Topic | Type | DRL Usage | SMC Usage |
|-------|------|-----------|-----------|
| `/odom` | Subscribe | Position, velocity, heading | Position, velocity, heading |
| `/scan` | Subscribe | LiDAR for observation | LiDAR for obstacle avoidance |
| `/cmd_vel` | **Publish** | **Commands from agent** | **Commands from SMC** |

**⚠️ Conflict**: Both publish to `/cmd_vel` - only one can run at a time!

### 5. **Reward System Details**

#### DRL Environment Rewards (Training)

```python
# In ackermann_city_env.py - compute_reward()

Total Reward = 
  + Progress Reward (getting closer to goal)
  + Goal Reward (reaching delivery point)
  + On-time Delivery Reward
  + Battery Conservation Reward (zone-based)
  + Efficiency Reward
  - Late Delivery Penalty
  - Off-road Penalty
  - Collision Penalty
  - High Speed Penalty
  - Aggressive Change Penalty
  - Time Penalty
```

#### SMC Controller Rewards (Evaluation Only)

```python
# In smc_control_node.py - _compute_reward()

Reward = 
  + Small progress reward (staying on road)
  - Off-road Penalty (same as DRL)
  - Collision Penalty (same as DRL)
  - Time Penalty (same as DRL)

# Note: SMC doesn't use rewards for control,
# only tracks them for performance comparison
```

### 6. **Coherence Checklist**

✅ **Same collision thresholds**: Both use `1.5m` for LiDAR, `1.0m` for off-road  
✅ **Same off-road penalty**: Both use `-0.05 × distance`  
✅ **Same collision penalty**: Both use `-20.0`  
✅ **Same time penalty**: Both use `-0.01` per step  
✅ **Same road geometry**: Both use `RoadsGeometry` class  
✅ **Same sensor data**: Both use `/odom` and `/scan`  

❌ **Different control methods**: DRL uses neural network, SMC uses mathematical control  
❌ **Different goals**: DRL learns from rewards, SMC follows road directly  
❌ **Cannot run together**: Both publish to `/cmd_vel`  

### 7. **When to Use Which**

#### Use **DRL** when:
- You want the agent to **learn** from experience
- You need **adaptability** to new situations
- You have time for **training**
- You want **complex behavior** (delivery, battery management, etc.)

#### Use **SMC** when:
- You need **immediate control** (no training)
- You want **precise path following**
- You need **theoretical guarantees**
- You want **low computational cost**
- You're **comparing** with DRL performance

### 8. **Running the Systems**

#### Running DRL (Training)
```bash
# Terminal 1: Start Gazebo
ros2 launch saye_bringup saye_spawn.launch.py gui:=true

# Terminal 2: Train DRL agent
python3 ackermann_drl/scripts/train_ppo.py
```

#### Running SMC (Direct Control)
```bash
# Terminal 1: Start Gazebo
ros2 launch saye_bringup saye_spawn.launch.py gui:=true

# Terminal 2: Run SMC controller
ros2 launch ackermann_drl smc_control.launch.py
```

**⚠️ Do NOT run both simultaneously!**

### 9. **Reward Tracking in SMC**

SMC tracks rewards for **evaluation purposes only**:

```python
# Metrics tracked (for comparison with DRL)
self.metrics = {
    'total_steps': 0,
    'collision_count': 0,
    'offroad_count': 0,
    'offroad_distance_sum': 0.0,
    'total_reward': 0.0,
    'goal_reached': False,
}
```

These metrics allow you to:
- Compare SMC vs DRL performance
- Evaluate which system is better
- Use same reward scale for fair comparison

### 10. **Summary**

| Aspect | DRL | SMC |
|--------|-----|-----|
| **Control Method** | Neural network (learned) | Mathematical (SMC) |
| **Uses Rewards** | Yes (for training) | No (for control), Yes (for metrics) |
| **Training Required** | Yes | No |
| **Publishes to** | `/cmd_vel` | `/cmd_vel` |
| **Can Run Together** | ❌ No | ❌ No |
| **Reward Formula** | Full (all components) | Simplified (for comparison) |
| **Constraints** | Same thresholds | Same thresholds |

## Conclusion

The systems are **coherent** in terms of:
- ✅ Same constraints and thresholds
- ✅ Same reward formulas (for comparison)
- ✅ Same sensor usage
- ✅ Same road geometry

But they are **separate** in terms of:
- ❌ Control method (learning vs mathematical)
- ❌ Cannot run simultaneously
- ❌ Different purposes (training vs direct control)

The reward system in SMC is for **evaluation/comparison only**, not for control decisions.

