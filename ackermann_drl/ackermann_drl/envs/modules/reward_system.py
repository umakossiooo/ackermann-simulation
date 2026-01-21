import numpy as np
from typing import Dict


class RewardSystem:
    """Multi-Objective Reward System for DRL Training.
    
    Combines multiple reward/penalty signals to guide learning:
    - Progress: reward for getting closer to goal
    - Safety: penalties for collisions and off-road driving
    - Efficiency: penalties for energy consumption and aggressive changes
    - Mission: rewards for on-time delivery, penalties for being late
    """
    
    def __init__(self, config: Dict = None):
        # Reward weights (positive = reward, negative = penalty)
        config = config or {}
        if isinstance(config, dict) and isinstance(config.get('drl'), dict):
            config = config.get('drl', {})
        if isinstance(config, dict) and isinstance(config.get('env'), dict):
            config = config.get('env', {})
        rewards = config.get('rewards', {}) if isinstance(config, dict) else {}
        
        # --- Primary Objectives (Navigation) ---
        self.w_progress = rewards.get('w_progress', 2.0)         # Reward for moving closer to goal
        self.w_goal = rewards.get('w_goal', 200.0)           # HUGE reward for reaching the goal (primary objective)
        self.w_collision = rewards.get('w_collision', -100.0)     # Critical failure penalty
        self.w_offroad = rewards.get('w_offroad', -2.0)         # Strict penalty for leaving road area
        
        # --- Secondary Constraints (Safety & Comfort) ---
        self.w_obstacle_proximity = rewards.get('w_obstacle_proximity', -5.0) # Warning for getting too close to edges/obstacles
        self.w_aggressive = rewards.get('w_aggressive', -2.0)         # Penalize sharp control changes (jerk)
        self.w_acceleration = rewards.get('w_acceleration', -1.0)       # Penalize high acceleration/braking
        self.w_lateral_accel = rewards.get('w_lateral_accel', -5.0)      # Penalize high lateral acceleration (load stability)
        self.w_speeding = rewards.get('w_speeding', -2.0)           # Penalize exceeding maxspeed (per m/s)
        self.w_oneway = rewards.get('w_oneway', -10.0)              # Penalize driving against oneway
        
        # --- Tertiary Constraints (Energy/Time) ---
        self.w_time = rewards.get('w_time', -0.05)              # Time penalty to encourage speed
        self.w_energy = rewards.get('w_energy', -50.0)            # Penalty for energy consumption (weighted by load)
        self.w_battery_conservation = rewards.get('w_battery_conservation', -0.0) # Focus on consumption per step rather than total level
        self.w_delivery_late = rewards.get('w_delivery_late', -2.0)      # Penalty per second if deadline missed
        self.w_reverse = rewards.get('w_reverse', -2.0)            # Penalty for driving in reverse
        
        self.prev_dist_to_goal = None
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.prev_late_time = 0.0

    def reset(self, initial_dist):
        self.prev_dist_to_goal = initial_dist
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.prev_late_time = 0.0

    def compute_reward(self, current_dist_to_goal, is_collision, road_dist,
                      battery_consumed, battery_level, mission_status, current_vel, goal_reached,
                      acceleration, obstacle_proximity, speed_excess=0.0, oneway_violation=0.0):
        reward = 0.0
        info = {}
        
        # Progress reward
        if self.prev_dist_to_goal is not None:
            progress = self.prev_dist_to_goal - current_dist_to_goal
            reward += self.w_progress * progress
            info['reward_progress'] = self.w_progress * progress
        else:
            info['reward_progress'] = 0.0
        self.prev_dist_to_goal = current_dist_to_goal
        
        # Off-road penalty
        if road_dist > 0.0:
            penalty = self.w_offroad * road_dist
            reward += penalty
            info['penalty_offroad'] = penalty
        else:
            info['penalty_offroad'] = 0.0
        
        # Energy and battery penalties
        reward += self.w_energy * battery_consumed
        info['penalty_efficiency'] = self.w_energy * battery_consumed
        if not np.isfinite(battery_level):
            battery_level = 1.0
        battery_level = np.clip(battery_level, 0.0, 1.0)
        battery_penalty = self.w_battery_conservation * (1.0 - battery_level)
        reward += battery_penalty
        info['penalty_battery_conservation'] = battery_penalty
        
        # Acceleration penalty (quadratic)
        if not np.isfinite(acceleration):
            acceleration = 0.0
        penalty_acc = self.w_acceleration * (acceleration ** 2)
        reward += penalty_acc
        info['penalty_acceleration'] = penalty_acc
        
        # Lateral acceleration penalty (v^2/R approx or v * omega)
        # Load stability check
        linear_v = current_vel[0]
        angular_v = current_vel[1]
        lateral_accel = abs(linear_v * angular_v)
        penalty_lat = self.w_lateral_accel * (lateral_accel ** 2)
        reward += penalty_lat
        info['penalty_lateral_accel'] = penalty_lat
        
        # Reverse driving penalty
        if linear_v < -0.1:
            reward += self.w_reverse
            info['penalty_reverse'] = self.w_reverse
        else:
            info['penalty_reverse'] = 0.0
        
        # Obstacle proximity penalty
        if not np.isfinite(obstacle_proximity):
            obstacle_proximity = 0.0
        obstacle_proximity = np.clip(obstacle_proximity, 0.0, 1.0)
        penalty_obst = self.w_obstacle_proximity * obstacle_proximity
        reward += penalty_obst
        info['penalty_obstacle_proximity'] = penalty_obst

        # Speed limit and oneway penalties
        if not np.isfinite(speed_excess):
            speed_excess = 0.0
        speed_excess = max(0.0, float(speed_excess))
        penalty_speed = self.w_speeding * speed_excess
        reward += penalty_speed
        info['penalty_speeding'] = penalty_speed

        if not np.isfinite(oneway_violation):
            oneway_violation = 0.0
        oneway_violation = np.clip(oneway_violation, 0.0, 1.0)
        penalty_oneway = self.w_oneway * oneway_violation
        reward += penalty_oneway
        info['penalty_oneway'] = penalty_oneway
        
        # Aggressive change penalty (jerk)
        accel_jerk = abs(current_vel[0] - self.prev_linear_vel)
        steer_jerk = abs(current_vel[1] - self.prev_angular_vel)
        penalty = 0.0
        if accel_jerk > 0.5:  # threshold
            penalty += self.w_aggressive * accel_jerk
        if steer_jerk > 0.2:  # threshold
            penalty += self.w_aggressive * steer_jerk
        reward += penalty
        info['penalty_aggressive_change'] = penalty
        
        self.prev_linear_vel = current_vel[0]
        self.prev_angular_vel = current_vel[1]
        
        reward += self.w_time
        info['penalty_time'] = self.w_time
        
        # Deadline penalty (incremental once deadline is exceeded)
        late_penalty = 0.0
        if isinstance(mission_status, dict):
            elapsed = mission_status.get('elapsed', 0.0)
            deadline = mission_status.get('deadline', 0.0)
            late_time = max(0.0, elapsed - deadline)
            late_increment = max(0.0, late_time - self.prev_late_time)
            if late_increment > 0.0:
                late_penalty = self.w_delivery_late * late_increment
            self.prev_late_time = late_time
        else:
            self.prev_late_time = 0.0
        reward += late_penalty
        info['penalty_delivery_late'] = late_penalty
        info['reward_delivery_on_time'] = 0.0
        
        if is_collision:
            reward += self.w_collision
            info['penalty_collision'] = self.w_collision
            info['reward_goal'] = 0.0
            info['reward_delivery_on_time'] = 0.0
        elif goal_reached:
            reward += self.w_goal
            info['penalty_collision'] = 0.0
            info['reward_goal'] = self.w_goal
            
            # Check delivery deadline
            if mission_status and not mission_status.get('is_late', False):
                reward += 30.0  # Bonus for on-time delivery
                info['reward_delivery_on_time'] = 30.0
        else:
            info['penalty_collision'] = 0.0
            info['reward_goal'] = 0.0
            info['reward_delivery_on_time'] = 0.0
        
        return reward, info
