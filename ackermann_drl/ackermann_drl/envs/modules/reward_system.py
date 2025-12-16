import numpy as np
from typing import Dict, Tuple


class RewardSystem:
    """Multi-Objective Reward System for DRL Training.
    
    Combines multiple reward/penalty signals to guide learning:
    - Progress: reward for getting closer to goal
    - Safety: penalties for collisions and off-road driving
    - Efficiency: penalties for energy consumption and aggressive changes
    - Mission: rewards for on-time delivery, penalties for being late
    """
    
    def __init__(self):
        # Reward weights (positive = reward, negative = penalty)
        self.w_progress = 4.0  # Reward for moving toward goal
        self.w_goal = 100.0  # Large reward for reaching goal
        self.w_collision = -50.0  # Heavy penalty for collisions
        self.w_offroad = -1.0  # Penalty for driving off roads
        self.w_path_deviation = -0.5  # Penalty for deviating from planned path
        self.w_energy = -30.0  # Penalty for energy consumption
        self.w_battery_conservation = -0.1  # Penalty for low battery
        self.w_time = -0.01  # Small time penalty per step
        self.w_delivery_late = -0.5  # Penalty for late delivery
        self.w_aggressive = -0.3  # Penalty for sudden velocity changes
        
        # Track previous state for computing progress and jerk
        self.prev_dist_to_goal = None
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0

    def reset(self, initial_dist):
        self.prev_dist_to_goal = initial_dist
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0

    def compute_reward(self, current_dist_to_goal, is_collision, road_dist, cross_track_error,
                      battery_consumed, battery_level, mission_status, current_vel, goal_reached):
        reward = 0.0
        info = {}
        
        # Progress reward: positive if getting closer to goal
        if self.prev_dist_to_goal is not None:
            progress = self.prev_dist_to_goal - current_dist_to_goal
            reward += self.w_progress * progress
            info['reward_progress'] = self.w_progress * progress
        else:
            info['reward_progress'] = 0.0
        self.prev_dist_to_goal = current_dist_to_goal
        
        # Path deviation penalty: penalize if too far from planned path
        if cross_track_error > 1.5:
            penalty = self.w_path_deviation * (cross_track_error ** 1.5)
            reward += penalty
            info['penalty_path_deviation'] = penalty
        else:
            info['penalty_path_deviation'] = 0.0
        
        # Off-road penalty: penalize driving away from road centerlines
        if road_dist > 0.0:
            penalty = self.w_offroad * road_dist
            reward += penalty
            info['penalty_offroad'] = penalty
        else:
            info['penalty_offroad'] = 0.0
        
        # Energy efficiency: penalize energy consumption
        reward += self.w_energy * battery_consumed
        info['penalty_efficiency'] = self.w_energy * battery_consumed
        
        # Battery conservation: penalize low battery levels
        reward += self.w_battery_conservation * (1.0 - battery_level)
        info['penalty_battery_conservation'] = self.w_battery_conservation * (1.0 - battery_level)
        
        # Aggressive change penalty: penalize sudden velocity changes (jerk)
        accel_jerk = abs(current_vel[0] - self.prev_linear_vel)
        steer_jerk = abs(current_vel[1] - self.prev_angular_vel)
        penalty = 0.0
        if accel_jerk > 0.7:  # Threshold for acceleration jerk
            penalty += self.w_aggressive * accel_jerk
        if steer_jerk > 0.3:  # Threshold for steering jerk
            penalty += self.w_aggressive * steer_jerk
        reward += penalty
        info['penalty_aggressive_change'] = penalty
        self.prev_linear_vel = current_vel[0]
        self.prev_angular_vel = current_vel[1]
        
        reward += self.w_time
        info['penalty_time'] = self.w_time
        
        if is_collision:
            reward += self.w_collision
            info['penalty_collision'] = self.w_collision
            info['reward_goal'] = 0.0
            info['reward_delivery_on_time'] = 0.0
            info['penalty_delivery_late'] = 0.0
        elif goal_reached:
            reward += self.w_goal
            info['penalty_collision'] = 0.0
            info['reward_goal'] = self.w_goal
            if mission_status['is_late']:
                late_penalty = self.w_delivery_late * (mission_status['elapsed'] - mission_status['deadline'])
                reward += late_penalty
                info['penalty_delivery_late'] = late_penalty
                info['reward_delivery_on_time'] = 0.0
            else:
                reward += 30.0
                info['penalty_delivery_late'] = 0.0
                info['reward_delivery_on_time'] = 30.0
        else:
            info['penalty_collision'] = 0.0
            info['reward_goal'] = 0.0
            info['reward_delivery_on_time'] = 0.0
            info['penalty_delivery_late'] = 0.0
        
        return reward, info
