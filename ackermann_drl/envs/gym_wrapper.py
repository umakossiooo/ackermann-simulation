"""Gymnasium wrapper for AckermannCityEnv.

MUST RUN INSIDE DOCKER CONTAINER.
Wraps the ROS 2 environment to make it compatible with stable-baselines3.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Tuple, Dict, Any, Optional
import rclpy
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv


class AckermannGymEnv(gym.Env):
    """Gymnasium wrapper for AckermannCityEnv.
    
    This wrapper makes the ROS 2 environment compatible with stable-baselines3
    by providing a standard Gymnasium interface.
    """
    
    def __init__(self, node_name: str = 'ackermann_drl_env'):
        """Initialize the Gymnasium wrapper.
        
        Args:
            node_name: Name for the ROS 2 node
        """
        super().__init__()
        
        # Initialize ROS 2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create the ROS 2 environment
        self.env = AckermannCityEnv(node_name=node_name)
        
        # Define action space: [linear_velocity, angular_velocity]
        # Linear velocity: -2.0 to 2.0 m/s (forward and reverse)
        # Angular velocity: -1.0 to 1.0 rad/s (steering)
        self.action_space = spaces.Box(
            low=np.array([-2.0, -1.0], dtype=np.float32),
            high=np.array([2.0, 1.0], dtype=np.float32),
            shape=(2,),
            dtype=np.float32
        )
        
        # Define observation space: 187 elements
        # - 180 LiDAR samples (0-50m)
        # - 1 velocity (0-10 m/s)
        # - 1 steering (-5 to 5 rad/s)
        # - 3 goal deltas (Δx, Δy, Δθ)
        # - 1 battery level (0-1)
        # - 1 road distance (0-100m)
        self.observation_space = spaces.Box(
            low=np.array([0.0] * 180 + [-10.0, -5.0, -100.0, -100.0, -np.pi, 0.0, 0.0], dtype=np.float32),
            high=np.array([50.0] * 180 + [10.0, 5.0, 100.0, 100.0, np.pi, 1.0, 100.0], dtype=np.float32),
            shape=(187,),
            dtype=np.float32
        )
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset the environment.
        
        Args:
            seed: Random seed (not used, but required by Gymnasium)
            options: Additional options (not used)
            
        Returns:
            Tuple of (observation, info)
        """
        obs = self.env.reset()
        info = {}
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step in the environment.
        
        Args:
            action: Action array [linear_velocity, angular_velocity]
            
        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        obs, reward, terminated, truncated, info = self.env.step(action)
        return obs, reward, terminated, truncated, info
    
    def close(self):
        """Close the environment."""
        try:
            self.env.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
    
    def render(self, mode: str = 'human'):
        """Render the environment (not implemented for headless training)."""
        pass

