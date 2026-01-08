"""Gymnasium wrapper for AckermannCityEnv to make it compatible with stable-baselines3."""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
from typing import Tuple, Dict, Optional
import rclpy
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv


class AckermannGymEnv(gym.Env):
    """Gymnasium wrapper for AckermannCityEnv compatible with stable-baselines3."""
    
    def __init__(self):
        super().__init__()
        self.env = AckermannCityEnv()
        
        self.action_space = spaces.Box(
            low=np.array([-1.0, -0.5], dtype=np.float32),
            high=np.array([3.0, 0.5], dtype=np.float32),
            shape=(2,),
            dtype=np.float32
        )
        
        low_bounds = [0.0] * 180 + [-10.0, -5.0, -100.0, -100.0, -np.pi, 0.0, 0.0, 0.0, 0.0, 0.0]
        high_bounds = [50.0] * 180 + [10.0, 5.0, 100.0, 100.0, np.pi, 1.0, 100.0, 1.0, 1.0, 1.0]
        self.observation_space = spaces.Box(
            low=np.array(low_bounds, dtype=np.float32),
            high=np.array(high_bounds, dtype=np.float32),
            shape=(190,),
            dtype=np.float32
        )
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset the environment."""
        return self.env.reset(seed=seed, options=options)
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step: [linear_velocity, angular_velocity] -> (obs, reward, done, truncated, info)."""
        obs, reward, terminated, truncated, info = self.env.step(action)
        return obs, reward, terminated, truncated, info
    
    def close(self):
        """Stop vehicle and close environment."""
        try:
            self.env.ros.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.2)
            self.env.close()
        except Exception:
            pass
    
    def render(self, mode: str = 'human'):
        """Render not implemented (headless training)."""
        pass
