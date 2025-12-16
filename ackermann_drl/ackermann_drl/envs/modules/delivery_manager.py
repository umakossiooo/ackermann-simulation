import numpy as np
import time
from typing import List, Tuple


class DeliveryManager:
    """Manages delivery missions, goals, and deadlines."""
    
    def __init__(self, delivery_points: List[Tuple[float, float, float]]):
        self.delivery_points = delivery_points
        self.current_goal_idx = 0
        self.delivery_start_time = None
        self.delivery_deadline = 60.0
        self.current_load_weight = 0.0 
        self.has_active_mission = False

    def reset(self):
        self.current_goal_idx = 0
        self.has_active_mission = False
        self.delivery_start_time = None
        self.start_new_mission()

    def start_new_mission(self):
        self.delivery_start_time = time.time()
        self.has_active_mission = True
        self.current_load_weight = np.random.uniform(0.0, 50.0) 

    def get_current_goal(self):
        return self.delivery_points[self.current_goal_idx]

    def check_goal_reached(self, robot_pos, threshold=2.0):
        gx, gy, _ = self.get_current_goal()
        dist = np.sqrt((gx - robot_pos[0])**2 + (gy - robot_pos[1])**2)
        if dist <= threshold:
            self.current_goal_idx = (self.current_goal_idx + 1) % len(self.delivery_points)
            self.start_new_mission()
            return True
        return False

    def get_mission_status(self):
        if self.delivery_start_time is None:
            elapsed = 0.0
        else:
            elapsed = time.time() - self.delivery_start_time
        return {
            'elapsed': elapsed,
            'deadline': self.delivery_deadline,
            'remaining': max(0.0, self.delivery_deadline - elapsed),
            'is_late': elapsed > self.delivery_deadline,
            'load_weight': self.current_load_weight
        }
