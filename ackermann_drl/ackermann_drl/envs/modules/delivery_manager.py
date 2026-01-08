import numpy as np
from typing import List, Tuple


class DeliveryManager:
    """Manages delivery missions, goals, and deadlines."""
    
    def __init__(self, delivery_points: List[dict]):
        self.delivery_points = delivery_points
        self.current_goal_idx = 0
        self.default_deadline = 120.0
        self.current_load_weight = 0.0 
        self.has_active_mission = False
        self.min_load_weight = 5.0
        self.max_load_weight = 50.0
        self.randomize_start = True
        self.elapsed_time = 0.0

    def reset(self):
        if not self.delivery_points:
            self.current_goal_idx = 0
        elif self.randomize_start:
            self.current_goal_idx = int(np.random.randint(0, len(self.delivery_points)))
        else:
            self.current_goal_idx = 0
        self.has_active_mission = False
        self.elapsed_time = 0.0
        self.start_new_mission()

    def start_new_mission(self):
        self.has_active_mission = True
        self.elapsed_time = 0.0
        # Load weight simulation (simulating a package)
        self.current_load_weight = float(np.random.uniform(self.min_load_weight, self.max_load_weight))

    def get_current_goal(self):
        point_data = self.delivery_points[self.current_goal_idx]
        pos = point_data.get('position', {})
        return (pos.get('east', 0.0), pos.get('north', 0.0), pos.get('up', 0.0))

    def get_current_deadline(self):
        point_data = self.delivery_points[self.current_goal_idx]
        return point_data.get('deadline_seconds', self.default_deadline)

    def check_goal_reached(self, robot_pos, threshold=2.0):
        gx, gy, _ = self.get_current_goal()
        dist = np.sqrt((gx - robot_pos[0])**2 + (gy - robot_pos[1])**2)
        return dist <= threshold

    def advance_to_next_goal(self):
        if not self.delivery_points:
            return
        self.current_goal_idx = (self.current_goal_idx + 1) % len(self.delivery_points)
        self.start_new_mission()

    def advance_time(self, dt: float):
        """Advance mission clock by dt seconds (simulation time)."""
        if dt <= 0.0:
            return
        self.elapsed_time += dt

    def get_mission_status(self):
        deadline = self.get_current_deadline()
        
        return {
            'elapsed': self.elapsed_time,
            'deadline': deadline,
            'remaining': max(0.0, deadline - self.elapsed_time),
            'is_late': self.elapsed_time > deadline,
            'load_weight': self.current_load_weight
        }

    def get_load_range(self) -> Tuple[float, float]:
        return self.min_load_weight, self.max_load_weight
