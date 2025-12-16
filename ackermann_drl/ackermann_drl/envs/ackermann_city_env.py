import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
import math
import time
from typing import Tuple, Dict
from std_msgs.msg import Bool

from ackermann_drl.envs.modules.ros_interface import RosInterface
from ackermann_drl.envs.modules.reward_system import RewardSystem
from ackermann_drl.envs.modules.navigation import NavigationSystem
from ackermann_drl.envs.modules.delivery_manager import DeliveryManager
from ackermann_drl.utils.battery_model import BatteryModel
from ackermann_drl.utils.delivery_points import DeliveryPoints


class AckermannCityEnv(gym.Env):
    """Gym Environment for Ackermann Vehicle DRL Training with A* path planning."""
    metadata = {'render_modes': ['human']}

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        
        self.ros = RosInterface()
        self.reset_pub = self.ros.create_publisher(Bool, '/reset_simulation', 1)
        
        dp = DeliveryPoints()
        delivery_points = [dp.get_point_position(point) for point in dp.get_all_points()]
        self.delivery = DeliveryManager(delivery_points)
        self.navigation = NavigationSystem()
        self.reward_system = RewardSystem()
        self.battery = BatteryModel(vehicle_weight=1000.0)
        
        self.action_space = spaces.Box(low=np.array([-1.0, -0.5]), high=np.array([3.0, 0.5]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(187,), dtype=np.float32)
        
        self.collision_threshold = 0.8
        self.offroad_threshold = 1.5
        self.goal_threshold = 2.0
        self.steps = 0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        self.delivery.reset()
        self.battery.reset()
        self.reward_system.reset(0.0)
        
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.5)
        self._wait_for_sensors()
        
        pos = self._get_position()
        goal = self.delivery.get_current_goal()
        self.navigation.plan_path(pos[:2], goal[:2])
        initial_dist = np.linalg.norm(np.array(pos[:2]) - np.array(goal[:2]))
        self.reward_system.prev_dist_to_goal = initial_dist
        
        # Build info for reset
        info = {
            'road_dist': self.navigation.get_road_distance(pos[0], pos[1]),
            'battery': self.battery.get_battery_level(),
            'load': self.delivery.get_mission_status()['load_weight'],
            'is_collision': False,
            'is_collision_lidar': False,
            'is_collision_offroad': False,
            'min_lidar_distance': 10.0,
            'velocity': 0.0,
            'pos_x': pos[0],
            'pos_y': pos[1],
            'distance_to_goal': initial_dist
        }
        
        return self._build_obs(), info

    def step(self, action):
        self.steps += 1
        action = np.asarray(action, dtype=np.float32).flatten()
        if len(action) < 2:
            action = np.pad(action, (0, 2 - len(action)), 'constant', constant_values=0.0)
        self.current_linear_vel = float(action[0])
        self.current_angular_vel = float(action[1]) if len(action) > 1 else 0.0
        self.ros.publish_cmd_vel(self.current_linear_vel, self.current_angular_vel)
        
        odom = self.ros.get_odom()
        scan = self.ros.get_scan()
        pos = self._get_position()
        vel = np.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2) if odom else 0.0
        
        load = self.delivery.get_mission_status()['load_weight']
        self.battery.set_vehicle_weight(1000.0 + load)
        self.battery.update(np.array([pos[0], pos[1]]), vel)
        
        lidar_collision, min_lidar = self._check_lidar(scan)
        road_dist = self.navigation.get_road_distance(pos[0], pos[1])
        offroad_collision = road_dist > self.offroad_threshold
        collision = lidar_collision or offroad_collision
        
        goal_reached = self.delivery.check_goal_reached(pos[:2], self.goal_threshold)
        if goal_reached:
            new_goal = self.delivery.get_current_goal()
            self.navigation.plan_path(pos[:2], new_goal[:2])
        
        target, cte = self.navigation.get_local_target(pos[:2])
        goal_pos = self.delivery.get_current_goal()
        dist_to_goal = np.linalg.norm(np.array(pos[:2]) - np.array(goal_pos[:2]))
        
        reward, info = self.reward_system.compute_reward(
            current_dist_to_goal=dist_to_goal,
            is_collision=collision,
            road_dist=road_dist,
            cross_track_error=cte,
            battery_consumed=self.battery.last_energy_drop,
            battery_level=self.battery.get_battery_level(),
            mission_status=self.delivery.get_mission_status(),
            current_vel=(self.current_linear_vel, self.current_angular_vel),
            goal_reached=goal_reached
        )
        
        terminated = collision
        truncated = self.battery.is_depleted() or self.steps > 2000
        
        info.update({
            'road_dist': road_dist, 'battery': self.battery.get_battery_level(), 'load': load,
            'is_collision': terminated, 'is_collision_lidar': lidar_collision,
            'is_collision_offroad': offroad_collision, 'min_lidar_distance': min_lidar,
            'velocity': vel, 'pos_x': pos[0], 'pos_y': pos[1], 'distance_to_goal': dist_to_goal
        })
        
        return self._build_obs(), reward, terminated, truncated, info

    def _check_lidar(self, scan):
        if not scan or len(scan.ranges) == 0:
            return False, 10.0
        valid = [r for r in scan.ranges if np.isfinite(r) and r > 0]
        if not valid:
            return False, 10.0
        min_dist = min(valid)
        return min_dist < self.collision_threshold, min_dist

    def _get_position(self):
        odom = self.ros.get_odom()
        if odom:
            p = odom.pose.pose.position
            return (p.x, p.y, p.z)
        return (0.0, 0.0, 0.0)

    def _wait_for_sensors(self, timeout=5.0):
        start = time.time()
        while (self.ros.get_odom() is None or self.ros.get_scan() is None) and (time.time() - start < timeout):
            time.sleep(0.1)

    def _build_obs(self):
        obs = np.zeros(187, dtype=np.float32)
        
        scan = self.ros.get_scan()
        if scan and len(scan.ranges) > 0:
            ranges = np.array(scan.ranges)
            if len(ranges) >= 180:
                ranges = ranges[::(len(ranges) // 180)][:180]
            else:
                ranges = np.pad(ranges, (0, 180 - len(ranges)), 'constant', constant_values=10.0)
            obs[:180] = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
        
        odom = self.ros.get_odom()
        if odom:
            obs[180] = odom.twist.twist.linear.x
            obs[181] = odom.twist.twist.angular.z
        
        pos = self._get_position()
        target, cte = self.navigation.get_local_target(pos[:2])
        target_point = target if target else self.delivery.get_current_goal()[:2]
        dx, dy, dtheta = self._to_relative(target_point)
        obs[182:185] = [dx, dy, dtheta]
        obs[185] = self.battery.get_battery_level()
        obs[186] = cte
        
        return obs

    def _to_relative(self, target):
        odom = self.ros.get_odom()
        if not odom:
            return 0.0, 0.0, 0.0
        
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        dx_w = target[0] - p.x
        dy_w = target[1] - p.y
        dx_r = dx_w * math.cos(-yaw) - dy_w * math.sin(-yaw)
        dy_r = dx_w * math.sin(-yaw) + dy_w * math.cos(-yaw)
        
        target_yaw = math.atan2(dy_w, dx_w)
        dtheta = target_yaw - yaw
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        
        return dx_r, dy_r, dtheta

    def close(self):
        self.ros.stop()
        super().close()
