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
        # Pass full point objects to DeliveryManager, not just positions
        self.delivery = DeliveryManager(dp.get_all_points())
        self.navigation = NavigationSystem()
        self.reward_system = RewardSystem()
        self.battery = BatteryModel(vehicle_weight=1000.0)
        self.load_min, self.load_max = self.delivery.get_load_range()
        self.obs_dim = 190
        self.max_episode_steps = 2000
        self.deadline_grace = 0.0
        self.last_mission_status = None
        
        self.action_space = spaces.Box(low=np.array([-1.0, -0.5]), high=np.array([3.0, 0.5]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.obs_dim,), dtype=np.float32)
        
        # Vehicle dimensions (approximate for collision check)
        self.vehicle_width = 1.0  # meters
        self.vehicle_half_width = self.vehicle_width / 2.0
        
        self.collision_threshold = 0.8
        self.goal_threshold = 2.0
        self.safe_distance = 2.0  # threshold for obstacle proximity
        self.steps = 0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.prev_vel = 0.0  # previous velocity for acceleration calculation
        self.dt = 0.1  # seconds - time step for acceleration
        self.offroad_steps = 0
        self.offroad_patience = 5  # steps tolerated off-road before aborting
        self.offroad_buffer = 0.1  # meters - treat car as outside slightly before curb

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        self.delivery.reset()
        self.battery.reset()
        self.prev_vel = 0.0
        self.last_mission_status = self.delivery.get_mission_status()
        
        self.reset_pub.publish(Bool(data=True))
        self.ros.reset_simulation()
        time.sleep(0.5)
        self._wait_for_sensors()
        self.offroad_steps = 0
        
        pos = self._get_position()
        goal = self.delivery.get_current_goal()
        if not self.navigation.plan_path(pos[:2], goal[:2]):
            raise RuntimeError(f"[AckermannCityEnv] Failed to plan path from {pos[:2]} to {goal[:2]}")
        initial_dist = np.linalg.norm(np.array(pos[:2]) - np.array(goal[:2]))
        self.reward_system.reset(initial_dist)
        
        info = {
            'road_dist': self.navigation.get_road_distance(pos[0], pos[1]),
            'battery': self.battery.get_battery_level(),
            'load': self.last_mission_status['load_weight'],
            'is_collision': False,
            'is_collision_lidar': False,
            'is_collision_offroad': False,
            'min_lidar_distance': 10.0,
            'velocity': 0.0,
            'pos_x': pos[0],
            'pos_y': pos[1],
            'distance_to_goal': initial_dist,
            'mission_elapsed': self.last_mission_status['elapsed'],
            'mission_deadline': self.last_mission_status['deadline'],
            'mission_remaining': self.last_mission_status['remaining'],
            'deadline_exceeded': False,
            'goal_success': False
        }
        
        return self._build_obs(), info

    def step(self, action):
        """Execute one step: apply action, get sensors, compute reward."""
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
        
        # Calculate physical acceleration (m/s^2)
        if not np.isfinite(vel):
            vel = 0.0
        if not np.isfinite(self.prev_vel):
            self.prev_vel = 0.0
        
        acceleration = (vel - self.prev_vel) / self.dt
        acceleration = np.clip(acceleration, -20.0, 20.0)  # clamp to ±20 m/s^2
        self.prev_vel = vel
        
        self.delivery.advance_time(self.dt)
        mission_status = self.delivery.get_mission_status()
        self.last_mission_status = mission_status
        load = mission_status['load_weight']
        self.battery.set_vehicle_weight(1000.0 + load)
        self.battery.update(np.array([pos[0], pos[1]]), vel)
        
        lidar_collision, min_lidar = self._check_lidar(scan)
        road_dist_center, road_width, road_dist_edge, is_offroad = self.navigation.get_road_info(pos[0], pos[1])
        
        # User requirement: penalize only if outside road area.
        # We consider "offroad collision" if the vehicle body is outside the road (hitting the curb).
        # road_dist_edge is: dist_from_center - (road_width / 2.0)
        # Positive means center is outside. Negative means center is inside.
        # Collision occurs if: center is outside OR (center is inside but side of car hits edge)
        # Side of car hits edge if: dist_from_center + car_half_width > road_width / 2.0
        # This is equivalent to: dist_edge > -self.vehicle_half_width
        
        # However, we want to be slightly lenient to avoid false positives on the exact boundary,
        # but strict enough to catch the curb.
        # If the car is hitting the banqueta (curb), it's likely at the edge.
        offroad_margin = -self.vehicle_half_width + self.offroad_buffer
        offroad_collision = road_dist_edge > offroad_margin
        if road_dist_edge > 0.0:
            self.offroad_steps += 1
        else:
            self.offroad_steps = 0
        if self.offroad_steps >= self.offroad_patience:
            offroad_collision = True
        
        collision = lidar_collision or offroad_collision
        
        # Calculate obstacle proximity (LiDAR only now, road edge handled by penalty_offroad logic below)
        # Considers both physical obstacles (LiDAR)
        if not np.isfinite(min_lidar) or min_lidar < 0:
            min_lidar = self.safe_distance
        
        # LiDAR proximity: normalized [0, 1] where 1 = very close
        if min_lidar < self.safe_distance:
            obstacle_proximity = (self.safe_distance - min_lidar) / self.safe_distance
        else:
            obstacle_proximity = 0.0
        obstacle_proximity = np.clip(obstacle_proximity, 0.0, 1.0)
        
        # Road edge proximity / Off-road penalty logic
        # We want to penalize when the car gets close to the edge (inside the road)
        # road_dist_edge is negative inside the road. e.g. -2.0 means 2m inside. -0.1 means 0.1m inside.
        # We want penalty to be high when road_dist_edge is close to -vehicle_half_width (from negative side).
        # i.e. when we are about to hit the curb.
        
        dist_from_edge_contact = (-road_dist_edge) - self.vehicle_half_width
        # If dist_from_edge_contact is positive, we are safely inside.
        # If it is close to 0, we are close to collision.
        
        edge_warning_dist = 0.5
        road_violation_dist = 0.0
        
        if not offroad_collision:
            # Inside safe zone (but maybe close to edge)
            if dist_from_edge_contact < edge_warning_dist:
                # We map the remaining distance to a "violation distance" for the reward system
                # 0.5m buffer -> 0.0 violation
                # 0.0m buffer -> 1.0 violation (approx)
                road_violation_dist = (edge_warning_dist - dist_from_edge_contact)
        else:
            # Outside safe zone (collision)
            road_violation_dist = 1.0 + road_dist_edge # Just a positive value to ensure penalty
            
        goal_reached = self.delivery.check_goal_reached(pos[:2], self.goal_threshold)
        
        target, cte = self.navigation.get_local_target(pos[:2])
        goal_pos = self.delivery.get_current_goal()
        dist_to_goal = np.linalg.norm(np.array(pos[:2]) - np.array(goal_pos[:2]))
        
        reward, info = self.reward_system.compute_reward(
            current_dist_to_goal=dist_to_goal,
            is_collision=collision,
            road_dist=road_violation_dist,  # Passing violation distance (proximity to edge)
            cross_track_error=cte,
            battery_consumed=self.battery.last_energy_drop,
            battery_level=self.battery.get_battery_level(),
            mission_status=mission_status,
            current_vel=(self.current_linear_vel, self.current_angular_vel),
            goal_reached=goal_reached,
            acceleration=acceleration,
            obstacle_proximity=obstacle_proximity
        )
        
        deadline_violation = mission_status['elapsed'] > (mission_status['deadline'] + self.deadline_grace)
        terminated = collision or goal_reached
        truncated = self.battery.is_depleted() or self.steps > self.max_episode_steps or deadline_violation
        
        info.update({
            'road_dist': road_dist_edge,
            'battery': self.battery.get_battery_level(),
            'load': load,
            'is_collision': collision,
            'is_collision_lidar': lidar_collision,
            'is_collision_offroad': offroad_collision,
            'min_lidar_distance': min_lidar,
            'velocity': vel,
            'pos_x': pos[0],
            'pos_y': pos[1],
            'distance_to_goal': dist_to_goal,
            'mission_elapsed': mission_status['elapsed'],
            'mission_deadline': mission_status['deadline'],
            'mission_remaining': mission_status['remaining'],
            'deadline_exceeded': deadline_violation,
            'goal_success': goal_reached
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
        """Build observation vector: [LiDAR(180), velocity(2), target(3), battery, CTE, time_left, late_flag, load]."""
        obs = np.zeros(self.obs_dim, dtype=np.float32)
        
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
        
        mission = self.last_mission_status or {
            'remaining': 0.0,
            'deadline': 1.0,
            'is_late': False,
            'load_weight': self.load_min
        }
        deadline = max(1e-3, mission.get('deadline', 1.0))
        remaining_ratio = np.clip(mission.get('remaining', 0.0) / deadline, 0.0, 1.0)
        load_ratio = 0.0
        load_span = max(1e-3, self.load_max - self.load_min)
        load_ratio = np.clip((mission.get('load_weight', self.load_min) - self.load_min) / load_span, 0.0, 1.0)
        obs[187] = remaining_ratio
        obs[188] = 1.0 if mission.get('is_late', False) else 0.0
        obs[189] = load_ratio
        
        return obs

    def _to_relative(self, target):
        """Convert target position from world frame to robot frame."""
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
