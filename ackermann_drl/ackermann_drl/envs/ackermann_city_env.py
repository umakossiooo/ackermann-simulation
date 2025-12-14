"""Ackermann City Environment for DRL training with A* Path Planning.

This environment provides the interface between ROS 2 and the DRL agent
for training in the Bari city simulation. It integrates A* path planning
to guide the agent along roads.
"""

from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from typing import Tuple, Dict, Any, Optional, List
import time
import math
import sys
import os
from pathlib import Path

# --- A* IMPORT SETUP ---
# Add the path_planning directory to sys.path to import A* scripts
# We calculate the path relative to this file to be robust
current_file_path = Path(__file__).resolve()
# Go up to ackermann-vehicle-gzsim-ros2 root
# .../ackermann_drl/ackermann_drl/envs/ackermann_city_env.py -> .../ackermann-vehicle-gzsim-ros2
repo_root = current_file_path.parent.parent.parent.parent
planning_dir = repo_root / 'saye_bringup' / 'scripts' / 'path_planning'

if str(planning_dir) not in sys.path:
    sys.path.append(str(planning_dir))

try:
    from astar_path_planner import AStarPlanner
    from path_planner_utils import find_maps_directory
    print(f"[ENV] Successfully imported AStarPlanner from {planning_dir}", flush=True)
except ImportError as e:
    print(f"[ENV] WARNING: Could not import AStarPlanner: {e}", flush=True)
    print(f"[ENV] Checked path: {planning_dir}", flush=True)
    AStarPlanner = None  # Fallback handles this

from ackermann_drl.utils.battery_model import BatteryModel
from ackermann_drl.utils.delivery_points import DeliveryPoints
from ackermann_drl.utils.roads_geometry import RoadsGeometry


def safe_log(logger_func, message: str):
    """Safely log a message, handling invalid ROS context."""
    try:
        if rclpy.ok():
            logger_func(message)
    except Exception:
        pass


class AckermannCityEnv(Node):
    """ROS 2 environment for Ackermann vehicle DRL training with A*."""
    
    def __init__(self, node_name: str = 'ackermann_drl_env'):
        # Unique node name
        if node_name == 'ackermann_drl_env':
            node_name = f'ackermann_drl_env_{int(time.time() * 1000)}_{os.getpid()}'
        super().__init__(node_name)
        print(f"[INIT] Created ROS node: {node_name}", flush=True)
        
        # --- ROS SUBSCRIPTIONS & PUBLISHERS ---
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        # Create explicit Best Effort QoS profile for Gazebo sensors
        # This is more robust than qos_profile_sensor_data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, sensor_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Wait for connections
        self._wait_for_publishers()
        
        # --- STATE VARIABLES ---
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.scan_received = False
        self.odom_received = False
        self.last_scan_time = None
        self.last_odom_time = None
        self.scan_count = 0
        self.odom_count = 0
        
        # --- UTILS & MODELS ---
        self.battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01, vehicle_weight=1000.0)
        self.delivery_points = DeliveryPoints()
        self.current_goal: Optional[Dict] = None
        
        # Load Roads Geometry (for off-road detection safety net)
        try:
            self.roads_geometry = RoadsGeometry()
        except Exception as e:
            self.get_logger().error(f"Failed to load roads geometry: {e}")
            self.roads_geometry = None
        
        # --- A* PLANNER INITIALIZATION ---
        self.planner = None
        self.current_path: List[Tuple[float, float]] = []
        self.current_path_index = 0
        self.lookahead_distance = 5.0  # Look 5 meters ahead on the path
        self._init_astar_planner()
        
        # --- CONFIGURATION ---
        self.lidar_downsample_factor = 4
        self.lidar_downsampled_size = 720 // self.lidar_downsample_factor
        
        # Reward Weights
        self.reward_progress_scale = 1.0
        self.reward_goal_reached = 50.0
        self.reward_offroad_penalty = -0.05
        self.reward_collision_penalty = -20.0
        self.reward_battery_penalty_scale = -0.5
        self.reward_time_penalty = -0.01
        self.reward_path_deviation_penalty = -1.0  # Strong penalty for straying from A* path
        
        # Thresholds
        self.goal_reached_threshold = 2.0
        self.collision_threshold = 0.8 
        # Set strict off-road threshold (e.g. 1.5m) to terminate episode if driving fully on sidewalk
        # This acts as a "Virtual Wall" for the DRL agent
        self.offroad_collision_threshold = 1.5 
        self.path_deviation_threshold = 0.5 # Meters allowed from path center before penalty kicks in
        
        # Tracking variables
        self.prev_distance_to_goal = None
        self.prev_distance_along_path = None 
        self.prev_position = None
        self.current_step_velocity = 0.0
        self.prev_angular_vel = 0.0
        self.battery_consumed_this_step = 0.0
        
        # Delivery tracking
        self.delivery_start_time = None
        self.delivery_deadline = None
        self.delivery_elapsed_time = 0.0
        self.delivery_on_time = False
        self.reward_delivery_on_time = 30.0
        self.reward_delivery_late_penalty = -10.0
        
        # Episode stats
        self.step_count = 0
        self.episode_count = 0
        self.episode_cumulative_reward = 0.0
        self.training_cumulative_reward = 0.0
        self.battery_consumed_this_step = 0.0

        self.cumulative_reward_components = self._init_reward_component_totals()
        
        # --- EXECUTOR ---
        self.executor = None
        if rclpy.ok():
            try:
                self.executor = SingleThreadedExecutor(context=rclpy.get_default_context())
                self.executor.add_node(self)
            except Exception:
                self.executor = None

    def _init_astar_planner(self):
        """Initialize the A* planner if available."""
        if AStarPlanner is None:
            return

        try:
            # Try to find maps directory
            maps_dir = None
            try:
                maps_dir = find_maps_directory()
            except:
                pass
                
            if not maps_dir:
                 # Fallback paths
                candidates = [
                    Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/map2gazebo/maps'),
                    Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/map2gazebo_maps')
                ]
                for p in candidates:
                    if (p / 'edges.json').exists():
                        maps_dir = p
                        break
            
            if maps_dir:
                edges_file = maps_dir / 'edges.json'
                map_file = maps_dir / 'map.json'
                polygons_file = maps_dir / 'road_polygons_merged.json'
                
                if polygons_file.exists():
                    self.planner = AStarPlanner(str(edges_file), str(map_file), str(polygons_file))
                else:
                    self.planner = AStarPlanner(str(edges_file), str(map_file))
                
                self.get_logger().info(f"A* Planner initialized using maps at {maps_dir}")
            else:
                self.get_logger().error("Could not find map files for A* Planner!")
        except Exception as e:
            self.get_logger().error(f"Error initializing A* Planner: {e}")

    def _wait_for_publishers(self):
        """Wait for publishers to connect."""
        time.sleep(1.0)
        # We don't block indefinitely here to avoid hanging if bridge is slow, 
        # but we wait a bit.
        for _ in range(20):
            if self.cmd_vel_pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)

    def _init_reward_component_totals(self) -> Dict[str, float]:
        return {
            'reward_progress': 0.0,
            'reward_goal': 0.0,
            'reward_delivery_on_time': 0.0,
            'reward_battery_conservation': 0.0,
            'reward_efficiency': 0.0,
            'penalty_delivery_late': 0.0,
            'penalty_offroad': 0.0,
            'penalty_collision': 0.0,
            'penalty_high_speed': 0.0,
            'penalty_aggressive_change': 0.0,
            'penalty_time': 0.0,
            'penalty_path_deviation': 0.0, 
        }

    # --- CALLBACKS ---
    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_received = True
        self.last_scan_time = time.time()
        self.scan_count += 1
    
    def _odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.odom_received = True
        self.last_odom_time = time.time()
        self.odom_count += 1
    
    def spin_once(self, timeout_sec: float = 0.1):
        try:
            # Spin multiple times to ensure we catch up on messages
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=timeout_sec)
        except Exception:
            pass

    # --- MAIN GYM INTERFACE ---
    
    def reset(self) -> np.ndarray:
        """Reset environment, select new goal, calculate A* path."""
        self.episode_count += 1
        self.step_count = 0
        self.episode_cumulative_reward = 0.0
        
        print(f"\n{'='*80}\nEPISODE {self.episode_count} STARTED\n{'='*80}")
        
        self.battery.reset()
        self.scan_received = False
        self.odom_received = False
        self.prev_distance_to_goal = None
        self.current_step_velocity = 0.0
        
        # Reset Delivery
        self.delivery_start_time = time.time()
        self.delivery_elapsed_time = 0.0
        self.delivery_on_time = False
        
        # Select Goal
        try:
            self.current_goal = self.delivery_points.get_next_point(None)
            self.delivery_deadline = self.current_goal.get('deadline_seconds', 120.0)
        except:
            self.current_goal = None
            
        # Wait for sensors - INCREASED TIMEOUT
        # Try for 60 seconds (1200 * 0.05s) to allow Gazebo to fully initialize
        print("[INFO] Waiting for sensors...", flush=True)
        
        # Use EXPLICIT QoS profile matching the publisher (ros_gz_bridge)
        # Publisher is RELIABLE + VOLATILE (verified via ros2 topic info)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        for i in range(1200):
            self.spin_once(timeout_sec=0.05)
            if self.scan_received and self.odom_received:
                print("[INFO] Sensors connected successfully!", flush=True)
                break
        
            # Refresh subscription every 10 seconds (200 ticks) if still waiting
            # We recreate the subscription to clear any middleware 'stuck' states, but keep the correct QoS.
            if i > 0 and i % 200 == 0 and not self.scan_received:
                print(f"[WARN] Still waiting for sensors ({i*0.05:.0f}s)... Refreshing /scan subscription...", flush=True)
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_callback, sensor_qos)
                
                # If we have Odom but no Scan after 20 seconds, try dynamic search EARLY
                if i >= 400 and self.odom_received:
                     print("[WARN] Odom present but Scan missing > 20s. Triggering early topic search...", flush=True)
                     break # Break loop to trigger the dynamic search block below
        
        # DYNAMIC TOPIC RECOVERY
        if not self.scan_received and self.odom_received:
            safe_log(self.get_logger().warn, "Default /scan topic failed. Searching for alternative LiDAR topics...")
            topic_names_and_types = self.get_topic_names_and_types()
            
            # PERMANENT DEBUG: List all topics when sensor setup fails
            print("\n[DIAGNOSTIC] ALL AVAILABLE TOPICS:", flush=True)
            for name, types in topic_names_and_types:
                print(f"  - {name}: {types}", flush=True)
            print("------------------------------------------", flush=True)
            
            for name, types in topic_names_and_types:
                if 'sensor_msgs/msg/LaserScan' in types:
                    safe_log(self.get_logger().info, f"Found alternative LiDAR topic: {name}")
                    # Switch subscription
                    self.destroy_subscription(self.scan_sub)
                    self.scan_sub = self.create_subscription(LaserScan, name, self._scan_callback, sensor_qos)
                    # Wait a bit more for this new topic
                    for _ in range(50):
                        self.spin_once(timeout_sec=0.05)
                        if self.scan_received:
                            safe_log(self.get_logger().info, "Successfully connected to alternative LiDAR topic!")
                            break
                    if self.scan_received:
                        break
        
        if not self.scan_received or not self.odom_received:
             safe_log(self.get_logger().error, "[CRITICAL] Reset timed out waiting for sensors! Check if Gazebo/ROS bridge is running.")
             safe_log(self.get_logger().error, f"Status: scan={self.scan_received}, odom={self.odom_received}")
             
             # FALLBACK: If we have odom but no scan, proceed anyway to allow debugging/movement
             if self.odom_received and not self.scan_received:
                 safe_log(self.get_logger().warn, "Proceeding with Odometry only (BLIND MODE). LiDAR not received.")
        else:
                 # If we don't even have odometry, we can't navigate A*, so we might as well wait or fail
                 pass

        # --- A* PATH CALCULATION ---
        self.current_path = []
        self.current_path_index = 0
        
        if self.current_goal is not None and self.latest_odom is not None and self.planner:
            try:
                pos = self.latest_odom.pose.pose.position
                start_node = (pos.x, pos.y)
                
                goal_pos = self.delivery_points.get_point_position(self.current_goal)
                goal_node = (goal_pos[0], goal_pos[1])
                
                safe_log(self.get_logger().info, f"Calculating A* path from ({pos.x:.1f}, {pos.y:.1f}) to ({goal_pos[0]:.1f}, {goal_pos[1]:.1f})...")
                
                # A* search
                path, _ = self.planner.search(start_node, goal_node)
                
                if path:
                    self.current_path = path
                    safe_log(self.get_logger().info, f"Path found! Length: {len(path)} nodes")
                else:
                    safe_log(self.get_logger().warn, "A* failed to find a path! Will use euclidean guidance.")
            except Exception as e:
                safe_log(self.get_logger().error, f"Path planning error: {e}")

        # Initialize tracking vars
        if self.current_goal is not None:
             self.prev_distance_to_goal = self.get_distance_to_goal()
             
        # Stop car
        self._publish_cmd(0.0, 0.0)
        
        return self.get_observation()

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # Execute Action
        linear_vel = float(action[0])
        angular_vel = float(action[1])
        self._publish_cmd(linear_vel, angular_vel)
        
        self.step_count += 1
        
        # Wait for physics
        self.spin_once(timeout_sec=0.1)
        
        # Update Velocity tracking
        self._update_velocity_tracking()
        
        # Update Battery
        self.battery_consumed_this_step = 0.0
        if self.latest_odom:
             old_level = self.battery.get_battery_level()
             pos = self.latest_odom.pose.pose.position
             self.battery.update(np.array([pos.x, pos.y, pos.z]), self.current_step_velocity, dt=0.1)
             self.battery_consumed_this_step = max(0.0, old_level - self.battery.get_battery_level())
        
        # Compute Reward (pass current angular vel for smoothness calc)
        reward, reward_info = self.compute_reward(angular_vel)
        self.episode_cumulative_reward += reward
        self.training_cumulative_reward += reward
        
        # Update previous action for next step
        self.prev_angular_vel = angular_vel
        
        # Check Termination
        terminated, truncated = self.check_termination()
        
        # Next Goal Logic (if current reached)
        if self.is_goal_reached() and self.current_goal:
             self._handle_goal_reached()
        
        observation = self.get_observation()
        
        pos_x = self.latest_odom.pose.pose.position.x if self.latest_odom else 0.0
        pos_y = self.latest_odom.pose.pose.position.y if self.latest_odom else 0.0

        info = {
            'pos_x': pos_x,
            'pos_y': pos_y,
            'velocity': self.current_step_velocity,
            'distance_to_goal': self.get_distance_to_goal(),
            'battery_level': self.battery.get_battery_level(),
            'has_scan': self.scan_received,
            'has_odom': self.odom_received,
            'scan_count': self.scan_count,
            'odom_count': self.odom_count,
            'has_goal': self.current_goal is not None,
            'road_distance': self.get_road_distance(),
            'is_collision': self.is_collision(),
            **reward_info
        }
        
        # Add freshness check (if data is older than 1.0s)
        now = time.time()
        info['scan_is_fresh'] = (self.last_scan_time is not None) and (now - self.last_scan_time < 1.0)
        info['odom_is_fresh'] = (self.last_odom_time is not None) and (now - self.last_odom_time < 1.0)
        
        # Add min lidar for debug
        if self.latest_scan:
            ranges = np.array(self.latest_scan.ranges)
            valid = ranges[np.isfinite(ranges)]
            if len(valid) > 0:
                info['min_lidar_distance'] = float(np.min(valid))
            else:
                info['min_lidar_distance'] = -1.0
        else:
            info['min_lidar_distance'] = -1.0
        
        return observation, reward, terminated, truncated, info
    
    # --- PATH PLANNING HELPERS ---
    
    def get_local_target(self) -> Tuple[Optional[Tuple[float, float]], float]:
        """
        Get the local target point on the path and cross-track error.
            
        Returns:
            (target_point, cross_track_error)
            target_point: (x, y) of the point ~lookahead_distance ahead on path
            cross_track_error: distance from car to nearest point on path
        """
        if not self.current_path or self.latest_odom is None:
            return None, 0.0
            
        pos = self.latest_odom.pose.pose.position
        robot_xy = np.array([pos.x, pos.y])
        
        # 1. Find nearest point on path
        min_dist = float('inf')
        closest_idx = self.current_path_index
        
        # Optimization: only search a window around last known position
        search_start = max(0, self.current_path_index - 5)
        search_end = min(len(self.current_path), self.current_path_index + 50)
        
        for i in range(search_start, search_end):
            p = np.array(self.current_path[i])
            dist = np.linalg.norm(p - robot_xy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        self.current_path_index = closest_idx
        cross_track_error = min_dist
        
        # 2. Find target point 'lookahead_distance' ahead of closest point
        target_idx = closest_idx
        accumulated_dist = 0.0
        
        for i in range(closest_idx, len(self.current_path) - 1):
            p1 = np.array(self.current_path[i])
            p2 = np.array(self.current_path[i+1])
            segment_len = np.linalg.norm(p2 - p1)
            accumulated_dist += segment_len
            
            if accumulated_dist >= self.lookahead_distance:
                target_idx = i + 1
                break
        else:
            # Reached end of path
            target_idx = len(self.current_path) - 1
            
        return self.current_path[target_idx], cross_track_error

    # --- OBSERVATION & REWARD ---
    
    def get_observation(self) -> np.ndarray:
        """
        Obs Structure (187):
        [0:180]   LiDAR
        [180]     Velocity
        [181]     Steering
        [182:185] DELTAS TO LOCAL TARGET (Rabbit) - Replaces Global Goal Deltas
        [185]     Battery
        [186]     Cross Track Error (Path Deviation)
        """
        obs = np.zeros(187, dtype=np.float32)
        idx = 0
        
        # 1. LiDAR (0-179)
        if self.latest_scan:
            ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
            # Clip and normalize
            ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
            # Downsample
            if len(ranges) >= self.lidar_downsampled_size * self.lidar_downsample_factor:
                 ds = ranges[::self.lidar_downsample_factor][:self.lidar_downsampled_size]
                 obs[idx:idx+len(ds)] = ds
        idx += self.lidar_downsampled_size
        
        # 2. Vehicle State
        obs[idx] = self.current_step_velocity
        idx += 1
        twist = self.latest_odom.twist.twist if self.latest_odom else Twist()
        obs[idx] = twist.angular.z
        idx += 1
        
        # 3. Target Deltas (The "Rabbit")
        # If we have a path, use the local target. If not, use global goal.
        local_target, cte = self.get_local_target()
        
        if local_target:
            dx, dy, dtheta = self.compute_relative_to_point(local_target)
        else:
            dx, dy, dtheta = self.compute_goal_deltas() # Fallback
            
        obs[idx] = dx
        idx += 1
        obs[idx] = dy
        idx += 1
        obs[idx] = dtheta
        idx += 1
        
        # 4. Battery
        obs[idx] = self.battery.get_battery_level()
        idx += 1
        
        # 5. Path Deviation / Road Distance
        # Used by agent to know if it's straying
        if self.current_path:
             obs[idx] = cte
        else:
             obs[idx] = self.get_road_distance()
        
        return obs
    
    def compute_reward(self, current_angular_vel: float = 0.0) -> Tuple[float, Dict[str, float]]:
        reward = 0.0
        info = {}
        
        if not self.latest_odom or not self.current_goal:
            return 0.0, info

        # --- UMA KOSSIO SPECIFIC REWARDS ---
        
        # 1. Energy Efficiency (Minimize Consumption)
        # Penalize the actual energy amount used this step
        # Scale: -100.0 means 1% battery usage costs -1.0 reward
        energy_penalty = -100.0 * self.battery_consumed_this_step
        reward += energy_penalty
        info['reward_efficiency'] = energy_penalty

        # 2. Smoothness/Stability (Minimize Aggressive Steering)
        # Penalize rapid changes in steering (jerk) to prevent instability
        steering_jerk = abs(current_angular_vel - self.prev_angular_vel)
        if steering_jerk > 0.2: # Threshold for "aggressive"
             smoothness_penalty = -0.5 * steering_jerk
             reward += smoothness_penalty
             info['penalty_aggressive_change'] = smoothness_penalty

        # --- STANDARD NAVIGATION REWARDS ---

        # 3. Progress Reward
        # We stick to global progress but heavily penalized by path deviation
        current_dist = self.get_distance_to_goal()
        if self.prev_distance_to_goal is not None:
            progress = self.prev_distance_to_goal - current_dist
            reward += self.reward_progress_scale * progress
            info['reward_progress'] = progress
        self.prev_distance_to_goal = current_dist
        
        # 2. Path Deviation Penalty (The "Stay on Road" logic)
        local_target, cross_track_error = self.get_local_target()
        if local_target:
            # If we deviate more than threshold from the A* path, apply penalty
            if cross_track_error > self.path_deviation_threshold:
                # Quadratic penalty for smooth but firm correction
                # e.g. 1m error -> (1)^1.5 * -1.0 = -1.0
                # e.g. 2m error -> (2)^1.5 * -1.0 = -2.8
                dev_penalty = self.reward_path_deviation_penalty * (cross_track_error ** 1.5)
                reward += dev_penalty
                info['penalty_path_deviation'] = dev_penalty
        
        # 3. Off-road Penalty (Safety net)
        # Even if on path, if we are somehow off road mesh (e.g. cutting corner too much)
        road_dist = self.get_road_distance()
        if road_dist > 0.0:
            reward += self.reward_offroad_penalty * road_dist
            info['penalty_offroad'] = self.reward_offroad_penalty * road_dist
            
        # 4. Collision
        if self.is_collision():
            reward += self.reward_collision_penalty
            info['penalty_collision'] = self.reward_collision_penalty
            
        # 5. Goal & Delivery (Only if NOT crashed)
        elif self.is_goal_reached():
            reward += self.reward_goal_reached
            info['reward_goal'] = self.reward_goal_reached
            
            # Punctuality
            if self.delivery_start_time:
                elapsed = time.time() - self.delivery_start_time
                if self.delivery_deadline and elapsed <= self.delivery_deadline:
                    reward += self.reward_delivery_on_time
                    info['reward_delivery_on_time'] = self.reward_delivery_on_time
                elif self.delivery_deadline:
                    # Scaled down penalty (e.g. -0.5 per second late)
                    late_pen = -0.5 * (elapsed - self.delivery_deadline)
                    reward += late_pen
                    info['penalty_delivery_late'] = late_pen

        # 6. Battery
        batt = self.battery.get_battery_level()
        if batt < 0.2:
             reward += -0.05 # Critical low
             info['reward_battery_conservation'] = -0.05
        elif batt > 0.8:
             reward += 0.01 # Good charge
             info['reward_battery_conservation'] = 0.01
             
        # Time penalty
        reward += self.reward_time_penalty
        info['penalty_time'] = self.reward_time_penalty
        
        # Logging breakdown
        for k, v in info.items():
            self.cumulative_reward_components[k] = self.cumulative_reward_components.get(k, 0.0) + v
            
        return reward, info

    # --- HELPERS ---
    
    def compute_relative_to_point(self, target_point: Tuple[float, float]) -> Tuple[float, float, float]:
        """Compute dx, dy, dtheta to a specific (x,y) point in robot frame."""
        if not self.latest_odom: return 0.0, 0.0, 0.0
        
        pos = self.latest_odom.pose.pose.position
        robot_x, robot_y = pos.x, pos.y
        
        # Robot Yaw
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        dx_world = target_point[0] - robot_x
        dy_world = target_point[1] - robot_y
        
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        
        dx_robot = dx_world * cos_yaw - dy_world * sin_yaw
        dy_robot = dx_world * sin_yaw + dy_world * cos_yaw
        
        target_yaw = math.atan2(dy_world, dx_world)
        dtheta = target_yaw - robot_yaw
        while dtheta > math.pi: dtheta -= 2*math.pi
        while dtheta < -math.pi: dtheta += 2*math.pi
        
        return dx_robot, dy_robot, dtheta

    def compute_goal_deltas(self):
        if not self.current_goal: return 0.0, 0.0, 0.0
        gp = self.delivery_points.get_point_position(self.current_goal)
        return self.compute_relative_to_point((gp[0], gp[1]))

    def get_distance_to_goal(self) -> float:
        if not self.current_goal or not self.latest_odom: return float('inf')
        gp = self.delivery_points.get_point_position(self.current_goal)
        pos = self.latest_odom.pose.pose.position
        return math.sqrt((gp[0]-pos.x)**2 + (gp[1]-pos.y)**2)

    def get_road_distance(self) -> float:
        if self.roads_geometry and self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            dist, _ = self.roads_geometry.distance_to_nearest_road(pos.x, pos.y)
            return dist
        return 0.0

    def is_collision(self) -> bool:
        if self.latest_scan:
            ranges = np.array(self.latest_scan.ranges)
            valid = ranges[np.isfinite(ranges)]
            if len(valid) > 0 and np.min(valid) <= self.collision_threshold:
                return True
        # Offroad collision check - if we go too far off road, treat as collision
        # This prevents the car from driving on sidewalks
        if self.get_road_distance() > self.offroad_collision_threshold:
            return True
        return False

    def is_goal_reached(self) -> bool:
        return self.get_distance_to_goal() <= self.goal_reached_threshold
    
    def check_termination(self) -> Tuple[bool, bool]:
        terminated = False
        truncated = False
        
        if self.is_collision():
            terminated = True
        
        if self.battery.is_depleted():
            truncated = True
        
        return terminated, truncated
    
    def _publish_cmd(self, linear, angular):
        if self.cmd_vel_pub and rclpy.ok():
            try:
                cmd = Twist()
                cmd.linear.x = float(linear)
                cmd.angular.z = float(angular)
                self.cmd_vel_pub.publish(cmd)
            except Exception:
                pass

    def _update_velocity_tracking(self):
        if self.latest_odom:
             t = self.latest_odom.twist.twist
             self.current_step_velocity = math.sqrt(t.linear.x**2 + t.linear.y**2)

    def _handle_goal_reached(self):
        self.current_goal = self.delivery_points.get_next_point(self.current_goal)
        self.delivery_start_time = time.time()
        # Recalculate path to new goal
        if self.planner and self.latest_odom:
             pos = self.latest_odom.pose.pose.position
             gp = self.delivery_points.get_point_position(self.current_goal)
             path, _ = self.planner.search((pos.x, pos.y), (gp[0], gp[1]))
             if path:
                 self.current_path = path
                 self.current_path_index = 0
                 safe_log(self.get_logger().info, f"New goal set! Path length: {len(path)}")
        else:
             self.current_path = []
             safe_log(self.get_logger().warn, "Could not find path to new goal!")

    def stop(self):
        self._publish_cmd(0.0, 0.0)
