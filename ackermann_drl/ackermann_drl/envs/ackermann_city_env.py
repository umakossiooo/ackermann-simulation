"""Ackermann City Environment for DRL training.

This environment provides the interface between ROS 2 and the DRL agent
for training in the Bari city simulation.

MUST RUN INSIDE DOCKER CONTAINER.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from typing import Tuple, Dict, Any, Optional
import time
import math
from ackermann_drl.utils.battery_model import BatteryModel
from ackermann_drl.utils.delivery_points import DeliveryPoints
from ackermann_drl.utils.roads_geometry import RoadsGeometry


def safe_log(logger_func, message: str):
    """Safely log a message, handling invalid ROS context."""
    try:
        if rclpy.ok():
            logger_func(message)
    except Exception:
        # Silently ignore logging errors when context is invalid
        pass


class AckermannCityEnv(Node):
    """ROS 2 environment for Ackermann vehicle DRL training.
    
    This class implements a minimal Gymnasium-like interface for training DRL agents
    in the Gazebo simulation environment. Designed to run inside Docker container.
    
    Subscribes to:
    - /odom (nav_msgs/msg/Odometry) - Robot odometry
    - /scan (sensor_msgs/msg/LaserScan) - LiDAR scan data
    
    Publishes to:
    - /cmd_vel (geometry_msgs/msg/Twist) - Velocity commands (same as saye_control)
    """
    
    def __init__(self, node_name: str = 'ackermann_drl_env'):
        """Initialize the DRL environment node.
        
        Args:
            node_name: Name for the ROS 2 node
        """
        super().__init__(node_name)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.scan_received = False
        self.odom_received = False
        self.last_scan_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None
        self.scan_count = 0
        self.odom_count = 0
        self.scan_count_before_reset = 0
        self.odom_count_before_reset = 0
        
        self.battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01, vehicle_weight=1000.0)
        self.delivery_points = DeliveryPoints()
        self.current_goal: Optional[Dict] = None
        try:
            self.roads_geometry = RoadsGeometry()
            road_count = self.roads_geometry.get_all_roads_count()
            self.get_logger().info(f"Roads geometry loaded successfully: {road_count} roads")
            if road_count == 0:
                self.get_logger().error("CRITICAL: No roads loaded! Off-road collision detection will NOT work!")
        except Exception as e:
            self.get_logger().error(f"CRITICAL: Failed to load roads geometry: {e}. Off-road collision detection will NOT work!")
            self.get_logger().error("This means the car can crash into sidewalks without penalty!")
            self.roads_geometry = None
        
        self.lidar_downsample_factor = 4
        self.lidar_downsampled_size = 720 // self.lidar_downsample_factor
        
        self.reward_progress_scale = 1.0
        self.reward_goal_reached = 50.0
        self.reward_offroad_penalty = -0.05
        self.reward_collision_penalty = -20.0
        self.reward_battery_penalty_scale = -0.5
        self.reward_time_penalty = -0.01
        self.goal_reached_threshold = 2.0
        self.collision_threshold = 1.5
        self.offroad_collision_threshold = 0.5  # Lower threshold to detect sidewalk collisions
        
        self.prev_distance_to_goal: Optional[float] = None
        self.prev_position: Optional[np.ndarray] = None
        self.prev_position_time: Optional[float] = None
        self.delivery_start_time: Optional[float] = None
        self.delivery_deadline: Optional[float] = None
        self.delivery_elapsed_time: float = 0.0
        self.delivery_on_time: bool = False
        self.reward_delivery_on_time = 30.0
        self.reward_delivery_late_penalty = -10.0
        
        self.step_count = 0
        self.episode_count = 0
        self.episode_step_count = 0
        self.prev_velocity_for_efficiency = None
        self.battery_consumed_this_step = 0.0
        self.executor = None
        if rclpy.ok():
            try:
                context = rclpy.get_default_context()
                if context is not None and context.ok():
                    executor = SingleThreadedExecutor(context=context)
                    executor.add_node(self)
                    self.executor = executor
                    self.get_logger().info("Executor created successfully")
                else:
                    self.get_logger().warn("Context is None or invalid, executor not created - will use rclpy.spin_once")
                    self.executor = None
            except Exception as e:
                self.get_logger().warn(f"Failed to create executor: {e}, will use rclpy.spin_once fallback")
                self.executor = None
        else:
            self.get_logger().error("CRITICAL: rclpy not ok during initialization! ROS context is invalid!")
            self.get_logger().error("This will prevent the environment from working. Check ROS initialization.")
            self.executor = None
        
        self.get_logger().info("AckermannCityEnv initialized (Docker-ready)")
    
    def _scan_callback(self, msg: LaserScan):
        """Callback for laser scan messages."""
        self.latest_scan = msg
        self.scan_received = True
        self.last_scan_time = time.time()
        self.scan_count += 1
    
    def _odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        self.latest_odom = msg
        self.odom_received = True
        self.last_odom_time = time.time()
        self.odom_count += 1
    
    def spin_once(self, timeout_sec: float = 0.1):
        """Spin executor once to process callbacks.
        
        Args:
            timeout_sec: Timeout for spinning
        """
        if self.executor is not None:
            try:
                self.executor.spin_once(timeout_sec=timeout_sec)
            except Exception as e:
                # If executor fails, use rclpy.spin_once as fallback
                try:
                    rclpy.spin_once(self, timeout_sec=timeout_sec)
                except Exception:
                    pass
        else:
            # Fallback: use rclpy.spin_once to process callbacks
            try:
                rclpy.spin_once(self, timeout_sec=timeout_sec)
            except Exception as e:
                # Last resort: just wait a bit
                import time
                time.sleep(0.01)
    
    def reset(self) -> np.ndarray:
        """Reset the environment and return initial observation.
        
        Selects a new random goal (delivery point) on reset.
        
        Returns:
            Initial observation array with complete observation vector
        """
        self.episode_count += 1
        self.step_count = 0
        print(f"\n{'='*80}")
        print(f"EPISODE {self.episode_count} STARTED - Resetting environment")
        print(f"{'='*80}\n")
        safe_log(self.get_logger().info, "Resetting environment")
        self.scan_received = False
        self.odom_received = False
        self.scan_count_before_reset = self.scan_count
        self.odom_count_before_reset = self.odom_count
        
        self.battery.reset()
        self.prev_distance_to_goal = None
        self.prev_position = None
        self.prev_position_time = None
        self.episode_step_count = 0
        self.prev_velocity_for_efficiency = None
        self.battery_consumed_this_step = 0.0
        self.delivery_start_time = time.time()
        self.delivery_elapsed_time = 0.0
        self.delivery_on_time = False
        
        try:
            self.current_goal = self.delivery_points.get_next_point(None)
            goal_pos = self.delivery_points.get_point_position(self.current_goal)
            self.delivery_deadline = self.current_goal.get('deadline_seconds', 120.0)
            
            safe_log(self.get_logger().info,
                f"Selected delivery: {self.current_goal.get('name', 'unknown')} "
                f"at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) "
                f"| Deadline: {self.delivery_deadline:.1f}s"
            )
        except Exception as e:
            safe_log(self.get_logger().warn, f"Failed to select goal: {e}. Continuing without goal.")
            self.current_goal = None
            self.delivery_deadline = None
        
        max_wait_iterations = 200
        for i in range(max_wait_iterations):
            self.spin_once(timeout_sec=0.05)
            scan_new = self.scan_count > self.scan_count_before_reset
            odom_new = self.odom_count > self.odom_count_before_reset
            if scan_new and odom_new:
                safe_log(self.get_logger().info, f"[RESET] Received initial sensor data after {i+1} spins (scan={self.scan_count}, odom={self.odom_count})")
                break
        
        if self.scan_count <= self.scan_count_before_reset:
            safe_log(self.get_logger().warn, f"[RESET] No initial LiDAR data received (count={self.scan_count}, before={self.scan_count_before_reset})")
        if self.odom_count <= self.odom_count_before_reset:
            safe_log(self.get_logger().warn, f"[RESET] No initial odometry data received (count={self.odom_count}, before={self.odom_count_before_reset})")
        
        if self.current_goal is not None and self.latest_odom is not None:
            dx, dy, _ = self.compute_goal_deltas()
            self.prev_distance_to_goal = np.sqrt(dx**2 + dy**2)
            # Initialize prev_position for velocity calculation
            pos = self.latest_odom.pose.pose.position
            self.prev_position = np.array([pos.x, pos.y, pos.z])
            self.prev_position_time = time.time()
        
        obs = self.get_observation()
        return obs
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step in the environment.
        
        Args:
            action: Action array [linear_velocity, angular_velocity]
                   - linear_velocity: m/s (forward/backward)
                   - angular_velocity: rad/s (steering)
            
        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        
        context_valid = rclpy.ok()
        if context_valid and self.cmd_vel_pub is not None:
            try:
                self.cmd_vel_pub.publish(cmd)
            except Exception as e:
                if context_valid and "context" not in str(e).lower():
                    try:
                        self.get_logger().error(f"Failed to publish command: {e}")
                    except:
                        pass
        
        self.step_count += 1
        scan_count_before = self.scan_count
        odom_count_before = self.odom_count
        
        start_time = time.time()
        max_wait_time = 2.0
        min_spins = 20
        
        spin_iterations = 0
        while (time.time() - start_time) < max_wait_time or spin_iterations < min_spins:
            self.spin_once(timeout_sec=0.05)
            spin_iterations += 1
            if spin_iterations >= min_spins and (self.scan_count > scan_count_before and self.odom_count > odom_count_before):
                break
        
        if self.scan_count == scan_count_before:
            safe_log(self.get_logger().warn, f"[SENSOR] No new LiDAR data received after {spin_iterations} spins (count={self.scan_count})")
        if self.odom_count == odom_count_before:
            safe_log(self.get_logger().warn, f"[SENSOR] No new odometry data received after {spin_iterations} spins (count={self.odom_count})")
        
        # Calculate velocity BEFORE updating prev_position (so it uses previous step's position)
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            position = np.array([pos.x, pos.y, pos.z])
            # Get velocity BEFORE updating prev_position (uses previous step's position)
            self.current_step_velocity, _ = self.get_velocity_and_steering()
            
            # NOW update previous position for NEXT step's velocity calculation
            self.prev_position = position.copy()
            self.prev_position_time = time.time()
            
            battery_before = self.battery.get_battery_level()
            self.battery.update(position, self.current_step_velocity, dt=0.1)
            battery_after = self.battery.get_battery_level()
            self.battery_consumed_this_step = battery_before - battery_after
        else:
            self.current_step_velocity = 0.0
            self.battery_consumed_this_step = 0.0
        
        self.episode_step_count += 1
        reward, reward_info = self.compute_reward()
        
        if self.is_goal_reached() and self.current_goal is not None:
            next_goal = self.delivery_points.get_next_point(self.current_goal)
            if next_goal is not None:
                self.current_goal = next_goal
                goal_pos = self.delivery_points.get_point_position(self.current_goal)
                safe_log(self.get_logger().info,
                    f"Goal reached! Moving to next goal: {self.current_goal.get('name', 'unknown')} "
                    f"at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})"
                )
                self.prev_distance_to_goal = None
        
        observation = self.get_observation()
        terminated, truncated = self.check_termination()
        
        min_lidar_dist = -1.0
        if self.latest_scan is not None:
            try:
                ranges = np.array(self.latest_scan.ranges)
                valid_ranges = ranges[np.isfinite(ranges)]
                if len(valid_ranges) > 0:
                    min_lidar_dist = float(np.min(valid_ranges))
            except:
                pass
        
        velocity = -1.0
        distance_to_goal = -1.0
        if self.latest_odom is not None:
            velocity, _ = self.get_velocity_and_steering()
            distance_to_goal = self.get_distance_to_goal()
        
        current_time = time.time()
        scan_is_fresh = (self.last_scan_time is not None and 
                        (current_time - self.last_scan_time) < 2.0)
        odom_is_fresh = (self.last_odom_time is not None and 
                        (current_time - self.last_odom_time) < 2.0)
        
        info = {
            'scan_received': self.scan_received,
            'odom_received': self.odom_received,
            'scan_is_fresh': scan_is_fresh,
            'odom_is_fresh': odom_is_fresh,
            'scan_count': self.scan_count,
            'odom_count': self.odom_count,
            'battery_level': self.battery.get_battery_level(),
            'battery_depleted': self.battery.is_depleted(),
            'current_goal': self.current_goal.get('name') if self.current_goal else None,
            'has_scan': self.latest_scan is not None,
            'has_odom': self.latest_odom is not None,
            'has_goal': self.current_goal is not None,
            'velocity': velocity,
            'distance_to_goal': distance_to_goal,
            **reward_info
        }
        
        if not scan_is_fresh and self.latest_scan is not None:
            safe_log(self.get_logger().warn, f"[STALE DATA] LiDAR data is stale! Last update: {current_time - self.last_scan_time:.2f}s ago")
        if not odom_is_fresh and self.latest_odom is not None:
            safe_log(self.get_logger().warn, f"[STALE DATA] Odometry data is stale! Last update: {current_time - self.last_odom_time:.2f}s ago")
        
        return observation, reward, terminated, truncated, info
    
    def compute_goal_deltas(self) -> Tuple[float, float, float]:
        """Compute Δx, Δy, Δθ to current goal.
        
        Returns:
            Tuple of (Δx, Δy, Δθ) in robot frame:
            - Δx: Distance along robot's forward direction (east)
            - Δy: Distance along robot's left direction (north)
            - Δθ: Angle to goal relative to robot heading (radians)
        """
        if self.current_goal is None or self.latest_odom is None:
            return (0.0, 0.0, 0.0)
        
        goal_pos = self.delivery_points.get_point_position(self.current_goal)
        goal_x = goal_pos[0]
        goal_y = goal_pos[1]
        
        robot_pos = self.latest_odom.pose.pose.position
        robot_x = robot_pos.x
        robot_y = robot_pos.y
        
        robot_orient = self.latest_odom.pose.pose.orientation
        siny_cosp = 2.0 * (robot_orient.w * robot_orient.z + robot_orient.x * robot_orient.y)
        cosy_cosp = 1.0 - 2.0 * (robot_orient.y * robot_orient.y + robot_orient.z * robot_orient.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        dx_world = goal_x - robot_x
        dy_world = goal_y - robot_y
        
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        dx = dx_world * cos_yaw - dy_world * sin_yaw
        dy = dx_world * sin_yaw + dy_world * cos_yaw
        
        goal_yaw = math.atan2(dy_world, dx_world)
        dtheta = goal_yaw - robot_yaw
        
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        
        return (dx, dy, dtheta)
    
    def downsample_lidar(self, ranges: np.ndarray) -> np.ndarray:
        """Downsample LiDAR scan data.
        
        Args:
            ranges: Full LiDAR scan array (720 samples)
            
        Returns:
            Downsampled array (180 samples)
        """
        downsampled = ranges[::self.lidar_downsample_factor]
        return downsampled[:self.lidar_downsampled_size]
    
    def get_velocity_and_steering(self) -> Tuple[float, float]:
        """Extract velocity and steering from odometry.
        
        Calculates velocity from position changes if odometry twist is not available/accurate.
        
        Returns:
            Tuple of (velocity, steering):
            - velocity: Linear velocity magnitude (m/s)
            - steering: Angular velocity (rad/s) - represents steering rate
        """
        if self.latest_odom is None:
            return (0.0, 0.0)
        
        # Try to get velocity from odometry twist first
        twist = self.latest_odom.twist.twist
        velocity_from_twist = np.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        steering = twist.angular.z
        
        # Always calculate velocity from position changes if we have previous position (more accurate)
        if self.prev_position is not None and self.prev_position_time is not None:
            pos = self.latest_odom.pose.pose.position
            current_position = np.array([pos.x, pos.y, pos.z])
            current_time = time.time()
            dt = current_time - self.prev_position_time
            
            if dt > 0.001:  # Avoid division by zero
                position_change = current_position - self.prev_position
                distance = np.linalg.norm(position_change)
                velocity_from_position = distance / dt
                # Use position-based velocity (more accurate than twist)
                velocity = velocity_from_position
                # Debug logging
                if distance > 0.001:
                    safe_log(self.get_logger().debug, 
                        f"[VELOCITY] Position-based: {velocity:.3f} m/s (moved {distance:.4f}m in {dt:.3f}s, prev_pos={self.prev_position}, curr_pos={current_position})")
            else:
                # dt too small, use twist
                velocity = velocity_from_twist
                safe_log(self.get_logger().debug, f"[VELOCITY] dt too small ({dt:.6f}s), using twist: {velocity_from_twist:.3f} m/s")
        else:
            # No previous position, use twist
            velocity = velocity_from_twist
            safe_log(self.get_logger().debug, f"[VELOCITY] No prev_position (prev={self.prev_position is not None}, time={self.prev_position_time is not None}), using twist: {velocity_from_twist:.3f} m/s")
        
        return (velocity, steering)
    
    def get_road_distance(self) -> float:
        """Get distance to nearest road.
        
        Returns:
            Distance to nearest road in meters (0.0 if on road, positive if off-road)
        """
        if self.roads_geometry is None:
            return 0.0
        
        if self.latest_odom is None:
            return 0.0
        
        # Get robot position
        robot_pos = self.latest_odom.pose.pose.position
        robot_x = robot_pos.x  # east
        robot_y = robot_pos.y  # north
        
        # Get distance to nearest road
        distance, _ = self.roads_geometry.distance_to_nearest_road(robot_x, robot_y)
        return distance
    
    def get_observation(self) -> np.ndarray:
        """Get current observation from sensors.
        
        Complete observation vector includes:
        - Downsampled LiDAR (180 samples)
        - Velocity (1 value)
        - Steering (1 value)
        - Goal deltas: Δx, Δy, Δθ (3 values)
        - Battery level (1 value)
        - Road distance (1 value)
        
        Total: 180 + 1 + 1 + 3 + 1 + 1 = 187 elements
        
        Returns:
            Observation array: 187 elements total
        """
        # Observation structure:
        # [0:180]   - Downsampled LiDAR (180 samples)
        # [180]     - Velocity (m/s)
        # [181]     - Steering (rad/s)
        # [182:185] - Goal deltas: Δx, Δy, Δθ (3 values)
        # [185]     - Battery level [0,1]
        # [186]     - Road distance (m)
        
        obs = np.zeros(187, dtype=np.float32)
        idx = 0
        
        # 1. Downsampled LiDAR (indices 0-179)
        if self.latest_scan is not None:
            ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
            # Replace inf/nan with max range
            ranges = np.nan_to_num(
                ranges, 
                nan=self.latest_scan.range_max, 
                posinf=self.latest_scan.range_max,
                neginf=self.latest_scan.range_max
            )
            # Downsample
            downsampled = self.downsample_lidar(ranges)
            obs[idx:idx+self.lidar_downsampled_size] = downsampled
        idx += self.lidar_downsampled_size  # 180
        
        # 2. Velocity and steering (indices 180-181)
        # Use stored velocity (calculated before prev_position update)
        velocity = self.current_step_velocity
        _, steering = self.get_velocity_and_steering()  # Only need steering from twist
        obs[idx] = velocity
        idx += 1  # 181
        obs[idx] = steering
        idx += 1  # 182
        
        # 3. Goal deltas: Δx, Δy, Δθ (indices 182-184)
        dx, dy, dtheta = self.compute_goal_deltas()
        obs[idx] = dx
        idx += 1  # 183
        obs[idx] = dy
        idx += 1  # 184
        obs[idx] = dtheta
        idx += 1  # 185
        
        # 4. Battery level (index 185)
        obs[idx] = self.battery.get_battery_level()
        idx += 1  # 186
        
        # 5. Road distance (index 186)
        obs[idx] = self.get_road_distance()
        idx += 1  # 187
        
        return obs
    
    def get_distance_to_goal(self) -> float:
        """Get current distance to goal.
        
        Returns:
            Distance to goal in meters
        """
        if self.current_goal is None or self.latest_odom is None:
            return float('inf')
        
        dx, dy, _ = self.compute_goal_deltas()
        distance = np.sqrt(dx**2 + dy**2)
        return distance
    
    def is_goal_reached(self) -> bool:
        """Check if goal is reached.
        
        Returns:
            True if robot is within goal_reached_threshold of goal
        """
        distance = self.get_distance_to_goal()
        return distance <= self.goal_reached_threshold
    
    def is_collision(self) -> bool:
        """Check if robot has collided with obstacle.
        
        Returns:
            True if any LiDAR scan is within collision_threshold
        """
        if self.latest_scan is None:
            return False
        
        ranges = np.array(self.latest_scan.ranges)
        # Filter out inf and nan
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return False
        
        # Check if any scan is too close
        min_distance = np.min(valid_ranges)
        # Use <= to catch cases where min_distance exactly equals threshold
        collision_detected = min_distance <= self.collision_threshold
        
        # Debug: log collision if detected
        if collision_detected:
            self.get_logger().warn(f"Collision detected! min_distance={min_distance:.3f}m <= threshold={self.collision_threshold}m")
        
        return collision_detected
    
    def compute_reward(self) -> Tuple[float, Dict[str, float]]:
        """Compute reward for current step.
        
        Reward components:
        - Progress reward: +reward_progress_scale * (prev_distance - current_distance)
        - Goal reward: +reward_goal_reached if goal reached
        - Off-road penalty: -reward_offroad_penalty * road_distance
        - Collision penalty: -reward_collision_penalty if collision
        - Battery penalty: -reward_battery_penalty_scale * (1 - battery_level)
        - Time penalty: -reward_time_penalty (per step)
        
        Returns:
            Tuple of (total_reward, reward_breakdown_dict)
        """
        reward = 0.0
        reward_info = {
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
            'delivery_elapsed_time': 0.0,
            'delivery_deadline': 0.0,
            'delivery_time_remaining': 0.0,
            'delivery_on_time': False
        }
        
        # Check if sensors are working
        # Use stale data if available - better than nothing
        has_odom = self.latest_odom is not None
        has_scan = self.latest_scan is not None
        has_goal = self.current_goal is not None
        
        # Update delivery elapsed time
        if self.delivery_start_time is not None:
            self.delivery_elapsed_time = time.time() - self.delivery_start_time
            reward_info['delivery_elapsed_time'] = self.delivery_elapsed_time
            if self.delivery_deadline is not None:
                reward_info['delivery_deadline'] = self.delivery_deadline
                reward_info['delivery_time_remaining'] = max(0.0, self.delivery_deadline - self.delivery_elapsed_time)
        
        # Log if we're using stale data
        if has_odom and self.last_odom_time is not None:
            time_since_odom = time.time() - self.last_odom_time
            if time_since_odom > 1.0:  # More than 1 second old
                safe_log(self.get_logger().warn, f"[REWARD] Using stale odometry data ({time_since_odom:.2f}s old)")
        if has_scan and self.last_scan_time is not None:
            time_since_scan = time.time() - self.last_scan_time
            if time_since_scan > 1.0:  # More than 1 second old
                safe_log(self.get_logger().warn, f"[REWARD] Using stale LiDAR data ({time_since_scan:.2f}s old)")
        
        # 1. Progress reward (getting closer to goal)
        if has_goal and has_odom:
            current_distance = self.get_distance_to_goal()
            
            if self.prev_distance_to_goal is not None:
                progress = self.prev_distance_to_goal - current_distance
                progress_reward = self.reward_progress_scale * progress
                reward += progress_reward
                reward_info['reward_progress'] = progress_reward
                # Debug: log if no progress despite movement
                if abs(progress) < 0.01 and self.latest_odom is not None:
                    velocity = self.current_step_velocity
                    if velocity > 0.1:  # Car is moving
                        safe_log(self.get_logger().debug, 
                            f"[PROGRESS] Car moving ({velocity:.2f} m/s) but no progress: prev={self.prev_distance_to_goal:.2f}m, curr={current_distance:.2f}m, diff={progress:.4f}m")
            else:
                reward_info['reward_progress'] = 0.0
            
            self.prev_distance_to_goal = current_distance
            if self.is_goal_reached():
                reward += self.reward_goal_reached
                reward_info['reward_goal'] = self.reward_goal_reached
                safe_log(self.get_logger().info, f"[GOAL] Goal reached! Reward: +{self.reward_goal_reached:.4f}")
                
                if self.delivery_deadline is not None and self.delivery_elapsed_time <= self.delivery_deadline:
                    reward += self.reward_delivery_on_time
                    reward_info['reward_delivery_on_time'] = self.reward_delivery_on_time
                    self.delivery_on_time = True
                    reward_info['delivery_on_time'] = True
                    safe_log(self.get_logger().info, 
                        f"[DELIVERY] On-time delivery! Elapsed: {self.delivery_elapsed_time:.1f}s / Deadline: {self.delivery_deadline:.1f}s | Reward: +{self.reward_delivery_on_time}")
                elif self.delivery_deadline is not None:
                    seconds_late = self.delivery_elapsed_time - self.delivery_deadline
                    late_penalty = self.reward_delivery_late_penalty * seconds_late
                    reward += late_penalty
                    reward_info['penalty_delivery_late'] = late_penalty
                    self.delivery_on_time = False
                    reward_info['delivery_on_time'] = False
                    safe_log(self.get_logger().warn,
                        f"[DELIVERY] Late delivery! Elapsed: {self.delivery_elapsed_time:.1f}s / Deadline: {self.delivery_deadline:.1f}s | Late by: {seconds_late:.1f}s | Penalty: {late_penalty:.2f}")
                else:
                    reward_info['delivery_on_time'] = False
            else:
                if self.delivery_deadline is not None and self.delivery_elapsed_time > self.delivery_deadline:
                    seconds_late = self.delivery_elapsed_time - self.delivery_deadline
                    late_penalty = self.reward_delivery_late_penalty * 0.1 * seconds_late
                    reward += late_penalty
                    reward_info['penalty_delivery_late'] = late_penalty
                    reward_info['delivery_on_time'] = False
        elif not has_goal:
            reward_info['reward_progress'] = 0.0
        elif not has_odom:
            reward_info['reward_progress'] = 0.0
        if has_odom:
            road_distance = self.get_road_distance()
            if road_distance > 0.0:
                offroad_penalty = self.reward_offroad_penalty * road_distance
                reward += offroad_penalty
                reward_info['penalty_offroad'] = offroad_penalty
            else:
                reward_info['penalty_offroad'] = 0.0
        else:
            reward_info['penalty_offroad'] = 0.0
        
        is_colliding_lidar = False
        is_colliding_offroad = False
        min_lidar_dist = -1.0
        road_distance = -1.0
        
        if self.latest_scan is not None:
            try:
                ranges = np.array(self.latest_scan.ranges)
                valid_ranges = ranges[np.isfinite(ranges)]
                if len(valid_ranges) > 0:
                    min_lidar_dist = float(np.min(valid_ranges))
                    # Check collision AFTER getting min_distance
                    is_colliding_lidar = min_lidar_dist <= self.collision_threshold
                    if is_colliding_lidar:
                        safe_log(self.get_logger().warn, f"[COLLISION] LiDAR collision! min={min_lidar_dist:.3f}m <= threshold={self.collision_threshold}m")
            except:
                pass
        
        if self.latest_odom is not None:
            road_distance = self.get_road_distance()
            # Check for off-road collision (sidewalk, etc.) - more sensitive threshold
            if road_distance > self.offroad_collision_threshold:
                is_colliding_offroad = True
                safe_log(self.get_logger().warn, f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m")
            # Also check if road_distance is significant (car is off-road) even if below collision threshold
            elif road_distance > 0.3:  # Car is noticeably off-road but not quite collision threshold
                safe_log(self.get_logger().debug, f"[OFF-ROAD] Car is off-road: road_distance={road_distance:.3f}m (threshold={self.offroad_collision_threshold}m)")
        
        is_colliding = is_colliding_lidar or is_colliding_offroad
        
        if self.latest_odom is not None and road_distance == -1.0:
            road_distance = self.get_road_distance()
        
        reward_info['is_collision'] = bool(is_colliding)
        reward_info['is_collision_lidar'] = bool(is_colliding_lidar)
        reward_info['is_collision_offroad'] = bool(is_colliding_offroad)
        reward_info['min_lidar_distance'] = float(min_lidar_dist)
        reward_info['road_distance'] = float(road_distance)
        
        if is_colliding:
            reward += self.reward_collision_penalty
            reward_info['penalty_collision'] = self.reward_collision_penalty
            collision_types = []
            if is_colliding_lidar:
                collision_types.append(f"LiDAR (min={min_lidar_dist:.2f}m)")
            if is_colliding_offroad:
                collision_types.append(f"Off-road (dist={road_distance:.2f}m)")
            safe_log(self.get_logger().warn, f"[COLLISION] Collision detected! Type: {', '.join(collision_types)} | Penalty: {self.reward_collision_penalty:.4f}")
            if is_colliding_lidar:
                safe_log(self.get_logger().warn, f"[COLLISION] LiDAR collision detected! min_distance={min_lidar_dist:.3f}m <= threshold={self.collision_threshold}m | Penalty: {self.reward_collision_penalty}")
            if is_colliding_offroad:
                safe_log(self.get_logger().warn, f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m | Penalty: {self.reward_collision_penalty}")
        else:
            reward_info['penalty_collision'] = 0.0
        
        
        # 5. Battery efficiency system (zone-based - rewards high battery, penalizes critical)
        battery_level = self.battery.get_battery_level()
        battery_consumed = self.battery_consumed_this_step
        
        # Zone-based battery reward system
        if battery_level > 0.8:      # High battery (80-100%)
            battery_conservation_reward = 0.1
            battery_zone = "HIGH"
        elif battery_level > 0.5:   # Medium battery (50-80%)
            battery_conservation_reward = 0.05
            battery_zone = "MEDIUM"
        elif battery_level > 0.2:    # Low battery (20-50%)
            battery_conservation_reward = 0.01
            battery_zone = "LOW"
        else:                         # Critical battery (<20%)
            battery_conservation_reward = -0.05
            battery_zone = "CRITICAL"
        
        reward += battery_conservation_reward
        reward_info['reward_battery_conservation'] = battery_conservation_reward
        
        if battery_consumed > 0.0 and reward_info.get('reward_progress', 0.0) > 0.01:
            efficiency_reward = 0.2 * (reward_info['reward_progress'] / battery_consumed)
            reward += efficiency_reward
            reward_info['reward_efficiency'] = efficiency_reward
        else:
            reward_info['reward_efficiency'] = 0.0
        
        if self.latest_odom is not None:
            # Use stored velocity (calculated before prev_position update)
            velocity = self.current_step_velocity
            _, steering = self.get_velocity_and_steering()  # Only need steering from twist
            
            if velocity > 3.0:
                high_speed_penalty = -0.02 * (velocity - 3.0)
                reward += high_speed_penalty
                reward_info['penalty_high_speed'] = high_speed_penalty
            else:
                reward_info['penalty_high_speed'] = 0.0
            
            if self.prev_velocity_for_efficiency is not None:
                velocity_change = abs(velocity - self.prev_velocity_for_efficiency)
                if velocity_change > 1.0:
                    aggressive_penalty = -0.05 * (velocity_change - 1.0)
                    reward += aggressive_penalty
                    reward_info['penalty_aggressive_change'] = aggressive_penalty
                else:
                    reward_info['penalty_aggressive_change'] = 0.0
            self.prev_velocity_for_efficiency = velocity
        
        # Battery info stored in reward_info, will be shown in breakdown
        
        reward += self.reward_time_penalty
        reward_info['penalty_time'] = self.reward_time_penalty
        
        # Get diagnostic info (use stored velocity)
        velocity = self.current_step_velocity
        distance_to_goal = -1.0
        if self.latest_odom is not None:
            distance_to_goal = self.get_distance_to_goal()
        
        # Print formatted reward breakdown
        print(f"\n{'='*70}")
        print(f"STEP #{self.episode_step_count} - REWARD BREAKDOWN:")
        print(f"{'='*70}")
        print(f"  Progress Reward:        {reward_info.get('reward_progress', 0.0):+10.4f}")
        print(f"  Goal Reward:            {reward_info.get('reward_goal', 0.0):+10.4f}")
        print(f"  Delivery On-time:       {reward_info.get('reward_delivery_on_time', 0.0):+10.4f}")
        print(f"  Battery Conservation:   {reward_info.get('reward_battery_conservation', 0.0):+10.4f}")
        print(f"  Efficiency Reward:      {reward_info.get('reward_efficiency', 0.0):+10.4f}")
        print(f"  Delivery Late Penalty:  {reward_info.get('penalty_delivery_late', 0.0):+10.4f}")
        print(f"  Off-road Penalty:       {reward_info.get('penalty_offroad', 0.0):+10.4f}")
        print(f"  Collision Penalty:      {reward_info.get('penalty_collision', 0.0):+10.4f}")
        print(f"  High Speed Penalty:     {reward_info.get('penalty_high_speed', 0.0):+10.4f}")
        print(f"  Aggressive Change:      {reward_info.get('penalty_aggressive_change', 0.0):+10.4f}")
        print(f"  Time Penalty:           {reward_info.get('penalty_time', 0.0):+10.4f}")
        print(f"{'-'*70}")
        print(f"  TOTAL REWARD:           {reward:+10.4f}")
        print(f"{'='*70}")
        print(f"  Diagnostics: velocity={velocity:.2f} m/s | dist_to_goal={distance_to_goal:.2f}m | min_lidar={min_lidar_dist:.2f}m | road_dist={road_distance:.2f}m")
        if reward_info.get('prev_distance') is not None:
            print(f"  Progress: moved {reward_info.get('progress_meters', 0.0):+.3f}m (prev={reward_info.get('prev_distance', 0.0):.2f}m -> curr={reward_info.get('current_distance', 0.0):.2f}m)")
        print(f"  Collision Check: min_lidar={min_lidar_dist:.2f}m vs threshold={self.collision_threshold:.2f}m {'[COLLISION!]' if is_colliding else '[OK]'}")
        if self.delivery_deadline is not None:
            status = "(LATE)" if self.delivery_elapsed_time > self.delivery_deadline else "(on-time)"
            print(f"  Delivery: {self.delivery_elapsed_time:.1f}s / {self.delivery_deadline:.1f}s {status}")
        print(f"{'='*70}\n")
        
        return reward, reward_info
    
    def check_termination(self) -> Tuple[bool, bool]:
        """Check if episode should terminate.
        
        Returns:
            Tuple of (terminated, truncated):
            - terminated: True if goal reached or collision
            - truncated: True if battery depleted or max steps (not implemented yet)
        """
        terminated = False
        truncated = False
        
        # Note: Goal reached is handled in step() to move to next goal
        # Only terminate if all goals are completed (handled in step())
        
        # Terminate if collision (LiDAR or off-road)
        if self.is_collision():
            terminated = True
        
        # Also check off-road collision (sidewalk/curb)
        if self.latest_odom is not None:
            road_distance = self.get_road_distance()
            if road_distance > self.offroad_collision_threshold:
                terminated = True
        
        # Truncate if battery depleted
        if self.battery.is_depleted():
            truncated = True
        
        return terminated, truncated
    
    def stop(self):
        """Stop the car by sending zero velocity command.
        
        This should be called before closing the environment to ensure
        the car stops moving when training ends.
        """
        if self.cmd_vel_pub is not None and rclpy.ok():
            try:
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                # Publish stop command multiple times to ensure it's received
                for _ in range(5):
                    self.cmd_vel_pub.publish(stop_cmd)
                    # Spin briefly to ensure message is sent
                    self.spin_once(timeout_sec=0.05)
                safe_log(self.get_logger().info, "Car stopped (zero velocity command sent)")
            except Exception as e:
                # Ignore errors if context is invalid or publisher is unavailable
                safe_log(self.get_logger().warn, f"Could not send stop command: {e}")

