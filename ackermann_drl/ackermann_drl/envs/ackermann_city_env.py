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
        
        # Subscriptions
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
        
        # Publishers - publishes to same topic as saye_control
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State storage
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.scan_received = False
        self.odom_received = False
        
        # Battery model
        self.battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
        
        # Delivery points (goals)
        self.delivery_points = DeliveryPoints()
        self.current_goal: Optional[Dict] = None
        
        # Roads geometry (for road distance calculation)
        try:
            self.roads_geometry = RoadsGeometry()
            self.get_logger().info("Roads geometry loaded successfully")
        except Exception as e:
            self.get_logger().warn(f"Failed to load roads geometry: {e}. Road distance will be zero.")
            self.roads_geometry = None
        
        # Observation parameters
        self.lidar_downsample_factor = 4  # Downsample 720 to 180 samples
        self.lidar_downsampled_size = 720 // self.lidar_downsample_factor  # 180
        
        # Reward parameters
        self.reward_progress_scale = 0.1  # Scale for progress reward
        self.reward_goal_reached = 10.0  # Reward for reaching goal
        self.reward_offroad_penalty = -0.1  # Penalty per meter off-road
        self.reward_collision_penalty = -10.0  # Penalty for collision
        self.reward_battery_penalty_scale = -0.5  # Penalty scale for low battery
        self.reward_time_penalty = -0.01  # Small penalty per step
        
        # Goal reached threshold (meters)
        self.goal_reached_threshold = 2.0  # Consider goal reached if within 2m
        
        # Collision detection threshold (meters)
        # Note: LiDAR might not detect low obstacles (sidewalks) perfectly, so we use a more aggressive threshold
        # Also check road distance as a proxy for collision with curbs/sidewalks
        self.collision_threshold = 0.5  # Consider collision if obstacle within 0.5m (increased for better sidewalk detection)
        self.offroad_collision_threshold = 0.2  # If off-road by more than this, consider it a collision with sidewalk/curb (lowered for better detection)
        
        # Track previous distance for progress calculation
        self.prev_distance_to_goal: Optional[float] = None
        
        # Executor for spinning (minimal - single thread)
        # Create executor - it will use the default context from rclpy.init()
        # Note: rclpy.init() must be called before creating executor (done in gym_wrapper)
        self.executor = None
        if rclpy.ok():
            try:
                # Get the default context explicitly
                context = rclpy.get_default_context()
                if context is not None and context.ok():
                    executor = SingleThreadedExecutor(context=context)
                    executor.add_node(self)
                    self.executor = executor
                    self.get_logger().info("Executor created successfully")
                else:
                    self.get_logger().warn("Context is None or invalid, executor not created")
                    self.executor = None
            except Exception as e:
                self.get_logger().warn(f"Failed to create executor: {e}, will use rclpy.spin_once fallback")
                self.executor = None
        else:
            self.get_logger().warn("rclpy not ok, executor not created")
            self.executor = None
        
        self.get_logger().info("AckermannCityEnv initialized (Docker-ready)")
    
    def _scan_callback(self, msg: LaserScan):
        """Callback for laser scan messages."""
        self.latest_scan = msg
        self.scan_received = True
    
    def _odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        self.latest_odom = msg
        self.odom_received = True
    
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
        self.get_logger().info("Resetting environment")
        # Reset state storage
        self.latest_scan = None
        self.latest_odom = None
        self.scan_received = False
        self.odom_received = False
        
        # Reset battery
        self.battery.reset()
        
        # Reset previous distance tracking
        self.prev_distance_to_goal = None
        
        # Select new random goal
        try:
            self.current_goal = self.delivery_points.get_random_point()
            goal_pos = self.delivery_points.get_point_position(self.current_goal)
            self.get_logger().info(
                f"Selected goal: {self.current_goal.get('name', 'unknown')} "
                f"at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to select goal: {e}. Continuing without goal.")
            self.current_goal = None
        
        # Spin briefly to allow any pending messages
        # Wait longer for initial sensor data
        for _ in range(50):
            self.spin_once(timeout_sec=0.05)  # Wait up to 2.5 seconds for initial data
        
        # Initialize previous distance
        if self.current_goal is not None and self.latest_odom is not None:
            dx, dy, _ = self.compute_goal_deltas()
            self.prev_distance_to_goal = np.sqrt(dx**2 + dy**2)
        
        # Return observation
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
        # Publish control command to /cmd_vel (same topic as saye_control)
        # Note: AckermannSteering plugin handles conversion to steering angles
        if not rclpy.ok():
            self.get_logger().error("rclpy context invalid, cannot publish command")
            # Return zero observation and negative reward
            obs = self.get_observation()
            return obs, -10.0, True, False, {'error': 'rclpy context invalid'}
        
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        
        try:
            self.cmd_vel_pub.publish(cmd)
        except Exception as e:
            self.get_logger().error(f"Failed to publish command: {e}")
            # Return zero observation and negative reward
            obs = self.get_observation()
            return obs, -10.0, True, False, {'error': f'publish failed: {e}'}
        
        # Spin briefly to process any incoming messages
        # Give more time for sensor data to arrive - use longer timeout
        # We need to ensure callbacks are processed, so spin more aggressively
        import time
        start_time = time.time()
        max_wait_time = 0.5  # Wait up to 500ms for sensor data
        
        # Track if we received new data
        scan_received_before = self.scan_received
        odom_received_before = self.odom_received
        
        while (time.time() - start_time) < max_wait_time:
            self.spin_once(timeout_sec=0.05)
            # Break early if we got both sensors (check if we received new data)
            if self.latest_scan is not None and self.latest_odom is not None:
                # Make sure we actually received new data, not just old data
                if self.scan_received and self.odom_received:
                    break
        
        # Update battery based on odometry
        if self.latest_odom is not None:
            # Extract position and velocity from odometry
            pos = self.latest_odom.pose.pose.position
            position = np.array([pos.x, pos.y, pos.z])
            
            # Calculate velocity magnitude from twist
            twist = self.latest_odom.twist.twist
            velocity = np.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
            
            # Update battery (dt is approximate, not used in formula)
            self.battery.update(position, velocity, dt=0.1)
        
        # Compute reward FIRST while sensor data is still available
        # (get_observation() might access sensor data, so compute reward before it)
        reward, reward_info = self.compute_reward()
        
        # Get observation from sensors (includes battery level)
        observation = self.get_observation()
        
        # Check termination conditions
        terminated, truncated = self.check_termination()
        
        # Calculate min LiDAR distance safely
        min_lidar_dist = -1.0
        if self.latest_scan is not None:
            try:
                ranges = np.array(self.latest_scan.ranges)
                valid_ranges = ranges[np.isfinite(ranges)]
                if len(valid_ranges) > 0:
                    min_lidar_dist = float(np.min(valid_ranges))
            except:
                pass
        
        info = {
            'scan_received': self.scan_received,
            'odom_received': self.odom_received,
            'battery_level': self.battery.get_battery_level(),
            'battery_depleted': self.battery.is_depleted(),
            'current_goal': self.current_goal.get('name') if self.current_goal else None,
            'has_scan': self.latest_scan is not None,
            'has_odom': self.latest_odom is not None,
            'has_goal': self.current_goal is not None,
            'road_distance': self.get_road_distance() if self.latest_odom is not None else -1.0,
            'min_lidar_distance': min_lidar_dist,
            **reward_info  # Add reward breakdown (includes penalty_collision, is_collision, etc.)
        }
        
        # DEBUG: Verify collision info is in the dict
        if reward_info.get('is_collision', False):
            self.get_logger().debug(f"DEBUG: info dict collision state: is_collision={info.get('is_collision')}, penalty_collision={info.get('penalty_collision')}")
        
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
        
        # Get goal position
        goal_pos = self.delivery_points.get_point_position(self.current_goal)
        goal_x = goal_pos[0]  # east
        goal_y = goal_pos[1]  # north
        
        # Get robot position and orientation
        robot_pos = self.latest_odom.pose.pose.position
        robot_x = robot_pos.x  # east
        robot_y = robot_pos.y  # north
        
        # Get robot orientation (quaternion to yaw)
        robot_orient = self.latest_odom.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (robot_orient.w * robot_orient.z + robot_orient.x * robot_orient.y)
        cosy_cosp = 1.0 - 2.0 * (robot_orient.y * robot_orient.y + robot_orient.z * robot_orient.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Compute deltas in world frame
        dx_world = goal_x - robot_x  # east
        dy_world = goal_y - robot_y  # north
        
        # Transform to robot frame (rotate by -robot_yaw)
        # In robot frame: x is forward (east when yaw=0), y is left (north when yaw=0)
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        dx = dx_world * cos_yaw - dy_world * sin_yaw  # Forward
        dy = dx_world * sin_yaw + dy_world * cos_yaw  # Left
        
        # Compute angle to goal
        goal_yaw = math.atan2(dy_world, dx_world)
        dtheta = goal_yaw - robot_yaw
        
        # Normalize angle to [-π, π]
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
        # Simple downsampling: take every Nth sample
        downsampled = ranges[::self.lidar_downsample_factor]
        return downsampled[:self.lidar_downsampled_size]
    
    def get_velocity_and_steering(self) -> Tuple[float, float]:
        """Extract velocity and steering from odometry.
        
        Returns:
            Tuple of (velocity, steering):
            - velocity: Linear velocity magnitude (m/s)
            - steering: Angular velocity (rad/s) - represents steering rate
        """
        if self.latest_odom is None:
            return (0.0, 0.0)
        
        twist = self.latest_odom.twist.twist
        velocity = np.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        steering = twist.angular.z  # Angular velocity (rad/s)
        
        return (velocity, steering)
    
    def get_road_distance(self) -> float:
        """Get distance to nearest road.
        
        Returns:
            Distance to nearest road in meters (0.0 if on road, positive if off-road)
        """
        if self.roads_geometry is None or self.latest_odom is None:
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
        velocity, steering = self.get_velocity_and_steering()
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
            'penalty_offroad': 0.0,
            'penalty_collision': 0.0,
            'penalty_battery': 0.0,
            'penalty_time': 0.0
        }
        
        # Debug: Check if sensors are working
        has_odom = self.latest_odom is not None
        has_scan = self.latest_scan is not None
        has_goal = self.current_goal is not None
        
        # 1. Progress reward (getting closer to goal)
        if has_goal and has_odom:
            current_distance = self.get_distance_to_goal()
            
            if self.prev_distance_to_goal is not None:
                progress = self.prev_distance_to_goal - current_distance
                progress_reward = self.reward_progress_scale * progress
                reward += progress_reward
                reward_info['reward_progress'] = progress_reward
            else:
                # First step after reset - initialize prev_distance
                # No progress reward on first step, but set it for next step
                reward_info['reward_progress'] = 0.0
                # Initialize for next step
                self.prev_distance_to_goal = current_distance
            
            # Update previous distance for next step
            if self.prev_distance_to_goal is not None:
                self.prev_distance_to_goal = current_distance
            
            # 2. Goal reward (reaching goal)
            if self.is_goal_reached():
                reward += self.reward_goal_reached
                reward_info['reward_goal'] = self.reward_goal_reached
        elif not has_goal:
            # No goal selected - this shouldn't happen but handle it
            reward_info['reward_progress'] = 0.0
        elif not has_odom:
            # No odometry - can't compute progress
            reward_info['reward_progress'] = 0.0
        
        # 3. Off-road penalty
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
        
        # 4. Collision penalty
        # Check both LiDAR collision and off-road collision (sidewalk/curb)
        # IMPORTANT: Always check collisions if we have ANY sensor data, even if flags are False
        is_colliding_lidar = False
        is_colliding_offroad = False
        min_lidar_dist = -1.0
        road_distance = -1.0
        
        # Check LiDAR collision - use latest_scan directly, not has_scan flag
        if self.latest_scan is not None:
            is_colliding_lidar = self.is_collision()
            # Get min LiDAR distance for logging
            try:
                ranges = np.array(self.latest_scan.ranges)
                valid_ranges = ranges[np.isfinite(ranges)]
                if len(valid_ranges) > 0:
                    min_lidar_dist = float(np.min(valid_ranges))
            except:
                pass
        
        # Also check if severely off-road (likely hitting sidewalk/curb)
        # Use latest_odom directly, not has_odom flag
        if self.latest_odom is not None:
            road_distance = self.get_road_distance()
            if road_distance > self.offroad_collision_threshold:
                is_colliding_offroad = True
                self.get_logger().warn(f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m")
        
        is_colliding = is_colliding_lidar or is_colliding_offroad
        
        if is_colliding:
            reward += self.reward_collision_penalty
            reward_info['penalty_collision'] = self.reward_collision_penalty
            # Log collision with detailed information
            if is_colliding_lidar:
                self.get_logger().warn(f"[COLLISION] LiDAR collision detected! min_distance={min_lidar_dist:.3f}m <= threshold={self.collision_threshold}m | Penalty: {self.reward_collision_penalty}")
            if is_colliding_offroad:
                self.get_logger().warn(f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m | Penalty: {self.reward_collision_penalty}")
            # Always log when collision penalty is applied
            self.get_logger().info(f"[REWARD] Collision penalty applied: {self.reward_collision_penalty} (lidar={is_colliding_lidar}, offroad={is_colliding_offroad})")
        else:
            reward_info['penalty_collision'] = 0.0
        
        # Add collision status to info for debugging - ALWAYS set these, even if False
        reward_info['is_collision'] = is_colliding
        reward_info['is_collision_lidar'] = is_colliding_lidar
        reward_info['is_collision_offroad'] = is_colliding_offroad
        reward_info['min_lidar_distance'] = min_lidar_dist
        reward_info['road_distance'] = road_distance
        
        # 5. Battery penalty (penalty for low battery)
        battery_level = self.battery.get_battery_level()
        battery_penalty = self.reward_battery_penalty_scale * (1.0 - battery_level)
        reward += battery_penalty
        reward_info['penalty_battery'] = battery_penalty
        
        # 6. Time penalty (small negative per step)
        reward += self.reward_time_penalty
        reward_info['penalty_time'] = self.reward_time_penalty
        
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
        
        # Terminate if goal reached
        if self.is_goal_reached():
            terminated = True
        
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

