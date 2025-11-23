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
        
        # State storage
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.scan_received = False
        self.odom_received = False
        # Track timestamps to detect stale data
        self.last_scan_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None
        self.scan_count = 0
        self.odom_count = 0
        # Track counts before reset to detect new data after reset
        self.scan_count_before_reset = 0
        self.odom_count_before_reset = 0
        
        # Battery model
        self.battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
        
        # Delivery points (goals)
        self.delivery_points = DeliveryPoints()
        self.current_goal: Optional[Dict] = None
        
        # Roads geometry (for road distance calculation)
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
        
        # Observation parameters
        self.lidar_downsample_factor = 4  # Downsample 720 to 180 samples
        self.lidar_downsampled_size = 720 // self.lidar_downsample_factor  # 180
        
        # Reward parameters
        # Increased progress reward to guide agent better toward goals
        self.reward_progress_scale = 1.0  # Increased from 0.1 to give stronger guidance toward goal
        self.reward_goal_reached = 50.0  # Increased from 10.0 to make goal more attractive
        self.reward_offroad_penalty = -0.05  # Reduced from -0.1 to be less harsh (gentle guidance back to road)
        self.reward_collision_penalty = -20.0  # Increased from -10.0 to strongly discourage collisions
        self.reward_battery_penalty_scale = -0.5  # Penalty scale for low battery
        self.reward_time_penalty = -0.01  # Small penalty per step
        
        # Goal reached threshold (meters)
        self.goal_reached_threshold = 2.0  # Consider goal reached if within 2m
        
        # Collision detection threshold (meters)
        # Note: LiDAR might not detect low obstacles (sidewalks) perfectly, so we use a more aggressive threshold
        # Also check road distance as a proxy for collision with curbs/sidewalks
        # Adjusted for narrow streets - only trigger on very close obstacles
        self.collision_threshold = 0.3  # Consider collision if obstacle within 0.3m (reduced from 0.5m for narrow streets)
        self.offroad_collision_threshold = 1.0  # If off-road by more than 1.0m, consider it a collision with sidewalk/curb (increased to avoid false positives on narrow streets)
        
        # Track previous distance for progress calculation
        self.prev_distance_to_goal: Optional[float] = None
        
        # Delivery deadline tracking
        self.delivery_start_time: Optional[float] = None  # When delivery started (reset time)
        self.delivery_deadline: Optional[float] = None  # Deadline in seconds from start
        self.delivery_elapsed_time: float = 0.0  # Time elapsed since delivery started
        self.delivery_on_time: bool = False  # Whether delivery was completed on time
        self.reward_delivery_on_time = 30.0  # Reward for delivering on time
        self.reward_delivery_late_penalty = -10.0  # Penalty per second late (if late)
        
        # Step and episode counters for tracking
        self.step_count = 0
        self.episode_count = 0
        self.episode_step_count = 0  # Steps within current episode (resets on reset())
        
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
        # DON'T reset latest_scan and latest_odom to None - keep stale data if available
        # This allows rewards to be computed even if new data isn't received immediately
        # Only reset the flags, not the data itself
        self.scan_received = False
        self.odom_received = False
        # Keep last timestamps for staleness detection - don't reset them
        # Reset counters to track new data reception after reset
        # Store current counts before reset to detect new data
        self.scan_count_before_reset = self.scan_count
        self.odom_count_before_reset = self.odom_count
        
        # Reset battery
        self.battery.reset()
        
        # Reset previous distance tracking
        self.prev_distance_to_goal = None
        
        # Reset episode step counter
        self.step_count = 0
        
        # Reset delivery deadline tracking
        self.delivery_start_time = time.time()
        self.delivery_elapsed_time = 0.0
        self.delivery_on_time = False
        
        # Select new goal - start from first point on reset (single delivery)
        try:
            self.current_goal = self.delivery_points.get_next_point(None)  # Start from first point
            goal_pos = self.delivery_points.get_point_position(self.current_goal)
            
            # Get delivery deadline from goal config (default 120 seconds if not specified)
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
        
        # Spin to get initial sensor data - wait longer and be more aggressive
        # Check if we received NEW data (count increased from before reset)
        max_wait_iterations = 200  # Wait up to 10 seconds for initial data
        
        for i in range(max_wait_iterations):
            self.spin_once(timeout_sec=0.05)
            # Break if we received NEW data from both sensors (count increased from before reset)
            scan_new = self.scan_count > self.scan_count_before_reset
            odom_new = self.odom_count > self.odom_count_before_reset
            if scan_new and odom_new:
                safe_log(self.get_logger().info, f"[RESET] Received initial sensor data after {i+1} spins (scan={self.scan_count}, odom={self.odom_count})")
                break
        
        # Log if we didn't receive initial data
        if self.scan_count <= self.scan_count_before_reset:
            safe_log(self.get_logger().warn, f"[RESET] No initial LiDAR data received (count={self.scan_count}, before={self.scan_count_before_reset})")
        if self.odom_count <= self.odom_count_before_reset:
            safe_log(self.get_logger().warn, f"[RESET] No initial odometry data received (count={self.odom_count}, before={self.odom_count_before_reset})")
        
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
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        
        # Check if ROS context is valid before publishing
        # If context is invalid, skip publishing but continue with step execution
        context_valid = rclpy.ok()
        if context_valid and self.cmd_vel_pub is not None:
            try:
                # Try to publish
                self.cmd_vel_pub.publish(cmd)
            except Exception as e:
                # If publishing fails, continue anyway - don't fail the step
                # Most errors here are due to invalid context
                if context_valid and "context" not in str(e).lower():
                    try:
                        self.get_logger().error(f"Failed to publish command: {e}")
                    except:
                        pass  # If logging fails, context is definitely invalid
                # Don't terminate - just continue without publishing
                # The car will stop, but we can still compute rewards from sensor data
        
        # Increment step count
        self.step_count += 1
        
        # Spin briefly to process any incoming messages
        # Track message counts before spinning to detect new data
        scan_count_before = self.scan_count
        odom_count_before = self.odom_count
        
        start_time = time.time()
        max_wait_time = 2.0  # Increased to 2 seconds to wait longer for sensor data
        min_spins = 20  # Minimum number of spins to ensure callbacks are processed
        
        # Spin to get fresh sensor data - be more aggressive about getting data
        spin_iterations = 0
        while (time.time() - start_time) < max_wait_time or spin_iterations < min_spins:
            self.spin_once(timeout_sec=0.05)  # Shorter timeout per spin, but more spins
            spin_iterations += 1
            # Break early if we received new data from both sensors AND we've done minimum spins
            if spin_iterations >= min_spins and (self.scan_count > scan_count_before and self.odom_count > odom_count_before):
                break
        
        # Log if we didn't get new data (but don't fail - use stale data if available)
        # Use safe_log to prevent ROS context errors
        if self.scan_count == scan_count_before:
            safe_log(self.get_logger().warn, f"[SENSOR] No new LiDAR data received after {spin_iterations} spins (count={self.scan_count})")
        if self.odom_count == odom_count_before:
            safe_log(self.get_logger().warn, f"[SENSOR] No new odometry data received after {spin_iterations} spins (count={self.odom_count})")
        
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
        
        # Increment episode step counter BEFORE computing reward (so logs show correct step)
        self.episode_step_count += 1
        
        # Compute reward FIRST while sensor data is still available
        # (get_observation() might access sensor data, so compute reward before it)
        reward, reward_info = self.compute_reward()
        
        # Check if goal reached and move to next delivery point
        if self.is_goal_reached() and self.current_goal is not None:
            # Get next delivery point in sequence
            next_goal = self.delivery_points.get_next_point(self.current_goal)
            if next_goal is not None:
                # Move to next goal (don't terminate - continue episode)
                self.current_goal = next_goal
                goal_pos = self.delivery_points.get_point_position(self.current_goal)
                safe_log(self.get_logger().info,
                    f"Goal reached! Moving to next goal: {self.current_goal.get('name', 'unknown')} "
                    f"at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})"
                )
                # Reset previous distance tracking for new goal
                self.prev_distance_to_goal = None
        
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
        
        # Get velocity and distance to goal for logging
        velocity = -1.0
        distance_to_goal = -1.0
        if self.latest_odom is not None:
            velocity, _ = self.get_velocity_and_steering()
            distance_to_goal = self.get_distance_to_goal()
        
        # Check if data is fresh (received within last 2 seconds)
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
            **reward_info  # Add reward breakdown (includes penalty_collision, is_collision, road_distance, min_lidar_distance, etc.)
        }
        
        
        # Log warnings if data is stale (use safe_log to prevent ROS context errors)
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
        if self.roads_geometry is None:
            # If roads geometry not loaded, return 0.0 (assume on road)
            # This prevents false positives but means off-road detection won't work
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
            'reward_delivery_on_time': 0.0,
            'penalty_delivery_late': 0.0,
            'penalty_offroad': 0.0,
            'penalty_collision': 0.0,
            'penalty_battery': 0.0,
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
                if abs(progress_reward) > 0.0001:
                    print(f"[REWARD] Step {self.episode_step_count} | Progress: {progress_reward:+.4f} (moved {progress:+.2f}m closer, dist={current_distance:.2f}m)")
            else:
                reward_info['reward_progress'] = 0.0
            
            self.prev_distance_to_goal = current_distance
            if self.is_goal_reached():
                reward += self.reward_goal_reached
                reward_info['reward_goal'] = self.reward_goal_reached
                print(f"[REWARD] Step {self.episode_step_count} | GOAL REACHED! Reward: +{self.reward_goal_reached:.4f}")
                
                if self.delivery_deadline is not None and self.delivery_elapsed_time <= self.delivery_deadline:
                    reward += self.reward_delivery_on_time
                    reward_info['reward_delivery_on_time'] = self.reward_delivery_on_time
                    self.delivery_on_time = True
                    reward_info['delivery_on_time'] = True
                    print(f"[REWARD] Step {self.episode_step_count} | ON-TIME DELIVERY! Elapsed: {self.delivery_elapsed_time:.1f}s / Deadline: {self.delivery_deadline:.1f}s | Reward: +{self.reward_delivery_on_time:.4f}")
                    safe_log(self.get_logger().info, 
                        f"[DELIVERY] On-time delivery! Elapsed: {self.delivery_elapsed_time:.1f}s / Deadline: {self.delivery_deadline:.1f}s | Reward: +{self.reward_delivery_on_time}")
                elif self.delivery_deadline is not None:
                    seconds_late = self.delivery_elapsed_time - self.delivery_deadline
                    late_penalty = self.reward_delivery_late_penalty * seconds_late
                    reward += late_penalty
                    reward_info['penalty_delivery_late'] = late_penalty
                    self.delivery_on_time = False
                    reward_info['delivery_on_time'] = False
                    print(f"[REWARD] Step {self.episode_step_count} | LATE DELIVERY! Elapsed: {self.delivery_elapsed_time:.1f}s / Deadline: {self.delivery_deadline:.1f}s | Late by: {seconds_late:.1f}s | Penalty: {late_penalty:.4f}")
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
                    print(f"[REWARD] Step {self.episode_step_count} | Running late: {seconds_late:.1f}s past deadline | Penalty: {late_penalty:.4f}")
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
                if abs(offroad_penalty) > 0.0001:
                    print(f"[REWARD] Step {self.episode_step_count} | Off-road: distance={road_distance:.2f}m | Penalty: {offroad_penalty:.4f}")
            else:
                reward_info['penalty_offroad'] = 0.0
        else:
            reward_info['penalty_offroad'] = 0.0
        
        is_colliding_lidar = False
        is_colliding_offroad = False
        min_lidar_dist = -1.0
        road_distance = -1.0
        
        if self.latest_scan is not None:
            is_colliding_lidar = self.is_collision()
            try:
                ranges = np.array(self.latest_scan.ranges)
                valid_ranges = ranges[np.isfinite(ranges)]
                if len(valid_ranges) > 0:
                    min_lidar_dist = float(np.min(valid_ranges))
            except:
                pass
        
        if self.latest_odom is not None:
            road_distance = self.get_road_distance()
            if road_distance > self.offroad_collision_threshold:
                is_colliding_offroad = True
                safe_log(self.get_logger().warn, f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m")
        
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
            print(f"\n[REWARD] Step {self.episode_step_count} | COLLISION DETECTED! Type: {', '.join(collision_types)} | Penalty: {self.reward_collision_penalty:.4f}\n")
            if is_colliding_lidar:
                safe_log(self.get_logger().warn, f"[COLLISION] LiDAR collision detected! min_distance={min_lidar_dist:.3f}m <= threshold={self.collision_threshold}m | Penalty: {self.reward_collision_penalty}")
            if is_colliding_offroad:
                safe_log(self.get_logger().warn, f"[COLLISION] Off-road collision detected! road_distance={road_distance:.3f}m > threshold={self.offroad_collision_threshold}m | Penalty: {self.reward_collision_penalty}")
            safe_log(self.get_logger().info, f"[REWARD] Collision penalty applied: {self.reward_collision_penalty} (lidar={is_colliding_lidar}, offroad={is_colliding_offroad})")
        else:
            reward_info['penalty_collision'] = 0.0
        
        
        # 5. Battery penalty (penalty for low battery)
        battery_level = self.battery.get_battery_level()
        battery_penalty = self.reward_battery_penalty_scale * (1.0 - battery_level)
        reward += battery_penalty
        reward_info['penalty_battery'] = battery_penalty
        if abs(battery_penalty) > 0.0001:
            print(f"[REWARD] Step {self.episode_step_count} | Battery: level={battery_level:.3f} | Penalty: {battery_penalty:.4f}")
        
        reward += self.reward_time_penalty
        reward_info['penalty_time'] = self.reward_time_penalty
        print(f"[REWARD] Step {self.episode_step_count} | Time penalty: {self.reward_time_penalty:.4f} (per step)")
        
        print(f"[REWARD] Step {self.episode_step_count} | TOTAL REWARD: {reward:+.4f}")
        
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

