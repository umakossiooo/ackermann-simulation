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
        
        # Executor for spinning (minimal - single thread)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self)
        
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
        self.executor.spin_once(timeout_sec=timeout_sec)
    
    def reset(self) -> np.ndarray:
        """Reset the environment and return initial observation.
        
        Selects a new random goal (delivery point) on reset.
        
        Returns:
            Initial observation array (720 scan samples + 1 battery level + 3 goal deltas = 724 total)
        """
        self.get_logger().info("Resetting environment")
        # Reset state storage
        self.latest_scan = None
        self.latest_odom = None
        self.scan_received = False
        self.odom_received = False
        
        # Reset battery
        self.battery.reset()
        
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
        for _ in range(5):
            self.spin_once(timeout_sec=0.01)
        
        # Return observation: 720 scan + 1 battery + 3 goal deltas (Δx, Δy, Δθ) = 724
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
        self.cmd_vel_pub.publish(cmd)
        
        # Spin briefly to process any incoming messages
        for _ in range(3):
            self.spin_once(timeout_sec=0.01)
        
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
        
        # Get observation from sensors (includes battery level)
        observation = self.get_observation()
        
        # Minimal implementation: return zero reward, not terminated, not truncated
        reward = 0.0
        terminated = False
        truncated = False
        info = {
            'scan_received': self.scan_received,
            'odom_received': self.odom_received,
            'battery_level': self.battery.get_battery_level(),
            'battery_depleted': self.battery.is_depleted(),
            'current_goal': self.current_goal.get('name') if self.current_goal else None
        }
        
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
    
    def get_observation(self) -> np.ndarray:
        """Get current observation from sensors.
        
        Returns:
            Observation array: 720 laser scan + 1 battery + 3 goal deltas (Δx, Δy, Δθ) = 724 total
        """
        # Initialize observation array (720 scan + 1 battery + 3 goal deltas = 724)
        obs = np.zeros(724, dtype=np.float32)
        
        # Fill laser scan data (indices 0-719)
        if self.latest_scan is not None:
            ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
            # Replace inf/nan with max range
            ranges = np.nan_to_num(
                ranges, 
                nan=self.latest_scan.range_max, 
                posinf=self.latest_scan.range_max,
                neginf=self.latest_scan.range_max
            )
            obs[:720] = ranges
        
        # Add battery level (index 720)
        obs[720] = self.battery.get_battery_level()
        
        # Add goal deltas (indices 721-723: Δx, Δy, Δθ)
        dx, dy, dtheta = self.compute_goal_deltas()
        obs[721] = dx
        obs[722] = dy
        obs[723] = dtheta
        
        return obs

