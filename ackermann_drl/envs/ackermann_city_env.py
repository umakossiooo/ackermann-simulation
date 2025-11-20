"""Ackermann City Environment for DRL training.

This environment provides the interface between ROS 2 and the DRL agent
for training in the Bari city simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from typing import Tuple, Dict, Any, Optional


class AckermannCityEnv(Node):
    """ROS 2 environment for Ackermann vehicle DRL training.
    
    This class implements a Gymnasium-like interface for training DRL agents
    in the Gazebo simulation environment.
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
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State storage
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        
        self.get_logger().info("AckermannCityEnv initialized")
    
    def _scan_callback(self, msg: LaserScan):
        """Callback for laser scan messages."""
        self.latest_scan = msg
    
    def _odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        self.latest_odom = msg
    
    def reset(self) -> np.ndarray:
        """Reset the environment and return initial observation.
        
        Returns:
            Initial observation array
        """
        self.get_logger().info("Resetting environment")
        # TODO: Implement reset logic (spawn robot, reset state)
        return np.zeros(720)  # Placeholder: 720 scan samples
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """Execute one step in the environment.
        
        Args:
            action: Action array [linear_velocity, angular_velocity]
            
        Returns:
            Tuple of (observation, reward, done, info)
        """
        # Publish control command
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd)
        
        # TODO: Get observation, compute reward, check done condition
        observation = np.zeros(720)  # Placeholder
        reward = 0.0
        done = False
        info = {}
        
        return observation, reward, done, info
    
    def get_observation(self) -> np.ndarray:
        """Get current observation from sensors.
        
        Returns:
            Observation array (laser scan data)
        """
        if self.latest_scan is None:
            return np.zeros(720)
        
        # Convert scan ranges to numpy array
        ranges = np.array(self.latest_scan.ranges)
        # Replace inf/nan with max range
        ranges = np.nan_to_num(ranges, nan=self.latest_scan.range_max, 
                              posinf=self.latest_scan.range_max,
                              neginf=self.latest_scan.range_max)
        return ranges

