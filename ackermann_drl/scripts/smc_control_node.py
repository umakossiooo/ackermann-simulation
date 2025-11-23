#!/usr/bin/env python3
"""Sliding Mode Control (SMC) node for Ackermann vehicle path following.

MUST RUN INSIDE DOCKER CONTAINER.

This node replaces the DRL agent with SMC controller:
- Subscribes to: /odom, /scan
- Publishes to: /cmd_vel
- Uses SMC to follow road polylines
- Handles obstacle avoidance using LiDAR
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from typing import Tuple
import numpy as np
import math
import sys
import os

# Add package path
sys.path.insert(0, '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl')

from ackermann_drl.utils.sliding_mode_control import SlidingModeController
from ackermann_drl.utils.roads_geometry import RoadsGeometry


class SMCControlNode(Node):
    """ROS 2 node implementing SMC path following control."""
    
    def __init__(self):
        super().__init__('smc_control_node')
        
        # Initialize SMC controller
        self.smc = SlidingModeController(
            lambda_param=1.0,      # Weight for heading error
            K_smc=2.0,            # Control gain
            boundary_layer=0.1,   # Boundary layer thickness
            max_steering=1.0,      # Max angular velocity (rad/s)
            desired_velocity=2.0   # Desired forward velocity (m/s)
        )
        
        # Initialize roads geometry
        try:
            self.roads_geometry = RoadsGeometry()
            self.get_logger().info(f"Loaded {self.roads_geometry.get_all_roads_count()} roads")
        except Exception as e:
            self.get_logger().error(f"Failed to load roads geometry: {e}")
            self.roads_geometry = None
        
        # State variables
        self.latest_odom = None
        self.latest_scan = None
        self.current_position = None
        self.current_heading = 0.0
        self.current_velocity = 0.0
        
        # Obstacle avoidance parameters
        self.obstacle_threshold = 1.5  # meters (stop if obstacle closer than this)
        self.slowdown_threshold = 3.0  # meters (slow down if obstacle closer than this)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Control timer (50 Hz = 20ms)
        self.control_timer = self.create_timer(0.02, self.control_callback)
        
        self.get_logger().info("SMC Control Node started")
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry updates."""
        self.latest_odom = msg
        
        # Extract position
        pos = msg.pose.pose.position
        self.current_position = (pos.x, pos.y)
        
        # Extract heading from quaternion
        orientation = msg.pose.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        # Convert quaternion to yaw (heading)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)
        
        # Extract velocity
        twist = msg.twist.twist
        self.current_velocity = math.sqrt(
            twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2
        )
    
    def scan_callback(self, msg: LaserScan):
        """Callback for LiDAR scan updates."""
        self.latest_scan = msg
    
    def check_obstacle_ahead(self) -> Tuple[bool, float]:
        """Check if there's an obstacle ahead using LiDAR.
        
        Returns:
            Tuple of (obstacle_detected, min_distance)
        """
        if self.latest_scan is None:
            return False, float('inf')
        
        ranges = np.array(self.latest_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) == 0:
            return False, float('inf')
        
        # Check front sector (roughly -45 to +45 degrees from heading)
        num_ranges = len(ranges)
        front_start = int(num_ranges * 0.375)  # -45 degrees
        front_end = int(num_ranges * 0.625)    # +45 degrees
        
        front_ranges = ranges[front_start:front_end]
        front_valid = front_ranges[np.isfinite(front_ranges)]
        
        if len(front_valid) == 0:
            return False, float('inf')
        
        min_distance = np.min(front_valid)
        obstacle_detected = min_distance < self.obstacle_threshold
        
        return obstacle_detected, min_distance
    
    def control_callback(self):
        """Main control callback - computes and publishes SMC commands."""
        if self.current_position is None or self.roads_geometry is None:
            return
        
        x, y = self.current_position
        
        # Check for obstacles
        obstacle_detected, min_distance = self.check_obstacle_ahead()
        
        if obstacle_detected:
            # Stop if obstacle too close
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m - stopping")
            return
        
        # Get road distance and heading
        road_distance, _ = self.roads_geometry.distance_to_nearest_road(x, y)
        desired_heading, nearest_polyline = self.roads_geometry.get_road_heading_at_point(x, y)
        
        # Compute heading error (normalize to [-π, π])
        heading_error = desired_heading - self.current_heading
        # Normalize to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Determine lateral error sign (which side of road we're on)
        # Use cross product: if we're to the right of the road direction, error is positive
        if nearest_polyline is not None:
            from shapely.geometry import Point as ShapelyPoint
            robot_point = ShapelyPoint(x, y)
            # Project point onto polyline
            try:
                projected = nearest_polyline.interpolate(nearest_polyline.project(robot_point))
                # Get road direction vector (tangent)
                road_dir_x = math.cos(desired_heading)
                road_dir_y = math.sin(desired_heading)
                # Vector from projected point to robot
                to_robot_x = x - projected.x
                to_robot_y = y - projected.y
                # Cross product: road_dir × to_robot
                # Positive = robot is to the right of road direction
                cross_product = road_dir_x * to_robot_y - road_dir_y * to_robot_x
                # Lateral error: positive = right of road, negative = left of road
                lateral_error = road_distance if cross_product >= 0 else -road_distance
            except Exception:
                # Fallback: use road distance without sign
                lateral_error = road_distance
        else:
            lateral_error = road_distance
        
        # Compute SMC control
        steering, velocity = self.smc.compute_control_with_velocity_adaptation(
            lateral_error=lateral_error,
            heading_error=heading_error,
            current_velocity=self.current_velocity
        )
        
        # Slow down if obstacle is close
        if min_distance < self.slowdown_threshold:
            velocity *= 0.5
        
        # Create and publish command
        cmd = Twist()
        cmd.linear.x = float(velocity)
        cmd.angular.z = float(steering)
        self.cmd_vel_pub.publish(cmd)
        
        # Log control info (throttled)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 50 == 0:  # Log every second (50 Hz * 20 = 1s)
            self.get_logger().info(
                f"SMC Control: lateral_error={lateral_error:.3f}m, "
                f"heading_error={math.degrees(heading_error):.1f}°, "
                f"steering={steering:.3f}, velocity={velocity:.2f}m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = SMCControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the vehicle
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

