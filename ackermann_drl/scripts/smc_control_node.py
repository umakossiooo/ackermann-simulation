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
import time

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
        
        # Metrics tracking (for comparison with DRL)
        self.metrics = {
            'total_steps': 0,
            'collision_count': 0,
            'offroad_count': 0,
            'offroad_distance_sum': 0.0,
            'total_reward': 0.0,
            'goal_reached': False,
            'episode_start_time': time.time(),
        }
        
        # Reward parameters (same as DRL environment for comparison)
        self.reward_progress_scale = 1.0
        self.reward_goal_reached = 50.0
        self.reward_offroad_penalty = -0.05
        self.reward_collision_penalty = -20.0
        self.reward_time_penalty = -0.01
        self.goal_reached_threshold = 2.0
        self.collision_threshold = 1.5  # Same as DRL environment
        self.offroad_collision_threshold = 1.0  # Same as DRL environment
        
        # Goal tracking (for delivery points)
        try:
            from ackermann_drl.utils.delivery_points import DeliveryPoints
            self.delivery_points = DeliveryPoints()
            self.current_goal = None
            self.prev_distance_to_goal = None
        except Exception as e:
            self.get_logger().warn(f"Could not load delivery points: {e}")
            self.delivery_points = None
            self.current_goal = None
        
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
        
        # Check collisions and off-road (same thresholds as DRL environment)
        is_collision = False
        is_offroad_collision = False
        
        # Check LiDAR collision
        if self.latest_scan is not None:
            ranges = np.array(self.latest_scan.ranges)
            valid_ranges = ranges[np.isfinite(ranges)]
            if len(valid_ranges) > 0:
                min_lidar_dist = np.min(valid_ranges)
                if min_lidar_dist <= self.collision_threshold:
                    is_collision = True
        
        # Check off-road collision
        if road_distance > self.offroad_collision_threshold:
            is_offroad_collision = True
            is_collision = True
        
        # Track metrics (for comparison with DRL)
        self.metrics['total_steps'] += 1
        if is_collision:
            self.metrics['collision_count'] += 1
        if road_distance > 0.0:
            self.metrics['offroad_count'] += 1
            self.metrics['offroad_distance_sum'] += road_distance
        
        # Compute reward (same as DRL environment for comparison)
        reward = self._compute_reward(road_distance, is_collision, min_distance)
        self.metrics['total_reward'] += reward
        
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
                f"steering={steering:.3f}, velocity={velocity:.2f}m/s, "
                f"road_dist={road_distance:.2f}m, reward={reward:.3f}"
            )
            
            # Log metrics summary
            if self._log_counter % 500 == 0:  # Every 10 seconds
                avg_reward = self.metrics['total_reward'] / max(1, self.metrics['total_steps'])
                self.get_logger().info(
                    f"SMC Metrics: steps={self.metrics['total_steps']}, "
                    f"collisions={self.metrics['collision_count']}, "
                    f"offroad_steps={self.metrics['offroad_count']}, "
                    f"avg_reward={avg_reward:.4f}"
                )
    
    def _compute_reward(self, road_distance: float, is_collision: bool, min_obstacle_distance: float) -> float:
        """Compute reward using same formula as DRL environment for comparison.
        
        Args:
            road_distance: Distance from road (meters)
            is_collision: Whether collision detected
            min_obstacle_distance: Minimum LiDAR distance (meters)
        
        Returns:
            Reward value (same scale as DRL environment)
        """
        reward = 0.0
        
        # Progress reward (simplified - would need goal tracking for full implementation)
        # For now, reward staying on road
        if road_distance < 0.5:
            reward += 0.1  # Small reward for staying close to road
        
        # Off-road penalty (same as DRL)
        if road_distance > 0.0:
            reward += self.reward_offroad_penalty * road_distance
        
        # Collision penalty (same as DRL)
        if is_collision:
            reward += self.reward_collision_penalty
        
        # Time penalty (same as DRL)
        reward += self.reward_time_penalty
        
        return reward


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

