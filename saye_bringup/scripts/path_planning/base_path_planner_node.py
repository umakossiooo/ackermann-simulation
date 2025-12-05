#!/usr/bin/env python3
"""
Base Path Planner Node for ROS 2

Contains all common ROS node functionality shared between Dijkstra and A* nodes.
"""

import math
import numpy as np
from pathlib import Path
from typing import Optional, Tuple
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from base_path_planner import BasePathPlanner


# Global variable for current pose
current_pose = None


class BasePathPlannerNode(Node):
    """Base ROS 2 node for path planning with common control logic."""
    
    def __init__(self, node_name: str, planner: BasePathPlanner):
        """Initialize the path planner node.
        
        Args:
            node_name: Name for the ROS node
            planner: Path planner instance (Dijkstra or A*)
        """
        super().__init__(node_name)
        
        # Store planner
        self.planner = planner
        
        # Path following parameters
        self.lookahead_distance = 2.5  # Reduced for tighter path following
        self.max_velocity = 1.5  # Reduced for better control
        self.min_velocity = 0.5
        self.waypoint_tolerance = 0.1  # Strict: car must be within 10cm of goal
        self.max_off_road_distance = 2.0  # Maximum allowed distance from road (meters)
        self.road_check_interval = 3  # Check road distance every N control cycles (more frequent)
        self.road_check_counter = 0
        
        # Path state
        self.path = None
        self.current_waypoint_idx = 0
        self.goal_reached = False
        self.path_following = False
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"{node_name} initialized")
    
    def odom_callback(self, msg):
        """Callback to update current position."""
        global current_pose
        if current_pose is None:
            self.get_logger().info("Received first odometry message!")
        current_pose = msg
    
    def get_current_position(self):
        """Get current position from global current_pose."""
        global current_pose
        if current_pose is None:
            return None, None, None
        
        pos = current_pose.pose.pose.position
        x = pos.x  # east
        y = pos.y  # north
        
        q = current_pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        return x, y, yaw
    
    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """Plan path using the planner's search algorithm."""
        self.get_logger().info(f"Planning path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
        
        path, metrics = self.planner.search((start_x, start_y), (goal_x, goal_y))
        
        if path is None:
            self.get_logger().error("No path found!")
            return False
        
        self.path = path
        self.current_waypoint_idx = 0
        self.goal_reached = False
        self.path_following = True
        
        # Log metrics
        self.get_logger().info(f"Path found! Length: {metrics['path_cost']:.2f}m, Waypoints: {len(path)}")
        self.get_logger().info(f"Metrics: Time={metrics['computation_time']*1000:.2f}ms, "
                              f"Nodes={metrics['nodes_expanded']}, Memory={metrics['memory_usage']:.2f}MB")
        return True
    
    def update_waypoint(self, current_x, current_y):
        """Check if waypoint reached - advance to next waypoint if close enough."""
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return
        
        # Check if we've passed the current waypoint or are close to it
        waypoint = self.path[self.current_waypoint_idx]
        distance = self.planner._euclidean_distance((current_x, current_y), waypoint)
        
        # Also check if we're past the waypoint (projected along path direction)
        if self.current_waypoint_idx < len(self.path) - 1:
            next_wp = self.path[self.current_waypoint_idx + 1]
            # Vector from current waypoint to next
            path_dx = next_wp[0] - waypoint[0]
            path_dy = next_wp[1] - waypoint[1]
            path_len = math.sqrt(path_dx * path_dx + path_dy * path_dy)
            
            if path_len > 0.01:
                # Vector from waypoint to current position
                to_curr_dx = current_x - waypoint[0]
                to_curr_dy = current_y - waypoint[1]
                # Project onto path direction
                proj = (to_curr_dx * path_dx + to_curr_dy * path_dy) / path_len
                # If projection is positive and significant, we've passed the waypoint
                passed_waypoint = proj > 0.5  # 0.5m past waypoint
            else:
                passed_waypoint = False
        else:
            passed_waypoint = False
        
        if distance < self.waypoint_tolerance or passed_waypoint:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.path):
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx-1}/{len(self.path)-1}")
            else:
                self.goal_reached = True
                self.path_following = False
                self.get_logger().info("Goal reached!")
                self.stop_vehicle()
    
    def compute_control(self, current_x, current_y, current_yaw, target_x, target_y):
        """Compute velocity and steering commands."""
        distance = self.planner._euclidean_distance((current_x, current_y), (target_x, target_y))
        
        desired_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = self._normalize_angle(desired_yaw - current_yaw)
        
        # Velocity: reduce as approaching waypoint (more aggressive for precision)
        if distance < 1.0:
            velocity = self.min_velocity + (distance / 1.0) * (self.max_velocity - self.min_velocity)
        else:
            velocity = self.max_velocity
        
        # Steering: proportional control
        max_steering = 1.0  # rad/s
        steering_gain = 2.0
        steering = steering_gain * yaw_error
        steering = np.clip(steering, -max_steering, max_steering)
        
        # Reduce velocity for large steering (safety)
        if abs(steering) > 0.5:
            velocity *= 0.7
        
        return velocity, steering
    
    def stop_vehicle(self):
        """Stop the vehicle."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _compute_road_correction(self, current_x: float, current_y: float, current_yaw: float) -> Tuple[float, Tuple[float, float]]:
        """Compute road correction yaw error and road point.
        
        Args:
            current_x: Current east coordinate
            current_y: Current north coordinate
            current_yaw: Current yaw angle
            
        Returns:
            Tuple of (road_yaw_error, road_point) for correction
        """
        road_point = self.planner.project_to_road(current_x, current_y)
        road_dx = road_point[0] - current_x
        road_dy = road_point[1] - current_y
        road_heading = math.atan2(road_dy, road_dx)
        road_yaw_error = self._normalize_angle(road_heading - current_yaw)
        return road_yaw_error, road_point
    
    def _get_lookahead_point(self, current_x: float, current_y: float, lookahead_dist: float) -> Optional[Tuple[float, float]]:
        """Get lookahead point along the path at specified distance ahead from current position.
        
        This ensures the car follows the path smoothly rather than cutting corners.
        """
        if self.path is None or len(self.path) == 0:
            return None
        
        # Find closest point on path to current position
        min_dist_to_path = float('inf')
        closest_segment_idx = 0
        closest_t = 0.0
        
        # Check all path segments
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            
            # Project current position onto this segment
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg
            
            if seg_len_sq < 0.001:  # Very short segment
                # Just use distance to p1
                dx = current_x - p1[0]
                dy = current_y - p1[1]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < min_dist_to_path:
                    min_dist_to_path = dist
                    closest_segment_idx = i
                    closest_t = 0.0
                continue
            
            # Project current point onto segment
            dx_curr = current_x - p1[0]
            dy_curr = current_y - p1[1]
            t = (dx_curr * dx_seg + dy_curr * dy_seg) / seg_len_sq
            t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
            
            # Point on segment
            proj_x = p1[0] + t * dx_seg
            proj_y = p1[1] + t * dy_seg
            
            # Distance to projected point
            dx = current_x - proj_x
            dy = current_y - proj_y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_segment_idx = i
                closest_t = t
        
        # Now look ahead from the closest point on path
        accumulated_dist = 0.0
        start_idx = closest_segment_idx
        
        # Start from the projection point on the closest segment
        if closest_t < 1.0:
            # We're on a segment, start from projected point
            p1 = self.path[start_idx]
            p2 = self.path[start_idx + 1]
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len = math.sqrt(dx_seg * dx_seg + dy_seg * dy_seg)
            
            # Distance remaining on current segment
            remaining_on_seg = (1.0 - closest_t) * seg_len
            if remaining_on_seg >= lookahead_dist:
                # Lookahead is on current segment
                t = closest_t + (lookahead_dist / seg_len) if seg_len > 0 else 1.0
                lookahead_x = p1[0] + t * dx_seg
                lookahead_y = p1[1] + t * dy_seg
                return (lookahead_x, lookahead_y)
            
            accumulated_dist = remaining_on_seg
            start_idx += 1
        
        # Continue from next segments
        for i in range(start_idx, len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            segment_length = math.sqrt(dx * dx + dy * dy)
            
            if accumulated_dist + segment_length >= lookahead_dist:
                # Lookahead point is on this segment
                remaining = lookahead_dist - accumulated_dist
                t = remaining / segment_length if segment_length > 0 else 0.0
                lookahead_x = p1[0] + t * dx
                lookahead_y = p1[1] + t * dy
                return (lookahead_x, lookahead_y)
            
            accumulated_dist += segment_length
        
        # If we've reached the end, return the last waypoint
        return self.path[-1]
    
    def control_loop(self):
        """Main control loop - runs at 10 Hz. Ensures car stays on roads."""
        if not self.path_following or self.goal_reached:
            return
        
        current_x, current_y, current_yaw = self.get_current_position()
        if current_x is None or self.path is None or self.current_waypoint_idx >= len(self.path):
            return
        
        # Check if car is too far from road (every N cycles for performance)
        self.road_check_counter += 1
        if self.road_check_counter >= self.road_check_interval:
            self.road_check_counter = 0
            road_distance = self.planner.distance_to_nearest_road(current_x, current_y)
            
            if road_distance > self.max_off_road_distance:
                # Car is too far from road - project back to road and adjust target
                self.get_logger().warn(
                    f"Car is {road_distance:.2f}m off-road! Projecting back to road..."
                )
                projected_x, projected_y = self.planner.project_to_road(current_x, current_y)
                # Use projected position for control (forces car back to road)
                current_x, current_y = projected_x, projected_y
            elif road_distance > 0.8:  # Lower threshold for earlier correction
                # Car is getting off-road - reduce speed and correct
                self.get_logger().info(
                    f"Car is {road_distance:.2f}m from road - correcting..."
                )
        
        self.update_waypoint(current_x, current_y)
        
        if self.goal_reached:
            return
        
        # Use lookahead point instead of direct waypoint (prevents cutting corners)
        target = self._get_lookahead_point(current_x, current_y, self.lookahead_distance)
        
        if target is None:
            # Fallback to current waypoint
            if self.current_waypoint_idx < len(self.path):
                target = self.path[self.current_waypoint_idx]
            else:
                return
        
        # Check road distance BEFORE computing control
        road_distance = self.planner.distance_to_nearest_road(current_x, current_y)
        
        # If significantly off-road (>1.5m), prioritize getting back to road
        if road_distance > 1.5:
            # Get nearest road point and steer directly toward it
            road_yaw_error, _ = self._compute_road_correction(current_x, current_y, current_yaw)
            
            # Strong correction steering toward road
            correction_gain = 3.0
            steering = correction_gain * road_yaw_error
            steering = np.clip(steering, -1.0, 1.0)
            
            # Moderate speed - don't stop completely
            velocity = self.min_velocity * 1.2  # Slightly faster to get back on road
        elif road_distance > 0.8:
            # Slightly off-road - blend road correction with path following
            road_yaw_error, _ = self._compute_road_correction(current_x, current_y, current_yaw)
            
            # Normal path following
            velocity, path_steering = self.compute_control(current_x, current_y, current_yaw, target[0], target[1])
            
            # Blend: 70% path following, 30% road correction
            correction_weight = 0.3 * (road_distance / 1.5)
            steering = (1.0 - correction_weight) * path_steering + correction_weight * 2.0 * road_yaw_error
            steering = np.clip(steering, -1.0, 1.0)
            
            # Slight speed reduction
            velocity *= (1.0 - 0.15 * (road_distance / 1.5))
        else:
            # On road - normal path following
            # Ensure target is on road (safety check)
            target_road_dist = self.planner.distance_to_nearest_road(target[0], target[1])
            if target_road_dist > 1.0:
                # Target is off-road, project it to nearest road
                target = self.planner.project_to_road(target[0], target[1])
            
            velocity, steering = self.compute_control(current_x, current_y, current_yaw, target[0], target[1])
        
        cmd = Twist()
        cmd.linear.x = float(velocity)
        cmd.angular.z = float(steering)
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress periodically (every 2 seconds at 10 Hz = every 20 iterations)
        if self.current_waypoint_idx % 20 == 0 or len(self.path) < 20:
            self.get_logger().info(
                f"Following path: waypoint {self.current_waypoint_idx}/{len(self.path)-1}, "
                f"vel={velocity:.2f}m/s, steering={math.degrees(steering):.1f}°, "
                f"road_dist={road_distance:.2f}m, target=({target[0]:.2f}, {target[1]:.2f})"
            )

