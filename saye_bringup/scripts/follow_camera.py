#!/usr/bin/env python3
"""Gazebo camera follower - updates camera pose to follow the robot."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import math
import time
import os


class FollowCamera(Node):
    """Updates Gazebo GUI camera to follow the robot."""
    
    def __init__(self):
        super().__init__('follow_camera')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.last_update_time = 0.0
        self.update_interval = 1.0  # Update camera every 1 second (reduced frequency to minimize service calls)
        self.startup_delay = 10.0  # Wait 10 seconds after node start before making service calls (ensures Gazebo GUI is fully ready)
        self.node_start_time = time.time()  # Track when node started
        self.camera_offset_x = -5.0  # 5m behind the car
        self.camera_offset_y = 0.0
        self.camera_offset_z = 3.0   # 3m above ground
        self.pitch = 0.4  # Look down at the car
        self.last_camera_pos = None  # Track last camera position
        self.position_threshold = 0.5  # Only update if robot moved more than 0.5m
        self.service_available = True  # Track if service is available
        self.get_logger().info('Follow camera node started - camera will follow robot')
    
    def odom_callback(self, msg):
        """Update camera position based on robot odometry."""
        # Wait for startup delay before making any service calls
        if time.time() - self.node_start_time < self.startup_delay:
            return
        
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return
        
        self.last_update_time = current_time
        
        # Get robot position
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Extract yaw from quaternion
        yaw = math.atan2(
            2.0 * (orient.w * orient.z + orient.x * orient.y),
            1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        )
        
        # Calculate camera position (behind and above the robot)
        camera_x = pos.x + self.camera_offset_x * math.cos(yaw) - self.camera_offset_y * math.sin(yaw)
        camera_y = pos.y + self.camera_offset_x * math.sin(yaw) + self.camera_offset_y * math.cos(yaw)
        camera_z = pos.z + self.camera_offset_z
        
        # Only update if robot has moved significantly (reduces service calls)
        if self.last_camera_pos is not None:
            dx = camera_x - self.last_camera_pos[0]
            dy = camera_y - self.last_camera_pos[1]
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < self.position_threshold:
                return  # Robot hasn't moved enough, skip update
        
        self.last_camera_pos = (camera_x, camera_y, camera_z)
        
        # Calculate camera orientation to look at the car
        # Camera yaw should be opposite to robot yaw (looking at back of car)
        camera_yaw = yaw + math.pi
        
        # Convert to quaternion (roll=0, pitch=self.pitch, yaw=camera_yaw)
        cy = math.cos(camera_yaw * 0.5)
        sy = math.sin(camera_yaw * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)
        
        qx = cy * sp
        qy = sy * sp
        qz = -sy * cp
        qw = cy * cp
        
        # Update Gazebo camera pose (non-blocking to avoid "Host unreachable" errors)
        # Skip if service is not available
        if not self.service_available:
            return
            
        try:
            # Build pose string with proper escaping for nested braces
            pose_str = (
                f'pose: {{position: {{x: {camera_x:.3f}, y: {camera_y:.3f}, z: {camera_z:.3f}}}, '
                f'orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}}}'
            )
            cmd = [
                'gz', 'service',
                '-s', '/gui/move_to/pose',
                '--reqtype', 'gz.msgs.GUICamera',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '50',
                '--req', pose_str
            ]
            
            # Use Popen with completely detached process
            # Redirect all output to /dev/null to suppress errors
            # Process is fully detached with start_new_session=True
            with open(os.devnull, 'w') as devnull:
                subprocess.Popen(
                    cmd,
                    stdout=devnull,
                    stderr=devnull,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True  # Create new process group to detach completely
                )
                # Process is detached and will complete on its own
                    
        except Exception as e:
            # If service fails repeatedly, disable it
            self.service_available = False
            self.get_logger().warn(f'Camera service unavailable, disabling follow camera: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
