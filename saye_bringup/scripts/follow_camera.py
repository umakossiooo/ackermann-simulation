#!/usr/bin/env python3
"""Gazebo camera follower - updates camera pose to follow the robot."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import math
import time


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
        self.update_interval = 0.1  # Update camera every 100ms
        self.camera_offset_x = -5.0  # 5m behind the car
        self.camera_offset_y = 0.0
        self.camera_offset_z = 3.0   # 3m above ground
        self.pitch = 0.4  # Look down at the car
        self.get_logger().info('Follow camera node started - camera will follow robot')
    
    def odom_callback(self, msg):
        """Update camera position based on robot odometry."""
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
        
        # Update Gazebo camera pose
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
                '--timeout', '500',
                '--req', pose_str
            ]
            subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass  # Ignore errors (camera service might not be available)


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
