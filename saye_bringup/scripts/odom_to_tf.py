#!/usr/bin/env python3
"""
Node to publish odom->base_link transform from odometry messages.
This ensures the TF chain is connected when the odometry publisher doesn't publish TF directly.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # Publish odom -> saye (not saye/base_link) to connect with robot_state_publisher
        # robot_state_publisher publishes saye -> saye/base_link
        # This creates: map -> odom -> saye -> saye/base_link
        self.base_frame = self.declare_parameter('base_frame', 'saye').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.get_logger().info(f'Publishing {self.odom_frame} -> {self.base_frame} from /odom topic')
        self.message_count = 0

    def odom_callback(self, msg):
        self.message_count += 1
        if self.message_count % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f'Received {self.message_count} odometry messages. Frame: {msg.header.frame_id} -> {msg.child_frame_id}')
        
        transform = TransformStamped()
        # Use the timestamp from the odometry message to ensure proper timing
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        
        # Copy pose from odometry message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

