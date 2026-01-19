#!/usr/bin/env python3
"""SMC control node for Ackermann road following."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shapely.geometry import Point

from ackermann_drl.utils.sliding_mode_control import SlidingModeController
from ackermann_drl.utils.roads_geometry import RoadsGeometry


class SMCControlNode(Node):
    """ROS 2 node that follows the nearest road centerline using SMC."""

    def __init__(self):
        super().__init__('smc_control_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.latest_odom: Optional[Odometry] = None
        self._warned_no_roads = False

        self.declare_parameter('lambda_param', 1.0)
        self.declare_parameter('K_smc', 2.0)
        self.declare_parameter('boundary_layer', 0.1)
        self.declare_parameter('desired_velocity', 2.0)
        self.declare_parameter('max_steering', 1.0)
        self.declare_parameter('control_rate', 10.0)

        lambda_param = float(self.get_parameter('lambda_param').value)
        K_smc = float(self.get_parameter('K_smc').value)
        boundary_layer = float(self.get_parameter('boundary_layer').value)
        desired_velocity = float(self.get_parameter('desired_velocity').value)
        max_steering = float(self.get_parameter('max_steering').value)
        control_rate = float(self.get_parameter('control_rate').value)

        self.controller = SlidingModeController(
            lambda_param=lambda_param,
            K_smc=K_smc,
            boundary_layer=boundary_layer,
            desired_velocity=desired_velocity,
            max_steering=max_steering,
        )

        try:
            self.roads_geometry = RoadsGeometry()
        except Exception as exc:
            self.roads_geometry = None
            self.get_logger().error(f"Failed to load road geometry: {exc}")

        if not math.isfinite(control_rate) or control_rate <= 0.0:
            control_rate = 10.0
        self.timer = self.create_timer(1.0 / control_rate, self._control_cb)

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _control_cb(self):
        if self.roads_geometry is None:
            if not self._warned_no_roads:
                self.get_logger().error("No road geometry available; stopping vehicle.")
                self._warned_no_roads = True
            self._publish_cmd(0.0, 0.0)
            return

        odom = self.latest_odom
        if odom is None:
            self._publish_cmd(0.0, 0.0)
            return

        pos = odom.pose.pose.position
        yaw = self._get_yaw_from_odom(odom)
        road_heading, polyline = self.roads_geometry.get_road_heading_at_point(pos.x, pos.y)

        if polyline is None:
            if not self._warned_no_roads:
                self.get_logger().error("No road centerline found; stopping vehicle.")
                self._warned_no_roads = True
            self._publish_cmd(0.0, 0.0)
            return

        lateral_error = self._signed_lateral_error(pos.x, pos.y, road_heading, polyline)
        heading_error = self._normalize_angle(road_heading - yaw)

        current_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        steering_cmd, velocity_cmd = self.controller.compute_control(
            lateral_error, heading_error, current_speed
        )
        self._publish_cmd(velocity_cmd, steering_cmd)

    def _signed_lateral_error(self, x: float, y: float, heading: float, polyline):
        point = Point(x, y)
        projected = polyline.interpolate(polyline.project(point))
        dist = point.distance(polyline)

        tang_x = math.cos(heading)
        tang_y = math.sin(heading)
        vec_x = x - projected.x
        vec_y = y - projected.y
        cross = tang_x * vec_y - tang_y * vec_x

        if abs(cross) < 1e-6:
            sign = 0.0
        elif cross > 0.0:
            sign = -1.0
        else:
            sign = 1.0

        return dist * sign

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self._publish_cmd(0.0, 0.0)

    @staticmethod
    def _get_yaw_from_odom(odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SMCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
