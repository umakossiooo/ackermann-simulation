import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import threading
import time
import subprocess
import shlex
import math


class RosInterface(Node):
    """Handles all ROS 2 communication for the environment."""
    
    def __init__(self):
        super().__init__('ackermann_gym_env_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        self.reset_sim_client = self.create_client(Empty, '/reset_simulation')
        
        self.latest_scan = None
        self.latest_odom = None
        self.scan_lock = threading.Lock()
        self.odom_lock = threading.Lock()
        
        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _scan_cb(self, msg):
        with self.scan_lock:
            self.latest_scan = msg

    def _odom_cb(self, msg):
        with self.odom_lock:
            self.latest_odom = msg

    def get_scan(self):
        with self.scan_lock:
            return self.latest_scan

    def get_odom(self):
        with self.odom_lock:
            return self.latest_odom

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish_cmd_vel(0.0, 0.0)
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

    def reset_simulation(self, initial_pose=None, timeout=2.0):
        """Call Gazebo reset service and publish initial pose."""
        if initial_pose is None:
            initial_pose = {'x': 0.0, 'y': 0.0, 'z': 0.3, 'yaw': 0.0}

        # 1. Publish 0 velocity to stop the car
        self.publish_cmd_vel(0.0, 0.0)
        
        # 2. Try to reset via services
        request = Empty.Request()
        service_called = False
        for client in (self.reset_world_client, self.reset_sim_client):
            if client is None:
                continue
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            try:
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
                if future.done():
                    service_called = True
            except Exception:
                continue
        
        # 3. Force republish initial pose (Teleport if supported by simulation bridge)
        # Note: This relies on the simulation subscribing to /initialpose to reset the model
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        # Set to origin (or specific start point if known)
        pose_msg.pose.pose.position.x = float(initial_pose.get('x', 0.0))
        pose_msg.pose.pose.position.y = float(initial_pose.get('y', 0.0))
        pose_msg.pose.pose.position.z = float(initial_pose.get('z', 0.3))
        
        yaw = float(initial_pose.get('yaw', 0.0))
        # Quaternion from yaw (rotation around Z axis)
        # q = [w, x, y, z] = [cos(yaw/2), 0, 0, sin(yaw/2)]
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        
        # 4. Fallback: Force Gazebo model teleport using `gz service` (CLI)
        # This is a robust way to force the physics engine to move the model if ROS bridge fails.
        try:
            # We assume model name is 'saye'
            x = initial_pose.get('x', 0.0)
            y = initial_pose.get('y', 0.0)
            z = initial_pose.get('z', 0.3)
            # Use calculated quaternion components
            qw = pose_msg.pose.pose.orientation.w
            qx = 0.0
            qy = 0.0
            qz = pose_msg.pose.pose.orientation.z
            
            def build_cmd(world: str) -> str:
                return (
                    f"gz service -s /world/{world}/set_pose "
                    f"--reqtype gz.msgs.Pose "
                    f"--reptype gz.msgs.Boolean "
                    f"--timeout 500 "
                    f"--req 'name: \"saye\", position: {{x: {x}, y: {y}, z: {z}}}, "
                    f"orientation: {{w: {qw}, x: {qx}, y: {qy}, z: {qz}}}'"
                )
            
            # Try the generic "map" service first (works in Docker), then the bari_world alias.
            for world_name in ("map", "bari_world"):
                subprocess.run(
                    shlex.split(build_cmd(world_name)),
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            
        except Exception:
            pass  # Ignore if gz CLI fails
        
        return service_called
