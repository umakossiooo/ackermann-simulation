import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import threading


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
