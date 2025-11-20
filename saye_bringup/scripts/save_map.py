#!/usr/bin/env python3
"""
Simple script to save the map from the /map topic.
This works when SLAM is running and publishing the /map topic.

Usage:
    python3 save_map.py <output_file_path>

Example:
    python3 save_map.py ~/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import sys
import os

# Try to import optional dependencies
try:
    import yaml
except ImportError:
    yaml = None
    print("Warning: PyYAML not installed. Install with: pip install pyyaml")

try:
    import numpy as np
except ImportError:
    np = None
    print("Warning: NumPy not installed. Install with: pip install numpy")

try:
    from PIL import Image
except ImportError:
    Image = None
    print("Warning: Pillow not installed. Install with: pip install pillow")

class MapSaver(Node):
    def __init__(self, output_file):
        super().__init__('map_saver_script')
        self.output_file = output_file
        self.map_received = False
        
        # Use transient_local QoS to match slam_toolbox's map publisher
        # slam_toolbox publishes maps with transient_local durability
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        self.get_logger().info(f'Waiting for map on /map topic (with transient_local QoS)...')
        self.get_logger().info(f'Will save to: {self.output_file}.pgm and {self.output_file}.yaml')

    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info('Received map! Saving...')
            try:
                self.save_map(msg, self.output_file, 0.25, 0.65)
                self.get_logger().info(f'Map saved successfully!')
                self.get_logger().info(f'  - Image: {self.output_file}.pgm')
                self.get_logger().info(f'  - YAML: {self.output_file}.yaml')
                self.map_received = True
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f'Failed to save map: {e}')
                import traceback
                traceback.print_exc()
                rclpy.shutdown()

    def save_map(self, map_msg, map_name, free_thresh, occupied_thresh):
        """Save the map to PGM and YAML files"""
        if np is None or Image is None or yaml is None:
            self.get_logger().error('Missing required dependencies!')
            self.get_logger().error('Install with: pip install numpy pillow pyyaml')
            raise ImportError('Missing required dependencies')
        
        # Convert occupancy grid to image
        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        
        # Convert to image format: -1 (unknown) -> 205, 0-100 (free-occupied) -> 0-254
        image_data = np.zeros((height, width), dtype=np.uint8)
        image_data[data == -1] = 205  # Unknown
        image_data[data >= 0] = np.round((100.0 - data[data >= 0]) * 254.0 / 100.0).astype(np.uint8)
        
        # Save PGM
        pgm_file = f'{map_name}.pgm'
        img = Image.fromarray(image_data, mode='L')
        img.save(pgm_file)
        
        # Save YAML
        yaml_file = f'{map_name}.yaml'
        yaml_data = {
            'image': os.path.basename(pgm_file),
            'resolution': map_msg.info.resolution,
            'origin': [
                map_msg.info.origin.position.x,
                map_msg.info.origin.position.y,
                map_msg.info.origin.position.z
            ],
            'negate': 0,
            'occupied_thresh': occupied_thresh,
            'free_thresh': free_thresh
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 save_map.py <output_file_path>")
        print("Example: python3 save_map.py ~/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/saye_bringup/maps/bari_map")
        sys.exit(1)
    
    output_file = sys.argv[1]
    # Expand ~ and make absolute path
    output_file = os.path.expanduser(output_file)
    output_file = os.path.abspath(output_file)
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    rclpy.init(args=args)
    node = MapSaver(output_file)
    
    # Spin with timeout
    timeout = 30.0
    start_time = node.get_clock().now()
    while rclpy.ok() and not node.map_received:
        rclpy.spin_once(node, timeout_sec=1.0)
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout:
            node.get_logger().error(f'Timeout after {timeout}s: No map received on /map topic.')
            node.get_logger().error('Make sure SLAM is running:')
            node.get_logger().error('  ros2 launch saye_bringup slam.launch.py gui:=true')
            node.get_logger().error('  OR')
            node.get_logger().error('  ros2 launch saye_bringup slam_navigation.launch.py gui:=true')
            rclpy.shutdown()
            sys.exit(1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

