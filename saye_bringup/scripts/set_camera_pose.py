#!/usr/bin/env python3
"""
Script to set Gazebo camera pose behind the robot.
Calculates camera position dynamically based on robot spawn parameters.
"""

import argparse
import math
import subprocess
import sys


def calculate_camera_pose(robot_x, robot_y, robot_z, robot_yaw, distance_behind=5.0, height_above=3.0, pitch_down=0.4):
    """Calculate camera pose behind the robot.
    
    Args:
        robot_x: Robot X position (east, meters)
        robot_y: Robot Y position (north, meters)
        robot_z: Robot Z position (height, meters)
        robot_yaw: Robot yaw angle (radians)
        distance_behind: Distance behind robot (meters)
        height_above: Height above robot (meters)
        pitch_down: Pitch angle to look down (radians)
    
    Returns:
        Tuple of (camera_x, camera_y, camera_z, quat_x, quat_y, quat_z, quat_w)
    """
    # Calculate camera position (behind robot in world frame)
    # Behind means opposite to forward direction
    cam_x = robot_x - distance_behind * math.cos(robot_yaw)
    cam_y = robot_y - distance_behind * math.sin(robot_yaw)
    cam_z = robot_z + height_above
    
    # Calculate quaternion for orientation
    # Roll=0, Pitch=pitch_down, Yaw=robot_yaw
    # Convert Euler to quaternion
    roll = 0.0
    pitch = pitch_down
    yaw = robot_yaw
    
    # Quaternion from Euler angles (ZYX convention)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    quat_w = cr * cp * cy + sr * sp * sy
    quat_x = sr * cp * cy - cr * sp * sy
    quat_y = cr * sp * cy + sr * cp * sy
    quat_z = cr * cp * sy - sr * sp * cy
    
    return (cam_x, cam_y, cam_z, quat_x, quat_y, quat_z, quat_w)


def set_gazebo_camera(cam_x, cam_y, cam_z, quat_x, quat_y, quat_z, quat_w):
    """Set Gazebo camera pose using gz service.
    
    Args:
        cam_x, cam_y, cam_z: Camera position
        quat_x, quat_y, quat_z, quat_w: Camera orientation quaternion
    """
    req = (
        f"pose: {{"
        f"position: {{x: {cam_x}, y: {cam_y}, z: {cam_z}}}, "
        f"orientation: {{x: {quat_x}, y: {quat_y}, z: {quat_z}, w: {quat_w}}"
        f"}}"
    )
    
    cmd = [
        'gz', 'service',
        '-s', '/gui/move_to/pose',
        '--reqtype', 'gz.msgs.GUICamera',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"Camera set to position: ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f})")
            return True
        else:
            print(f"Failed to set camera: {result.stderr}", file=sys.stderr)
            return False
    except subprocess.TimeoutExpired:
        print("Timeout setting camera pose", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Error setting camera: {e}", file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(description='Set Gazebo camera pose behind robot')
    parser.add_argument('--robot-x', type=float, required=True, help='Robot X position (meters)')
    parser.add_argument('--robot-y', type=float, required=True, help='Robot Y position (meters)')
    parser.add_argument('--robot-z', type=float, required=True, help='Robot Z position (meters)')
    parser.add_argument('--robot-yaw', type=float, required=True, help='Robot yaw angle (radians)')
    parser.add_argument('--distance', type=float, default=5.0, help='Distance behind robot (meters)')
    parser.add_argument('--height', type=float, default=3.0, help='Height above robot (meters)')
    parser.add_argument('--pitch', type=float, default=0.4, help='Pitch angle to look down (radians)')
    
    args = parser.parse_args()
    
    # Calculate camera pose
    cam_x, cam_y, cam_z, quat_x, quat_y, quat_z, quat_w = calculate_camera_pose(
        args.robot_x, args.robot_y, args.robot_z, args.robot_yaw,
        args.distance, args.height, args.pitch
    )
    
    # Set camera in Gazebo
    success = set_gazebo_camera(cam_x, cam_y, cam_z, quat_x, quat_y, quat_z, quat_w)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

