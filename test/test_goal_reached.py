#!/usr/bin/env python3
"""
Test Goal Reached - Verify Δx, Δy, Δθ computation to goal.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_goal_reached.py
"""

import sys
import rclpy
import numpy as np
import math
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
from ackermann_drl.utils.delivery_points import DeliveryPoints

def test_delta_computation():
    """Test computation of Δx, Δy, Δθ to goal."""
    print("=" * 60)
    print("Goal Delta Computation Test")
    print("=" * 60)
    print()
    
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
    
    print("Creating environment and loading delivery points...")
    try:
        env = AckermannCityEnv()
        delivery_points = DeliveryPoints()
        print("✓ Environment and delivery points loaded")
    except Exception as e:
        print(f"✗ Failed to create environment: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print()
    print("Testing Δx, Δy, Δθ computation...")
    print("-" * 60)
    
    # Get a delivery point
    if delivery_points.get_point_count() == 0:
        print("✗ No delivery points available")
        return 1
    
    test_point = delivery_points.get_random_point()
    goal_pos = delivery_points.get_point_position(test_point)
    print(f"Test goal: {test_point.get('name', 'unnamed')} at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
    
    # Set goal manually for testing
    env.current_goal = test_point
    
    # Test with different robot positions
    test_cases = [
        {
            'name': 'At goal position',
            'robot_x': goal_pos[0],
            'robot_y': goal_pos[1],
            'robot_yaw': 0.0,
            'expected_dx': 0.0,
            'expected_dy': 0.0,
            'expected_dtheta': 0.0,
            'tolerance': 0.1
        },
        {
            'name': '10m east of goal',
            'robot_x': goal_pos[0] + 10.0,
            'robot_y': goal_pos[1],
            'robot_yaw': math.pi,  # Facing west (toward goal)
            'expected_dx': -10.0,
            'expected_dy': 0.0,
            'expected_dtheta': 0.0,
            'tolerance': 0.5
        },
        {
            'name': '10m north of goal',
            'robot_x': goal_pos[0],
            'robot_y': goal_pos[1] + 10.0,
            'robot_yaw': -math.pi/2,  # Facing south (toward goal)
            'expected_dx': -10.0,
            'expected_dy': 0.0,
            'expected_dtheta': 0.0,
            'tolerance': 0.5
        },
    ]
    
    print()
    for i, test_case in enumerate(test_cases, 1):
        print(f"Test case {i}: {test_case['name']}")
        
        # Create mock odometry message
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Pose, Point, Quaternion
        
        mock_odom = Odometry()
        mock_odom.pose.pose.position.x = test_case['robot_x']
        mock_odom.pose.pose.position.y = test_case['robot_y']
        mock_odom.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = test_case['robot_yaw']
        mock_odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        mock_odom.pose.pose.orientation.x = 0.0
        mock_odom.pose.pose.orientation.y = 0.0
        mock_odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        
        # Set odometry in environment
        env.latest_odom = mock_odom
        
        # Compute deltas
        dx, dy, dtheta = env.compute_goal_deltas()
        
        print(f"  Robot position: ({test_case['robot_x']:.2f}, {test_case['robot_y']:.2f}), yaw: {yaw:.3f}")
        print(f"  Computed deltas: Δx={dx:.3f}, Δy={dy:.3f}, Δθ={dtheta:.3f}")
        print(f"  Expected deltas: Δx={test_case['expected_dx']:.3f}, "
              f"Δy={test_case['expected_dy']:.3f}, Δθ={test_case['expected_dtheta']:.3f}")
        
        # Check if within tolerance
        tolerance = test_case['tolerance']
        dx_ok = abs(dx - test_case['expected_dx']) < tolerance
        dy_ok = abs(dy - test_case['expected_dy']) < tolerance
        dtheta_ok = abs(dtheta - test_case['expected_dtheta']) < tolerance or \
                   abs(dtheta - test_case['expected_dtheta'] - 2*math.pi) < tolerance or \
                   abs(dtheta - test_case['expected_dtheta'] + 2*math.pi) < tolerance
        
        if dx_ok and dy_ok and dtheta_ok:
            print(f"  ✓ Deltas within tolerance ({tolerance})")
        else:
            print(f"  ✗ Deltas outside tolerance ({tolerance})")
            if not dx_ok:
                print(f"    Δx error: {abs(dx - test_case['expected_dx']):.3f}")
            if not dy_ok:
                print(f"    Δy error: {abs(dy - test_case['expected_dy']):.3f}")
            if not dtheta_ok:
                print(f"    Δθ error: {abs(dtheta - test_case['expected_dtheta']):.3f}")
    
    print()
    print("Testing observation includes goal deltas...")
    print("-" * 60)
    
    # Reset environment to get observation with goal
    obs = env.reset()
    
    print(f"Observation shape: {obs.shape} (expected: (724,))")
    assert obs.shape == (724,), f"Expected shape (724,), got {obs.shape}"
    
    # Extract goal deltas from observation
    dx_obs = obs[721]
    dy_obs = obs[722]
    dtheta_obs = obs[723]
    
    print(f"Goal deltas in observation: Δx={dx_obs:.3f}, Δy={dy_obs:.3f}, Δθ={dtheta_obs:.3f}")
    
    # Verify deltas are reasonable (not NaN or inf)
    assert not (np.isnan(dx_obs) or np.isinf(dx_obs)), "Δx is NaN or Inf"
    assert not (np.isnan(dy_obs) or np.isinf(dy_obs)), "Δy is NaN or Inf"
    assert not (np.isnan(dtheta_obs) or np.isinf(dtheta_obs)), "Δθ is NaN or Inf"
    
    print("✓ Goal deltas in observation are valid")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Goal delta computation test completed!")
    print("=" * 60)
    return 0

if __name__ == '__main__':
    sys.exit(test_delta_computation())

