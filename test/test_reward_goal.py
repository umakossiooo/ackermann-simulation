#!/usr/bin/env python3
"""
Test Reward Goal - Verify goal reward calculation.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_reward_goal.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_goal_reward():
    """Test goal reward calculation."""
    print("=" * 60)
    print("Goal Reward Test")
    print("=" * 60)
    print()
    
    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
    
    print("Creating AckermannCityEnv...")
    try:
        env = AckermannCityEnv()
        print("✓ Environment created")
    except Exception as e:
        print(f"✗ Failed to create environment: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print()
    print("Testing goal reward...")
    print("-" * 60)
    
    # Reset environment
    obs = env.reset()
    
    # Check if goal is set
    if env.current_goal is None:
        print("⚠ No goal set, cannot test goal reward")
        print("  This may be normal if delivery_points.yaml is not loaded")
        env.destroy_node()
        rclpy.shutdown()
        return 0
    
    # Get goal position
    goal_pos = env.delivery_points.get_point_position(env.current_goal)
    print(f"Goal position: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
    
    # Get initial distance
    initial_distance = env.get_distance_to_goal()
    print(f"Initial distance to goal: {initial_distance:.3f} m")
    print(f"Goal reached threshold: {env.goal_reached_threshold:.3f} m")
    
    # Check if goal is already reached
    if env.is_goal_reached():
        print("⚠ Goal already reached at reset")
    else:
        print("✓ Goal not reached initially")
    
    # Simulate reaching goal (move robot to goal position)
    if env.latest_odom is not None:
        from nav_msgs.msg import Odometry
        
        # Move robot to goal position
        mock_odom = Odometry()
        mock_odom.pose.pose.position.x = goal_pos[0]
        mock_odom.pose.pose.position.y = goal_pos[1]
        mock_odom.pose.pose.position.z = 0.0
        mock_odom.pose.pose.orientation = env.latest_odom.pose.pose.orientation
        mock_odom.twist.twist = env.latest_odom.twist.twist
        env.latest_odom = mock_odom
        
        # Reset previous distance to trigger progress calculation
        env.prev_distance_to_goal = initial_distance
        
        # Take a step
        action = np.array([0.0, 0.0], dtype=np.float32)  # Stop
        obs, reward, terminated, truncated, info = env.step(action)
        
        print()
        print("After moving to goal:")
        print(f"  Distance to goal: {env.get_distance_to_goal():.3f} m")
        print(f"  Goal reached: {env.is_goal_reached()}")
        print(f"  Terminated: {terminated}")
        
        print()
        print("Reward breakdown:")
        print(f"  Total reward: {reward:.3f}")
        print(f"  Goal reward: {info.get('reward_goal', 0.0):.3f}")
        print(f"  Progress reward: {info.get('reward_progress', 0.0):.3f}")
        
        # Check goal reward
        goal_reward = info.get('reward_goal', 0.0)
        
        if env.is_goal_reached():
            assert goal_reward == env.reward_goal_reached, \
                f"Goal reward should be {env.reward_goal_reached}, got {goal_reward}"
            print(f"✓ Goal reward matches expected value: {goal_reward:.3f}")
            
            assert terminated, "Episode should terminate when goal reached"
            print("✓ Episode terminated when goal reached")
        else:
            assert goal_reward == 0.0, f"Goal reward should be 0 when goal not reached, got {goal_reward}"
            print("✓ No goal reward when goal not reached")
    
    # Test goal detection threshold
    print()
    print("Testing goal detection threshold...")
    print("-" * 60)
    
    # Test with different distances
    test_distances = [
        (env.goal_reached_threshold * 0.5, True),   # Within threshold
        (env.goal_reached_threshold * 1.0, True),  # At threshold
        (env.goal_reached_threshold * 1.5, False), # Beyond threshold
    ]
    
    for distance, should_reach in test_distances:
        # Mock distance by moving robot
        if env.latest_odom is not None:
            from nav_msgs.msg import Odometry
            import math
            
            # Calculate position at distance from goal
            dx = distance * 0.707  # 45 degrees
            dy = distance * 0.707
            
            mock_odom = Odometry()
            mock_odom.pose.pose.position.x = goal_pos[0] + dx
            mock_odom.pose.pose.position.y = goal_pos[1] + dy
            mock_odom.pose.pose.position.z = 0.0
            mock_odom.pose.pose.orientation = env.latest_odom.pose.pose.orientation
            mock_odom.twist.twist = env.latest_odom.twist.twist
            env.latest_odom = mock_odom
            
            is_reached = env.is_goal_reached()
            actual_distance = env.get_distance_to_goal()
            
            print(f"  Distance {distance:.3f}m: reached={is_reached}, actual={actual_distance:.3f}m")
            
            if should_reach:
                assert is_reached, f"Goal should be reached at distance {distance:.3f}m"
            else:
                assert not is_reached, f"Goal should not be reached at distance {distance:.3f}m"
    
    print("✓ Goal detection threshold works correctly")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Goal reward test completed!")
    print("=" * 60)
    
    return 0

if __name__ == '__main__':
    sys.exit(test_goal_reward())

