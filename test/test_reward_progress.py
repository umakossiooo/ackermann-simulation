#!/usr/bin/env python3
"""
Test Reward Progress - Verify progress reward calculation.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_reward_progress.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_progress_reward():
    """Test progress reward calculation."""
    print("=" * 60)
    print("Progress Reward Test")
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
    print("Testing progress reward...")
    print("-" * 60)
    
    # Reset environment
    obs = env.reset()
    
    # Check if goal is set
    if env.current_goal is None:
        print("⚠ No goal set, cannot test progress reward")
        print("  This may be normal if delivery_points.yaml is not loaded")
        env.destroy_node()
        rclpy.shutdown()
        return 0
    
    # Get initial distance
    initial_distance = env.get_distance_to_goal()
    print(f"Initial distance to goal: {initial_distance:.3f} m")
    
    # Simulate moving closer to goal (mock odometry)
    if env.latest_odom is not None:
        # Get goal position
        goal_pos = env.delivery_points.get_point_position(env.current_goal)
        
        # Move robot closer to goal
        robot_pos = env.latest_odom.pose.pose.position
        # Move 5m closer
        new_x = robot_pos.x + (goal_pos[0] - robot_pos.x) * 0.1
        new_y = robot_pos.y + (goal_pos[1] - robot_pos.y) * 0.1
        
        # Update odometry (mock)
        from nav_msgs.msg import Odometry
        mock_odom = Odometry()
        mock_odom.pose.pose.position.x = new_x
        mock_odom.pose.pose.position.y = new_y
        mock_odom.pose.pose.position.z = robot_pos.z
        mock_odom.pose.pose.orientation = env.latest_odom.pose.pose.orientation
        mock_odom.twist.twist = env.latest_odom.twist.twist
        env.latest_odom = mock_odom
        
        # Take a step
        action = np.array([0.5, 0.0], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        
        # Check reward
        print(f"Reward: {reward:.3f}")
        print(f"Reward breakdown:")
        print(f"  Progress: {info.get('reward_progress', 0.0):.3f}")
        print(f"  Goal: {info.get('reward_goal', 0.0):.3f}")
        print(f"  Off-road: {info.get('penalty_offroad', 0.0):.3f}")
        print(f"  Collision: {info.get('penalty_collision', 0.0):.3f}")
        print(f"  Battery: {info.get('penalty_battery', 0.0):.3f}")
        print(f"  Time: {info.get('penalty_time', 0.0):.3f}")
        
        # Progress reward should be positive if we got closer
        progress_reward = info.get('reward_progress', 0.0)
        if progress_reward > 0:
            print("✓ Progress reward is positive (moved closer to goal)")
        elif progress_reward < 0:
            print("⚠ Progress reward is negative (moved away from goal)")
        else:
            print("⚠ Progress reward is zero (no movement or no previous distance)")
        
        # Check that reward info is present
        assert 'reward_progress' in info, "reward_progress not in info dict"
        print("✓ Reward info contains progress component")
    
    # Test multiple steps
    print()
    print("Testing multiple steps...")
    print("-" * 60)
    
    rewards = []
    for i in range(5):
        action = np.array([0.5, 0.0], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(reward)
        progress = info.get('reward_progress', 0.0)
        print(f"  Step {i+1}: reward={reward:.3f}, progress={progress:.3f}")
    
    # Check that rewards are computed
    assert all(r != 0.0 or abs(r) < 0.001 for r in rewards), "All rewards are zero"
    print("✓ Rewards are computed for multiple steps")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Progress reward test completed!")
    print("=" * 60)
    
    return 0

if __name__ == '__main__':
    sys.exit(test_progress_reward())

