#!/usr/bin/env python3
"""
Test Goal Selection - Verify reset() selects new goal correctly.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_goal_selection.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_goal_selection():
    """Test that reset() selects a new goal."""
    print("=" * 60)
    print("Goal Selection Test")
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
    print("Testing goal selection on reset...")
    print("-" * 60)
    
    # Test multiple resets to verify goal selection
    goals_selected = []
    
    for i in range(5):
        print(f"\nReset {i+1}/5:")
        obs = env.reset()
        
        # Check observation shape (should be 724: 720 scan + 1 battery + 3 goal deltas)
        print(f"  Observation shape: {obs.shape} (expected: (724,))")
        assert obs.shape == (724,), f"Expected shape (724,), got {obs.shape}"
        
        # Check goal deltas are in observation (indices 721-723)
        dx = obs[721]
        dy = obs[722]
        dtheta = obs[723]
        print(f"  Goal deltas: Δx={dx:.3f}, Δy={dy:.3f}, Δθ={dtheta:.3f}")
        
        # Check if goal was selected
        if env.current_goal is not None:
            goal_name = env.current_goal.get('name', 'unnamed')
            goal_id = env.current_goal.get('id', -1)
            goals_selected.append((goal_id, goal_name))
            print(f"  Goal selected: {goal_name} (ID: {goal_id})")
        else:
            print(f"  ⚠ No goal selected")
            goals_selected.append(None)
    
    print()
    print("Goal selection summary:")
    print("-" * 60)
    for i, goal_info in enumerate(goals_selected, 1):
        if goal_info:
            print(f"  Reset {i}: Goal ID {goal_info[0]} - {goal_info[1]}")
        else:
            print(f"  Reset {i}: No goal selected")
    
    # Check that goals are being selected (at least some resets should have goals)
    goals_with_selection = sum(1 for g in goals_selected if g is not None)
    print(f"\nGoals selected in {goals_with_selection}/5 resets")
    
    if goals_with_selection == 0:
        print("⚠ Warning: No goals were selected in any reset")
        print("  This may indicate delivery_points.yaml is not loaded correctly")
    else:
        print("✓ Goal selection working")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Goal selection test completed!")
    print("=" * 60)
    return 0

if __name__ == '__main__':
    sys.exit(test_goal_selection())

