#!/usr/bin/env python3
"""
Test Reward Battery - Verify battery penalty calculation.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_reward_battery.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_battery_penalty():
    """Test battery penalty calculation."""
    print("=" * 60)
    print("Battery Penalty Test")
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
    print("Testing battery penalty...")
    print("-" * 60)
    
    # Reset environment
    obs = env.reset()
    
    # Get battery level from observation
    battery_level = obs[185]  # Battery is at index 185
    print(f"Initial battery level: {battery_level:.3f} ({battery_level*100:.1f}%)")
    
    # Test step and check battery penalty
    action = np.array([0.5, 0.0], dtype=np.float32)
    obs, reward, terminated, truncated, info = env.step(action)
    
    print()
    print("Reward breakdown:")
    print(f"  Total reward: {reward:.3f}")
    print(f"  Battery penalty: {info.get('penalty_battery', 0.0):.3f}")
    print(f"  Battery level: {info.get('battery_level', 0.0):.3f}")
    
    # Check battery penalty
    battery_penalty = info.get('penalty_battery', 0.0)
    current_battery = info.get('battery_level', 1.0)
    
    # Calculate expected penalty
    expected_penalty = env.reward_battery_penalty_scale * (1.0 - current_battery)
    print(f"  Expected penalty: {expected_penalty:.3f} (scale={env.reward_battery_penalty_scale:.3f} * (1 - {current_battery:.3f}))")
    
    if abs(battery_penalty - expected_penalty) < 0.01:
        print("✓ Battery penalty matches expected value")
    else:
        print(f"⚠ Battery penalty mismatch: expected {expected_penalty:.3f}, got {battery_penalty:.3f}")
    
    # Battery penalty should be negative or zero (penalty for low battery)
    assert battery_penalty <= 0.0, f"Battery penalty should be negative or zero, got {battery_penalty}"
    print("✓ Battery penalty is negative or zero")
    
    # Test with different battery levels
    print()
    print("Testing with different battery levels...")
    print("-" * 60)
    
    test_levels = [1.0, 0.75, 0.5, 0.25, 0.0]
    for level in test_levels:
        env.battery.set_battery_level(level)
        expected = env.reward_battery_penalty_scale * (1.0 - level)
        print(f"  Battery {level*100:.0f}%: expected penalty {expected:.3f}")
        
        # Verify calculation
        obs, reward, _, _, info = env.step(action)
        actual = info.get('penalty_battery', 0.0)
        if abs(actual - expected) < 0.01:
            print(f"    ✓ Penalty matches: {actual:.3f}")
        else:
            print(f"    ✗ Penalty mismatch: expected {expected:.3f}, got {actual:.3f}")
    
    # Test battery depletion
    print()
    print("Testing battery depletion...")
    print("-" * 60)
    
    env.battery.set_battery_level(0.0)
    obs, reward, terminated, truncated, info = env.step(action)
    
    print(f"  Battery level: {info.get('battery_level', 0.0):.3f}")
    print(f"  Battery depleted: {info.get('battery_depleted', False)}")
    print(f"  Truncated: {truncated}")
    
    if info.get('battery_depleted', False):
        assert truncated, "Episode should be truncated when battery depleted"
        print("✓ Episode truncated when battery depleted")
    
    # Test that penalty increases as battery decreases
    print()
    print("Testing penalty increases with lower battery...")
    print("-" * 60)
    
    penalties = []
    for level in [1.0, 0.8, 0.6, 0.4, 0.2, 0.0]:
        env.battery.set_battery_level(level)
        obs, reward, _, _, info = env.step(action)
        penalty = info.get('penalty_battery', 0.0)
        penalties.append(penalty)
        print(f"  Battery {level*100:.0f}%: penalty {penalty:.3f}")
    
    # Penalties should be non-decreasing (more negative as battery decreases)
    for i in range(len(penalties) - 1):
        assert penalties[i] >= penalties[i+1], \
            f"Penalty should increase (become more negative) as battery decreases: {penalties[i]} < {penalties[i+1]}"
    
    print("✓ Penalty increases (becomes more negative) as battery decreases")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Battery penalty test completed!")
    print("=" * 60)
    
    return 0

if __name__ == '__main__':
    sys.exit(test_battery_penalty())

