#!/usr/bin/env python3
"""
Test Observation Values - Verify observation values are within expected ranges.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_obs_values.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_observation_values():
    """Test that observation values are within expected ranges."""
    print("=" * 60)
    print("Observation Values Test")
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
    print("Testing observation value ranges...")
    print("-" * 60)
    
    # Get observation
    obs = env.reset()
    
    idx = 0
    
    # 1. LiDAR downsampled (0-179)
    print("1. LiDAR downsampled (indices 0-179):")
    lidar = obs[0:180]
    print(f"   Shape: {lidar.shape}")
    print(f"   Min: {lidar.min():.3f}, Max: {lidar.max():.3f}, Mean: {lidar.mean():.3f}")
    
    # LiDAR should be non-negative (distance measurements)
    assert (lidar >= 0.0).all(), f"LiDAR contains negative values: min={lidar.min()}"
    print("   ✓ All values non-negative")
    
    # LiDAR should be finite (no NaN or Inf)
    assert np.isfinite(lidar).all(), "LiDAR contains NaN or Inf values"
    print("   ✓ All values finite")
    
    idx += 180
    
    # 2. Velocity (180)
    print()
    print("2. Velocity (index 180):")
    velocity = obs[180]
    print(f"   Value: {velocity:.3f} m/s")
    
    # Velocity should be non-negative (magnitude)
    assert velocity >= 0.0, f"Velocity is negative: {velocity}"
    print("   ✓ Non-negative")
    
    # Velocity should be finite
    assert np.isfinite(velocity), "Velocity is NaN or Inf"
    print("   ✓ Finite")
    
    idx += 1
    
    # 3. Steering (181)
    print()
    print("3. Steering (index 181):")
    steering = obs[181]
    print(f"   Value: {steering:.3f} rad/s")
    
    # Steering can be positive or negative (angular velocity)
    # Should be finite
    assert np.isfinite(steering), "Steering is NaN or Inf"
    print("   ✓ Finite")
    
    idx += 1
    
    # 4. Goal deltas (182-184)
    print()
    print("4. Goal deltas (indices 182-184):")
    dx = obs[182]
    dy = obs[183]
    dtheta = obs[184]
    print(f"   Δx: {dx:.3f} m")
    print(f"   Δy: {dy:.3f} m")
    print(f"   Δθ: {dtheta:.3f} rad ({np.degrees(dtheta):.1f}°)")
    
    # Deltas should be finite
    assert np.isfinite(dx) and np.isfinite(dy) and np.isfinite(dtheta), \
        "Goal deltas contain NaN or Inf"
    print("   ✓ All values finite")
    
    # Δθ should be in [-π, π]
    assert -np.pi <= dtheta <= np.pi, f"Δθ out of range [-π, π]: {dtheta}"
    print("   ✓ Δθ in range [-π, π]")
    
    idx += 3
    
    # 5. Battery (185)
    print()
    print("5. Battery level (index 185):")
    battery = obs[185]
    print(f"   Value: {battery:.3f} ({battery*100:.1f}%)")
    
    # Battery should be in [0, 1]
    assert 0.0 <= battery <= 1.0, f"Battery out of range [0,1]: {battery}"
    print("   ✓ In range [0, 1]")
    
    # Battery should be finite
    assert np.isfinite(battery), "Battery is NaN or Inf"
    print("   ✓ Finite")
    
    idx += 1
    
    # 6. Road distance (186)
    print()
    print("6. Road distance (index 186):")
    road_dist = obs[186]
    print(f"   Value: {road_dist:.3f} m")
    
    # Road distance should be non-negative
    assert road_dist >= 0.0, f"Road distance is negative: {road_dist}"
    print("   ✓ Non-negative")
    
    # Road distance should be finite
    assert np.isfinite(road_dist), "Road distance is NaN or Inf"
    print("   ✓ Finite")
    
    idx += 1
    
    # Verify we've checked all components
    assert idx == 187, f"Index mismatch: expected 187, got {idx}"
    
    # Test multiple observations
    print()
    print("Testing multiple observations...")
    print("-" * 60)
    
    for i in range(5):
        action = np.array([0.5 * (i % 2), 0.1 * (i % 3 - 1)], dtype=np.float32)
        obs, _, _, _, _ = env.step(action)
        
        # Quick sanity checks
        assert obs.shape == (187,), f"Observation shape incorrect: {obs.shape}"
        assert np.isfinite(obs).all(), f"Observation {i+1} contains NaN or Inf"
        assert (obs[0:180] >= 0.0).all(), f"LiDAR in observation {i+1} contains negative values"
        assert 0.0 <= obs[185] <= 1.0, f"Battery in observation {i+1} out of range: {obs[185]}"
        assert obs[186] >= 0.0, f"Road distance in observation {i+1} is negative: {obs[186]}"
        assert -np.pi <= obs[184] <= np.pi, f"Δθ in observation {i+1} out of range: {obs[184]}"
    
    print("✓ All 5 observations passed sanity checks")
    
    # Test observation consistency
    print()
    print("Testing observation consistency...")
    print("-" * 60)
    
    obs1 = env.get_observation()
    obs2 = env.get_observation()
    
    # Observations should be similar if state hasn't changed much
    # (allowing for small differences due to sensor noise)
    diff = np.abs(obs1 - obs2)
    max_diff = diff.max()
    mean_diff = diff.mean()
    
    print(f"   Max difference: {max_diff:.6f}")
    print(f"   Mean difference: {mean_diff:.6f}")
    
    # Battery and goal deltas should be identical if state hasn't changed
    # (but LiDAR might have small differences)
    battery_diff = abs(obs1[185] - obs2[185])
    assert battery_diff < 0.001, f"Battery changed unexpectedly: {battery_diff}"
    print("   ✓ Battery consistent")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Observation values test completed!")
    print("=" * 60)
    
    return 0

if __name__ == '__main__':
    sys.exit(test_observation_values())

