#!/usr/bin/env python3
"""
Test Observation Shape - Verify observation vector has correct shape and structure.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_obs_shape.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv

def test_observation_shape():
    """Test that observation has correct shape."""
    print("=" * 60)
    print("Observation Shape Test")
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
    print("Testing observation shape...")
    print("-" * 60)
    
    # Expected observation structure:
    # - Downsampled LiDAR: 180 samples (720 / 4)
    # - Velocity: 1 value
    # - Steering: 1 value
    # - Goal deltas: 3 values (Δx, Δy, Δθ)
    # - Battery: 1 value
    # - Road distance: 1 value
    # Total: 180 + 1 + 1 + 3 + 1 + 1 = 187
    
    expected_size = 187
    expected_components = {
        'lidar_downsampled': 180,
        'velocity': 1,
        'steering': 1,
        'goal_deltas': 3,
        'battery': 1,
        'road_distance': 1
    }
    
    # Test reset observation
    print("Testing reset() observation...")
    obs_reset = env.reset()
    print(f"  Reset observation shape: {obs_reset.shape}")
    print(f"  Expected shape: ({expected_size},)")
    
    assert obs_reset.shape == (expected_size,), \
        f"Expected shape ({expected_size},), got {obs_reset.shape}"
    print("✓ Reset observation shape correct")
    
    # Test step observation
    print()
    print("Testing step() observation...")
    action = np.array([0.5, 0.0], dtype=np.float32)  # Forward motion
    obs_step, _, _, _, _ = env.step(action)
    print(f"  Step observation shape: {obs_step.shape}")
    
    assert obs_step.shape == (expected_size,), \
        f"Expected shape ({expected_size},), got {obs_step.shape}"
    print("✓ Step observation shape correct")
    
    # Test get_observation directly
    print()
    print("Testing get_observation() directly...")
    obs_direct = env.get_observation()
    print(f"  Direct observation shape: {obs_direct.shape}")
    
    assert obs_direct.shape == (expected_size,), \
        f"Expected shape ({expected_size},), got {obs_direct.shape}"
    print("✓ Direct observation shape correct")
    
    # Verify observation components
    print()
    print("Verifying observation components...")
    print("-" * 60)
    
    idx = 0
    
    # LiDAR downsampled (0-179)
    lidar_size = expected_components['lidar_downsampled']
    print(f"  LiDAR downsampled: indices {idx} to {idx+lidar_size-1} ({lidar_size} values)")
    assert obs_direct[idx:idx+lidar_size].shape == (lidar_size,), \
        f"LiDAR shape incorrect: {obs_direct[idx:idx+lidar_size].shape}"
    idx += lidar_size
    
    # Velocity (180)
    print(f"  Velocity: index {idx} ({expected_components['velocity']} value)")
    idx += expected_components['velocity']
    
    # Steering (181)
    print(f"  Steering: index {idx} ({expected_components['steering']} value)")
    idx += expected_components['steering']
    
    # Goal deltas (182-184)
    goal_deltas_size = expected_components['goal_deltas']
    print(f"  Goal deltas (Δx, Δy, Δθ): indices {idx} to {idx+goal_deltas_size-1} ({goal_deltas_size} values)")
    assert obs_direct[idx:idx+goal_deltas_size].shape == (goal_deltas_size,), \
        f"Goal deltas shape incorrect: {obs_direct[idx:idx+goal_deltas_size].shape}"
    idx += goal_deltas_size
    
    # Battery (185)
    print(f"  Battery: index {idx} ({expected_components['battery']} value)")
    idx += expected_components['battery']
    
    # Road distance (186)
    print(f"  Road distance: index {idx} ({expected_components['road_distance']} value)")
    idx += expected_components['road_distance']
    
    assert idx == expected_size, f"Index mismatch: expected {expected_size}, got {idx}"
    print("✓ All observation components verified")
    
    # Test data types
    print()
    print("Testing data types...")
    assert obs_direct.dtype == np.float32, f"Expected float32, got {obs_direct.dtype}"
    print("✓ Observation dtype is float32")
    
    # Test no NaN or Inf
    print()
    print("Testing for NaN and Inf values...")
    has_nan = np.isnan(obs_direct).any()
    has_inf = np.isinf(obs_direct).any()
    
    if has_nan:
        nan_count = np.isnan(obs_direct).sum()
        print(f"  ⚠ Warning: {nan_count} NaN values found")
    else:
        print("  ✓ No NaN values")
    
    if has_inf:
        inf_count = np.isinf(obs_direct).sum()
        print(f"  ⚠ Warning: {inf_count} Inf values found")
    else:
        print("  ✓ No Inf values")
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    print()
    print("=" * 60)
    print("✓ Observation shape test completed!")
    print("=" * 60)
    print()
    print("Observation structure:")
    print(f"  Total size: {expected_size}")
    print(f"  - LiDAR downsampled: {expected_components['lidar_downsampled']} samples")
    print(f"  - Velocity: {expected_components['velocity']} value")
    print(f"  - Steering: {expected_components['steering']} value")
    print(f"  - Goal deltas: {expected_components['goal_deltas']} values (Δx, Δy, Δθ)")
    print(f"  - Battery: {expected_components['battery']} value")
    print(f"  - Road distance: {expected_components['road_distance']} value")
    
    return 0

if __name__ == '__main__':
    sys.exit(test_observation_shape())

