#!/usr/bin/env python3
"""
Test Environment Step - Verify environment can perform 10 random steps without crashing.

This test must be run inside the Docker container WITH Gazebo running:
    docker compose exec ackermann_sim bash
    # In one terminal, launch Gazebo:
    # ros2 launch saye_bringup saye_spawn.launch.py gui:=false
    # In another terminal or after Gazebo is running:
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_env_step.py
"""

import sys
import rclpy
import numpy as np
from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
import time

def main():
    """Run step tests."""
    print("=" * 60)
    print("Environment Step Test")
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
        return 1
    
    print()
    print("Testing reset()...")
    try:
        obs = env.reset()
        print(f"✓ reset() succeeded, observation shape: {obs.shape}")
        print(f"  Observation dtype: {obs.dtype}")
        print(f"  Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
    except Exception as e:
        print(f"✗ reset() failed: {e}")
        env.destroy_node()
        rclpy.shutdown()
        return 1
    
    print()
    print("Testing 10 random steps...")
    print("-" * 60)
    
    steps_completed = 0
    steps_failed = 0
    
    for i in range(10):
        try:
            # Generate random action: [linear_velocity, angular_velocity]
            # Linear: -2.0 to 2.0 m/s
            # Angular: -1.0 to 1.0 rad/s
            action = np.array([
                np.random.uniform(-2.0, 2.0),
                np.random.uniform(-1.0, 1.0)
            ], dtype=np.float32)
            
            # Perform step
            obs, reward, terminated, truncated, info = env.step(action)
            
            steps_completed += 1
            print(f"Step {i+1}/10: action=[{action[0]:.3f}, {action[1]:.3f}], "
                  f"obs_shape={obs.shape}, reward={reward:.3f}, "
                  f"scan={info.get('scan_received', False)}, "
                  f"odom={info.get('odom_received', False)}")
            
            # Small delay to allow message processing
            time.sleep(0.1)
            
        except Exception as e:
            steps_failed += 1
            print(f"✗ Step {i+1}/10 failed: {e}")
            import traceback
            traceback.print_exc()
    
    print()
    print("=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"Steps completed: {steps_completed}/10")
    print(f"Steps failed: {steps_failed}/10")
    print()
    
    # Cleanup
    try:
        env.destroy_node()
        rclpy.shutdown()
    except:
        pass
    
    if steps_failed == 0:
        print("✓ All step tests passed!")
        return 0
    else:
        print("✗ Some step tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

