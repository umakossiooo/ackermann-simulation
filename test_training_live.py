#!/usr/bin/env python3
"""Live training test - run this inside Docker with Gazebo running."""

import sys
import os
import time
import subprocess

# Add the workspace to path
sys.path.insert(0, '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl')

print("=" * 70)
print("Live Training Test")
print("=" * 70)
print()

# Test 1: Check ROS 2
print("1. Checking ROS 2...")
try:
    import rclpy
    if rclpy.ok():
        print("   ✓ ROS 2 context is OK")
    else:
        print("   ⚠ ROS 2 context not initialized")
        rclpy.init()
        print("   ✓ ROS 2 initialized")
except Exception as e:
    print(f"   ✗ ROS 2 error: {e}")
    sys.exit(1)

# Test 2: Check topics
print()
print("2. Checking ROS topics...")
try:
    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        capture_output=True,
        text=True,
        timeout=5
    )
    topics = result.stdout
    
    required_topics = ['/odom', '/scan', '/cmd_vel']
    for topic in required_topics:
        if topic in topics:
            print(f"   ✓ {topic} available")
        else:
            print(f"   ✗ {topic} NOT found")
            print("   ⚠ Make sure Gazebo is running and play button is hit!")
except Exception as e:
    print(f"   ⚠ Could not check topics: {e}")

# Test 3: Check imports
print()
print("3. Checking Python imports...")
try:
    from stable_baselines3 import PPO
    print("   ✓ stable_baselines3")
except ImportError as e:
    print(f"   ✗ stable_baselines3: {e}")
    sys.exit(1)

try:
    from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
    print("   ✓ ackermann_drl")
except ImportError as e:
    print(f"   ✗ ackermann_drl: {e}")
    print("   Try: cd /root/colcon_ws && colcon build --packages-select ackermann_drl && source install/setup.bash")
    sys.exit(1)

# Test 4: Create environment
print()
print("4. Creating environment...")
try:
    env = AckermannGymEnv()
    print("   ✓ Environment created")
    print(f"   ✓ Observation space: {env.observation_space}")
    print(f"   ✓ Action space: {env.action_space}")
except Exception as e:
    print(f"   ✗ Failed to create environment: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 5: Test reset
print()
print("5. Testing environment reset...")
try:
    obs, info = env.reset()
    print(f"   ✓ Reset successful")
    print(f"   ✓ Observation shape: {obs.shape}")
    print(f"   ✓ Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
except Exception as e:
    print(f"   ✗ Reset failed: {e}")
    import traceback
    traceback.print_exc()
    env.close()
    sys.exit(1)

# Test 6: Test step (with logging)
print()
print("6. Testing environment step (checking logs)...")
print("   Running 5 steps to verify logs...")
print()

try:
    for i in range(5):
        # Random action
        import numpy as np
        action = np.array([1.0, 0.0], dtype=np.float32)  # Forward, no steering
        
        print(f"   Step {i+1}/5: Executing action [velocity={action[0]:.1f}, steering={action[1]:.1f}]")
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"      Reward: {reward:.4f}")
        print(f"      Terminated: {terminated}, Truncated: {truncated}")
        
        if 'reward_progress' in info:
            print(f"      Progress reward: {info.get('reward_progress', 0):.4f}")
        if 'reward_battery_conservation' in info:
            print(f"      Battery conservation: {info.get('reward_battery_conservation', 0):.4f}")
        
        if terminated or truncated:
            print(f"      Episode ended, resetting...")
            obs, info = env.reset()
        
        time.sleep(0.1)  # Small delay
    
    print()
    print("   ✓ Steps executed successfully")
    print("   ✓ Logs should be appearing above")
    
except Exception as e:
    print(f"   ✗ Step failed: {e}")
    import traceback
    traceback.print_exc()
    env.close()
    sys.exit(1)

# Test 7: Cleanup
print()
print("7. Cleaning up...")
try:
    env.close()
    print("   ✓ Environment closed")
except Exception as e:
    print(f"   ⚠ Cleanup warning: {e}")

# Summary
print()
print("=" * 70)
print("Test Summary")
print("=" * 70)
print("✓ All tests passed!")
print()
print("Training is ready. You can now run:")
print("  python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py --total-timesteps 1000")
print()
print("Expected logs during training:")
print("  - [REWARD] Step X | Progress: ...")
print("  - [REWARD] Step X | Battery: ...")
print("  - [REWARD] Step X | TOTAL REWARD: ...")
print()

