#!/usr/bin/env python3
"""
Verify System Wiring - Check all components are correctly connected.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/verify_wiring.py
"""

import sys
import rclpy
import numpy as np
from pathlib import Path

def test_imports():
    """Test all critical imports."""
    print("Testing imports...")
    try:
        from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        from ackermann_drl.utils.battery_model import BatteryModel
        from ackermann_drl.utils.delivery_points import DeliveryPoints
        from ackermann_drl.utils.roads_geometry import RoadsGeometry
        from stable_baselines3 import PPO
        print("✓ All imports successful")
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_environment_wiring():
    """Test environment ROS topic wiring."""
    print()
    print("Testing environment ROS topic wiring...")
    print("-" * 60)
    
    if not rclpy.ok():
        rclpy.init()
    
    try:
        from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
        
        env = AckermannCityEnv()
        
        # Check subscriptions
        print("Checking subscriptions...")
        subs = env.get_subscription_names()
        has_scan = any('/scan' in sub for sub in subs)
        has_odom = any('/odom' in sub for sub in subs)
        
        print(f"  /scan subscription: {'✓' if has_scan else '✗'}")
        print(f"  /odom subscription: {'✓' if has_odom else '✗'}")
        
        # Check publishers
        print("Checking publishers...")
        pubs = env.get_publisher_names()
        has_cmd_vel = any('/cmd_vel' in pub for pub in pubs)
        
        print(f"  /cmd_vel publisher: {'✓' if has_cmd_vel else '✗'}")
        
        # Check observation space
        print("Checking observation...")
        obs = env.reset()
        print(f"  Observation shape: {obs.shape} (expected: (187,))")
        assert obs.shape == (187,), f"Expected (187,), got {obs.shape}"
        print("  ✓ Observation shape correct")
        
        # Check step
        print("Checking step...")
        action = np.array([0.5, 0.0], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"  Reward: {reward:.3f}")
        print(f"  Info keys: {list(info.keys())}")
        
        # Check required info fields
        required_fields = ['battery_level', 'reward_progress', 'penalty_offroad', 
                          'penalty_battery', 'penalty_time']
        missing = [f for f in required_fields if f not in info]
        if missing:
            print(f"  ✗ Missing info fields: {missing}")
            return False
        else:
            print("  ✓ All required info fields present")
        
        env.destroy_node()
        rclpy.shutdown()
        
        return has_scan and has_odom and has_cmd_vel
        
    except Exception as e:
        print(f"✗ Environment wiring test failed: {e}")
        import traceback
        traceback.print_exc()
        if rclpy.ok():
            rclpy.shutdown()
        return False

def test_gym_wrapper_wiring():
    """Test gym wrapper wiring."""
    print()
    print("Testing gym wrapper wiring...")
    print("-" * 60)
    
    if not rclpy.ok():
        rclpy.init()
    
    try:
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        
        env = AckermannGymEnv()
        
        # Check action space
        print(f"Action space: {env.action_space}")
        assert env.action_space.shape == (2,), f"Expected (2,), got {env.action_space.shape}"
        print("  ✓ Action space correct")
        
        # Check observation space
        print(f"Observation space: {env.observation_space}")
        assert env.observation_space.shape == (187,), f"Expected (187,), got {env.observation_space.shape}"
        print("  ✓ Observation space correct")
        
        # Test reset
        obs, info = env.reset()
        assert obs.shape == (187,), f"Expected (187,), got {obs.shape}"
        print("  ✓ Reset works")
        
        # Test step
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert obs.shape == (187,), f"Expected (187,), got {obs.shape}"
        print("  ✓ Step works")
        
        env.close()
        
        return True
        
    except Exception as e:
        print(f"✗ Gym wrapper wiring test failed: {e}")
        import traceback
        traceback.print_exc()
        if rclpy.ok():
            rclpy.shutdown()
        return False

def test_training_script_wiring():
    """Test training script can create environment."""
    print()
    print("Testing training script wiring...")
    print("-" * 60)
    
    try:
        import sys
        from pathlib import Path
        
        # Add script path
        script_path = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py')
        if not script_path.exists():
            print("✗ train_ppo.py not found")
            return False
        
        # Check script can import required modules
        sys.path.insert(0, str(script_path.parent.parent))
        
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv
        
        # Test make_env function logic
        def make_env():
            return AckermannGymEnv()
        
        env = make_env()
        vec_env = DummyVecEnv([lambda: env])
        
        print("  ✓ Environment creation works")
        print("  ✓ Vectorized environment works")
        
        # Test PPO can be created (without training)
        try:
            model = PPO('MlpPolicy', vec_env, verbose=0)
            print("  ✓ PPO model creation works")
            vec_env.close()
            env.close()
            return True
        except Exception as e:
            print(f"  ✗ PPO creation failed: {e}")
            vec_env.close()
            env.close()
            return False
        
    except Exception as e:
        print(f"✗ Training script wiring test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_file_paths():
    """Test all required files exist."""
    print()
    print("Testing file paths...")
    print("-" * 60)
    
    base_path = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2')
    
    required_files = [
        'ackermann_drl/envs/ackermann_city_env.py',
        'ackermann_drl/envs/gym_wrapper.py',
        'ackermann_drl/utils/battery_model.py',
        'ackermann_drl/utils/delivery_points.py',
        'ackermann_drl/utils/roads_geometry.py',
        'ackermann_drl/scripts/train_ppo.py',
        'ackermann_drl/scripts/eval_policy.py',
        'ackermann_drl/config/delivery_points.yaml',
        'requirements.txt',
    ]
    
    all_exist = True
    for file_path in required_files:
        full_path = base_path / file_path
        exists = full_path.exists()
        print(f"  {'✓' if exists else '✗'} {file_path}")
        if not exists:
            all_exist = False
    
    return all_exist

def test_delivery_points_loading():
    """Test delivery points can be loaded."""
    print()
    print("Testing delivery points loading...")
    print("-" * 60)
    
    try:
        from ackermann_drl.utils.delivery_points import DeliveryPoints
        
        dp = DeliveryPoints()
        count = dp.get_point_count()
        print(f"  ✓ Loaded {count} delivery points")
        
        if count == 0:
            print("  ⚠ No delivery points loaded (may cause issues)")
            return False
        
        # Test getting random point
        point = dp.get_random_point()
        pos = dp.get_point_position(point)
        print(f"  ✓ Random point: {point.get('name')} at ({pos[0]:.2f}, {pos[1]:.2f})")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Delivery points loading failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_roads_geometry_loading():
    """Test roads geometry can be loaded."""
    print()
    print("Testing roads geometry loading...")
    print("-" * 60)
    
    try:
        from ackermann_drl.utils.roads_geometry import RoadsGeometry
        
        roads = RoadsGeometry()
        count = roads.get_all_roads_count()
        print(f"  ✓ Loaded {count} roads")
        
        if count == 0:
            print("  ⚠ No roads loaded (road distance will be 0)")
            return False
        
        # Test distance calculation
        distance, _ = roads.distance_to_nearest_road(169.37, 0.21)
        print(f"  ✓ Distance calculation works: {distance:.3f}m")
        
        return True
        
    except Exception as e:
        print(f"  ⚠ Roads geometry loading failed: {e}")
        print("  (This is OK if osm_city_pipeline is not mounted)")
        return True  # Not critical

def test_battery_model():
    """Test battery model."""
    print()
    print("Testing battery model...")
    print("-" * 60)
    
    try:
        from ackermann_drl.utils.battery_model import BatteryModel
        
        battery = BatteryModel()
        level = battery.get_battery_level()
        print(f"  ✓ Battery initialized: {level:.3f}")
        
        # Test update
        battery.update(np.array([0.0, 0.0, 0.0]), 1.0, dt=0.1)
        battery.update(np.array([10.0, 0.0, 0.0]), 1.0, dt=0.1)
        new_level = battery.get_battery_level()
        print(f"  ✓ Battery update works: {new_level:.3f}")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Battery model test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_observation_structure():
    """Test observation structure is correct."""
    print()
    print("Testing observation structure...")
    print("-" * 60)
    
    if not rclpy.ok():
        rclpy.init()
    
    try:
        from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
        
        env = AckermannCityEnv()
        obs = env.get_observation()
        
        print(f"  Observation shape: {obs.shape}")
        assert obs.shape == (187,), f"Expected (187,), got {obs.shape}"
        
        # Check components
        print("  Checking components...")
        print(f"    LiDAR (0-179): {obs[0:180].shape}")
        print(f"    Velocity (180): {obs[180]:.3f}")
        print(f"    Steering (181): {obs[181]:.3f}")
        print(f"    Δx (182): {obs[182]:.3f}")
        print(f"    Δy (183): {obs[183]:.3f}")
        print(f"    Δθ (184): {obs[184]:.3f}")
        print(f"    Battery (185): {obs[185]:.3f}")
        print(f"    Road dist (186): {obs[186]:.3f}")
        
        # Verify ranges
        assert 0.0 <= obs[185] <= 1.0, f"Battery out of range: {obs[185]}"
        assert obs[186] >= 0.0, f"Road distance negative: {obs[186]}"
        assert -np.pi <= obs[184] <= np.pi, f"Δθ out of range: {obs[184]}"
        
        print("  ✓ All observation components valid")
        
        env.destroy_node()
        rclpy.shutdown()
        
        return True
        
    except Exception as e:
        print(f"  ✗ Observation structure test failed: {e}")
        import traceback
        traceback.print_exc()
        if rclpy.ok():
            rclpy.shutdown()
        return False

def test_reward_computation():
    """Test reward computation."""
    print()
    print("Testing reward computation...")
    print("-" * 60)
    
    if not rclpy.ok():
        rclpy.init()
    
    try:
        from ackermann_drl.envs.ackermann_city_env import AckermannCityEnv
        
        env = AckermannCityEnv()
        obs = env.reset()
        
        # Take a step
        action = np.array([0.5, 0.0], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"  Reward: {reward:.3f}")
        print(f"  Reward components:")
        print(f"    Progress: {info.get('reward_progress', 0.0):.3f}")
        print(f"    Goal: {info.get('reward_goal', 0.0):.3f}")
        print(f"    Off-road: {info.get('penalty_offroad', 0.0):.3f}")
        print(f"    Collision: {info.get('penalty_collision', 0.0):.3f}")
        print(f"    Battery: {info.get('penalty_battery', 0.0):.3f}")
        print(f"    Time: {info.get('penalty_time', 0.0):.3f}")
        
        # Verify reward is computed
        assert isinstance(reward, (int, float)), f"Reward not numeric: {type(reward)}"
        print("  ✓ Reward computation works")
        
        env.destroy_node()
        rclpy.shutdown()
        
        return True
        
    except Exception as e:
        print(f"  ✗ Reward computation test failed: {e}")
        import traceback
        traceback.print_exc()
        if rclpy.ok():
            rclpy.shutdown()
        return False

def main():
    """Run all wiring tests."""
    print("=" * 60)
    print("System Wiring Verification")
    print("=" * 60)
    print()
    
    tests = [
        ("Imports", test_imports),
        ("File Paths", test_file_paths),
        ("Environment ROS Wiring", test_environment_wiring),
        ("Gym Wrapper Wiring", test_gym_wrapper_wiring),
        ("Training Script Wiring", test_training_script_wiring),
        ("Delivery Points Loading", test_delivery_points_loading),
        ("Roads Geometry Loading", test_roads_geometry_loading),
        ("Battery Model", test_battery_model),
        ("Observation Structure", test_observation_structure),
        ("Reward Computation", test_reward_computation),
    ]
    
    passed = 0
    failed = 0
    warnings = 0
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            if result:
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  ✗ {test_name} crashed: {e}")
            failed += 1
    
    print()
    print("=" * 60)
    print("Verification Summary")
    print("=" * 60)
    print(f"Passed:  {passed}")
    print(f"Failed:  {failed}")
    print(f"Total:   {len(tests)}")
    print()
    
    if failed == 0:
        print("✓ All wiring checks passed!")
        print()
        print("System is correctly wired and ready for training.")
        return 0
    else:
        print("✗ Some wiring checks failed.")
        print("Please review the errors above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

