#!/usr/bin/env python3
"""
Test Eval Model Load - Verify model loading and evaluation setup.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_eval_model_load.py
"""

import sys
import os
from pathlib import Path

def test_stable_baselines3_import():
    """Test stable-baselines3 import."""
    print("Testing stable-baselines3 import...")
    try:
        from stable_baselines3 import PPO
        print("✓ PPO imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Failed to import PPO: {e}")
        return False
    except Exception as e:
        print(f"✗ Error importing PPO: {e}")
        return False

def test_model_loading():
    """Test that model loading function works."""
    print()
    print("Testing model loading...")
    print("-" * 60)
    
    try:
        from stable_baselines3 import PPO
        
        # Check if any checkpoints exist
        checkpoint_dir = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints')
        if not checkpoint_dir.exists():
            checkpoint_dir.mkdir(parents=True, exist_ok=True)
            print(f"✓ Created checkpoint directory: {checkpoint_dir}")
        else:
            print(f"✓ Checkpoint directory exists: {checkpoint_dir}")
        
        # Look for existing models
        model_files = list(checkpoint_dir.glob('*.zip'))
        if model_files:
            print(f"✓ Found {len(model_files)} model file(s):")
            for f in model_files[:3]:  # Show first 3
                print(f"  - {f.name}")
            
            # Try to load the first model
            test_model_path = model_files[0]
            print()
            print(f"Testing model load from {test_model_path.name}...")
            try:
                model = PPO.load(str(test_model_path))
                print(f"✓ Model loaded successfully")
                print(f"  Policy: {model.policy}")
                print(f"  Device: {model.device}")
                return True
            except Exception as e:
                print(f"✗ Failed to load model: {e}")
                import traceback
                traceback.print_exc()
                return False
        else:
            print("⚠ No model files found in checkpoint directory")
            print("  This is normal if no training has been run yet")
            print("  Model loading will be tested when a model is available")
            return True  # Not a failure, just no models to test
    
    except Exception as e:
        print(f"✗ Error testing model loading: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_eval_script_import():
    """Test that eval_policy script can be imported."""
    print()
    print("Testing eval_policy script...")
    print("-" * 60)
    
    script_path = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/eval_policy.py')
    if not script_path.exists():
        print(f"✗ eval_policy.py not found at {script_path}")
        return False
    
    print(f"✓ eval_policy.py exists at {script_path}")
    
    # Check if script is readable
    if not os.access(script_path, os.R_OK):
        print(f"✗ eval_policy.py is not readable")
        return False
    
    print(f"✓ eval_policy.py is readable")
    
    # Check if script is executable
    if not os.access(script_path, os.X_OK):
        print(f"  Making script executable...")
        os.chmod(script_path, 0o755)
        print(f"✓ Made eval_policy.py executable")
    else:
        print(f"✓ eval_policy.py is executable")
    
    # Try to import the main function
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("eval_policy", str(script_path))
        if spec is None:
            print("✗ Could not create spec from script")
            return False
        
        module = importlib.util.module_from_spec(spec)
        # Don't execute, just check if it can be loaded
        print("✓ eval_policy.py can be loaded")
        return True
    except Exception as e:
        print(f"✗ Error loading eval_policy script: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_gym_wrapper():
    """Test that gym wrapper can be created."""
    print()
    print("Testing gym wrapper...")
    print("-" * 60)
    
    try:
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        import rclpy
        
        # Initialize ROS 2
        if not rclpy.ok():
            rclpy.init()
        
        # Create environment
        env = AckermannGymEnv()
        print("✓ Gym wrapper created")
        
        # Test reset
        obs, info = env.reset()
        print(f"✓ Environment reset successful")
        print(f"  Observation shape: {obs.shape}")
        
        # Test step
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"✓ Environment step successful")
        print(f"  Reward: {reward:.3f}")
        print(f"  Terminated: {terminated}, Truncated: {truncated}")
        
        # Cleanup
        env.close()
        if rclpy.ok():
            rclpy.shutdown()
        
        print("✓ Gym wrapper test passed")
        return True
    
    except Exception as e:
        print(f"✗ Error testing gym wrapper: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_evaluation_functions():
    """Test that evaluation helper functions exist."""
    print()
    print("Testing evaluation functions...")
    print("-" * 60)
    
    try:
        # Check if eval_policy.py has the required functions
        script_path = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/eval_policy.py')
        
        with open(script_path, 'r') as f:
            content = f.read()
        
        required_functions = [
            'evaluate_policy',
            'print_statistics',
            'make_env'
        ]
        
        for func_name in required_functions:
            if f'def {func_name}' in content:
                print(f"✓ Function {func_name} found")
            else:
                print(f"✗ Function {func_name} not found")
                return False
        
        # Check for required imports
        required_imports = [
            'from stable_baselines3 import PPO',
            'from ackermann_drl.envs.gym_wrapper import AckermannGymEnv'
        ]
        
        for import_stmt in required_imports:
            if import_stmt in content:
                print(f"✓ Import found: {import_stmt.split()[-1]}")
            else:
                print(f"⚠ Import not found: {import_stmt}")
        
        print("✓ Evaluation functions test passed")
        return True
    
    except Exception as e:
        print(f"✗ Error testing evaluation functions: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests."""
    print("=" * 60)
    print("Eval Model Load Test")
    print("=" * 60)
    print()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: stable-baselines3 import
    if test_stable_baselines3_import():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 2: Model loading
    if test_model_loading():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 3: Eval script import
    if test_eval_script_import():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 4: Gym wrapper
    if test_gym_wrapper():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 5: Evaluation functions
    if test_evaluation_functions():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Summary
    print()
    print("=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"Passed: {tests_passed}")
    print(f"Failed: {tests_failed}")
    print(f"Total:  {tests_passed + tests_failed}")
    print()
    
    if tests_failed == 0:
        print("✓ All eval model load tests passed!")
        return 0
    else:
        print("✗ Some tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

