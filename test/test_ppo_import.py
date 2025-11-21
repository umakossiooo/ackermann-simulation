#!/usr/bin/env python3
"""
Test PPO Import - Verify stable-baselines3 and PPO can be imported.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_ppo_import.py
"""

import sys

def test_torch_import():
    """Test torch import."""
    print("Testing torch import...")
    try:
        import torch
        print(f"✓ torch version: {torch.__version__}")
        
        # Check if CUDA is available (optional)
        if torch.cuda.is_available():
            print(f"✓ CUDA available: {torch.cuda.get_device_name(0)}")
        else:
            print("  CUDA not available (using CPU)")
        
        return True
    except ImportError as e:
        print(f"✗ Failed to import torch: {e}")
        return False
    except Exception as e:
        print(f"✗ Error importing torch: {e}")
        return False

def test_stable_baselines3_import():
    """Test stable-baselines3 import."""
    print()
    print("Testing stable-baselines3 import...")
    try:
        import stable_baselines3
        print(f"✓ stable-baselines3 version: {stable_baselines3.__version__}")
        
        # Test PPO import
        from stable_baselines3 import PPO
        print("✓ PPO imported successfully")
        
        # Test Monitor import
        from stable_baselines3.common.monitor import Monitor
        print("✓ Monitor imported successfully")
        
        # Test callbacks import
        from stable_baselines3.common.callbacks import CheckpointCallback
        print("✓ CheckpointCallback imported successfully")
        
        # Test vec_env import
        from stable_baselines3.common.vec_env import DummyVecEnv
        print("✓ DummyVecEnv imported successfully")
        
        return True
    except ImportError as e:
        print(f"✗ Failed to import stable-baselines3: {e}")
        return False
    except Exception as e:
        print(f"✗ Error importing stable-baselines3: {e}")
        return False

def test_gym_wrapper_import():
    """Test gym wrapper import."""
    print()
    print("Testing gym wrapper import...")
    try:
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        print("✓ AckermannGymEnv imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Failed to import AckermannGymEnv: {e}")
        import traceback
        traceback.print_exc()
        return False
    except Exception as e:
        print(f"✗ Error importing AckermannGymEnv: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_train_ppo_import():
    """Test train_ppo script can be imported."""
    print()
    print("Testing train_ppo script import...")
    try:
        import importlib.util
        script_path = '/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py'
        spec = importlib.util.spec_from_file_location("train_ppo", script_path)
        if spec is None:
            print(f"✗ Could not load spec from {script_path}")
            return False
        
        # Just check if file exists and is readable
        import os
        if os.path.exists(script_path):
            print(f"✓ train_ppo.py exists at {script_path}")
            if os.access(script_path, os.R_OK):
                print("✓ train_ppo.py is readable")
                return True
            else:
                print("✗ train_ppo.py is not readable")
                return False
        else:
            print(f"✗ train_ppo.py not found at {script_path}")
            return False
    except Exception as e:
        print(f"✗ Error checking train_ppo script: {e}")
        return False

def main():
    """Run all import tests."""
    print("=" * 60)
    print("PPO Import Test")
    print("=" * 60)
    print()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: torch
    if test_torch_import():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 2: stable-baselines3
    if test_stable_baselines3_import():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 3: gym wrapper
    if test_gym_wrapper_import():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Test 4: train_ppo script
    if test_train_ppo_import():
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
        print("✓ All import tests passed!")
        return 0
    else:
        print("✗ Some tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

