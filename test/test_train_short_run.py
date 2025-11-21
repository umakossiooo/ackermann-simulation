#!/usr/bin/env python3
"""
Test Train Short Run - Verify PPO training can run for a short duration.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_train_short_run.py

Note: This test requires Gazebo to be running with bari_world.sdf.
"""

import sys
import os
import subprocess
import time
from pathlib import Path

def test_train_short_run():
    """Test that PPO training can run for a short duration."""
    print("=" * 60)
    print("PPO Training Short Run Test")
    print("=" * 60)
    print()
    
    # Check if train_ppo.py exists
    script_path = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py')
    if not script_path.exists():
        print(f"✗ train_ppo.py not found at {script_path}")
        return 1
    
    print(f"✓ Found train_ppo.py at {script_path}")
    
    # Check if script is executable
    if not os.access(script_path, os.X_OK):
        print(f"  Making script executable...")
        os.chmod(script_path, 0o755)
    
    # Create checkpoint and log directories
    checkpoint_dir = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/checkpoints')
    log_dir = Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/logs')
    
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"✓ Checkpoint directory: {checkpoint_dir}")
    print(f"✓ Log directory: {log_dir}")
    print()
    
    # Test with very short training (100 steps)
    print("Testing short training run (100 steps)...")
    print("-" * 60)
    
    try:
        # Run training script with minimal timesteps
        cmd = [
            'python3',
            str(script_path),
            '--total-timesteps', '100',
            '--checkpoint-interval', '50',
            '--checkpoint-dir', str(checkpoint_dir),
            '--log-dir', str(log_dir),
            '--n-steps', '32',  # Small n_steps for quick test
            '--batch-size', '16',  # Small batch size
            '--n-epochs', '2',  # Few epochs
        ]
        
        print(f"Running command: {' '.join(cmd)}")
        print()
        
        # Run with timeout (60 seconds)
        result = subprocess.run(
            cmd,
            cwd=str(script_path.parent),
            timeout=60,
            capture_output=True,
            text=True
        )
        
        print("Training output:")
        print(result.stdout)
        
        if result.stderr:
            print("Training errors/warnings:")
            print(result.stderr)
        
        if result.returncode == 0:
            print()
            print("✓ Training completed successfully!")
            
            # Check if checkpoint was created
            checkpoint_files = list(checkpoint_dir.glob('ppo_ackermann_*'))
            if checkpoint_files:
                print(f"✓ Checkpoints created: {len(checkpoint_files)} files")
                for f in checkpoint_files[:3]:  # Show first 3
                    print(f"  - {f.name}")
            else:
                print("⚠ No checkpoints found (may be normal for very short run)")
            
            # Check if logs were created
            log_files = list(log_dir.glob('*.monitor.csv'))
            if log_files:
                print(f"✓ Log files created: {len(log_files)} files")
            else:
                print("⚠ No log files found (may be normal for very short run)")
            
            return 0
        else:
            print()
            print(f"✗ Training failed with return code {result.returncode}")
            if result.stderr:
                print("Error output:")
                print(result.stderr)
            return 1
    
    except subprocess.TimeoutExpired:
        print()
        print("✗ Training timed out (exceeded 60 seconds)")
        return 1
    
    except KeyboardInterrupt:
        print()
        print("✗ Test interrupted by user")
        return 1
    
    except Exception as e:
        print()
        print(f"✗ Error running training: {e}")
        import traceback
        traceback.print_exc()
        return 1

def test_imports():
    """Test that all required imports work."""
    print()
    print("Testing imports...")
    print("-" * 60)
    
    try:
        import torch
        print(f"✓ torch: {torch.__version__}")
    except Exception as e:
        print(f"✗ torch import failed: {e}")
        return False
    
    try:
        from stable_baselines3 import PPO
        print("✓ stable-baselines3 PPO")
    except Exception as e:
        print(f"✗ stable-baselines3 import failed: {e}")
        return False
    
    try:
        from ackermann_drl.envs.gym_wrapper import AckermannGymEnv
        print("✓ AckermannGymEnv")
    except Exception as e:
        print(f"✗ AckermannGymEnv import failed: {e}")
        return False
    
    return True

def main():
    """Run all tests."""
    # First test imports
    if not test_imports():
        print()
        print("✗ Import tests failed. Cannot proceed with training test.")
        return 1
    
    # Then test short training run
    result = test_train_short_run()
    
    print()
    print("=" * 60)
    if result == 0:
        print("✓ All tests passed!")
    else:
        print("✗ Some tests failed.")
    print("=" * 60)
    
    return result

if __name__ == '__main__':
    sys.exit(main())

