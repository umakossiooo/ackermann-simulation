#!/usr/bin/env python3
"""Run training test with the exact command from QUICK_START_TRAINING.md
This script runs the training and monitors for errors and logs."""

import sys
import subprocess
import os
import time

print("=" * 70)
print("Training Test - Running Command from QUICK_START_TRAINING.md")
print("=" * 70)
print()

# Check ROS topics first
print("1. Checking ROS topics...")
try:
    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        capture_output=True,
        text=True,
        timeout=5
    )
    topics = result.stdout
    
    required = ['/odom', '/scan', '/cmd_vel']
    all_found = True
    for topic in required:
        if topic in topics:
            print(f"   ✓ {topic} found")
        else:
            print(f"   ✗ {topic} NOT found")
            all_found = False
    
    if not all_found:
        print("\n   ERROR: Required topics not found!")
        print("   Make sure Gazebo is running with play button on")
        sys.exit(1)
except Exception as e:
    print(f"   ⚠ Could not check topics: {e}")
    print("   Continuing anyway...")

print()
print("2. Running training command...")
print("   Using parameters from QUICK_START_TRAINING.md")
print("   (Reduced to 500 timesteps for testing)")
print()

# Set up command
script_path = "/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/scripts/train_ppo.py"

cmd = [
    'python3', script_path,
    '--total-timesteps', '500',
    '--checkpoint-interval', '250',
    '--learning-rate', '3e-4',
    '--batch-size', '128',
    '--n-steps', '2048',
    '--device', 'auto'
]

print(f"   Command: {' '.join(cmd)}")
print()
print("   Monitoring for:")
print("   - Environment creation")
print("   - Reward logs: [REWARD] Step X | ...")
print("   - Total reward logs")
print("   - Any errors")
print()
print("-" * 70)

# Run training
try:
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        universal_newlines=True
    )
    
    # Monitor output
    reward_logs_found = False
    total_reward_logs_found = False
    errors_found = []
    
    for line in process.stdout:
        print(line, end='')
        
        # Check for reward logs
        if '[REWARD]' in line:
            reward_logs_found = True
            if 'TOTAL REWARD' in line:
                total_reward_logs_found = True
        
        # Check for errors
        if any(keyword in line.lower() for keyword in ['error', 'exception', 'traceback', 'failed']):
            if 'warning' not in line.lower() and 'info' not in line.lower():
                errors_found.append(line.strip())
    
    # Wait for process to complete
    return_code = process.wait()
    
    print()
    print("-" * 70)
    print()
    print("=" * 70)
    print("Test Results")
    print("=" * 70)
    
    if return_code == 0:
        print("✓ Training completed successfully (exit code: 0)")
    else:
        print(f"✗ Training failed (exit code: {return_code})")
    
    if reward_logs_found:
        print("✓ Reward logs are working")
    else:
        print("⚠ No reward logs found (may be normal for very short training)")
    
    if total_reward_logs_found:
        print("✓ Total reward logs are working")
    else:
        print("⚠ No total reward logs found")
    
    if errors_found:
        print(f"\n✗ {len(errors_found)} potential errors found:")
        for error in errors_found[:5]:  # Show first 5
            print(f"   - {error}")
    else:
        print("✓ No errors detected")
    
    print()
    if return_code == 0 and reward_logs_found and not errors_found:
        print("=" * 70)
        print("✓ ALL TESTS PASSED!")
        print("=" * 70)
        print()
        print("Training is working correctly. You can now run full training:")
        print()
        print("python3", script_path, "\\")
        print("    --total-timesteps 1000000 \\")
        print("    --checkpoint-interval 50000 \\")
        print("    --learning-rate 3e-4 \\")
        print("    --batch-size 128 \\")
        print("    --n-steps 2048 \\")
        print("    --device auto")
        print()
        sys.exit(0)
    else:
        print("=" * 70)
        print("⚠ Some issues detected - review output above")
        print("=" * 70)
        sys.exit(1)
        
except KeyboardInterrupt:
    print("\n\nTraining interrupted by user")
    process.terminate()
    sys.exit(130)
except Exception as e:
    print(f"\n\nERROR running training: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

