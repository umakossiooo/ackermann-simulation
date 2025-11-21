#!/usr/bin/env python3
"""
Phase 1 Import Test - Verify ackermann_drl package can be imported inside Docker.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase1_import_test.py
"""

import sys
import importlib

def test_import(module_name, description):
    """Test importing a module."""
    try:
        module = importlib.import_module(module_name)
        print(f"✓ {description}: {module_name}")
        return True
    except ImportError as e:
        print(f"✗ {description}: {module_name} - {e}")
        return False
    except Exception as e:
        print(f"✗ {description}: {module_name} - Unexpected error: {e}")
        return False

def main():
    """Run all import tests."""
    print("=" * 60)
    print("Phase 1 - ackermann_drl Package Import Test")
    print("=" * 60)
    print()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test DRL dependencies
    print("Testing DRL Dependencies:")
    print("-" * 60)
    if test_import("gymnasium", "Gymnasium"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("stable_baselines3", "Stable-Baselines3"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("torch", "PyTorch"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("numpy", "NumPy"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("shapely", "Shapely"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    print()
    
    # Test ROS 2 dependencies
    print("Testing ROS 2 Dependencies:")
    print("-" * 60)
    if test_import("rclpy", "rclpy"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("sensor_msgs", "sensor_msgs"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("nav_msgs", "nav_msgs"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("geometry_msgs", "geometry_msgs"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    print()
    
    # Test ackermann_drl package
    print("Testing ackermann_drl Package:")
    print("-" * 60)
    if test_import("ackermann_drl", "ackermann_drl (main package)"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.envs", "ackermann_drl.envs"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.envs.ackermann_city_env", "AckermannCityEnv"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.utils", "ackermann_drl.utils"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.utils.roads_geometry", "RoadsGeometry"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.utils.battery_model", "BatteryModel"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    if test_import("ackermann_drl.utils.reset_helpers", "ResetHelpers"):
        tests_passed += 1
    else:
        tests_failed += 1
    
    print()
    
    # Summary
    print("=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"Passed: {tests_passed}")
    print(f"Failed: {tests_failed}")
    print(f"Total:  {tests_passed + tests_failed}")
    print()
    
    if tests_failed == 0:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

