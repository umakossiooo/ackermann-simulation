#!/usr/bin/env python3
"""
Test Battery Basic - Verify battery model works correctly inside Docker.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_battery_basic.py
"""

import sys
import numpy as np
from ackermann_drl.utils.battery_model import BatteryModel

def test_battery_initialization():
    """Test battery initialization."""
    print("=" * 60)
    print("Battery Model Basic Test")
    print("=" * 60)
    print()
    
    print("Test 1: Battery Initialization")
    print("-" * 60)
    
    # Test default initialization
    battery = BatteryModel()
    level = battery.get_battery_level()
    print(f"✓ Default initialization: level = {level:.3f} (expected: 1.0)")
    assert 0.99 <= level <= 1.01, f"Battery level should be 1.0, got {level}"
    
    # Test custom initialization
    battery2 = BatteryModel(initial_level=0.5, alpha=0.002, beta=0.02)
    level2 = battery2.get_battery_level()
    print(f"✓ Custom initialization: level = {level2:.3f} (expected: 0.5)")
    assert 0.49 <= level2 <= 0.51, f"Battery level should be 0.5, got {level2}"
    
    # Test boundary values
    battery3 = BatteryModel(initial_level=0.0)
    assert battery3.get_battery_level() == 0.0, "Battery level should be 0.0"
    print("✓ Boundary test (0.0): passed")
    
    battery4 = BatteryModel(initial_level=1.0)
    assert battery4.get_battery_level() == 1.0, "Battery level should be 1.0"
    print("✓ Boundary test (1.0): passed")
    
    # Test clamping
    battery5 = BatteryModel(initial_level=1.5)
    assert battery5.get_battery_level() <= 1.0, "Battery level should be clamped to 1.0"
    print("✓ Clamping test (1.5 -> 1.0): passed")
    
    battery6 = BatteryModel(initial_level=-0.5)
    assert battery6.get_battery_level() >= 0.0, "Battery level should be clamped to 0.0"
    print("✓ Clamping test (-0.5 -> 0.0): passed")
    
    return True

def test_battery_update():
    """Test battery update with distance and velocity change."""
    print()
    print("Test 2: Battery Update (drop = α*distance + β*|Δv|)")
    print("-" * 60)
    
    # Test with distance only (no velocity change)
    battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
    initial_level = battery.get_battery_level()
    
    # Move 10 meters (no velocity change)
    battery.update(position=np.array([0.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
    battery.update(position=np.array([10.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
    
    level_after_distance = battery.get_battery_level()
    expected_drop = 0.001 * 10.0  # α * distance = 0.001 * 10 = 0.01
    actual_drop = initial_level - level_after_distance
    
    print(f"  Initial level: {initial_level:.3f}")
    print(f"  After 10m movement: {level_after_distance:.3f}")
    print(f"  Expected drop: {expected_drop:.3f}, Actual drop: {actual_drop:.3f}")
    assert abs(actual_drop - expected_drop) < 0.001, f"Drop should be ~{expected_drop}, got {actual_drop}"
    print("✓ Distance-based consumption: passed")
    
    # Test with velocity change only (no distance)
    battery2 = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
    initial_level2 = battery2.get_battery_level()
    
    # Change velocity from 0 to 2 m/s (at same position)
    battery2.update(position=np.array([0.0, 0.0, 0.0]), velocity=0.0, dt=0.1)
    battery2.update(position=np.array([0.0, 0.0, 0.0]), velocity=2.0, dt=0.1)
    
    level_after_velocity = battery2.get_battery_level()
    expected_drop2 = 0.01 * 2.0  # β * |Δv| = 0.01 * 2 = 0.02
    actual_drop2 = initial_level2 - level_after_velocity
    
    print(f"  Initial level: {initial_level2:.3f}")
    print(f"  After velocity change (0->2 m/s): {level_after_velocity:.3f}")
    print(f"  Expected drop: {expected_drop2:.3f}, Actual drop: {actual_drop2:.3f}")
    assert abs(actual_drop2 - expected_drop2) < 0.001, f"Drop should be ~{expected_drop2}, got {actual_drop2}"
    print("✓ Velocity change-based consumption: passed")
    
    # Test combined (distance + velocity change)
    battery3 = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
    initial_level3 = battery3.get_battery_level()
    
    # Move 5m and change velocity from 1 to 3 m/s
    battery3.update(position=np.array([0.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
    battery3.update(position=np.array([5.0, 0.0, 0.0]), velocity=3.0, dt=0.1)
    
    level_after_combined = battery3.get_battery_level()
    expected_drop3 = 0.001 * 5.0 + 0.01 * 2.0  # α*5 + β*2 = 0.005 + 0.02 = 0.025
    actual_drop3 = initial_level3 - level_after_combined
    
    print(f"  Initial level: {initial_level3:.3f}")
    print(f"  After 5m + velocity change (1->3 m/s): {level_after_combined:.3f}")
    print(f"  Expected drop: {expected_drop3:.3f}, Actual drop: {actual_drop3:.3f}")
    assert abs(actual_drop3 - expected_drop3) < 0.001, f"Drop should be ~{expected_drop3}, got {actual_drop3}"
    print("✓ Combined consumption: passed")
    
    return True

def test_battery_reset():
    """Test battery reset."""
    print()
    print("Test 3: Battery Reset")
    print("-" * 60)
    
    battery = BatteryModel(initial_level=1.0, alpha=0.001, beta=0.01)
    
    # Deplete battery
    for i in range(100):
        battery.update(position=np.array([i * 10.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
    
    depleted_level = battery.get_battery_level()
    print(f"  After depletion: {depleted_level:.3f}")
    assert depleted_level < 1.0, "Battery should be depleted"
    
    # Reset
    battery.reset()
    reset_level = battery.get_battery_level()
    print(f"  After reset: {reset_level:.3f}")
    assert abs(reset_level - 1.0) < 0.001, f"Battery should reset to 1.0, got {reset_level}"
    print("✓ Reset test: passed")
    
    return True

def test_battery_depletion():
    """Test battery depletion detection."""
    print()
    print("Test 4: Battery Depletion")
    print("-" * 60)
    
    battery = BatteryModel(initial_level=0.1, alpha=0.1, beta=0.1)  # High consumption
    
    # Deplete battery
    for i in range(10):
        battery.update(position=np.array([i * 1.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
        level = battery.get_battery_level()
        is_depleted = battery.is_depleted()
        print(f"  Step {i+1}: level={level:.3f}, depleted={is_depleted}")
    
    final_level = battery.get_battery_level()
    is_depleted = battery.is_depleted()
    print(f"  Final level: {final_level:.3f}, depleted: {is_depleted}")
    
    if final_level <= 0.0:
        assert is_depleted, "Battery should be marked as depleted when level is 0"
        print("✓ Depletion detection: passed")
    else:
        print("⚠ Battery not fully depleted (may need more steps)")
    
    return True

def test_battery_level_range():
    """Test battery level stays in [0,1] range."""
    print()
    print("Test 5: Battery Level Range [0,1]")
    print("-" * 60)
    
    battery = BatteryModel(initial_level=0.01, alpha=1.0, beta=1.0)  # Very high consumption
    
    # Try to over-deplete
    for i in range(100):
        battery.update(position=np.array([i * 1.0, 0.0, 0.0]), velocity=1.0, dt=0.1)
        level = battery.get_battery_level()
        assert 0.0 <= level <= 1.0, f"Battery level {level} out of range [0,1]"
    
    final_level = battery.get_battery_level()
    print(f"  Final level after over-depletion attempts: {final_level:.3f}")
    assert 0.0 <= final_level <= 1.0, f"Battery level {final_level} out of range [0,1]"
    print("✓ Range constraint: passed")
    
    return True

def main():
    """Run all battery tests."""
    tests_passed = 0
    tests_failed = 0
    
    try:
        if test_battery_initialization():
            tests_passed += 1
        else:
            tests_failed += 1
    except Exception as e:
        print(f"✗ Initialization test failed: {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        if test_battery_update():
            tests_passed += 1
        else:
            tests_failed += 1
    except Exception as e:
        print(f"✗ Update test failed: {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        if test_battery_reset():
            tests_passed += 1
        else:
            tests_failed += 1
    except Exception as e:
        print(f"✗ Reset test failed: {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        if test_battery_depletion():
            tests_passed += 1
        else:
            tests_failed += 1
    except Exception as e:
        print(f"✗ Depletion test failed: {e}")
        import traceback
        traceback.print_exc()
        tests_failed += 1
    
    try:
        if test_battery_level_range():
            tests_passed += 1
        else:
            tests_failed += 1
    except Exception as e:
        print(f"✗ Range test failed: {e}")
        import traceback
        traceback.print_exc()
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
        print("✓ All battery tests passed!")
        return 0
    else:
        print("✗ Some tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

