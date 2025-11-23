#!/usr/bin/env python3
"""System readiness test - verifies all components are ready for use.

Run this inside Docker container to test the system with Gazebo running.
"""

import sys
import time
from pathlib import Path

def test_imports():
    """Test that all required modules can be imported."""
    print("=" * 70)
    print("1. Testing Imports")
    print("=" * 70)
    
    errors = []
    
    # Test ROS 2 imports
    try:
        import rclpy
        print("✓ rclpy imported")
    except ImportError as e:
        errors.append(f"rclpy: {e}")
        print(f"✗ rclpy: {e}")
    
    try:
        from geometry_msgs.msg import Twist
        print("✓ geometry_msgs imported")
    except ImportError as e:
        errors.append(f"geometry_msgs: {e}")
        print(f"✗ geometry_msgs: {e}")
    
    try:
        from nav_msgs.msg import Odometry
        print("✓ nav_msgs imported")
    except ImportError as e:
        errors.append(f"nav_msgs: {e}")
        print(f"✗ nav_msgs: {e}")
    
    try:
        from sensor_msgs.msg import LaserScan
        print("✓ sensor_msgs imported")
    except ImportError as e:
        errors.append(f"sensor_msgs: {e}")
        print(f"✗ sensor_msgs: {e}")
    
    # Test ackermann_drl imports
    try:
        from ackermann_drl.utils.battery_model import BatteryModel
        print("✓ BatteryModel imported")
    except ImportError as e:
        errors.append(f"BatteryModel: {e}")
        print(f"✗ BatteryModel: {e}")
    
    try:
        from ackermann_drl.utils.sliding_mode_control import SlidingModeController
        print("✓ SlidingModeController imported")
    except ImportError as e:
        errors.append(f"SlidingModeController: {e}")
        print(f"✗ SlidingModeController: {e}")
    
    try:
        from ackermann_drl.utils.roads_geometry import RoadsGeometry
        print("✓ RoadsGeometry imported")
    except ImportError as e:
        errors.append(f"RoadsGeometry: {e}")
        print(f"✗ RoadsGeometry: {e}")
    
    try:
        from ackermann_drl.utils.delivery_points import DeliveryPoints
        print("✓ DeliveryPoints imported")
    except ImportError as e:
        errors.append(f"DeliveryPoints: {e}")
        print(f"✗ DeliveryPoints: {e}")
    
    print()
    return errors

def test_battery_model():
    """Test battery model functionality."""
    print("=" * 70)
    print("2. Testing Battery Model")
    print("=" * 70)
    
    errors = []
    
    try:
        from ackermann_drl.utils.battery_model import BatteryModel
        
        # Test initialization with weight
        battery = BatteryModel(vehicle_weight=1500.0)
        weight = battery.get_vehicle_weight()
        factor = battery.get_weight_factor()
        
        if abs(weight - 1500.0) < 0.1:
            print(f"✓ Battery weight: {weight} kg")
        else:
            errors.append(f"Battery weight incorrect: {weight}")
            print(f"✗ Battery weight incorrect: {weight}")
        
        if abs(factor - 1.5) < 0.01:
            print(f"✓ Weight factor: {factor:.2f}")
        else:
            errors.append(f"Weight factor incorrect: {factor}")
            print(f"✗ Weight factor incorrect: {factor}")
        
        # Test update
        battery.update(np.array([0.0, 0.0, 0.0]), 0.0)
        battery.update(np.array([1.0, 0.0, 0.0]), 1.0)
        level = battery.get_battery_level()
        
        if 0.0 <= level <= 1.0:
            print(f"✓ Battery update works: level = {level:.4f}")
        else:
            errors.append(f"Battery level out of range: {level}")
            print(f"✗ Battery level out of range: {level}")
        
    except Exception as e:
        errors.append(f"Battery model test failed: {e}")
        print(f"✗ Battery model test failed: {e}")
    
    print()
    return errors

def test_smc_controller():
    """Test SMC controller functionality."""
    print("=" * 70)
    print("3. Testing SMC Controller")
    print("=" * 70)
    
    errors = []
    
    try:
        from ackermann_drl.utils.sliding_mode_control import SlidingModeController
        
        smc = SlidingModeController(
            lambda_param=1.0,
            K_smc=2.0,
            boundary_layer=0.1,
            desired_velocity=2.0
        )
        
        # Test control computation
        steering, velocity = smc.compute_control(
            lateral_error=0.5,
            heading_error=0.2,
            current_velocity=1.5
        )
        
        if -1.0 <= steering <= 1.0:
            print(f"✓ SMC steering output: {steering:.4f} (within bounds)")
        else:
            errors.append(f"SMC steering out of bounds: {steering}")
            print(f"✗ SMC steering out of bounds: {steering}")
        
        if 0.0 <= velocity <= 5.0:
            print(f"✓ SMC velocity output: {velocity:.2f} m/s (within bounds)")
        else:
            errors.append(f"SMC velocity out of bounds: {velocity}")
            print(f"✗ SMC velocity out of bounds: {velocity}")
        
    except Exception as e:
        errors.append(f"SMC controller test failed: {e}")
        print(f"✗ SMC controller test failed: {e}")
    
    print()
    return errors

def test_roads_geometry():
    """Test roads geometry (may fail if roads file not available)."""
    print("=" * 70)
    print("4. Testing Roads Geometry")
    print("=" * 70)
    
    errors = []
    warnings = []
    
    try:
        from ackermann_drl.utils.roads_geometry import RoadsGeometry
        
        roads = RoadsGeometry()
        count = roads.get_all_roads_count()
        
        if count > 0:
            print(f"✓ Roads loaded: {count} roads")
            
            # Test distance calculation
            distance, _ = roads.distance_to_nearest_road(169.37, 0.21)
            if distance >= 0.0:
                print(f"✓ Distance calculation works: {distance:.3f}m")
            else:
                errors.append(f"Distance calculation failed: {distance}")
                print(f"✗ Distance calculation failed: {distance}")
            
            # Test heading calculation
            heading, _ = roads.get_road_heading_at_point(169.37, 0.21)
            if -3.15 <= heading <= 3.15:  # Roughly -π to π
                print(f"✓ Heading calculation works: {heading:.3f} rad")
            else:
                warnings.append(f"Heading may be incorrect: {heading}")
                print(f"⚠ Heading may be incorrect: {heading}")
        else:
            warnings.append("No roads loaded (may be OK if roads file not mounted)")
            print("⚠ No roads loaded (may be OK if roads file not mounted)")
        
    except FileNotFoundError as e:
        warnings.append(f"Roads file not found: {e}")
        print(f"⚠ Roads file not found (may be OK): {e}")
    except Exception as e:
        errors.append(f"Roads geometry test failed: {e}")
        print(f"✗ Roads geometry test failed: {e}")
    
    print()
    return errors, warnings

def test_delivery_points():
    """Test delivery points loading."""
    print("=" * 70)
    print("5. Testing Delivery Points")
    print("=" * 70)
    
    errors = []
    warnings = []
    
    try:
        from ackermann_drl.utils.delivery_points import DeliveryPoints
        
        points = DeliveryPoints()
        count = points.get_total_points()
        
        if count > 0:
            print(f"✓ Delivery points loaded: {count} points")
            
            # Test getting first point
            first_point = points.get_next_point(None)
            if first_point is not None:
                print(f"✓ First point: {first_point.get('name', 'unknown')}")
            else:
                errors.append("Cannot get first delivery point")
                print("✗ Cannot get first delivery point")
        else:
            warnings.append("No delivery points loaded")
            print("⚠ No delivery points loaded")
        
    except Exception as e:
        errors.append(f"Delivery points test failed: {e}")
        print(f"✗ Delivery points test failed: {e}")
    
    print()
    return errors, warnings

def main():
    """Run all tests."""
    print("\n" + "=" * 70)
    print("System Readiness Test")
    print("=" * 70)
    print()
    
    all_errors = []
    all_warnings = []
    
    # Test imports
    import_errors = test_imports()
    all_errors.extend(import_errors)
    
    if import_errors:
        print("⚠ Cannot continue tests due to import errors")
        print("\n" + "=" * 70)
        print("Test Summary")
        print("=" * 70)
        print(f"Errors: {len(all_errors)}")
        print(f"Warnings: {len(all_warnings)}")
        if all_errors:
            print("\nErrors found:")
            for e in all_errors:
                print(f"  ✗ {e}")
        return 1
    
    # Test battery model
    battery_errors = test_battery_model()
    all_errors.extend(battery_errors)
    
    # Test SMC controller
    smc_errors = test_smc_controller()
    all_errors.extend(smc_errors)
    
    # Test roads geometry
    roads_errors, roads_warnings = test_roads_geometry()
    all_errors.extend(roads_errors)
    all_warnings.extend(roads_warnings)
    
    # Test delivery points
    points_errors, points_warnings = test_delivery_points()
    all_errors.extend(points_errors)
    all_warnings.extend(points_warnings)
    
    # Summary
    print("=" * 70)
    print("Test Summary")
    print("=" * 70)
    print(f"Errors: {len(all_errors)}")
    print(f"Warnings: {len(all_warnings)}")
    print()
    
    if all_errors:
        print("Errors found:")
        for e in all_errors:
            print(f"  ✗ {e}")
        print()
    
    if all_warnings:
        print("Warnings:")
        for w in all_warnings:
            print(f"  ⚠ {w}")
        print()
    
    if not all_errors:
        print("✓ All critical tests passed!")
        print("✓ System is ready for use")
        if all_warnings:
            print("⚠ Some warnings present (may be OK)")
        return 0
    else:
        print("✗ Some tests failed")
        return 1

if __name__ == '__main__':
    # Add numpy import for battery test
    try:
        import numpy as np
    except ImportError:
        print("⚠ numpy not available (tests may fail)")
        np = None
    
    sys.exit(main())

