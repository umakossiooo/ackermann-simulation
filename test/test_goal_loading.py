#!/usr/bin/env python3
"""
Test Goal Loading - Verify delivery_points.yaml can be loaded inside Docker.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_goal_loading.py
"""

import sys
import yaml
from pathlib import Path
from ackermann_drl.utils.delivery_points import DeliveryPoints

def test_import():
    """Test importing DeliveryPoints."""
    try:
        from ackermann_drl.utils.delivery_points import DeliveryPoints
        print("✓ DeliveryPoints imported successfully")
        return DeliveryPoints
    except ImportError as e:
        print(f"✗ Failed to import DeliveryPoints: {e}")
        return None
    except Exception as e:
        print(f"✗ Unexpected error importing DeliveryPoints: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_file_paths():
    """Test finding delivery_points.yaml file."""
    print("\nTesting file path discovery...")
    print("-" * 60)
    
    # Check various possible paths
    possible_paths = [
        Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/delivery_points.yaml'),
        Path('/root/colcon_ws/install/ackermann_drl/share/ackermann_drl/config/delivery_points.yaml'),
    ]
    
    found_path = None
    for path in possible_paths:
        if path.exists():
            print(f"✓ Found delivery points file: {path}")
            found_path = path
            break
        else:
            print(f"✗ Not found: {path}")
    
    if found_path is None:
        print("\n⚠ Warning: delivery_points.yaml not found in expected locations")
        print("  The test will try to load anyway (may fail)")
    
    return found_path

def test_loading():
    """Test loading delivery points."""
    print("\nTesting delivery points loading...")
    print("-" * 60)
    
    DeliveryPoints = test_import()
    if DeliveryPoints is None:
        return False
    
    try:
        # Try to create DeliveryPoints instance
        delivery_points = DeliveryPoints()
        print("✓ DeliveryPoints instance created")
        
        # Check if points were loaded
        count = delivery_points.get_point_count()
        print(f"✓ Delivery points loaded: {count} points")
        
        if count == 0:
            print("  ⚠ Warning: No delivery points loaded (may indicate data issue)")
            return False
        
        # Get all points
        all_points = delivery_points.get_all_points()
        print(f"✓ Retrieved all points: {len(all_points)}")
        
        # Test getting point by ID
        if count > 0:
            first_point = all_points[0]
            point_id = first_point.get('id')
            retrieved = delivery_points.get_point_by_id(point_id)
            if retrieved:
                print(f"✓ Retrieved point by ID {point_id}: {retrieved.get('name', 'unnamed')}")
            else:
                print(f"✗ Failed to retrieve point by ID {point_id}")
                return False
        
        # Test getting random point
        random_point = delivery_points.get_random_point()
        print(f"✓ Retrieved random point: {random_point.get('name', 'unnamed')}")
        
        # Test getting position
        pos = delivery_points.get_point_position(random_point)
        print(f"✓ Extracted position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        
        # Display all points
        print("\nAll delivery points:")
        for point in all_points:
            pos = delivery_points.get_point_position(point)
            print(f"  ID {point.get('id')}: {point.get('name', 'unnamed')} "
                  f"at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        
        return True
        
    except FileNotFoundError as e:
        print(f"✗ File not found: {e}")
        print("\n  This is expected if delivery_points.yaml is not in the expected location.")
        print("  Ensure the file exists in ackermann_drl/config/delivery_points.yaml")
        return False
    except Exception as e:
        print(f"✗ Error loading delivery points: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all loading tests."""
    print("=" * 60)
    print("Goal Loading Test")
    print("=" * 60)
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Import
    DeliveryPoints = test_import()
    if DeliveryPoints:
        tests_passed += 1
    else:
        tests_failed += 1
        print("\n✗ Cannot continue without DeliveryPoints import")
        return 1
    
    # Test 2: File paths
    found_path = test_file_paths()
    if found_path:
        tests_passed += 1
    else:
        tests_passed += 1  # Not a failure, just a warning
    
    # Test 3: Loading
    if test_loading():
        tests_passed += 1
    else:
        tests_failed += 1
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"Passed: {tests_passed}")
    print(f"Failed: {tests_failed}")
    print()
    
    if tests_failed == 0:
        print("✓ All loading tests passed!")
        return 0
    else:
        print("✗ Some tests failed. Check output above.")
        return 1

if __name__ == '__main__':
    sys.exit(main())

