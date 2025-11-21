#!/usr/bin/env python3
"""
Test Roads Loading - Verify roads_geometry can load bari_roads.json inside Docker.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_roads_loading.py
"""

import sys
import os
from pathlib import Path

def test_import():
    """Test importing RoadsGeometry."""
    try:
        from ackermann_drl.utils.roads_geometry import RoadsGeometry
        print("✓ RoadsGeometry imported successfully")
        return RoadsGeometry
    except ImportError as e:
        print(f"✗ Failed to import RoadsGeometry: {e}")
        return None
    except Exception as e:
        print(f"✗ Unexpected error importing RoadsGeometry: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_file_paths():
    """Test finding bari_roads.json file."""
    print("\nTesting file path discovery...")
    print("-" * 60)
    
    # Check various possible paths
    possible_paths = [
        Path('/root/colcon_ws/src/osm_city_pipeline/maps/bari_roads.json'),
        Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/../osm_city_pipeline/maps/bari_roads.json'),
        Path('/home/studente/ackermann_sim/src/osm_city_pipeline/maps/bari_roads.json'),
    ]
    
    found_path = None
    for path in possible_paths:
        if path.exists():
            print(f"✓ Found roads file: {path}")
            found_path = path
            break
        else:
            print(f"✗ Not found: {path}")
    
    if found_path is None:
        print("\n⚠ Warning: bari_roads.json not found in expected locations")
        print("  This is OK if osm_city_pipeline is not mounted")
        print("  The test will try to load anyway (may fail)")
    
    return found_path

def test_loading():
    """Test loading roads data."""
    print("\nTesting roads loading...")
    print("-" * 60)
    
    RoadsGeometry = test_import()
    if RoadsGeometry is None:
        return False
    
    try:
        # Try to create RoadsGeometry instance
        # It will try to find the file automatically
        roads_geom = RoadsGeometry()
        print("✓ RoadsGeometry instance created")
        
        # Check if data was loaded
        if hasattr(roads_geom, 'roads_data') and roads_geom.roads_data:
            print("✓ Roads data loaded")
            print(f"  Roads file: {roads_geom.roads_file}")
        else:
            print("✗ Roads data not loaded")
            return False
        
        # Check projection center
        lat, lon, height = roads_geom.get_projection_center()
        print(f"✓ Projection center: ({lat:.6f}, {lon:.6f}, {height:.3f})")
        
        # Check polylines
        if hasattr(roads_geom, 'roads_polylines'):
            count = len(roads_geom.roads_polylines)
            print(f"✓ Polylines built: {count} roads")
            if count == 0:
                print("  ⚠ Warning: No polylines built (may indicate data issue)")
        else:
            print("✗ Polylines not built")
            return False
        
        # Check total roads count
        total_roads = roads_geom.get_all_roads_count()
        print(f"✓ Total roads: {total_roads}")
        
        # Test getting a road by name (if any roads exist)
        if total_roads > 0:
            # Try to find a named road
            for metadata in roads_geom.roads_metadata[:10]:  # Check first 10
                if metadata.get('name'):
                    road_name = metadata['name']
                    road = roads_geom.get_road_by_name(road_name)
                    if road:
                        print(f"✓ Found road by name: {road_name}")
                        break
        
        return True
        
    except FileNotFoundError as e:
        print(f"✗ File not found: {e}")
        print("\n  This is expected if osm_city_pipeline is not mounted.")
        print("  To fix: Mount osm_city_pipeline volume in docker-compose.yaml")
        return False
    except Exception as e:
        print(f"✗ Error loading roads: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all loading tests."""
    print("=" * 60)
    print("Roads Loading Test")
    print("=" * 60)
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Import
    RoadsGeometry = test_import()
    if RoadsGeometry:
        tests_passed += 1
    else:
        tests_failed += 1
        print("\n✗ Cannot continue without RoadsGeometry import")
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

