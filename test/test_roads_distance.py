#!/usr/bin/env python3
"""
Test Roads Distance - Verify distance_to_nearest_road works correctly.

This test must be run inside the Docker container:
    docker compose exec ackermann_sim bash
    python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_roads_distance.py
"""

import sys
import numpy as np
from ackermann_drl.utils.roads_geometry import RoadsGeometry

def test_distance_calculation():
    """Test distance_to_nearest_road function."""
    print("=" * 60)
    print("Roads Distance Test")
    print("=" * 60)
    print()
    
    try:
        # Load roads
        print("Loading roads...")
        roads_geom = RoadsGeometry()
        print(f"✓ Loaded {roads_geom.get_all_roads_count()} roads")
        print()
        
        if roads_geom.get_all_roads_count() == 0:
            print("⚠ No roads loaded. Cannot test distance calculation.")
            return 1
        
        # Test points
        # Use known spawn point coordinates (from Phase 0 analysis)
        # Default spawn: x=169.37, y=0.21 (Via Dante, spawn_point_173)
        test_points = [
            (169.37, 0.21, "Default spawn point (should be on road)"),
            (0.0, 0.0, "Origin (may be off-road)"),
            (100.0, 100.0, "Random point"),
            (200.0, 200.0, "Another random point"),
        ]
        
        print("Testing distance calculations...")
        print("-" * 60)
        
        for x, y, description in test_points:
            distance, nearest_road = roads_geom.distance_to_nearest_road(x, y)
            
            print(f"\nPoint: ({x:.2f}, {y:.2f}) - {description}")
            print(f"  Distance to nearest road: {distance:.3f} meters")
            
            if nearest_road:
                print(f"  Nearest road:")
                print(f"    Name: {nearest_road.get('name', 'Unnamed')}")
                print(f"    Type: {nearest_road.get('highway_type', 'unknown')}")
                print(f"    Way ID: {nearest_road.get('way_id', 'N/A')}")
            else:
                print(f"  ⚠ No nearest road found")
        
        print()
        print("Testing edge cases...")
        print("-" * 60)
        
        # Test very far point
        far_x, far_y = 10000.0, 10000.0
        distance, nearest_road = roads_geom.distance_to_nearest_road(far_x, far_y)
        print(f"\nFar point ({far_x:.1f}, {far_y:.1f}):")
        print(f"  Distance: {distance:.3f} meters")
        if nearest_road:
            print(f"  Nearest road: {nearest_road.get('name', 'Unnamed')}")
        
        # Test negative coordinates
        neg_x, neg_y = -100.0, -100.0
        distance, nearest_road = roads_geom.distance_to_nearest_road(neg_x, neg_y)
        print(f"\nNegative point ({neg_x:.1f}, {neg_y:.1f}):")
        print(f"  Distance: {distance:.3f} meters")
        if nearest_road:
            print(f"  Nearest road: {nearest_road.get('name', 'Unnamed')}")
        
        # Test multiple points to verify consistency
        print()
        print("Testing consistency (10 random points)...")
        print("-" * 60)
        
        np.random.seed(42)  # For reproducibility
        for i in range(10):
            x = np.random.uniform(-500, 500)
            y = np.random.uniform(-500, 500)
            distance, nearest_road = roads_geom.distance_to_nearest_road(x, y)
            
            road_name = nearest_road.get('name', 'Unnamed') if nearest_road else 'None'
            print(f"  Point {i+1}: ({x:7.2f}, {y:7.2f}) -> {distance:6.3f}m to {road_name}")
        
        print()
        print("=" * 60)
        print("✓ All distance tests completed successfully!")
        print("=" * 60)
        return 0
        
    except FileNotFoundError as e:
        print(f"✗ Roads file not found: {e}")
        print("\n  This is expected if osm_city_pipeline is not mounted.")
        print("  To fix: Mount osm_city_pipeline volume in docker-compose.yaml")
        return 1
    except Exception as e:
        print(f"✗ Error in distance test: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(test_distance_calculation())

