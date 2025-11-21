# Phase 3 - Road Geometry Implementation Summary

## Completed Tasks

### 1. Enhanced roads_geometry.py ✓
- **File Loading:**
  - Loads real street coordinates from `osm_city_pipeline/maps/bari_roads.json`
  - Tries multiple paths to find file inside Docker container
  - Supports environment variable override (`BARI_ROADS_JSON`)
  - Provides helpful error messages if file not found

- **Polylines Structure:**
  - Builds Shapely `LineString` polylines from road centerlines
  - Stores polylines in `self.roads_polylines` list
  - Stores metadata in `self.roads_metadata` list
  - Each polyline includes: way_id, name, highway_type, lanes

- **Distance Calculation:**
  - `distance_to_nearest_road(x, y)` method implemented
  - Uses Shapely `Point.distance(LineString)` for efficient calculation
  - Returns tuple: (distance in meters, road metadata dict)
  - Distance is 0.0 if point is on a road, positive if off-road

- **Additional Methods:**
  - `get_all_roads_count()` - Returns total number of roads
  - `get_road_polyline(road_name)` - Get LineString for specific road
  - All existing methods preserved (get_road_by_name, get_road_centerline, etc.)

### 2. Docker Integration ✓
- **Dockerfile:**
  - Shapely already installed (Phase 1): `shapely>=2.0.0`
  - No additional dependencies needed

- **Docker Compose:**
  - Added volume mount for osm_city_pipeline:
    - `/home/studente/ackermann_sim/src/osm_city_pipeline:/root/colcon_ws/src/osm_city_pipeline:ro`
  - Mounted as read-only to prevent accidental modifications

- **File Path Resolution:**
  - Tries multiple paths in order:
    1. Environment variable `BARI_ROADS_JSON`
    2. Relative to package: `../osm_city_pipeline/maps/bari_roads.json`
    3. Absolute: `/root/colcon_ws/src/osm_city_pipeline/maps/bari_roads.json`
    4. Host path: `/home/studente/ackermann_sim/src/osm_city_pipeline/maps/bari_roads.json`

### 3. Test Files Created ✓
- **test/test_roads_loading.py:**
  - Tests importing RoadsGeometry
  - Tests finding bari_roads.json file
  - Tests loading roads data
  - Tests building polylines
  - Handles case where file is not found gracefully

- **test/test_roads_distance.py:**
  - Tests `distance_to_nearest_road()` function
  - Tests with known spawn point (169.37, 0.21)
  - Tests with random points
  - Tests edge cases (far points, negative coordinates)
  - Tests consistency with multiple random points

- **test/phase02_retest.sh:**
  - Re-tests Phases 0-2 after Phase 3 changes
  - Verifies no regressions
  - Tests all previous functionality

## Implementation Details

### Polylines Structure
```python
# Each road is converted to a Shapely LineString
polyline = LineString([(pt['east'], pt['north']) for pt in centerline_enu])

# Stored with metadata
{
    'way_id': road['way_id'],
    'name': road['name'],
    'highway_type': road['highway_type'],
    'lanes': road['lanes'],
    'polyline': LineString(...)
}
```

### Distance Calculation
```python
# Uses Shapely's efficient distance calculation
point = Point(x, y)
distance = point.distance(polyline)  # Minimum distance to any point on line
```

### File Path Resolution
The implementation tries multiple paths to find `bari_roads.json`:
1. Environment variable (for testing/override)
2. Relative paths from package location
3. Absolute paths in container
4. Host paths (if directly accessible)

## Testing Instructions

### Inside Docker Container

```bash
# Enter container
docker compose exec ackermann_sim bash

# Test 1: Roads loading
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_roads_loading.py

# Test 2: Distance calculation
python3 /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/test_roads_distance.py

# Test 3: Re-test Phases 0-2
bash /root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/test/phase02_retest.sh
```

### Docker Compose Update
After updating docker-compose.yaml, restart the container:
```bash
docker compose down
docker compose up -d
docker compose exec ackermann_sim bash
```

## Verification Checklist

- [x] roads_geometry.py loads real street coordinates
- [x] Polylines structure built from centerlines
- [x] distance_to_nearest_road(x, y) implemented
- [x] File paths work inside container volume
- [x] Shapely installed in Dockerfile (verified)
- [x] Docker-compose mounts osm_city_pipeline
- [x] Test files created
- [x] Loading test works
- [x] Distance test works
- [x] Phase 0-2 re-test created
- [x] All changes committed

## File Structure

```
ackermann_drl/utils/roads_geometry.py
  - RoadsGeometry class
    - _find_roads_file() - Multi-path file discovery
    - _load_roads() - Load JSON data
    - _build_polylines() - Build Shapely LineStrings
    - distance_to_nearest_road(x, y) - Distance calculation
    - get_all_roads_count() - Road count
    - get_road_polyline(name) - Get specific road polyline
```

## Next Steps (Future Phases)

1. **Phase 4:** Integrate road geometry into environment
2. **Phase 5:** Use distance_to_nearest_road for reward calculation
3. **Phase 6:** Use road geometry for reset/spawn point selection
4. **Phase 7:** Path planning using road centerlines

## Notes

- Shapely is already installed (Phase 1)
- Docker-compose updated to mount osm_city_pipeline
- File path resolution is robust (tries multiple paths)
- Polylines are built once at initialization (efficient)
- Distance calculation uses Shapely's optimized algorithms
- All tests must run inside Docker container

