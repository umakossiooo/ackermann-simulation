# Delivery Points Guide

## How Delivery Points Are Defined

Delivery points are **goal locations** where the robot should navigate to. They are currently defined manually in a YAML configuration file.

## Current Definition

### File Location
`ackermann_drl/config/delivery_points.yaml`

### Structure
```yaml
delivery_points:
  - id: 0
    name: "delivery_point_0"
    position:
      east: 169.37      # East coordinate (meters)
      north: 0.21       # North coordinate (meters)
      up: 0.0           # Height (meters)
    description: "Default spawn location"
  
  - id: 1
    name: "delivery_point_1"
    position:
      east: 200.0
      north: 50.0
      up: 0.0
    description: "Example delivery point 1"
```

### Coordinate System
- **ENU (East-North-Up)**: Same coordinate system as the Gazebo world
- **East (X)**: Positive = eastward
- **North (Y)**: Positive = northward  
- **Up (Z)**: Height above ground (usually 0.0)

## How They're Used

### In Training
1. On each `reset()`, a random delivery point is selected as the goal
2. The robot must navigate to that goal
3. Success is measured by reaching within 2.0 meters of the goal

### In Code
```python
from ackermann_drl.utils.delivery_points import DeliveryPoints

# Load delivery points
delivery_points = DeliveryPoints()

# Get random goal
goal = delivery_points.get_random_point()

# Get position
east, north, up = delivery_points.get_point_position(goal)
```

## Adding More Delivery Points

### Manual Method (Current)

Edit `ackermann_drl/config/delivery_points.yaml`:

```yaml
delivery_points:
  - id: 0
    name: "delivery_point_0"
    position:
      east: 169.37
      north: 0.21
      up: 0.0
    description: "Default spawn location"
  
  - id: 3
    name: "delivery_point_3"
    position:
      east: 180.0
      north: 30.0
      up: 0.0
    description: "New delivery point"
  
  # Add more points...
```

### Using OSM Spawn Points (Future Enhancement)

You could generate delivery points from `osm_city_pipeline/maps/bari_spawn_points.yaml`:

```python
# Example: Convert spawn points to delivery points
import yaml

with open('bari_spawn_points.yaml', 'r') as f:
    spawn_data = yaml.safe_load(f)

delivery_points = []
for sp in spawn_data['spawn_points']:
    delivery_points.append({
        'id': sp['id'],
        'name': f"delivery_{sp['name']}",
        'position': sp['position'],  # Already has east, north, up
        'description': f"On {sp.get('road_name', 'unknown')} street"
    })
```

## Finding Good Delivery Point Locations

### Method 1: Use Spawn Points
Spawn points are already on roads, so they make good delivery points:

```bash
# View available spawn points
cat /home/studente/ackermann_sim/src/osm_city_pipeline/maps/bari_spawn_points.yaml | head -50
```

### Method 2: Use Road Centerlines
Extract points from road centerlines in `bari_roads.json`:

```python
from ackermann_drl.utils.roads_geometry import RoadsGeometry

roads = RoadsGeometry()
# Get points along roads
for road in roads.get_all_roads():
    centerline = roads.get_road_centerline(road['name'])
    # Use points from centerline as delivery points
```

### Method 3: Manual Selection
1. Launch Gazebo with bari_world.sdf
2. Use RViz or Gazebo to find interesting locations
3. Note the ENU coordinates
4. Add to delivery_points.yaml

## Best Practices

### 1. Place on Roads
- Delivery points should be on or near roads
- Use `roads_geometry.distance_to_nearest_road()` to verify

### 2. Good Distribution
- Spread points across the map
- Include different road types (primary, secondary, etc.)
- Avoid clustering

### 3. Meaningful Names
- Use descriptive names
- Include road name in description
- Number sequentially

### 4. Coordinate Accuracy
- Use 2-3 decimal places (sufficient for meters)
- Ensure `up` is usually 0.0 (ground level)
- Verify coordinates match Gazebo world

## Current Status

**Current delivery points:** 3 (example points)
- `delivery_point_0`: Default spawn (169.37, 0.21)
- `delivery_point_1`: Example (200.0, 50.0)
- `delivery_point_2`: Example (150.0, -30.0)

**Recommendation:** Add more delivery points from `bari_spawn_points.yaml` for better training diversity.

## Example: Generate from Spawn Points

Here's a script to generate delivery points from spawn points:

```python
#!/usr/bin/env python3
"""Generate delivery_points.yaml from bari_spawn_points.yaml"""

import yaml
from pathlib import Path

# Load spawn points
spawn_file = Path('/home/studente/ackermann_sim/src/osm_city_pipeline/maps/bari_spawn_points.yaml')
with open(spawn_file, 'r') as f:
    spawn_data = yaml.safe_load(f)

# Convert to delivery points
delivery_points = []
for sp in spawn_data['spawn_points'][:20]:  # Use first 20
    delivery_points.append({
        'id': sp['id'],
        'name': f"delivery_{sp['name']}",
        'position': {
            'east': sp['position']['east'],
            'north': sp['position']['north'],
            'up': sp['position']['up']
        },
        'description': f"On {sp.get('road_name', 'unknown')} ({sp.get('highway_type', 'unknown')})"
    })

# Save to delivery_points.yaml
output = {
    'delivery_points': delivery_points
}

output_file = Path('ackermann_drl/config/delivery_points.yaml')
with open(output_file, 'w') as f:
    yaml.dump(output, f, default_flow_style=False, sort_keys=False)

print(f"Generated {len(delivery_points)} delivery points")
```

## Verification

Check delivery points are loaded correctly:

```python
from ackermann_drl.utils.delivery_points import DeliveryPoints

dp = DeliveryPoints()
print(f"Loaded {dp.get_point_count()} delivery points")

for point in dp.get_all_points():
    pos = dp.get_point_position(point)
    print(f"{point['name']}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
```

