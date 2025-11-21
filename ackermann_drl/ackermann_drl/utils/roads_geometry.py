"""Road geometry utilities for navigation and path planning.

MUST RUN INSIDE DOCKER CONTAINER.
Loads real street coordinates from osm_city_pipeline (mounted volume).
"""

import json
import os
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import numpy as np
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points


class RoadsGeometry:
    """Utilities for working with OSM road geometry data.
    
    Builds polylines from road centerlines and provides distance calculations.
    Designed to work inside Docker container with mounted volumes.
    """
    
    def __init__(self, roads_file: Optional[str] = None):
        """Initialize roads geometry from JSON file.
        
        Args:
            roads_file: Path to bari_roads.json file. If None, uses default.
                       Tries multiple paths to find the file inside Docker container.
        """
        if roads_file is None:
            roads_file = self._find_roads_file()
        
        self.roads_file = Path(roads_file)
        self.roads_data: Dict = {}
        self.roads_polylines: List[LineString] = []
        self.roads_metadata: List[Dict] = []
        self._load_roads()
        self._build_polylines()
    
    def _find_roads_file(self) -> str:
        """Find bari_roads.json file in Docker container.
        
        Tries multiple paths:
        1. Relative to package (if osm_city_pipeline is mounted)
        2. Absolute path from workspace root
        3. Environment variable override
        
        Returns:
            Path to roads file
            
        Raises:
            FileNotFoundError: If file cannot be found
        """
        # Try environment variable first
        env_path = os.getenv('BARI_ROADS_JSON')
        if env_path and Path(env_path).exists():
            return env_path
        
        # Try relative to package (workspace structure)
        package_path = Path(__file__).parent.parent.parent
        candidate_paths = [
            # Path 1: osm_city_pipeline sibling to ackermann-vehicle-gzsim-ros2
            package_path.parent / 'osm_city_pipeline' / 'maps' / 'bari_roads.json',
            # Path 2: Direct from workspace root
            Path('/root/colcon_ws/src/osm_city_pipeline/maps/bari_roads.json'),
            # Path 3: From workspace root relative
            package_path.parent.parent / 'osm_city_pipeline' / 'maps' / 'bari_roads.json',
            # Path 4: Absolute host path (if mounted)
            Path('/home/studente/ackermann_sim/src/osm_city_pipeline/maps/bari_roads.json'),
        ]
        
        for candidate in candidate_paths:
            if candidate.exists():
                return str(candidate)
        
        # If not found, raise error with helpful message
        raise FileNotFoundError(
            f"bari_roads.json not found. Tried paths:\n" +
            "\n".join(f"  - {p}" for p in candidate_paths) +
            "\n\nEnsure osm_city_pipeline is mounted or set BARI_ROADS_JSON environment variable."
        )
    
    def _load_roads(self):
        """Load road data from JSON file."""
        if not self.roads_file.exists():
            raise FileNotFoundError(f"Roads file not found: {self.roads_file}")
        
        with open(self.roads_file, 'r') as f:
            self.roads_data = json.load(f)
    
    def _build_polylines(self):
        """Build Shapely LineString polylines from road centerlines.
        
        Creates a list of LineString objects for efficient distance calculations.
        """
        self.roads_polylines = []
        self.roads_metadata = []
        
        for road in self.roads_data.get('roads', []):
            centerline_enu = road.get('centerline_enu', [])
            if len(centerline_enu) < 2:
                continue  # Skip roads with insufficient points
            
            # Extract (east, north) coordinates (ignore up/z for 2D distance)
            points = [(pt['east'], pt['north']) for pt in centerline_enu]
            
            # Create LineString polyline
            try:
                polyline = LineString(points)
                self.roads_polylines.append(polyline)
                self.roads_metadata.append({
                    'way_id': road.get('way_id'),
                    'name': road.get('name', ''),
                    'highway_type': road.get('highway_type', ''),
                    'lanes': road.get('lanes', 1),
                    'polyline': polyline
                })
            except Exception as e:
                # Skip invalid geometries
                continue
    
    def get_road_by_name(self, name: str) -> Optional[Dict]:
        """Get road data by street name.
        
        Args:
            name: Street name (e.g., "Via Dante")
            
        Returns:
            Road dictionary or None if not found
        """
        for road in self.roads_data.get('roads', []):
            if road.get('name') == name:
                return road
        return None
    
    def get_road_centerline(self, road_name: str) -> List[Tuple[float, float, float]]:
        """Get centerline coordinates for a road.
        
        Args:
            road_name: Street name
            
        Returns:
            List of (east, north, up) tuples
        """
        road = self.get_road_by_name(road_name)
        if road is None:
            return []
        
        centerline = []
        for point in road.get('centerline_enu', []):
            centerline.append((
                point['east'],
                point['north'],
                point['up']
            ))
        return centerline
    
    def distance_to_nearest_road(self, x: float, y: float) -> Tuple[float, Optional[Dict]]:
        """Calculate distance from point (x, y) to nearest road.
        
        Args:
            x: East coordinate (meters)
            y: North coordinate (meters)
            
        Returns:
            Tuple of (distance in meters, road metadata dict or None)
            Distance is 0.0 if point is on a road, positive if off-road.
        """
        if not self.roads_polylines:
            return float('inf'), None
        
        point = Point(x, y)
        min_distance = float('inf')
        nearest_road = None
        
        for polyline, metadata in zip(self.roads_polylines, self.roads_metadata):
            distance = point.distance(polyline)
            if distance < min_distance:
                min_distance = distance
                nearest_road = metadata.copy()
        
        return min_distance, nearest_road
    
    def get_projection_center(self) -> Tuple[float, float, float]:
        """Get the ENU projection center.
        
        Returns:
            Tuple of (latitude, longitude, height)
        """
        center = self.roads_data.get('projection_center', {})
        return (
            center.get('latitude', 0.0),
            center.get('longitude', 0.0),
            center.get('height', 0.0)
        )
    
    def get_all_roads_count(self) -> int:
        """Get total number of roads loaded.
        
        Returns:
            Number of roads
        """
        return len(self.roads_polylines)
    
    def get_road_polyline(self, road_name: str) -> Optional[LineString]:
        """Get Shapely LineString polyline for a road by name.
        
        Args:
            road_name: Street name
            
        Returns:
            LineString polyline or None if not found
        """
        for metadata in self.roads_metadata:
            if metadata['name'] == road_name:
                return metadata['polyline']
        return None

