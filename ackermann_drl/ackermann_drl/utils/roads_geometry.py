"""Road geometry utilities for navigation and path planning.

MUST RUN INSIDE DOCKER CONTAINER.
Loads real street coordinates from map2gazebo (mounted volume).
"""

import json
import math
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
    
    def __init__(self, edges_file: Optional[str] = None, map_file: Optional[str] = None):
        """Initialize roads geometry from map2gazebo JSON files.
        
        Args:
            edges_file: Path to edges.json file. If None, uses default.
            map_file: Path to map.json file. If None, uses default.
                     Tries multiple paths to find the files inside Docker container.
        """
        if edges_file is None:
            edges_file = self._find_map2gazebo_file('edges.json')
        if map_file is None:
            map_file = self._find_map2gazebo_file('map.json')
        
        self.edges_file = Path(edges_file)
        self.map_file = Path(map_file)
        self.edges_data: Dict = {}
        self.nodes_data: Dict = {}
        self.roads_polylines: List[LineString] = []
        self.roads_metadata: List[Dict] = []
        self._load_roads()
        self._build_polylines()
    
    def _find_map2gazebo_file(self, filename: str) -> str:
        """Find map2gazebo file (edges.json or map.json) in Docker container.
        
        Tries multiple paths:
        1. Relative to package (if map2gazebo is mounted)
        2. Absolute path from workspace root
        3. Environment variable override
        
        Args:
            filename: Name of the file to find (e.g., 'edges.json', 'map.json')
        
        Returns:
            Path to the file
            
        Raises:
            FileNotFoundError: If file cannot be found
        """
        # Try environment variable first
        env_var = f'MAP2GAZEBO_{filename.upper().replace(".", "_")}'
        env_path = os.getenv(env_var)
        if env_path and Path(env_path).exists():
            return env_path
        
        # Try relative to package (workspace structure)
        package_path = Path(__file__).parent.parent.parent
        candidate_paths = [
            # Path 1: Mounted maps folder inside ackermann-vehicle-gzsim-ros2 (preferred)
            package_path.parent / 'map2gazebo_maps' / filename,
            # Path 2: Direct from workspace root (absolute path)
            Path(f'/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/map2gazebo_maps/{filename}'),
            # Path 3: map2gazebo sibling to ackermann-vehicle-gzsim-ros2 (fallback)
            package_path.parent / 'map2gazebo' / 'maps' / filename,
            # Path 4: Direct from workspace root (old location, fallback)
            Path(f'/root/colcon_ws/src/map2gazebo/maps/{filename}'),
            # Path 5: Absolute host path (if mounted differently)
            Path(f'/home/studente/ackermann_sim/src/map2gazebo/maps/{filename}'),
        ]
        
        for candidate in candidate_paths:
            if candidate.exists():
                return str(candidate)
        
        # If not found, raise error with helpful message
        raise FileNotFoundError(
            f"{filename} not found. Tried paths:\n" +
            "\n".join(f"  - {p}" for p in candidate_paths) +
            f"\n\nEnsure map2gazebo is mounted or set {env_var} environment variable."
        )
    
    def _load_roads(self):
        """Load road data from map2gazebo JSON files (edges.json and map.json)."""
        if not self.edges_file.exists():
            raise FileNotFoundError(f"Edges file not found: {self.edges_file}")
        if not self.map_file.exists():
            raise FileNotFoundError(f"Map file not found: {self.map_file}")
        
        with open(self.edges_file, 'r') as f:
            self.edges_data = json.load(f)
        
        with open(self.map_file, 'r') as f:
            map_data = json.load(f)
            self.nodes_data = map_data.get('nodes_enu', {})
    
    def _build_polylines(self):
        """Build Shapely LineString polylines from road centerlines.
        
        Creates a list of LineString objects for efficient distance calculations.
        Uses map2gazebo format: centerline_nodes (node IDs) looked up in nodes_enu.
        """
        self.roads_polylines = []
        self.roads_metadata = []
        
        for way_id, edge_data in self.edges_data.items():
            centerline_nodes = edge_data.get('centerline_nodes', [])
            if len(centerline_nodes) < 2:
                continue  # Skip edges with insufficient nodes
            
            # Look up coordinates for each node ID
            points = []
            for node_id in centerline_nodes:
                node_id_str = str(node_id)
                if node_id_str in self.nodes_data:
                    coords = self.nodes_data[node_id_str]
                    if len(coords) >= 2:
                        # coords is [east, north] or [east, north, up]
                        points.append((coords[0], coords[1]))
            
            if len(points) < 2:
                continue  # Skip if we couldn't resolve enough points
            
            # Create LineString polyline
            try:
                polyline = LineString(points)
                self.roads_polylines.append(polyline)
                self.roads_metadata.append({
                    'way_id': way_id,
                    'name': edge_data.get('name', ''),
                    'highway_type': edge_data.get('highway_type', ''),
                    'width': edge_data.get('width', 0.0),
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
        for metadata in self.roads_metadata:
            if metadata.get('name') == name:
                way_id = metadata.get('way_id')
                if way_id and way_id in self.edges_data:
                    return self.edges_data[way_id]
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
        centerline_nodes = road.get('centerline_nodes', [])
        for node_id in centerline_nodes:
            node_id_str = str(node_id)
            if node_id_str in self.nodes_data:
                coords = self.nodes_data[node_id_str]
                if len(coords) >= 2:
                    # coords is [east, north] or [east, north, up]
                    up = coords[2] if len(coords) >= 3 else 0.0
                    centerline.append((coords[0], coords[1], up))
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
    
    def get_road_heading_at_point(self, x: float, y: float) -> Tuple[float, Optional[LineString]]:
        """Get desired heading from nearest road at point (x, y).
        
        Args:
            x: East coordinate (meters)
            y: North coordinate (meters)
            
        Returns:
            Tuple of (heading in radians, nearest polyline or None)
            Heading is 0.0 if no road found or polyline is invalid.
            Heading is in range [-π, π], where 0 is east, π/2 is north.
        """
        if not self.roads_polylines:
            return 0.0, None
        
        point = Point(x, y)
        min_distance = float('inf')
        nearest_polyline = None
        
        # Find nearest polyline
        for polyline in self.roads_polylines:
            distance = point.distance(polyline)
            if distance < min_distance:
                min_distance = distance
                nearest_polyline = polyline
        
        if nearest_polyline is None:
            return 0.0, None
        
        # Project point onto polyline
        try:
            # Get the point on the polyline closest to the query point
            projected_point = nearest_polyline.interpolate(nearest_polyline.project(point))
            
            # Get coordinates of projected point
            proj_x = projected_point.x
            proj_y = projected_point.y
            
            # Find the segment containing the projected point
            coords = list(nearest_polyline.coords)
            
            # Find segment that contains the projected point
            for i in range(len(coords) - 1):
                p1 = Point(coords[i])
                p2 = Point(coords[i + 1])
                segment = LineString([p1, p2])
                
                # Check if projected point is on this segment
                if segment.distance(projected_point) < 0.01:  # Tolerance
                    # Calculate heading from segment direction
                    dx = coords[i + 1][0] - coords[i][0]  # east
                    dy = coords[i + 1][1] - coords[i][1]  # north
                    
                    # Heading: atan2(dy, dx) where 0 is east, π/2 is north
                    heading = math.atan2(dy, dx)
                    return heading, nearest_polyline
            
            # Fallback: use direction from first to last point
            if len(coords) >= 2:
                dx = coords[-1][0] - coords[0][0]
                dy = coords[-1][1] - coords[0][1]
                heading = math.atan2(dy, dx)
                return heading, nearest_polyline
            
            return 0.0, nearest_polyline
            
        except Exception:
            return 0.0, nearest_polyline
    
    def get_projection_center(self) -> Tuple[float, float, float]:
        """Get the ENU projection center.
        
        Returns:
            Tuple of (latitude, longitude, height)
        Note: map2gazebo format may not include projection center in the same way.
        Returns default values if not available.
        """
        # map2gazebo format may store projection center differently
        # For now, return defaults. Can be extended if needed.
        return (0.0, 0.0, 0.0)
    
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

