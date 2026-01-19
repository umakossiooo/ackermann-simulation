"""Road geometry utilities for navigation and path planning.

Loads road centerlines from map JSON files and provides distance calculations.
"""

import json
import math
import os
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from shapely.geometry import Point, LineString, Polygon


class RoadsGeometry:
    """Loads road centerlines from map JSON files and computes distances to roads."""
    
    def __init__(self, edges_file: Optional[str] = None, map_file: Optional[str] = None):
        """Initialize from map JSON files. Auto-finds files if paths not provided."""
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
        self.road_polygons: List[Polygon] = []
        self._load_roads()
        self._build_polylines()
        self._load_road_polygons()
    
    def _find_map2gazebo_file(self, filename: str) -> str:
        """Find map file by trying multiple candidate paths."""
        env_var = f'MAP2GAZEBO_{filename.upper().replace(".", "_")}'
        env_path = os.getenv(env_var)
        if env_path and Path(env_path).exists():
            return env_path
        
        package_path = Path(__file__).parent.parent.parent
        candidate_paths = [
            package_path.parent / 'maps' / filename,
            Path(f'/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/maps/{filename}'),
            package_path.parent / 'map2gazebo_maps' / filename,
            Path(f'/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/map2gazebo_maps/{filename}'),
            package_path.parent / 'map2gazebo' / 'maps' / filename,
            Path(f'/root/colcon_ws/src/map2gazebo/maps/{filename}'),
            Path(f'/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/maps/{filename}'),
        ]
        
        for candidate in candidate_paths:
            if candidate.exists():
                return str(candidate)
        
        raise FileNotFoundError(
            f"{filename} not found. Tried paths:\n" +
            "\n".join(f"  - {p}" for p in candidate_paths) +
            f"\n\nEnsure map2gazebo is mounted or set {env_var} environment variable."
        )

    def _load_road_polygons(self):
        """Load road polygons if available (for curb/sidewalk detection)."""
        try:
            polygons_file = self._find_map2gazebo_file('road_polygons_merged.json')
        except FileNotFoundError:
            return
        try:
            with open(polygons_file, 'r') as f:
                data = json.load(f)
            for entry in data.values():
                merged_polys = entry.get('merged_polygons', [])
                for coords in merged_polys:
                    if len(coords) < 3:
                        continue
                    try:
                        poly = Polygon(coords)
                        if not poly.is_empty and poly.is_valid:
                            self.road_polygons.append(poly)
                    except Exception:
                        continue
        except Exception:
            self.road_polygons = []
    
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
                continue
            
            points = []
            for node_id in centerline_nodes:
                node_id_str = str(node_id)
                if node_id_str in self.nodes_data:
                    coords = self.nodes_data[node_id_str]
                    if len(coords) >= 2:
                        points.append((coords[0], coords[1]))
            
            if len(points) < 2:
                continue
            
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
            except Exception:
                continue
    
    def get_road_by_name(self, name: str) -> Optional[Dict]:
        """Get road data by street name."""
        for metadata in self.roads_metadata:
            if metadata.get('name') == name:
                way_id = metadata.get('way_id')
                if way_id and way_id in self.edges_data:
                    return self.edges_data[way_id]
        return None
    
    def get_road_centerline(self, road_name: str) -> List[Tuple[float, float, float]]:
        """Get centerline coordinates for a road as list of (east, north, up) tuples."""
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
                    up = coords[2] if len(coords) >= 3 else 0.0
                    centerline.append((coords[0], coords[1], up))
        return centerline
    
    def distance_to_nearest_road(self, x: float, y: float) -> Tuple[float, Optional[Dict]]:
        """Calculate distance from point (x, y) to nearest road centerline.
        
        Returns (distance in meters, road metadata). Distance is 0.0 if on road.
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

    def signed_distance_to_road(self, x: float, y: float) -> Optional[float]:
        """Signed distance to nearest road polygon boundary.
        
        Returns negative distance when inside a road polygon, positive when outside.
        Returns None if no road polygons are available.
        """
        if not self.road_polygons:
            return None
        point = Point(x, y)
        inside_dist = []
        outside_dist = []
        
        for poly in self.road_polygons:
            if poly.contains(point) or poly.touches(point):
                inside_dist.append(poly.boundary.distance(point))
            else:
                outside_dist.append(poly.distance(point))
        
        if inside_dist:
            return -min(inside_dist)
        if outside_dist:
            return min(outside_dist)
        return None
    
    def get_road_heading_at_point(self, x: float, y: float) -> Tuple[float, Optional[LineString]]:
        """Get road heading at nearest point. Returns (heading in radians, polyline).
        
        Heading: 0 = east, pi/2 = north, range [-pi, pi].
        """
        if not self.roads_polylines:
            return 0.0, None
        
        point = Point(x, y)
        min_distance = float('inf')
        nearest_polyline = None
        
        for polyline in self.roads_polylines:
            distance = point.distance(polyline)
            if distance < min_distance:
                min_distance = distance
                nearest_polyline = polyline
        
        if nearest_polyline is None:
            return 0.0, None
        
        try:
            projected_point = nearest_polyline.interpolate(nearest_polyline.project(point))
            coords = list(nearest_polyline.coords)
            
            for i in range(len(coords) - 1):
                p1 = Point(coords[i])
                p2 = Point(coords[i + 1])
                segment = LineString([p1, p2])
                
                if segment.distance(projected_point) < 0.01:
                    dx = coords[i + 1][0] - coords[i][0]
                    dy = coords[i + 1][1] - coords[i][1]
                    heading = math.atan2(dy, dx)
                    return heading, nearest_polyline
            
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
