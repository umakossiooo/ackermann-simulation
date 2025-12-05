#!/usr/bin/env python3
"""
Base Path Planner for Ackermann Vehicle

Contains all common functionality shared between Dijkstra and A* planners.
"""

import math
import json
from pathlib import Path
from collections import defaultdict
from typing import List, Dict, Tuple, Optional
from shapely.geometry import Point, Polygon
from abc import ABC, abstractmethod


class BasePathPlanner(ABC):
    """Base class for path planners with common functionality."""
    
    @staticmethod
    def _euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def __init__(self, edges_file: str, map_file: str, polygons_file: Optional[str] = None):
        """Initialize planner by reading map2gazebo files directly.
        
        Args:
            edges_file: Path to edges.json
            map_file: Path to map.json
            polygons_file: Optional path to road_polygons_merged.json for road boundary validation
        """
        # Load files
        with open(edges_file, 'r') as f:
            self.edges_data = json.load(f)
        
        with open(map_file, 'r') as f:
            map_data = json.load(f)
            self.nodes_data = map_data.get('nodes_enu', {})
        
        # Load road polygons for boundary validation
        self.road_polygons: List[Polygon] = []
        if polygons_file and Path(polygons_file).exists():
            self._load_road_polygons(polygons_file)
        else:
            # Try to find polygons file automatically
            polygons_file = self._find_polygons_file(edges_file)
            if polygons_file:
                self._load_road_polygons(polygons_file)
        
        # Build graph directly from edges
        self.graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], float]]] = defaultdict(list)
        self.node_coords: List[Tuple[float, float]] = []
        self._build_graph()
    
    def _find_polygons_file(self, edges_file: str) -> Optional[str]:
        """Try to find road_polygons_merged.json file near edges_file."""
        edges_path = Path(edges_file)
        candidate_paths = [
            edges_path.parent / 'road_polygons_merged.json',
            edges_path.parent.parent / 'map2gazebo' / 'maps' / 'road_polygons_merged.json',
        ]
        for path in candidate_paths:
            if path.exists():
                return str(path)
        return None
    
    def _load_road_polygons(self, polygons_file: str):
        """Load road polygons from merged polygons JSON file."""
        try:
            with open(polygons_file, 'r') as f:
                data = json.load(f)
            
            self.road_polygons = []
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
            
            print(f"[INFO] Loaded {len(self.road_polygons)} road polygons for boundary validation")
        except Exception as e:
            print(f"[WARN] Could not load road polygons: {e}")
            self.road_polygons = []
    
    def is_point_in_road(self, x: float, y: float) -> bool:
        """Check if a point is within any road polygon.
        
        Args:
            x: East coordinate
            y: North coordinate
            
        Returns:
            True if point is within a road polygon, False otherwise
        """
        if not self.road_polygons:
            # If no polygons loaded, fall back to centerline distance check
            return self.distance_to_nearest_road(x, y) < 5.0  # Within 5m of centerline
        
        point = Point(x, y)
        for poly in self.road_polygons:
            if poly.contains(point) or poly.touches(point):
                return True
        return False
    
    def _build_graph(self):
        """Build graph from edges.json - all nodes are validated to be within road meshes."""
        nodes_added = 0
        nodes_filtered = 0
        
        for way_id, edge_data in self.edges_data.items():
            centerline_nodes = edge_data.get('centerline_nodes', [])
            if len(centerline_nodes) < 2:
                continue
            
            # Convert node IDs to coordinates (road centerline points)
            points = []
            for node_id in centerline_nodes:
                node_id_str = str(node_id)
                if node_id_str in self.nodes_data:
                    coords = self.nodes_data[node_id_str]
                    if len(coords) >= 2:
                        point = (coords[0], coords[1])  # (east, north)
                        # Validate point is within road polygon if polygons are loaded
                        if self.road_polygons and not self.is_point_in_road(point[0], point[1]):
                            nodes_filtered += 1
                            continue
                        points.append(point)
            
            if len(points) < 2:
                continue
            
            # Add edges along the road (connecting road centerline points)
            # Also add intermediate nodes for better path resolution
            for i in range(len(points) - 1):
                p1 = points[i]
                p2 = points[i + 1]
                
                # Calculate distance (points already validated when added to list)
                distance = self._euclidean_distance(p1, p2)
                
                # Add intermediate nodes if edge is long (for better path resolution)
                if distance > 5.0:  # If edge is longer than 5m, add intermediate node
                    mid_x = (p1[0] + p2[0]) / 2.0
                    mid_y = (p1[1] + p2[1]) / 2.0
                    mid_point = (mid_x, mid_y)
                    
                    # Validate intermediate point is in road
                    if not self.is_point_in_road(mid_x, mid_y):
                        # Skip intermediate point, just connect directly
                        self.graph[p1].append((p2, distance))
                        self.graph[p2].append((p1, distance))
                        nodes_added += 2
                    else:
                        # Add edges: p1 -> mid -> p2
                        half_dist = distance / 2.0
                        self.graph[p1].append((mid_point, half_dist))
                        self.graph[mid_point].append((p1, half_dist))
                        self.graph[mid_point].append((p2, half_dist))
                        self.graph[p2].append((mid_point, half_dist))
                        nodes_added += 3
                else:
                    # Add bidirectional edge directly
                    self.graph[p1].append((p2, distance))
                    self.graph[p2].append((p1, distance))
                    nodes_added += 2
        
        self.node_coords = list(self.graph.keys())
        if nodes_filtered > 0:
            print(f"[INFO] Filtered {nodes_filtered} nodes outside road boundaries")
        print(f"[INFO] Graph built with {len(self.node_coords)} nodes, {nodes_added} edges")
    
    def _find_nearest_node(self, x: float, y: float) -> Tuple[Tuple[float, float], float]:
        """Find nearest graph node (which is on a road centerline)."""
        if not self.node_coords:
            return ((x, y), 0.0)
        
        min_dist = float('inf')
        nearest = None
        point = (x, y)
        
        for node in self.node_coords:
            dist = self._euclidean_distance(point, node)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return (nearest, min_dist) if nearest else ((x, y), 0.0)
    
    def distance_to_nearest_road(self, x: float, y: float) -> float:
        """Calculate distance from point to nearest road centerline or polygon boundary.
        
        Args:
            x: East coordinate
            y: North coordinate
            
        Returns:
            Distance to nearest road in meters (0.0 if on road, positive if off-road)
        """
        # First check if point is in any road polygon
        if self.road_polygons:
            point = Point(x, y)
            for poly in self.road_polygons:
                if poly.contains(point) or poly.touches(point):
                    # Point is IN the road - return 0 (or very small value)
                    # The distance to boundary is not relevant when inside
                    return 0.0
            
            # Point is NOT in any polygon - calculate distance to nearest polygon boundary
            min_dist = float('inf')
            for poly in self.road_polygons:
                dist = poly.distance(point)  # Distance from point to polygon
                if dist < min_dist:
                    min_dist = dist
            return min_dist
        
        # Fall back to centerline distance (if no polygons loaded)
        _, distance = self._find_nearest_node(x, y)
        return distance
    
    def project_to_road(self, x: float, y: float) -> Tuple[float, float]:
        """Project a point to the nearest road centerline or polygon boundary.
        
        Args:
            x: East coordinate
            y: North coordinate
            
        Returns:
            Projected coordinates (x, y) on nearest road
        """
        # If polygons are loaded, try to project to nearest polygon boundary
        if self.road_polygons:
            point = Point(x, y)
            min_dist = float('inf')
            best_projection = None
            
            for poly in self.road_polygons:
                if poly.contains(point) or poly.touches(point):
                    # Point is already in road, return nearest centerline node
                    nearest_node, _ = self._find_nearest_node(x, y)
                    return nearest_node
                
                # Project to polygon boundary
                boundary = poly.boundary
                projected = boundary.interpolate(boundary.project(point))
                dist = point.distance(projected)
                
                if dist < min_dist:
                    min_dist = dist
                    best_projection = (projected.x, projected.y)
            
            if best_projection:
                # Project to nearest road centerline from boundary point
                nearest_node, _ = self._find_nearest_node(best_projection[0], best_projection[1])
                return nearest_node
        
        # Fall back to centerline projection
        nearest_node, _ = self._find_nearest_node(x, y)
        return nearest_node
    
    def _interpolate_path(self, path: List[Tuple[float, float]], max_segment_length: float = 2.0) -> List[Tuple[float, float]]:
        """Interpolate path to add intermediate waypoints, ensuring car follows road closely.
        
        Args:
            path: List of waypoints (east, north)
            max_segment_length: Maximum distance between consecutive waypoints (meters)
            
        Returns:
            Interpolated path with more waypoints
        """
        if len(path) < 2:
            return path
        
        interpolated = [path[0]]  # Start with first waypoint
        
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            
            # Calculate distance and direction vector
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            distance = math.sqrt(dx * dx + dy * dy)
            
            # If segment is too long, add intermediate points
            if distance > max_segment_length:
                num_segments = int(math.ceil(distance / max_segment_length))
                
                for j in range(1, num_segments + 1):
                    t = j / num_segments
                    interp_x = p1[0] + t * dx
                    interp_y = p1[1] + t * dy
                    interpolated.append((interp_x, interp_y))
            else:
                # Segment is short enough, just add the next waypoint
                interpolated.append(p2)
        
        return interpolated
    
    def _validate_and_interpolate_path(self, path: List[Tuple[float, float]], 
                                       start: Tuple[float, float], goal: Tuple[float, float],
                                       start_dist: float, goal_dist: float) -> List[Tuple[float, float]]:
        """Validate path waypoints and interpolate for smoother following.
        
        Args:
            path: Raw path from search algorithm
            start: Start position
            goal: Goal position
            start_dist: Distance from start to nearest road node
            goal_dist: Distance from goal to nearest road node
            
        Returns:
            Validated and interpolated path
        """
        # Project start/goal to nearest road nodes instead of adding off-road positions
        # This ensures all waypoints are on roads
        if start_dist > 0.1:
            # Only add start if it's very close to road (within 0.5m)
            if start_dist <= 0.5:
                path.insert(0, start)
            # Otherwise, path already starts at nearest road node
        
        if goal_dist > 0.1:
            # Only add goal if it's very close to road (within 0.5m)
            if goal_dist <= 0.5:
                path.append(goal)
            # Otherwise, path already ends at nearest road node
        
        # Interpolate path to add intermediate waypoints (ensures car stays on road)
        # Use smaller max_segment_length for tighter path following
        interpolated_path = self._interpolate_path(path, max_segment_length=1.5)
        
        # Validate all waypoints are within road boundaries
        validated_path = []
        for waypoint in interpolated_path:
            if self.is_point_in_road(waypoint[0], waypoint[1]):
                validated_path.append(waypoint)
            else:
                # Project waypoint to nearest road
                projected = self.project_to_road(waypoint[0], waypoint[1])
                validated_path.append(projected)
        
        return validated_path
    
    def _calculate_path_cost(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path cost (distance)."""
        if len(path) < 2:
            return 0.0
        return sum(
            self._euclidean_distance(path[i], path[i+1])
            for i in range(len(path) - 1)
        )
    
    @abstractmethod
    def search(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], dict]:
        """Abstract method for path search algorithm.
        
        Args:
            start: Start position (x, y) in ENU coordinates
            goal: Goal position (x, y) in ENU coordinates
            
        Returns:
            Tuple of (path, metrics_dict) where metrics contains:
            - path_cost: Total path length in meters
            - computation_time: Execution time in seconds
            - nodes_expanded: Number of nodes explored
            - memory_usage: Peak memory usage in MB
            - optimal: Whether path is optimal
        """
        pass

