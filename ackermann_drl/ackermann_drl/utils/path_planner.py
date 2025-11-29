"""Dijkstra path planner for road network navigation.

MUST RUN INSIDE DOCKER CONTAINER.
Builds a graph from road polylines and computes shortest paths using Dijkstra's algorithm.
"""

import math
from typing import List, Dict, Tuple, Optional
import numpy as np
from shapely.geometry import Point, LineString
from collections import defaultdict
import heapq

from ackermann_drl.utils.roads_geometry import RoadsGeometry


class PathPlanner:
    """Dijkstra path planner for road network navigation.
    
    Builds a graph from road centerlines and computes shortest paths.
    """
    
    def __init__(self, roads_geometry: RoadsGeometry):
        """Initialize path planner with roads geometry.
        
        Args:
            roads_geometry: RoadsGeometry instance with loaded road data
        """
        self.roads_geometry = roads_geometry
        self.graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], float]]] = defaultdict(list)
        self.node_coords: List[Tuple[float, float]] = []
        self._build_graph()
    
    def _build_graph(self):
        """Build graph from road polylines.
        
        Creates nodes at road endpoints and intersections, with edges weighted by distance.
        """
        self.graph.clear()
        self.node_coords = []
        
        # Collect all points from road polylines
        point_to_roads = defaultdict(list)
        
        for polyline, metadata in zip(self.roads_geometry.roads_polylines, 
                                      self.roads_geometry.roads_metadata):
            coords = list(polyline.coords)
            if len(coords) < 2:
                continue
            
            # Add start and end points as nodes
            start = coords[0]
            end = coords[-1]
            point_to_roads[start].append((polyline, metadata))
            point_to_roads[end].append((polyline, metadata))
            
            # Add intermediate points as nodes (for better path resolution)
            # Use every 3rd point to avoid too many nodes
            for i in range(2, len(coords) - 1, 3):
                point_to_roads[coords[i]].append((polyline, metadata))
        
        # Build graph edges
        for polyline, metadata in zip(self.roads_geometry.roads_polylines,
                                     self.roads_geometry.roads_metadata):
            coords = list(polyline.coords)
            if len(coords) < 2:
                continue
            
            # Add edges along the polyline
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i + 1]
                
                # Calculate distance
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                distance = math.sqrt(dx * dx + dy * dy)
                
                # Add bidirectional edge
                self.graph[p1].append((p2, distance))
                self.graph[p2].append((p1, distance))
        
        # Store all node coordinates
        self.node_coords = list(self.graph.keys())
    
    def _find_nearest_node(self, x: float, y: float) -> Tuple[Tuple[float, float], float]:
        """Find nearest graph node to given coordinates.
        
        Args:
            x: East coordinate
            y: North coordinate
            
        Returns:
            Tuple of (nearest_node_coords, distance)
        """
        if not self.node_coords:
            return ((x, y), 0.0)
        
        point = Point(x, y)
        min_distance = float('inf')
        nearest_node = None
        
        for node in self.node_coords:
            node_point = Point(node[0], node[1])
            distance = point.distance(node_point)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        if nearest_node is None:
            return ((x, y), 0.0)
        
        return (nearest_node, min_distance)
    
    def _project_to_road(self, x: float, y: float) -> Tuple[Tuple[float, float], float]:
        """Project point to nearest road and find nearest graph node.
        
        Args:
            x: East coordinate
            y: North coordinate
            
        Returns:
            Tuple of (nearest_node_coords, distance_to_road)
        """
        # First, find nearest road
        distance_to_road, road_metadata = self.roads_geometry.distance_to_nearest_road(x, y)
        
        if road_metadata is None or road_metadata.get('polyline') is None:
            # No road found, use nearest graph node
            return self._find_nearest_node(x, y)
        
        polyline = road_metadata['polyline']
        point = Point(x, y)
        
        # Project point onto polyline
        projected_point = polyline.interpolate(polyline.project(point))
        proj_x = projected_point.x
        proj_y = projected_point.y
        
        # Find nearest graph node to projected point
        return self._find_nearest_node(proj_x, proj_y)
    
    def _validate_point_on_road(self, x: float, y: float, max_distance: float = 0.5) -> bool:
        """Validate that a point is on or very close to a road.
        
        STRICT: Only allows points within 0.5m of roads to ensure road-only navigation.
        
        Args:
            x: East coordinate
            y: North coordinate
            max_distance: Maximum allowed distance from road (meters) - default 0.5m (strict)
            
        Returns:
            True if point is within max_distance of a road
        """
        distance, _ = self.roads_geometry.distance_to_nearest_road(x, y)
        return distance <= max_distance
    
    def _validate_path_on_roads(self, path: List[Tuple[float, float]], max_distance: float = 0.5) -> bool:
        """Validate that all waypoints in a path are on roads.
        
        STRICT: All waypoints must be within 0.5m of roads.
        
        Args:
            path: List of waypoints (east, north)
            max_distance: Maximum allowed distance from road for each waypoint (meters) - default 0.5m (strict)
            
        Returns:
            True if all waypoints are on roads
        """
        if not path:
            return False
        
        for waypoint in path:
            if not self._validate_point_on_road(waypoint[0], waypoint[1], max_distance):
                return False
        return True
    
    def dijkstra(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Compute shortest path using Dijkstra's algorithm.
        
        Ensures all waypoints in the path are on roads (road-only navigation).
        
        Args:
            start: Start coordinates (east, north)
            goal: Goal coordinates (east, north)
            
        Returns:
            List of waypoints (east, north) from start to goal, or None if no path found
            All waypoints are guaranteed to be on roads.
        """
        # VALIDATION: Start and goal must be on or close to roads
        # Automatically project to nearest road point if off-road
        start_dist, _ = self.roads_geometry.distance_to_nearest_road(start[0], start[1])
        goal_dist, _ = self.roads_geometry.distance_to_nearest_road(goal[0], goal[1])
        
        # If start or goal is too far from any road (>10m), reject the path
        # Otherwise, project to nearest road point
        if start_dist > 10.0:  # Start point way too far from roads
            return None
        if goal_dist > 10.0:  # Goal point way too far from roads
            return None
        
        # Project start/goal to nearest road point if they're off-road (>0.5m)
        # This handles spawn points that may be several meters from roads
        if start_dist > 0.5:
            # Project start to nearest road point
            from shapely.geometry import Point
            from shapely.ops import nearest_points
            start_point = Point(start[0], start[1])
            min_dist = float('inf')
            projected_start = start
            for polyline in self.roads_geometry.roads_polylines:
                proj_point, _ = nearest_points(polyline, start_point)
                dist = start_point.distance(proj_point)
                if dist < min_dist:
                    min_dist = dist
                    projected_start = (proj_point.x, proj_point.y)
            start = projected_start
        
        if goal_dist > 0.5:
            # Project goal to nearest road point
            from shapely.geometry import Point
            from shapely.ops import nearest_points
            goal_point = Point(goal[0], goal[1])
            min_dist = float('inf')
            projected_goal = goal
            for polyline in self.roads_geometry.roads_polylines:
                proj_point, _ = nearest_points(polyline, goal_point)
                dist = goal_point.distance(proj_point)
                if dist < min_dist:
                    min_dist = dist
                    projected_goal = (proj_point.x, proj_point.y)
            goal = projected_goal
        
        # Find nearest nodes to start and goal (project to roads)
        start_node, start_dist = self._project_to_road(start[0], start[1])
        goal_node, goal_dist = self._project_to_road(goal[0], goal[1])
        
        # If start and goal are the same node, return direct path (validate it's on road)
        if start_node == goal_node:
            path = [start_node, goal_node]
            if self._validate_path_on_roads(path):
                return path
            return None
        
        # Dijkstra's algorithm
        distances = {start_node: 0.0}
        previous = {}
        pq = [(0.0, start_node)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            # Check if we reached the goal
            if current == goal_node:
                # Reconstruct path
                path = []
                node = goal_node
                while node is not None:
                    path.append(node)
                    node = previous.get(node)
                path.reverse()
                
                # STRICT VALIDATION: All waypoints must be within 0.5m of roads
                if not self._validate_path_on_roads(path, max_distance=0.5):
                    # Path contains off-road waypoints, reject it
                    return None
                
                # Add actual start and goal points (if they're different from nodes)
                if path[0] != start:
                    # Validate start point is on road before adding (strict: 0.5m)
                    if self._validate_point_on_road(start[0], start[1], max_distance=0.5):
                        path.insert(0, start)
                if path[-1] != goal:
                    # Validate goal point is on road before adding (strict: 0.5m)
                    if self._validate_point_on_road(goal[0], goal[1], max_distance=0.5):
                        path.append(goal)
                
                # Final validation of complete path (strict: 0.5m)
                if self._validate_path_on_roads(path, max_distance=0.5):
                    return path
                else:
                    return None
            
            # Explore neighbors
            for neighbor, edge_weight in self.graph.get(current, []):
                if neighbor in visited:
                    continue
                
                # STRICT VALIDATION: Neighbor must be within 0.5m of a road
                if not self._validate_point_on_road(neighbor[0], neighbor[1], max_distance=0.5):
                    continue  # Skip off-road neighbors
                
                new_dist = current_dist + edge_weight
                
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # No path found - log warning if start/goal are valid
        start_dist, _ = self.roads_geometry.distance_to_nearest_road(start[0], start[1])
        goal_dist, _ = self.roads_geometry.distance_to_nearest_road(goal[0], goal[1])
        if start_dist <= 2.0 and goal_dist <= 2.0:
            # Start and goal are on roads but no path found - this shouldn't happen often
            # Could be due to disconnected road network or strict validation
            pass  # Path not found, return None
        return None
    
    def get_path(self, start_x: float, start_y: float, goal_x: float, goal_y: float) -> Optional[List[Tuple[float, float]]]:
        """Get path from start to goal coordinates.
        
        Args:
            start_x: Start east coordinate
            start_y: Start north coordinate
            goal_x: Goal east coordinate
            goal_y: Goal north coordinate
            
        Returns:
            List of waypoints [(east, north), ...] or None if no path found
        """
        return self.dijkstra((start_x, start_y), (goal_x, goal_y))
    
    def get_next_waypoint(self, current_x: float, current_y: float, 
                          path: List[Tuple[float, float]], 
                          lookahead_distance: float = 5.0) -> Optional[Tuple[float, float]]:
        """Get next waypoint along path based on current position.
        
        Args:
            current_x: Current east coordinate
            current_y: Current north coordinate
            path: List of waypoints from path planner
            lookahead_distance: Distance ahead to look for next waypoint (meters)
            
        Returns:
            Next waypoint (east, north) or None if path is empty
        """
        if not path or len(path) < 2:
            return None
        
        current_point = Point(current_x, current_y)
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(path):
            waypoint_point = Point(waypoint[0], waypoint[1])
            dist = current_point.distance(waypoint_point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead along path
        for i in range(closest_idx, len(path)):
            waypoint = path[i]
            waypoint_point = Point(waypoint[0], waypoint[1])
            dist = current_point.distance(waypoint_point)
            
            if dist >= lookahead_distance:
                return waypoint
        
        # If we're past all waypoints, return the last one
        return path[-1]
    
    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path length.
        
        Args:
            path: List of waypoints
            
        Returns:
            Total path length in meters
        """
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_length += math.sqrt(dx * dx + dy * dy)
        
        return total_length

