#!/usr/bin/env python3
"""
Standalone A* Path Planner for Ackermann Vehicle

Reads map2gazebo files directly - no dependency on ackermann_drl package.
Simple navigation from point A to point B using A* algorithm.
Similar structure to dijkstra_path_planner.py but uses A* for potentially faster path finding.
"""

import rclpy
from rclpy.node import Node
import math
import json
import time
import subprocess
import numpy as np
from pathlib import Path
from collections import defaultdict
import heapq
from typing import List, Dict, Tuple, Optional
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from shapely.geometry import Point, Polygon


# Global variable for current pose (similar to rrt_ob.py)
current_pose = None


class SimpleAStarPlanner:
    """Simple A* path planner that reads map2gazebo files directly.
    
    Ensures all paths stay within road meshes by validating waypoints against road polygons.
    Uses A* algorithm with Euclidean distance heuristic for efficient path finding.
    """
    
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
    
    def astar(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], dict]:
        """Compute shortest path using A* algorithm.
        
        A* uses f(n) = g(n) + h(n) where:
        - g(n) = actual distance from start to node n
        - h(n) = heuristic estimate (Euclidean distance) from node n to goal
        
        All waypoints in the returned path are guaranteed to be on roads.
        
        Args:
            start: Start position (x, y) in ENU coordinates
            goal: Goal position (x, y) in ENU coordinates
            
        Returns:
            Tuple of (path, metrics_dict) where metrics contains:
            - path_cost: Total path length in meters
            - computation_time: Execution time in seconds
            - nodes_expanded: Number of nodes explored
            - memory_usage: Peak memory usage in MB
            - optimal: Whether path is optimal (True if heuristic is admissible)
        """
        import tracemalloc
        
        # Start tracking metrics
        tracemalloc.start()
        start_time = time.perf_counter()
        
        # Find nearest road nodes
        start_node, start_dist = self._find_nearest_node(start[0], start[1])
        goal_node, goal_dist = self._find_nearest_node(goal[0], goal[1])
        
        # If start and goal are the same node, return path with start position
        if start_node == goal_node:
            if start_dist > 0.1:
                path = [start, start_node]
            else:
                path = [start_node]
            
            path_cost = sum(
                self._euclidean_distance(path[i], path[i+1])
                for i in range(len(path) - 1)
            ) if len(path) > 1 else 0.0
            
            metrics = {
                'path_cost': path_cost,
                'computation_time': time.perf_counter() - start_time,
                'nodes_expanded': 0,
                'memory_usage': tracemalloc.get_traced_memory()[1] / 1024 / 1024,
                'optimal': True
            }
            tracemalloc.stop()
            return path, metrics
        
        # A* algorithm
        # Priority queue stores (f_score, g_score, node)
        # f_score = g_score + heuristic
        g_scores = {start_node: 0.0}  # Actual distance from start
        previous = {start_node: None}
        
        # Heuristic function: Euclidean distance to goal
        def heuristic(node: Tuple[float, float]) -> float:
            return self._euclidean_distance(node, goal_node)
        
        # Initialize priority queue with start node
        f_score_start = g_scores[start_node] + heuristic(start_node)
        pq = [(f_score_start, 0.0, start_node)]  # (f_score, g_score, node)
        visited = set()
        nodes_expanded = 0  # Track nodes explored
        
        while pq:
            f_current, g_current, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            
            visited.add(current)
            nodes_expanded += 1  # Count this node as expanded
            
            if current == goal_node:
                # Reconstruct path (all nodes are on road centerlines)
                path = []
                node = goal_node
                while node is not None:
                    path.append(node)
                    node = previous.get(node)
                path.reverse()
                
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
                
                # Calculate path cost
                path_cost = sum(
                    self._euclidean_distance(validated_path[i], validated_path[i+1])
                    for i in range(len(validated_path) - 1)
                )
                
                # Collect metrics
                computation_time = time.perf_counter() - start_time
                memory_usage = tracemalloc.get_traced_memory()[1] / 1024 / 1024
                tracemalloc.stop()
                
                metrics = {
                    'path_cost': path_cost,
                    'computation_time': computation_time,
                    'nodes_expanded': nodes_expanded,
                    'memory_usage': memory_usage,
                    'optimal': True  # A* with Euclidean heuristic is optimal
                }
                
                return validated_path, metrics
            
            # Explore neighbors (all are road centerline points)
            for neighbor, edge_weight in self.graph.get(current, []):
                if neighbor in visited:
                    continue
                
                # Calculate tentative g_score
                tentative_g = g_current + edge_weight
                
                # If we found a better path to neighbor, update it
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    previous[neighbor] = current
                    
                    # Calculate f_score = g_score + heuristic
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(pq, (f_score, tentative_g, neighbor))
        
        # No path found
        tracemalloc.stop()
        return None, {
            'path_cost': float('inf'),
            'computation_time': time.perf_counter() - start_time,
            'nodes_expanded': nodes_expanded,
            'memory_usage': tracemalloc.get_traced_memory()[1] / 1024 / 1024,
            'optimal': False
        }


class AStarPathPlannerNode(Node):
    """ROS 2 node for A* path planning."""
    
    def __init__(self):
        super().__init__('astar_path_planner_node')
        
        # Find map2gazebo files (match docker-compose mount)
        candidate_paths = [
            Path('/root/colcon_ws/src/map2gazebo/maps'),  # Current mount location
            Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/map2gazebo_maps'),  # Alternative
            Path('/home/studente/ackermann_sim/src/map2gazebo/maps'),  # Host path
        ]
        
        maps_dir = None
        for path in candidate_paths:
            if (path / 'edges.json').exists() and (path / 'map.json').exists():
                maps_dir = path
                break
        
        if maps_dir is None:
            self.get_logger().error("Map files not found! Check docker-compose mount.")
            raise FileNotFoundError("Map files not found")
        
        edges_file = maps_dir / 'edges.json'
        map_file = maps_dir / 'map.json'
        polygons_file = maps_dir / 'road_polygons_merged.json'
        
        # Initialize planner (reads files directly)
        self.get_logger().info(f"Loading road data from {maps_dir}...")
        if polygons_file.exists():
            self.get_logger().info(f"Loading road polygons for boundary validation...")
            self.planner = SimpleAStarPlanner(str(edges_file), str(map_file), str(polygons_file))
        else:
            self.get_logger().warn(f"Road polygons file not found, using centerline-only validation")
            self.planner = SimpleAStarPlanner(str(edges_file), str(map_file))
        self.get_logger().info(f"Loaded {len(self.planner.node_coords)} road nodes")
        if self.planner.road_polygons:
            self.get_logger().info(f"Loaded {len(self.planner.road_polygons)} road polygons for boundary validation")
        
        # Path following parameters
        self.lookahead_distance = 2.5  # Reduced for tighter path following
        self.max_velocity = 1.5  # Reduced for better control
        self.min_velocity = 0.5
        self.waypoint_tolerance = 0.8  # Reduced to keep car closer to path
        self.max_off_road_distance = 2.0  # Maximum allowed distance from road (meters)
        self.road_check_interval = 3  # Check road distance every N control cycles (more frequent)
        self.road_check_counter = 0
        
        # Path state
        self.path = None
        self.current_waypoint_idx = 0
        self.goal_reached = False
        self.path_following = False
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("A* Path Planner Node initialized")
    
    def odom_callback(self, msg):
        """Callback to update current position."""
        global current_pose
        if current_pose is None:
            self.get_logger().info("Received first odometry message!")
        current_pose = msg
    
    def get_current_position(self):
        """Get current position from global current_pose."""
        global current_pose
        if current_pose is None:
            return None, None, None
        
        pos = current_pose.pose.pose.position
        x = pos.x  # east
        y = pos.y  # north
        
        q = current_pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        return x, y, yaw
    
    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """Plan path using A*."""
        self.get_logger().info(f"Planning path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
        
        path, metrics = self.planner.astar((start_x, start_y), (goal_x, goal_y))
        
        if path is None:
            self.get_logger().error("No path found!")
            return False
        
        self.path = path
        self.current_waypoint_idx = 0
        self.goal_reached = False
        self.path_following = True
        
        # Log metrics
        self.get_logger().info(f"Path found! Length: {metrics['path_cost']:.2f}m, Waypoints: {len(path)}")
        self.get_logger().info(f"Metrics: Time={metrics['computation_time']*1000:.2f}ms, "
                              f"Nodes={metrics['nodes_expanded']}, Memory={metrics['memory_usage']:.2f}MB")
        return True
    
    def update_waypoint(self, current_x, current_y):
        """Check if waypoint reached - advance to next waypoint if close enough."""
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return
        
        # Check if we've passed the current waypoint or are close to it
        waypoint = self.path[self.current_waypoint_idx]
        distance = SimpleAStarPlanner._euclidean_distance((current_x, current_y), waypoint)
        
        # Also check if we're past the waypoint (projected along path direction)
        if self.current_waypoint_idx < len(self.path) - 1:
            next_wp = self.path[self.current_waypoint_idx + 1]
            # Vector from current waypoint to next
            path_dx = next_wp[0] - waypoint[0]
            path_dy = next_wp[1] - waypoint[1]
            path_len = math.sqrt(path_dx * path_dx + path_dy * path_dy)
            
            if path_len > 0.01:
                # Vector from waypoint to current position
                to_curr_dx = current_x - waypoint[0]
                to_curr_dy = current_y - waypoint[1]
                # Project onto path direction
                proj = (to_curr_dx * path_dx + to_curr_dy * path_dy) / path_len
                # If projection is positive and significant, we've passed the waypoint
                passed_waypoint = proj > 0.5  # 0.5m past waypoint
            else:
                passed_waypoint = False
        else:
            passed_waypoint = False
        
        if distance < self.waypoint_tolerance or passed_waypoint:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.path):
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx-1}/{len(self.path)-1}")
            else:
                self.goal_reached = True
                self.path_following = False
                self.get_logger().info("Goal reached!")
                self.stop_vehicle()
    
    def compute_control(self, current_x, current_y, current_yaw, target_x, target_y):
        """Compute velocity and steering commands."""
        distance = SimpleAStarPlanner._euclidean_distance((current_x, current_y), (target_x, target_y))
        
        desired_yaw = math.atan2(target_y - current_y, target_x - current_x)
        yaw_error = self._normalize_angle(desired_yaw - current_yaw)
        
        # Velocity: reduce as approaching waypoint
        if distance < 2.0:
            velocity = self.min_velocity + (distance / 2.0) * (self.max_velocity - self.min_velocity)
        else:
            velocity = self.max_velocity
        
        # Steering: proportional control
        max_steering = 1.0  # rad/s
        steering_gain = 2.0
        steering = steering_gain * yaw_error
        steering = np.clip(steering, -max_steering, max_steering)
        
        # Reduce velocity for large steering (safety)
        if abs(steering) > 0.5:
            velocity *= 0.7
        
        return velocity, steering
    
    def stop_vehicle(self):
        """Stop the vehicle."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _compute_road_correction(self, current_x: float, current_y: float, current_yaw: float) -> Tuple[float, Tuple[float, float]]:
        """Compute road correction yaw error and road point.
        
        Args:
            current_x: Current east coordinate
            current_y: Current north coordinate
            current_yaw: Current yaw angle
            
        Returns:
            Tuple of (road_yaw_error, road_point) for correction
        """
        road_point = self.planner.project_to_road(current_x, current_y)
        road_dx = road_point[0] - current_x
        road_dy = road_point[1] - current_y
        road_heading = math.atan2(road_dy, road_dx)
        road_yaw_error = self._normalize_angle(road_heading - current_yaw)
        return road_yaw_error, road_point
    
    def _get_lookahead_point(self, current_x: float, current_y: float, lookahead_dist: float) -> Optional[Tuple[float, float]]:
        """Get lookahead point along the path at specified distance ahead from current position.
        
        This ensures the car follows the path smoothly rather than cutting corners.
        """
        if self.path is None or len(self.path) == 0:
            return None
        
        # Find closest point on path to current position
        min_dist_to_path = float('inf')
        closest_segment_idx = 0
        closest_t = 0.0
        
        # Check all path segments
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            
            # Project current position onto this segment
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg
            
            if seg_len_sq < 0.001:  # Very short segment
                # Just use distance to p1
                dx = current_x - p1[0]
                dy = current_y - p1[1]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < min_dist_to_path:
                    min_dist_to_path = dist
                    closest_segment_idx = i
                    closest_t = 0.0
                continue
            
            # Project current point onto segment
            dx_curr = current_x - p1[0]
            dy_curr = current_y - p1[1]
            t = (dx_curr * dx_seg + dy_curr * dy_seg) / seg_len_sq
            t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
            
            # Point on segment
            proj_x = p1[0] + t * dx_seg
            proj_y = p1[1] + t * dy_seg
            
            # Distance to projected point
            dx = current_x - proj_x
            dy = current_y - proj_y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_segment_idx = i
                closest_t = t
        
        # Now look ahead from the closest point on path
        accumulated_dist = 0.0
        start_idx = closest_segment_idx
        
        # Start from the projection point on the closest segment
        if closest_t < 1.0:
            # We're on a segment, start from projected point
            p1 = self.path[start_idx]
            p2 = self.path[start_idx + 1]
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            seg_len = math.sqrt(dx_seg * dx_seg + dy_seg * dy_seg)
            
            # Distance remaining on current segment
            remaining_on_seg = (1.0 - closest_t) * seg_len
            if remaining_on_seg >= lookahead_dist:
                # Lookahead is on current segment
                t = closest_t + (lookahead_dist / seg_len) if seg_len > 0 else 1.0
                lookahead_x = p1[0] + t * dx_seg
                lookahead_y = p1[1] + t * dy_seg
                return (lookahead_x, lookahead_y)
            
            accumulated_dist = remaining_on_seg
            start_idx += 1
        
        # Continue from next segments
        for i in range(start_idx, len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]
            
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            segment_length = math.sqrt(dx * dx + dy * dy)
            
            if accumulated_dist + segment_length >= lookahead_dist:
                # Lookahead point is on this segment
                remaining = lookahead_dist - accumulated_dist
                t = remaining / segment_length if segment_length > 0 else 0.0
                lookahead_x = p1[0] + t * dx
                lookahead_y = p1[1] + t * dy
                return (lookahead_x, lookahead_y)
            
            accumulated_dist += segment_length
        
        # If we've reached the end, return the last waypoint
        return self.path[-1]
    
    def control_loop(self):
        """Main control loop - runs at 10 Hz. Ensures car stays on roads."""
        if not self.path_following or self.goal_reached:
            return
        
        current_x, current_y, current_yaw = self.get_current_position()
        if current_x is None or self.path is None or self.current_waypoint_idx >= len(self.path):
            return
        
        # Check if car is too far from road (every N cycles for performance)
        self.road_check_counter += 1
        if self.road_check_counter >= self.road_check_interval:
            self.road_check_counter = 0
            road_distance = self.planner.distance_to_nearest_road(current_x, current_y)
            
            if road_distance > self.max_off_road_distance:
                # Car is too far from road - project back to road and adjust target
                self.get_logger().warn(
                    f"Car is {road_distance:.2f}m off-road! Projecting back to road..."
                )
                projected_x, projected_y = self.planner.project_to_road(current_x, current_y)
                # Use projected position for control (forces car back to road)
                current_x, current_y = projected_x, projected_y
            elif road_distance > 0.8:  # Lower threshold for earlier correction
                # Car is getting off-road - reduce speed and correct
                self.get_logger().info(
                    f"Car is {road_distance:.2f}m from road - correcting..."
                )
        
        self.update_waypoint(current_x, current_y)
        
        if self.goal_reached:
            return
        
        # Use lookahead point instead of direct waypoint (prevents cutting corners)
        target = self._get_lookahead_point(current_x, current_y, self.lookahead_distance)
        
        if target is None:
            # Fallback to current waypoint
            if self.current_waypoint_idx < len(self.path):
                target = self.path[self.current_waypoint_idx]
            else:
                return
        
        # Check road distance BEFORE computing control
        road_distance = self.planner.distance_to_nearest_road(current_x, current_y)
        
        # If significantly off-road (>1.5m), prioritize getting back to road
        if road_distance > 1.5:
            # Get nearest road point and steer directly toward it
            road_yaw_error, _ = self._compute_road_correction(current_x, current_y, current_yaw)
            
            # Strong correction steering toward road
            correction_gain = 3.0
            steering = correction_gain * road_yaw_error
            steering = np.clip(steering, -1.0, 1.0)
            
            # Moderate speed - don't stop completely
            velocity = self.min_velocity * 1.2  # Slightly faster to get back on road
        elif road_distance > 0.8:
            # Slightly off-road - blend road correction with path following
            road_yaw_error, _ = self._compute_road_correction(current_x, current_y, current_yaw)
            
            # Normal path following
            velocity, path_steering = self.compute_control(current_x, current_y, current_yaw, target[0], target[1])
            
            # Blend: 70% path following, 30% road correction
            correction_weight = 0.3 * (road_distance / 1.5)
            steering = (1.0 - correction_weight) * path_steering + correction_weight * 2.0 * road_yaw_error
            steering = np.clip(steering, -1.0, 1.0)
            
            # Slight speed reduction
            velocity *= (1.0 - 0.15 * (road_distance / 1.5))
        else:
            # On road - normal path following
            # Ensure target is on road (safety check)
            target_road_dist = self.planner.distance_to_nearest_road(target[0], target[1])
            if target_road_dist > 1.0:
                # Target is off-road, project it to nearest road
                target = self.planner.project_to_road(target[0], target[1])
            
            velocity, steering = self.compute_control(current_x, current_y, current_yaw, target[0], target[1])
        
        cmd = Twist()
        cmd.linear.x = float(velocity)
        cmd.angular.z = float(steering)
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress periodically (every 2 seconds at 10 Hz = every 20 iterations)
        if self.current_waypoint_idx % 20 == 0 or len(self.path) < 20:
            self.get_logger().info(
                f"Following path: waypoint {self.current_waypoint_idx}/{len(self.path)-1}, "
                f"vel={velocity:.2f}m/s, steering={math.degrees(steering):.1f}°, "
                f"road_dist={road_distance:.2f}m, target=({target[0]:.2f}, {target[1]:.2f})"
            )


def move_vehicle(path, node):
    """Start following the path (similar to move_drone in rrt_ob.py)."""
    node.get_logger().info("Starting to move along path")
    node.path = path
    node.current_waypoint_idx = 0
    node.goal_reached = False
    node.path_following = True


if __name__ == "__main__":
    rclpy.init()
    
    node = AStarPathPlannerNode()
    
    # Wait for odometry (similar to rrt_ob.py)
    node.get_logger().info("Waiting for odometry...")
    max_wait_time = 10.0  # Maximum wait time in seconds
    start_time = node.get_clock().now()
    
    # Give subscription time to establish connection
    time.sleep(0.5)
    
    while current_pose is None and rclpy.ok():
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > max_wait_time:
            node.get_logger().error("Timeout waiting for odometry! Check if Gazebo is running.")
            # Check if topic exists
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=2)
            if '/odom' in result.stdout:
                node.get_logger().error("Topic /odom exists but no messages received. Check if vehicle is spawned.")
            else:
                node.get_logger().error("Topic /odom does not exist!")
            rclpy.shutdown()
            exit(1)
        
        rclpy.spin_once(node, timeout_sec=0.1)
        if current_pose is None:
            if int(elapsed * 10) % 10 == 0:  # Log every second
                node.get_logger().info(f"Waiting for current position... (elapsed: {elapsed:.1f}s)")
    
    if current_pose is not None:
        node.get_logger().info("Odometry received!")
    
    if not rclpy.ok():
        rclpy.shutdown()
        exit(1)
    
    # Get start position from current pose
    start_x = current_pose.pose.pose.position.x
    start_y = current_pose.pose.pose.position.y
    start = (start_x, start_y)
    
    # Route goals for testing (A->B, B->C, C->D, ...) - 15 chained routes
    # Uncomment one goal at a time to test sequentially
    # Each route's goal becomes the next route's start
    # NOTE: These coordinates are EXTRACTED FROM ACTUAL ROAD NETWORK in map2gazebo/maps/
    # They are guaranteed to be on roads (from centerline_points and nodes_enu)
    # IMPORTANT: Keep this list IDENTICAL in both dijkstra_path_planner.py and astar_path_planner.py
    # for fair comparison!
    # Generated by: map2gazebo/scripts/extract_route_goals.py
    ROUTE_GOALS = [
        # Route 1: Current Position -> A (short route)
        (-0.27, -102.52),
        # Route 2: A -> B (short route)
        # (4.80, -82.97),
        # Route 3: B -> C (short route)
        # (-15.17, -103.69),
        # Route 4: C -> D (medium route)
        # (-21.63, -123.24),
        # Route 5: D -> E (medium route)
        # (-20.17, -143.25),
        # Route 6: E -> F (medium route)
        # (-40.98, -141.76),
        # Route 7: F -> G (medium route)
        # (-59.76, -134.67),
        # Route 8: G -> H (medium route)
        # (-75.78, -122.24),
        # Route 9: H -> I (medium route)
        # (-69.87, -142.90),
        # Route 10: I -> J (medium route)
        # (-69.84, -163.39),
        # Route 11: J -> K (long route)
        # (-85.20, -177.28),
        # Route 12: K -> L (long route)
        # (-70.45, -191.99),
        # Route 13: L -> M (long route)
        # (-51.69, -200.13),
        # Route 14: M -> N (long route)
        # (-50.16, -220.24),
        # Route 15: N -> O (long route)
        # (-53.70, -200.26),
    ]
    
    # Get active goal (first uncommented)
    goal = ROUTE_GOALS[0]
    goal_x, goal_y = goal
    
    node.get_logger().info("=" * 80)
    node.get_logger().info("PATH PLANNING METRICS")
    node.get_logger().info("=" * 80)
    node.get_logger().info(f"Start position: ({start[0]:.2f}, {start[1]:.2f})")
    node.get_logger().info(f"Goal position: ({goal_x:.2f}, {goal_y:.2f})")
    
    # Plan path and get metrics
    path, metrics = node.planner.astar(start, goal)
    
    if path is None:
        node.get_logger().error("No path found!")
        node.get_logger().info("Metrics:")
        node.get_logger().info(f"  Computation time: {metrics['computation_time']*1000:.2f}ms")
        node.get_logger().info(f"  Nodes expanded: {metrics['nodes_expanded']}")
        rclpy.shutdown()
        exit(1)
    else:
        node.get_logger().info("=" * 80)
        node.get_logger().info("PATH PLANNING RESULTS")
        node.get_logger().info("=" * 80)
        node.get_logger().info(f"Path found: {len(path)} waypoints")
        
        # Print all metrics
        node.get_logger().info("")
        node.get_logger().info("1. PATH COST (Total Distance)")
        node.get_logger().info(f"   {metrics['path_cost']:.4f} meters")
        
        node.get_logger().info("")
        node.get_logger().info("2. COMPUTATION TIME")
        node.get_logger().info(f"   {metrics['computation_time']*1000:.4f} milliseconds")
        node.get_logger().info(f"   {metrics['computation_time']:.6f} seconds")
        
        node.get_logger().info("")
        node.get_logger().info("3. NODES EXPANDED")
        node.get_logger().info(f"   {metrics['nodes_expanded']} nodes explored")
        
        node.get_logger().info("")
        node.get_logger().info("4. MEMORY USAGE")
        node.get_logger().info(f"   {metrics['memory_usage']:.4f} MB (peak)")
        
        node.get_logger().info("")
        node.get_logger().info("5. OPTIMALITY")
        node.get_logger().info(f"   {'✓ Optimal (guaranteed shortest path)' if metrics['optimal'] else '✗ Not optimal'}")
        
        node.get_logger().info("")
        node.get_logger().info("=" * 80)
        
        if len(path) <= 10:
            node.get_logger().info(f"Full path: {path}")
        else:
            node.get_logger().info(f"Path preview: {path[:3]}...{path[-3:]}")
    
    # Start following path
    move_vehicle(path, node)
    node.get_logger().info(f"Path following started. Total waypoints: {len(path)}")
    
    # Main loop - follow path (control loop runs via timer)
    try:
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()

