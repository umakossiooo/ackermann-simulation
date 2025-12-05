#!/usr/bin/env python3
"""
Dijkstra Path Planner for Ackermann Vehicle

Refactored implementation using base classes.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports when running as script
_SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(_SCRIPT_DIR))

import time
import tracemalloc
import heapq
from typing import List, Tuple, Optional
import rclpy

from base_path_planner import BasePathPlanner
from base_path_planner_node import BasePathPlannerNode, current_pose
from path_planner_utils import (
    find_maps_directory, wait_for_odometry, get_start_position,
    print_metrics, move_vehicle, ROUTE_GOALS
)


class DijkstraPlanner(BasePathPlanner):
    """Dijkstra path planner implementation."""
    
    def search(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], dict]:
        """Compute shortest path using Dijkstra's algorithm.
        
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
            - optimal: Whether path is optimal (always True for Dijkstra)
        """
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
            
            path_cost = self._calculate_path_cost(path)
            
            metrics = {
                'path_cost': path_cost,
                'computation_time': time.perf_counter() - start_time,
                'nodes_expanded': 0,
                'memory_usage': tracemalloc.get_traced_memory()[1] / 1024 / 1024,
                'optimal': True
            }
            tracemalloc.stop()
            return path, metrics
        
        # Dijkstra's algorithm
        distances = {start_node: 0.0}
        previous = {start_node: None}
        pq = [(0.0, start_node)]
        visited = set()
        nodes_expanded = 0  # Track nodes explored
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
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
                
                # Validate and interpolate path
                validated_path = self._validate_and_interpolate_path(path, start, goal, start_dist, goal_dist)
                
                # Calculate path cost
                path_cost = self._calculate_path_cost(validated_path)
                
                # Collect metrics
                computation_time = time.perf_counter() - start_time
                memory_usage = tracemalloc.get_traced_memory()[1] / 1024 / 1024
                tracemalloc.stop()
                
                metrics = {
                    'path_cost': path_cost,
                    'computation_time': computation_time,
                    'nodes_expanded': nodes_expanded,
                    'memory_usage': memory_usage,
                    'optimal': True  # Dijkstra always finds optimal path
                }
                
                return validated_path, metrics
            
            # Explore neighbors (all are road centerline points)
            for neighbor, edge_weight in self.graph.get(current, []):
                if neighbor in visited:
                    continue
                
                new_dist = current_dist + edge_weight
                
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # No path found
        tracemalloc.stop()
        return None, {
            'path_cost': float('inf'),
            'computation_time': time.perf_counter() - start_time,
            'nodes_expanded': nodes_expanded,
            'memory_usage': tracemalloc.get_traced_memory()[1] / 1024 / 1024,
            'optimal': False
        }


class DijkstraPathPlannerNode(BasePathPlannerNode):
    """ROS 2 node for Dijkstra path planning."""
    
    def __init__(self, planner: DijkstraPlanner):
        """Initialize the Dijkstra path planner node.
        
        Args:
            planner: Dijkstra planner instance
        """
        super().__init__('dijkstra_path_planner_node', planner)


if __name__ == "__main__":
    rclpy.init()
    
    # Find maps directory
    try:
        maps_dir = find_maps_directory()
    except FileNotFoundError as e:
        print(f"Error: {e}")
        exit(1)
    
    edges_file = maps_dir / 'edges.json'
    map_file = maps_dir / 'map.json'
    polygons_file = maps_dir / 'road_polygons_merged.json'
    
    # Initialize planner
    if polygons_file.exists():
        planner = DijkstraPlanner(str(edges_file), str(map_file), str(polygons_file))
    else:
        planner = DijkstraPlanner(str(edges_file), str(map_file))
    
    # Initialize node
    node = DijkstraPathPlannerNode(planner)
    
    # Wait for odometry
    wait_for_odometry(node)
    
    if not rclpy.ok():
        rclpy.shutdown()
        exit(1)
    
    # Get start position
    start_x, start_y = get_start_position()
    if start_x is None:
        node.get_logger().error("Could not get start position!")
        rclpy.shutdown()
        exit(1)
    
    start = (start_x, start_y)
    
    # Get active goal (first uncommented)
    goal = ROUTE_GOALS[0]
    goal_x, goal_y = goal
    
    # Plan path and get metrics
    path, metrics = planner.search(start, goal)
    
    # Print metrics
    print_metrics(node, start, goal, path, metrics)
    
    if path is None:
        rclpy.shutdown()
        exit(1)
    
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

