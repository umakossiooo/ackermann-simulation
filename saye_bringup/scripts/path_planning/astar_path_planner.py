#!/usr/bin/env python3
"""
A* Path Planner for Ackermann Vehicle

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


class AStarPlanner(BasePathPlanner):
    """A* path planner implementation."""
    
    def search(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], dict]:
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


class AStarPathPlannerNode(BasePathPlannerNode):
    """ROS 2 node for A* path planning."""
    
    def __init__(self, planner: AStarPlanner):
        """Initialize the A* path planner node.
        
        Args:
            planner: A* planner instance
        """
        super().__init__('astar_path_planner_node', planner)


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
        planner = AStarPlanner(str(edges_file), str(map_file), str(polygons_file))
    else:
        planner = AStarPlanner(str(edges_file), str(map_file))
    
    # Initialize node
    node = AStarPathPlannerNode(planner)
    
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

