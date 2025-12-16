import numpy as np
import sys
from pathlib import Path
from typing import Tuple, List, Optional
from ackermann_drl.utils.roads_geometry import RoadsGeometry


class NavigationSystem:
    """A* path planning and local target (carrot) selection.
    
    Uses A* to plan global paths, then selects a local target point (carrot)
    a fixed distance ahead along the path. The robot follows this carrot,
    which provides smooth path following behavior.
    """
    
    def __init__(self, lookahead_distance=5.0):
        self.lookahead_distance = lookahead_distance  # Distance ahead to place carrot (meters)
        self.current_path = []  # List of (x, y) waypoints from A* planner
        self.current_path_index = 0  # Current position along path
        self.roads_geometry = RoadsGeometry()  # For computing distance to roads
        self.planner = None  # A* planner instance
        self._init_planner()

    def _init_planner(self):
        """Locate and initialize the external A* planner implementation."""
        current_file = Path(__file__).resolve()
        repo_root = current_file.parent.parent.parent.parent.parent
        planning_dir = repo_root / 'saye_bringup' / 'scripts' / 'path_planning'
        
        if planning_dir.exists() and str(planning_dir) not in sys.path:
            sys.path.append(str(planning_dir))
        
        try:
            from astar_path_planner import AStarPlanner
            from path_planner_utils import find_maps_directory
            
            maps_dir = find_maps_directory()
            edges = maps_dir / 'edges.json'
            map_file = maps_dir / 'map.json'
            polygons = maps_dir / 'road_polygons_merged.json'
            
            if polygons.exists():
                self.planner = AStarPlanner(str(edges), str(map_file), str(polygons))
            else:
                self.planner = AStarPlanner(str(edges), str(map_file))
        except (ImportError, FileNotFoundError, Exception) as e:
            print(f"[NAV] Warning: Could not initialize A* planner: {e}")
            self.planner = None

    def plan_path(self, start, goal):
        """Plan a global path between start and goal (both 2D tuples)."""
        if not self.planner:
            return False
        path, _ = self.planner.search(start, goal)
        if path and len(path) > 0:
            self.current_path = path
            self.current_path_index = 0
            return True
        return False

    def get_local_target(self, robot_pos):
        """Return the carrot (local target) and cross-track error.
        
        The carrot is a point on the path that's lookahead_distance ahead.
        Cross-track error (CTE) measures how far the robot is from the path.
        
        Args:
            robot_pos: (x, y) position of the robot in world coordinates.
        Returns:
            - target point on the path a lookahead distance ahead
            - cross-track error (distance from robot to closest path point)
        """
        if not self.current_path:
            return None, 0.0
        
        robot_xy = np.array(robot_pos)
        closest_idx, cte = self._find_closest(robot_xy)  # Find nearest waypoint
        target_idx = self._find_ahead(closest_idx)  # Move forward by lookahead_distance
        return self.current_path[target_idx], cte

    def _find_closest(self, robot_xy):
        """Find index of the closest waypoint to the robot and its distance.
        
        Searches a window around the last known position for efficiency.
        Returns the closest waypoint index and the distance to it (CTE).
        """
        min_dist = float('inf')
        closest = self.current_path_index
        # Search window: 5 waypoints back, 50 waypoints forward
        start = max(0, self.current_path_index - 5)
        end = min(len(self.current_path), self.current_path_index + 50)
        
        for i in range(start, end):
            dist = np.linalg.norm(np.array(self.current_path[i]) - robot_xy)
            if dist < min_dist:
                min_dist = dist
                closest = i
        
        self.current_path_index = closest
        return closest, min_dist

    def _find_ahead(self, start_idx):
        """Walk along the path from start_idx until lookahead_distance is reached.

        Accumulates distance along path segments until we've traveled
        lookahead_distance meters. Returns the index of the carrot waypoint.
        """
        acc_dist = 0.0
        target = start_idx
        
        for i in range(start_idx, len(self.current_path) - 1):
            # Distance between consecutive waypoints
            acc_dist += np.linalg.norm(np.array(self.current_path[i+1]) - np.array(self.current_path[i]))
            if acc_dist >= self.lookahead_distance:
                target = i + 1
                break
        
        return min(target, len(self.current_path) - 1)

    def get_road_distance(self, x, y):
        """Distance from (x, y) to the nearest road centerline."""
        dist, _ = self.roads_geometry.distance_to_nearest_road(x, y)
        return dist
