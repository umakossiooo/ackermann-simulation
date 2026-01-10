"""Delivery points loader for goal-based navigation.

MUST RUN INSIDE DOCKER CONTAINER.
Loads delivery points from YAML configuration file.
"""

import yaml
import os
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import numpy as np
import random
from shapely.geometry import Point


class DeliveryPoints:
    """Utility for loading and managing delivery points (goals).
    
    Delivery points are target locations the robot should navigate to.
    Coordinates are in ENU (East-North-Up) format.
    """
    
    def __init__(self, delivery_points_file: Optional[str] = None):
        """Initialize delivery points from YAML file.
        
        Args:
            delivery_points_file: Path to delivery_points.yaml file. If None, uses default.
        """
        if delivery_points_file is None:
            delivery_points_file = self._find_delivery_points_file()
        
        self.delivery_points_file = Path(delivery_points_file)
        self.delivery_points: List[Dict] = []
        self._load_delivery_points()
    
    def _find_delivery_points_file(self) -> str:
        """Find delivery_points.yaml file in Docker container.
        
        Tries multiple paths:
        1. Relative to package (config/delivery_points.yaml)
        2. Environment variable override
        3. Absolute paths
        
        Returns:
            Path to delivery points file
            
        Raises:
            FileNotFoundError: If file cannot be found
        """
        # Try environment variable first
        env_path = os.getenv('DELIVERY_POINTS_YAML')
        if env_path and Path(env_path).exists():
            return env_path
        
        # Try relative to package
        package_path = Path(__file__).parent.parent
        candidate_paths = [
            # Path 1: Config directory in package
            package_path / 'config' / 'delivery_points.yaml',
            # Path 2: Absolute path in container
            Path('/root/colcon_ws/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/delivery_points.yaml'),
            # Path 3: From workspace root
            Path('/root/colcon_ws/install/ackermann_drl/share/ackermann_drl/config/delivery_points.yaml'),
            # Path 4: Host workspace path (non-container runs)
            Path('/home/studente/ackermann_sim/src/ackermann-vehicle-gzsim-ros2/ackermann_drl/config/delivery_points.yaml'),
        ]
        
        for candidate in candidate_paths:
            if candidate.exists():
                return str(candidate)
        
        # If not found, raise error with helpful message
        raise FileNotFoundError(
            f"delivery_points.yaml not found. Tried paths:\n" +
            "\n".join(f"  - {p}" for p in candidate_paths) +
            "\n\nEnsure delivery_points.yaml exists in ackermann_drl/config/"
        )
    
    def _load_delivery_points(self):
        """Load delivery points from YAML file and ensure they are on roads."""
        if not self.delivery_points_file.exists():
            raise FileNotFoundError(f"Delivery points file not found: {self.delivery_points_file}")
        
        with open(self.delivery_points_file, 'r') as f:
            data = yaml.safe_load(f)
        
        raw_points = data.get('delivery_points', [])
        self.delivery_points = []
        
        # Validate and fix points using RoadsGeometry
        try:
            from ackermann_drl.utils.roads_geometry import RoadsGeometry
            rg = RoadsGeometry()
            
            for point in raw_points:
                pos = point.get('position', {})
                x = pos.get('east', 0.0)
                y = pos.get('north', 0.0)
                
                # Check if point is on road
                dist, metadata = rg.distance_to_nearest_road(x, y)
                width = 5.0
                if metadata and 'width' in metadata:
                    width = float(metadata['width'])
                
                half_width = width / 2.0
                margin = 0.5  # Safety margin to be well inside
                
                # If distance to center is greater than half width (minus margin), it might be offroad
                if dist > (half_width - margin):
                    # Point is offroad or too close to edge. Project it to centerline.
                    if metadata:
                        polyline = metadata['polyline']
                        p = Point(x, y)
                        # Project to centerline
                        projected = polyline.interpolate(polyline.project(p))
                        x_new, y_new = projected.x, projected.y
                        
                        # Update point
                        print(f"[DeliveryPoints] Correcting point {point.get('id')} from ({x:.1f}, {y:.1f}) to ({x_new:.1f}, {y_new:.1f})")
                        if 'position' not in point:
                            point['position'] = {}
                        point['position']['east'] = float(x_new)
                        point['position']['north'] = float(y_new)
                
                self.delivery_points.append(point)
                
        except Exception as e:
            print(f"[DeliveryPoints] Warning: Could not validate points against roads geometry: {e}")
            self.delivery_points = raw_points
    
    def get_all_points(self) -> List[Dict]:
        """Get all delivery points.
        
        Returns:
            List of delivery point dictionaries
        """
        return self.delivery_points.copy()
    
    def get_point_by_id(self, point_id: int) -> Optional[Dict]:
        """Get delivery point by ID.
        
        Args:
            point_id: Delivery point ID
            
        Returns:
            Delivery point dictionary or None if not found
        """
        for point in self.delivery_points:
            if point.get('id') == point_id:
                return point
        return None
    
    def get_random_point(self) -> Dict:
        """Get a random delivery point.
        
        Returns:
            Random delivery point dictionary
            
        Raises:
            ValueError: If no delivery points are available
        """
        if not self.delivery_points:
            raise ValueError("No delivery points available")
        return random.choice(self.delivery_points)
    
    def get_point_count(self) -> int:
        """Get total number of delivery points.
        
        Returns:
            Number of delivery points
        """
        return len(self.delivery_points)
    
    def get_next_point(self, current_point: Optional[Dict] = None) -> Optional[Dict]:
        """Get the next delivery point in sequence.
        
        Args:
            current_point: Current delivery point dictionary. If None, returns first point.
            
        Returns:
            Next delivery point dictionary, or None if no more points
        """
        if not self.delivery_points:
            return None
        
        if current_point is None:
            # Return first point if no current point
            return self.delivery_points[0]
        
        # Find current point index
        current_id = current_point.get('id')
        current_idx = None
        for i, point in enumerate(self.delivery_points):
            if point.get('id') == current_id:
                current_idx = i
                break
        
        if current_idx is None:
            # Current point not found, return first point
            return self.delivery_points[0]
        
        # Get next point (wrap around to first if at end)
        next_idx = (current_idx + 1) % len(self.delivery_points)
        return self.delivery_points[next_idx]
    
    def get_point_position(self, point: Dict) -> Tuple[float, float, float]:
        """Extract position from delivery point.
        
        Args:
            point: Delivery point dictionary
            
        Returns:
            Tuple of (east, north, up) coordinates
        """
        pos = point.get('position', {})
        return (
            pos.get('east', 0.0),
            pos.get('north', 0.0),
            pos.get('up', 0.0)
        )
