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
        """Load delivery points from YAML file."""
        if not self.delivery_points_file.exists():
            raise FileNotFoundError(f"Delivery points file not found: {self.delivery_points_file}")
        
        with open(self.delivery_points_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.delivery_points = data.get('delivery_points', [])
    
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

