"""Road geometry utilities for navigation and path planning."""

import json
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import numpy as np


class RoadsGeometry:
    """Utilities for working with OSM road geometry data."""
    
    def __init__(self, roads_file: Optional[str] = None):
        """Initialize roads geometry from JSON file.
        
        Args:
            roads_file: Path to bari_roads.json file. If None, uses default.
        """
        if roads_file is None:
            # Default path relative to package
            package_path = Path(__file__).parent.parent.parent
            roads_file = package_path / 'osm_city_pipeline' / 'maps' / 'bari_roads.json'
        
        self.roads_file = Path(roads_file)
        self.roads_data: Dict = {}
        self._load_roads()
    
    def _load_roads(self):
        """Load road data from JSON file."""
        if not self.roads_file.exists():
            raise FileNotFoundError(f"Roads file not found: {self.roads_file}")
        
        with open(self.roads_file, 'r') as f:
            self.roads_data = json.load(f)
    
    def get_road_by_name(self, name: str) -> Optional[Dict]:
        """Get road data by street name.
        
        Args:
            name: Street name (e.g., "Via Dante")
            
        Returns:
            Road dictionary or None if not found
        """
        for road in self.roads_data.get('roads', []):
            if road.get('name') == name:
                return road
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
        for point in road.get('centerline_enu', []):
            centerline.append((
                point['east'],
                point['north'],
                point['up']
            ))
        return centerline
    
    def get_projection_center(self) -> Tuple[float, float, float]:
        """Get the ENU projection center.
        
        Returns:
            Tuple of (latitude, longitude, height)
        """
        center = self.roads_data.get('projection_center', {})
        return (
            center.get('latitude', 0.0),
            center.get('longitude', 0.0),
            center.get('height', 0.0)
        )

