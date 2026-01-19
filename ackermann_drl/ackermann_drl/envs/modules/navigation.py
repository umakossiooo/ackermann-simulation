import numpy as np
from typing import Tuple, Optional
from ackermann_drl.utils.roads_geometry import RoadsGeometry


class NavigationSystem:
    """Goal-directed navigation helpers.

    Computes a lookahead target toward the goal and road-centered metrics
    for reward shaping and safety checks.
    """

    def __init__(self, lookahead_distance: float = 9.0):
        self.lookahead_distance = max(0.0, float(lookahead_distance))
        self.roads_geometry = RoadsGeometry()  # For computing distance to roads

    def get_local_target(
        self,
        robot_pos: Tuple[float, float],
        goal_pos: Optional[Tuple[float, float]],
    ):
        """Return a local target toward the goal and a cross-track error.

        The target is placed along the straight line to the goal at
        lookahead_distance meters (or the goal if it is closer).
        Cross-track error is the distance to the nearest road centerline.
        """
        if goal_pos is None or robot_pos is None:
            return None, 0.0

        robot_xy = np.asarray(robot_pos, dtype=np.float32).reshape(-1)
        goal_xy = np.asarray(goal_pos, dtype=np.float32).reshape(-1)
        if robot_xy.size < 2 or goal_xy.size < 2:
            return None, 0.0

        robot_xy = robot_xy[:2]
        goal_xy = goal_xy[:2]

        direction = goal_xy - robot_xy
        distance = float(np.linalg.norm(direction))
        if not np.isfinite(distance) or distance <= 1e-6:
            target = goal_xy
        else:
            step = min(self.lookahead_distance, distance)
            target = robot_xy + (direction / distance) * step

        cte = self.get_cross_track_error(float(robot_xy[0]), float(robot_xy[1]))
        return (float(target[0]), float(target[1])), float(cte)

    def get_cross_track_error(self, x: float, y: float) -> float:
        """Distance from (x, y) to the nearest road centerline."""
        dist, _ = self.roads_geometry.distance_to_nearest_road(x, y)
        if not np.isfinite(dist):
            return 0.0
        return float(dist)

    def get_road_distance(self, x, y):
        """Distance from (x, y) to the nearest road centerline."""
        dist, _ = self.roads_geometry.distance_to_nearest_road(x, y)
        return dist

    def get_road_info(self, x, y):
        """Get detailed road information for point (x, y).

        Returns:
            dist_center: Distance to road centerline
            width: Width of the nearest road
            dist_edge: Distance to road edge (positive = outside, negative = inside)
            is_offroad: Boolean, True if outside road area
        """
        dist_center, metadata = self.roads_geometry.distance_to_nearest_road(x, y)
        signed_edge = self.roads_geometry.signed_distance_to_road(x, y)

        width = 5.0  # Default width if not found
        if metadata and 'width' in metadata:
            width = float(metadata['width'])
        if not np.isfinite(width) or width <= 0.1:
            width = 5.0

        if signed_edge is not None and np.isfinite(signed_edge):
            dist_edge = signed_edge
        else:
            dist_edge = dist_center - (width / 2.0)

        is_offroad = dist_edge > 0.0

        return dist_center, width, dist_edge, is_offroad
