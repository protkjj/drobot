"""
Frontier detection and clustering.

Identifies boundaries between known and unknown space in the occupancy grid.
"""
from typing import List, Optional, Tuple
import numpy as np
from scipy import ndimage

from drobot_simulation.config import Config
from drobot_simulation.types import FrontierCluster, MapInfo
from drobot_simulation import utils


class FrontierDetector:
    """Detects and clusters frontier points in occupancy grids."""

    def __init__(
        self,
        safety_margin: int = Config.SAFETY_MARGIN,
        min_frontier_size: int = Config.MIN_FRONTIER_SIZE
    ):
        """
        Initialize frontier detector.

        Args:
            safety_margin: Grid cells to keep away from obstacles
            min_frontier_size: Minimum cluster size to consider
        """
        self.safety_margin = safety_margin
        self.min_frontier_size = min_frontier_size

    def find_frontiers(self, map_info: MapInfo) -> np.ndarray:
        """
        Find frontier points in the occupancy grid.

        Frontiers are free cells adjacent to unknown cells, away from obstacles.

        Args:
            map_info: Map data and metadata

        Returns:
            Array of (y, x) frontier point coordinates
        """
        if not map_info.is_valid():
            return np.array([])

        map_data = map_info.data

        # Identify cell types
        free = (map_data == 0)
        unknown = (map_data == -1)
        occupied = (map_data == 100)

        # Frontiers: free cells adjacent to unknown cells
        unknown_dilated = ndimage.binary_dilation(unknown, iterations=1)
        frontier_mask = free & unknown_dilated

        # Remove frontiers near obstacles
        obstacle_nearby = ndimage.binary_dilation(occupied, iterations=self.safety_margin)
        frontier_mask = frontier_mask & ~obstacle_nearby

        # Return as array of points
        frontier_points = np.argwhere(frontier_mask)
        return frontier_points

    def cluster_frontiers(
        self,
        frontier_points: np.ndarray,
        map_shape: Tuple[int, int]
    ) -> List[FrontierCluster]:
        """
        Group frontier points into connected clusters.

        Args:
            frontier_points: Array of (y, x) frontier coordinates
            map_shape: (height, width) of the map

        Returns:
            List of FrontierCluster objects
        """
        if len(frontier_points) == 0:
            return []

        # Create mask from points
        frontier_mask = np.zeros(map_shape, dtype=bool)
        for y, x in frontier_points:
            frontier_mask[y, x] = True

        # Label connected components
        labeled, num_features = ndimage.label(frontier_mask)

        # Extract clusters
        clusters = []
        for i in range(1, num_features + 1):
            points = np.argwhere(labeled == i)
            if len(points) >= self.min_frontier_size:
                # Calculate center (mean of points)
                cy, cx = points.mean(axis=0).astype(int)
                clusters.append(FrontierCluster(
                    center_grid=(cx, cy),  # Note: (x, y) order
                    size=len(points),
                    points=points
                ))

        return clusters

    def find_nearest_valid_point(
        self,
        gx: int,
        gy: int,
        map_info: MapInfo,
        failed_goals: set,
        max_search: int = 20
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        Find the nearest valid (navigable) grid point.

        Args:
            gx, gy: Starting grid coordinates
            map_info: Map data and metadata
            failed_goals: Set of previously failed goal regions
            max_search: Maximum search radius

        Returns:
            Tuple of (grid_x, grid_y) or (None, None) if not found
        """
        for r in range(max_search):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self._is_goal_valid(nx, ny, map_info, failed_goals):
                            return nx, ny
        return None, None

    def _is_goal_valid(
        self,
        gx: int,
        gy: int,
        map_info: MapInfo,
        failed_goals: set
    ) -> bool:
        """
        Check if a grid position is valid for navigation.

        Args:
            gx, gy: Grid coordinates
            map_info: Map data and metadata
            failed_goals: Set of previously failed goal regions

        Returns:
            True if position is valid
        """
        if not map_info.is_valid():
            return False

        h, w = map_info.height, map_info.width
        if not utils.is_in_bounds(gx, gy, w, h):
            return False

        # Must be free space
        if map_info.data[gy, gx] != 0:
            return False

        # Check against failed goals
        if (gx // Config.FAILED_GOAL_GRID_DIVISOR,
            gy // Config.FAILED_GOAL_GRID_DIVISOR) in failed_goals:
            return False

        # Check safety margin for obstacles
        margin = self.safety_margin
        y_min, y_max = max(0, gy - margin), min(h, gy + margin + 1)
        x_min, x_max = max(0, gx - margin), min(w, gx + margin + 1)
        region = map_info.data[y_min:y_max, x_min:x_max]
        if np.any(region == 100):
            return False

        return True
