"""
Frontier scoring and selection.

Evaluates frontiers based on information gain, openness, distance, and size.
"""
from typing import Optional, Tuple, Set
import math
import numpy as np

from drobot_simulation.config import Config
from drobot_simulation.types import Frontier, FrontierCluster, MapInfo, RobotState
from drobot_simulation import utils


class FrontierScorer:
    """Scores and ranks frontier candidates."""

    def __init__(self, info_gain_radius: int = Config.INFO_GAIN_RADIUS):
        """
        Initialize frontier scorer.

        Args:
            info_gain_radius: Radius for information gain calculation (grid cells)
        """
        self.info_gain_radius = info_gain_radius

    def calculate_information_gain(self, gx: int, gy: int, map_data: np.ndarray) -> float:
        """
        Calculate potential information gain at a position.

        Higher values indicate more unknown area visible from this position.

        Args:
            gx, gy: Grid coordinates
            map_data: Occupancy grid data

        Returns:
            Weighted information gain score
        """
        if map_data is None:
            return 0.0

        h, w = map_data.shape
        radius = self.info_gain_radius

        # Extract region around point
        y_min = max(0, gy - radius)
        y_max = min(h, gy + radius + 1)
        x_min = max(0, gx - radius)
        x_max = min(w, gx + radius + 1)

        region = map_data[y_min:y_max, x_min:x_max]

        # Calculate distance weights (closer cells worth more)
        cy, cx = gy - y_min, gx - x_min
        y_coords, x_coords = np.ogrid[0:region.shape[0], 0:region.shape[1]]
        distances = np.sqrt((y_coords - cy) ** 2 + (x_coords - cx) ** 2)
        distances = np.maximum(distances, 1)  # Avoid division by zero

        # Weighted sum of unknown cells
        unknown_mask = (region == -1)
        weighted_gain = np.sum(unknown_mask / distances)

        return weighted_gain

    def calculate_openness(self, gx: int, gy: int, map_data: np.ndarray) -> float:
        """
        Calculate openness score at a position.

        Higher values indicate more open space (fewer obstacles nearby).

        Args:
            gx, gy: Grid coordinates
            map_data: Occupancy grid data

        Returns:
            Openness score between 0.3 and 1.0
        """
        if map_data is None:
            return 0.5

        h, w = map_data.shape
        radius = Config.OPENNESS_RADIUS

        y_min = max(0, gy - radius)
        y_max = min(h, gy + radius + 1)
        x_min = max(0, gx - radius)
        x_max = min(w, gx + radius + 1)

        region = map_data[y_min:y_max, x_min:x_max]
        total_cells = region.size

        if total_cells == 0:
            return 0.5

        # Count cell types
        free_cells = np.sum(region == 0)
        obstacle_cells = np.sum(region == 100)
        unknown_cells = np.sum(region == -1)

        # Openness: free + weighted unknown, minus obstacle penalty
        openness = (free_cells + unknown_cells * 0.7) / total_cells
        obstacle_penalty = (obstacle_cells / total_cells) * 1.0

        # Clamp to minimum score
        score = max(Config.OPENNESS_MIN_SCORE, min(1.0, openness - obstacle_penalty))
        return score

    def is_near_wall(
        self,
        gx: int,
        gy: int,
        map_data: np.ndarray,
        threshold: int = Config.WALL_CHECK_THRESHOLD
    ) -> bool:
        """
        Check if position is near a wall/obstacle.

        Args:
            gx, gy: Grid coordinates
            map_data: Occupancy grid data
            threshold: Search radius for wall detection

        Returns:
            True if obstacles exceed threshold in nearby area
        """
        if map_data is None:
            return False

        h, w = map_data.shape

        y_min = max(0, gy - threshold)
        y_max = min(h, gy + threshold + 1)
        x_min = max(0, gx - threshold)
        x_max = min(w, gx + threshold + 1)

        region = map_data[y_min:y_max, x_min:x_max]
        obstacle_count = np.sum(region == 100)

        # Wall if >30% of area is obstacles
        return obstacle_count > (threshold * threshold * 0.3)

    def score_frontier(
        self,
        cluster: FrontierCluster,
        map_info: MapInfo,
        robot: RobotState,
        valid_grid_pos: Tuple[int, int],
        path_length: Optional[float],
        last_direction: Optional[float] = None
    ) -> Frontier:
        """
        Calculate comprehensive score for a frontier.

        Args:
            cluster: Frontier cluster to score
            map_info: Map data and metadata
            robot: Current robot state
            valid_grid_pos: Valid grid position near cluster center
            path_length: Path length from path planner (or None)
            last_direction: Direction to previous goal (radians), for consistency

        Returns:
            Scored Frontier object
        """
        vx, vy = valid_grid_pos
        wx, wy = utils.grid_to_world(vx, vy, map_info)

        # Calculate distances
        euclidean_dist = utils.euclidean_distance(wx, wy, robot.x, robot.y)
        actual_dist = path_length if path_length else euclidean_dist

        # Calculate scores
        info_gain = self.calculate_information_gain(vx, vy, map_info.data)
        info_gain_normalized = min(info_gain / 100.0, 1.0)

        openness = self.calculate_openness(vx, vy, map_info.data)

        # Apply wall penalty
        if self.is_near_wall(vx, vy, map_info.data):
            openness *= Config.WALL_PENALTY_FACTOR

        # Compute component scores
        dist_score = 1.0 / (actual_dist + 0.1)
        size_score = min(cluster.size / 20.0, 1.0)

        # Direction consistency score (prefer same direction as previous goal)
        direction_score = 0.5  # Default: neutral
        if last_direction is not None and euclidean_dist > 0.5:
            # Calculate direction to this frontier
            frontier_direction = math.atan2(wy - robot.y, wx - robot.x)
            # Calculate angle difference (0 = same direction, π = opposite)
            angle_diff = abs(utils.normalize_angle(frontier_direction - last_direction))
            # Convert to score: 0° → 1.0, 90° → 0.5, 180° → 0.0
            direction_score = 1.0 - (angle_diff / math.pi)

        # Weighted combination
        score = (
            info_gain_normalized * Config.WEIGHT_INFO_GAIN +
            openness * Config.WEIGHT_OPENNESS +
            dist_score * Config.WEIGHT_DISTANCE +
            size_score * Config.WEIGHT_SIZE +
            direction_score * Config.WEIGHT_DIRECTION
        )

        return Frontier(
            world_pos=(wx, wy),
            grid_pos=(vx, vy),
            size=cluster.size,
            euclidean_dist=euclidean_dist,
            path_dist=actual_dist,
            info_gain=info_gain,
            openness=openness,
            score=score
        )

    def select_best_frontier(
        self,
        frontiers: list,
        min_dist: float,
        max_dist: float,
        unreachable: Set[Tuple[int, int]]
    ) -> Optional[Frontier]:
        """
        Select the best frontier from scored candidates.

        Args:
            frontiers: List of scored Frontier objects
            min_dist: Minimum distance filter
            max_dist: Maximum distance filter
            unreachable: Set of unreachable frontier regions

        Returns:
            Best Frontier or None
        """
        best = None
        best_score = -1

        for frontier in frontiers:
            # Distance filter
            if frontier.euclidean_dist < min_dist or frontier.euclidean_dist > max_dist:
                continue

            # Unreachable filter
            gx, gy = frontier.grid_pos
            frontier_key = (gx // Config.UNREACHABLE_GRID_DIVISOR,
                           gy // Config.UNREACHABLE_GRID_DIVISOR)
            if frontier_key in unreachable:
                continue

            if frontier.score > best_score:
                best_score = frontier.score
                best = frontier

        return best
