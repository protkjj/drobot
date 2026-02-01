"""
Data type definitions for Auto Explorer.

Provides structured data classes for type safety and clarity.
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import numpy as np


@dataclass
class Frontier:
    """Scored frontier ready for navigation."""
    world_pos: Tuple[float, float]
    grid_pos: Tuple[int, int]
    size: int
    euclidean_dist: float
    path_dist: float
    info_gain: float
    openness: float
    score: float


@dataclass
class FrontierCluster:
    """Raw frontier cluster from detection."""
    center_grid: Tuple[int, int]
    size: int
    points: np.ndarray = field(repr=False)  # Hide in repr for readability


@dataclass
class RobotState:
    """Robot pose and velocity state."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class MapInfo:
    """Map metadata for coordinate transformations."""
    data: Optional[np.ndarray] = field(default=None, repr=False)
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: int = 0
    height: int = 0

    def is_valid(self) -> bool:
        """Check if map data is available."""
        return self.data is not None and self.width > 0 and self.height > 0


@dataclass
class ExplorationStats:
    """Exploration progress statistics."""
    total_goals: int = 0
    successful_goals: int = 0
    consecutive_failures: int = 0
    coverage: float = 0.0
    stuck_count: int = 0

    @property
    def success_rate(self) -> float:
        """Calculate goal success rate."""
        if self.total_goals == 0:
            return 0.0
        return (self.successful_goals / self.total_goals) * 100


@dataclass
class NavigationState:
    """Current navigation state."""
    is_navigating: bool = False
    goal_handle: Optional[object] = field(default=None, repr=False)
    current_goal: Optional[Tuple[float, float]] = None
    goal_start_time: Optional[float] = None
    nav_start_pos: Optional[Tuple[float, float]] = None
    collision_count: int = 0  # Collisions for current goal
