"""
Utility functions for Auto Explorer.

Common coordinate transformations and math utilities.
"""
import math
from typing import Tuple, Optional

from drobot_simulation.types import MapInfo


def grid_to_world(gx: int, gy: int, map_info: MapInfo) -> Tuple[Optional[float], Optional[float]]:
    """
    Convert grid coordinates to world coordinates.

    Args:
        gx: Grid x coordinate
        gy: Grid y coordinate
        map_info: Map metadata

    Returns:
        Tuple of (world_x, world_y) or (None, None) if invalid
    """
    if not map_info.is_valid():
        return None, None

    wx = map_info.origin_x + (gx + 0.5) * map_info.resolution
    wy = map_info.origin_y + (gy + 0.5) * map_info.resolution
    return wx, wy


def world_to_grid(wx: float, wy: float, map_info: MapInfo) -> Tuple[Optional[int], Optional[int]]:
    """
    Convert world coordinates to grid coordinates.

    Args:
        wx: World x coordinate
        wy: World y coordinate
        map_info: Map metadata

    Returns:
        Tuple of (grid_x, grid_y) or (None, None) if invalid
    """
    if not map_info.is_valid():
        return None, None

    gx = int((wx - map_info.origin_x) / map_info.resolution)
    gy = int((wy - map_info.origin_y) / map_info.resolution)
    return gx, gy


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Calculate Euclidean distance between two points.

    Args:
        x1, y1: First point coordinates
        x2, y2: Second point coordinates

    Returns:
        Distance between the points
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Extract yaw angle from quaternion.

    Args:
        x, y, z, w: Quaternion components

    Returns:
        Yaw angle in radians
    """
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw angle to quaternion.

    Args:
        yaw: Yaw angle in radians

    Returns:
        Tuple of (x, y, z, w) quaternion components
    """
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def calculate_path_length(poses: list) -> float:
    """
    Calculate total path length from a list of poses.

    Args:
        poses: List of PoseStamped messages

    Returns:
        Total path length in meters
    """
    if not poses or len(poses) < 2:
        return 0.0

    length = 0.0
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        length += math.sqrt(dx * dx + dy * dy)
    return length


def is_in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    """
    Check if grid coordinates are within map bounds.

    Args:
        gx, gy: Grid coordinates
        width, height: Map dimensions

    Returns:
        True if coordinates are valid
    """
    return 0 <= gx < width and 0 <= gy < height


def discretize_position(x: float, y: float, resolution: float = 0.5) -> Tuple[float, float]:
    """
    Discretize position for history tracking.

    Args:
        x, y: Position coordinates
        resolution: Discretization resolution

    Returns:
        Discretized (x, y) tuple
    """
    return (round(x / resolution) * resolution, round(y / resolution) * resolution)
