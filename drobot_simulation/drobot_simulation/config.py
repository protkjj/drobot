"""
Configuration constants for Auto Explorer.

All magic numbers are centralized here for easy tuning and maintenance.
"""
import math


class Config:
    """Exploration configuration constants."""

    # ==================== Collision Avoidance ====================
    COLLISION_STOP_DIST = 0.38      # m - emergency stop (LiDAR min + 약간)
    COLLISION_WARN_DIST = 0.50      # m - slow down threshold
    COLLISION_CHECK_ANGLE = math.pi / 3  # 60 degrees (양쪽 30도씩 - 넓은 감지)
    COLLISION_COOLDOWN = 3.0        # sec - cooldown between collision events

    # ==================== Navigation ====================
    MIN_FRONTIER_SIZE = 3           # minimum frontier cluster size
    EXPLORATION_TIMEOUT = 20.0      # sec - max time per goal
    SAFETY_MARGIN = 10              # grid cells (10 * 0.05m = 0.50m) - inflation 영역 + 여유
    MIN_GOAL_DISTANCE = 0.4         # m - minimum distance to goal
    WAYPOINT_INTERVAL = 1.5         # m - distance between waypoints
    INFO_GAIN_RADIUS = 40           # grid cells (~2m) - information gain calc radius
    DISTANCE_STAGES = [2.0, 4.0, 6.0, 10.0, 15.0]  # m - progressive distance stages

    # ==================== Recovery ====================
    BACKUP_SPEED = -0.08            # m/s - gentle backup
    ROTATION_SPEED = 0.3            # rad/s - recovery rotation
    AGGRESSIVE_BACKUP_SPEED = -0.15 # m/s - oscillation recovery
    RECOVERY_BACKUP_SPEED = -0.1    # m/s - normal recovery
    RECOVERY_ROTATION_SPEED = 0.5   # rad/s - recovery rotation
    AGGRESSIVE_ROTATION_SPEED = 0.8 # rad/s - oscillation rotation

    # ==================== Timers ====================
    EXPLORE_INTERVAL = 0.8          # sec - main exploration loop
    TIMEOUT_CHECK_INTERVAL = 0.5    # sec - goal timeout check
    COLLISION_CHECK_INTERVAL = 0.1  # sec - collision detection

    # ==================== Memory Management ====================
    MAX_FAILED_GOALS = 100          # limit failed goals set size
    MAX_UNREACHABLE_FRONTIERS = 100 # limit unreachable frontiers set size
    POSITION_HISTORY_SIZE = 20      # oscillation detection window
    OSCILLATION_THRESHOLD = 8       # same position repeat count

    # ==================== Frontier Scoring Weights ====================
    WEIGHT_INFO_GAIN = 0.20         # information gain weight (낮춤)
    WEIGHT_OPENNESS = 0.35          # open space preference weight (높임 - 열린 공간 강하게 선호)
    WEIGHT_DISTANCE = 0.15          # distance weight (closer is better)
    WEIGHT_SIZE = 0.10              # frontier size weight
    WEIGHT_DIRECTION = 0.20         # direction consistency weight (같은 방향 선호)

    # ==================== Openness Calculation ====================
    OPENNESS_RADIUS = 12            # grid cells - openness check radius
    OPENNESS_MIN_SCORE = 0.3        # minimum openness score
    WALL_PENALTY_FACTOR = 0.3       # multiply openness for wall-adjacent frontiers (강한 페널티)
    WALL_CHECK_THRESHOLD = 8        # grid cells - wall proximity check

    # ==================== Path Checking ====================
    PATH_CHECK_TIMEOUT = 0.8        # sec - path existence check timeout
    PROGRESS_CHECK_TIME = 10.0      # sec - check movement progress after this
    MIN_PROGRESS_DISTANCE = 0.5     # m - minimum movement in progress check time

    # ==================== Random Goal ====================
    RANDOM_GOAL_MIN_DIST = 20       # grid cells - min distance for random goal
    RANDOM_GOAL_MAX_DIST = 60       # grid cells - max distance for random goal

    # ==================== Goal Repetition Prevention ====================
    SAME_TARGET_MAX_COUNT = 2       # block goal after this many repeats
    FAILED_GOAL_GRID_DIVISOR = 5    # grid discretization for failed goals
    BLOCKED_GOAL_GRID_DIVISOR = 2   # grid discretization for blocked goals
    UNREACHABLE_GRID_DIVISOR = 3    # grid discretization for unreachable frontiers
