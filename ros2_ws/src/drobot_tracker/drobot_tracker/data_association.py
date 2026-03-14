import numpy as np
from scipy.optimize import linear_sum_assignment


def associate_detections_to_tracks(detections, tracks, gate_threshold=2.0):
    """
    Match detections to tracks using Hungarian algorithm with distance gating.

    Args:
        detections: list of 3D positions [[x,y,z], ...]
        tracks: list of Track objects (must have .predicted_position property)
        gate_threshold: maximum distance for valid assignment (meters)

    Returns:
        matches: list of (detection_idx, track_idx) tuples
        unmatched_detections: list of detection indices
        unmatched_tracks: list of track indices
    """
    if len(detections) == 0 and len(tracks) == 0:
        return [], [], []

    if len(tracks) == 0:
        return [], list(range(len(detections))), []

    if len(detections) == 0:
        return [], [], list(range(len(tracks)))

    # Build cost matrix: Euclidean distance between each detection and track
    cost_matrix = np.zeros((len(detections), len(tracks)))
    for d_idx, det_pos in enumerate(detections):
        for t_idx, track in enumerate(tracks):
            cost_matrix[d_idx, t_idx] = np.linalg.norm(
                np.array(det_pos) - np.array(track.predicted_position)
            )

    # Apply gate: set costs above threshold to a large value
    gated_cost = cost_matrix.copy()
    gated_cost[gated_cost > gate_threshold] = gate_threshold + 1e6

    # Hungarian assignment
    det_indices, trk_indices = linear_sum_assignment(gated_cost)

    matches = []
    unmatched_detections = set(range(len(detections)))
    unmatched_tracks = set(range(len(tracks)))

    for d_idx, t_idx in zip(det_indices, trk_indices):
        if cost_matrix[d_idx, t_idx] <= gate_threshold:
            matches.append((d_idx, t_idx))
            unmatched_detections.discard(d_idx)
            unmatched_tracks.discard(t_idx)

    return matches, sorted(unmatched_detections), sorted(unmatched_tracks)
