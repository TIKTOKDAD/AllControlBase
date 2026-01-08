import numpy as np
from typing import Tuple, Optional, Any
from ...core.data_types import Trajectory

class LookaheadSearcher:
    """
    Handles lookahead point search algorithm with windowed optimization.
    """
    def __init__(self, 
                 lookahead_dist: float, 
                 min_lookahead: float, 
                 max_lookahead: float, 
                 lookahead_ratio: float,
                 v_max: float,
                 default_speed_ratio: float,
                 v_min: float):
        self.lookahead_dist = lookahead_dist
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_ratio = lookahead_ratio
        
        self.v_max = v_max
        self.default_speed_ratio = default_speed_ratio
        self.v_min = v_min
        
        self._last_closest_idx = 0

    def reset_state(self):
        self._last_closest_idx = 0

    def compute_lookahead_dist(self, current_v: float) -> float:
        lookahead = self.lookahead_dist + self.lookahead_ratio * current_v
        return np.clip(lookahead, self.min_lookahead, self.max_lookahead)

    def find_lookahead_point(self, points: np.ndarray, px: float, py: float, 
                             current_v: float) -> Tuple[Optional[np.ndarray], int]:
        n_points = len(points)
        if n_points == 0:
            return None, 0
            
        pos = np.array([px, py])
        lookahead = self.compute_lookahead_dist(current_v)
        
        # Windowed Search Logic
        SEARCH_WINDOW = 50
        start_idx = 0
        end_idx = n_points
        
        if (self._last_closest_idx is not None and 
            n_points > SEARCH_WINDOW and 
            self._last_closest_idx < n_points):
             start_idx = max(0, self._last_closest_idx - SEARCH_WINDOW // 2)
             end_idx = min(n_points, self._last_closest_idx + SEARCH_WINDOW)
        
        points_window = points[start_idx:end_idx, :2]
        dists_sq_window = np.sum((points_window - pos)**2, axis=1)
        
        local_min_idx = np.argmin(dists_sq_window)
        min_idx = start_idx + local_min_idx
        
        # Edge case handling
        EDGE_BUFFER = 2 
        is_edge_case = False
        if start_idx > 0 and (min_idx - start_idx) < EDGE_BUFFER:
            is_edge_case = True
        elif end_idx < n_points and (end_idx - min_idx) < EDGE_BUFFER + 1:
            is_edge_case = True
            
        if is_edge_case:
             dists_sq_global = np.sum((points[:, :2] - pos)**2, axis=1)
             min_idx = np.argmin(dists_sq_global)
        
        self._last_closest_idx = min_idx
        
        # Find Target
        lookahead_sq = lookahead**2
        
        # Optimization: Search forward from min_idx
        # Check window remainder first if valid
        if not is_edge_case:
             window_future_dists_sq = dists_sq_window[local_min_idx:]
             candidates_mask = window_future_dists_sq >= lookahead_sq
             
             if np.any(candidates_mask):
                 offset = np.argmax(candidates_mask)
                 target_idx = min_idx + offset
                 return points[target_idx], target_idx
        
        # Global/Tail search
        remaining_points = points[min_idx:, :2]
        remaining_dists_sq = np.sum((remaining_points - pos)**2, axis=1)
        candidates_mask = remaining_dists_sq >= lookahead_sq
        
        if np.any(candidates_mask):
             offset = np.argmax(candidates_mask)
             target_idx = min_idx + offset
             return points[target_idx], target_idx

        return points[-1], n_points - 1

    def compute_target_velocity(self, trajectory: Trajectory, target_idx: int, 
                                  alpha: float = 1.0) -> float:
        target_v = trajectory.get_blended_speed(target_idx, alpha)
        
        if len(trajectory.points) < 2:
            target_v = self.v_max * self.default_speed_ratio
        
        target_v = min(target_v, self.v_max)
        v_min_forward = max(0.0, self.v_min)
        return np.clip(target_v, v_min_forward, self.v_max)
