"""Pure Pursuit 备用控制器"""
from typing import Dict, Any, Optional, List, Tuple
import numpy as np
import logging

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.enums import PlatformType
from ..core.velocity_smoother import VelocitySmoother
from ..config.default_config import PLATFORM_CONFIG

from .pure_pursuit_core.lookahead import LookaheadSearcher
from .pure_pursuit_core.control_law import PurePursuitControlLaw

logger = logging.getLogger(__name__)

class PurePursuitController(ITrajectoryTracker):
    """Pure Pursuit 备用控制器
    
    Refactored to delegate logic to pure_pursuit_core components.
    """
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        backup_config = config.get('backup', {})
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        
        # Init helpers
        self.control_law = PurePursuitControlLaw(platform_config, constraints, backup_config, 
                                                 platform_config.get('output_frame', 'base_link'))
        
        self.lookahead_searcher = LookaheadSearcher(
            lookahead_dist=backup_config['lookahead_dist'],
            min_lookahead=backup_config['min_lookahead'],
            max_lookahead=backup_config['max_lookahead'],
            lookahead_ratio=backup_config['lookahead_ratio'],
            v_max=constraints.get('v_max', 2.0),
            default_speed_ratio=backup_config['default_speed_ratio'],
            v_min=constraints.get('v_min', 0.0)
        )
        
        # Local state
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.output_type = platform_config.get('output_type', 'differential')
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        
        self.dt = 1.0 / config.get('system', {}).get('ctrl_freq', 50)
        
        # ITrajectoryTracker 接口一致性：虽然 Pure Pursuit 不使用 horizon 概念，
        # 但需要提供属性以满足接口契约
        self._horizon: int = 0
        
        self.kp_z = backup_config['kp_z']
        self.vz_max = constraints.get('vz_max', 2.0)
        
        self.omega_max = constraints.get('omega_max', 2.0)
        self.omega_max_low = constraints.get('omega_max_low', 1.0)
        self.v_low_thresh = constraints.get('v_low_thresh', 0.1)
        self.low_speed_transition_factor = backup_config['low_speed_transition_factor']
        
        self.last_cmd: Optional[ControlOutput] = None
        self._current_position: Optional[np.ndarray] = None
        self._is_shutdown = False
        
        # Velocity smoother
        self._velocity_smoother = VelocitySmoother(
            a_max=constraints.get('a_max', 1.5), 
            az_max=constraints.get('az_max', 1.0), 
            alpha_max=constraints.get('alpha_max', 3.0), 
            dt=self.dt
        )

    def set_manual_heading(self, heading: float) -> None:
        self.control_law.set_manual_heading(heading)

    def reset(self) -> None:
        self.last_cmd = None
        self._current_position = None
        self.lookahead_searcher.reset_state()
        self.control_law.reset_state()
    
    def shutdown(self) -> None:
        self._is_shutdown = True
        self.reset()
    
    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        if self._is_shutdown:
            logger.warning("PurePursuitController.compute() called after shutdown, returning stop command")
            return self._smooth_stop_command()
        
        if len(trajectory.points) < 2:
            return self._smooth_stop_command()
        
        px, py, pz = state[0], state[1], state[2]
        current_v = np.sqrt(state[3]**2 + state[4]**2)
        theta = state[6]
        
        self._current_position = np.array([px, py, pz])
        points_mat = trajectory.get_points_matrix()
        
        # Lookahead
        lookahead_point, target_idx = self.lookahead_searcher.find_lookahead_point(
            points_mat, px, py, current_v)
        
        if lookahead_point is None:
            return self._smooth_stop_command()
        
        dx = lookahead_point[0] - px
        dy = lookahead_point[1] - py
        dist_to_target = np.sqrt(dx**2 + dy**2)
        target_v = self.lookahead_searcher.compute_target_velocity(trajectory, target_idx, consistency.alpha)
        
        # Omega Limit
        omega_limit = self._get_omega_limit(current_v)
        
        # Compute Control
        if self.output_type == 'differential':
            cmd = self.control_law.compute_differential(
                dx, dy, theta, target_v, dist_to_target, omega_limit, self.last_cmd)
        elif self.output_type == 'omni':
             cmd = self.control_law.compute_omni(
                 dx, dy, theta, target_v, dist_to_target, omega_limit, trajectory, target_idx, self._current_position)
        else:
             # 3D
             vz = 0.0
             if target_idx < len(points_mat):
                 target_z = points_mat[target_idx, 2]
                 vz = np.clip(self.kp_z * (target_z - pz), -self.vz_max, self.vz_max)
             
             cmd = self.control_law.compute_omni(
                 dx, dy, theta, target_v, dist_to_target, omega_limit, trajectory, target_idx, self._current_position)
             cmd.vz = vz
             
        # Smoothing
        cmd = self._velocity_smoother.smooth(cmd, self.last_cmd)
        
        # Omni Constraints (Clip)
        if self.is_omni:
             # Simplistic clipping, ideally moved to ControlLaw or Smoother
             cmd.vx = np.clip(cmd.vx, self.control_law.vx_min, self.control_law.vx_max)
             cmd.vy = np.clip(cmd.vy, self.control_law.vy_min, self.control_law.vy_max)
        
        self.last_cmd = cmd
        return cmd
    
    def _smooth_stop_command(self) -> ControlOutput:
        cmd = self._velocity_smoother.smooth_to_stop(self.last_cmd, self.control_law.output_frame)
        self.last_cmd = cmd
        return cmd
        
    def _get_omega_limit(self, current_v: float) -> float:
        low_speed_boundary = self.v_low_thresh * self.low_speed_transition_factor
        if low_speed_boundary >= self.v_low_thresh:
            return self.omega_max_low if current_v < self.v_low_thresh else self.omega_max
            
        if current_v < low_speed_boundary:
            return self.omega_max_low
        elif current_v < self.v_low_thresh:
            ratio = (current_v - low_speed_boundary) / (self.v_low_thresh - low_speed_boundary)
            return self.omega_max_low + ratio * (self.omega_max - self.omega_max_low)
        return self.omega_max

    def get_health_metrics(self) -> Dict[str, Any]:
        return {'type': 'pure_pursuit', 'active': True}
    
    def set_horizon(self, horizon: int) -> bool:
        # Pure Pursuit 不使用 horizon 概念，但需要保持接口一致性
        self._horizon = horizon
        return True
    
    def get_predicted_next_state(self) -> Optional[np.ndarray]:
        return None
