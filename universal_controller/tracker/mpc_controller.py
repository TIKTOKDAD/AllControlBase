"""MPC 轨迹跟踪控制器"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging
import weakref
import gc
import os

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.indices import MPCStateIdx
from ..core.enums import PlatformType
from ..config.default_config import PLATFORM_CONFIG

# New imports from split modules
from .mpc_core.solver_manager import MPCSolverManager, ACADOS_AVAILABLE
from .mpc_core.reference_generator import MPCReferenceGenerator

logger = logging.getLogger(__name__)

class MPCController(ITrajectoryTracker):
    """MPC 轨迹跟踪控制器
    
    Refactored to delegate logic to mpc_core components.
    """
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        mpc_config = config.get('mpc', config)
        
        self.horizon = mpc_config.get('horizon', 20)
        self.dt = mpc_config.get('dt', 0.1)
        self.output_frame = platform_config.get('output_frame', 'base_link')
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        
        self.visualize_prediction = mpc_config.get('visualize_prediction', False)
        
        # Unique Model Name Logic
        pid = os.getpid()
        self._model_name_base = f"mpc_model_pid{pid}_id{id(self)}_{self.platform_type.name}"
        
        # Initialize Managers
        self.solver_manager = MPCSolverManager(config, platform_config, self._model_name_base)
        self.ref_generator = MPCReferenceGenerator(self.horizon, self.dt)
        
        self._solver = None
        self._is_initialized = False
        self._acados_creation_failed = False
        
        # Resource cleanup
        self._finalizer = weakref.finalize(self, self._cleanup_resources, self.solver_manager)
        
        if ACADOS_AVAILABLE:
            self._initialize_solver(self.horizon)
            
            # Pre-warm
            prewarm_horizons = mpc_config.get('prewarm_horizons', [10, 20])
            for h in prewarm_horizons:
                if h != self.horizon:
                    try:
                        self.solver_manager.get_solver(h)
                        logger.info(f"Pre-warmed MPC solver for horizon {h}")
                    except Exception as e:
                        logger.warning(f"Failed to pre-warm solver for horizon {h}: {e}")
        else:
            logger.warning("ACADOS library not installed. MPC Controller will not function.")
        
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_cmd: Optional[ControlOutput] = None
        self._last_predicted_next_state: Optional[np.ndarray] = None
    
    @staticmethod
    def _cleanup_resources(manager):
        if manager:
            manager.clear_cache()
        gc.collect()

    def _initialize_solver(self, horizon: int = None) -> None:
        if not ACADOS_AVAILABLE:
            self._is_initialized = False
            return
            
        target_horizon = horizon if horizon is not None else self.horizon
        
        try:
            solver = self.solver_manager.get_solver(target_horizon)
            self._solver = solver
            self.horizon = target_horizon
            self._is_initialized = True
            self._acados_creation_failed = False
            
            # Update horizon in ref generator
            self.ref_generator.resize_buffer_if_needed(target_horizon)
            
            logger.info(f"ACADOS MPC solver initialized/switched to horizon {target_horizon}")
            
        except Exception as e:
            logger.error(f"ACADOS initialization failed: {e}")
            self._solver = None
            self._is_initialized = False
            self._acados_creation_failed = True
            gc.collect()

    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        start_time = time.time()
        
        if not ACADOS_AVAILABLE:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'acados_not_installed'})
        
        if not self._is_initialized:
            if not self._acados_creation_failed:
                self._initialize_solver()
            if not self._is_initialized:
                return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                                   health_metrics={'error_type': 'solver_not_initialized'})

        if len(trajectory.points) < 2:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'traj_too_short'})
        
        if len(state) > 8:
            state = state[:8]
        
        try:
            # Generate References
            yrefs, ps, yref_e = self.ref_generator.generate_references(state, trajectory, consistency)
            
            # Setup Solver
            solver_set = self._solver.set
            
            # Initial state
            solver_set(0, 'lbx', state)
            solver_set(0, 'ubx', state)
            
            # Set references
            horizon = self.horizon # Ensure local var matches
            
            # Note: yrefs is guaranteed contiguous by generator
            for i in range(horizon):
                solver_set(i, 'yref', yrefs[i])
                solver_set(i, 'p', ps[i])
                
            solver_set(horizon, 'yref', yref_e)
            solver_set(horizon, 'p', ps[-1])
            
            # Solve
            status = self._solver.solve()
            
            self._last_kkt_residual = self._solver.get_residuals()[0] if hasattr(self._solver, 'get_residuals') else 0.0
            
            if status == 0:
                x1 = self._solver.get(1, "x")
                self._last_predicted_next_state = x1
                
                # Extract Output
                vx_world = x1[MPCStateIdx.VX]
                vy_world = x1[MPCStateIdx.VY]
                vz_world = x1[MPCStateIdx.VZ]
                
                use_world_frame = (self.output_frame or '').lower() in ('world', 'map', 'odom')
                if use_world_frame:
                    vx_out, vy_out = vx_world, vy_world
                else:
                    current_theta = state[MPCStateIdx.THETA]
                    cos_th = np.cos(current_theta)
                    sin_th = np.sin(current_theta)
                    
                    vx_out = vx_world * cos_th + vy_world * sin_th
                    vy_out = -vx_world * sin_th + vy_world * cos_th
                
                result = ControlOutput(
                    vx=vx_out, 
                    vy=vy_out, 
                    vz=vz_world,
                    omega=x1[MPCStateIdx.OMEGA],
                    frame_id=self.output_frame, 
                    success=True,
                    health_metrics={'kkt_residual': self._last_kkt_residual}
                )
                
                if self.visualize_prediction:
                    pred_states = []
                    solver_get = self._solver.get
                    for i in range(self.horizon + 1):
                        pred_states.append(solver_get(i, "x"))
                    result.extras['predicted_trajectory'] = pred_states
            else:
                logger.warning(f"ACADOS solve failed with status {status}")
                result = ControlOutput(
                    vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                    health_metrics={'error_type': 'solver_failure', 'status': status}
                )
            
            result.solve_time_ms = (time.time() - start_time) * 1000
            self._last_solve_time_ms = result.solve_time_ms
            self._last_cmd = result
            return result
                
        except Exception as e:
            if isinstance(e, (AttributeError, NameError, TypeError, SyntaxError)):
                raise e
            logger.exception(f"MPC compute error: {e}")
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'compute_exception', 'msg': str(e)})

    def _release_solver_resources(self) -> None:
        self._solver = None
        if self.solver_manager:
            self.solver_manager.clear_cache()

    # =========================================================================
    # ITrajectoryTracker Interface Implementation
    # =========================================================================

    def reset(self) -> None:
        """
        Reset controller state.
        
        Note: We do not destroy the solver instance here to avoid cold-start overhead.
        We only reset the internal state tracking variables.
        """
        self._last_cmd = None
        self._last_predicted_next_state = None
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        # If necessary, we could call self._initialize_solver(self.horizon) again,
        # but usually simply clearing the result cache is enough.

    def set_horizon(self, horizon: int) -> bool:
        """
        Dynamically update the MPC prediction horizon.
        
        Args:
            horizon: New horizon length (N)
            
        Returns:
            bool: True if horizon was changed (or already set), False on failure.
        """
        if horizon == self.horizon and self._is_initialized:
            return True
            
        logger.info(f"MPCController: Switching horizon from {self.horizon} to {horizon}")
        try:
            self._initialize_solver(horizon)
            return self._is_initialized
        except Exception as e:
            logger.error(f"Failed to set horizon to {horizon}: {e}")
            return False

    def get_health_metrics(self) -> Dict[str, Any]:
        """
        Get current health metrics of the controller.
        
        Returns:
            Dict containing initialized status, solve times, residuals, etc.
        """
        return {
            'healthy': self._is_initialized,
            'initialized': self._is_initialized,
            'solve_time_ms': self._last_solve_time_ms,
            'kkt_residual': self._last_kkt_residual,
            'horizon': self.horizon,
            'acados_available': ACADOS_AVAILABLE,
            'acados_creation_failed': self._acados_creation_failed
        }

    def get_predicted_next_state(self) -> Optional[np.ndarray]:
        """
        Get the state predicted by MPC for the next time step.
        Useful for calculating prediction error.
        """
        return self._last_predicted_next_state
