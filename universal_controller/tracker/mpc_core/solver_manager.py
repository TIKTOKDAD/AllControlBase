from typing import Dict, Any, Optional, List
from collections import OrderedDict
import numpy as np
import logging
import gc
import os

try:
    from acados_template import AcadosOcp, AcadosOcpSolver
    ACADOS_AVAILABLE = True
    ACADOS_INF = 1e9
except ImportError:
    ACADOS_AVAILABLE = False
    AcadosOcp = None
    AcadosOcpSolver = None
    ACADOS_INF = 1e9

from ...core.constants import (
    EPSILON,
    MPC_QP_SOLVER,
    MPC_INTEGRATOR_TYPE,
    MPC_NLP_SOLVER_TYPE,
    MPC_NLP_MAX_ITER,
)
from ...core.enums import PlatformType
from .mpc_model import create_mpc_model

logger = logging.getLogger(__name__)

class MPCSolverManager:
    """
    Manages ACADOS solver instances, including creation, caching, and resource cleanup.
    """
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any], model_name_base: str):
        self.config = config
        self.platform_config = platform_config
        self._model_name_base = model_name_base
        
        mpc_config = config.get('mpc', config)
        self.dt = mpc_config.get('dt', 0.1)
        
        # Constraints
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.v_min = constraints.get('v_min', 0.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.a_max = constraints.get('a_max', 1.5)
        self.vz_max = constraints.get('vz_max', 2.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)
        self.max_curvature = constraints.get('max_curvature', 10.0)
        
        # Platform Details
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.can_rotate_in_place = platform_config.get(
            'can_rotate_in_place', 
            self.platform_type != PlatformType.ACKERMANN
        )
        
        # Weights
        mpc_weights = mpc_config.get('weights', {})
        self.Q_pos = max(mpc_weights.get('position', 10.0), EPSILON)
        self.Q_vel = max(mpc_weights.get('velocity', 1.0), EPSILON)
        self.Q_heading = max(mpc_weights.get('heading', 5.0), EPSILON)
        self.R_accel = max(mpc_weights.get('control_accel', 0.1), EPSILON)
        self.R_alpha = max(mpc_weights.get('control_alpha', 0.1), EPSILON)
        
        # Solver Settings
        self.nlp_max_iter = MPC_NLP_MAX_ITER
        self.qp_solver = MPC_QP_SOLVER
        self.integrator_type = MPC_INTEGRATOR_TYPE
        self.nlp_solver_type = MPC_NLP_SOLVER_TYPE
        
        # Cache - Use OrderedDict for O(1) LRU
        self._solver_cache = OrderedDict()
        self._solver_cache_max_size = mpc_config.get('solver_cache_max_size', 5)
        self._solver_lib_path = {}

    def get_solver(self, horizon: int) -> Optional[Any]:
        """Returns a cached solver or creates a new one."""
        if not ACADOS_AVAILABLE:
            return None
            
        if horizon in self._solver_cache:
            # LRU update: move accessed item to end (most recently used)
            self._solver_cache.move_to_end(horizon)
            return self._solver_cache[horizon]
        
        # Eviction: Remove first item (least recently used) if cache is full
        if len(self._solver_cache) >= self._solver_cache_max_size:
            oldest_horizon, solver_to_remove = self._solver_cache.popitem(last=False)
            logger.debug(f"Evicted MPC solver for horizon {oldest_horizon} from cache (LRU)")
            # Explicitly delete the solver object to trigger any internal cleanup (e.g., in __del__)
            del solver_to_remove
            # Immediately trigger GC to release ACADOS C resources
            # This prevents resource accumulation during high-frequency horizon switching
            gc.collect()
        
        return self._create_solver(horizon)

    def _create_solver(self, horizon: int):
        model_name = f'{self._model_name_base}_h{horizon}'
        
        # Call model generator
        model = create_mpc_model(model_name, self.platform_type, self.can_rotate_in_place, self.max_curvature)
        if model is None:
            return None
            
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = horizon
        ocp.solver_options.tf = horizon * self.dt
        
        # Cost setup
        ny = 8 + 4
        ny_e = 8
        Q = np.diag([self.Q_pos, self.Q_pos, self.Q_pos,
                     self.Q_vel, self.Q_vel, self.Q_vel,
                     self.Q_heading, 0.1])
        R = np.diag([self.R_accel, self.R_accel, self.R_accel, self.R_alpha])
        
        W = np.zeros((ny, ny))
        W[:8, :8] = Q
        W[8:12, 8:12] = R
        
        Vx_ext = np.zeros((ny, 8))
        Vx_ext[:8, :8] = np.eye(8)
        Vu_ext = np.zeros((ny, 4))
        Vu_ext[8:12, :4] = np.eye(4)
        
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = W
        ocp.cost.W_e = Q
        ocp.cost.Vx = Vx_ext
        ocp.cost.Vu = Vu_ext
        ocp.cost.Vx_e = np.eye(8)
        
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)
        
        # Constraints setup
        ocp.constraints.lbu = np.array([-self.a_max, -self.a_max, -self.a_max, -self.alpha_max])
        ocp.constraints.ubu = np.array([self.a_max, self.a_max, self.a_max, self.alpha_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        
        ocp.constraints.lbx = np.array([-ACADOS_INF, -ACADOS_INF, -ACADOS_INF, 
                                       -self.v_max, -self.v_max, -self.vz_max,
                                       -ACADOS_INF, -self.omega_max])
        ocp.constraints.ubx = np.array([ACADOS_INF, ACADOS_INF, ACADOS_INF,
                                       self.v_max, self.v_max, self.vz_max,
                                       ACADOS_INF, self.omega_max])
        ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        
        if not self.can_rotate_in_place:
             # Additional constraint setup for Ackermann/Car-like
             ocp.dims.nh = 1
             ocp.constraints.lh = np.array([-ACADOS_INF])
             ocp.constraints.uh = np.array([0.0])
        
        ocp.constraints.x0 = np.zeros(8)
        ocp.parameter_values = np.zeros(7)
        
        ocp.solver_options.qp_solver = self.qp_solver
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = self.integrator_type
        ocp.solver_options.nlp_solver_type = self.nlp_solver_type
        ocp.solver_options.nlp_solver_max_iter = self.nlp_max_iter
        
        json_file = f'{model_name}.json'
        
        generate_code = True
        build_code = True
        
        if horizon in self._solver_lib_path:
            generate_code = False
            build_code = False
            logger.debug(f"Reusing existing solver library for horizon {horizon}")

        solver = AcadosOcpSolver(ocp, json_file=json_file, generate=generate_code, build=build_code)
        self._solver_lib_path[horizon] = True
        
        self._solver_cache[horizon] = solver
        return solver

    def clear_cache(self):
        """Clear the solver cache and release resources.
        
        设计说明:
        - 不直接调用 __del__()，因为这是 Python 反模式
        - ACADOS solver 的 __del__ 会在对象被 GC 时自动调用
        - 显式 del 引用 + gc.collect() 确保及时释放
        """
        # 清空缓存并删除引用
        while self._solver_cache:
            _, solver = self._solver_cache.popitem()
            # 删除引用，让 Python GC 自动调用 __del__
            del solver
        # Note: 无需调用 clear()，while 循环已将缓存完全清空
        
        # 显式触发垃圾回收以确保 ACADOS C 资源及时释放
        gc.collect()
