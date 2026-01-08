from typing import Any
import sys
import logging

try:
    from acados_template import AcadosModel
    import casadi as ca
    ACADOS_AVAILABLE = True
    # ACADOS constants
    ACADOS_INF = 1e9
except ImportError:
    ACADOS_AVAILABLE = False
    AcadosModel = None
    ca = None
    ACADOS_INF = 1e9

from ...core.enums import PlatformType
from ...core.indices import MPCStateIdx

logger = logging.getLogger(__name__)

def create_mpc_model(model_name: str, 
                     platform_type: PlatformType, 
                     can_rotate_in_place: bool, 
                     max_curvature: float) -> Any:
    """
    Creates and returns an AcadosModel for the specified platform configuration.
    Returns None if Acados is not available.
    """
    if not ACADOS_AVAILABLE:
        return None

    model = AcadosModel()
    model.name = model_name
    
    # State variables: [px, py, pz, vx, vy, vz, theta, omega]
    px = ca.SX.sym('px')
    py = ca.SX.sym('py')
    pz = ca.SX.sym('pz')
    vx = ca.SX.sym('vx')
    vy = ca.SX.sym('vy')
    vz = ca.SX.sym('vz')
    theta = ca.SX.sym('theta')
    omega = ca.SX.sym('omega')
    x = ca.vertcat(px, py, pz, vx, vy, vz, theta, omega)
    
    # Control inputs: [ax, ay, az, alpha]
    ax = ca.SX.sym('ax')
    ay = ca.SX.sym('ay')
    az = ca.SX.sym('az')
    alpha = ca.SX.sym('alpha')
    u = ca.vertcat(ax, ay, az, alpha)
    
    # Parameters: [px_ref, py_ref, pz_ref, vx_ref, vy_ref, vz_ref, theta_ref]
    p = ca.SX.sym('p', 7)
    
    # Platform-specific dynamics
    is_3d = (platform_type == PlatformType.QUADROTOR)
    is_omni = (platform_type == PlatformType.OMNI)

    if is_omni or is_3d:
        # Omni/3D dynamics
        theta_fn = theta
        
        c_th = ca.cos(theta_fn)
        s_th = ca.sin(theta_fn)
        
        # ax, ay are Body accel, need World accel for state update
        ax_world = ax * c_th - ay * s_th
        ay_world = ax * s_th + ay * c_th
        
        xdot_expr = [0] * 8
        xdot_expr[MPCStateIdx.X] = vx
        xdot_expr[MPCStateIdx.Y] = vy
        xdot_expr[MPCStateIdx.Z] = vz
        xdot_expr[MPCStateIdx.VX] = ax_world
        xdot_expr[MPCStateIdx.VY] = ay_world
        xdot_expr[MPCStateIdx.VZ] = az 
        xdot_expr[MPCStateIdx.THETA] = omega
        xdot_expr[MPCStateIdx.OMEGA] = alpha
        xdot = ca.vertcat(*xdot_expr)
    else:
        # Differential / Ackermann
        theta_fn = theta
        v_forward = vx * ca.cos(theta_fn) + vy * ca.sin(theta_fn)
        
        xdot_expr = [0] * 8
        xdot_expr[MPCStateIdx.X] = v_forward * ca.cos(theta_fn)
        xdot_expr[MPCStateIdx.Y] = v_forward * ca.sin(theta_fn)
        xdot_expr[MPCStateIdx.Z] = vz
        xdot_expr[MPCStateIdx.VX] = ax * ca.cos(theta_fn) - v_forward * omega * ca.sin(theta_fn)
        xdot_expr[MPCStateIdx.VY] = ax * ca.sin(theta_fn) + v_forward * omega * ca.cos(theta_fn)
        xdot_expr[MPCStateIdx.VZ] = az
        xdot_expr[MPCStateIdx.THETA] = omega
        xdot_expr[MPCStateIdx.OMEGA] = alpha
        
        xdot = ca.vertcat(*xdot_expr)
        
        # Non-holonomic constraint for Ackermann-like behavior (optional)
        if not can_rotate_in_place:
            # omega^2 - (v_forward * max_curvature)^2 <= 0
            # Note: v_forward calculation assumes velocity vector aligns with heading
            constraint_h = omega**2 - (v_forward * max_curvature)**2
            model.con_h_expr = constraint_h
    
    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = xdot
    model.xdot = ca.SX.sym('xdot', 8)
    model.f_impl_expr = model.xdot - xdot
    
    return model
