from typing import Optional, Dict, Any
import numpy as np

from ...core.data_types import ControlOutput, Trajectory
from ...core.enums import HeadingMode
from ...core.ros_compat import angle_difference
from ...core.constants import (
    EPSILON,
    PURE_PURSUIT_ANGLE_THRESH, 
    HEADING_CONTROL_ANGLE_THRESH, 
    REAR_ANGLE_THRESH
)

class PurePursuitControlLaw:
    """
    Encapsulates the geometric control laws for Pure Pursuit (Differential, Omni, 3D).
    Manages state for reversing strategies.
    """
    def __init__(self, 
                 platform_type_config: Dict[str, Any],
                 constraints: Dict[str, Any],
                 backup_config: Dict[str, Any],
                 output_frame: str):
        
        self.output_frame = output_frame
        
        # Constraints
        self.max_curvature = backup_config['max_curvature']
        self.v_max = constraints.get('v_max', 2.0)
        self.vx_min = constraints.get('vx_min', -self.v_max)
        self.vx_max = constraints.get('vx_max', self.v_max)
        self.vy_min = constraints.get('vy_min', -self.v_max)
        self.vy_max = constraints.get('vy_max', self.v_max)
        
        # Heading / Turning
        self.kp_heading = backup_config['kp_heading']
        self.heading_error_thresh = backup_config['heading_error_thresh']
        self.pure_pursuit_angle_thresh = PURE_PURSUIT_ANGLE_THRESH
        self.heading_control_angle_thresh = HEADING_CONTROL_ANGLE_THRESH
        
        
        # Reversing Logic
        self.can_rotate_in_place = platform_type_config.get('can_rotate_in_place', True)
        self.rear_angle_thresh = REAR_ANGLE_THRESH
        self.rear_direction_min_thresh = backup_config['rear_direction_min_thresh']
        default_turn_str = backup_config['default_turn_direction']
        self.default_turn_direction = 1 if default_turn_str.lower() == 'left' else -1
        self.min_distance_thresh = backup_config['min_distance_thresh']
        
        # Omega Rate Limit
        self.omega_rate_limit = backup_config.get('omega_rate_limit', 10.0)
        
        self._last_turn_direction: Optional[int] = None
        self._manual_heading: Optional[float] = None
        
        # Heading Mode
        self.heading_mode = self._parse_heading_mode(backup_config['heading_mode'])
        self.fixed_heading: Optional[float] = backup_config.get('fixed_heading')
        
    def _parse_heading_mode(self, mode_str: str) -> HeadingMode:
        return {
            'follow_velocity': HeadingMode.FOLLOW_VELOCITY,
            'fixed': HeadingMode.FIXED,
            'target_point': HeadingMode.TARGET_POINT,
            'manual': HeadingMode.MANUAL
        }.get(mode_str.lower(), HeadingMode.FOLLOW_VELOCITY)
        
    def reset_state(self):
        self._last_turn_direction = None
    
    def set_manual_heading(self, heading: float) -> None:
        self._manual_heading = heading

    def compute_differential(self, dx: float, dy: float, theta: float,
                           target_v: float, dist_to_target: float,
                           omega_limit: float, 
                           last_cmd: Optional[ControlOutput]) -> ControlOutput:
        
        # Body frame conversion
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)
        L_sq = local_x**2 + local_y**2
        
        if L_sq > EPSILON:
            target_angle = np.arctan2(local_y, local_x)
            abs_target_angle = abs(target_angle)
            
            if abs_target_angle > self.heading_control_angle_thresh:
                return self._handle_reversing(dx, dy, local_y, theta, target_v, omega_limit, last_cmd)
            elif abs_target_angle > self.pure_pursuit_angle_thresh:
                return self._handle_side_transition(dx, dy, local_y, L_sq, theta, target_v, omega_limit, abs_target_angle)
            else:
                curvature = np.clip(2.0 * local_y / L_sq, -self.max_curvature, self.max_curvature)
        else:
            curvature = 0.0
            
        omega = target_v * curvature
        omega = np.clip(omega, -omega_limit, omega_limit)
        
        return ControlOutput(vx=target_v, vy=0.0, vz=0.0, omega=omega, 
                            frame_id=self.output_frame, success=True)

    def _handle_reversing(self, dx: float, dy: float, local_y: float, theta: float, 
                          target_v: float, omega_limit: float, 
                          last_cmd: Optional[ControlOutput]) -> ControlOutput:
        
        if not self.can_rotate_in_place:
             # Reversing strategy for car-like
             reverse_speed = max(0.0, -self.vx_min)
             target_v_rev = min(target_v, reverse_speed)
             
             L_sq = dx**2 + dy**2
             curvature = 0.0
             if L_sq > EPSILON:
                 curvature = np.clip(2.0 * local_y / L_sq, -self.max_curvature, self.max_curvature)
                 
             cmd_vx = -target_v_rev
             cmd_omega = np.clip(cmd_vx * curvature, -omega_limit, omega_limit)
             return ControlOutput(vx=cmd_vx, vy=0.0, vz=0.0, omega=cmd_omega, frame_id=self.output_frame, success=True)

        # Rotate in place
        target_heading = np.arctan2(dy, dx)
        heading_error = angle_difference(target_heading, theta)
        abs_err = abs(heading_error)
        
        # Hysteresis
        active_thresh = self.rear_angle_thresh
        if self._last_turn_direction is not None:
            active_thresh = max(0.0, self.rear_angle_thresh - 0.2)
            
        if abs_err > active_thresh:
            if self._last_turn_direction is not None:
                omega_des = self._last_turn_direction * omega_limit
            elif last_cmd and abs(last_cmd.omega) > 0.1:
                self._last_turn_direction = 1 if last_cmd.omega > 0 else -1
                omega_des = self._last_turn_direction * omega_limit
            else:
                rear_thresh = max(self.min_distance_thresh * 0.5, self.rear_direction_min_thresh)
                if abs(local_y) > rear_thresh:
                    self._last_turn_direction = 1 if local_y > 0 else -1
                else:
                    self._last_turn_direction = self.default_turn_direction
                omega_des = self._last_turn_direction * omega_limit
        else:
            self._last_turn_direction = None
            omega_des = np.clip(self.kp_heading * heading_error, -omega_limit, omega_limit)
            
        # Rate limit
        if last_cmd:
            change = np.clip(omega_des - last_cmd.omega, -self.omega_rate_limit, self.omega_rate_limit)
            omega = last_cmd.omega + change
        else:
            omega = omega_des
            
        omega = np.clip(omega, -omega_limit, omega_limit)
        
        # Stop vs Slow
        if abs_err > self.heading_error_thresh:
            vx = 0.0
        else:
            factor = np.cos(abs_err) * (1.0 - abs_err / self.heading_error_thresh * 0.5)
            vx = target_v * factor
            
        return ControlOutput(vx=vx, vy=0.0, vz=0.0, omega=omega, frame_id=self.output_frame, success=True)

    def _handle_side_transition(self, dx, dy, local_y, L_sq, theta, target_v, omega_limit, abs_target_angle):
        # Blend
        norm = (abs_target_angle - self.pure_pursuit_angle_thresh) / (self.heading_control_angle_thresh - self.pure_pursuit_angle_thresh)
        blend = 0.5 * (1 - np.cos(np.pi * norm))
        
        # PP
        k = np.clip(2.0 * local_y / L_sq, -self.max_curvature, self.max_curvature)
        pp_omega = np.clip(target_v * k, -omega_limit, omega_limit)
        
        # HC
        tgt_h = np.arctan2(dy, dx)
        h_err = angle_difference(tgt_h, theta)
        hc_omega = np.clip(self.kp_heading * h_err, -omega_limit, omega_limit)
        
        omega = (1 - blend) * pp_omega + blend * hc_omega
        omega = np.clip(omega, -omega_limit, omega_limit)
        
        # Velocity blend
        if abs(h_err) > self.heading_error_thresh:
            hc_vx = 0.0
        else:
            hc_vx = target_v * np.cos(abs(h_err)) # Simplified linear factor
            
        vx = (1 - blend) * target_v + blend * hc_vx
        return ControlOutput(vx=vx, vy=0.0, vz=0.0, omega=omega, frame_id=self.output_frame, success=True)
    
    def compute_omni(self, dx, dy, theta, target_v, dist, omega_limit, trajectory, target_idx, pos):
        vx_w, vy_w = 0.0, 0.0
        if dist > EPSILON:
             vx_w = target_v * dx / dist
             vy_w = target_v * dy / dist
        
        use_world_frame = (self.output_frame or '').lower() in ('world', 'map', 'odom')
        if use_world_frame:
             vx_out, vy_out = vx_w, vy_w
        else:
             c = np.cos(theta)
             s = np.sin(theta)
             vx_out = vx_w * c + vy_w * s
             vy_out = -vx_w * s + vy_w * c
              
        omega_cmd = self._compute_heading_control(dx, dy, theta, dist, trajectory, target_idx, omega_limit, pos)
        
        # Constraints
        vx_out = np.clip(vx_out, self.vx_min, self.vx_max)
        vy_out = np.clip(vy_out, self.vy_min, self.vy_max)
        
        return ControlOutput(vx=vx_out, vy=vy_out, vz=0.0, omega=omega_cmd, frame_id=self.output_frame, success=True)
        
    def _compute_heading_control(self, dx, dy, theta, dist, trajectory, target_idx, omega_limit, pos):
         omega = 0.0
         if self.heading_mode == HeadingMode.FOLLOW_VELOCITY:
             if dist > self.min_distance_thresh:
                 tgt = np.arctan2(dy, dx)
                 omega = self.kp_heading * angle_difference(tgt, theta)
         elif self.heading_mode == HeadingMode.FIXED and self.fixed_heading is not None:
             omega = self.kp_heading * angle_difference(self.fixed_heading, theta)
         elif self.heading_mode == HeadingMode.TARGET_POINT and len(trajectory.points) > 0:
             # Note: trajectory.points is a numpy array [N, 3], not a list of Point3D objects
             # Access elements via indexing: [0]=x, [1]=y, [2]=z
             end_pt = trajectory.points[-1]
             tgt = np.arctan2(end_pt[1] - pos[1], end_pt[0] - pos[0])
             omega = self.kp_heading * angle_difference(tgt, theta)
         elif self.heading_mode == HeadingMode.MANUAL:
             if self._manual_heading is not None:
                 omega = self.kp_heading * angle_difference(self._manual_heading, theta)
         
         return np.clip(omega, -omega_limit, omega_limit)
