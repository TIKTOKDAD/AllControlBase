"""
Status build logic for Dashboard
"""
import math
import time
from typing import Dict, Any, Optional, List, Tuple

from ..models import (
    DisplayData, EnvironmentStatus, PlatformConfig, ControllerStatus,
    MPCHealthStatus, ConsistencyStatus, TimeoutStatus, TrackingStatus,
    EstimatorStatus, TransformStatus, ControlCommand, TrajectoryData,
    SafetyStatus, ControllerStateEnum, DataAvailability
)
from .constants import PLATFORM_NAMES, STATE_INFO

# Environment detection
try:
    from ...core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE
except ImportError:
    ROS_AVAILABLE = False
    TF2_AVAILABLE = False

try:
    from acados_template import AcadosOcp
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False


class StatusBuilder:
    def __init__(self, config: Dict[str, Any], manager=None):
        self.config = config or {}
        self.manager = manager

    def build_availability(self, diagnostics: Dict[str, Any]) -> DataAvailability:
        """Build data availability status"""
        has_diag = bool(diagnostics)
        
        # Check data existence
        mpc_health = diagnostics.get('mpc_health', {})
        consistency = diagnostics.get('consistency', {})
        tracking = diagnostics.get('tracking', {})
        estimator = diagnostics.get('estimator_health', {})
        transform = diagnostics.get('transform', {})
        timeout = diagnostics.get('timeout', {})
        
        # Check trajectory availability
        traj_timeout = timeout.get('traj_timeout', True) if isinstance(timeout, dict) else True
        in_startup_grace = timeout.get('in_startup_grace', False) if isinstance(timeout, dict) else False
        has_trajectory = has_diag and (not traj_timeout or in_startup_grace)
        
        # Check position data
        has_position = False
        if self.manager and self.manager.state_estimator:
            state = self.manager.state_estimator.get_state()
            if state and hasattr(state, 'state'):
                has_position = True
        
        return DataAvailability(
            diagnostics_available=has_diag,
            trajectory_available=has_trajectory,
            position_available=has_position,
            odom_available=has_diag and not timeout.get('odom_timeout', True),
            imu_data_available=has_diag and estimator.get('imu_available', False),
            mpc_data_available=has_diag and isinstance(mpc_health, dict) and bool(mpc_health),
            consistency_data_available=has_diag and isinstance(consistency, dict) and bool(consistency),
            tracking_data_available=has_diag and isinstance(tracking, dict) and bool(tracking),
            estimator_data_available=has_diag and isinstance(estimator, dict) and bool(estimator),
            transform_data_available=has_diag and isinstance(transform, dict) and bool(transform),
            last_update_time=time.time(),
            data_age_ms=0.0,
        )

    def build_environment_status(self) -> EnvironmentStatus:
        """Build environment status"""
        return EnvironmentStatus(
            ros_available=ROS_AVAILABLE,
            tf2_available=TF2_AVAILABLE,
            acados_available=ACADOS_AVAILABLE,
            imu_available=True,
            is_mock_mode=False,  # This data source does not support mock mode
        )

    def build_platform_config(self) -> PlatformConfig:
        """Build platform configuration"""
        platform = self.config.get('system', {}).get('platform', 'differential')
        return PlatformConfig(
            platform=platform,
            platform_display=PLATFORM_NAMES.get(platform, platform),
            ctrl_freq=self.config.get('system', {}).get('ctrl_freq', 50),
            mpc_horizon=self.config.get('mpc', {}).get('horizon', 20),
            mpc_horizon_degraded=self.config.get('mpc', {}).get('horizon_degraded', 10),
            mpc_dt=self.config.get('mpc', {}).get('dt', 0.1),
        )

    def build_controller_status(self, diag: Dict) -> ControllerStatus:
        """Build controller status"""
        state = diag.get('state', 0)
        state_name, state_desc = STATE_INFO.get(state, ('UNKNOWN', '未知'))
        
        consistency = diag.get('consistency', {})
        alpha = consistency.get('alpha_soft', 0) if isinstance(consistency, dict) else 0

        return ControllerStatus(
            state=ControllerStateEnum(state) if 0 <= state <= 6 else ControllerStateEnum.INIT,
            state_name=state_name,
            state_desc=state_desc,
            mpc_success=diag.get('mpc_success', False),
            backup_active=diag.get('backup_active', False),
            current_controller='Backup' if diag.get('backup_active', False) else 'MPC',
            soft_head_enabled=alpha > 0.1,
            alpha_soft=alpha,
        )

    def build_mpc_health(self, diag: Dict) -> MPCHealthStatus:
        """Build MPC health status"""
        health = diag.get('mpc_health', {})
        if not isinstance(health, dict):
            health = {}

        mpc_dt = self.config.get('mpc', {}).get('dt', 0.1)
        ctrl_freq = self.config.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / ctrl_freq if ctrl_freq > 0 else 0.02
        
        dt_mismatch = abs(mpc_dt - control_period) / control_period > 0.1 if control_period > 0 else False

        consecutive_good = 0
        if self.manager and hasattr(self.manager, 'mpc_health_monitor'):
            monitor = self.manager.mpc_health_monitor
            if monitor and hasattr(monitor, 'consecutive_good'):
                consecutive_good = monitor.consecutive_good

        return MPCHealthStatus(
            kkt_residual=health.get('kkt_residual', 0),
            condition_number=health.get('condition_number', 0),
            solve_time_ms=diag.get('mpc_solve_time_ms', 0),
            consecutive_near_timeout=health.get('consecutive_near_timeout', 0),
            degradation_warning=health.get('degradation_warning', False),
            can_recover=health.get('can_recover', True),
            healthy=diag.get('mpc_success', False) and not health.get('degradation_warning', False),
            dt_mismatch=dt_mismatch,
            mpc_dt=mpc_dt,
            control_period=control_period,
            consecutive_good=consecutive_good,
        )

    def build_consistency(self, diag: Dict) -> ConsistencyStatus:
        """Build consistency status"""
        cons = diag.get('consistency', {})
        if not isinstance(cons, dict):
            cons = {}

        return ConsistencyStatus(
            curvature=cons.get('curvature', 0),
            velocity_dir=cons.get('velocity_dir', 0),
            temporal=cons.get('temporal', 0),
            alpha_soft=cons.get('alpha_soft', 0),
            data_valid=cons.get('data_valid', True),
        )

    def build_timeout_status(self, diag: Dict) -> TimeoutStatus:
        """Build timeout status"""
        timeout = diag.get('timeout', {})
        if not isinstance(timeout, dict):
            timeout = {}

        return TimeoutStatus(
            odom_timeout=timeout.get('odom_timeout', False),
            traj_timeout=timeout.get('traj_timeout', False),
            traj_grace_exceeded=timeout.get('traj_grace_exceeded', False),
            imu_timeout=timeout.get('imu_timeout', False),
            last_odom_age_ms=timeout.get('last_odom_age_ms', 0),
            last_traj_age_ms=timeout.get('last_traj_age_ms', 0),
            last_imu_age_ms=timeout.get('last_imu_age_ms', 0),
            in_startup_grace=timeout.get('in_startup_grace', False),
        )

    def build_tracking_status(self, diag: Dict) -> TrackingStatus:
        """Build tracking status"""
        tracking = diag.get('tracking', {})
        if not isinstance(tracking, dict):
            tracking = {}

        prediction_error = tracking.get('prediction_error', float('nan'))
        
        return TrackingStatus(
            lateral_error=tracking.get('lateral_error', 0),
            longitudinal_error=tracking.get('longitudinal_error', 0),
            heading_error=tracking.get('heading_error', 0),
            prediction_error=prediction_error,
        )

    def build_estimator_status(self, diag: Dict) -> EstimatorStatus:
        """Build estimator status"""
        est = diag.get('estimator_health', {})
        if not isinstance(est, dict):
            est = {}

        bias = est.get('imu_bias', [0, 0, 0])
        if not isinstance(bias, (list, tuple)) or len(bias) < 3:
            bias = [0, 0, 0]

        ekf_config = self.config.get('ekf', {})
        transform_config = self.config.get('transform', {})
        adaptive_config = ekf_config.get('adaptive', {})
        
        ekf_enabled = True
        slip_detection_enabled = adaptive_config.get('base_slip_thresh', 0) > 0
        drift_correction_enabled = transform_config.get('recovery_correction_enabled', True)
        heading_fallback_enabled = ekf_config.get('use_odom_orientation_fallback', True)

        innovation_norm = est.get('innovation_norm', 0)
        covariance_norm = est.get('covariance_norm', 0)
        slip_probability = est.get('slip_probability', 0)
        
        innovation_warning = innovation_norm > 0.5
        covariance_warning = covariance_norm > 1.0
        high_slip_rate = 100.0 if slip_probability > 0.3 else 0.0

        return EstimatorStatus(
            covariance_norm=covariance_norm,
            innovation_norm=innovation_norm,
            slip_probability=slip_probability,
            imu_drift_detected=est.get('imu_drift_detected', False),
            imu_bias=(bias[0], bias[1], bias[2]),
            imu_available=est.get('imu_available', False),
            ekf_enabled=ekf_enabled,
            slip_detection_enabled=slip_detection_enabled,
            drift_correction_enabled=drift_correction_enabled,
            heading_fallback_enabled=heading_fallback_enabled,
            imu_drift_rate=0.0,
            high_slip_rate=high_slip_rate,
            innovation_warning=innovation_warning,
            covariance_warning=covariance_warning,
        )

    def build_transform_status(self, diag: Dict) -> TransformStatus:
        """Build transform status"""
        transform = diag.get('transform', {})
        if not isinstance(transform, dict):
            transform = {}

        fallback_ms = transform.get('fallback_duration_ms', 0)
        tf2_available = transform.get('tf2_available', TF2_AVAILABLE)
        fallback_active = transform.get('fallback_active')
        if fallback_active is None:
            fallback_active = fallback_ms > 0

        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}
        output_frame = (
            cmd.get('frame_id') or
            self.config.get('system', {}).get('output_frame') or
            self.config.get('transform', {}).get('output_frame') or
            'base_link'
        )
        target_frame = transform.get('target_frame') or self.config.get('transform', {}).get('target_frame', 'odom')

        return TransformStatus(
            tf2_available=tf2_available,
            fallback_active=fallback_active,
            fallback_duration_ms=fallback_ms,
            accumulated_drift=transform.get('accumulated_drift', 0),
            target_frame=target_frame,
            output_frame=output_frame,
        )

    def build_safety_status(self, diag: Dict) -> SafetyStatus:
        """Build safety status"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}

        vx = cmd.get('vx', 0)
        vy = cmd.get('vy', 0)
        current_v = math.sqrt(vx ** 2 + vy ** 2)

        return SafetyStatus(
            v_max=self.config.get('constraints', {}).get('v_max', 2.0),
            omega_max=self.config.get('constraints', {}).get('omega_max', 2.0),
            a_max=self.config.get('constraints', {}).get('a_max', 1.5),
            current_v=current_v,
            current_omega=abs(cmd.get('omega', 0)),
            low_speed_protection_active=current_v < 0.1,
            safety_check_passed=diag.get('safety_check_passed', True),
            emergency_stop=diag.get('emergency_stop', False),
        )

    def build_control_command(self, diag: Dict) -> ControlCommand:
        """Build control command"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}

        return ControlCommand(
            vx=cmd.get('vx', 0),
            vy=cmd.get('vy', 0),
            vz=cmd.get('vz', 0),
            omega=cmd.get('omega', 0),
            frame_id=cmd.get('frame_id', 'base_link'),
        )

    def build_trajectory_data(self) -> TrajectoryData:
        """Build trajectory data"""
        if self.manager:
            return self._get_trajectory_from_manager()
        return TrajectoryData()

    def _get_trajectory_from_manager(self) -> TrajectoryData:
        """Get trajectory data from manager state (limited)"""
        data = TrajectoryData()
        if self.manager and self.manager.state_estimator:
            state_output = self.manager.state_estimator.get_state()
            if state_output and hasattr(state_output, 'state'):
                state = state_output.state
                data.current_position = (state[0], state[1], state[2])
                data.current_heading = state[6]
                data.current_velocity = (state[3], state[4])
        return data
