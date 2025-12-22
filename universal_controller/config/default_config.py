"""默认配置"""
from typing import Dict, Any
from ..core.enums import PlatformType


PLATFORM_CONFIG = {
    "ackermann": {
        "type": PlatformType.ACKERMANN,
        "active_dims": [0, 1, 3, 6],
        "control_dims": [3, 7],
        "constraints": {"pz": 0, "vy": 0, "vz": 0, "curvature": True},
        "velocity_heading_coupled": True,
        "output_type": "differential",
        "output_frame": "base_link"
    },
    "differential": {
        "type": PlatformType.DIFFERENTIAL,
        "active_dims": [0, 1, 3, 6, 7],
        "control_dims": [3, 7],
        "constraints": {"pz": 0, "vy": 0, "vz": 0, "curvature": False},
        "velocity_heading_coupled": True,
        "output_type": "differential",
        "output_frame": "base_link"
    },
    "omni": {
        "type": PlatformType.OMNI,
        "active_dims": [0, 1, 3, 4, 6, 7],
        "control_dims": [3, 4, 7],
        "constraints": {"pz": 0, "vz": 0},
        "velocity_heading_coupled": False,
        "output_type": "omni",
        "output_frame": "world"
    },
    "quadrotor": {
        "type": PlatformType.QUADROTOR,
        "active_dims": [0, 1, 2, 3, 4, 5, 6, 7],
        "control_dims": [3, 4, 5, 7],
        "constraints": {},
        "attitude_interface": True,
        "velocity_heading_coupled": False,
        "output_type": "3d",
        "output_frame": "world"
    }
}


DEFAULT_CONFIG = {
    'system': {
        'ctrl_freq': 50,
        'platform': 'differential',
    },
    'mpc': {
        'horizon': 20,
        'horizon_degraded': 10,
        'dt': 0.02,
        'weights': {
            'position': 10.0,
            'velocity': 1.0,
            'heading': 5.0,
            'control_v': 0.1,
            'control_omega': 0.1,
        },
        'health_monitor': {
            'time_warning_thresh_ms': 8,
            'time_critical_thresh_ms': 15,
            'time_recovery_thresh_ms': 6,
            'condition_number_thresh': 1e8,
            'condition_number_recovery': 1e5,
            'kkt_residual_thresh': 1e-3,
            'consecutive_warning_limit': 3,
            'consecutive_recovery_limit': 5,
        },
    },
    'watchdog': {
        'odom_timeout_ms': 200,
        'traj_timeout_ms': 200,
        'traj_grace_ms': 100,
        'imu_timeout_ms': 100,
        'startup_grace_ms': 1000,
    },
    'consistency': {
        'kappa_thresh': 0.5,
        'v_dir_thresh': 0.8,
        'temporal_smooth_thresh': 0.5,
        'low_speed_thresh': 0.1,
        'alpha_min': 0.1,
        'weights': {
            'kappa': 1.0,
            'velocity': 1.5,
            'temporal': 0.8,
        },
    },
    'safety': {
        'v_stop_thresh': 0.05,
        'vz_stop_thresh': 0.1,
        'stopping_timeout': 5.0,
        'emergency_decel': 3.0,
        'velocity_margin': 1.1,
        'accel_margin': 1.5,
        'state_machine': {
            'alpha_recovery_thresh': 5,
            'alpha_recovery_value': 0.3,
            'alpha_disable_thresh': 0.1,
            'mpc_recovery_thresh': 5,
            'mpc_fail_thresh': 3,  # 新增：MPC 连续失败阈值
        },
    },
    'transform': {
        'target_frame': 'odom',
        'source_frame': 'odom',  # 轨迹和状态都在 odom 坐标系，无需变换
        'fallback_duration_limit_ms': 500,
        'fallback_critical_limit_ms': 1000,
        'tf2_timeout_ms': 10,
        'drift_estimation_enabled': False,  # 无外部定位，禁用漂移估计
        'recovery_correction_enabled': False,  # 无外部定位，禁用恢复校正
        'drift_rate': 0.01,
        'tf2_required': False,  # 不强制要求 TF2
    },
    'transition': {
        'type': 'exponential',
        'tau': 0.1,
        'max_duration': 0.5,
        'completion_threshold': 0.95,
        'duration': 0.2,
    },
    'backup': {
        'lookahead_dist': 1.0,
        'min_lookahead': 0.5,
        'max_lookahead': 3.0,
        'lookahead_ratio': 0.5,
        'kp_z': 1.0,
        'kp_heading': 1.5,
        'heading_mode': 'follow_velocity',
        'dt': 0.02,
    },
    'constraints': {
        'v_max': 2.0,
        'v_min': 0.0,
        'omega_max': 2.0,
        'omega_max_low': 1.0,
        'v_low_thresh': 0.1,
        'a_max': 1.5,
        'az_max': 1.0,
        'alpha_max': 3.0,
        'vx_max': 1.5,
        'vx_min': -1.5,
        'vy_max': 1.5,
        'vy_min': -1.5,
        'vz_max': 2.0,
    },
    'ekf': {
        'use_odom_orientation_fallback': True,
        'theta_covariance_fallback_thresh': 0.5,
        'imu_motion_compensation': False,
        'adaptive': {
            'base_slip_thresh': 2.0,
            'slip_velocity_factor': 0.5,
            'slip_covariance_scale': 10.0,
            'stationary_covariance_scale': 0.1,
            'stationary_thresh': 0.05,
        },
        'measurement_noise': {
            'odom_position': 0.01,
            'odom_velocity': 0.1,
            'imu_accel': 0.5,
            'imu_gyro': 0.01,
        },
        'process_noise': {
            'position': 0.001,
            'velocity': 0.1,
            'orientation': 0.01,
            'angular_velocity': 0.1,
            'imu_bias': 0.0001,
        },
        'anomaly_detection': {
            'drift_thresh': 0.1,
            'jump_thresh': 0.5,
        },
        'covariance': {
            'min_eigenvalue': 1e-6,
        },
    },
    # F14: 无人机姿态控制配置
    'attitude': {
        'mass': 1.5,  # kg
        'gravity': 9.81,  # m/s^2
        # F14.2: 姿态角速度限制
        'roll_rate_max': 3.0,  # rad/s
        'pitch_rate_max': 3.0,  # rad/s
        'yaw_rate_max': 2.0,  # rad/s
        # 姿态角限制
        'roll_max': 0.5,  # rad (~30 deg)
        'pitch_max': 0.5,  # rad (~30 deg)
        # 速度控制增益
        'kp_vx': 0.5,
        'kp_vy': 0.5,
        'kp_vz': 1.0,
        # F14.3: 悬停 yaw 漂移补偿
        'hover_yaw_compensation': True,
        'hover_speed_thresh': 0.1,  # m/s (水平速度阈值)
        'hover_vz_thresh': 0.05,  # m/s (垂直速度阈值，更严格)
        'yaw_drift_rate': 0.001,  # rad/s
        # F14.4: 位置-姿态解耦
        'position_attitude_decoupled': False,
        # 推力限制 (归一化到悬停推力)
        'thrust_min': 0.1,  # 最小推力 (防止自由落体)
        'thrust_max': 2.0,  # 最大推力
        'dt': 0.02,
    },
}


def get_config_value(config: Dict[str, Any], key_path: str, default: Any = None) -> Any:
    """从配置字典中获取值，支持点分隔的路径"""
    keys = key_path.split('.')
    value = config
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            default_value = DEFAULT_CONFIG
            for k in keys:
                if isinstance(default_value, dict) and k in default_value:
                    default_value = default_value[k]
                else:
                    return default
            return default_value
    return value
