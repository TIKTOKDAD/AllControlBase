"""
模拟诊断数据生成

用于 Dashboard 独立运行时生成模拟的诊断数据。
"""
import time
import random
import math
from typing import Dict, Any


def generate_mock_diagnostics(start_time: float = None) -> Dict[str, Any]:
    """
    生成模拟诊断数据
    
    Args:
        start_time: 起始时间戳，用于计算运行时间。如果为 None，使用当前时间。
    
    Returns:
        模拟的诊断数据字典，格式与 DiagnosticsV2 一致
    """
    if start_time is None:
        start_time = time.time() - 60  # 默认假设运行了 60 秒
    
    t = time.time() - start_time
    
    return {
        # 基本状态
        'state': 1,  # NORMAL
        'mpc_success': True,
        'mpc_solve_time_ms': 6 + random.random() * 4,
        'backup_active': False,
        
        # MPC 健康状态
        'mpc_health': {
            'kkt_residual': 0.0008 + random.random() * 0.0005,
            'condition_number': 1e5 + random.random() * 1e4,
            'consecutive_near_timeout': 0,
            'degradation_warning': False,
            'can_recover': True,
        },
        
        # 一致性分析
        'consistency': {
            'curvature': 0.7 + random.random() * 0.2,
            'velocity_dir': 0.85 + random.random() * 0.1,
            'temporal': 0.8 + random.random() * 0.15,
            'alpha_soft': 0.75 + random.random() * 0.2,
            'data_valid': True,
        },
        
        # 状态估计器健康
        'estimator_health': {
            'covariance_norm': 0.01 + random.random() * 0.005,
            'innovation_norm': 0.008 + random.random() * 0.004,
            'slip_probability': random.random() * 0.05,
            'imu_drift_detected': False,
            'imu_bias': [0.02, 0.01, 0.03],
            'imu_available': True,
        },
        
        # 跟踪误差
        'tracking': {
            'lateral_error': 0.05 + math.sin(t * 0.5) * 0.03,
            'longitudinal_error': 0.1 + math.sin(t * 0.3) * 0.05,
            'heading_error': 0.03 + random.random() * 0.02,
            'prediction_error': 0.08 + random.random() * 0.04,
        },
        
        # 坐标变换状态
        'transform': {
            'tf2_available': True,
            'fallback_duration_ms': 0,
            'accumulated_drift': 0,
        },
        
        # 超时状态
        'timeout': {
            'odom_timeout': False,
            'traj_timeout': False,
            'traj_grace_exceeded': False,
            'imu_timeout': False,
            'last_odom_age_ms': 10 + random.random() * 5,
            'last_traj_age_ms': 40 + random.random() * 10,
            'last_imu_age_ms': 5 + random.random() * 5,
            'in_startup_grace': False,
        },
        
        # 控制命令
        'cmd': {
            'vx': 1.2 + math.sin(t * 0.2) * 0.3,
            'vy': 0.0,
            'vz': 0.0,
            'omega': 0.3 + math.sin(t * 0.4) * 0.2,
            'frame_id': 'base_link',
        },
        
        # 过渡进度
        'transition_progress': 1.0,
    }


def generate_mock_diagnostics_degraded(start_time: float = None, 
                                       degradation_level: int = 3) -> Dict[str, Any]:
    """
    生成降级状态的模拟诊断数据
    
    Args:
        start_time: 起始时间戳
        degradation_level: 降级级别 (0-6)
    
    Returns:
        模拟的诊断数据字典
    """
    data = generate_mock_diagnostics(start_time)
    
    # 根据降级级别调整数据
    data['state'] = degradation_level
    
    if degradation_level >= 2:  # SOFT_DISABLED
        data['consistency']['alpha_soft'] = 0.05 + random.random() * 0.05
    
    if degradation_level >= 3:  # MPC_DEGRADED
        data['mpc_health']['degradation_warning'] = True
        data['mpc_health']['kkt_residual'] = 0.005 + random.random() * 0.005
        data['mpc_solve_time_ms'] = 12 + random.random() * 5
    
    if degradation_level >= 4:  # BACKUP_ACTIVE
        data['backup_active'] = True
        data['mpc_success'] = False
    
    if degradation_level >= 5:  # STOPPING
        data['cmd']['vx'] = max(0, data['cmd']['vx'] - 0.5)
        data['cmd']['omega'] = 0.0
    
    if degradation_level >= 6:  # STOPPED
        data['cmd']['vx'] = 0.0
        data['cmd']['vy'] = 0.0
        data['cmd']['vz'] = 0.0
        data['cmd']['omega'] = 0.0
    
    return data


def generate_mock_diagnostics_timeout(start_time: float = None,
                                      timeout_type: str = 'odom') -> Dict[str, Any]:
    """
    生成超时状态的模拟诊断数据
    
    Args:
        start_time: 起始时间戳
        timeout_type: 超时类型 ('odom', 'traj', 'imu')
    
    Returns:
        模拟的诊断数据字典
    """
    data = generate_mock_diagnostics(start_time)
    
    if timeout_type == 'odom':
        data['timeout']['odom_timeout'] = True
        data['timeout']['last_odom_age_ms'] = 250 + random.random() * 50
        data['state'] = 5  # STOPPING
    elif timeout_type == 'traj':
        data['timeout']['traj_timeout'] = True
        data['timeout']['traj_grace_exceeded'] = True
        data['timeout']['last_traj_age_ms'] = 300 + random.random() * 100
        data['state'] = 5  # STOPPING
    elif timeout_type == 'imu':
        data['timeout']['imu_timeout'] = True
        data['timeout']['last_imu_age_ms'] = 150 + random.random() * 50
        data['estimator_health']['imu_available'] = False
    
    return data
