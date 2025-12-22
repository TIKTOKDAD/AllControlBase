"""诊断输入数据类型

提供强类型的诊断数据结构，替代 Dict[str, Any]，提高类型安全性。
"""
from dataclasses import dataclass, field
from typing import Optional

from .data_types import MPCHealthStatus
from .enums import ControllerState


@dataclass
class DiagnosticsInput:
    """
    诊断输入数据类
    
    用于在模块间传递诊断信息，替代 Dict[str, Any]。
    提供类型安全和 IDE 自动补全支持。
    
    Attributes:
        alpha: 一致性 alpha 值 (0.0-1.0)
        data_valid: 一致性数据是否有效
        mpc_health: MPC 健康状态
        mpc_success: MPC 求解是否成功
        odom_timeout: 里程计是否超时
        traj_timeout_exceeded: 轨迹超时宽限期是否已过
        v_horizontal: 水平速度 (m/s)
        vz: 垂直速度 (m/s)
        has_valid_data: 是否有有效轨迹数据
        tf2_critical: TF2 是否处于临界状态
        safety_failed: 安全检查是否失败
        current_state: 当前控制器状态
    """
    # 一致性相关
    alpha: float = 1.0
    data_valid: bool = True
    
    # MPC 相关
    mpc_health: Optional[MPCHealthStatus] = None
    mpc_success: bool = False
    
    # 超时相关
    odom_timeout: bool = False
    traj_timeout_exceeded: bool = False
    
    # 状态相关
    v_horizontal: float = 0.0
    vz: float = 0.0
    has_valid_data: bool = False
    
    # 变换相关
    tf2_critical: bool = False
    
    # 安全相关
    safety_failed: bool = False
    current_state: ControllerState = ControllerState.INIT
    
    def to_dict(self) -> dict:
        """
        转换为字典格式
        
        用于向后兼容和日志记录
        """
        return {
            'alpha': self.alpha,
            'data_valid': self.data_valid,
            'mpc_health': self.mpc_health,
            'mpc_success': self.mpc_success,
            'odom_timeout': self.odom_timeout,
            'traj_timeout_exceeded': self.traj_timeout_exceeded,
            'v_horizontal': self.v_horizontal,
            'vz': self.vz,
            'has_valid_data': self.has_valid_data,
            'tf2_critical': self.tf2_critical,
            'safety_failed': self.safety_failed,
            'current_state': self.current_state,
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'DiagnosticsInput':
        """
        从字典创建实例
        
        用于向后兼容
        """
        return cls(
            alpha=data.get('alpha', 1.0),
            data_valid=data.get('data_valid', True),
            mpc_health=data.get('mpc_health'),
            mpc_success=data.get('mpc_success', False),
            odom_timeout=data.get('odom_timeout', False),
            traj_timeout_exceeded=data.get('traj_timeout_exceeded', False),
            v_horizontal=data.get('v_horizontal', 0.0),
            vz=data.get('vz', 0.0),
            has_valid_data=data.get('has_valid_data', False),
            tf2_critical=data.get('tf2_critical', False),
            safety_failed=data.get('safety_failed', False),
            current_state=data.get('current_state', ControllerState.INIT),
        )
