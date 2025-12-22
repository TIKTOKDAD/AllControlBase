"""
ROS 消息定义模块

本模块提供控制器使用的消息类型定义。

在 ROS 环境下，这些定义应该通过 .msg 文件生成。
在非 ROS 环境下，使用 Python dataclass 作为替代。

消息类型:
- DiagnosticsV2: 诊断消息
- LocalTrajectoryV4: 轨迹消息
- UnifiedCmd: 统一控制命令
"""

from ..core.data_types import DiagnosticsV2, Trajectory, ControlOutput

__all__ = ['DiagnosticsV2', 'Trajectory', 'ControlOutput']
