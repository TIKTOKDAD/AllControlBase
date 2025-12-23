"""
ROS 兼容层模块

用于非 ROS 环境下的兼容实现，使 universal_controller 可以独立运行。

重要说明:
=========
此模块是**生产代码**的一部分，不是测试 mock。

包含:
- ros_compat_impl: ROS 替代实现 (StandaloneRospy, StandaloneTF2Buffer 等)

命名约定:
=========
- "Standalone" 前缀表示独立运行模式的实现
- 保留 "Mock" 前缀别名以保持向后兼容

注意:
=========
- 模拟诊断数据生成器已移动到 tests/fixtures/mock_diagnostics.py
- 测试数据生成器位于 tests/fixtures/test_data_generator.py
"""

from .ros_compat_impl import (
    # 推荐使用的名称
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    # 异常类型
    LookupException,
    ExtrapolationException,
    ConnectivityException,
    # 向后兼容别名
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
)

# 从 core.data_types 导入数据类型 (统一使用生产数据类型)
from ..core.data_types import (
    Vector3,
    Quaternion,
    Transform,
    TransformStamped,
    Header,
    Point3D,
    Odometry,
    Imu,
)

__all__ = [
    # 推荐使用的名称
    'StandaloneRospy',
    'StandaloneTF2Buffer',
    'StandaloneTF2Ros',
    'StandaloneTFTransformations',
    # 异常类型
    'LookupException',
    'ExtrapolationException',
    'ConnectivityException',
    # 向后兼容别名
    'MockRospy',
    'MockTF2BufferCore',
    'MockTF2Ros',
    'MockTFTransformations',
    # 数据类型 (来自 core.data_types)
    'Vector3',
    'Quaternion',
    'Transform',
    'TransformStamped',
    'Header',
    'Point3D',
    'Odometry',
    'Imu',
]
