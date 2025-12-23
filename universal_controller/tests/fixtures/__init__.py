"""
测试夹具模块

包含测试数据生成器和模拟数据生成器。

此模块仅用于测试，不应在生产代码中使用。

包含:
- test_data_generator: 测试数据生成 (轨迹、里程计、IMU)
- mock_diagnostics: 模拟诊断数据生成 (用于 Dashboard 测试)
"""

from .test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu,
    create_test_state_sequence,
    create_local_trajectory_with_transform,
)

from .mock_diagnostics import (
    generate_mock_diagnostics,
    generate_mock_diagnostics_degraded,
    generate_mock_diagnostics_timeout,
)

__all__ = [
    # 测试数据生成
    'create_test_trajectory',
    'create_test_odom',
    'create_test_imu',
    'create_test_state_sequence',
    'create_local_trajectory_with_transform',
    # 模拟诊断数据
    'generate_mock_diagnostics',
    'generate_mock_diagnostics_degraded',
    'generate_mock_diagnostics_timeout',
]
