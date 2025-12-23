"""
测试数据生成器 (向后兼容模块)

此模块已移动到 tests/fixtures/test_data_generator.py
为保持向后兼容，此文件重新导出所有符号。

推荐使用:
    from universal_controller.tests.fixtures import (
        create_test_trajectory,
        create_test_odom,
        create_test_imu,
        create_test_state_sequence,
        create_local_trajectory_with_transform,
    )
"""
import warnings

# 发出弃用警告
warnings.warn(
    "从 universal_controller.tests.test_data_generator 导入已弃用。"
    "请使用 universal_controller.tests.fixtures.test_data_generator 或 "
    "universal_controller.tests.fixtures 代替。",
    DeprecationWarning,
    stacklevel=2
)

# 重新导出所有符号以保持向后兼容
from .fixtures.test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu,
    create_test_state_sequence,
    create_local_trajectory_with_transform,
)

__all__ = [
    'create_test_trajectory',
    'create_test_odom',
    'create_test_imu',
    'create_test_state_sequence',
    'create_local_trajectory_with_transform',
]
