"""
测试模块

此模块仅用于测试，不应在生产代码中使用。

包含:
- fixtures/: 测试夹具和数据生成器
- test_*.py: 各种测试文件
- run_*.py: 测试运行脚本

使用示例:
    from universal_controller.tests.fixtures import (
        # 测试数据生成
        create_test_trajectory,
        create_test_odom,
        create_test_imu,
        # 模拟诊断数据 (仅用于 Dashboard 测试)
        generate_mock_diagnostics,
    )

注意:
=====
- 生产代码不应导入此模块
- ROS 兼容层 (独立运行模式) 请使用 universal_controller.compat
- 此模块中的数据生成器仅用于测试目的
"""
