"""
模拟数据模块

用于测试和非 ROS 环境下的模拟实现。

包含:
- ros_mock: ROS 模拟类 (MockRospy, MockTF2BufferCore 等)
- data_mock: 模拟数据类型 (MockVector3, MockQuaternion 等)
- diagnostics_mock: 模拟诊断数据生成
- test_data_generator: 测试数据生成器 (轨迹、里程计等)
"""

from .ros_mock import (
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
    LookupException,
    ExtrapolationException,
    ConnectivityException,
)

from .data_mock import (
    MockVector3,
    MockQuaternion,
    MockTransform,
    MockHeader,
    MockTransformStamped,
    MockPoint,
    MockPose,
    MockTwist,
    MockPoseWithCovariance,
    MockTwistWithCovariance,
    MockOdometry,
    MockImu,
)

from .diagnostics_mock import (
    generate_mock_diagnostics,
    generate_mock_diagnostics_degraded,
    generate_mock_diagnostics_timeout,
)

from .test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu,
    create_test_state_sequence,
)

__all__ = [
    # ROS Mock
    'MockRospy',
    'MockTF2BufferCore',
    'MockTF2Ros',
    'MockTFTransformations',
    'LookupException',
    'ExtrapolationException',
    'ConnectivityException',
    # Data Mock
    'MockVector3',
    'MockQuaternion',
    'MockTransform',
    'MockHeader',
    'MockTransformStamped',
    'MockPoint',
    'MockPose',
    'MockTwist',
    'MockPoseWithCovariance',
    'MockTwistWithCovariance',
    'MockOdometry',
    'MockImu',
    # Diagnostics Mock
    'generate_mock_diagnostics',
    'generate_mock_diagnostics_degraded',
    'generate_mock_diagnostics_timeout',
    # Test Data Generator
    'create_test_trajectory',
    'create_test_odom',
    'create_test_imu',
    'create_test_state_sequence',
]
