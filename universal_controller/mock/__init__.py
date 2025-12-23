"""
模拟数据模块 (已弃用)

此模块已重构，请使用以下替代:

ROS 兼容层 (独立运行模式):
============================
用于非 ROS 环境下的独立运行，这是生产代码的一部分。

    from universal_controller.compat import (
        StandaloneRospy,      # rospy 替代
        StandaloneTF2Buffer,  # tf2_ros.Buffer 替代
        # 或使用向后兼容别名
        MockRospy,
        MockTF2BufferCore,
    )

测试数据生成 (仅用于测试):
============================
用于单元测试和集成测试，不应在生产代码中使用。

    from universal_controller.tests.fixtures import (
        create_test_trajectory,
        create_test_odom,
        generate_mock_diagnostics,
    )

迁移指南:
=========
旧代码:
    from universal_controller.mock import MockRospy, create_test_trajectory

新代码 (生产):
    from universal_controller.compat import StandaloneRospy  # 或 MockRospy

新代码 (测试):
    from universal_controller.tests.fixtures import create_test_trajectory

警告:
=====
此模块将在未来版本中移除。
为保持向后兼容，此模块仍然导出 ROS 兼容层符号。
测试数据生成器不再从此模块导出，请直接从 tests.fixtures 导入。
"""
import warnings

# 发出弃用警告
warnings.warn(
    "universal_controller.mock 模块已弃用。"
    "请使用 universal_controller.compat (ROS 兼容层)。"
    "测试数据生成器请从 universal_controller.tests.fixtures 导入。",
    DeprecationWarning,
    stacklevel=2
)

# =============================================================================
# 仅导出 ROS 兼容层 (生产代码的一部分)
# =============================================================================

# 从 compat 模块导入 ROS 兼容层
from ..compat.ros_compat_impl import (
    # 推荐名称
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    # 向后兼容别名
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
    # 异常类型
    LookupException,
    ExtrapolationException,
    ConnectivityException,
)

# 从 core.data_types 导入数据类型
from ..core.data_types import (
    Vector3 as MockVector3,
    Quaternion as MockQuaternion,
    Transform as MockTransform,
    Header as MockHeader,
    TransformStamped as MockTransformStamped,
    Point3D as MockPoint,
    Odometry as MockOdometry,
    Imu as MockImu,
)

# 为了兼容性，创建 MockPose, MockTwist 等别名
from dataclasses import dataclass, field
from ..core.data_types import Point3D, Quaternion, Vector3


@dataclass
class MockPose:
    """模拟 geometry_msgs/Pose (向后兼容)"""
    position: Point3D = field(default_factory=lambda: Point3D(0, 0, 0))
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class MockTwist:
    """模拟 geometry_msgs/Twist (向后兼容)"""
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class MockPoseWithCovariance:
    """模拟 geometry_msgs/PoseWithCovariance (向后兼容)"""
    pose: MockPose = field(default_factory=MockPose)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class MockTwistWithCovariance:
    """模拟 geometry_msgs/TwistWithCovariance (向后兼容)"""
    twist: MockTwist = field(default_factory=MockTwist)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


# =============================================================================
# 不再导出测试数据生成器
# =============================================================================
# 以下函数已移除，请从 universal_controller.tests.fixtures 导入:
# - create_test_trajectory
# - create_test_odom
# - create_test_imu
# - create_test_state_sequence
# - create_local_trajectory_with_transform
# - generate_mock_diagnostics
# - generate_mock_diagnostics_degraded
# - generate_mock_diagnostics_timeout


__all__ = [
    # ROS 兼容实现 (推荐名称)
    'StandaloneRospy',
    'StandaloneTF2Buffer',
    'StandaloneTF2Ros',
    'StandaloneTFTransformations',
    # ROS 兼容实现 (向后兼容别名)
    'MockRospy',
    'MockTF2BufferCore',
    'MockTF2Ros',
    'MockTFTransformations',
    # 异常类型
    'LookupException',
    'ExtrapolationException',
    'ConnectivityException',
    # 数据类型别名
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
]
