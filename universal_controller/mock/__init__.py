"""
模拟数据模块 (已弃用)

⚠️ 此模块已弃用，将在未来版本中移除。

请使用以下替代:

ROS 兼容层 (独立运行模式):
============================
用于非 ROS 环境下的独立运行，这是生产代码的一部分。

    from universal_controller.compat import (
        StandaloneRospy,      # rospy 替代
        StandaloneTF2Buffer,  # tf2_ros.Buffer 替代
    )

数据类型:
============================
所有数据类型已移至 core.data_types:

    from universal_controller.core.data_types import (
        Pose, Twist, PoseWithCovariance, TwistWithCovariance,
        Vector3, Quaternion, Transform, TransformStamped,
        Odometry, Imu, Header, Point3D,
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
    from universal_controller.mock import MockRospy, MockPose

新代码 (生产):
    from universal_controller.compat import StandaloneRospy
    from universal_controller.core.data_types import Pose

新代码 (测试):
    from universal_controller.tests.fixtures import create_test_trajectory
"""
import warnings

# 发出弃用警告
warnings.warn(
    "universal_controller.mock 模块已弃用，将在未来版本中移除。"
    "请使用 universal_controller.compat (ROS 兼容层) 和 "
    "universal_controller.core.data_types (数据类型)。"
    "测试数据生成器请从 universal_controller.tests.fixtures 导入。",
    DeprecationWarning,
    stacklevel=2
)

# =============================================================================
# 从 compat 模块导入 ROS 兼容层
# =============================================================================

from ..compat.ros_compat_impl import (
    # 推荐名称
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    # 向后兼容别名 (已弃用)
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
    # 异常类型
    LookupException,
    ExtrapolationException,
    ConnectivityException,
)

# =============================================================================
# 从 core.data_types 导入数据类型 (统一使用生产数据类型)
# =============================================================================

from ..core.data_types import (
    # 基础类型
    Vector3,
    Quaternion,
    Transform,
    Header,
    TransformStamped,
    Point3D,
    Odometry,
    Imu,
    # ROS geometry_msgs 兼容类型
    Pose,
    Twist,
    PoseWithCovariance,
    TwistWithCovariance,
)

# =============================================================================
# 向后兼容别名 (已弃用，请使用 core.data_types 中的类型)
# =============================================================================

# 数据类型别名
MockVector3 = Vector3
MockQuaternion = Quaternion
MockTransform = Transform
MockHeader = Header
MockTransformStamped = TransformStamped
MockPoint = Point3D
MockOdometry = Odometry
MockImu = Imu
MockPose = Pose
MockTwist = Twist
MockPoseWithCovariance = PoseWithCovariance
MockTwistWithCovariance = TwistWithCovariance


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
    # ROS 兼容实现 (向后兼容别名，已弃用)
    'MockRospy',
    'MockTF2BufferCore',
    'MockTF2Ros',
    'MockTFTransformations',
    # 异常类型
    'LookupException',
    'ExtrapolationException',
    'ConnectivityException',
    # 数据类型 (推荐直接从 core.data_types 导入)
    'Vector3',
    'Quaternion',
    'Transform',
    'Header',
    'TransformStamped',
    'Point3D',
    'Odometry',
    'Imu',
    'Pose',
    'Twist',
    'PoseWithCovariance',
    'TwistWithCovariance',
    # 数据类型别名 (已弃用)
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
