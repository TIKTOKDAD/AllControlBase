"""
测试 fixtures 模块

提供所有测试共享的 Mock 类和工具函数。

Mock 类分类:
- ROS 消息 Mock: 模拟 ROS 消息类型 (Odometry, Imu, Trajectory 等)
- ROS 基础类型 Mock: 模拟 ROS 基础类型 (Time, Header, Point 等)
- 桥接层 Mock: 模拟 TF Bridge 等

使用方法:
    from controller_ros.test.fixtures import (
        MockRosOdometry, MockRosImu, MockRosTrajectory,
        MockTFBridge, create_mock_odom, create_mock_trajectory
    )
"""
from .ros_message_mocks import (
    # ROS 基础类型
    MockRosTime,
    MockRosHeader,
    MockRosPoint,
    MockRosQuaternion,
    MockRosVector3,
    MockRosPose,
    MockRosTwist,
    MockRosPoseWithCovariance,
    MockRosTwistWithCovariance,
    # ROS 消息类型
    MockRosOdometry,
    MockRosImu,
    MockRosTrajectory,
    # 工厂函数
    create_mock_odom,
    create_mock_imu,
    create_mock_trajectory,
)

from .bridge_mocks import (
    MockTFBridge,
    MockControllerBridge,
)

__all__ = [
    # ROS 基础类型
    'MockRosTime',
    'MockRosHeader',
    'MockRosPoint',
    'MockRosQuaternion',
    'MockRosVector3',
    'MockRosPose',
    'MockRosTwist',
    'MockRosPoseWithCovariance',
    'MockRosTwistWithCovariance',
    # ROS 消息类型
    'MockRosOdometry',
    'MockRosImu',
    'MockRosTrajectory',
    # 工厂函数
    'create_mock_odom',
    'create_mock_imu',
    'create_mock_trajectory',
    # 桥接层 Mock
    'MockTFBridge',
    'MockControllerBridge',
]
