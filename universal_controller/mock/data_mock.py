"""
模拟数据类型

模拟 ROS geometry_msgs, std_msgs 等消息类型。
"""
from dataclasses import dataclass, field
from typing import Any
import time


@dataclass
class MockVector3:
    """模拟 geometry_msgs/Vector3"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockQuaternion:
    """模拟 geometry_msgs/Quaternion"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class MockTransform:
    """模拟 geometry_msgs/Transform"""
    translation: MockVector3 = field(default_factory=MockVector3)
    rotation: MockQuaternion = field(default_factory=MockQuaternion)


@dataclass
class MockHeader:
    """模拟 std_msgs/Header"""
    stamp: Any = None
    frame_id: str = ""
    seq: int = 0
    
    def __post_init__(self):
        if self.stamp is None:
            # 延迟导入避免循环依赖
            from .ros_mock import MockRospy
            self.stamp = MockRospy.Time.now()


@dataclass
class MockTransformStamped:
    """模拟 geometry_msgs/TransformStamped"""
    header: MockHeader = field(default_factory=MockHeader)
    child_frame_id: str = ""
    transform: MockTransform = field(default_factory=MockTransform)


@dataclass
class MockPoint:
    """模拟 geometry_msgs/Point"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class MockPose:
    """模拟 geometry_msgs/Pose"""
    position: MockPoint = field(default_factory=MockPoint)
    orientation: MockQuaternion = field(default_factory=MockQuaternion)


@dataclass
class MockTwist:
    """模拟 geometry_msgs/Twist"""
    linear: MockVector3 = field(default_factory=MockVector3)
    angular: MockVector3 = field(default_factory=MockVector3)


@dataclass
class MockPoseWithCovariance:
    """模拟 geometry_msgs/PoseWithCovariance"""
    pose: MockPose = field(default_factory=MockPose)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class MockTwistWithCovariance:
    """模拟 geometry_msgs/TwistWithCovariance"""
    twist: MockTwist = field(default_factory=MockTwist)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class MockOdometry:
    """模拟 nav_msgs/Odometry"""
    header: MockHeader = field(default_factory=MockHeader)
    child_frame_id: str = "base_link"
    pose: MockPoseWithCovariance = field(default_factory=MockPoseWithCovariance)
    twist: MockTwistWithCovariance = field(default_factory=MockTwistWithCovariance)


@dataclass
class MockImu:
    """模拟 sensor_msgs/Imu"""
    header: MockHeader = field(default_factory=MockHeader)
    orientation: MockQuaternion = field(default_factory=MockQuaternion)
    orientation_covariance: list = field(default_factory=lambda: [0.0] * 9)
    angular_velocity: MockVector3 = field(default_factory=MockVector3)
    angular_velocity_covariance: list = field(default_factory=lambda: [0.0] * 9)
    linear_acceleration: MockVector3 = field(default_factory=MockVector3)
    linear_acceleration_covariance: list = field(default_factory=lambda: [0.0] * 9)
