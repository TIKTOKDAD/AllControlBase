"""
ROS 消息 Mock 类

模拟 ROS 消息类型，用于单元测试。
支持 ROS1 和 ROS2 消息格式。

这些 Mock 类模拟的是 ROS 消息类型（如 nav_msgs/Odometry），
而不是 ROS 模块（如 rospy）。ROS 模块的模拟在 universal_controller/compat/ 中。
"""
from typing import Tuple, List, Optional


# =============================================================================
# ROS 基础类型 Mock
# =============================================================================

class MockRosTime:
    """
    模拟 ROS 时间戳
    
    支持 ROS1 (secs/nsecs) 和 ROS2 (sec/nanosec) 两种格式。
    """
    
    def __init__(self, sec: int = 0, nanosec: int = 0):
        # ROS2 风格
        self.sec = sec
        self.nanosec = nanosec
        # ROS1 风格
        self.secs = sec
        self.nsecs = nanosec
    
    def to_sec(self) -> float:
        """ROS1 风格的时间转换"""
        return self.sec + self.nanosec * 1e-9
    
    @classmethod
    def from_sec(cls, sec: float) -> 'MockRosTime':
        """从秒数创建时间戳"""
        int_sec = int(sec)
        nanosec = int((sec - int_sec) * 1e9)
        return cls(int_sec, nanosec)
    
    def __repr__(self) -> str:
        return f"MockRosTime(sec={self.sec}, nanosec={self.nanosec})"


class MockRosHeader:
    """模拟 std_msgs/Header"""
    
    def __init__(self, stamp: Optional[MockRosTime] = None, frame_id: str = ''):
        self.stamp = stamp or MockRosTime()
        self.frame_id = frame_id
    
    def __repr__(self) -> str:
        return f"MockRosHeader(frame_id='{self.frame_id}')"


class MockRosPoint:
    """模拟 geometry_msgs/Point"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self) -> str:
        return f"MockRosPoint(x={self.x}, y={self.y}, z={self.z})"


class MockRosQuaternion:
    """模拟 geometry_msgs/Quaternion"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def __repr__(self) -> str:
        return f"MockRosQuaternion(x={self.x}, y={self.y}, z={self.z}, w={self.w})"


class MockRosVector3:
    """模拟 geometry_msgs/Vector3"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self) -> str:
        return f"MockRosVector3(x={self.x}, y={self.y}, z={self.z})"


class MockRosPose:
    """模拟 geometry_msgs/Pose"""
    
    def __init__(self, 
                 position: Optional[MockRosPoint] = None,
                 orientation: Optional[MockRosQuaternion] = None):
        self.position = position or MockRosPoint()
        self.orientation = orientation or MockRosQuaternion()


class MockRosTwist:
    """模拟 geometry_msgs/Twist"""
    
    def __init__(self,
                 linear: Optional[MockRosVector3] = None,
                 angular: Optional[MockRosVector3] = None):
        self.linear = linear or MockRosVector3()
        self.angular = angular or MockRosVector3()


class MockRosPoseWithCovariance:
    """模拟 geometry_msgs/PoseWithCovariance"""
    
    def __init__(self, pose: Optional[MockRosPose] = None):
        self.pose = pose or MockRosPose()
        self.covariance = [0.0] * 36


class MockRosTwistWithCovariance:
    """模拟 geometry_msgs/TwistWithCovariance"""
    
    def __init__(self, twist: Optional[MockRosTwist] = None):
        self.twist = twist or MockRosTwist()
        self.covariance = [0.0] * 36


# =============================================================================
# ROS 消息类型 Mock
# =============================================================================

class MockRosOdometry:
    """
    模拟 nav_msgs/Odometry
    
    用于测试 OdomAdapter 的消息转换。
    """
    
    def __init__(self, 
                 x: float = 1.0, y: float = 2.0, z: float = 0.0,
                 qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                 vx: float = 0.5, vy: float = 0.0, vz: float = 0.0,
                 omega_x: float = 0.0, omega_y: float = 0.0, omega_z: float = 0.1,
                 frame_id: str = 'odom',
                 stamp: Optional[MockRosTime] = None):
        self.header = MockRosHeader(stamp or MockRosTime(1000, 500000000), frame_id)
        self.pose = MockRosPoseWithCovariance()
        self.twist = MockRosTwistWithCovariance()
        
        # 设置位置
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        
        # 设置姿态
        self.pose.pose.orientation.x = qx
        self.pose.pose.orientation.y = qy
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw
        
        # 设置线速度
        self.twist.twist.linear.x = vx
        self.twist.twist.linear.y = vy
        self.twist.twist.linear.z = vz
        
        # 设置角速度
        self.twist.twist.angular.x = omega_x
        self.twist.twist.angular.y = omega_y
        self.twist.twist.angular.z = omega_z


class MockRosImu:
    """
    模拟 sensor_msgs/Imu
    
    用于测试 ImuAdapter 的消息转换。
    """
    
    def __init__(self, 
                 orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
                 angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 linear_acceleration: Tuple[float, float, float] = (0.0, 0.0, 9.81),
                 frame_id: str = 'imu_link',
                 stamp: Optional[MockRosTime] = None):
        self.header = MockRosHeader(stamp or MockRosTime(1000, 500000000), frame_id)
        self.orientation = MockRosQuaternion(*orientation)
        self.angular_velocity = MockRosVector3(*angular_velocity)
        self.linear_acceleration = MockRosVector3(*linear_acceleration)
        
        # 协方差矩阵 (可选)
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


class MockRosTrajectory:
    """
    模拟 controller_ros/LocalTrajectoryV4
    
    用于测试 TrajectoryAdapter 的消息转换。
    """
    
    def __init__(self,
                 num_points: int = 10,
                 dt_sec: float = 0.1,
                 confidence: float = 0.9,
                 soft_enabled: bool = False,
                 mode: int = 0,
                 frame_id: str = 'base_link',
                 stamp: Optional[MockRosTime] = None):
        self.header = MockRosHeader(stamp or MockRosTime(1000, 500000000), frame_id)
        self.mode = mode
        self.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(num_points)]
        self.velocities_flat: List[float] = []
        self.dt_sec = dt_sec
        self.confidence = confidence
        self.soft_enabled = soft_enabled
    
    def set_velocities(self, velocities_flat: List[float]) -> None:
        """
        设置速度数据
        
        Args:
            velocities_flat: 扁平化的速度数组 [vx0, vy0, vz0, wz0, vx1, vy1, ...]
        """
        self.velocities_flat = velocities_flat
        if velocities_flat:
            self.soft_enabled = True
    
    def set_points(self, points: List[Tuple[float, float, float]]) -> None:
        """
        设置轨迹点
        
        Args:
            points: 点列表 [(x0, y0, z0), (x1, y1, z1), ...]
        """
        self.points = [MockRosPoint(x, y, z) for x, y, z in points]


# =============================================================================
# 工厂函数
# =============================================================================

def create_mock_odom(x: float = 0.0, y: float = 0.0, theta: float = 0.0,
                     vx: float = 0.0, vy: float = 0.0, omega: float = 0.0,
                     frame_id: str = 'odom') -> MockRosOdometry:
    """
    创建 Mock Odometry 消息
    
    Args:
        x, y: 位置 (米)
        theta: 航向角 (弧度)
        vx, vy: 线速度 (米/秒)
        omega: 角速度 (弧度/秒)
        frame_id: 坐标系
    
    Returns:
        MockRosOdometry 对象
    """
    import math
    qz = math.sin(theta / 2)
    qw = math.cos(theta / 2)
    
    return MockRosOdometry(
        x=x, y=y, z=0.0,
        qx=0.0, qy=0.0, qz=qz, qw=qw,
        vx=vx, vy=vy, vz=0.0,
        omega_z=omega,
        frame_id=frame_id
    )


def create_mock_imu(orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
                    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                    linear_acceleration: Tuple[float, float, float] = (0.0, 0.0, 9.81),
                    frame_id: str = 'imu_link') -> MockRosImu:
    """
    创建 Mock IMU 消息
    
    Args:
        orientation: 四元数 (x, y, z, w)
        angular_velocity: 角速度 (rad/s)
        linear_acceleration: 线加速度 (m/s²)
        frame_id: 坐标系
    
    Returns:
        MockRosImu 对象
    """
    return MockRosImu(
        orientation=orientation,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        frame_id=frame_id
    )


def create_mock_trajectory(num_points: int = 10,
                           dt_sec: float = 0.1,
                           trajectory_type: str = 'straight',
                           soft_enabled: bool = False,
                           frame_id: str = 'base_link') -> MockRosTrajectory:
    """
    创建 Mock Trajectory 消息
    
    Args:
        num_points: 轨迹点数量
        dt_sec: 时间间隔
        trajectory_type: 轨迹类型 ('straight', 'sine', 'circle')
        soft_enabled: 是否启用 soft 模式
        frame_id: 坐标系
    
    Returns:
        MockRosTrajectory 对象
    """
    import math
    
    traj = MockRosTrajectory(
        num_points=0,  # 先创建空轨迹
        dt_sec=dt_sec,
        soft_enabled=soft_enabled,
        frame_id=frame_id
    )
    
    points = []
    velocities = []
    
    for i in range(num_points):
        t = i * dt_sec
        
        if trajectory_type == 'straight':
            x = t * 0.5
            y = 0.0
            vx, vy = 0.5, 0.0
        elif trajectory_type == 'sine':
            x = t * 0.5
            y = 0.3 * math.sin(t * 2)
            vx = 0.5
            vy = 0.6 * math.cos(t * 2)
        elif trajectory_type == 'circle':
            radius = 1.0
            x = radius * math.cos(t)
            y = radius * math.sin(t)
            vx = -radius * math.sin(t)
            vy = radius * math.cos(t)
        else:
            x = i * 0.1
            y = 0.0
            vx, vy = 0.1, 0.0
        
        points.append((x, y, 0.0))
        if soft_enabled:
            velocities.extend([vx, vy, 0.0, 0.0])
    
    traj.set_points(points)
    if soft_enabled:
        traj.set_velocities(velocities)
    
    return traj
