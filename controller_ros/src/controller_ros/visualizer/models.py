"""
可视化数据模型

定义可视化模块使用的数据结构，与 ROS 消息解耦。
"""
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import IntEnum
import numpy as np


class ControlMode(IntEnum):
    """控制模式"""
    NETWORK = 0      # 网络轨迹控制
    JOYSTICK = 1     # 手柄控制


class TrajectoryMode(IntEnum):
    """轨迹模式 (与 UC TrajectoryMode 对应)"""
    TRACK = 0
    STOP = 1
    HOVER = 2
    EMERGENCY = 3


@dataclass
class Point2D:
    """2D 点"""
    x: float = 0.0
    y: float = 0.0


@dataclass
class Point3D:
    """3D 点"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class VelocityData:
    """速度数据"""
    linear_x: float = 0.0      # 线速度 x (m/s)
    linear_y: float = 0.0      # 线速度 y (m/s)
    angular_z: float = 0.0     # 角速度 z (rad/s)
    timestamp: float = 0.0     # 时间戳 (秒)


@dataclass
class TrajectoryData:
    """轨迹数据"""
    points: List[Point3D] = field(default_factory=list)
    mode: TrajectoryMode = TrajectoryMode.TRACK
    confidence: float = 0.0
    dt_sec: float = 0.1
    frame_id: str = 'base_link'
    timestamp: float = 0.0
    
    @property
    def num_points(self) -> int:
        return len(self.points)
    
    @property
    def mode_name(self) -> str:
        return self.mode.name


@dataclass
class RobotPose:
    """机器人位姿"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0           # 航向角 (rad)
    timestamp: float = 0.0


@dataclass
class JoystickState:
    """手柄状态"""
    connected: bool = False
    left_x: float = 0.0        # 左摇杆 X 轴 [-1, 1]
    left_y: float = 0.0        # 左摇杆 Y 轴 [-1, 1]
    right_x: float = 0.0       # 右摇杆 X 轴 [-1, 1]
    right_y: float = 0.0       # 右摇杆 Y 轴 [-1, 1]
    enable_pressed: bool = False  # 使能键是否按下 (LB)
    estop_pressed: bool = False   # 紧急停止键是否按下 (RB)
    resume_pressed: bool = False  # 恢复键是否按下 (Start)
    buttons: List[int] = field(default_factory=list)
    
    @property
    def linear_cmd(self) -> float:
        """从左摇杆 Y 轴获取线速度命令 [-1, 1]"""
        return self.left_y
    
    @property
    def angular_cmd(self) -> float:
        """从右摇杆 X 轴获取角速度命令 [-1, 1]
        
        Xbox 360 Controller (Linux xpad 驱动):
        - 向右推杆 -> axes[3] 为负值 -> 应产生负 angular.z -> 机器人右转
        - 向左推杆 -> axes[3] 为正值 -> 应产生正 angular.z -> 机器人左转
        """
        return self.right_x  # 直接使用，不取反


@dataclass
class ControllerStatus:
    """控制器状态"""
    state: int = 0             # ControllerState 枚举值
    state_name: str = 'INIT'
    mpc_success: bool = False
    mpc_solve_time_ms: float = 0.0
    backup_active: bool = False
    odom_timeout: bool = False
    traj_timeout: bool = False
    emergency_stop: bool = False
    error_message: str = ''


@dataclass
class VisualizerData:
    """可视化器聚合数据"""
    # 速度数据
    target_velocity: VelocityData = field(default_factory=VelocityData)
    actual_velocity: VelocityData = field(default_factory=VelocityData)
    
    # 轨迹数据
    trajectory: TrajectoryData = field(default_factory=TrajectoryData)
    
    # 机器人位姿
    robot_pose: RobotPose = field(default_factory=RobotPose)
    
    # 手柄状态
    joystick: JoystickState = field(default_factory=JoystickState)
    
    # 控制器状态
    controller_status: ControllerStatus = field(default_factory=ControllerStatus)
    
    # 控制模式
    control_mode: ControlMode = ControlMode.NETWORK
    
    # 相机图像 (可选)
    camera_image: Optional[np.ndarray] = None
    
    # 连接状态
    ros_connected: bool = False
