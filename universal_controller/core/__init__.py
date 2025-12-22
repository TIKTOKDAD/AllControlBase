"""核心模块"""
from .enums import ControllerState, TrajectoryMode, TransformStatus, HeadingMode, PlatformType
from .data_types import (
    Header, Point3D, Twist3D, Trajectory, EstimatorOutput, ControlOutput,
    ConsistencyResult, SafetyDecision, MPCHealthStatus, TimeoutStatus,
    AttitudeCommand, DiagnosticsV2, Odometry, Imu
)
from .interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IAttitudeController
)
from .ros_compat import (
    ROS_AVAILABLE, TF2_AVAILABLE,
    euler_from_quaternion, quaternion_from_euler,
    get_current_time, create_time, create_duration
)
