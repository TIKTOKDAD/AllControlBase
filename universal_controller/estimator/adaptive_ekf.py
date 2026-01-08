"""自适应 EKF 状态估计器"""
from typing import Dict, Any, Optional, List
from collections import deque
import numpy as np
import logging
import threading

from ..core.interfaces import IStateEstimator
from ..core.data_types import EstimatorOutput, Odometry, Imu
from ..config.default_config import PLATFORM_CONFIG
from ..core.ros_compat import euler_from_quaternion, get_monotonic_time, normalize_angle
from ..core.indices import StateIdx
from ..core.enums import PlatformType
from ..core.constants import (
    QUATERNION_NORM_SQ_MIN,
    QUATERNION_NORM_SQ_MAX,
    COVARIANCE_MIN_EIGENVALUE,
    COVARIANCE_INITIAL_VALUE,
    DEFAULT_GRAVITY,
    EKF_MAX_TILT_ANGLE,
    EKF_MIN_VELOCITY_FOR_JACOBIAN,
    EKF_COVARIANCE_EXPLOSION_THRESH,
    EKF_INNOVATION_ANOMALY_THRESH,
)

from .components import (
    EKFMathMixin,
    EKFDiagnosticsMixin,
    EKFImuMixin,
    EKFOdomMixin,
    EKFPredictMixin
)

logger = logging.getLogger(__name__)

class AdaptiveEKFEstimator(EKFMathMixin, EKFDiagnosticsMixin, EKFImuMixin, EKFOdomMixin, EKFPredictMixin, IStateEstimator):
    """自适应 EKF 状态估计器

    线程安全性:
        此类提供可选的线程安全支持：
        - 默认情况下不启用锁（thread_safe=False），以获得最佳性能
        - 设置 thread_safe=True 启用内部锁保护
        - 启用后，predict(), update_odom(), update_imu(), get_state() 方法是线程安全的
        - 如果不启用内部锁，调用者应确保在单线程中调用或在外部加锁

    典型调用顺序:
        1. predict(dt)      - 运动学预测
        2. update_odom(odom) - 里程计更新
        3. update_imu(imu)   - IMU 更新（可选）
        4. get_state()       - 获取估计结果
    """

    def __init__(self, config: Dict[str, Any], thread_safe: bool = False):
        """
        初始化 EKF 估计器

        Args:
            config: 配置字典
            thread_safe: 是否启用线程安全模式（默认 False 以获得最佳性能）
            
        Note:
            当 thread_safe=False 时，调用者必须确保在单线程中调用，
            或在外部加锁。在多线程环境中使用非线程安全模式可能导致数据竞争。
        """
        # 线程安全支持 - 优化：避免在非线程安全模式下的检查开销
        self._thread_safe = thread_safe
        if thread_safe:
            self._lock = threading.RLock()
            self._acquire_lock = self._lock.acquire
            self._release_lock = self._lock.release
        else:
            self._lock = None
            # 零开销的 No-op，返回 True 以匹配 RLock.acquire() 的返回类型
            self._acquire_lock = lambda: True
            self._release_lock = lambda: None
        
        # 记录调用线程 ID，用于检测多线程误用
        self._creation_thread_id = threading.get_ident()
        self._thread_warning_logged = False

        ekf_config = config.get('ekf', config)
        
        # 初始协方差值 - 使用常量
        self.initial_covariance = COVARIANCE_INITIAL_VALUE
        
        # 状态向量 [px, py, pz, vx, vy, vz, θ, ω, bias_ax, bias_ay, bias_az]
        self.x = np.zeros(11)
        self.P = np.eye(11) * self.initial_covariance
        
        # 平台类型
        platform_name = config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential'])
        self.velocity_heading_coupled = self.platform_config.get('velocity_heading_coupled', True)
        self.is_quadrotor = self.platform_config.get('type') == PlatformType.QUADROTOR
        
        # 重力加速度 - 使用物理常量
        self.gravity = DEFAULT_GRAVITY
        
        # 自适应参数
        adaptive = ekf_config.get('adaptive', {})
        self.base_slip_thresh = adaptive.get('base_slip_thresh', 2.0)
        self.slip_velocity_factor = adaptive.get('slip_velocity_factor', 0.5)
        self.slip_covariance_scale = adaptive.get('slip_covariance_scale', 10.0)
        self.stationary_covariance_scale = adaptive.get('stationary_covariance_scale', 0.1)
        self.stationary_thresh = adaptive.get('stationary_thresh', 0.05)
        self.slip_probability_k_factor = adaptive.get('slip_probability_k_factor', 5.0)
        # 打滑概率衰减参数
        self.slip_decay_rate = adaptive.get('slip_decay_rate', 2.0)  # 每秒衰减量
        self.slip_history_window = adaptive.get('slip_history_window', 20)
        
        # IMU 相关参数 - 使用常量
        self.max_tilt_angle = EKF_MAX_TILT_ANGLE
        self.accel_freshness_thresh = ekf_config.get('accel_freshness_thresh', 0.1)  # 100ms
        
        # Jacobian 计算参数 - 使用常量
        self.min_velocity_for_jacobian = EKF_MIN_VELOCITY_FOR_JACOBIAN
        self.jacobian_smooth_epsilon = ekf_config.get('jacobian_smooth_epsilon', 0.1)
        
        # 航向备选参数
        self.use_odom_orientation_fallback = ekf_config.get('use_odom_orientation_fallback', True)
        self.theta_covariance_fallback_thresh = ekf_config.get('theta_covariance_fallback_thresh', 0.5)
        self._last_odom_orientation = None
        
        # 测量噪声
        meas_noise = ekf_config.get('measurement_noise', {})
        self.R_odom_base = np.diag([
            meas_noise.get('odom_position', 0.01)] * 3 +
            [meas_noise.get('odom_velocity', 0.1)] * 3
        )
        self.R_odom_current = self.R_odom_base.copy()
        
        # 航向角测量噪声
        self._odom_orientation_noise = meas_noise.get('odom_orientation', 0.01)
        
        # 角速度测量噪声
        self._odom_angular_velocity_noise = meas_noise.get('odom_angular_velocity', 0.05)
        
        self.R_imu = np.diag([
            meas_noise.get('imu_accel', 0.5)] * 3 +
            [meas_noise.get('imu_gyro', 0.01)]
        )

        
        # 过程噪声
        proc_noise = ekf_config.get('process_noise', {})
        self.Q = np.diag([
            proc_noise.get('position', 0.001)] * 3 +
            [proc_noise.get('velocity', 0.1)] * 3 +
            [proc_noise.get('orientation', 0.01),
             proc_noise.get('angular_velocity', 0.1)] +
            [proc_noise.get('imu_bias', 0.0001)] * 3
        )
        
        # 异常检测阈值
        anomaly = ekf_config.get('anomaly_detection', {})
        self.drift_thresh = anomaly.get('drift_thresh', 0.1)
        self.jump_thresh = anomaly.get('jump_thresh', 0.5)
        # 使用常量作为协方差最小特征值
        self.min_eigenvalue = COVARIANCE_MIN_EIGENVALUE
        # 协方差爆炸检测阈值 - 使用常量
        self.covariance_explosion_thresh = EKF_COVARIANCE_EXPLOSION_THRESH
        # 创新度异常阈值 - 使用常量
        self.innovation_anomaly_thresh = EKF_INNOVATION_ANOMALY_THRESH
        
        # IMU 运动加速度补偿
        self.imu_motion_compensation = ekf_config.get('imu_motion_compensation', False)
        
        # 约束配置
        constraints = ekf_config.get('constraints', {})
        self.enable_non_holonomic_constraint = constraints.get('non_holonomic', True)
        
        # 智能配置修正: 全向平台不应启用非完整约束
        if platform_name in ('omni', 'mecanum') and self.enable_non_holonomic_constraint:
            logger.warning(
                f"Platform '{platform_name}' detected but non_holonomic constraint is ENABLED. "
                f"Auto-disabling non-holonomic constraint to allow lateral movement."
            )
            self.enable_non_holonomic_constraint = False
            
        self.non_holonomic_slip_threshold = constraints.get('non_holonomic_slip_threshold', 0.5)
        
        # 状态变量
        self.slip_detected = False
        self.slip_probability = 0.0
        self.last_innovation_norm = 0.0
        self.last_position = np.zeros(3)
        self.position_jump = 0.0
        self.gyro_z = 0.0
        self._imu_drift_detected = False
        
        # 打滑概率计算
        self.slip_history = deque(maxlen=self.slip_history_window)
        
        # Body Frame 加速度 (用于打滑检测)
        self.last_body_velocity = np.zeros(2)
        self.current_body_velocity = np.zeros(2)
        self._raw_odom_twist_norm = 0.0
        self._raw_odom_angular_velocity = 0.0  # 新增: 确保在 reset 前也有定义
        self.last_odom_time: Optional[float] = None
        self.body_accel_vec = np.zeros(2) # Derived from Odom
        self._body_accel_initialized = False
        self._initialized = False
        
        self.last_imu_time: Optional[float] = None
        self._imu_available = True
        self._time_since_last_odom = 1.0  # Time since last odom update (init high)
        self._last_imu_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Default Identity Quat
        
        # 日志节流标志 - 避免重复警告
        # 使用字典统一管理，便于 reset() 时清理
        self._warning_logged = {
            'quaternion_error': False,
            'euler_nan': False,
            'tilt_exceeded': False,
            'accel_nan': False,
            'gyro_nan': False,
        }

        # ----------------------------------------------------------------------
        # 性能优化: 预分配矩阵和向量
        # ----------------------------------------------------------------------
        # EKF 矩阵 (均在 __init__ 一次性分配)
        self._F = np.eye(11)
        self._H_odom = np.zeros((8, 11))
        # 预设固定 H_odom 元素
        self._H_odom[0:3, 0:3] = np.eye(3)
        self._H_odom[3:6, 3:6] = np.eye(3)
        self._H_odom[StateIdx.YAW, StateIdx.YAW] = 1.0
        self._H_odom[7, StateIdx.YAW_RATE] = 1.0
        
        self._R_odom = np.zeros((8, 8))
        self._z_odom = np.zeros(8)
        self._y_odom = np.zeros(8) # Innovation buffer
        
        self._H_imu = np.zeros((4, 11))
        # 预设固定 H_imu 元素
        self._H_imu[0, StateIdx.ACCEL_BIAS_X] = 1.0  # bias_ax
        self._H_imu[1, StateIdx.ACCEL_BIAS_Y] = 1.0  # bias_ay
        self._H_imu[2, StateIdx.ACCEL_BIAS_Z] = 1.0 # bias_az
        self._H_imu[3, StateIdx.YAW_RATE] = 1.0  # omega
        
        self._z_imu = np.zeros(4)
        self._y_imu = np.zeros(4) # Innovation buffer
        self._z_imu_expected = np.zeros(4)
        self._I_11 = np.eye(11) # 恒等矩阵

        # ----------------------------------------------------------------------
        # 优化: 中间计算 Buffer (避免运行时 malloc)
        # 命名约定: _temp_{rows}_{cols}_{usage}
        # ----------------------------------------------------------------------
        # Predict 阶段: P = F @ P @ F.T + Q * dt
        self._temp_F_P = np.zeros((11, 11))        # F @ P
        self._temp_P_FT = np.zeros((11, 11))       # P @ F.T (Not used directly if using F@P@FT) -> F@P -> (F@P)@F.T
        self._temp_FP_FT = np.zeros((11, 11))      # (F @ P) @ F.T
        
        # Update 阶段 (Odom 8维): 
        # y = z - Hx
        # S = H @ P @ H.T + R
        # K = P @ H.T @ S_inv
        self._temp_H8_P = np.zeros((8, 11))        # H @ P
        self._temp_H8_P_HT = np.zeros((8, 8))      # (H @ P) @ H.T
        self._temp_PHt_8 = np.zeros((11, 8))       # P @ H.T
        self._temp_K_8 = np.zeros((11, 8))         # K (Gain)
        self._temp_K_y_8 = np.zeros(11)            # K @ y
        self._temp_K_S_8 = np.zeros((11, 8))       # K @ S
        self._temp_P_update_11 = np.zeros((11, 11)) # K @ S @ K.T (Joseph term)

        # Update 阶段 (IMU 4维):
        self._temp_H4_P = np.zeros((4, 11))        # H @ P
        self._temp_H4_P_HT = np.zeros((4, 4))      # (H @ P) @ H.T
        self._temp_PHt_4 = np.zeros((11, 4))       # P @ H.T
        self._temp_K_4 = np.zeros((11, 4))         # K
        self._temp_K_y_4 = np.zeros(11)            # K @ y
        self._temp_K_S_4 = np.zeros((11, 4))       # K @ S
        
        # 对称化操作 Buffer (用于 _ensure_positive_definite)
        self._temp_P_sym = np.zeros((11, 11))      # P.T 拷贝

    def _get_theta_for_transform(self) -> float:
        """获取用于坐标变换的航向角"""
        theta_var = self.P[StateIdx.YAW, StateIdx.YAW]
        
        if (self.use_odom_orientation_fallback and 
            theta_var > self.theta_covariance_fallback_thresh and
            self._last_odom_orientation is not None):
            _, _, yaw = euler_from_quaternion(self._last_odom_orientation)
            return yaw
        
        return self.x[6]

    def get_state(self) -> EstimatorOutput:
        """获取当前状态估计结果（线程安全）"""
        self._acquire_lock()
        try:
            if self.P is None:  # Shutdown check
                return EstimatorOutput(
                    state=np.zeros(3), covariance=np.zeros((3,3)), covariance_norm=0, 
                    innovation_norm=0, imu_bias=np.zeros(3), slip_probability=0, anomalies=['SHUTDOWN']
                )

            return EstimatorOutput(
                state=self.x[:StateIdx.ACCEL_BIAS_X].copy(),
                covariance=self.P[:StateIdx.ACCEL_BIAS_X, :StateIdx.ACCEL_BIAS_X].copy(),
                covariance_norm=np.linalg.norm(self.P[:StateIdx.ACCEL_BIAS_X, :StateIdx.ACCEL_BIAS_X]),
                innovation_norm=self.last_innovation_norm,
                imu_bias=self.x[StateIdx.ACCEL_BIAS_X : StateIdx.ACCEL_BIAS_Z + 1].copy(),
                slip_probability=self.slip_probability,
                anomalies=self._detect_anomalies_unlocked(),
                imu_available=self._imu_available,
                imu_drift_detected=self._imu_drift_detected,
                # Fix Ghost Locking: Use helper function to guaranteed valid orientation
                orientation_quat=self._get_orientation_quat()
            )
        finally:
            self._release_lock()

    def reset(self, initial_state: Optional[np.ndarray] = None) -> None:
        """重置估计器状态（线程安全）
        
        Args:
            initial_state: 可选的初始状态向量 (11维). 如果提供，将立即初始化为该状态;
                          否则将重置为 False 等待第一次 Odom 更新.
        """
        self._acquire_lock()
        try:
            self.x = np.zeros(11)
            self.P = np.eye(11) * self.initial_covariance
            self.slip_detected = False
            self.slip_probability = 0.0
            self.last_innovation_norm = 0.0
            self.slip_history.clear()
            self.last_imu_time = None
            self.last_odom_time = None
            
            # 修复: 变量名必须与 __init__ 和 update_odom 中使用的 Body Frame 变量一致
            # 之前错误地使用了 world_ 前缀，导致 reset 无效，引发暂停后的打滑误报
            self.last_body_velocity = np.zeros(2)
            self.current_body_velocity = np.zeros(2)
            self._raw_odom_twist_norm = 0.0  # 必须重置，否则 static check 可能失效
            self._raw_odom_angular_velocity = 0.0 # Fix: 确保状态完全重置
            self.body_accel_vec = np.zeros(2)
            self._body_accel_initialized = False
            
            if initial_state is not None and len(initial_state) == 11:
                self.x = initial_state.copy()
                self._initialized = True
            else:
                self._initialized = False
            
            self.last_position = np.zeros(3)
            self.position_jump = 0.0
            self.gyro_z = 0.0
            self._imu_available = True
            self._imu_drift_detected = False
            self._last_odom_orientation = None
            # 重置日志节流标志，允许重置后再次记录警告
            for key in self._warning_logged:
                self._warning_logged[key] = False
        finally:
            self._release_lock()

    def shutdown(self) -> None:
        """关闭并释放所有资源
        
        调用后组件不应再使用。get_state() 将返回 SHUTDOWN 标记。
        
        Note:
            不调用 reset()，因为 reset() 会重新分配矩阵，
            而 shutdown 的目的是释放资源并标记组件为不可用。
        """
        self._acquire_lock()
        try:
            # 核心状态置空 - 确保 get_state() 的 shutdown 检查生效
            self.x = None
            self.P = None
            
            # 清理历史记录
            self.slip_history.clear()
            self.last_imu_time = None
            self.last_odom_time = None
            self._initialized = False
            
            # 释放大数组引用，帮助 GC
            self._F = None
            self._H_odom = None
            self._H_imu = None
            self._R_odom = None
            self._temp_F_P = None
            self._temp_FP_FT = None
            self._temp_P_update_11 = None
            self._temp_H8_P = None
            self._temp_H8_P_HT = None
            self._temp_PHt_8 = None
            self._temp_K_8 = None
            self._temp_H4_P = None
            self._temp_PHt_4 = None
            self._temp_K_4 = None
            self._temp_P_sym = None
            self._I_11 = None
            
            logger.info("AdaptiveEKFEstimator shutdown complete")
        finally:
            self._release_lock()
