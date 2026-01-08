from typing import Optional
import numpy as np
import logging

from ...core.data_types import Imu
from ...core.indices import StateIdx
from ...core.ros_compat import euler_from_quaternion
from ...core.constants import QUATERNION_NORM_SQ_MIN, QUATERNION_NORM_SQ_MAX

logger = logging.getLogger(__name__)

class EKFImuMixin:
    """
    Mixin for EKF IMU processing logic.
    """

    def set_imu_available(self, available: bool) -> None:
        """设置 IMU 可用状态（线程安全）"""
        self._acquire_lock()
        try:
            self._imu_available = available
        finally:
            self._release_lock()

    def update_imu(self, imu: Imu, current_time: float) -> None:
        """
        IMU 观测更新
        
        Args:
           imu: IMU 数据
           current_time: 当前时间戳
           
        IMU 测量模型 (比力模型):
        - 加速度计测量的是比力 (specific force): a_measured = a_true - g_body + bias + noise
        - 其中 a_true 是真实加速度，g_body 是重力在机体坐标系的投影

        - 静止时: a_measured = -g_body + bias ≈ [0, 0, +g] + bias (假设水平)
        - 陀螺仪测量: omega_measured = omega + bias + noise

        坐标系约定 (ENU):
        - 世界坐标系: X-东, Y-北, Z-上
        - 重力向量 (世界坐标系): g_world = [0, 0, -9.81] m/s²

        重力在机体坐标系的投影 (ZYX 欧拉角约定):
        - g_body = R_world_to_body @ [0, 0, -g]
        - g_body_x = -g * sin(pitch)
        - g_body_y = g * sin(roll) * cos(pitch)
        - g_body_z = -g * cos(roll) * cos(pitch)

        加速度计静止时的期望测量值 (比力 = -g_body):
        - a_expected_x = g * sin(pitch)
        - a_expected_y = -g * sin(roll) * cos(pitch)
        - a_expected_z = g * cos(roll) * cos(pitch)  (约 +9.81 当水平时)
        """
        if not self._imu_available:
            return

        # 安全关键: 检查 IMU 数据有效性
        # 传感器故障可能返回 NaN/Inf，必须在处理前检测
        accel = imu.linear_acceleration
        gyro = imu.angular_velocity

        if not (np.isfinite(accel[0]) and np.isfinite(accel[1]) and np.isfinite(accel[2])):
            if not self._warning_logged.get('accel_nan', False):
                logger.warning(
                    f"Invalid IMU acceleration data (NaN/Inf): "
                    f"ax={accel[0]}, ay={accel[1]}, az={accel[2]}. Skipping IMU update."
                )
                self._warning_logged['accel_nan'] = True
            return

        if not (np.isfinite(gyro[0]) and np.isfinite(gyro[1]) and np.isfinite(gyro[2])):
            if not self._warning_logged.get('gyro_nan', False):
                logger.warning(
                    f"Invalid IMU angular velocity data (NaN/Inf): "
                    f"wx={gyro[0]}, wy={gyro[1]}, wz={gyro[2]}. Skipping IMU update."
                )
                self._warning_logged['gyro_nan'] = True
            return

        self._acquire_lock()
        try:
            # 资源有效性检查
            if self._F is None:
                return
            
            self._update_imu_internal(imu, accel, gyro, current_time)
        finally:
            self._release_lock()

    def _update_imu_internal(self, imu: Imu, accel, gyro, current_time: float) -> None:
        """IMU 更新的内部实现（已在锁保护下调用）"""
        self.gyro_z = imu.angular_velocity[2]
        
        # 缓存最新的 IMU 姿态（如果有效）
        if imu.orientation is not None:
             try:
                # 简单的有效性检查和转换
                q = imu.orientation
                if hasattr(q, '__len__') and len(q) == 4:
                     q_arr = np.array(q, dtype=np.float64)
                     if np.isfinite(q_arr).all() and abs(np.sum(q_arr**2) - 1.0) < 0.1:
                         self._last_imu_orientation = q_arr
             except Exception:
                 pass
                 
        theta = self.x[StateIdx.YAW]  # yaw 角
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # 首先计算重力在机体坐标系的期望测量值（比力）
        # 对于地面车辆，假设 roll 和 pitch 接近 0
        # 对于无人机，使用 IMU 提供的姿态信息
        g = self.gravity  # 使用配置的重力加速度
        use_imu_orientation = False
        cached_quat = None  # 缓存解析后的四元数分量 (qx, qy, qz, qw)
        
        if imu.orientation is not None:
            # 检查四元数有效性
            q = imu.orientation
            try:
                # 尝试提取四元数分量
                # 支持多种数据格式：tuple, list, numpy array, 或带索引的自定义对象
                if hasattr(q, '__len__') and len(q) == 4:
                    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
                elif hasattr(q, 'x') and hasattr(q, 'y') and hasattr(q, 'z') and hasattr(q, 'w'):
                    # 支持 geometry_msgs/Quaternion 风格的对象
                    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
                else:
                    # 不支持的格式
                    qx, qy, qz, qw = None, None, None, None
                
                if qx is not None:
                    # 检查各分量是否为 NaN 或 Inf
                    if not (np.isfinite(qx) and np.isfinite(qy) and 
                            np.isfinite(qz) and np.isfinite(qw)):
                        # 四元数包含无效值
                        use_imu_orientation = False
                    else:
                        norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
                        
                        # 四元数有效性检查:
                        # 1. 范数不能太小（接近零向量）
                        # 2. 范数应该接近 1（单位四元数）
                        is_valid = (
                            norm_sq > QUATERNION_NORM_SQ_MIN and
                            norm_sq < QUATERNION_NORM_SQ_MAX
                        )
                        
                        if is_valid:
                            roll, pitch, _ = euler_from_quaternion((qx, qy, qz, qw))
                            
                            # 检查 euler_from_quaternion 返回值是否有效
                            # 虽然 euler_from_quaternion 内部有归一化，但仍可能返回 NaN
                            if not (np.isfinite(roll) and np.isfinite(pitch)):
                                # 欧拉角计算失败，使用默认姿态
                                if not self._warning_logged['euler_nan']:
                                    logger.warning(
                                        f"euler_from_quaternion returned NaN/Inf: "
                                        f"roll={roll}, pitch={pitch}, q=({qx},{qy},{qz},{qw})"
                                    )
                                    self._warning_logged['euler_nan'] = True
                                use_imu_orientation = False
                            # 额外检查: roll 和 pitch 应该在合理范围内
                            # 使用配置的最大倾斜角阈值
                            elif abs(roll) < self.max_tilt_angle and abs(pitch) < self.max_tilt_angle:
                                use_imu_orientation = True
                                # 缓存四元数分量，避免后续重复解析
                                cached_quat = (qx, qy, qz, qw)
                            else:
                                # roll/pitch 超出最大倾斜角阈值，使用默认姿态
                                # 这可能是传感器故障或机器人翻倒
                                if not self._warning_logged['tilt_exceeded']:
                                    logger.warning(
                                        f"IMU tilt angle exceeded max_tilt_angle ({np.degrees(self.max_tilt_angle):.1f}°): "
                                        f"roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°. "
                                        f"Using default horizontal orientation."
                                    )
                                    self._warning_logged['tilt_exceeded'] = True
                                use_imu_orientation = False
            except (TypeError, ValueError, IndexError, AttributeError) as e:
                # 四元数数据格式错误，使用默认姿态
                # 只在首次遇到时记录警告，避免日志泛滥
                if not self._warning_logged['quaternion_error']:
                    logger.warning(f"Invalid quaternion format in IMU data: {e}")
                    self._warning_logged['quaternion_error'] = True
                use_imu_orientation = False
        if use_imu_orientation:
            # 加速度计静止时的期望测量值 (比力 = -g_body)
            g_measured_x = g * np.sin(pitch)
            g_measured_y = -g * np.sin(roll) * np.cos(pitch)
            g_measured_z = g * np.cos(roll) * np.cos(pitch)
        else:
            # 没有有效姿态信息，假设水平
            g_measured_x = 0.0
            g_measured_y = 0.0
            g_measured_z = g
        
        # 从 IMU 测量值中提取真实加速度（去除重力和 bias）
        # a_true_body = a_measured - g_measured - bias
        
        ax_body_true = imu.linear_acceleration[0] - g_measured_x - self.x[StateIdx.ACCEL_BIAS_X]
        ay_body_true = imu.linear_acceleration[1] - g_measured_y - self.x[StateIdx.ACCEL_BIAS_Y]
        az_body_true = imu.linear_acceleration[2] - g_measured_z - self.x[StateIdx.ACCEL_BIAS_Z]
        
        # 将真实加速度转换到世界坐标系
        # 根据 use_imu_orientation 选择旋转策略:
        # - True: 使用完整的四元数进行 3D 旋转（支持无人机/倾斜地形）
        # - False: 使用简化的仅 Yaw 旋转（地面车辆假设水平）
        
        if use_imu_orientation and cached_quat is not None:
            # 完整的 3D 四元数旋转
            # 优化: 使用显式代数运算代替 NumPy 数组分配和 np.cross
            
            # 使用已缓存的四元数分量（在上面的验证过程中已解析）
            qx, qy, qz, qw = cached_quat
            
            # 高性能归一化: 仅在明显偏离单位四元数时进行 (容差 0.1%)
            q_norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
            if abs(q_norm_sq - 1.0) > 1e-3:
                if q_norm_sq > 1e-10:
                    q_norm_inv = 1.0 / np.sqrt(q_norm_sq)
                    qx, qy, qz, qw = qx * q_norm_inv, qy * q_norm_inv, qz * q_norm_inv, qw * q_norm_inv
                else:
                    # 四元数接近零，使用单位四元数 (无旋转)
                    logger.warning("IMU quaternion near zero, using identity rotation")
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            
            # 目标向量 v = [ax_body, ay_body, az_body]
            vx, vy, vz = ax_body_true, ay_body_true, az_body_true
            
            # 四元数旋转向量: v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + qw * v)
            # 展开计算: t = 2 * cross(q_xyz, v), v' = v + qw * t + cross(q_xyz, t)
            tx = 2.0 * (qy * vz - qz * vy)
            ty = 2.0 * (qz * vx - qx * vz)
            tz = 2.0 * (qx * vy - qy * vx)
            
            ax_world = vx + qw * tx + (qy * tz - qz * ty)
            ay_world = vy + qw * ty + (qz * tx - qx * tz)
            # az_world = vz + qw * tz + (qx * ty - qy * tx)  # 暂不使用
        else:
            # 简化的 2D 旋转 (仅使用 EKF 状态中的 yaw)
            # 适用于地面车辆（假设水平）或无有效姿态信息时
            # ax_world = ax_body * cos(yaw) - ay_body * sin(yaw)
            # ay_world = ax_body * sin(yaw) + ay_body * cos(yaw)
            ax_world = ax_body_true * cos_theta - ay_body_true * sin_theta
            ay_world = ax_body_true * sin_theta + ay_body_true * cos_theta
        
        # 打滑检测：比较 IMU 测量的加速度 (Body Frame) 与 Odom 计算的加速度 (Body Frame)
        # 修正: 统一在 Body Frame 下进行比较，避免 Yaw 漂移 带来的虚假打滑报警
        
        odom_accel_fresh = (
            self._body_accel_initialized and 
            self.last_odom_time is not None and
            (current_time - self.last_odom_time) < self.accel_freshness_thresh
        )
        
        if odom_accel_fresh:
            # 1. 获取 IMU 真实加速度 (Body Frame, 去重力和 Bias)
            # ax_body_true, ay_body_true 已经在上面计算好了
            imu_accel_body_vec = np.array([ax_body_true, ay_body_true])
            
            # 2. 获取 Odom 加速度 (Body Frame)
            # self.body_accel_vec 已经在 update_odom 中计算好
            
            # 3. 比较模长差
            accel_diff = np.linalg.norm(imu_accel_body_vec - self.body_accel_vec)
            
            self.slip_probability = self._compute_slip_probability(accel_diff)
            self.slip_detected = self.slip_probability > 0.5
        else:
            # odom 加速度数据过旧，不进行打滑检测
            # 保持上一次的打滑状态，但基于时间间隔衰减概率
            if self.slip_probability > 0 and self.last_imu_time is not None:
                dt_decay = current_time - self.last_imu_time
                if dt_decay > 0:
                    # 基于时间的指数衰减: P(t) = P(0) * exp(-decay_rate * dt)
                    decay_factor = np.exp(-self.slip_decay_rate * dt_decay)
                    self.slip_probability *= decay_factor
            self.slip_detected = self.slip_probability > 0.5
        
        self.last_imu_time = current_time
        
        # 复用预分配的观测向量
        self._z_imu[0] = imu.linear_acceleration[0]
        self._z_imu[1] = imu.linear_acceleration[1]
        self._z_imu[2] = imu.linear_acceleration[2]
        self._z_imu[3] = imu.angular_velocity[2]
        
        # H 矩阵是 constant 且已预分配 (self._H_imu)
        
        # 计算期望值
        bias_x = self.x[StateIdx.ACCEL_BIAS_X]
        bias_y = self.x[StateIdx.ACCEL_BIAS_Y]
        bias_z = self.x[StateIdx.ACCEL_BIAS_Z]
        omega_current = self.x[StateIdx.YAW_RATE]
        
        if self.imu_motion_compensation and self._body_accel_initialized:
            # Motion Compensation in Body Frame
            # Accel_measured_expected = a_kinematic_body + gravity_body
            # a_kinematic_body is approximated by Odom derived body acceleration
            
            self._z_imu_expected[0] = bias_x + self.body_accel_vec[0] + g_measured_x
            self._z_imu_expected[1] = bias_y + self.body_accel_vec[1] + g_measured_y
            self._z_imu_expected[2] = bias_z + g_measured_z
            self._z_imu_expected[3] = omega_current
        else:
            self._z_imu_expected[0] = bias_x + g_measured_x
            self._z_imu_expected[1] = bias_y + g_measured_y
            self._z_imu_expected[2] = bias_z + g_measured_z
            self._z_imu_expected[3] = omega_current
        
        self._kalman_update_with_expected(self._z_imu, self._z_imu_expected, self._H_imu, self.R_imu,
                                        buffers=(self._temp_H4_P, self._temp_H4_P_HT, self._temp_K_4, self._temp_K_y_4, self._temp_PHt_4, self._temp_K_S_4, self._temp_P_update_11),
                                        y_buffer=self._y_imu)

    def _get_orientation_quat(self) -> np.ndarray:
        """获取方位角四元数 (Robust)
        
        如果 IMU 可用且有效，使用 IMU 数据。
        否则 (Odom Only 模式)，从状态向量 x 中的 Yaw 角合成一个水平四元数。
        这解决了 "Ghost Locking" 问题：
        即无 IMU 时，Transformer 会读取到默认的 Identity Quaternion (Yaw=0)，
        而忽略了 EKF 状态中已经正确估计出的 Yaw。
        """
        if self._imu_available and self._last_imu_orientation is not None:
             # 简单的有效性检查 (非零)
            if np.sum(np.abs(self._last_imu_orientation)) > 1e-3:
                return self._last_imu_orientation.copy()
        
        # Fallback: Synthesize from Yaw
        yaw = self.x[StateIdx.YAW]
        half_yaw = yaw * 0.5
        # [x, y, z, w] -> [0, 0, sin(y/2), cos(y/2)]
        return np.array([0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)], dtype=np.float64)
