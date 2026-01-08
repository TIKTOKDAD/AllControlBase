from typing import Optional
import numpy as np
import logging

from ...core.data_types import Odometry
from ...core.indices import StateIdx
from ...core.ros_compat import euler_from_quaternion, normalize_angle

logger = logging.getLogger(__name__)

class EKFOdomMixin:
    """
    Mixin for EKF Odometry processing logic.
    """

    def update_odom(self, odom: Odometry, current_time: float) -> None:
        """Odom 更新
        
        Args:
            odom: 里程计数据
            current_time: 当前时间戳 (使用与 predict 一致的时间源)
        """
        self._acquire_lock()
        try:
            # 重置陈旧时间
            self._time_since_last_odom = 0.0
            
            # 资源有效性检查
            if self._F is None:
                return
            
            self._last_odom_orientation = odom.pose_orientation

            new_position = np.array([
                odom.pose_position.x,
                odom.pose_position.y,
                odom.pose_position.z
            ])

            # ------------------------------------------------------------------
            # 初始化逻辑: 如果是 Reset 后的第一帧，直接硬初始化状态
            # 避免 "Zero State" 与 "Actual Position" 之间的巨大差异触发跳变检测 (Phantom Jump)
            # ------------------------------------------------------------------
            if not self._initialized:
                self.x[StateIdx.X] = new_position[0]
                self.x[StateIdx.Y] = new_position[1]
                self.x[StateIdx.Z] = new_position[2]
                
                # Parse Quaternion once
                yaw = self._quaternion_to_yaw(odom.pose_orientation)
                
                self.x[StateIdx.YAW] = normalize_angle(yaw)
                self.x[StateIdx.YAW_RATE] = odom.twist_angular[2]
                
                vx_b, vy_b = odom.twist_linear[0], odom.twist_linear[1]
                c, s = np.cos(yaw), np.sin(yaw)
                self.x[StateIdx.VX] = vx_b * c - vy_b * s
                self.x[StateIdx.VY] = vx_b * s + vy_b * c
                self.x[StateIdx.VZ] = odom.twist_linear[2]

                self.last_position = new_position
                self.position_jump = 0.0
                
                # 初始化缓存变量，防止虚假打滑
                self.current_body_velocity[0] = vx_b
                self.current_body_velocity[1] = vy_b
                self.last_body_velocity[:] = self.current_body_velocity
                self.last_odom_time = current_time
                
                self._initialized = True
                logger.info(f"EKF state hard-initialized from Odom at {current_time:.3f}s")
                return

            self.position_jump = np.linalg.norm(new_position - self.last_position)
            self.last_position = new_position

            vx_body = odom.twist_linear[0]
            vy_body = odom.twist_linear[1]
            vz_body = odom.twist_linear[2]
            v_body = np.sqrt(vx_body**2 + vy_body**2)
            self._raw_odom_twist_norm = v_body  # Store raw for drift detection

            # 从 odom 获取角速度 (z 轴)
            omega_z = odom.twist_angular[2]
            self._raw_odom_angular_velocity = omega_z

            # 从 odom 四元数提取航向角 (一次性解析)
            odom_yaw = self._quaternion_to_yaw(odom.pose_orientation)

            # 获取航向角用于坐标变换
            # 使用 odom 的航向角，而不是 EKF 估计的航向角
            # 这确保速度变换使用正确的航向

            
            # 记录用于后续步骤的全局转换
            # Record global transformation for subsequent steps (odom_yaw already computed above)

            
            # ------------------------------------------------------------------
            # 打滑检测专用: 计算 Body Frame 下的加速度
            # a_body = dv_body/dt + omega x v_body
            # ------------------------------------------------------------------
            self.last_body_velocity[:] = self.current_body_velocity
            self.current_body_velocity[0] = vx_body
            self.current_body_velocity[1] = vy_body
            
            if self.last_odom_time is not None:
                dt_odom = current_time - self.last_odom_time
                if dt_odom > 0.0001:
                    # 1. 线性加速度项: (v_curr - v_last) / dt
                    dv_dt_x = (vx_body - self.last_body_velocity[0]) / dt_odom
                    dv_dt_y = (vy_body - self.last_body_velocity[1]) / dt_odom
                    
                    # 2. 科氏/向心项: omega x v
                    # a_cor_x = -v_y * omega
                    # a_cor_y = +v_x * omega
                    # 使用当前和上一时刻速度的平均值会让积分更准，但这里用当前值即可
                    a_cor_x = -vy_body * omega_z
                    a_cor_y =  vx_body * omega_z
                    
                    self.body_accel_vec[0] = dv_dt_x + a_cor_x
                    self.body_accel_vec[1] = dv_dt_y + a_cor_y
                    self._body_accel_initialized = True
                else:
                    self.body_accel_vec = np.zeros(2)
            else:
                self.body_accel_vec = np.zeros(2)
             
            # ------------------------------------------------------------------
            # 观测向量构建
            # ------------------------------------------------------------------
            theta = odom_yaw
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            vx_world = vx_body * cos_theta - vy_body * sin_theta
            vy_world = vx_body * sin_theta + vy_body * cos_theta
            
            if self.is_quadrotor:
                # 3D 平台 (无人机) 使用完整的四元数旋转
                # 之前简单的 2D 投影会导致 Pitch/Roll 倾斜时垂直速度估计严重错误
                # v_world = R * v_body
                
                # 向量旋转公式: v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)
                # v = [vx_body, vy_body, vz_body]
                # t = 2 * cross(q_xyz, v)
                # v' = v + qw * t + cross(q_xyz, t)

                # Fix: Extract quaternion components from odom
                qx, qy, qz, qw = odom.pose_orientation[0], odom.pose_orientation[1], \
                                 odom.pose_orientation[2], odom.pose_orientation[3]
                
                tx = 2.0 * (qy * vz_body - qz * vy_body)
                ty = 2.0 * (qz * vx_body - qx * vz_body)
                tz = 2.0 * (qx * vy_body - qy * vx_body)
                
                vx_world = vx_body + qw * tx + (qy * tz - qz * ty)
                vy_world = vy_body + qw * ty + (qz * tx - qx * tz)
                vz_world = vz_body + qw * tz + (qx * ty - qy * tx)
            else:
                # 地面车辆假设 Pitch/Roll 接近 0，使用高效的 2D 投影
                vz_world = vz_body

            self.last_odom_time = current_time
            self._update_odom_covariance(v_body)

            # 观测向量 z (复用预分配数组)
            self._z_odom[0] = odom.pose_position.x
            self._z_odom[1] = odom.pose_position.y
            self._z_odom[2] = odom.pose_position.z
            self._z_odom[3] = vx_world
            self._z_odom[4] = vy_world
            self._z_odom[5] = vz_world
            self._z_odom[6] = odom_yaw
            self._z_odom[7] = omega_z

            # H 矩阵在 __init__ 中已初始化 (self._H_odom)
            
            # R 矩阵 (复用预分配矩阵)
            self._R_odom.fill(0.0)
            
            # 填充对角线
            self._R_odom[0:6, 0:6] = self.R_odom_current
            self._R_odom[6, 6] = self._odom_orientation_noise
            self._R_odom[7, 7] = self._odom_angular_velocity_noise

            # 里程计跳变处理
            if self.position_jump > self.jump_thresh:
                jump_scale = min(self.position_jump / self.jump_thresh, 10.0)
                # 原地修改 R 的前三行前三列
                self._R_odom[0:3, 0:3] *= (jump_scale ** 2)
                logger.warning(
                    f"Odom position jump detected: {self.position_jump:.3f}m > {self.jump_thresh}m, "
                    f"increasing position measurement noise by {jump_scale**2:.1f}x"
                )

            self._kalman_update(self._z_odom, self._H_odom, self._R_odom, 
                              angle_indices=[StateIdx.YAW],
                              buffers=(self._temp_H8_P, self._temp_H8_P_HT, self._temp_K_8, self._temp_K_y_8, self._temp_PHt_8, self._temp_K_S_8, self._temp_P_update_11),
                              y_buffer=self._y_odom)
        finally:
            self._release_lock()

    def _update_odom_covariance(self, v_body: float) -> None:
        """更新 odom 测量噪声协方差"""
        if self.slip_detected:
            self.R_odom_current = self.R_odom_base * self.slip_covariance_scale
        elif self._is_stationary():
            self.R_odom_current = self.R_odom_base * self.stationary_covariance_scale
        else:
            self.R_odom_current = self.R_odom_base.copy()
        
        # 当速度较高时，航向角不确定性会影响速度估计的不确定性
        # 使用 stationary_thresh 作为阈值，保持一致性
        theta_var = self.P[StateIdx.YAW, StateIdx.YAW]
        if theta_var > 0 and v_body > self.stationary_thresh * 2:
            velocity_transform_var = (v_body ** 2) * theta_var
            self.R_odom_current[3, 3] += velocity_transform_var
            self.R_odom_current[4, 4] += velocity_transform_var

    def _quaternion_to_yaw(self, q: np.ndarray) -> float:
        """
        Helper to convert quaternion [x, y, z, w] to yaw (Z-axis rotation).
        
        Args:
            q: Quaternion array [x, y, z, w]
            
        Returns:
            float: Yaw angle in radians
        """
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return np.arctan2(siny_cosp, cosy_cosp)
