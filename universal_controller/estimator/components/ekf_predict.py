import numpy as np
import logging
from ...core.indices import StateIdx
from ...core.ros_compat import normalize_angle

logger = logging.getLogger(__name__)

class EKFPredictMixin:
    """
    Mixin for EKF Prediction logic.
    """

    def apply_drift_correction(self, dx: float, dy: float, dtheta: float) -> None:
        """应用外部漂移校正（线程安全）

        对位置和航向进行外部校正（如来自 SLAM 或 GPS 的修正）。
        对于速度-航向耦合平台，同时更新速度方向以保持与新航向一致。

        Args:
            dx: X 方向位置校正量（米）
            dy: Y 方向位置校正量（米）
            dtheta: 航向角校正量（弧度）
        """
        self._acquire_lock()
        try:
            self.x[StateIdx.X] += dx
            self.x[StateIdx.Y] += dy
            self.x[StateIdx.YAW] += dtheta
            self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

            if self.velocity_heading_coupled:
                # 优雅修复: 使用旋转矩阵对速度向量进行旋转
                # 之前的方法 (project -> rotate) 会丢失横向速度分量 (Lateral Velocity)
                # 正确做法是保留 Body Frame 下的速度不变，随着 Yaw 的变化旋转 World Frame 速度
                
                # 1. 计算旋转前的 World 速度
                vx_old = self.x[StateIdx.VX]
                vy_old = self.x[StateIdx.VY]
                
                # 2. 旋转速度向量 (Rotate by dtheta)
                # [vx_new]   [cos(dtheta)  -sin(dtheta)] [vx_old]
                # [vy_new] = [sin(dtheta)   cos(dtheta)] [vy_old]
                cos_d = np.cos(dtheta)
                sin_d = np.sin(dtheta)
                
                self.x[StateIdx.VX] = vx_old * cos_d - vy_old * sin_d
                self.x[StateIdx.VY] = vx_old * sin_d + vy_old * cos_d

            self.P[StateIdx.X, StateIdx.X] += abs(dx) * 0.1
            self.P[StateIdx.Y, StateIdx.Y] += abs(dy) * 0.1
            self.P[StateIdx.YAW, StateIdx.YAW] += abs(dtheta) * 0.1
        finally:
            self._release_lock()
    
    def predict(self, dt: float) -> None:
        """
        运动学预测

        标准 EKF 预测步骤:
        1. 在当前状态上计算 Jacobian F
        2. 更新状态 x = f(x)
        3. 更新协方差 P = F @ P @ F.T + Q

        注意: Jacobian 必须在状态更新之前计算，使用预测前的状态值

        Args:
            dt: 时间步长（秒），必须为正有限值
        """
        # 验证 dt 是有效的正有限值
        # - dt <= 0: 无效的时间步长
        # - np.isnan(dt): NaN 值（NaN <= 0 返回 False，需要单独检查）
        # - np.isinf(dt): 无穷大值
        if dt <= 0 or not np.isfinite(dt):
            return

        self._acquire_lock()
        try:
            # 资源有效性检查 (防止 Shutdown 竞态)
            if self._F is None:
                return

            # 1. 先在当前状态上计算 Jacobian (预测前的状态)
            # 使用 StateIdx 访问状态
            
            # Fix: 如果尚未初始化，不进行预测，防止基于全零状态的发散
            if not self._initialized:
                return

            theta_before = self.x[StateIdx.YAW]
            omega_before = self.x[StateIdx.YAW_RATE]
            vx_before = self.x[StateIdx.VX]
            vy_before = self.x[StateIdx.VY]
            
            # 使用预分配的 _F 矩阵，并在 _update_jacobian_F 中填充
            # 注意：原代码逻辑是 _compute_jacobian 返回新的 F
            # 这里我们需要改为原地更新 _F
            self._update_jacobian_F(dt, theta_before, omega_before, vx_before, vy_before)

            # 2. 更新状态 (x = f(x))
            
            # Step 2.1: 先更新航向角 (因为速度方向依赖于航向)
            # 使用半隐式欧拉或简单欧拉更新
            d_theta = self.x[StateIdx.YAW_RATE] * dt
            self.x[StateIdx.YAW] += d_theta
            self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])
            
            # Step 2.2: 更新速度 (如果是耦合模型)
            # Jacobian 中假设了 vx_global 是随着 theta 旋转的 (即 body frame 下速度恒定)
            # 因此这里必须显式旋转速度向量，以匹配 Jacobian 的定义
            if self.velocity_heading_coupled:
                # [vx_new]   [cos(dtheta)  -sin(dtheta)] [vx_old]
                # [vy_new] = [sin(dtheta)   cos(dtheta)] [vy_old]
                # 这相当于保持 body frame 速度不变，旋转 global 速度向量
                c = np.cos(d_theta)
                s = np.sin(d_theta)
                vx_old = self.x[StateIdx.VX]
                vy_old = self.x[StateIdx.VY]
                
                self.x[StateIdx.VX] = vx_old * c - vy_old * s
                self.x[StateIdx.VY] = vx_old * s + vy_old * c
                
            # Step 2.3: 更新位置
            # 使用更新后的速度进行积分 (Semi-implicit Euler)，通常比显式欧拉更稳定
            self.x[StateIdx.X] += self.x[StateIdx.VX] * dt
            self.x[StateIdx.Y] += self.x[StateIdx.VY] * dt
            self.x[StateIdx.Z] += self.x[StateIdx.VZ] * dt
            
            # Step 2.4: 其他状态更新
            # omega 假设恒定 (零阶保持) 或已有模型，这里保持不变
            
            # 累计 Odom 陈旧时间
            self._time_since_last_odom += dt

            # 对于速度-航向耦合平台（差速车/阿克曼车），强制速度方向与航向一致
            if self.velocity_heading_coupled and self.enable_non_holonomic_constraint:
                v_magnitude = np.sqrt(self.x[StateIdx.VX]**2 + self.x[StateIdx.VY]**2)
                
                # 低速时不强制约束
                if v_magnitude > self.stationary_thresh:
                    if self.slip_probability < self.non_holonomic_slip_threshold:
                        yaw = self.x[StateIdx.YAW]
                        # 使用点积计算带符号的投影速度
                        v_signed = self.x[StateIdx.VX] * np.cos(yaw) + self.x[StateIdx.VY] * np.sin(yaw)
                        self.x[StateIdx.VX] = v_signed * np.cos(yaw)
                        self.x[StateIdx.VY] = v_signed * np.sin(yaw)
                    else:
                        # 打滑时允许横向滑移
                        pass

            # 3. 更新协方差 P = F @ P @ F.T + Q * dt
            # 优化: 使用原地操作避免分配
            # Step 3.1: _temp_F_P = F @ P
            np.matmul(self._F, self.P, out=self._temp_F_P)
            
            # Step 3.2: P_pred = _temp_F_P @ F.T
            # 注意: self._F 是 array, .T 是属性，不是拷贝
            np.matmul(self._temp_F_P, self._F.T, out=self._temp_FP_FT)
            
            # Step 3.3: P = P_pred + Q * dt
            # Q 是对角阵，Q*dt 也是对角阵，直接加到 P_pred 上
            # In-place: self.P[:] = ...
            np.multiply(self.Q, dt, out=self._temp_F_P) # 复用 _temp_F_P 暂存 Q*dt
            np.add(self._temp_FP_FT, self._temp_F_P, out=self.P)
            
            self._ensure_positive_definite()
        finally:
            self._release_lock()

    def _update_jacobian_F(self, dt: float, theta: float, omega: float, 
                          vx: float, vy: float) -> None:
        """
        更新状态转移 Jacobian 矩阵 (原地修改 self._F)
        """
        # 我们只修改变动的项。_F 在 init 中被设为 eye(11)
        
        # 必须先清除上一帧可能设置的值（特别是那些根据条件分支设置的值）
        self._F[StateIdx.VX, StateIdx.YAW] = 0.0
        self._F[StateIdx.VY, StateIdx.YAW] = 0.0
        self._F[StateIdx.VX, StateIdx.YAW_RATE] = 0.0
        self._F[StateIdx.VY, StateIdx.YAW_RATE] = 0.0
        
        # 更新标准项
        self._F[StateIdx.X, StateIdx.VX] = dt  # px/vx
        self._F[StateIdx.Y, StateIdx.VY] = dt  # py/vy
        self._F[StateIdx.Z, StateIdx.VZ] = dt  # pz/vz
        self._F[StateIdx.YAW, StateIdx.YAW_RATE] = dt  # theta/omega
        
        if self.velocity_heading_coupled:
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # 计算带符号的速度投影
            v_signed = vx * cos_theta + vy * sin_theta
            
            # 平滑符号函数处理
            epsilon = self.jacobian_smooth_epsilon
            smooth_sign = np.tanh(v_signed / epsilon)
            v_magnitude_smooth = np.sqrt(v_signed**2 + self.min_velocity_for_jacobian**2)
            effective_v = smooth_sign * v_magnitude_smooth
            
            # ∂vx_world/∂theta
            self._F[StateIdx.VX, StateIdx.YAW] = -effective_v * sin_theta
            self._F[StateIdx.VY, StateIdx.YAW] = effective_v * cos_theta
            
            # ∂vx_world/∂omega
            self._F[StateIdx.VX, StateIdx.YAW_RATE] = -effective_v * sin_theta * dt
            self._F[StateIdx.VY, StateIdx.YAW_RATE] = effective_v * cos_theta * dt
