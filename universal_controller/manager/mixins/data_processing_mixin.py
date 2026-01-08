from typing import Dict, Any, Optional, Tuple
import numpy as np
import logging

from ...core.data_types import (
    Odometry, Imu, TimeoutStatus, EstimatorOutput, 
    Trajectory, ConsistencyResult
)

logger = logging.getLogger(__name__)

class DataProcessingMixin:
    """
    Mixin for ControllerManager data processing (time, state, consistency, transform).
    """

    def _compute_time_step(self, current_time: float) -> Tuple[float, bool]:
        """
        计算时间步长，处理暂停检测
        
        Returns:
            (actual_dt, skip_prediction): 实际时间步长和是否跳过预测
        """
        long_pause_threshold = self.config.get('system', {}).get('long_pause_threshold', 0.5)
        ekf_reset_threshold = self.config.get('system', {}).get('ekf_reset_threshold', 2.0)
        skip_prediction = False
        
        if self._last_update_time is not None:
            actual_dt = current_time - self._last_update_time
            
            if actual_dt > ekf_reset_threshold:
                logger.warning(f"Long pause detected ({actual_dt:.2f}s), resetting EKF and controllers")
                self._reset_all_stateful_components()
                    
                skip_prediction = True
                actual_dt = self.dt
            elif actual_dt > long_pause_threshold:
                logger.info(f"Medium pause detected ({actual_dt:.2f}s), using multi-step prediction")
                # 限制最大补算步数，防止死循环 (例如 System Time Step = 20ms, Pause = 60s -> 3000 steps)
                # 如果暂停太久，与其卡死 CPU 补算，不如只补算最近的一段，或者重置
                MAX_CATCHUP_STEPS = 20  # Max 0.2s - 0.5s of catchup
                
                num_steps = int(actual_dt / self.dt)
                if num_steps > MAX_CATCHUP_STEPS:
                    logger.warning(f"Pause ({actual_dt:.2f}s) exceeds catchup limit. Resetting EKF to current state.")
                    self._reset_all_stateful_components()
                    
                    # 标记跳过预测，actual_dt 设为正常帧率，让 estimator 重新锁定 odom
                    skip_prediction = True
                    actual_dt = self.dt
                
                if not skip_prediction and self.state_estimator and num_steps > 1:
                    for _ in range(num_steps - 1):
                        self.state_estimator.predict(self.dt)
                actual_dt = actual_dt - (num_steps - 1) * self.dt
            
            # 修复 Bug: 移除 1ms 的硬性下限钳位，避免在高速高频调用时人为制造巨大的加速度计算值
            # 仅保留上限钳位用于处理暂停恢复，下限仅用于防除零 (1e-6)
            actual_dt = min(actual_dt, long_pause_threshold)
            actual_dt = max(actual_dt, 1e-6)
        else:
            actual_dt = self.dt
        
        self._last_update_time = current_time
        return actual_dt, skip_prediction

    def _update_state_estimation(self, odom: Odometry, imu: Optional[Imu],
                                 timeout_status: TimeoutStatus, actual_dt: float,
                                 current_time: float,
                                 skip_prediction: bool) -> Tuple[np.ndarray, Optional[EstimatorOutput]]:
        """
        更新状态估计
        
        Returns:
            (state, state_output): 状态向量和完整估计输出
        """
        state_output: Optional[EstimatorOutput] = None
        
        if self.state_estimator:
            if not skip_prediction:
                self.state_estimator.predict(actual_dt)
            
            # Fix: 防止陈旧的 Odom 数据被注入 EKF
            # 只有在 Odom 数据未超时的情况下才进行更新
            # 如果超时，EKF 将只依靠 predict (且可能被 skip_prediction 限制)
            if not timeout_status.odom_timeout:
                self.state_estimator.update_odom(odom, current_time)
            
            if timeout_status.imu_timeout:
                self.state_estimator.set_imu_available(False)
            elif imu:
                self.state_estimator.set_imu_available(True)
                self.state_estimator.update_imu(imu, current_time)
            
            state_output = self.state_estimator.get_state()
            state = state_output.state
        else:
            state = np.zeros(8)
        
        return state, state_output

    def _compute_consistency(self, trajectory: Trajectory) -> ConsistencyResult:
        """计算一致性"""
        if self.consistency_checker:
            consistency = self.consistency_checker.compute(trajectory)
        else:
            consistency = ConsistencyResult(0, 1, 1, 1, True, True)
        self._last_consistency = consistency
        return consistency

    def _transform_trajectory(self, trajectory: Trajectory, 
                              current_time: float) -> Tuple[Trajectory, bool]:
        """
        坐标变换
        
        将网络输出的局部轨迹变换到控制器工作坐标系。
        
        坐标系说明:
        - 输入轨迹: base_link (局部坐标系，当前位置为原点)
        - 输出轨迹: odom (世界坐标系)
        
        Returns:
            (transformed_traj, tf2_critical): 变换后轨迹和是否临界
        """
        # 优化: 仅变换所需轨迹片段，避免全量变换浪费算力
        # Fix: 之前的 slice_len (Horizon + 10) 对于 Pure Pursuit 可能过短
        # 增加切片长度以确保覆盖:
        # 1. MPC Horizon (e.g. 20-50)
        # 2. Pure Pursuit Lookahead (e.g. 3m @ 0.05m = 60 pts)
        # 3. Solver Buffer
        # 采用 max(Horizon * 4, 150) 作为保守且高效的阈值
        if trajectory.dt_sec > 1e-6:
            # 基于时间窗口切片，而不是固定点数
            # 5.0s 通常足够覆盖 MPC horizon (2s) 和 PurePursuit lookahead
            time_window = 5.0 
            safe_slice_len = int(np.ceil(time_window / trajectory.dt_sec))
            # 确保至少覆盖 MPC Horizon 的 4 倍（历史逻辑保留作为下限）
            safe_slice_len = max(safe_slice_len, self._current_horizon * 4)
        else:
            # Fallback for invalid dt
            safe_slice_len = 500
        
        if len(trajectory.points) > safe_slice_len:
             # 高性能切片: 使用 Trajectory.get_slice() 获取独立副本
             # 这确保了数据隔离，避免原轨迹修改影响切片数据
             traj_to_transform = trajectory.get_slice(0, safe_slice_len)
             
        else:
             traj_to_transform = trajectory

        if self.coord_transformer:
            # 准备 fallback_state
            fallback_state = None
            if self.state_estimator:
                fallback_state = self.state_estimator.get_state()
            
            transformed_traj, tf_status = self.coord_transformer.transform_trajectory(
                traj_to_transform, self.transform_target_frame, current_time,
                fallback_state=fallback_state)
            tf2_critical = tf_status.is_critical()
        else:
            # 无坐标变换器时，假设轨迹已经在正确的坐标系
            # 这种情况下，轨迹应该已经是世界坐标系
            transformed_traj = traj_to_transform
            tf2_critical = False
            
            # 如果轨迹声明在局部坐标系但没有变换器，发出警告
            if trajectory.header.frame_id in ['base_link', 'base_link_0']:
                logger.warning(
                    f"Trajectory in local frame '{trajectory.header.frame_id}' "
                    f"but no coordinate transformer configured. "
                    f"Control may be incorrect."
                )
        
        return transformed_traj, tf2_critical
