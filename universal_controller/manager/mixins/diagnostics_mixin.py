from typing import Dict, Any, Optional
import numpy as np
import logging

from ...core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, EstimatorOutput,
    TimeoutStatus, DiagnosticsV2
)
from ...core.diagnostics_input import DiagnosticsInput
from ...core.constants import EPSILON, MIN_SEGMENT_LENGTH
from ...core.ros_compat import normalize_angle

logger = logging.getLogger(__name__)

class DiagnosticsMixin:
    """
    Mixin for ControllerManager diagnostics and metrics calculation.
    """

    def set_diagnostics_callback(self, callback: callable) -> None:
        """
        设置诊断回调函数
        
        回调函数签名: callback(diagnostics: DiagnosticsV2) -> None
        
        用于非 ROS 环境下获取诊断数据，或用于日志记录、监控等。
        如需字典格式，可在回调中调用 diagnostics.to_ros_msg()。
        """
        self._diagnostics_publisher.add_callback(callback)
    
    def get_last_published_diagnostics(self) -> Optional['DiagnosticsV2']:
        """获取最后发布的诊断数据（DiagnosticsV2 对象）"""
        return self._diagnostics_publisher.get_last_published()

    def get_tracking_quality(self) -> Optional[Dict[str, Any]]:
        """获取最新的跟踪质量评估结果"""
        return self._last_tracking_quality
        
    def get_diagnostics(self) -> Dict[str, Any]:
        if self._last_diagnostics is None:
            return {}
        return self._last_diagnostics.to_dict()

    def _update_tracking_metrics(self, state: np.ndarray, transformed_traj: Trajectory) -> None:
        """更新跟踪误差和质量评估指标"""
        self._last_tracking_error = self._compute_tracking_error(state, transformed_traj)
        self._last_tracking_quality = self._compute_tracking_quality(self._last_tracking_error)

    def _compute_tracking_error(self, state: np.ndarray, trajectory: Trajectory) -> Dict[str, float]:
        """
        计算跟踪误差
        
        包括:
        - lateral_error: 横向误差绝对值 (垂直于轨迹方向)
        - longitudinal_error: 纵向误差绝对值 (沿轨迹方向)
        - heading_error: 航向误差绝对值
        - prediction_error: 预测误差 (上一次 MPC 预测状态与当前实际状态的差异)
        
        注意:
        - 所有误差均为绝对值，用于诊断和质量评估
        - prediction_error 为 NaN 表示无预测数据（如使用 fallback 求解器时）
        
        边界情况处理:
        - 空轨迹 (0 点): 返回零误差，无法计算有意义的跟踪误差
        - 单点轨迹 (1 点): 计算到该点的距离作为误差，航向误差为指向该点的方向差
        - 正常轨迹 (≥2 点): 使用完整的横向/纵向误差分解
        """
        px, py = state[0], state[1]
        theta = state[6]
        
        # 计算预测误差 (在任何情况下都需要更新)
        prediction_error = float('nan')
        if self._last_mpc_predicted_state is not None:
            pred_px, pred_py = self._last_mpc_predicted_state[0], self._last_mpc_predicted_state[1]
            prediction_error = np.sqrt((px - pred_px)**2 + (py - pred_py)**2)
        
        # 更新预测状态
        if self.mpc_tracker is not None:
            self._last_mpc_predicted_state = self.mpc_tracker.get_predicted_next_state()
        else:
            self._last_mpc_predicted_state = None
        
        # 获取点矩阵 (兼容 numpy 数组和 Point3D 列表)
        points_matrix = trajectory.get_points_matrix()
        num_points = len(points_matrix)
        
        # 空轨迹: 无法计算有意义的误差
        if num_points == 0:
            return {'lateral_error': 0.0, 'longitudinal_error': 0.0, 
                    'heading_error': 0.0, 'prediction_error': prediction_error}
        
        # 单点轨迹: 计算到该点的距离和航向误差
        if num_points == 1:
            target_x, target_y = points_matrix[0, 0], points_matrix[0, 1]
            dx = target_x - px
            dy = target_y - py
            dist = np.sqrt(dx**2 + dy**2)
            
            # 航向误差: 当前航向与指向目标点方向的差值
            if dist > EPSILON:
                target_heading = np.arctan2(dy, dx)
                heading_error = abs(normalize_angle(theta - target_heading))
            else:
                heading_error = 0.0
            
            # 单点轨迹无法区分横向/纵向，将距离作为横向误差
            return {'lateral_error': dist, 'longitudinal_error': 0.0, 
                    'heading_error': heading_error, 'prediction_error': prediction_error}
        
        # 正常轨迹 (≥2 点): 完整的横向/纵向误差分解
        # 向量化计算最近点
        dists = np.sqrt((points_matrix[:, 0] - px)**2 + (points_matrix[:, 1] - py)**2)
        closest_idx = np.argmin(dists)
        min_dist = dists[closest_idx]
        
        # 确定用于计算误差的线段
        if closest_idx < num_points - 1:
            p0_x, p0_y = points_matrix[closest_idx, 0], points_matrix[closest_idx, 1]
            p1_x, p1_y = points_matrix[closest_idx + 1, 0], points_matrix[closest_idx + 1, 1]
        else:
            # closest_idx 是最后一个点，使用前一个线段
            p0_x, p0_y = points_matrix[closest_idx - 1, 0], points_matrix[closest_idx - 1, 1]
            p1_x, p1_y = points_matrix[closest_idx, 0], points_matrix[closest_idx, 1]
        
        dx = p1_x - p0_x
        dy = p1_y - p0_y
        traj_length = np.sqrt(dx**2 + dy**2)
        
        if traj_length < MIN_SEGMENT_LENGTH:
            return {'lateral_error': min_dist, 'longitudinal_error': 0.0,
                    'heading_error': 0.0, 'prediction_error': prediction_error}
        
        tx, ty = dx / traj_length, dy / traj_length
        ex = px - p0_x
        ey = py - p0_y
        
        longitudinal_error = abs(ex * tx + ey * ty)
        lateral_error = abs(-ex * ty + ey * tx)
        
        traj_heading = np.arctan2(dy, dx)
        heading_error = abs(normalize_angle(theta - traj_heading))
        
        return {
            'lateral_error': lateral_error,
            'longitudinal_error': longitudinal_error,
            'heading_error': heading_error,
            'prediction_error': prediction_error
        }
    
    def _compute_tracking_quality(self, tracking_error: Dict[str, float]) -> Dict[str, Any]:
        """
        计算跟踪质量评分
        
        使用 tracking 配置中的阈值和权重计算质量评分。
        """
        scores = {}
        
        # 计算各分量评分
        for key in ['lateral', 'longitudinal', 'heading']:
            error_key = f'{key}_error'
            if error_key in tracking_error:
                error = tracking_error[error_key]
                threshold = self._tracking_thresholds.get(key, 1.0)
                if threshold > 0:
                    # 归一化评分: 误差为0时100分，误差>=阈值时0分
                    score = max(0.0, 1.0 - error / threshold) * 100
                else:
                    score = 100.0 if error == 0 else 0.0
                scores[key] = score
            else:
                scores[key] = 100.0  # 无数据时默认满分
        
        # 预测误差评分 (可选，不参与加权平均)
        prediction_error = tracking_error.get('prediction_error', float('nan'))
        if np.isfinite(prediction_error):
            threshold = self._tracking_thresholds.get('prediction', 0.5)
            if threshold > 0:
                scores['prediction'] = max(0.0, 1.0 - prediction_error / threshold) * 100
            else:
                scores['prediction'] = 100.0 if prediction_error == 0 else 0.0
        else:
            scores['prediction'] = float('nan')
        
        # 计算加权总分 (只使用 lateral, longitudinal, heading)
        total_weight = 0.0
        weighted_sum = 0.0
        for key in ['lateral', 'longitudinal', 'heading']:
            weight = self._tracking_weights.get(key, 0.0)
            if key in scores:
                weighted_sum += scores[key] * weight
                total_weight += weight
        
        if total_weight > 0:
            overall_score = weighted_sum / total_weight
        else:
            overall_score = 0.0
        
        # 确定质量等级
        if overall_score >= self._tracking_rating.get('excellent', 90):
            rating = 'excellent'
        elif overall_score >= self._tracking_rating.get('good', 70):
            rating = 'good'
        elif overall_score >= self._tracking_rating.get('fair', 50):
            rating = 'fair'
        else:
            rating = 'poor'
        
        return {
            'scores': scores,
            'overall_score': overall_score,
            'rating': rating,
            'thresholds': self._tracking_thresholds.copy(),
            'weights': self._tracking_weights.copy(),
        }

    def _build_diagnostics(self, consistency: ConsistencyResult,
                          mpc_health: Any, mpc_cmd: Optional[ControlOutput],
                          timeout_status: TimeoutStatus, state: np.ndarray,
                          trajectory: Trajectory, tf2_critical: bool) -> DiagnosticsInput:
        """构建诊断信息 (复用对象)"""
        d = self._diagnostics_input
        d.alpha = consistency.alpha
        d.data_valid = consistency.data_valid
        d.mpc_health = mpc_health
        d.mpc_success = mpc_cmd.success if mpc_cmd else False
        d.odom_timeout = timeout_status.odom_timeout
        d.traj_timeout_exceeded = timeout_status.traj_grace_exceeded
        d.v_horizontal = np.sqrt(state[3]**2 + state[4]**2)
        d.vz = state[5]
        d.has_valid_data = len(trajectory.points) > 0
        d.tf2_critical = tf2_critical
        d.safety_failed = self._safety_failed
        d.current_state = self._last_state
        
        self._last_diagnostics = d
        return d

    def _publish_diagnostics(self, state_output: Optional[EstimatorOutput],
                            timeout_status: TimeoutStatus, tf2_critical: bool,
                            cmd: ControlOutput, current_time: float) -> None:
        """发布诊断信息"""
        transform_status = self.coord_transformer.get_status() if self.coord_transformer else {
            'fallback_duration_ms': 0.0,
            'accumulated_drift': 0.0,
            'tf2_available': True,
            'tf2_injected': False,
            'source_frame': self.config.get('transform', {}).get('source_frame', ''),
            'target_frame': self.transform_target_frame,
            'error_message': ''
        }
        # 检查是否有外部紧急停止请求
        emergency_stop = self.state_machine.is_stop_requested() if self.state_machine else False
        
        self._diagnostics_publisher.publish(
            current_time=current_time,
            state=self._last_state,
            cmd=cmd,
            state_output=state_output,
            consistency=self._last_consistency,
            mpc_health=self._last_mpc_health,
            timeout_status=timeout_status,
            transform_status=transform_status,
            tracking_error=self._last_tracking_error,
            transition_progress=self.smooth_transition.get_progress() if self.smooth_transition else 0.0,
            tf2_critical=tf2_critical,
            safety_check_passed=self._safety_check_passed,
            emergency_stop=emergency_stop,
            tracking_quality=self._last_tracking_quality
        )
