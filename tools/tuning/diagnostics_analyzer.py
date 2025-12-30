#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断数据分析器

分析控制器诊断数据，识别性能问题并生成调优建议。

设计原则：
1. 基于统计数据（均值、最大值、95%分位数）做决策
2. 使用置信度机制过滤不确定的建议
3. 按严重程度分类（critical > warning > info）
4. 不建议降低控制频率，而是优化其他参数
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional

# 直接从核心模块导入枚举，确保与控制器定义一致
from universal_controller.core.enums import ControllerState


@dataclass
class AnalysisResult:
    """分析结果"""
    category: str           # 分类: mpc, tracking, timeout, safety, consistency, etc.
    severity: str           # 严重程度: info, warning, critical
    parameter: str          # 相关参数路径 (如 "mpc.weights.position")
    current_value: Any      # 当前值
    suggested_value: Any    # 建议值
    reason: str             # 原因说明
    confidence: float       # 置信度 [0, 1]


@dataclass
class DiagnosticsStats:
    """诊断统计数据"""
    # MPC 相关
    mpc_solve_times: List[float] = field(default_factory=list)
    mpc_success_count: int = 0
    mpc_fail_count: int = 0
    kkt_residuals: List[float] = field(default_factory=list)
    condition_numbers: List[float] = field(default_factory=list)
    mpc_degradation_warnings: int = 0
    
    # 跟踪误差
    lateral_errors: List[float] = field(default_factory=list)
    longitudinal_errors: List[float] = field(default_factory=list)
    heading_errors: List[float] = field(default_factory=list)
    prediction_errors: List[float] = field(default_factory=list)
    
    # 一致性
    alpha_values: List[float] = field(default_factory=list)
    curvature_scores: List[float] = field(default_factory=list)
    velocity_dir_scores: List[float] = field(default_factory=list)
    temporal_scores: List[float] = field(default_factory=list)
    data_valid_count: int = 0
    data_invalid_count: int = 0
    
    # 超时
    odom_ages: List[float] = field(default_factory=list)
    traj_ages: List[float] = field(default_factory=list)
    imu_ages: List[float] = field(default_factory=list)
    odom_timeout_count: int = 0
    traj_timeout_count: int = 0
    traj_grace_exceeded_count: int = 0
    imu_timeout_count: int = 0
    in_startup_grace_count: int = 0
    
    # 控制命令
    cmd_vx: List[float] = field(default_factory=list)
    cmd_vy: List[float] = field(default_factory=list)
    cmd_omega: List[float] = field(default_factory=list)
    
    # 状态统计 (与 ControllerState 枚举对应)
    state_counts: Dict[int, int] = field(default_factory=dict)
    backup_active_count: int = 0
    soft_disabled_count: int = 0
    mpc_degraded_count: int = 0
    stopping_count: int = 0           # STOPPING 状态计数
    stopped_count: int = 0            # STOPPED 状态计数
    
    # EKF/状态估计
    estimator_covariance_norms: List[float] = field(default_factory=list)
    estimator_innovation_norms: List[float] = field(default_factory=list)
    slip_probabilities: List[float] = field(default_factory=list)
    imu_drift_detected_count: int = 0
    
    # 坐标变换
    tf2_fallback_durations: List[float] = field(default_factory=list)
    tf2_fallback_count: int = 0
    accumulated_drifts: List[float] = field(default_factory=list)
    
    # 安全状态
    safety_check_failed_count: int = 0
    emergency_stop_count: int = 0
    consecutive_errors_max: int = 0
    
    # 总样本数
    total_samples: int = 0


class DiagnosticsAnalyzer:
    """诊断数据分析器
    
    分析控制器诊断数据，识别性能问题并生成调优建议。
    """
    
    def __init__(self, current_config: Dict[str, Any]):
        """
        初始化分析器
        
        Args:
            current_config: 当前配置字典 (从 YAML 加载)
        """
        self.config = current_config
        self.stats = DiagnosticsStats()
        self.results: List[AnalysisResult] = []
        
        # 缓存常用配置判断，避免重复计算
        self._alpha_check_disabled: Optional[bool] = None
    
    def _is_alpha_check_disabled(self) -> bool:
        """
        检查 alpha 检查是否被禁用
        
        alpha_disable_thresh 的语义：
        - 当 alpha < alpha_disable_thresh 时，触发 SOFT_DISABLED 状态
        - 当 alpha_disable_thresh <= 0.0 时，由于 alpha 范围是 [0, 1]，
          条件 alpha < 0.0 永远不成立，所以 alpha 检查被有效禁用
        
        Returns:
            True 如果 alpha 检查被禁用
        """
        if self._alpha_check_disabled is None:
            state_machine = self.config.get('safety', {}).get('state_machine', {})
            alpha_disable_thresh = state_machine.get('alpha_disable_thresh', 0.0)
            # alpha_disable_thresh <= 0 表示禁用 alpha 检查
            self._alpha_check_disabled = alpha_disable_thresh <= 0.0
        return self._alpha_check_disabled

    def add_sample(self, diagnostics: Dict[str, Any]):
        """添加一个诊断样本
        
        数据格式严格遵循 DiagnosticsV2.to_ros_msg() 的输出格式:
            - state: 控制器状态 (int)
            - mpc_success: MPC 是否成功
            - mpc_solve_time_ms: MPC 求解时间
            - backup_active: 备用控制器是否激活
            - mpc_health: {kkt_residual, condition_number, consecutive_near_timeout, 
                          degradation_warning, can_recover}
            - consistency: {curvature, velocity_dir, temporal, alpha_soft, data_valid}
            - estimator_health: {covariance_norm, innovation_norm, slip_probability, 
                                imu_drift_detected, imu_bias, imu_available}
            - tracking: {lateral_error, longitudinal_error, heading_error, prediction_error}
            - transform: {tf2_available, tf2_injected, fallback_duration_ms, accumulated_drift}
            - timeout: {odom_timeout, traj_timeout, traj_grace_exceeded, imu_timeout,
                       last_odom_age_ms, last_traj_age_ms, last_imu_age_ms, in_startup_grace}
            - cmd: {vx, vy, vz, omega, frame_id}
            - transition_progress: 过渡进度
            - safety_check_passed: 安全检查是否通过
            - emergency_stop: 是否紧急停止
            - consecutive_errors: 连续错误计数
        """
        self.stats.total_samples += 1
        
        # MPC 数据
        if 'mpc_solve_time_ms' in diagnostics:
            self.stats.mpc_solve_times.append(diagnostics['mpc_solve_time_ms'])
        
        if diagnostics.get('mpc_success', False):
            self.stats.mpc_success_count += 1
        else:
            self.stats.mpc_fail_count += 1
            
        mpc_health = diagnostics.get('mpc_health', {})
        if isinstance(mpc_health, dict):
            if 'kkt_residual' in mpc_health:
                self.stats.kkt_residuals.append(mpc_health['kkt_residual'])
            if 'condition_number' in mpc_health:
                self.stats.condition_numbers.append(mpc_health['condition_number'])
            if mpc_health.get('degradation_warning', False):
                self.stats.mpc_degradation_warnings += 1
        
        # 跟踪误差
        # 注意: 所有误差值现在都是绝对值
        # prediction_error 为 NaN 表示无预测数据（如使用 fallback 求解器时）
        tracking = diagnostics.get('tracking', {})
        if isinstance(tracking, dict):
            if 'lateral_error' in tracking:
                val = tracking['lateral_error']
                if np.isfinite(val):
                    self.stats.lateral_errors.append(val)
            if 'longitudinal_error' in tracking:
                val = tracking['longitudinal_error']
                if np.isfinite(val):
                    self.stats.longitudinal_errors.append(val)
            if 'heading_error' in tracking:
                val = tracking['heading_error']
                if np.isfinite(val):
                    self.stats.heading_errors.append(val)
            if 'prediction_error' in tracking:
                val = tracking['prediction_error']
                # NaN 表示无预测数据，不加入统计
                if np.isfinite(val):
                    self.stats.prediction_errors.append(val)
        
        # 一致性
        consistency = diagnostics.get('consistency', {})
        if isinstance(consistency, dict):
            if 'alpha_soft' in consistency:
                self.stats.alpha_values.append(consistency['alpha_soft'])
            if 'curvature' in consistency:
                self.stats.curvature_scores.append(consistency['curvature'])
            if 'velocity_dir' in consistency:
                self.stats.velocity_dir_scores.append(consistency['velocity_dir'])
            if 'temporal' in consistency:
                self.stats.temporal_scores.append(consistency['temporal'])
            if consistency.get('data_valid', True):
                self.stats.data_valid_count += 1
            else:
                self.stats.data_invalid_count += 1
        
        # 超时状态
        timeout = diagnostics.get('timeout', {})
        if isinstance(timeout, dict):
            if 'last_odom_age_ms' in timeout:
                self.stats.odom_ages.append(timeout['last_odom_age_ms'])
            if 'last_traj_age_ms' in timeout:
                self.stats.traj_ages.append(timeout['last_traj_age_ms'])
            if 'last_imu_age_ms' in timeout:
                self.stats.imu_ages.append(timeout['last_imu_age_ms'])
            if timeout.get('odom_timeout', False):
                self.stats.odom_timeout_count += 1
            if timeout.get('traj_timeout', False):
                self.stats.traj_timeout_count += 1
            if timeout.get('traj_grace_exceeded', False):
                self.stats.traj_grace_exceeded_count += 1
            if timeout.get('imu_timeout', False):
                self.stats.imu_timeout_count += 1
            if timeout.get('in_startup_grace', False):
                self.stats.in_startup_grace_count += 1
        
        # 控制命令
        cmd = diagnostics.get('cmd', {})
        if isinstance(cmd, dict):
            if 'vx' in cmd:
                self.stats.cmd_vx.append(cmd['vx'])
            if 'vy' in cmd:
                self.stats.cmd_vy.append(cmd['vy'])
            if 'omega' in cmd:
                self.stats.cmd_omega.append(cmd['omega'])
        
        # 状态统计
        state = diagnostics.get('state', 0)
        self.stats.state_counts[state] = self.stats.state_counts.get(state, 0) + 1
        
        if diagnostics.get('backup_active', False):
            self.stats.backup_active_count += 1
        
        # 按状态类型分别统计
        if state == ControllerState.SOFT_DISABLED:
            self.stats.soft_disabled_count += 1
        elif state == ControllerState.MPC_DEGRADED:
            self.stats.mpc_degraded_count += 1
        elif state == ControllerState.STOPPING:
            self.stats.stopping_count += 1
        elif state == ControllerState.STOPPED:
            self.stats.stopped_count += 1
        
        # EKF/状态估计
        estimator = diagnostics.get('estimator_health', {})
        if isinstance(estimator, dict):
            if 'covariance_norm' in estimator:
                self.stats.estimator_covariance_norms.append(estimator['covariance_norm'])
            if 'innovation_norm' in estimator:
                self.stats.estimator_innovation_norms.append(estimator['innovation_norm'])
            if 'slip_probability' in estimator:
                self.stats.slip_probabilities.append(estimator['slip_probability'])
            if estimator.get('imu_drift_detected', False):
                self.stats.imu_drift_detected_count += 1
        
        # 坐标变换
        transform = diagnostics.get('transform', {})
        if isinstance(transform, dict):
            fallback_ms = transform.get('fallback_duration_ms', 0)
            if fallback_ms > 0:
                self.stats.tf2_fallback_durations.append(fallback_ms)
                self.stats.tf2_fallback_count += 1
            if 'accumulated_drift' in transform:
                self.stats.accumulated_drifts.append(transform['accumulated_drift'])
        
        # 安全状态 (顶层字段)
        if not diagnostics.get('safety_check_passed', True):
            self.stats.safety_check_failed_count += 1
        if diagnostics.get('emergency_stop', False):
            self.stats.emergency_stop_count += 1
        
        # 连续错误计数
        # 注意: consecutive_errors 是 ROS 节点层的概念，由 controller_ros/node/base_node.py 维护
        # 在 ROS 环境下，此字段由 ROS 层添加到诊断消息中
        # 在非 ROS 环境下（如直接使用 ControllerManager），此字段不存在，默认为 0
        consecutive_errors = diagnostics.get('consecutive_errors', 0)
        if consecutive_errors > self.stats.consecutive_errors_max:
            self.stats.consecutive_errors_max = consecutive_errors

    def analyze(self) -> List[AnalysisResult]:
        """执行完整分析
        
        Returns:
            分析结果列表
        """
        self.results = []
        
        if self.stats.total_samples < 10:
            self.results.append(AnalysisResult(
                category="general",
                severity="warning",
                parameter="sample_count",
                current_value=self.stats.total_samples,
                suggested_value=100,
                reason="样本数量不足，建议收集更多数据以获得准确分析",
                confidence=0.5
            ))
            return self.results
        
        # 执行各项分析（按优先级排序）
        self._analyze_config_consistency()      # 配置一致性检查（最重要）
        self._analyze_mpc_performance()         # MPC 性能分析
        self._analyze_tracking_errors()         # 跟踪误差分析
        self._analyze_timeout_config()          # 超时配置分析
        self._analyze_safety_config()           # 安全配置分析
        self._analyze_state_machine()           # 状态机分析
        self._analyze_control_smoothness()      # 控制平滑度分析
        self._analyze_consistency()             # 一致性分析
        self._analyze_backup_controller()       # 备份控制器分析
        self._analyze_ekf_estimator()           # EKF 分析
        self._analyze_transform()               # 坐标变换分析
        self._analyze_trajectory_config()       # 轨迹配置分析
        
        return self.results

    def _analyze_config_consistency(self):
        """分析配置一致性 - 检查关键配置之间的逻辑关系"""
        mpc_config = self.config.get('mpc', {})
        trajectory_config = self.config.get('trajectory', {})
        system_config = self.config.get('system', {})
        watchdog_config = self.config.get('watchdog', {})
        constraints_config = self.config.get('constraints', {})
        
        # 1. 检查 MPC dt 与轨迹 dt 是否一致（关键）
        mpc_dt = mpc_config.get('dt', 0.1)
        traj_dt = trajectory_config.get('default_dt_sec', 0.1)
        
        if abs(mpc_dt - traj_dt) > 0.001:
            self.results.append(AnalysisResult(
                category="config",
                severity="critical",
                parameter="mpc.dt",
                current_value=mpc_dt,
                suggested_value=traj_dt,
                reason=f"MPC时间步长({mpc_dt}s)与轨迹时间步长({traj_dt}s)不一致，会导致跟踪误差",
                confidence=1.0
            ))
        
        # 2. 检查 MPC horizon 配置合理性
        # 注意: DiagnosticsV2 不包含轨迹点数信息，只能基于配置进行静态检查
        mpc_horizon = mpc_config.get('horizon', 7)
        
        # 3. 检查 omega_max 是否为 0（常见错误）
        omega_max = constraints_config.get('omega_max', 1.0)
        if omega_max <= 0:
            self.results.append(AnalysisResult(
                category="config",
                severity="critical",
                parameter="constraints.omega_max",
                current_value=omega_max,
                suggested_value=1.0,
                reason="omega_max 为 0 或负数，机器人无法转向！",
                confidence=1.0
            ))
        
        # 4. 检查控制频率与超时配置的关系
        ctrl_freq = system_config.get('ctrl_freq', 20)
        ctrl_period_ms = 1000.0 / ctrl_freq
        
        odom_timeout = watchdog_config.get('odom_timeout_ms', 500)
        if odom_timeout > 0 and odom_timeout < ctrl_period_ms * 2:
            self.results.append(AnalysisResult(
                category="config",
                severity="warning",
                parameter="watchdog.odom_timeout_ms",
                current_value=odom_timeout,
                suggested_value=int(ctrl_period_ms * 3),
                reason=f"里程计超时({odom_timeout}ms)小于2个控制周期({ctrl_period_ms*2:.0f}ms)，可能误报",
                confidence=0.7
            ))
        
        # 5. 检查 MPC 求解时间与控制周期的关系
        if self.stats.mpc_solve_times:
            solve_arr = np.array(self.stats.mpc_solve_times)
            p95_solve = np.percentile(solve_arr, 95)
            max_solve = np.max(solve_arr)
            
            # 如果求解时间超过控制周期，这是严重问题
            if max_solve > ctrl_period_ms:
                self.results.append(AnalysisResult(
                    category="config",
                    severity="critical",
                    parameter="mpc.horizon",
                    current_value=mpc_horizon,
                    suggested_value=max(mpc_horizon - 2, 3),
                    reason=f"MPC最大求解时间({max_solve:.1f}ms)超过控制周期({ctrl_period_ms:.1f}ms)，需要减小预测时域",
                    confidence=0.9
                ))
            # 如果求解时间占用控制周期 > 50%，建议优化
            elif p95_solve > ctrl_period_ms * 0.5:
                self.results.append(AnalysisResult(
                    category="config",
                    severity="warning",
                    parameter="mpc.horizon",
                    current_value=mpc_horizon,
                    suggested_value=max(mpc_horizon - 1, 3),
                    reason=f"MPC求解时间({p95_solve:.1f}ms)占用控制周期({ctrl_period_ms:.1f}ms)>50%，建议减小预测时域",
                    confidence=0.7
                ))

    def _analyze_mpc_performance(self):
        """分析 MPC 性能"""
        if not self.stats.mpc_solve_times:
            return
        
        solve_times = np.array(self.stats.mpc_solve_times)
        avg_time = np.mean(solve_times)
        max_time = np.max(solve_times)
        p95_time = np.percentile(solve_times, 95)
        
        mpc_config = self.config.get('mpc', {})
        health_config = mpc_config.get('health_monitor', {})
        current_warning = health_config.get('time_warning_thresh_ms', 20)
        current_critical = health_config.get('time_critical_thresh_ms', 40)
        
        # 分析求解时间阈值
        if p95_time > current_warning:
            suggested_warning = round(p95_time * 1.2, 1)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="warning",
                parameter="mpc.health_monitor.time_warning_thresh_ms",
                current_value=current_warning,
                suggested_value=suggested_warning,
                reason=f"95%分位求解时间({p95_time:.1f}ms)超过警告阈值，建议放宽",
                confidence=0.8
            ))
        
        if max_time > current_critical:
            suggested_critical = round(max_time * 1.2, 1)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="critical",
                parameter="mpc.health_monitor.time_critical_thresh_ms",
                current_value=current_critical,
                suggested_value=suggested_critical,
                reason=f"最大求解时间({max_time:.1f}ms)超过临界阈值，可能导致降级",
                confidence=0.9
            ))
        
        # 分析 MPC 成功率
        total = self.stats.mpc_success_count + self.stats.mpc_fail_count
        if total > 0:
            success_rate = self.stats.mpc_success_count / total
            if success_rate < 0.9:
                current_horizon = mpc_config.get('horizon', 7)
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.horizon",
                    current_value=current_horizon,
                    suggested_value=max(current_horizon - 1, 3),
                    reason=f"MPC成功率({success_rate*100:.1f}%)较低，建议减小预测时域",
                    confidence=0.7
                ))
        
        # 分析 KKT 残差
        if self.stats.kkt_residuals:
            kkt_arr = np.array(self.stats.kkt_residuals)
            max_kkt = np.max(kkt_arr)
            current_kkt_thresh = health_config.get('kkt_residual_thresh', 0.001)
            
            if max_kkt > current_kkt_thresh:
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.health_monitor.kkt_residual_thresh",
                    current_value=current_kkt_thresh,
                    suggested_value=round(max_kkt * 1.5, 6),
                    reason=f"KKT残差({max_kkt:.6f})超过阈值，可能影响求解质量",
                    confidence=0.7
                ))

    def _analyze_tracking_errors(self):
        """分析轨迹跟踪误差"""
        tracking_config = self.config.get('tracking', {})
        mpc_config = self.config.get('mpc', {})
        weights = mpc_config.get('weights', {})
        
        # 横向误差分析
        if self.stats.lateral_errors:
            lat_arr = np.array(self.stats.lateral_errors)
            avg_lat = np.mean(lat_arr)
            p95_lat = np.percentile(lat_arr, 95)
            
            current_thresh = tracking_config.get('lateral_thresh', 0.25)
            current_weight = weights.get('position', 15.0)
            
            if avg_lat > current_thresh * 0.5:
                suggested_weight = min(current_weight * 1.3, 30.0)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.position",
                    current_value=current_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均横向误差({avg_lat*100:.1f}cm)较大，建议增加位置权重",
                    confidence=0.8
                ))
            
            # 检查横向误差阈值是否合理
            if p95_lat > current_thresh:
                suggested_thresh = round(p95_lat * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="tracking.lateral_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位横向误差({p95_lat*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)",
                    confidence=0.7
                ))
        
        # 纵向误差分析
        if self.stats.longitudinal_errors:
            lon_arr = np.array(self.stats.longitudinal_errors)
            avg_lon = np.mean(lon_arr)
            p95_lon = np.percentile(lon_arr, 95)
            
            current_thresh = tracking_config.get('longitudinal_thresh', 0.6)
            current_weight = weights.get('velocity', 6.0)
            
            if avg_lon > current_thresh * 0.5:
                suggested_weight = min(current_weight * 1.3, 15.0)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.velocity",
                    current_value=current_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均纵向误差({avg_lon*100:.1f}cm)较大，建议增加速度权重",
                    confidence=0.8
                ))
            
            # 检查纵向误差阈值是否合理
            if p95_lon > current_thresh:
                suggested_thresh = round(p95_lon * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="tracking.longitudinal_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位纵向误差({p95_lon*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)",
                    confidence=0.7
                ))
        
        # 航向误差分析
        if self.stats.heading_errors:
            head_arr = np.array(self.stats.heading_errors)
            avg_head = np.mean(head_arr)
            p95_head = np.percentile(head_arr, 95)
            
            current_thresh = tracking_config.get('heading_thresh', 0.5)
            current_weight = weights.get('heading', 8.0)
            
            if avg_head > current_thresh * 0.5:
                suggested_weight = min(current_weight * 1.2, 15.0)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.heading",
                    current_value=current_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均航向误差({np.degrees(avg_head):.1f}deg)较大，建议增加航向权重",
                    confidence=0.8
                ))
        
        # 预测误差分析
        # 预测误差 = 上一次 MPC 预测的状态与当前实际状态的差异
        # 注意: NaN 值已在 add_sample 中过滤，这里的数据都是有效的 MPC 预测误差
        if self.stats.prediction_errors:
            pred_arr = np.array(self.stats.prediction_errors)
            
            if len(pred_arr) > 10:
                avg_pred = np.mean(pred_arr)
                p95_pred = np.percentile(pred_arr, 95)
                max_pred = np.max(pred_arr)
                
                current_thresh = tracking_config.get('prediction_thresh', 0.5)
                mpc_config = self.config.get('mpc', {})
                mpc_dt = mpc_config.get('dt', 0.1)
                
                # 预测误差过大的可能原因:
                # 1. MPC 时间步长与实际控制周期不匹配
                # 2. 动力学模型不准确
                # 3. 状态估计延迟
                if avg_pred > 0.1:  # 10cm
                    system_config = self.config.get('system', {})
                    ctrl_freq = system_config.get('ctrl_freq', 20)
                    ctrl_period = 1.0 / ctrl_freq
                    
                    # 检查 MPC dt 与控制周期是否匹配
                    if abs(mpc_dt - ctrl_period) > 0.01:
                        self.results.append(AnalysisResult(
                            category="tracking",
                            severity="warning",
                            parameter="mpc.dt",
                            current_value=mpc_dt,
                            suggested_value=round(ctrl_period, 3),
                            reason=f"预测误差大({avg_pred*100:.1f}cm)，MPC时间步长({mpc_dt}s)与控制周期({ctrl_period:.3f}s)不匹配",
                            confidence=0.7
                        ))
                    else:
                        # MPC dt 匹配，可能是模型问题
                        self.results.append(AnalysisResult(
                            category="tracking",
                            severity="info",
                            parameter="mpc.weights.position",
                            current_value=weights.get('position', 10.0),
                            suggested_value=weights.get('position', 10.0),
                            reason=f"预测误差大({avg_pred*100:.1f}cm)，可能是动力学模型不准确或状态估计延迟",
                            confidence=0.5
                        ))
                
                # 检查预测误差阈值是否合理
                if p95_pred > current_thresh:
                    suggested_thresh = round(p95_pred * 1.2, 2)
                    self.results.append(AnalysisResult(
                        category="tracking",
                        severity="info",
                        parameter="tracking.prediction_thresh",
                        current_value=current_thresh,
                        suggested_value=suggested_thresh,
                        reason=f"95%分位预测误差({p95_pred*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)",
                        confidence=0.6
                    ))
        else:
            # 没有预测误差数据，说明主要使用 fallback 求解器
            mpc_config = self.config.get('mpc', {})
            if self.stats.total_samples > 10:
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="mpc.horizon",
                    current_value=mpc_config.get('horizon', 7),
                    suggested_value=mpc_config.get('horizon', 7),
                    reason="无预测误差数据（可能主要使用fallback求解器），无法分析MPC预测质量",
                    confidence=0.3
                ))
        
        # 跟踪质量权重分析
        # tracking.weights 用于 Dashboard 显示的质量评分计算
        self._analyze_tracking_weights(tracking_config)
    
    def _analyze_tracking_weights(self, tracking_config: Dict[str, Any]):
        """分析跟踪质量评分权重配置
        
        tracking.weights 用于计算综合跟踪质量评分，权重总和应为 1.0。
        根据实际误差分布，建议调整权重以更准确反映跟踪质量。
        """
        weights = tracking_config.get('weights', {})
        lateral_weight = weights.get('lateral', 0.4)
        longitudinal_weight = weights.get('longitudinal', 0.4)
        heading_weight = weights.get('heading', 0.2)
        
        # 检查权重总和
        total_weight = lateral_weight + longitudinal_weight + heading_weight
        if abs(total_weight - 1.0) > 0.01:
            self.results.append(AnalysisResult(
                category="tracking",
                severity="warning",
                parameter="tracking.weights",
                current_value={'lateral': lateral_weight, 'longitudinal': longitudinal_weight, 'heading': heading_weight},
                suggested_value={'lateral': 0.4, 'longitudinal': 0.4, 'heading': 0.2},
                reason=f"跟踪权重总和({total_weight:.2f})不等于1.0，会影响质量评分准确性",
                confidence=0.9
            ))
            return  # 权重配置错误，不继续分析
        
        # 根据实际误差分布建议权重调整
        if (self.stats.lateral_errors and self.stats.longitudinal_errors and 
            len(self.stats.lateral_errors) > 10 and len(self.stats.longitudinal_errors) > 10):
            
            lat_arr = np.array(self.stats.lateral_errors)
            lon_arr = np.array(self.stats.longitudinal_errors)
            
            avg_lat = np.mean(lat_arr)
            avg_lon = np.mean(lon_arr)
            
            # 获取阈值
            lat_thresh = tracking_config.get('lateral_thresh', 0.25)
            lon_thresh = tracking_config.get('longitudinal_thresh', 0.6)
            
            # 计算归一化误差（相对于阈值的比例）
            norm_lat = avg_lat / lat_thresh if lat_thresh > 0 else 0
            norm_lon = avg_lon / lon_thresh if lon_thresh > 0 else 0
            
            # 如果某个方向的归一化误差明显更大，建议增加该方向的权重
            if norm_lat > norm_lon * 1.5 and lateral_weight < 0.5:
                suggested_lat = min(lateral_weight + 0.1, 0.5)
                suggested_lon = max(longitudinal_weight - 0.1, 0.3)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.weights.lateral",
                    current_value=lateral_weight,
                    suggested_value=round(suggested_lat, 2),
                    reason=f"横向误差({avg_lat*100:.1f}cm)相对更大，建议增加横向权重以更准确反映跟踪质量",
                    confidence=0.5
                ))
            elif norm_lon > norm_lat * 1.5 and longitudinal_weight < 0.5:
                suggested_lon = min(longitudinal_weight + 0.1, 0.5)
                suggested_lat = max(lateral_weight - 0.1, 0.3)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.weights.longitudinal",
                    current_value=longitudinal_weight,
                    suggested_value=round(suggested_lon, 2),
                    reason=f"纵向误差({avg_lon*100:.1f}cm)相对更大，建议增加纵向权重以更准确反映跟踪质量",
                    confidence=0.5
                ))

    def _analyze_timeout_config(self):
        """分析超时配置"""
        watchdog_config = self.config.get('watchdog', {})
        
        # 里程计超时分析
        if self.stats.odom_ages:
            odom_arr = np.array(self.stats.odom_ages)
            p95_odom = np.percentile(odom_arr, 95)
            max_odom = np.max(odom_arr)
            
            current_timeout = watchdog_config.get('odom_timeout_ms', 500)
            
            if self.stats.odom_timeout_count > self.stats.total_samples * 0.05:
                timeout_rate = self.stats.odom_timeout_count / self.stats.total_samples * 100
                suggested_timeout = int(max(max_odom * 1.5, current_timeout * 1.5))
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="critical",
                    parameter="watchdog.odom_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"里程计超时率({timeout_rate:.1f}%)过高，建议放宽超时",
                    confidence=0.9
                ))
            elif p95_odom > current_timeout * 0.8:
                suggested_timeout = int(p95_odom * 1.3)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="warning",
                    parameter="watchdog.odom_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"95%分位里程计延迟({p95_odom:.0f}ms)接近超时阈值",
                    confidence=0.8
                ))
        
        # 轨迹超时分析
        if self.stats.traj_ages:
            traj_arr = np.array(self.stats.traj_ages)
            p95_traj = np.percentile(traj_arr, 95)
            
            current_timeout = watchdog_config.get('traj_timeout_ms', 1000)
            current_grace = watchdog_config.get('traj_grace_ms', 600)
            
            if p95_traj > current_timeout * 0.8:
                suggested_timeout = int(p95_traj * 1.3)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="warning",
                    parameter="watchdog.traj_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"95%分位轨迹延迟({p95_traj:.0f}ms)接近超时阈值",
                    confidence=0.8
                ))
            
            # 宽限期超时
            if self.stats.traj_grace_exceeded_count > 0:
                grace_rate = self.stats.traj_grace_exceeded_count / self.stats.total_samples * 100
                suggested_grace = int(current_grace * 1.5)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="critical",
                    parameter="watchdog.traj_grace_ms",
                    current_value=current_grace,
                    suggested_value=suggested_grace,
                    reason=f"轨迹宽限期超时({grace_rate:.1f}%)，可能导致安全停止",
                    confidence=0.9
                ))

    def _analyze_safety_config(self):
        """分析安全配置"""
        constraints_config = self.config.get('constraints', {})
        
        # 速度约束分析
        if self.stats.cmd_vx:
            vx_arr = np.array(self.stats.cmd_vx)
            current_v_max = constraints_config.get('v_max', 0.5)
            
            # 检查是否经常达到速度上限
            at_limit_count = np.sum(np.abs(vx_arr) > current_v_max * 0.95)
            at_limit_rate = at_limit_count / len(vx_arr)
            
            if at_limit_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="safety",
                    severity="info",
                    parameter="constraints.v_max",
                    current_value=current_v_max,
                    suggested_value=min(current_v_max * 1.2, 1.0),
                    reason=f"速度经常({at_limit_rate*100:.1f}%)达到上限，可考虑提高",
                    confidence=0.5
                ))
        
        # 角速度约束分析
        if self.stats.cmd_omega:
            omega_arr = np.array(self.stats.cmd_omega)
            current_omega_max = constraints_config.get('omega_max', 1.0)
            
            if current_omega_max > 0:
                at_limit_count = np.sum(np.abs(omega_arr) > current_omega_max * 0.95)
                at_limit_rate = at_limit_count / len(omega_arr)
                
                if at_limit_rate > 0.1:
                    self.results.append(AnalysisResult(
                        category="safety",
                        severity="info",
                        parameter="constraints.omega_max",
                        current_value=current_omega_max,
                        suggested_value=min(current_omega_max * 1.2, 2.0),
                        reason=f"角速度经常({at_limit_rate*100:.1f}%)达到上限",
                        confidence=0.5
                    ))

    def _analyze_state_machine(self):
        """分析状态机配置
        
        分析内容:
        1. MPC 降级频率与成功率的关系
        2. 备用控制器激活率与恢复条件
        3. Alpha 禁用阈值配置
        4. 状态超时配置
        5. MPC 失败检测窗口配置
        """
        safety_config = self.config.get('safety', {})
        state_machine = safety_config.get('state_machine', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 分析 MPC 降级频率
        degraded_rate = self.stats.mpc_degraded_count / self.stats.total_samples
        backup_rate = self.stats.backup_active_count / self.stats.total_samples
        
        total_mpc = self.stats.mpc_success_count + self.stats.mpc_fail_count
        mpc_success_rate = self.stats.mpc_success_count / total_mpc if total_mpc > 0 else 0
        
        # 1. 如果 MPC 成功率高但降级频繁，说明阈值太敏感
        if degraded_rate > 0.1 and mpc_success_rate > 0.8:
            current_fail_thresh = state_machine.get('mpc_fail_thresh', 3)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine.mpc_fail_thresh",
                current_value=current_fail_thresh,
                suggested_value=min(current_fail_thresh + 2, 10),
                reason=f"MPC成功率({mpc_success_rate*100:.1f}%)高但降级频繁({degraded_rate*100:.1f}%)，阈值过敏感",
                confidence=0.8
            ))
        
        # 2. 如果备用控制器激活率过高，可能恢复条件太严格
        if backup_rate > 0.15:
            current_success_ratio = state_machine.get('mpc_recovery_success_ratio', 0.8)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine.mpc_recovery_success_ratio",
                current_value=current_success_ratio,
                suggested_value=max(current_success_ratio - 0.1, 0.6),
                reason=f"备用控制器激活率({backup_rate*100:.1f}%)过高，可能恢复条件太严格",
                confidence=0.7
            ))
        
        # 3. Alpha 禁用阈值分析
        # 仅当 alpha 检查未被禁用时才分析
        if self.stats.alpha_values and not self._is_alpha_check_disabled():
            alpha_arr = np.array(self.stats.alpha_values)
            avg_alpha = np.mean(alpha_arr)
            alpha_disable_thresh = state_machine.get('alpha_disable_thresh', 0.0)
            soft_disabled_rate = self.stats.soft_disabled_count / self.stats.total_samples
            
            # 如果 alpha 普遍低于 0.3 且 SOFT_DISABLED 频繁，建议禁用 alpha 检查
            if avg_alpha < 0.3 and soft_disabled_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.alpha_disable_thresh",
                    current_value=alpha_disable_thresh,
                    suggested_value=0.0,
                    reason=f"平均alpha({avg_alpha:.2f})较低且SOFT_DISABLED频繁({soft_disabled_rate*100:.1f}%)，建议禁用alpha检查",
                    confidence=0.7
                ))
        
        # 4. MPC 失败检测窗口分析
        # 如果 MPC 失败率波动大，可能需要调整窗口大小
        if total_mpc > 20:
            mpc_fail_window = state_machine.get('mpc_fail_window_size', 10)
            mpc_fail_ratio_thresh = state_machine.get('mpc_fail_ratio_thresh', 0.5)
            
            # 计算实际失败率
            actual_fail_rate = self.stats.mpc_fail_count / total_mpc if total_mpc > 0 else 0
            
            # 如果实际失败率接近阈值但未触发降级，可能窗口太大
            if 0.3 < actual_fail_rate < mpc_fail_ratio_thresh and degraded_rate < 0.05:
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.state_machine.mpc_fail_window_size",
                    current_value=mpc_fail_window,
                    suggested_value=max(mpc_fail_window - 2, 5),
                    reason=f"MPC失败率({actual_fail_rate*100:.1f}%)接近阈值但降级少，窗口可能过大",
                    confidence=0.5
                ))
        
        # 5. STOPPING/STOPPED 状态分析
        # 如果频繁进入停止状态，可能是超时配置或安全配置问题
        stopping_rate = (self.stats.stopping_count + self.stats.stopped_count) / self.stats.total_samples
        if stopping_rate > 0.1:
            # 检查是否与轨迹超时相关
            traj_grace_rate = self.stats.traj_grace_exceeded_count / self.stats.total_samples
            if traj_grace_rate > 0.05:
                watchdog_config = self.config.get('watchdog', {})
                current_grace = watchdog_config.get('traj_grace_ms', 500)
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="watchdog.traj_grace_ms",
                    current_value=current_grace,
                    suggested_value=int(current_grace * 1.5),
                    reason=f"停止状态频繁({stopping_rate*100:.1f}%)，与轨迹宽限期超时({traj_grace_rate*100:.1f}%)相关",
                    confidence=0.8
                ))
            else:
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.stopping_timeout",
                    current_value=self.config.get('safety', {}).get('stopping_timeout', 5.0),
                    suggested_value=self.config.get('safety', {}).get('stopping_timeout', 5.0),
                    reason=f"停止状态频繁({stopping_rate*100:.1f}%)，请检查轨迹输入或安全配置",
                    confidence=0.5
                ))

    def _analyze_control_smoothness(self):
        """分析控制平滑度"""
        mpc_config = self.config.get('mpc', {})
        weights = mpc_config.get('weights', {})
        system_config = self.config.get('system', {})
        ctrl_freq = system_config.get('ctrl_freq', 20)
        dt = 1.0 / ctrl_freq
        
        # 分析速度变化率
        if len(self.stats.cmd_vx) > 10:
            vx_arr = np.array(self.stats.cmd_vx)
            vx_diff = np.diff(vx_arr)
            vx_jerk = np.std(vx_diff)
            
            current_accel_weight = weights.get('control_accel', 0.1)
            
            if vx_jerk > 0.1:
                suggested_weight = min(current_accel_weight * 1.5, 0.5)
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="info",
                    parameter="mpc.weights.control_accel",
                    current_value=current_accel_weight,
                    suggested_value=round(suggested_weight, 2),
                    reason=f"速度变化较剧烈(std={vx_jerk:.3f})，建议增加控制权重以平滑",
                    confidence=0.6
                ))
        
        # 分析角速度变化率
        if len(self.stats.cmd_omega) > 10:
            omega_arr = np.array(self.stats.cmd_omega)
            omega_diff = np.diff(omega_arr)
            omega_jerk = np.std(omega_diff)
            
            current_alpha_weight = weights.get('control_alpha', 0.1)
            
            if omega_jerk > 0.2:
                suggested_weight = min(current_alpha_weight * 1.5, 0.5)
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="info",
                    parameter="mpc.weights.control_alpha",
                    current_value=current_alpha_weight,
                    suggested_value=round(suggested_weight, 2),
                    reason=f"角速度变化较剧烈(std={omega_jerk:.3f})，建议增加控制权重",
                    confidence=0.6
                ))

    def _analyze_consistency(self):
        """分析一致性配置
        
        注意: 当 safety.state_machine.alpha_disable_thresh <= 0.0 时，
        alpha 检查被禁用，此时不应给出 alpha 相关的调优建议。
        """
        consistency_config = self.config.get('consistency', {})
        
        # Alpha 值分析 (仅当 alpha 检查未被禁用时)
        if self.stats.alpha_values and not self._is_alpha_check_disabled():
            alpha_arr = np.array(self.stats.alpha_values)
            avg_alpha = np.mean(alpha_arr)
            min_alpha = np.min(alpha_arr)
            
            current_alpha_min = consistency_config.get('alpha_min', 0.1)
            
            if avg_alpha < 0.3:
                self.results.append(AnalysisResult(
                    category="consistency",
                    severity="info",
                    parameter="consistency.alpha_min",
                    current_value=current_alpha_min,
                    suggested_value=max(min_alpha * 0.8, 0.05),
                    reason=f"平均alpha值({avg_alpha:.2f})较低，Soft Head信任度不高",
                    confidence=0.6
                ))
        
        # 曲率一致性
        if self.stats.curvature_scores:
            kappa_arr = np.array(self.stats.curvature_scores)
            avg_kappa = np.mean(kappa_arr)
            
            current_thresh = consistency_config.get('kappa_thresh', 0.5)
            
            if avg_kappa < 0.5:
                suggested_thresh = max(avg_kappa * 0.8, 0.2)
                self.results.append(AnalysisResult(
                    category="consistency",
                    severity="info",
                    parameter="consistency.kappa_thresh",
                    current_value=current_thresh,
                    suggested_value=round(suggested_thresh, 2),
                    reason=f"曲率一致性得分({avg_kappa:.2f})较低，建议调整阈值",
                    confidence=0.6
                ))

    def _analyze_backup_controller(self):
        """分析备份控制器配置"""
        backup_config = self.config.get('backup', {})
        constraints_config = self.config.get('constraints', {})
        
        if self.stats.total_samples == 0:
            return
        
        backup_rate = self.stats.backup_active_count / self.stats.total_samples
        
        # 只有当备用控制器激活频繁时才分析
        if backup_rate > 0.05:
            v_max = constraints_config.get('v_max', 0.5)
            
            # 检查前瞻距离
            lookahead = backup_config.get('lookahead_dist', 0.5)
            suggested_lookahead = v_max * 1.0  # 约1秒的前瞻
            
            if lookahead < suggested_lookahead * 0.5:
                self.results.append(AnalysisResult(
                    category="backup",
                    severity="info",
                    parameter="backup.lookahead_dist",
                    current_value=lookahead,
                    suggested_value=round(suggested_lookahead, 2),
                    reason=f"备用控制器激活率({backup_rate*100:.1f}%)较高，前瞻距离可能过小",
                    confidence=0.5
                ))
            
            # 检查航向增益
            if self.stats.heading_errors:
                avg_heading_error = np.mean(self.stats.heading_errors)
                kp_heading = backup_config.get('kp_heading', 2.0)
                
                if avg_heading_error > 0.3:  # > 17度
                    suggested_kp = min(kp_heading * 1.2, 3.0)
                    self.results.append(AnalysisResult(
                        category="backup",
                        severity="info",
                        parameter="backup.kp_heading",
                        current_value=kp_heading,
                        suggested_value=round(suggested_kp, 2),
                        reason=f"航向误差({np.degrees(avg_heading_error):.1f}deg)较大，可增加航向增益",
                        confidence=0.5
                    ))

    def _analyze_ekf_estimator(self):
        """分析 EKF 状态估计器
        
        分析内容:
        1. 协方差范数与爆炸阈值
        2. 新息范数异常检测
        3. 打滑检测频率
        4. IMU 漂移检测频率
        5. 过程噪声配置
        """
        ekf_config = self.config.get('ekf', {})
        anomaly_config = ekf_config.get('anomaly_detection', {})
        
        # 1. 分析协方差
        if self.stats.estimator_covariance_norms:
            cov_arr = np.array(self.stats.estimator_covariance_norms)
            max_cov = np.max(cov_arr)
            avg_cov = np.mean(cov_arr)
            
            cov_thresh = anomaly_config.get('covariance_explosion_thresh', 1000.0)
            
            if max_cov > cov_thresh * 0.5:
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="warning",
                    parameter="ekf.anomaly_detection.covariance_explosion_thresh",
                    current_value=cov_thresh,
                    suggested_value=max(max_cov * 2, cov_thresh),
                    reason=f"协方差范数({max_cov:.1f})接近爆炸阈值，可能需要调整",
                    confidence=0.6
                ))
        
        # 2. 分析新息范数 (innovation_norm)
        # 新息范数过大表示测量与预测不一致，可能是传感器问题或模型不准确
        if self.stats.estimator_innovation_norms:
            innov_arr = np.array(self.stats.estimator_innovation_norms)
            max_innov = np.max(innov_arr)
            avg_innov = np.mean(innov_arr)
            p95_innov = np.percentile(innov_arr, 95)
            
            innov_thresh = anomaly_config.get('innovation_anomaly_thresh', 10.0)
            
            # 如果 95% 分位数超过阈值的一半，说明新息经常较大
            if p95_innov > innov_thresh * 0.5:
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="warning",
                    parameter="ekf.anomaly_detection.innovation_anomaly_thresh",
                    current_value=innov_thresh,
                    suggested_value=round(p95_innov * 2.5, 1),
                    reason=f"新息范数95%分位({p95_innov:.2f})较大，可能传感器噪声高或模型不准确",
                    confidence=0.6
                ))
            
            # 如果新息范数波动很大，可能需要调整测量噪声
            innov_std = np.std(innov_arr)
            if innov_std > avg_innov * 0.5 and avg_innov > 1.0:
                measurement_noise = ekf_config.get('measurement_noise', {})
                current_odom_noise = measurement_noise.get('odom_position', 0.01)
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="info",
                    parameter="ekf.measurement_noise.odom_position",
                    current_value=current_odom_noise,
                    suggested_value=round(current_odom_noise * 1.5, 4),
                    reason=f"新息范数波动大(std={innov_std:.2f})，可增加测量噪声",
                    confidence=0.5
                ))
        
        # 3. 分析打滑检测
        if self.stats.slip_probabilities:
            slip_arr = np.array(self.stats.slip_probabilities)
            high_slip_count = np.sum(slip_arr > 0.5)
            high_slip_rate = high_slip_count / len(slip_arr)
            
            if high_slip_rate > 0.1:
                adaptive_config = ekf_config.get('adaptive', {})
                current_thresh = adaptive_config.get('base_slip_thresh', 2.0)
                
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="info",
                    parameter="ekf.adaptive.base_slip_thresh",
                    current_value=current_thresh,
                    suggested_value=round(current_thresh * 1.2, 2),
                    reason=f"打滑检测频繁({high_slip_rate*100:.1f}%)，可能阈值过敏感",
                    confidence=0.5
                ))
        
        # 4. 分析 IMU 漂移检测
        if self.stats.total_samples > 0:
            imu_drift_rate = self.stats.imu_drift_detected_count / self.stats.total_samples
            
            if imu_drift_rate > 0.05:  # 超过 5% 的时间检测到漂移
                drift_thresh = anomaly_config.get('drift_thresh', 0.1)
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="warning",
                    parameter="ekf.anomaly_detection.drift_thresh",
                    current_value=drift_thresh,
                    suggested_value=round(drift_thresh * 1.5, 3),
                    reason=f"IMU漂移检测频繁({imu_drift_rate*100:.1f}%)，阈值可能过敏感或IMU质量差",
                    confidence=0.6
                ))
        
        # 5. 分析过程噪声配置 (基于跟踪误差)
        if self.stats.lateral_errors and self.stats.longitudinal_errors:
            lat_arr = np.array(self.stats.lateral_errors)
            lon_arr = np.array(self.stats.longitudinal_errors)
            
            avg_pos_error = np.sqrt(np.mean(lat_arr**2) + np.mean(lon_arr**2))
            process_noise = ekf_config.get('process_noise', {})
            current_pos_noise = process_noise.get('position', 0.001)
            
            if avg_pos_error > 0.1:  # 10cm
                suggested_noise = min(current_pos_noise * 2, 0.01)
                self.results.append(AnalysisResult(
                    category="ekf",
                    severity="info",
                    parameter="ekf.process_noise.position",
                    current_value=current_pos_noise,
                    suggested_value=suggested_noise,
                    reason=f"位置误差({avg_pos_error*100:.1f}cm)较大，可增加过程噪声",
                    confidence=0.4
                ))

    def _analyze_transform(self):
        """分析坐标变换配置"""
        transform_config = self.config.get('transform', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 分析 TF2 降级
        if self.stats.tf2_fallback_count > 0:
            fallback_rate = self.stats.tf2_fallback_count / self.stats.total_samples
            
            if fallback_rate > 0.05 and self.stats.tf2_fallback_durations:
                max_duration = np.max(self.stats.tf2_fallback_durations)
                current_limit = transform_config.get('fallback_duration_limit_ms', 500)
                
                if max_duration > current_limit * 0.8:
                    self.results.append(AnalysisResult(
                        category="transform",
                        severity="warning",
                        parameter="transform.fallback_duration_limit_ms",
                        current_value=current_limit,
                        suggested_value=int(max_duration * 1.5),
                        reason=f"TF2降级频繁({fallback_rate*100:.1f}%)，最大持续{max_duration:.0f}ms",
                        confidence=0.7
                    ))
        
        # 分析漂移
        if self.stats.accumulated_drifts:
            drift_arr = np.array(self.stats.accumulated_drifts)
            max_drift = np.max(drift_arr)
            
            drift_enabled = transform_config.get('drift_estimation_enabled', False)
            
            if max_drift > 0.1 and not drift_enabled:
                self.results.append(AnalysisResult(
                    category="transform",
                    severity="info",
                    parameter="transform.drift_estimation_enabled",
                    current_value=drift_enabled,
                    suggested_value=True,
                    reason=f"累积漂移({max_drift:.3f}m)较大，建议启用漂移估计",
                    confidence=0.5
                ))

    def _analyze_trajectory_config(self):
        """分析轨迹配置
        
        注意: DiagnosticsV2 不包含轨迹时间步长信息，
        只能基于控制命令数据进行间接分析。
        """
        trajectory_config = self.config.get('trajectory', {})
        
        # 分析低速阈值
        if self.stats.cmd_vx:
            vx_arr = np.array(self.stats.cmd_vx)
            abs_vx = np.abs(vx_arr)
            
            current_low_thresh = trajectory_config.get('low_speed_thresh', 0.05)
            
            # 统计低速样本比例
            low_speed_count = np.sum(abs_vx < current_low_thresh)
            low_speed_rate = low_speed_count / len(abs_vx)
            
            # 如果大部分时间都在低速，可能阈值设置过高
            if low_speed_rate > 0.5 and np.sum(abs_vx > 0) > 10:
                p10_vx = np.percentile(abs_vx[abs_vx > 0], 10)
                suggested_thresh = max(p10_vx * 0.8, 0.02)
                
                self.results.append(AnalysisResult(
                    category="trajectory",
                    severity="info",
                    parameter="trajectory.low_speed_thresh",
                    current_value=current_low_thresh,
                    suggested_value=round(suggested_thresh, 3),
                    reason=f"低速样本比例({low_speed_rate*100:.1f}%)过高，阈值可能过大",
                    confidence=0.5
                ))

    def get_summary(self) -> Dict[str, Any]:
        """获取分析摘要
        
        Returns:
            包含各项统计数据的摘要字典
        """
        summary = {
            'total_samples': self.stats.total_samples,
            'mpc': {},
            'tracking': {},
            'timeout': {},
            'control': {},
            'state_machine': {},
            'consistency': {},
            'estimator': {},
            'transform': {}
        }
        
        # MPC 摘要
        if self.stats.mpc_solve_times:
            solve_times = np.array(self.stats.mpc_solve_times)
            total = self.stats.mpc_success_count + self.stats.mpc_fail_count
            summary['mpc'] = {
                'avg_solve_time_ms': float(round(np.mean(solve_times), 2)),
                'max_solve_time_ms': float(round(np.max(solve_times), 2)),
                'p95_solve_time_ms': float(round(np.percentile(solve_times, 95), 2)),
                'success_rate': float(round(self.stats.mpc_success_count / total * 100, 1)) if total > 0 else 0,
                'backup_rate': float(round(self.stats.backup_active_count / self.stats.total_samples * 100, 1)) if self.stats.total_samples > 0 else 0,
                'degradation_warning_count': int(self.stats.mpc_degradation_warnings)
            }
        
        # 跟踪摘要
        if self.stats.lateral_errors:
            summary['tracking']['lateral'] = {
                'avg_cm': float(round(np.mean(self.stats.lateral_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.lateral_errors) * 100, 2)),
                'p95_cm': float(round(np.percentile(self.stats.lateral_errors, 95) * 100, 2))
            }
        if self.stats.longitudinal_errors:
            summary['tracking']['longitudinal'] = {
                'avg_cm': float(round(np.mean(self.stats.longitudinal_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.longitudinal_errors) * 100, 2)),
                'p95_cm': float(round(np.percentile(self.stats.longitudinal_errors, 95) * 100, 2))
            }
        if self.stats.heading_errors:
            summary['tracking']['heading'] = {
                'avg_deg': float(round(np.degrees(np.mean(self.stats.heading_errors)), 2)),
                'max_deg': float(round(np.degrees(np.max(self.stats.heading_errors)), 2))
            }
        if self.stats.prediction_errors:
            summary['tracking']['prediction'] = {
                'avg_cm': float(round(np.mean(self.stats.prediction_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.prediction_errors) * 100, 2))
            }
        
        # 超时摘要
        summary['timeout'] = {
            'odom_timeout_count': int(self.stats.odom_timeout_count),
            'traj_timeout_count': int(self.stats.traj_timeout_count),
            'traj_grace_exceeded_count': int(self.stats.traj_grace_exceeded_count),
            'imu_timeout_count': int(self.stats.imu_timeout_count),
            'startup_grace_samples': int(self.stats.in_startup_grace_count)
        }
        if self.stats.odom_ages:
            summary['timeout']['avg_odom_age_ms'] = float(round(np.mean(self.stats.odom_ages), 1))
            summary['timeout']['max_odom_age_ms'] = float(round(np.max(self.stats.odom_ages), 1))
        if self.stats.traj_ages:
            summary['timeout']['avg_traj_age_ms'] = float(round(np.mean(self.stats.traj_ages), 1))
            summary['timeout']['max_traj_age_ms'] = float(round(np.max(self.stats.traj_ages), 1))
        
        # 控制摘要
        if self.stats.cmd_vx:
            summary['control']['avg_vx'] = float(round(np.mean(np.abs(self.stats.cmd_vx)), 3))
            summary['control']['max_vx'] = float(round(np.max(np.abs(self.stats.cmd_vx)), 3))
        if self.stats.cmd_omega:
            summary['control']['avg_omega'] = float(round(np.mean(np.abs(self.stats.cmd_omega)), 3))
            summary['control']['max_omega'] = float(round(np.max(np.abs(self.stats.cmd_omega)), 3))
        
        # 状态机摘要
        state_dist = {int(k): int(v) for k, v in self.stats.state_counts.items()}
        
        summary['state_machine'] = {
            'state_distribution': state_dist,
            'backup_active_rate': float(round(self.stats.backup_active_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'soft_disabled_rate': float(round(self.stats.soft_disabled_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'mpc_degraded_rate': float(round(self.stats.mpc_degraded_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopping_rate': float(round(self.stats.stopping_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopped_rate': float(round(self.stats.stopped_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0
        }
        
        # 一致性摘要
        if self.stats.alpha_values:
            total_valid = self.stats.data_valid_count + self.stats.data_invalid_count
            summary['consistency'] = {
                'avg_alpha': float(round(np.mean(self.stats.alpha_values), 3)),
                'min_alpha': float(round(np.min(self.stats.alpha_values), 3)),
                'data_valid_rate': float(round(self.stats.data_valid_count / total_valid * 100, 1)) if total_valid > 0 else 100
            }
        
        # 状态估计摘要
        if self.stats.estimator_covariance_norms:
            summary['estimator']['avg_covariance_norm'] = float(round(np.mean(self.stats.estimator_covariance_norms), 2))
            summary['estimator']['max_covariance_norm'] = float(round(np.max(self.stats.estimator_covariance_norms), 2))
        if self.stats.estimator_innovation_norms:
            summary['estimator']['avg_innovation_norm'] = float(round(np.mean(self.stats.estimator_innovation_norms), 2))
            summary['estimator']['max_innovation_norm'] = float(round(np.max(self.stats.estimator_innovation_norms), 2))
        if self.stats.slip_probabilities:
            summary['estimator']['avg_slip_probability'] = float(round(np.mean(self.stats.slip_probabilities), 3))
            summary['estimator']['high_slip_rate'] = float(round(
                np.sum(np.array(self.stats.slip_probabilities) > 0.5) / len(self.stats.slip_probabilities) * 100, 1))
        summary['estimator']['imu_drift_detected_count'] = int(self.stats.imu_drift_detected_count)
        if self.stats.total_samples > 0:
            summary['estimator']['imu_drift_rate'] = float(round(
                self.stats.imu_drift_detected_count / self.stats.total_samples * 100, 2))
        
        # 坐标变换摘要
        summary['transform'] = {
            'tf2_fallback_count': int(self.stats.tf2_fallback_count),
            'tf2_fallback_rate': float(round(self.stats.tf2_fallback_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0
        }
        if self.stats.accumulated_drifts:
            summary['transform']['max_accumulated_drift_m'] = float(round(np.max(self.stats.accumulated_drifts), 4))
        
        # 安全状态摘要
        summary['safety'] = {
            'safety_check_failed_count': int(self.stats.safety_check_failed_count),
            'emergency_stop_count': int(self.stats.emergency_stop_count),
            'consecutive_errors_max': int(self.stats.consecutive_errors_max)
        }
        if self.stats.total_samples > 0:
            summary['safety']['safety_check_failed_rate'] = float(round(
                self.stats.safety_check_failed_count / self.stats.total_samples * 100, 2))
        
        return summary
