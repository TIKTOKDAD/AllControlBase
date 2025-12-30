#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置加载器 - 为诊断脚本提供配置读取功能

此模块允许诊断脚本读取现有的 YAML 配置文件（如 turtlebot1.yaml），
并使用配置中的阈值进行诊断判断，而不是使用硬编码的默认值。

设计原则:
1. 所有诊断阈值都从配置文件读取，确保与控制器使用相同的参数
2. 阈值计算逻辑与 universal_controller 保持一致
3. 警告阈值 = 错误阈值 * WARN_RATIO，保持统一的比例关系
4. 不存在的配置项使用与 turtlebot1.yaml 一致的默认值

使用方法:
    from config_loader import ConfigLoader, load_config_for_diagnostics
    
    # 方式1: 直接加载
    loader = ConfigLoader('turtlebot1.yaml')
    ctrl_freq = loader.get('system.ctrl_freq', default=20)
    
    # 方式2: 获取诊断阈值对象
    loader, thresholds = load_config_for_diagnostics('turtlebot1.yaml')
    print(thresholds.TRACKING_LATERAL_THRESH)

作者: Kiro Auto-generated
版本: 2.0 (统一阈值来源，修复配置读取不完整问题)
"""
import os
import yaml
from typing import Any, Dict, Optional


class ConfigLoader:
    """
    配置加载器
    
    支持从 YAML 文件加载配置，并提供便捷的配置访问接口。
    """
    
    # 默认配置搜索路径
    DEFAULT_CONFIG_PATHS = [
        # 相对于脚本目录
        '../config',
        # 相对于工作目录
        'controller_ros/config',
        'config',
        # ROS 包路径
        '/opt/ros/noetic/share/controller_ros/config',
    ]
    
    def __init__(self, config_path: str = None):
        """
        初始化配置加载器
        
        Args:
            config_path: 配置文件路径（可选）
        """
        self.config: Dict[str, Any] = {}
        self.config_file: Optional[str] = None
        
        if config_path:
            self.load_config(config_path)
    
    def find_config_file(self, filename: str) -> Optional[str]:
        """
        在默认路径中查找配置文件
        
        Args:
            filename: 配置文件名（如 'turtlebot1.yaml'）
        
        Returns:
            找到的配置文件完整路径，或 None
        """
        # 如果是绝对路径，直接检查
        if os.path.isabs(filename):
            return filename if os.path.exists(filename) else None
        
        # 获取脚本所在目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 在默认路径中搜索
        search_paths = []
        for path in self.DEFAULT_CONFIG_PATHS:
            if path.startswith('/'):
                search_paths.append(path)
            else:
                search_paths.append(os.path.join(script_dir, path))
        
        for base_path in search_paths:
            full_path = os.path.join(base_path, filename)
            if os.path.exists(full_path):
                return full_path
        
        return None
    
    def load_config(self, config_path: str) -> Dict[str, Any]:
        """
        加载配置文件
        
        Args:
            config_path: 配置文件路径或文件名
        
        Returns:
            加载的配置字典
        
        Raises:
            FileNotFoundError: 配置文件不存在
            yaml.YAMLError: YAML 解析错误
        """
        # 查找配置文件
        full_path = self.find_config_file(config_path)
        if full_path is None:
            raise FileNotFoundError(f"配置文件未找到: {config_path}")
        
        # 加载 YAML
        with open(full_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f) or {}
        
        self.config_file = full_path
        return self.config
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        获取配置值（支持点分隔的路径）
        
        Args:
            key_path: 配置路径，如 'system.ctrl_freq' 或 'mpc.weights.position'
            default: 默认值
        
        Returns:
            配置值，如果不存在则返回默认值
        
        Example:
            >>> loader.get('system.ctrl_freq', default=20)
            20
            >>> loader.get('mpc.weights.position', default=10.0)
            15.0
        """
        keys = key_path.split('.')
        value = self.config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        
        return value
    
    def get_thresholds(self) -> 'DiagnosticsThresholdsFromConfig':
        """
        获取基于配置的诊断阈值
        
        Returns:
            DiagnosticsThresholdsFromConfig 实例
        """
        return DiagnosticsThresholdsFromConfig(self)


class DiagnosticsThresholdsFromConfig:
    """
    基于配置文件的诊断阈值
    
    此类从配置文件中读取阈值，而不是使用硬编码的默认值。
    这确保诊断脚本使用与控制器相同的阈值进行判断。
    """
    
    # 警告阈值与错误阈值的比例
    WARN_RATIO = 0.67
    
    def __init__(self, loader: ConfigLoader):
        """
        初始化阈值
        
        Args:
            loader: 配置加载器实例
        """
        self._loader = loader
        self._init_thresholds()
    
    def _init_thresholds(self):
        """从配置文件初始化所有阈值"""
        get = self._loader.get
        
        # ===== 系统配置 =====
        self.ctrl_freq = get('system.ctrl_freq', 20)
        self.ctrl_period_ms = 1000.0 / self.ctrl_freq
        
        # ===== MPC 健康监控阈值 =====
        self.MPC_SOLVE_TIME_WARNING_MS = get('mpc.health_monitor.time_warning_thresh_ms', 15.0)
        self.MPC_SOLVE_TIME_CRITICAL_MS = get('mpc.health_monitor.time_critical_thresh_ms', 
                                              self.ctrl_period_ms * 0.8)
        self.MPC_SOLVE_TIME_EXTREME_MS = self.MPC_SOLVE_TIME_CRITICAL_MS * 1.5
        self.MPC_KKT_RESIDUAL_THRESH = get('mpc.health_monitor.kkt_residual_thresh', 1e-3)
        self.MPC_CONDITION_NUMBER_THRESH = get('mpc.health_monitor.condition_number_thresh', 1e8)
        self.MPC_CONSECUTIVE_TIMEOUT_WARN = get('mpc.health_monitor.consecutive_warning_limit', 10)
        
        # ===== 跟踪误差阈值 =====
        self.TRACKING_LATERAL_THRESH = get('tracking.lateral_thresh', 0.3)
        self.TRACKING_LONGITUDINAL_THRESH = get('tracking.longitudinal_thresh', 0.5)
        self.TRACKING_HEADING_THRESH = get('tracking.heading_thresh', 0.5)
        self.TRACKING_PREDICTION_THRESH = get('tracking.prediction_thresh', 0.5)
        
        # 计算警告阈值
        self.TRACKING_LATERAL_WARN = self.TRACKING_LATERAL_THRESH * self.WARN_RATIO
        self.TRACKING_LONGITUDINAL_WARN = self.TRACKING_LONGITUDINAL_THRESH * self.WARN_RATIO
        self.TRACKING_HEADING_WARN_RAD = self.TRACKING_HEADING_THRESH * self.WARN_RATIO
        
        # ===== 状态机阈值 =====
        self.DEGRADED_STATE_TIMEOUT = get('safety.state_machine.degraded_state_timeout', 30.0)
        self.BACKUP_STATE_TIMEOUT = get('safety.state_machine.backup_state_timeout', 60.0)
        self.DEGRADED_STATE_WARN = 10.0
        
        # ===== 一致性检查阈值 =====
        self.ALPHA_CRITICAL = get('safety.state_machine.alpha_disable_thresh', 0.1)
        if self.ALPHA_CRITICAL == 0:
            self.ALPHA_CRITICAL = 0.3  # 如果禁用了 alpha 检查，使用默认值
        self.ALPHA_WARN = 0.5
        self.ALPHA_VERY_LOW = 0.2
        self.CONSISTENCY_LOW_THRESH = get('consistency.kappa_thresh', 0.5)
        self.TEMPORAL_SMOOTH_LOW = get('consistency.temporal_smooth_thresh', 0.3)
        
        # ===== 状态估计器阈值 =====
        self.COVARIANCE_NORM_CRITICAL = get('ekf.anomaly_detection.covariance_explosion_thresh', 1000.0) / 1000
        self.INNOVATION_NORM_WARN = get('ekf.anomaly_detection.innovation_anomaly_thresh', 10.0) / 20
        self.SLIP_PROBABILITY_CRITICAL = 0.5
        self.SLIP_PROBABILITY_WARN = 0.3
        
        # ===== 超时阈值 =====
        self.ODOM_TIMEOUT_MS = get('watchdog.odom_timeout_ms', 500)
        self.TRAJ_TIMEOUT_MS = get('watchdog.traj_timeout_ms', 1000)
        self.ODOM_AGE_WARN_MS = self.ODOM_TIMEOUT_MS * 0.5
        self.TRAJ_AGE_WARN_MS = self.TRAJ_TIMEOUT_MS * 0.5
        
        # ===== 坐标变换阈值 =====
        self.TF2_FALLBACK_WARN_MS = get('transform.fallback_duration_limit_ms', 500) * 0.2
        self.TF2_FALLBACK_CRITICAL_MS = get('transform.fallback_duration_limit_ms', 500)
        self.ACCUMULATED_DRIFT_WARN = get('ekf.anomaly_detection.drift_thresh', 0.1)
        
        # ===== 轨迹配置 =====
        self.LOW_SPEED_THRESH = get('trajectory.low_speed_thresh', 0.1)
        
        # ===== 约束配置 =====
        self.V_MAX = get('constraints.v_max', 0.5)
        self.OMEGA_MAX = get('constraints.omega_max', 1.0)
        self.A_MAX = get('constraints.a_max', 0.5)
        
        # ===== 运行时调优阈值 =====
        self.TUNING_LATERAL_ERROR_HIGH = self.TRACKING_LATERAL_THRESH * 0.5
        self.TUNING_LATERAL_ERROR_MED = self.TRACKING_LATERAL_THRESH * 0.33
        self.TUNING_HEADING_ERROR_HIGH = self.TRACKING_HEADING_THRESH * 0.6
        self.TUNING_HEADING_ERROR_MED = self.TRACKING_HEADING_THRESH * 0.4
        
        # MPC 成功率阈值
        self.MPC_SUCCESS_RATE_CRITICAL = 0.9
        self.MPC_SUCCESS_RATE_WARN = 0.98
        
        # 备用控制器使用率阈值
        self.BACKUP_ACTIVE_RATIO_WARN = 0.1
        
        # 一致性拒绝率阈值
        self.CONSISTENCY_REJECTION_HIGH = 0.1
        self.CONSISTENCY_REJECTION_MED = 0.05
        
        # 控制平滑性阈值
        self.MAX_ACCEL_SMOOTH = self.A_MAX
        self.MAX_ACCEL_JITTER = self.A_MAX * 2.5
        self.MAX_ANGULAR_ACCEL_JITTER = get('constraints.alpha_max', 1.5) * 10
    
    def get_source_frame(self) -> str:
        """获取源坐标系"""
        return self._loader.get('transform.source_frame', 'base_link')
    
    def get_target_frame(self) -> str:
        """获取目标坐标系"""
        return self._loader.get('transform.target_frame', 'odom')
    
    def get_topics(self) -> dict:
        """获取话题配置"""
        return {
            'odom': self._loader.get('topics.odom', '/odom'),
            'imu': self._loader.get('topics.imu', '/imu'),
            'trajectory': self._loader.get('topics.trajectory', '/nn/local_trajectory'),
            'cmd_unified': self._loader.get('topics.cmd_unified', '/cmd_unified'),
            'diagnostics': self._loader.get('topics.diagnostics', '/controller/diagnostics'),
            'cmd_vel': self._loader.get('cmd_vel_adapter.output_topic', '/cmd_vel'),
        }
    
    def summary(self) -> str:
        """生成阈值摘要"""
        lines = [
            "诊断阈值配置 (来自配置文件):",
            f"  配置文件: {self._loader.config_file}",
            "",
            "  系统:",
            f"    控制频率: {self.ctrl_freq} Hz",
            f"    控制周期: {self.ctrl_period_ms:.1f} ms",
            "",
            "  MPC 健康:",
            f"    求解时间警告: {self.MPC_SOLVE_TIME_WARNING_MS} ms",
            f"    求解时间临界: {self.MPC_SOLVE_TIME_CRITICAL_MS:.1f} ms",
            f"    KKT 残差阈值: {self.MPC_KKT_RESIDUAL_THRESH}",
            f"    条件数阈值: {self.MPC_CONDITION_NUMBER_THRESH:.0e}",
            "",
            "  跟踪误差:",
            f"    横向误差阈值: {self.TRACKING_LATERAL_THRESH} m (警告: {self.TRACKING_LATERAL_WARN:.2f} m)",
            f"    纵向误差阈值: {self.TRACKING_LONGITUDINAL_THRESH} m",
            f"    航向误差阈值: {self.TRACKING_HEADING_THRESH} rad",
            "",
            "  超时:",
            f"    里程计超时: {self.ODOM_TIMEOUT_MS} ms",
            f"    轨迹超时: {self.TRAJ_TIMEOUT_MS} ms",
            "",
            "  轨迹:",
            f"    低速阈值: {self.LOW_SPEED_THRESH} m/s",
            "",
            "  约束:",
            f"    最大速度: {self.V_MAX} m/s",
            f"    最大角速度: {self.OMEGA_MAX} rad/s",
            f"    最大加速度: {self.A_MAX} m/s²",
        ]
        return '\n'.join(lines)


def load_config_for_diagnostics(config_file: str = 'turtlebot1.yaml') -> tuple:
    """
    为诊断脚本加载配置
    
    Args:
        config_file: 配置文件名或路径
    
    Returns:
        (ConfigLoader, DiagnosticsThresholdsFromConfig) 元组
    
    Example:
        loader, thresholds = load_config_for_diagnostics('turtlebot1.yaml')
        print(thresholds.summary())
    """
    loader = ConfigLoader()
    try:
        loader.load_config(config_file)
        thresholds = loader.get_thresholds()
        return loader, thresholds
    except FileNotFoundError:
        print(f"警告: 配置文件 {config_file} 未找到，使用默认值")
        return loader, None


if __name__ == '__main__':
    # 测试配置加载
    import sys
    
    config_file = sys.argv[1] if len(sys.argv) > 1 else 'turtlebot1.yaml'
    
    try:
        loader, thresholds = load_config_for_diagnostics(config_file)
        if thresholds:
            print(thresholds.summary())
        else:
            print("使用默认阈值")
    except Exception as e:
        print(f"错误: {e}")
