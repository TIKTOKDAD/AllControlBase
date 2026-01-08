"""
控制器管理器

负责协调所有控制器组件，执行主控制循环。

坐标系说明 (不需要建图/定位):
==================================

    base_link (机体坐标系)              odom (里程计坐标系)
    ┌───────────────┐                   ┌─────────────────────┐
    │       ↑ X     │                   │                     │
    │       │       │    坐标变换        │    机器人轨迹       │
    │    ←──┼──→    │  ───────────→     │    ○──○──○──○       │
    │     Y │       │  base_link→odom   │                     │
    │       ↓       │                   │    启动位置 ●       │
    └───────────────┘                   └─────────────────────┘
    
    - 原点在机器人中心                   - 从启动位置开始累积
    - X轴朝前                            - 会有漂移（正常）
    - 随机器人移动                       - 不需要建图/定位

数据流:
    网络输出轨迹 (base_link, 局部坐标)
        ↓
    坐标变换 (base_link → odom)
        ↓
    控制器计算 (在 odom 坐标系下)
        ↓
    控制输出:
        - 差速车/阿克曼车: base_link (vx, omega)
        - 全向车/四旋翼: odom (vx, vy, omega)

注意:
    - odom 就是你的"世界坐标系"，不需要建图
    - 网络输出的轨迹应该设置 frame_id='base_link'
    - 如果轨迹已经在 odom 坐标系，会跳过变换直接使用
"""
from typing import Dict, Any, Optional
import numpy as np
import logging

# Mixins
from .mixins import (
    LifecycleMixin, DiagnosticsMixin, 
    ControlLogicMixin, DataProcessingMixin
)

from ..core.interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IControlProcessor
)
from ..core.velocity_smoother import VelocitySmoother
from ..core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, 
    TimeoutStatus, Odometry, Imu, AttitudeCommand,
    TrajectoryConfig
)
from ..core.enums import ControllerState, PlatformType
from ..core.diagnostics_input import DiagnosticsInput
from ..config.default_config import PLATFORM_CONFIG
from ..config.utils import deep_update
from ..safety.timeout_monitor import TimeoutMonitor
from ..diagnostics.publisher import DiagnosticsPublisher
from ..health.mpc_health_monitor import MPCHealthMonitor
from ..safety.state_machine import StateMachine

# ConfigValidator used in __init__
from ..config.validation import ConfigValidator

logger = logging.getLogger(__name__)


class ControllerManager(LifecycleMixin, DiagnosticsMixin, 
                       ControlLogicMixin, DataProcessingMixin):
    """
    控制器管理器
    
    负责协调所有控制器组件，执行主控制循环。
    
    线程安全性说明:
        - update() 方法不是线程安全的，应该在单个线程中调用
        - 如果需要在多线程环境中使用，调用者需要自行加锁
        - set_diagnostics_callback() 是线程安全的
    
    配置验证:
        - 默认在初始化时进行配置验证（可通过 validate_config=False 禁用）
        - 验证失败会记录警告但不会阻止初始化
        - 使用 strict_mode=True 可在验证失败时抛出异常
    
    使用示例:
        manager = ControllerManager(config)
        manager.initialize_default_components()
        
        # 在控制循环中调用
        cmd = manager.update(time.time(), odom, trajectory, data_ages)
    """
    
    def __init__(self, config: Dict[str, Any], validate_config: bool = True, 
                 strict_mode: bool = False, profile: Optional[str] = None):
        """
        初始化控制器管理器
        
        Args:
            config: 配置字典（如果提供了 profile，则此参数为基础配置或覆盖项）
            validate_config: 是否在初始化时验证配置（默认 True）
            strict_mode: 严格模式，验证失败时抛出异常（默认 False）
            profile: 配置预设名称 ("balanced", "aggressive", "safe")
        """
        # 加载 Profile 配置
        if profile:
            from ..config.profiles import create_default_config
            profile_config = create_default_config(profile)
            deep_update(profile_config, config) 
            self.config = profile_config
        else:
            self.config = config
        
        # 配置验证
        if validate_config:
            # 1. 结构验证 (Schema Validation)
            ConfigValidator.validate(self.config)
            
            # 2. 逻辑验证 (Logical Validation)
            self._validate_config(strict_mode)
        
        # 加载 TrajectoryConfig
        self.trajectory_config = TrajectoryConfig.from_dict(self.config)

        self.ctrl_freq = self.config.get('system', {}).get('ctrl_freq', 50)
        self.dt = 1.0 / self.ctrl_freq

        platform_name = self.config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential']).copy()
        
        transform_config = self.config.get('transform', {})
        system_config = self.config.get('system', {})
        
        user_output_frame = transform_config.get('output_frame') or system_config.get('output_frame')
        
        if user_output_frame:
            self.platform_config['output_frame'] = user_output_frame
            
        self.default_frame_id = self.platform_config.get('output_frame', 'base_link')
        self.transform_target_frame = transform_config.get('target_frame', 'odom')
        
        self.is_quadrotor = not self.platform_config.get(
            'is_ground_vehicle', 
            self.platform_config.get('type') != PlatformType.QUADROTOR
        )
        
        mpc_config = config.get('mpc', {})
        self.horizon_normal = mpc_config.get('horizon', 20)
        self.horizon_degraded = mpc_config.get('horizon_degraded', 10)
        self._current_horizon = self.horizon_normal
        
        # 组件
        self.state_estimator: Optional[IStateEstimator] = None
        self.mpc_tracker: Optional[ITrajectoryTracker] = None
        self.backup_tracker: Optional[ITrajectoryTracker] = None
        self.consistency_checker: Optional[IConsistencyChecker] = None
        self.state_machine: Optional[StateMachine] = None
        self.smooth_transition: Optional[ISmoothTransition] = None
        self.coord_transformer: Optional[ICoordinateTransformer] = None
        self.safety_monitor: Optional[ISafetyMonitor] = None
        self.mpc_health_monitor: Optional[MPCHealthMonitor] = None
        self.processors: list[IControlProcessor] = []
        self.timeout_monitor = TimeoutMonitor(config)
        
        # 状态
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._safety_check_passed = True
        self._diagnostics_input = DiagnosticsInput()
        self._last_diagnostics: Optional[DiagnosticsInput] = None
        self._last_mpc_health = None
        self._last_consistency: Optional[ConsistencyResult] = None
        self._last_mpc_cmd: Optional[ControlOutput] = None
        self._last_backup_cmd: Optional[ControlOutput] = None
        self._last_tracking_error: Optional[Dict[str, float]] = None
        self._last_tracking_quality: Optional[Dict[str, Any]] = None
        self._last_published_diagnostics: Optional[Dict[str, Any]] = None
        self._last_attitude_cmd: Optional[AttitudeCommand] = None
        self._last_update_time: Optional[float] = None
        
        # 跟踪质量评估配置
        from ..config.system_config import TRACKING_CONFIG
        tracking_config = config.get('tracking', {})
        self._tracking_thresholds = {
            'lateral': tracking_config.get('lateral_thresh', TRACKING_CONFIG['lateral_thresh']),
            'longitudinal': tracking_config.get('longitudinal_thresh', TRACKING_CONFIG['longitudinal_thresh']),
            'heading': tracking_config.get('heading_thresh', TRACKING_CONFIG['heading_thresh']),
            'prediction': tracking_config.get('prediction_thresh', TRACKING_CONFIG['prediction_thresh']),
        }
        self._tracking_weights = tracking_config.get('weights', TRACKING_CONFIG['weights'].copy())
        self._tracking_rating = tracking_config.get('rating', TRACKING_CONFIG['rating'].copy())
        
        # 预测误差计算相关状态
        self._last_mpc_predicted_state: Optional[np.ndarray] = None
        
        # 诊断发布器
        diagnostics_config = self.config.get('diagnostics', {})
        diag_rate = diagnostics_config.get(
            'publish_rate',
            self.config.get('system', {}).get('diagnostics_rate', 10.0)
        )
        self._diagnostics_publisher = DiagnosticsPublisher(publish_rate=diag_rate)
        
        # 处理器故障计数器
        self._processor_failure_counts = {}
        self._processor_max_failures = 10
        
        # 记录上一帧最终发布的命令，用于平滑器
        self._last_final_cmd: Optional[ControlOutput] = None
        
        # 速度平滑器
        self.control_freq = self.config.get('system', {}).get('ctrl_freq', 50)
        self.dt_ctrl = 1.0 / self.control_freq
        
        constraints = self.config.get('constraints', self.platform_config.get('constraints', {}))
        self.a_max = constraints.get('a_max', 1.5)
        self.az_max = constraints.get('az_max', 1.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)
        
        self.velocity_smoother = VelocitySmoother(
            a_max=self.a_max, az_max=self.az_max,
            alpha_max=self.alpha_max, dt=self.dt_ctrl
        )

    def update(self, current_time: float, odom: Odometry, trajectory: Trajectory, 
               data_ages: Dict[str, float], imu: Optional[Imu] = None) -> ControlOutput:
        """
        主控制循环
        
        Args:
            current_time: 当前时间 (秒) - 应使用 ROS Time / Sim Time 以确保 TF 同步
            odom: 里程计数据
            trajectory: 轨迹数据
            data_ages: 数据年龄字典 {'odom': sec, 'trajectory': sec, 'imu': sec}
            imu: IMU 数据 (可选)
            
        Returns:
            ControlOutput: 控制输出
        """
        
        # 1. 时间管理
        actual_dt, skip_prediction = self._compute_time_step(current_time)
        
        # 1.1 同步 MPC Horizon
        self._sync_mpc_horizon()
        
        # 1.5 轨迹验证
        if not trajectory.validate(self.trajectory_config) and len(trajectory.points) > 0:
            logger.error("Trajectory validation failed (e.g. detected jump > max_point_distance). Ignoring unsafe trajectory.")
            trajectory = Trajectory(
                header=trajectory.header,
                points=np.zeros((0, 3), dtype=np.float64),
                velocities=None,
                dt_sec=trajectory.dt_sec,
                confidence=trajectory.confidence,
                mode=trajectory.mode,
                soft_enabled=trajectory.soft_enabled,
                low_speed_thresh=trajectory.low_speed_thresh
            )
        
        # 2. 超时监控
        current_timeout_status = self.timeout_monitor.check(data_ages)
        self._last_timeout_status = current_timeout_status
        
        # 3. 状态估计
        state, state_output = self._update_state_estimation(
            odom, imu, current_timeout_status, actual_dt, current_time, skip_prediction)
        
        # 4. 一致性检查
        consistency = self._compute_consistency(trajectory)
        
        # 5. 坐标变换
        transformed_traj, tf2_critical = self._transform_trajectory(trajectory, current_time)
        
        # 6. MPC 计算和健康监控
        mpc_cmd, mpc_health = self._compute_mpc(state, transformed_traj, consistency)
        
        # 7. 构建诊断信息
        diagnostics = self._build_diagnostics(
            consistency, mpc_health, mpc_cmd, current_timeout_status, 
            state, trajectory, tf2_critical)
        
        # 8. 选择控制器输出
        cmd = self._select_controller_output(
            state, transformed_traj, consistency, mpc_cmd)
        
        # 9. 计算跟踪误差和质量评估
        self._update_tracking_metrics(state, transformed_traj)
        
        # 9.5 速度平滑
        if cmd.success:
             cmd = self.velocity_smoother.smooth(cmd, self._last_final_cmd)
        
        # 10. 安全检查
        cmd = self._apply_safety_check(state, cmd, diagnostics, actual_dt)
        
        # 记录最终命令
        self._last_final_cmd = cmd
        
        # 11. 状态机更新
        self._update_state_machine(diagnostics)
        
        # 12. 平滑过渡
        cmd = self._apply_smooth_transition(cmd, current_time)
        
        # 13. 发布诊断
        self._publish_diagnostics(
            state_output, current_timeout_status, tf2_critical, cmd, current_time)
        
        # 14. 运行额外处理器
        self._run_processors(state, cmd)
        
        return cmd
