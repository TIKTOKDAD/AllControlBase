from typing import Dict, Any, Optional
import logging

from ...core.interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IControlProcessor
)
from ...core.enums import ControllerState, PlatformType
from ...core.data_types import TrajectoryConfig, TimeoutStatus
from ...config.validation import validate_logical_consistency, ConfigValidationError, ValidationSeverity
from ...health.mpc_health_monitor import MPCHealthMonitor
from ...estimator.adaptive_ekf import AdaptiveEKFEstimator
from ...tracker.pure_pursuit import PurePursuitController
from ...tracker.mpc_controller import MPCController
from ...consistency.weighted_analyzer import WeightedConsistencyAnalyzer
from ...safety.safety_monitor import BasicSafetyMonitor
from ...transition.smooth_transition import ExponentialSmoothTransition, LinearSmoothTransition
from ...transform.robust_transformer import RobustCoordinateTransformer
from ...processors.attitude_processor import AttitudeProcessor
from ...safety.state_machine import StateMachine

logger = logging.getLogger(__name__)

class LifecycleMixin:
    """
    Mixin for ControllerManager lifecycle management (init, reset, shutdown).
    """

    def _validate_config(self, strict_mode: bool = False) -> None:
        """
        验证配置参数
        
        验证策略:
        - FATAL 级别错误: 始终阻止启动（如 v_max <= 0）
        - ERROR 级别错误: strict_mode=True 时阻止启动，否则只记录警告
        - WARNING 级别: 只记录日志，不阻止启动
        
        Args:
            strict_mode: 严格模式，ERROR 级别错误也会阻止启动
        
        Raises:
            ConfigValidationError: 当存在 FATAL 错误，或 strict_mode=True 且存在 ERROR 错误时
        """
        errors = validate_logical_consistency(self.config)
        
        if not errors:
            return
        
        # 按严重级别分类
        fatal_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.FATAL]
        error_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.ERROR]
        warning_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.WARNING]
        
        # 记录所有警告
        for key, msg, _ in warning_errors:
            logger.warning(f'配置警告 [{key}]: {msg}')
        
        # FATAL 错误始终阻止启动
        if fatal_errors:
            fatal_msgs = '\n'.join([f'  - [FATAL] {key}: {msg}' for key, msg, _ in fatal_errors])
            raise ConfigValidationError(f'配置存在致命错误，无法启动:\n{fatal_msgs}')
        
        # ERROR 错误根据 strict_mode 决定
        if error_errors:
            if strict_mode:
                error_msgs = '\n'.join([f'  - [ERROR] {key}: {msg}' for key, msg, _ in error_errors])
                raise ConfigValidationError(f'配置验证失败 (严格模式):\n{error_msgs}')
            else:
                # 非严格模式：记录警告但不阻止启动
                for key, msg, _ in error_errors:
                    logger.warning(f'配置问题 [{key}]: {msg} [ERROR]')

    def initialize_components(self,
                             state_estimator: Optional[IStateEstimator] = None,
                             mpc_tracker: Optional[ITrajectoryTracker] = None,
                             backup_tracker: Optional[ITrajectoryTracker] = None,
                             consistency_checker: Optional[IConsistencyChecker] = None,
                             safety_monitor: Optional[ISafetyMonitor] = None,
                             smooth_transition: Optional[ISmoothTransition] = None,
                             coord_transformer: Optional[ICoordinateTransformer] = None,
                             mpc_health_monitor: Optional[MPCHealthMonitor] = None,
                             processors: Optional[list[IControlProcessor]] = None) -> None:
        """
        组件注入方法
        
        依赖绑定:
        - 各组件独立，无显式相互依赖注入。
        - 运行时依赖（如 Transformer 需要 Estimator 状态）通过方法参数传递。
        """
        if state_estimator:
            self.state_estimator = state_estimator
        if mpc_tracker:
            self.mpc_tracker = mpc_tracker
        if backup_tracker:
            self.backup_tracker = backup_tracker
        if consistency_checker:
            self.consistency_checker = consistency_checker
        if safety_monitor:
            self.safety_monitor = safety_monitor
        if smooth_transition:
            self.smooth_transition = smooth_transition
        if coord_transformer:
            self.coord_transformer = coord_transformer
        if mpc_health_monitor:
            self.mpc_health_monitor = mpc_health_monitor
        if processors:
            self.processors = processors
        
        # 初始化状态机（如果尚未初始化）
        if self.state_machine is None:
            self.state_machine = StateMachine(self.config)

    def _get_default_component_classes(self) -> Dict[str, Any]:
        """
        获取默认组件类
        
        子类可以重写此方法以注入自定义组件类，而无需重写整个初始化逻辑。
        """
        return {
            'estimator_cls': AdaptiveEKFEstimator,
            'mpc_cls': MPCController,
            'backup_cls': PurePursuitController,
            'consistency_cls': WeightedConsistencyAnalyzer,
            'safety_cls': BasicSafetyMonitor,
            'transformer_cls': RobustCoordinateTransformer,
            'smooth_linear_cls': LinearSmoothTransition,
            'smooth_exp_cls': ExponentialSmoothTransition,
            'attitude_processor_cls': AttitudeProcessor,
        }

    def initialize_default_components(self) -> None:
        """
        使用默认组件初始化
        
        这是一个便利方法，用于快速设置标准组件栈。
        如果需要自定义各个组件实例，请直接调用 initialize_components()。
        """
        classes = self._get_default_component_classes()
        
        # 根据配置选择平滑过渡实现
        transition_type = self.config.get('transition', {}).get('type', 'exponential')
        if transition_type == 'linear':
            smooth_transition = classes['smooth_linear_cls'](self.config)
        else:
            smooth_transition = classes['smooth_exp_cls'](self.config)
        
        # 基础组件
        components = {
            'state_estimator': classes['estimator_cls'](self.config),
            'mpc_tracker': classes['mpc_cls'](self.config, self.platform_config),
            'backup_tracker': classes['backup_cls'](self.config, self.platform_config),
            'consistency_checker': classes['consistency_cls'](self.config),
            'safety_monitor': classes['safety_cls'](self.config, self.platform_config),
            'smooth_transition': smooth_transition,
            'coord_transformer': classes['transformer_cls'](self.config),
            'mpc_health_monitor': MPCHealthMonitor(self.config)
        }
        
        # Load processors
        proc_list = []
        if self.is_quadrotor:
            proc_list.append(classes['attitude_processor_cls'](self.config))
        components['processors'] = proc_list
        
        self.initialize_components(**components)

    def _reset_all_stateful_components(self) -> None:
        """
        重置所有有状态的组件
        
        在长暂停后调用，确保所有组件从干净状态开始。
        这避免了以下问题：
        - EKF 使用过时的状态估计
        - Safety Monitor 使用过时的 last_cmd 计算加速度
        - MPC/Backup Tracker 使用过时的 last_cmd
        - Consistency Checker 使用过时的时序窗口数据
        - Smooth Transition 使用过时的过渡起始命令
        - MPC Health Monitor 使用过时的健康历史
        - Timeout Monitor 使用过时的启动时间和超时状态
        - Coordinate Transformer 使用过时的降级状态
        - State Machine 使用过时的恢复计数器和 MPC 历史（保留状态和停止请求）
        - Processors (如 AttitudeProcessor) 使用过时的姿态和悬停状态
        """
        if self.state_estimator:
            self.state_estimator.reset()
        if self.safety_monitor:
            self.safety_monitor.reset()
        if self.mpc_tracker:
            self.mpc_tracker.reset()
        if self.backup_tracker:
            self.backup_tracker.reset()
        if self.consistency_checker:
            self.consistency_checker.reset()
        if self.smooth_transition:
            self.smooth_transition.reset()
        if self.mpc_health_monitor:
            self.mpc_health_monitor.reset()
        if self.timeout_monitor:
            self.timeout_monitor.reset()
        if self.coord_transformer:
            self.coord_transformer.reset()
        if self.state_machine:
            # 使用 reset_counters_only() 而非 reset()
            # 保留当前状态和停止请求，仅清除过时的历史数据
            # 这确保了：
            # - 暂停前的 STOPPING 状态会保持（安全关键）
            # - 暂停前的 BACKUP_ACTIVE 状态会保持（谨慎恢复）
            self.state_machine.reset_counters_only()
        # 重置所有处理器（如 AttitudeProcessor）
        for processor in self.processors:
            if hasattr(processor, 'reset'):
                processor.reset()

    def reset(self) -> None:
        """
        重置控制器管理器状态
        
        重置所有组件的内部状态，但保留资源。
        可以继续调用 update() 方法。
        """
        if self.state_estimator: self.state_estimator.reset()
        if self.consistency_checker: self.consistency_checker.reset()
        if self.state_machine: self.state_machine.reset()
        if self.safety_monitor: self.safety_monitor.reset()
        if self.mpc_health_monitor: self.mpc_health_monitor.reset()
        # 重置所有处理器
        for processor in self.processors:
            if hasattr(processor, 'reset'):
                processor.reset()
        if self.smooth_transition: self.smooth_transition.reset()
        if self.coord_transformer: self.coord_transformer.reset()
        # 重置轨迹跟踪器内部状态（不释放资源）
        if self.mpc_tracker: self.mpc_tracker.reset()
        if self.backup_tracker: self.backup_tracker.reset()
        self.timeout_monitor.reset()
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._safety_check_passed = True
        self._last_mpc_cmd = None
        self._last_backup_cmd = None
        self._last_tracking_error = None
        self._last_tracking_quality = None
        self._last_consistency = None
        self._last_mpc_health = None
        self._last_attitude_cmd = None
        self._last_update_time = None  # 重置时间跟踪
        # 重置 notify 跟踪状态
        self._current_horizon = self.horizon_normal
        self._last_mpc_predicted_state = None  # 重置预测状态
        if self.mpc_tracker:
            self.mpc_tracker.set_horizon(self.horizon_normal)
            
    def shutdown(self) -> None:
        """
        关闭控制器管理器并释放所有资源
        
        按正确顺序关闭所有组件：
        1. 先关闭依赖其他组件的组件
        2. 再关闭被依赖的组件
        3. 最后清理诊断发布器
        
        调用后管理器不应再使用。
        """
        logger.info("Shutting down ControllerManager...")
        
        # 1. 关闭轨迹跟踪器（可能有外部资源如 ACADOS）
        if self.backup_tracker:
            try:
                self.backup_tracker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down backup_tracker: {e}")
            self.backup_tracker = None
        
        if self.mpc_tracker:
            try:
                self.mpc_tracker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down mpc_tracker: {e}")
            self.mpc_tracker = None
        
        # 2. 关闭坐标变换器（依赖 state_estimator）
        if self.coord_transformer:
            try:
                self.coord_transformer.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down coord_transformer: {e}")
            self.coord_transformer = None
        
        # 3. 关闭附加处理器
        for processor in self.processors:
            try:
                if hasattr(processor, 'shutdown'):
                    processor.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down processor {processor}: {e}")
        self.processors.clear()
        
        # 4. 关闭状态估计器
        if self.state_estimator:
            try:
                self.state_estimator.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down state_estimator: {e}")
            self.state_estimator = None
        
        # 5. 关闭其他组件（无外部资源，但调用 shutdown 以保持一致性）
        if self.consistency_checker:
            try:
                self.consistency_checker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down consistency_checker: {e}")
            self.consistency_checker = None
        
        if self.safety_monitor:
            try:
                self.safety_monitor.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down safety_monitor: {e}")
            self.safety_monitor = None
        
        if self.smooth_transition:
            try:
                self.smooth_transition.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down smooth_transition: {e}")
            self.smooth_transition = None
        
        # 6. 清理非接口组件
        if self.mpc_health_monitor:
            self.mpc_health_monitor.reset()
            self.mpc_health_monitor = None
        
        if self.state_machine:
            self.state_machine.reset()
            self.state_machine = None
        
        # 7. 清理诊断发布器回调
        if hasattr(self, '_diagnostics_publisher'):
            self._diagnostics_publisher.clear_callbacks()
        
        # 8. 重置内部状态
        self._last_state = ControllerState.STOPPED
        self._last_diagnostics = None
        self._last_mpc_health = None
        self._last_consistency = None
        self._last_mpc_cmd = None
        self._last_backup_cmd = None
        self._last_tracking_error = None
        self._last_tracking_quality = None
        self._last_attitude_cmd = None
        self._last_update_time = None
        
        logger.info("ControllerManager shutdown complete")
