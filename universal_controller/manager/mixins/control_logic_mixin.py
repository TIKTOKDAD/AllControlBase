from typing import Dict, Any, Optional, Tuple, List
import numpy as np
import time
import logging

from ...core.enums import ControllerState
from ...core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, 
    TimeoutStatus
)
from ...core.diagnostics_input import DiagnosticsInput
from ...core.interfaces import IControlProcessor

logger = logging.getLogger(__name__)

class ControlLogicMixin:
    """
    Mixin for ControllerManager control logic (MPC, Safety, StateMachine).
    """

    def _sync_mpc_horizon(self):
        """
        同步 MPC Horizon
        
        如果在状态切换期间 set_horizon 被节流，此方法会在 update 循环中
        重试同步，确保底层求解器 Horizon 与管理器状态一致。
        """
        # 使用 getattr 防御性获取属性，虽然接口已定义，但增加运行时的健壮性
        tracker_horizon = getattr(self.mpc_tracker, 'horizon', None)
        if self.mpc_tracker and tracker_horizon is not None and self._current_horizon != tracker_horizon:
            # 尝试同步 (set_horizon 内部有节流检查)
            if self.mpc_tracker.set_horizon(self._current_horizon):
                logger.info(f"MPC horizon synced to {self._current_horizon} (delayed)")

    def _on_state_changed(self, old_state: ControllerState, new_state: ControllerState) -> None:
        """状态变化回调"""
        logger.info(f"Controller state changed: {old_state.name} -> {new_state.name}")
        
        # MPC horizon 动态调整
        if new_state == ControllerState.MPC_DEGRADED:
            target_horizon = self.horizon_degraded
            if self._current_horizon != target_horizon:
                self._current_horizon = target_horizon
                if self.mpc_tracker:
                    if self.mpc_tracker.set_horizon(target_horizon):
                        logger.info(f"MPC horizon reduced to {target_horizon}")
                    else:
                        logger.debug(f"MPC horizon change to {target_horizon} throttled, will sync later")
        elif new_state == ControllerState.NORMAL and old_state == ControllerState.MPC_DEGRADED:
            target_horizon = self.horizon_normal
            if self._current_horizon != target_horizon:
                self._current_horizon = target_horizon
                if self.mpc_tracker:
                    if self.mpc_tracker.set_horizon(target_horizon):
                        logger.info(f"MPC horizon restored to {target_horizon}")
                    else:
                        logger.debug(f"MPC horizon change to {target_horizon} throttled, will sync later")
        
        # 平滑过渡
        if (old_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED] and
            new_state == ControllerState.BACKUP_ACTIVE):
            if self.smooth_transition and self._last_mpc_cmd is not None:
                self.smooth_transition.start_transition(self._last_mpc_cmd)
        
        elif (old_state == ControllerState.BACKUP_ACTIVE and
              new_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED]):
            if self.smooth_transition and self._last_backup_cmd is not None:
                self.smooth_transition.start_transition(self._last_backup_cmd)

    def _compute_mpc(self, state: np.ndarray, trajectory: Trajectory,
                     consistency: ConsistencyResult) -> Tuple[Optional[ControlOutput], Optional[Any]]:
        """
        MPC 计算和健康监控
        
        Returns:
            (mpc_cmd, mpc_health): MPC 命令和健康状态
        """
        mpc_cmd = None
        mpc_health = None
        
        if self.mpc_tracker and self._last_state not in [
            ControllerState.BACKUP_ACTIVE, 
            ControllerState.STOPPING,
            ControllerState.STOPPED
        ]:
            mpc_cmd = self.mpc_tracker.compute(state, trajectory, consistency)
            if mpc_cmd.success:
                # mpc_cmd 是从 compute() 返回的新对象，无需 copy
                self._last_mpc_cmd = mpc_cmd
        
        if self.mpc_health_monitor and mpc_cmd is not None:
            mpc_health = self.mpc_health_monitor.update(
                mpc_cmd.solve_time_ms,
                mpc_cmd.health_metrics.get('kkt_residual', 0.0),
                mpc_cmd.health_metrics.get('condition_number', 1.0)
            )
        
        self._last_mpc_health = mpc_health
        return mpc_cmd, mpc_health

    def _select_controller_output(self, state: np.ndarray, trajectory: Trajectory,
                                  consistency: ConsistencyResult,
                                  mpc_cmd: Optional[ControlOutput]) -> ControlOutput:
        """选择控制器输出"""
        # STOPPING 状态：输出零速度，让机器人停下来
        if self._last_state == ControllerState.STOPPING:
            # 直接输出零速度，不再跟踪轨迹
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        
        # STOPPED 状态：完全停止
        elif self._last_state == ControllerState.STOPPED:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        
        # BACKUP_ACTIVE 状态：使用备用控制器
        elif self._last_state == ControllerState.BACKUP_ACTIVE:
            if self.backup_tracker:
                cmd = self.backup_tracker.compute(state, trajectory, consistency)
                self._last_backup_cmd = cmd.copy()
            else:
                cmd = ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
            return cmd
        
        # 其他状态：优先使用 MPC 输出
        else:
            if mpc_cmd and mpc_cmd.success:
                return mpc_cmd
            
            # MPC 失败或未运行：尝试使用备份控制器
            # 这是关键的架构变更：由 Manager 统一处理降级
            if self.backup_tracker:
                if mpc_cmd:  # 如果有失败的 MPC 命令，记录原因
                    logger.warning(f"MPC failed, falling back to backup controller. "
                                 f"Reason: {mpc_cmd.health_metrics.get('error_type', 'unknown')}")
                
                cmd = self.backup_tracker.compute(state, trajectory, consistency)
                self._last_backup_cmd = cmd.copy()
                
                # 标记为 fallback 来源，方便诊断
                if cmd.health_metrics is None:
                    cmd.health_metrics = {}
                cmd.health_metrics['source'] = 'backup_fallback'
                return cmd
            
            # 彻底失败：停车
            logger.error("MPC failed and no backup controller available. Stopping.")
            return ControlOutput(
                vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id, success=False)

    def _apply_safety_check(self, state: np.ndarray, cmd: ControlOutput,
                           diagnostics: DiagnosticsInput, dt: float) -> ControlOutput:
        """应用安全检查
        
        防御性设计:
        - 如果安全检查失败且提供了 limited_cmd，使用 limited_cmd
        - 如果安全检查失败但 limited_cmd 为 None，强制输出零速度（兜底）
        - 这确保不安全的命令永远不会被透传执行
        """
        if self.safety_monitor:
            safety_decision = self.safety_monitor.check(state, cmd, diagnostics, dt)
            if not safety_decision.safe:
                self._safety_failed = True
                self._safety_check_passed = False
                if safety_decision.limited_cmd is not None:
                    cmd = safety_decision.limited_cmd
                else:
                    # 防御性兜底：如果检查失败但未提供限制命令，强制停车
                    # 这是安全关键的最后防线
                    logger.warning(
                        f"Safety check failed without limited_cmd provided. "
                        f"Forcing zero velocity. Reason: {safety_decision.reason}"
                    )
                    cmd = ControlOutput(
                        vx=0.0, vy=0.0, vz=0.0, omega=0.0,
                        frame_id=getattr(self, 'default_frame_id', ''),
                        success=False,
                        health_metrics={'error_type': 'safety_fallback'}
                    )
                diagnostics.safety_failed = True
            else:
                self._safety_failed = False
                self._safety_check_passed = True
        else:
            self._safety_check_passed = True
        return cmd

    def _update_state_machine(self, diagnostics: DiagnosticsInput) -> None:
        """更新状态机"""
        if self.state_machine:
            new_state = self.state_machine.update(diagnostics)
            if new_state != self._last_state:
                self._on_state_changed(self._last_state, new_state)
                self._last_state = new_state

    def _apply_smooth_transition(self, cmd: ControlOutput, 
                                 current_time: float) -> ControlOutput:
        """应用平滑过渡"""
        if self.smooth_transition and not self.smooth_transition.is_complete():
            cmd = self.smooth_transition.get_blended_output(cmd, current_time)
        return cmd

    def _run_processors(self, state: np.ndarray, cmd: ControlOutput) -> None:
        """运行额外处理器 (带性能监控)
        
        性能监控策略:
        - 始终进行计时 (time.monotonic() 开销极小，约 50ns)
        - 超过 5ms 阈值时才记录警告
        - DEBUG 模式下额外记录每次调用的耗时
        """
        if not self.processors:
            return
            
        # DEBUG 模式下启用详细日志
        enable_debug_log = logger.isEnabledFor(logging.DEBUG)
        
        # 性能监控阈值 (毫秒)
        SLOW_PROCESSOR_THRESH_MS = 5.0

        # 将解耦的平台特定逻辑（如姿态控制）作为处理器运行
        for processor in self.processors:
            # 使用对象 ID 作为唯一键，避免同类实例冲突
            proc_id = id(processor)
            proc_name = processor.__class__.__name__
            
            # 检查是否因为连续失败而被禁用 (Defensive access)
            if not hasattr(self, '_processor_failure_counts'):
                self._processor_failure_counts = {}
            if self._processor_failure_counts.get(proc_id, 0) >= self._processor_max_failures:
                continue

            try:
                # 始终计时: time.monotonic() 开销极小 (~50ns)，但能捕获生产环境的性能问题
                proc_start = time.monotonic()
                extra_result = processor.compute_extra(state, cmd)
                proc_duration = (time.monotonic() - proc_start) * 1000
                
                # DEBUG 模式下记录每次调用
                if enable_debug_log:
                    logger.debug(f"Processor {proc_name} took {proc_duration:.2f}ms")
                
                # 性能告警：处理器耗时超过阈值 (生产环境也生效)
                if proc_duration > SLOW_PROCESSOR_THRESH_MS:
                    logger.warning(
                        f"Processor {proc_name} took too long: {proc_duration:.2f}ms. "
                        f"This may affect control loop stability."
                    )
                
                # 成功运行，重置失败计数
                if self._processor_failure_counts.get(proc_id, 0) > 0:
                    self._processor_failure_counts[proc_id] = 0
                
                if extra_result:
                    cmd.extras.update(extra_result)
            except Exception as e:
                # 累加失败计数
                self._processor_failure_counts[proc_id] = self._processor_failure_counts.get(proc_id, 0) + 1
                fail_count = self._processor_failure_counts[proc_id]
                
                if fail_count >= self._processor_max_failures:
                    logger.error(f"Processor {proc_name} (id={proc_id}) failed {fail_count} times. Disabling it. Last error: {e}")
                else:
                    logger.error(f"Processor {proc_name} (id={proc_id}) failed (count {fail_count}): {e}")

    def request_stop(self) -> bool:
        """
        请求控制器进入停止状态
        
        代理到状态机的 request_stop 方法
        
        Returns:
            bool: 成功请求返回 True，否则返回 False
        """
        if self.state_machine:
            return self.state_machine.request_stop()
        return False
    
    def get_state(self) -> ControllerState:
        return self._last_state

    # 辅助接口
    def set_hover_yaw(self, yaw: float) -> None:
        for p in self.processors:
            if hasattr(p, 'set_hover_yaw'):
                p.set_hover_yaw(yaw)

    def get_attitude_rate_limits(self) -> Optional[Dict[str, Any]]:
        for p in self.processors:
            if hasattr(p, 'get_attitude_rate_limits'):
                return p.get_attitude_rate_limits()
        return None
    
    def get_timeout_status(self) -> TimeoutStatus:
        """获取当前超时状态 (供外部查询)"""
        if hasattr(self, '_last_timeout_status'):
            return self._last_timeout_status
        # 如果尚未运行 update，返回默认超时状态
        return self.timeout_monitor.check({})
