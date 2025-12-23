"""
服务管理器

管理 ROS 服务。
"""
from typing import Optional, Callable
import logging

from rclpy.node import Node

logger = logging.getLogger(__name__)


class ServiceManager:
    """
    服务管理器
    
    职责:
    - 创建和管理服务
    - 处理服务请求
    """
    
    def __init__(self, node: Node,
                 reset_callback: Optional[Callable] = None,
                 get_diagnostics_callback: Optional[Callable] = None):
        """
        初始化服务管理器
        
        Args:
            node: ROS2 节点
            reset_callback: 重置回调函数
            get_diagnostics_callback: 获取诊断回调函数
        """
        self._node = node
        self._reset_callback = reset_callback
        self._get_diagnostics_callback = get_diagnostics_callback
        
        self._create_services()
    
    def _create_services(self):
        """创建所有服务"""
        from std_srvs.srv import Trigger
        
        # 重置服务
        self._reset_srv = self._node.create_service(
            Trigger,
            '/controller/reset',
            self._handle_reset
        )
        logger.info("Created reset service: /controller/reset")
        
        # 获取诊断服务 (使用自定义消息时启用)
        try:
            from controller_ros.srv import GetDiagnostics
            self._get_diag_srv = self._node.create_service(
                GetDiagnostics,
                '/controller/get_diagnostics',
                self._handle_get_diagnostics
            )
            logger.info("Created get_diagnostics service")
        except ImportError:
            self._get_diag_srv = None
            logger.warn("GetDiagnostics service not available")
    
    def _handle_reset(self, request, response):
        """处理重置请求"""
        try:
            if self._reset_callback:
                self._reset_callback()
            response.success = True
            response.message = "Controller reset successfully"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
        return response
    
    def _handle_get_diagnostics(self, request, response):
        """处理获取诊断请求"""
        try:
            if self._get_diagnostics_callback:
                diag = self._get_diagnostics_callback()
                if diag:
                    response.success = True
                    # 填充诊断数据到 response.diagnostics
                    # response.diagnostics 是 DiagnosticsV2 类型
                    d = response.diagnostics
                    d.header.stamp = self._node.get_clock().now().to_msg()
                    d.header.frame_id = 'controller'
                    
                    d.state = diag.get('state', 0)
                    d.mpc_success = diag.get('mpc_success', False)
                    d.mpc_solve_time_ms = float(diag.get('mpc_solve_time_ms', 0.0))
                    d.backup_active = diag.get('backup_active', False)
                    
                    # MPC 健康状态
                    mpc_health = diag.get('mpc_health', {})
                    d.mpc_health_kkt_residual = float(mpc_health.get('kkt_residual', 0.0))
                    d.mpc_health_condition_number = float(mpc_health.get('condition_number', 1.0))
                    d.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
                    d.mpc_health_degradation_warning = mpc_health.get('degradation_warning', False)
                    d.mpc_health_can_recover = mpc_health.get('can_recover', True)
                    
                    # 一致性指标
                    consistency = diag.get('consistency', {})
                    d.consistency_curvature = float(consistency.get('curvature', 0.0))
                    d.consistency_velocity_dir = float(consistency.get('velocity_dir', 1.0))
                    d.consistency_temporal = float(consistency.get('temporal', 1.0))
                    d.consistency_alpha_soft = float(consistency.get('alpha_soft', 0.0))
                    d.consistency_data_valid = consistency.get('data_valid', True)
                    
                    # 超时状态
                    timeout = diag.get('timeout', {})
                    d.timeout_odom = timeout.get('odom_timeout', False)
                    d.timeout_traj = timeout.get('traj_timeout', False)
                    d.timeout_traj_grace_exceeded = timeout.get('traj_grace_exceeded', False)
                    d.timeout_imu = timeout.get('imu_timeout', False)
                    d.timeout_last_odom_age_ms = float(timeout.get('last_odom_age_ms', 0.0))
                    d.timeout_last_traj_age_ms = float(timeout.get('last_traj_age_ms', 0.0))
                    d.timeout_last_imu_age_ms = float(timeout.get('last_imu_age_ms', 0.0))
                    d.timeout_in_startup_grace = timeout.get('in_startup_grace', False)
                    
                    # 控制命令
                    cmd = diag.get('cmd', {})
                    d.cmd_vx = float(cmd.get('vx', 0.0))
                    d.cmd_vy = float(cmd.get('vy', 0.0))
                    d.cmd_vz = float(cmd.get('vz', 0.0))
                    d.cmd_omega = float(cmd.get('omega', 0.0))
                    d.cmd_frame_id = cmd.get('frame_id', '')
                    
                    d.transition_progress = float(diag.get('transition_progress', 0.0))
                else:
                    response.success = False
            else:
                response.success = False
        except Exception as e:
            response.success = False
            logger.error(f"Get diagnostics failed: {e}")
        return response
    
    def set_reset_callback(self, callback: Callable):
        """设置重置回调"""
        self._reset_callback = callback
    
    def set_get_diagnostics_callback(self, callback: Callable):
        """设置获取诊断回调"""
        self._get_diagnostics_callback = callback
