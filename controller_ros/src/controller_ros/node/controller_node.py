"""
控制器 ROS2 节点

主节点实现，组装所有组件，管理控制循环。
支持 TF2 坐标变换集成。
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from ..bridge import ControllerBridge, TFBridge
from ..io import SubscriberManager, PublisherManager, ServiceManager
from ..utils import ParamLoader, TimeSync


class ControllerNode(Node):
    """
    控制器主节点 (ROS2)
    
    职责:
    - 初始化 ROS 节点
    - 加载配置参数
    - 组装各层组件
    - 管理控制循环
    - TF2 坐标变换集成
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    """
    
    def __init__(self):
        super().__init__('universal_controller_node')
        
        # 1. 加载参数
        self._params = ParamLoader.load(self)
        self._topics = ParamLoader.get_topics(self)
        
        # 2. 创建回调组 (用于并发控制)
        self._sensor_cb_group = ReentrantCallbackGroup()
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 3. 获取平台配置
        platform_type = self._params.get('system', {}).get('platform', 'differential')
        from universal_controller.config.default_config import PLATFORM_CONFIG
        platform_config = PLATFORM_CONFIG.get(platform_type, PLATFORM_CONFIG['differential'])
        default_frame_id = platform_config.get('output_frame', 'base_link')
        
        # 4. 创建 TF 桥接
        self._tf_bridge = TFBridge(self)
        
        # 5. 创建控制器桥接
        self._controller_bridge = ControllerBridge(self._params)
        
        # 6. 注入 TF2 到坐标变换器
        self._inject_tf2()
        
        # 7. 创建订阅管理器
        self._subscribers = SubscriberManager(
            self, self._topics, self._sensor_cb_group
        )
        
        # 8. 创建发布管理器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 5)
        self._publishers = PublisherManager(
            self, self._topics, default_frame_id,
            diag_publish_rate=diag_publish_rate
        )
        
        # 9. 创建服务管理器
        self._services = ServiceManager(
            self,
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state
        )
        
        # 10. 创建时间同步
        # 注意: _merge_params 将 time_sync 合并到 watchdog，所以从 watchdog 读取
        watchdog_config = self._params.get('watchdog', {})
        self._time_sync = TimeSync(
            max_odom_age_ms=watchdog_config.get('odom_timeout_ms', 100),
            max_traj_age_ms=watchdog_config.get('traj_timeout_ms', 200),
            max_imu_age_ms=watchdog_config.get('imu_timeout_ms', 50)
        )
        
        # 11. 设置诊断回调
        self._controller_bridge.set_diagnostics_callback(self._on_diagnostics)
        
        # 12. 创建控制定时器
        control_rate = self._params.get('node', {}).get('control_rate', 50.0)
        control_period = 1.0 / control_rate
        self._control_timer = self.create_timer(
            control_period,
            self._control_callback,
            callback_group=self._control_cb_group
        )
        
        # 状态
        self._waiting_for_data_logged = False
        
        self.get_logger().info(
            f'Controller node initialized (platform={platform_type}, '
            f'rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized})'
        )
    
    def _inject_tf2(self):
        """将 TF2 注入到坐标变换器"""
        if not self._tf_bridge.is_initialized:
            self.get_logger().info("TF2 not available, using fallback coordinate transform")
            return
        
        # 获取 universal_controller 的坐标变换器
        manager = self._controller_bridge.manager
        if manager is not None and hasattr(manager, 'coord_transformer') and manager.coord_transformer is not None:
            success = self._tf_bridge.inject_to_transformer(manager.coord_transformer)
            if success:
                self.get_logger().info("TF2 successfully injected to coordinate transformer")
            else:
                self.get_logger().warn("Failed to inject TF2 to coordinate transformer")
        else:
            self.get_logger().info("ControllerManager has no coord_transformer, TF2 injection skipped")
    
    def _control_callback(self):
        """控制循环回调"""
        # 1. 获取最新数据
        odom = self._subscribers.get_latest_odom()
        imu = self._subscribers.get_latest_imu()
        trajectory = self._subscribers.get_latest_trajectory()
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if not self._waiting_for_data_logged:
                self.get_logger().warn('Waiting for odom and trajectory data...')
                self._waiting_for_data_logged = True
            return
        
        if self._waiting_for_data_logged:
            self.get_logger().info('Data received, starting control')
            self._waiting_for_data_logged = False
        
        # 3. 检查数据新鲜度
        ages = self._subscribers.get_data_ages()
        timeouts = self._time_sync.check_freshness(ages)
        
        if timeouts.get('odom_timeout', False):
            self.get_logger().warn(
                f"Odom timeout: age={ages.get('odom', 0)*1000:.1f}ms",
                throttle_duration_sec=1.0
            )
        
        # 4. 执行控制更新
        try:
            cmd = self._controller_bridge.update(odom, trajectory, imu)
        except Exception as e:
            self.get_logger().error(f'Controller update failed: {e}')
            self._publishers.publish_stop_cmd()
            # 发布错误诊断信息
            error_diag = {
                'state': 0,  # INIT/ERROR state
                'mpc_success': False,
                'backup_active': False,
                'error_message': str(e),
                'timeout': {
                    'odom_timeout': timeouts.get('odom_timeout', False),
                    'traj_timeout': timeouts.get('traj_timeout', False),
                    'imu_timeout': timeouts.get('imu_timeout', False),
                },
                'cmd': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'omega': 0.0},
            }
            self._publishers.publish_diagnostics(error_diag, force=True)
            return
        
        # 5. 发布控制命令
        self._publishers.publish_cmd(cmd)
        
        # 6. 发布调试路径
        self._publishers.publish_debug_path(trajectory)
    
    def _on_diagnostics(self, diag):
        """诊断回调"""
        self._publishers.publish_diagnostics(diag)
    
    def _handle_reset(self):
        """处理重置请求"""
        self._controller_bridge.reset()
        self._waiting_for_data_logged = False
        self.get_logger().info('Controller reset')
    
    def _handle_get_diagnostics(self):
        """处理获取诊断请求"""
        return self._controller_bridge.get_diagnostics()
    
    def _handle_set_state(self, target_state: int) -> bool:
        """
        处理设置状态请求
        
        出于安全考虑，只支持请求 STOPPING 状态 (值为 5)。
        其他状态转换应该由状态机内部逻辑自动控制。
        
        Args:
            target_state: 目标状态值 (ControllerState 枚举)
        
        Returns:
            是否成功
        """
        from universal_controller.core.enums import ControllerState
        
        # 只允许请求 STOPPING 状态
        if target_state == ControllerState.STOPPING.value:
            success = self._controller_bridge.request_stop()
            if success:
                self.get_logger().info('Stop requested via service')
            return success
        else:
            self.get_logger().warn(
                f'Set state to {target_state} not allowed. '
                f'Only STOPPING ({ControllerState.STOPPING.value}) is supported for safety reasons.'
            )
            return False
    
    def shutdown(self):
        """清理资源"""
        self._controller_bridge.shutdown()
        self.get_logger().info('Controller node shutdown')


def main(args=None):
    """主入口"""
    rclpy.init(args=args)
    
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
