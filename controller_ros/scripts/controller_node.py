#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点

主节点实现，组装所有组件，管理控制循环。
支持 TF2 坐标变换集成。
"""
import rospy
import threading
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_srvs.srv import Trigger, TriggerResponse

# 导入自定义消息
from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2

# 导入 universal_controller
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry as UcOdometry, Imu as UcImu, Trajectory as UcTrajectory,
    Header, Point3D, ControlOutput
)
from universal_controller.core.enums import TrajectoryMode

import numpy as np

# TF2 支持 (可选)
TF2_AVAILABLE = False
try:
    import tf2_ros
    TF2_AVAILABLE = True
except ImportError:
    rospy.logwarn("tf2_ros not available, TF2 integration disabled")


def _get_ros_time_sec() -> float:
    """
    获取当前 ROS 时间（秒）
    
    支持仿真时间模式 (use_sim_time=true)
    
    注意: 在仿真时间模式下，如果 /clock 话题还未发布，
    rospy.Time.now() 会返回 0。此时回退到系统时间。
    """
    try:
        ros_time = rospy.Time.now()
        # 检查时间是否有效 (仿真时间模式下可能为 0)
        if ros_time.to_sec() > 0:
            return ros_time.to_sec()
        else:
            # 仿真时间为 0，回退到系统时间
            import time
            return time.time()
    except rospy.exceptions.ROSInitException:
        # ROS 未初始化时回退到系统时间
        import time
        return time.time()


class TF2Bridge:
    """TF2 桥接层 (ROS1 版本)"""
    
    def __init__(self):
        self._buffer = None
        self._listener = None
        self._initialized = False
        
        if TF2_AVAILABLE:
            try:
                self._buffer = tf2_ros.Buffer()
                self._listener = tf2_ros.TransformListener(self._buffer)
                self._initialized = True
                rospy.loginfo("TF2 bridge initialized")
            except Exception as e:
                rospy.logwarn(f"TF2 initialization failed: {e}")
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time=None, timeout_sec: float = 0.01) -> Optional[Dict]:
        """查询坐标变换"""
        if not self._initialized or self._buffer is None:
            return None
        
        try:
            if time is None:
                time = rospy.Time(0)
            
            transform = self._buffer.lookup_transform(
                target_frame, source_frame, time,
                timeout=rospy.Duration(timeout_sec)
            )
            
            return {
                'translation': (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ),
                'rotation': (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
            }
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            rospy.logdebug(f"TF lookup failed: {source_frame} -> {target_frame}: {e}")
            return None
        except Exception as e:
            rospy.logwarn(f"TF lookup error: {e}")
            return None
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.01) -> bool:
        """检查是否可以进行坐标变换"""
        if not self._initialized or self._buffer is None:
            return False
        
        try:
            if time is None:
                time = rospy.Time(0)
            return self._buffer.can_transform(
                target_frame, source_frame, time,
                timeout=rospy.Duration(timeout_sec)
            )
        except Exception:
            return False
    
    def inject_to_transformer(self, coord_transformer) -> bool:
        """将 TF2 变换注入到 universal_controller 的坐标变换器"""
        if not self._initialized:
            rospy.logwarn("TF2 not initialized, cannot inject to transformer")
            return False
        
        if coord_transformer is None:
            return False
        
        if hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            coord_transformer.set_tf2_lookup_callback(self.lookup_transform)
            rospy.loginfo("TF2 lookup callback injected to coordinate transformer")
            return True
        else:
            rospy.logwarn("Coordinate transformer does not support TF2 injection")
            return False
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized


class ControllerNode:
    """
    控制器主节点 (ROS1 Noetic)
    
    职责:
    - 初始化 ROS 节点
    - 加载配置参数
    - 管理控制循环
    - TF2 坐标变换集成
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    """
    
    def __init__(self):
        rospy.init_node('universal_controller_node')
        
        # 1. 加载参数
        self._load_parameters()
        
        # 2. 初始化 TF2 桥接
        self._tf_bridge = TF2Bridge()
        
        # 3. 初始化控制器
        self._init_controller()
        
        # 4. 注入 TF2 到坐标变换器
        self._inject_tf2()
        
        # 5. 创建订阅器
        self._create_subscribers()
        
        # 6. 创建发布器
        self._create_publishers()
        
        # 7. 创建服务
        self._create_services()
        
        # 8. 数据缓存 (线程安全)
        self._lock = threading.Lock()
        self._latest_odom: Optional[UcOdometry] = None
        self._latest_imu: Optional[UcImu] = None
        self._latest_traj: Optional[UcTrajectory] = None
        self._data_timestamps: Dict[str, float] = {}
        
        # 9. 状态
        self._waiting_for_data = True
        
        # 10. 创建控制定时器
        control_period = 1.0 / self._control_rate
        self._timer = rospy.Timer(rospy.Duration(control_period), self._control_callback)
        
        rospy.loginfo(f"Controller node initialized (platform={self._platform_type}, "
                     f"rate={self._control_rate}Hz, tf2={self._tf_bridge.is_initialized})")
    
    def _load_parameters(self):
        """加载 ROS 参数"""
        # 节点参数
        self._control_rate = rospy.get_param('~control_rate', 
                                             rospy.get_param('node/control_rate', 50.0))
        
        # 平台类型
        self._platform_type = rospy.get_param('~platform_type',
                                              rospy.get_param('platform/type', 'differential'))
        
        # 话题
        self._topics = {
            'odom': rospy.get_param('~odom', rospy.get_param('topics/odom', '/odom')),
            'imu': rospy.get_param('~imu', rospy.get_param('topics/imu', '/imu')),
            'trajectory': rospy.get_param('~trajectory', 
                                          rospy.get_param('topics/trajectory', '/nn/local_trajectory')),
            'cmd_unified': rospy.get_param('~cmd_unified',
                                           rospy.get_param('topics/cmd_unified', '/cmd_unified')),
            'diagnostics': rospy.get_param('~diagnostics',
                                           rospy.get_param('topics/diagnostics', '/controller/diagnostics')),
        }
        
        # TF 配置
        self._tf_source_frame = rospy.get_param('tf/source_frame', 'base_link')
        self._tf_target_frame = rospy.get_param('tf/target_frame', 'odom')
        
        # 时间同步
        self._max_odom_age = rospy.get_param('time_sync/max_odom_age_ms', 100) / 1000.0
        self._max_traj_age = rospy.get_param('time_sync/max_traj_age_ms', 200) / 1000.0
        self._max_imu_age = rospy.get_param('time_sync/max_imu_age_ms', 50) / 1000.0
        
        # 构建控制器配置
        self._config = DEFAULT_CONFIG.copy()
        self._config['system'] = DEFAULT_CONFIG['system'].copy()
        self._config['system']['platform'] = self._platform_type
        self._config['system']['ctrl_freq'] = int(self._control_rate)
        
        # TF 配置传递给 universal_controller
        self._config['transform'] = self._config.get('transform', {}).copy()
        self._config['transform']['source_frame'] = self._tf_source_frame
        self._config['transform']['target_frame'] = self._tf_target_frame
        
        # 加载额外的控制器参数
        controller_params = rospy.get_param('controller', {})
        for key, value in controller_params.items():
            if key in self._config:
                if isinstance(value, dict):
                    self._config[key].update(value)
                else:
                    self._config[key] = value
    
    def _init_controller(self):
        """初始化控制器"""
        self._platform_config = PLATFORM_CONFIG.get(
            self._platform_type, PLATFORM_CONFIG['differential']
        )
        self._default_frame_id = self._platform_config.get('output_frame', 'base_link')
        
        self._manager = ControllerManager(self._config)
        self._manager.initialize_default_components()
        
        # 设置诊断回调
        self._manager.set_diagnostics_callback(self._on_diagnostics)
        
        rospy.loginfo("ControllerManager initialized")
    
    def _inject_tf2(self):
        """将 TF2 注入到坐标变换器"""
        if not self._tf_bridge.is_initialized:
            rospy.loginfo("TF2 not available, using fallback coordinate transform")
            return
        
        # 获取 universal_controller 的坐标变换器
        if hasattr(self._manager, 'coord_transformer') and self._manager.coord_transformer is not None:
            success = self._tf_bridge.inject_to_transformer(self._manager.coord_transformer)
            if success:
                rospy.loginfo("TF2 successfully injected to coordinate transformer")
            else:
                rospy.logwarn("Failed to inject TF2 to coordinate transformer")
        else:
            rospy.loginfo("ControllerManager has no coord_transformer, TF2 injection skipped")
    
    def _create_subscribers(self):
        """创建订阅器"""
        # 里程计
        self._odom_sub = rospy.Subscriber(
            self._topics['odom'],
            RosOdometry,
            self._odom_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to odom: {self._topics['odom']}")
        
        # IMU
        self._imu_sub = rospy.Subscriber(
            self._topics['imu'],
            RosImu,
            self._imu_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to imu: {self._topics['imu']}")
        
        # 轨迹
        self._traj_sub = rospy.Subscriber(
            self._topics['trajectory'],
            LocalTrajectoryV4,
            self._traj_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to trajectory: {self._topics['trajectory']}")
    
    def _create_publishers(self):
        """创建发布器"""
        # 统一控制命令
        self._cmd_pub = rospy.Publisher(
            self._topics['cmd_unified'],
            UnifiedCmd,
            queue_size=1
        )
        rospy.loginfo(f"Publishing cmd to: {self._topics['cmd_unified']}")
        
        # 诊断
        self._diag_pub = rospy.Publisher(
            self._topics['diagnostics'],
            DiagnosticsV2,
            queue_size=10
        )
        rospy.loginfo(f"Publishing diagnostics to: {self._topics['diagnostics']}")
    
    def _create_services(self):
        """创建服务"""
        self._reset_srv = rospy.Service(
            '/controller/reset',
            Trigger,
            self._handle_reset
        )
        rospy.loginfo("Created reset service: /controller/reset")
    
    # ==================== 回调函数 ====================
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        uc_odom = UcOdometry(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            pose_position=Point3D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                z=msg.pose.pose.position.z
            ),
            pose_orientation=(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ),
            twist_linear=(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ),
            twist_angular=(
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            )
        )
        
        with self._lock:
            self._latest_odom = uc_odom
            self._data_timestamps['odom'] = _get_ros_time_sec()
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        uc_imu = UcImu(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            orientation=(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ),
            angular_velocity=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ),
            linear_acceleration=(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            )
        )
        
        with self._lock:
            self._latest_imu = uc_imu
            self._data_timestamps['imu'] = _get_ros_time_sec()
    
    def _traj_callback(self, msg: LocalTrajectoryV4):
        """轨迹回调"""
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in msg.points
        ]
        num_points = len(points)
        
        # 处理速度数组 (与 TrajectoryAdapter 保持一致)
        velocities = None
        soft_enabled = msg.soft_enabled
        
        if soft_enabled and len(msg.velocities_flat) > 0:
            flat_len = len(msg.velocities_flat)
            
            # 检查长度是否为 4 的倍数
            if flat_len % 4 != 0:
                rospy.logwarn_throttle(5.0,
                    f"velocities_flat length {flat_len} is not a multiple of 4, truncating")
                flat_len = (flat_len // 4) * 4
            
            if flat_len > 0:
                velocities = np.array(msg.velocities_flat[:flat_len]).reshape(-1, 4)
                num_vel_points = velocities.shape[0]
                
                # 检查速度点数与位置点数是否匹配
                if num_vel_points != num_points:
                    if num_vel_points > num_points:
                        # 速度点多于位置点，截断
                        velocities = velocities[:num_points]
                    else:
                        # 速度点少于位置点，使用最后一个速度点填充
                        rospy.logwarn_throttle(5.0,
                            f"Velocity points ({num_vel_points}) < position points ({num_points}), "
                            f"padding with last velocity")
                        last_vel = velocities[-1:, :]
                        padding = np.tile(last_vel, (num_points - num_vel_points, 1))
                        velocities = np.vstack([velocities, padding])
            else:
                # 无有效速度数据，禁用 soft 模式
                soft_enabled = False
        elif soft_enabled:
            # soft_enabled=True 但没有速度数据，禁用 soft 模式
            soft_enabled = False
        
        # 转换轨迹模式
        try:
            mode = TrajectoryMode(msg.mode)
        except ValueError:
            mode = TrajectoryMode.MODE_TRACK
        
        uc_traj = UcTrajectory(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=msg.dt_sec,
            confidence=msg.confidence,
            mode=mode,
            soft_enabled=soft_enabled
        )
        
        with self._lock:
            self._latest_traj = uc_traj
            self._data_timestamps['trajectory'] = _get_ros_time_sec()
    
    def _control_callback(self, event):
        """控制循环回调"""
        # 1. 获取最新数据
        now = _get_ros_time_sec()
        with self._lock:
            odom = self._latest_odom
            imu = self._latest_imu
            trajectory = self._latest_traj
            odom_age = now - self._data_timestamps.get('odom', 0) if 'odom' in self._data_timestamps else float('inf')
            traj_age = now - self._data_timestamps.get('trajectory', 0) if 'trajectory' in self._data_timestamps else float('inf')
            imu_age = now - self._data_timestamps.get('imu', 0) if 'imu' in self._data_timestamps else float('inf')
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if self._waiting_for_data:
                rospy.logwarn_throttle(5.0, "Waiting for odom and trajectory data...")
            return
        
        if self._waiting_for_data:
            rospy.loginfo("Data received, starting control")
            self._waiting_for_data = False
        
        # 3. 检查数据新鲜度
        if odom_age > self._max_odom_age:
            rospy.logwarn_throttle(1.0, f"Odom timeout: age={odom_age*1000:.1f}ms")
        
        # 4. 执行控制更新
        try:
            cmd = self._manager.update(odom, trajectory, imu)
        except Exception as e:
            rospy.logerr(f"Controller update failed: {e}")
            self._publish_stop_cmd()
            # 发布错误诊断信息
            error_diag = {
                'state': 0,  # INIT/ERROR state
                'mpc_success': False,
                'backup_active': False,
                'error_message': str(e),
                'timeout': {
                    'odom_timeout': odom_age > self._max_odom_age,
                    'traj_timeout': traj_age > self._max_traj_age,
                    'imu_timeout': imu_age > self._max_imu_age,
                    'last_odom_age_ms': odom_age * 1000,
                    'last_traj_age_ms': traj_age * 1000,
                    'last_imu_age_ms': imu_age * 1000,
                },
                'cmd': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'omega': 0.0},
            }
            self._publish_diagnostics(error_diag)
            return
        
        # 5. 发布控制命令
        self._publish_cmd(cmd)
    
    def _on_diagnostics(self, diag: Dict[str, Any]):
        """诊断回调"""
        self._publish_diagnostics(diag)
    
    # ==================== 发布函数 ====================
    
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        msg = UnifiedCmd()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = cmd.frame_id or self._default_frame_id
        msg.vx = float(cmd.vx)
        msg.vy = float(cmd.vy)
        msg.vz = float(cmd.vz)
        msg.omega = float(cmd.omega)
        msg.success = cmd.success
        msg.solve_time_ms = float(cmd.solve_time_ms)
        self._cmd_pub.publish(msg)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        msg = UnifiedCmd()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._default_frame_id
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0
        msg.omega = 0.0
        msg.success = True
        msg.solve_time_ms = 0.0
        self._cmd_pub.publish(msg)
    
    def _publish_diagnostics(self, diag: Dict[str, Any]):
        """发布诊断信息"""
        msg = DiagnosticsV2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'controller'
        
        msg.state = diag.get('state', 0)
        msg.mpc_success = diag.get('mpc_success', False)
        msg.mpc_solve_time_ms = float(diag.get('mpc_solve_time_ms', 0.0))
        msg.backup_active = diag.get('backup_active', False)
        
        # MPC 健康状态
        mpc_health = diag.get('mpc_health', {})
        msg.mpc_health_kkt_residual = float(mpc_health.get('kkt_residual', 0.0))
        msg.mpc_health_condition_number = float(mpc_health.get('condition_number', 1.0))
        msg.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
        msg.mpc_health_degradation_warning = mpc_health.get('degradation_warning', False)
        msg.mpc_health_can_recover = mpc_health.get('can_recover', True)
        
        # 一致性指标
        consistency = diag.get('consistency', {})
        msg.consistency_curvature = float(consistency.get('curvature', 0.0))
        msg.consistency_velocity_dir = float(consistency.get('velocity_dir', 1.0))
        msg.consistency_temporal = float(consistency.get('temporal', 1.0))
        msg.consistency_alpha_soft = float(consistency.get('alpha_soft', 0.0))
        msg.consistency_data_valid = consistency.get('data_valid', True)
        
        # 状态估计器健康
        estimator = diag.get('estimator_health', {})
        msg.estimator_covariance_norm = float(estimator.get('covariance_norm', 0.0))
        msg.estimator_innovation_norm = float(estimator.get('innovation_norm', 0.0))
        msg.estimator_slip_probability = float(estimator.get('slip_probability', 0.0))
        msg.estimator_imu_drift_detected = estimator.get('imu_drift_detected', False)
        msg.estimator_imu_available = estimator.get('imu_available', True)
        
        # IMU bias
        imu_bias = estimator.get('imu_bias', [0.0, 0.0, 0.0])
        if isinstance(imu_bias, (list, tuple)) and len(imu_bias) >= 3:
            msg.estimator_imu_bias = [float(imu_bias[0]), float(imu_bias[1]), float(imu_bias[2])]
        else:
            msg.estimator_imu_bias = [0.0, 0.0, 0.0]
        
        # 跟踪误差
        tracking = diag.get('tracking', {})
        msg.tracking_lateral_error = float(tracking.get('lateral_error', 0.0))
        msg.tracking_longitudinal_error = float(tracking.get('longitudinal_error', 0.0))
        msg.tracking_heading_error = float(tracking.get('heading_error', 0.0))
        msg.tracking_prediction_error = float(tracking.get('prediction_error', 0.0))
        
        # 坐标变换状态
        transform = diag.get('transform', {})
        msg.transform_tf2_available = transform.get('tf2_available', self._tf_bridge.is_initialized)
        msg.transform_fallback_duration_ms = float(transform.get('fallback_duration_ms', 0.0))
        msg.transform_accumulated_drift = float(transform.get('accumulated_drift', 0.0))
        
        # 超时状态
        timeout = diag.get('timeout', {})
        msg.timeout_odom = timeout.get('odom_timeout', False)
        msg.timeout_traj = timeout.get('traj_timeout', False)
        msg.timeout_traj_grace_exceeded = timeout.get('traj_grace_exceeded', False)
        msg.timeout_imu = timeout.get('imu_timeout', False)
        msg.timeout_last_odom_age_ms = float(timeout.get('last_odom_age_ms', 0.0))
        msg.timeout_last_traj_age_ms = float(timeout.get('last_traj_age_ms', 0.0))
        msg.timeout_last_imu_age_ms = float(timeout.get('last_imu_age_ms', 0.0))
        msg.timeout_in_startup_grace = timeout.get('in_startup_grace', False)
        
        # 控制命令
        cmd = diag.get('cmd', {})
        msg.cmd_vx = float(cmd.get('vx', 0.0))
        msg.cmd_vy = float(cmd.get('vy', 0.0))
        msg.cmd_vz = float(cmd.get('vz', 0.0))
        msg.cmd_omega = float(cmd.get('omega', 0.0))
        msg.cmd_frame_id = cmd.get('frame_id', '')
        
        msg.transition_progress = float(diag.get('transition_progress', 0.0))
        
        self._diag_pub.publish(msg)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset(self, req):
        """处理重置请求"""
        try:
            self._manager.reset()
            self._waiting_for_data = True
            rospy.loginfo("Controller reset")
            return TriggerResponse(success=True, message="Controller reset successfully")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Reset failed: {e}")
    
    def shutdown(self):
        """关闭节点"""
        self._manager.shutdown()
        rospy.loginfo("Controller node shutdown")


def main():
    try:
        node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()


if __name__ == '__main__':
    main()
