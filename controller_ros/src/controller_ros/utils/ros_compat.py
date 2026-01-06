"""
ROS 兼容层

支持 ROS1 (Noetic) 和 ROS2 (Humble) 双版本运行。

使用方法:
    from controller_ros.utils.ros_compat import (
        ROS_VERSION, ROS_AVAILABLE, TF2_AVAILABLE,
        get_time_sec, ros_time_to_sec, sec_to_ros_time
    )
"""
import os
import time
import logging
import threading
from collections import OrderedDict
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# ============================================================================
# ROS 版本检测
# ============================================================================

ROS_VERSION = 0  # 0=无, 1=ROS1, 2=ROS2
ROS_AVAILABLE = False
TF2_AVAILABLE = False

# 1. 优先使用环境变量 (标准做法)
env_ros_version = os.environ.get('ROS_VERSION')
if env_ros_version == '1':
    ROS_VERSION = 1
    ROS_AVAILABLE = True
elif env_ros_version == '2':
    ROS_VERSION = 2
    ROS_AVAILABLE = True

# 2. 如果环境变量未设置，尝试自动检测
if ROS_VERSION == 0:
    # 尝试检测 ROS2
    try:
        import rclpy
        from rclpy.node import Node
        ROS_VERSION = 2
        ROS_AVAILABLE = True
        logger.debug("ROS2 detected (via import)")
    except ImportError:
        pass

    # 尝试检测 ROS1
    if ROS_VERSION == 0:
        try:
            import rospy
            ROS_VERSION = 1
            ROS_AVAILABLE = True
            logger.debug("ROS1 detected (via import)")
        except ImportError:
            pass
else:
    # 根据环境变量尝试导入对应的库，确保环境一致性
    if ROS_VERSION == 2:
        try:
            import rclpy
            from rclpy.node import Node
            logger.debug("ROS2 environment detected")
        except ImportError:
             logger.warning("ROS_VERSION=2 but failed to import rclpy")
             ROS_AVAILABLE = False
    elif ROS_VERSION == 1:
        try:
            import rospy
            logger.debug("ROS1 environment detected")
        except ImportError:
            logger.warning("ROS_VERSION=1 but failed to import rospy")
            ROS_AVAILABLE = False

# 检测 TF2
if ROS_VERSION == 2:
    try:
        import tf2_ros
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        TF2_AVAILABLE = True
    except ImportError:
        TF2_AVAILABLE = False
elif ROS_VERSION == 1:
    try:
        import tf2_ros
        TF2_AVAILABLE = True
    except ImportError:
        TF2_AVAILABLE = False


# ============================================================================
# 时间工具 (统一接口)
# ============================================================================

def get_time_sec(node=None) -> float:
    """
    获取当前 ROS 时间（秒）- 统一接口
    
    支持仿真时间模式。当时钟返回 0 时（仿真时间未初始化），回退到系统时间。
    
    Args:
        node: ROS2 节点实例 (ROS2 必需，ROS1 可选)
    
    Returns:
        当前时间（秒）
    
    Note:
        - ROS1: 不需要 node 参数，直接使用 rospy.Time.now()
        - ROS2: 需要 node 参数来获取节点时钟。如果 node 为 None，回退到系统时间。
    """
    if ROS_VERSION == 1:
        try:
            import rospy
            ros_time = rospy.Time.now().to_sec()
            # 仿真时间模式下可能为 0
            return ros_time if ros_time > 0 else time.time()
        except Exception:
            return time.time()
    elif ROS_VERSION == 2:
        if node is not None:
            try:
                clock_time = node.get_clock().now().nanoseconds * 1e-9
                # 仿真时间模式下可能为 0
                return clock_time if clock_time > 0 else time.time()
            except Exception:
                return time.time()
        else:
            # ROS2 需要节点实例来获取时钟，回退到系统时间
            return time.time()
    return time.time()


def get_monotonic_time() -> float:
    """获取单调时钟时间（秒），用于计算时间间隔"""
    return time.monotonic()


def ros_time_to_sec(stamp) -> float:
    """
    将 ROS 时间戳转换为秒
    
    支持 ROS1 (secs/nsecs 或 to_sec()) 和 ROS2 (sec/nanosec) 格式
    也支持测试环境中的 Mock 对象
    """
    # 快速路径: float
    if isinstance(stamp, (float, int)):
        return float(stamp)

    # ROS2 (builtin_interfaces/Time) - Most likely in ROS2 env
    if hasattr(stamp, 'nanosec'):
        return stamp.sec + stamp.nanosec * 1e-9
        
    # ROS1 (rospy.Time) - Has to_sec method
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    
    # Fallback for ROS1 raw msg structures (secs/nsecs)
    if hasattr(stamp, 'nsecs'):
        return stamp.secs + stamp.nsecs * 1e-9
        
    return float(stamp)


def sec_to_ros_time(sec: float):
    """
    将秒转换为 ROS 时间戳
    
    支持 ROS1 和 ROS2
    """
    if ROS_VERSION == 1:
        import rospy
        return rospy.Time.from_sec(sec)
    elif ROS_VERSION == 2:
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sec)
        t.nanosec = int((sec - t.sec) * 1e9)
        return t
    return sec


# ============================================================================
# 日志工具
# ============================================================================

def _setup_logging_for_ros():
    """配置 logging 以便在 ROS 环境下正确输出"""
    if ROS_VERSION == 1:
        # ROS1: 配置 logging 使用 rospy 的日志系统
        import rospy
        
        class RospyHandler(logging.Handler):
            """将 Python logging 转发到 rospy"""
            def emit(self, record):
                try:
                    msg = self.format(record)
                    if record.levelno >= logging.ERROR:
                        rospy.logerr(msg)
                    elif record.levelno >= logging.WARNING:
                        rospy.logwarn(msg)
                    elif record.levelno >= logging.INFO:
                        rospy.loginfo(msg)
                    else:
                        rospy.logdebug(msg)
                except Exception:
                    self.handleError(record)
        
        # 为 controller_ros 模块配置 handler
        ros_handler = RospyHandler()
        ros_handler.setFormatter(logging.Formatter('%(name)s: %(message)s'))
        
        for module_name in ['controller_ros', 'universal_controller']:
            module_logger = logging.getLogger(module_name)
            # 避免重复添加 handler
            if not any(isinstance(h, RospyHandler) for h in module_logger.handlers):
                module_logger.addHandler(ros_handler)
                module_logger.setLevel(logging.DEBUG)


# 在模块加载时配置日志
if ROS_VERSION == 1:
    try:
        _setup_logging_for_ros()
    except Exception:
        pass  # 如果 rospy 未初始化，跳过配置


def setup_ros2_logging(node):
    """
    配置 logging 以便在 ROS2 环境下正确输出到 /rosout
    
    Args:
        node: ROS2 Node 实例
    """
    if ROS_VERSION != 2:
        return

    class Ros2Handler(logging.Handler):
        """将 Python logging 转发到 ROS2 Node Logger"""
        def __init__(self, node_instance):
            super().__init__()
            self._node = node_instance
            
        def emit(self, record):
            try:
                msg = self.format(record)
                # 使用 record.name 作为前缀，保留模块信息，看起来像: [universal_controller.mpc] Message...
                # 注意: 不使用 node.get_logger().get_child() 因为那会创建大量 logger 对象
                ros_msg = f"[{record.name}] {msg}"
                
                # 映射日志级别
                if record.levelno >= logging.ERROR:
                    self._node.get_logger().error(ros_msg)
                elif record.levelno >= logging.WARNING:
                    self._node.get_logger().warn(ros_msg)
                elif record.levelno >= logging.INFO:
                    self._node.get_logger().info(ros_msg)
                else:
                    self._node.get_logger().debug(ros_msg)
            except Exception:
                self.handleError(record)

    # 创建并配置 Handler
    handler = Ros2Handler(node)
    # 消息格式只包含消息本身，因为 Ros2Handler 会添加 name 前缀，ROS 自身会添加时间戳
    handler.setFormatter(logging.Formatter('%(message)s'))
    
    # 为关键模块配置日志
    for module_name in ['controller_ros', 'universal_controller']:
        logger_obj = logging.getLogger(module_name)
        
        # 避免重复添加 (幂等性)
        if not any(isinstance(h, Ros2Handler) for h in logger_obj.handlers):
            logger_obj.addHandler(handler)
            # 设置为 DEBUG，让 ROS 2 的 verbosity 级别 (通过 launch 或 ros2 param) 来决定最终显示
            logger_obj.setLevel(logging.DEBUG) 
            # 停止传播，防止同时打印到 stdout (造成双重日志)
            logger_obj.propagate = False
            
    # 记录一条调试信息确认配置成功
    node.get_logger().debug("Python logging bridged to ROS 2 /rosout")


def log_info(msg: str):
    """记录信息日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.loginfo(msg)
    elif ROS_VERSION == 2:
        logger.info(msg)
    else:
        print(f"[INFO] {msg}")


def log_warn(msg: str):
    """记录警告日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logwarn(msg)
    elif ROS_VERSION == 2:
        logger.warning(msg)
    else:
        print(f"[WARN] {msg}")


def log_error(msg: str):
    """记录错误日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logerr(msg)
    elif ROS_VERSION == 2:
        logger.error(msg)
    else:
        print(f"[ERROR] {msg}")


def log_warn_throttle(period: float, msg: str):
    """节流警告日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logwarn_throttle(period, msg)
    else:
        _log_throttle('warn', period, msg)


class _ThrottleState:
    """节流日志状态管理（单例）"""
    
    _instance = None
    _lock = threading.Lock()
    
    # 配置常量
    MAX_CACHE_SIZE = 100
    CACHE_EXPIRE_SEC = 300.0
    CLEANUP_INTERVAL_SEC = 60.0
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._init()
        return cls._instance
    
    def _init(self):
        self._cache: OrderedDict = OrderedDict()
        self._last_cleanup = 0.0
    
    def should_log(self, key: tuple, period: float) -> bool:
        """检查是否应该记录日志"""
        now = time.monotonic()
        
        with self._lock:
            # 定期清理
            if now - self._last_cleanup > self.CLEANUP_INTERVAL_SEC:
                self._cleanup(now)
                self._last_cleanup = now
            
            last_time = self._cache.get(key, 0)
            if now - last_time >= period:
                self._cache[key] = now
                self._cache.move_to_end(key)
                
                # LRU 清理
                while len(self._cache) > self.MAX_CACHE_SIZE:
                    self._cache.popitem(last=False)
                
                return True
            return False
    
    def _cleanup(self, now: float):
        """清理过期条目"""
        expired = [k for k, t in self._cache.items() 
                   if now - t > self.CACHE_EXPIRE_SEC]
        for k in expired:
            del self._cache[k]


_throttle_state = _ThrottleState()


def _log_throttle(level: str, period: float, msg: str):
    """
    节流日志实现（线程安全）
    
    使用单例 _ThrottleState 管理缓存状态。
    
    Args:
        level: 日志级别 ('info', 'warn', 'error')
        period: 节流周期（秒）
        msg: 日志消息
    """
    # 生成缓存键
    if len(msg) > 200:
        import hashlib
        key = (level, hashlib.sha256(msg.encode()).hexdigest())
    else:
        key = (level, msg)
    
    if _throttle_state.should_log(key, period):
        if level == 'info':
            logger.info(msg)
        elif level == 'warn':
            logger.warning(msg)
        elif level == 'error':
            logger.error(msg)


# ============================================================================
# TF2 兼容层
# ============================================================================

class TF2Compat:
    """TF2 兼容层，支持 ROS1 和 ROS2"""
    
    def __init__(self, node=None):
        """
        初始化 TF2 兼容层
        
        Args:
            node: ROS2 节点实例 (ROS1 不需要)
        """
        self._node = node
        self._buffer = None
        self._listener = None
        self._initialized = False
        
        self._initialize()
    
    def _initialize(self):
        """初始化 TF2"""
        if not TF2_AVAILABLE:
            logger.warning("TF2 not available")
            return
        
        try:
            if ROS_VERSION == 1:
                import tf2_ros
                self._buffer = tf2_ros.Buffer()
                self._listener = tf2_ros.TransformListener(self._buffer)
            elif ROS_VERSION == 2:
                from tf2_ros.buffer import Buffer
                from tf2_ros.transform_listener import TransformListener
                self._buffer = Buffer()
                self._listener = TransformListener(self._buffer, self._node)
            
            self._initialized = True
            logger.info("TF2 initialized")
        except Exception as e:
            logger.error(f"TF2 initialization failed: {e}")
            self._initialized = False
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time=None, timeout_sec: float = 0.0) -> Optional[dict]:
        """
        查询坐标变换
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 时间戳，可以是:
                - None: 使用最新可用的变换 (rospy.Time(0))
                - float: 秒数，会被转换为 ROS Time
                - rospy.Time / rclpy.time.Time: 直接使用
            timeout_sec: 超时时间 (秒)
        
        Returns:
            {'translation': (x, y, z), 'rotation': (x, y, z, w)} 或 None
        """
        if not self._initialized or self._buffer is None:
            return None
        
        try:
            if ROS_VERSION == 1:
                import rospy
                import tf2_ros
                
                # 处理时间参数
                if time is None:
                    ros_time = rospy.Time(0)  # 最新可用的变换
                elif isinstance(time, (int, float)):
                    # float 秒数 -> 转换为 ROS Time
                    ros_time = rospy.Time.from_sec(time)
                else:
                    ros_time = time  # 假设已经是 rospy.Time
                
                transform = self._buffer.lookup_transform(
                    target_frame, source_frame, ros_time,
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
            
            elif ROS_VERSION == 2:
                import tf2_ros
                from rclpy.time import Time
                from rclpy.duration import Duration
                
                # 处理时间参数
                if time is None:
                    ros_time = Time()  # 最新可用的变换
                elif isinstance(time, (int, float)):
                    # float 秒数 -> 转换为 rclpy Time
                    sec = int(time)
                    nanosec = int((time - sec) * 1e9)
                    ros_time = Time(seconds=sec, nanoseconds=nanosec)
                else:
                    ros_time = time  # 假设已经是 rclpy.time.Time
                
                transform = self._buffer.lookup_transform(
                    target_frame, source_frame, ros_time,
                    timeout=Duration(seconds=timeout_sec)
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
        
        except Exception as e:
            logger.debug(f"TF lookup failed: {source_frame} -> {target_frame}: {e}")
            return None
        
        return None
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.0) -> bool:
        """检查是否可以进行坐标变换"""
        if not self._initialized or self._buffer is None:
            return False
        
        try:
            if ROS_VERSION == 1:
                import rospy
                # 处理时间参数
                if time is None or isinstance(time, (int, float)):
                    ros_time = rospy.Time(0)
                else:
                    ros_time = time
                return self._buffer.can_transform(
                    target_frame, source_frame, ros_time,
                    timeout=rospy.Duration(timeout_sec)
                )
            elif ROS_VERSION == 2:
                from rclpy.time import Time
                from rclpy.duration import Duration
                # 处理时间参数
                if time is None or isinstance(time, (int, float)):
                    ros_time = Time()
                else:
                    ros_time = time
                return self._buffer.can_transform(
                    target_frame, source_frame, ros_time,
                    timeout=Duration(seconds=timeout_sec)
                )
        except Exception:
            return False
        
        return False
    
    def shutdown(self) -> None:
        """
        关闭 TF2 兼容层，释放资源
        
        ROS1: TransformListener 会自动清理
        ROS2: TransformListener 绑定到节点，节点销毁时自动清理
        """
        # 清理引用，让 GC 回收资源
        self._listener = None
        self._buffer = None
        self._node = None
        self._initialized = False
        logger.debug("TF2Compat shutdown complete")
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
    
    @property
    def buffer(self):
        return self._buffer
