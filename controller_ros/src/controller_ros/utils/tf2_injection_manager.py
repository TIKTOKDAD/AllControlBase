"""
TF2 注入管理器

管理 TF2 回调注入到 ControllerManager 的坐标变换器中。

功能:
- 初始注入: 在节点启动时尝试注入 TF2 回调
- 自动重试: 注入失败时按指数退避策略重试
- 状态跟踪: 提供注入状态查询接口

设计说明:
- 使用 threading.Event 实现线程安全的状态标志
- 支持可配置的重试策略（间隔、退避、最大次数）
- 与 ControllerNodeBase 解耦，可独立测试

配置参数 (从 transform.* 读取):
- buffer_warmup_timeout_sec: TF buffer 预热超时 (默认 2.0s)
- buffer_warmup_interval_sec: TF buffer 预热检查间隔 (默认 0.1s)
- retry_interval_sec: 初始重试间隔 (默认 1.0s)
- max_retry_interval_sec: 最大重试间隔 (默认 30.0s)
- backoff_multiplier: 指数退避倍数 (默认 2.0)
- max_retries: 最大重试次数，-1 表示无限 (默认 -1)
"""
from typing import Any, Callable, Dict, Optional
import threading
import time
import logging

logger = logging.getLogger(__name__)

# 默认配置
DEFAULT_CONFIG = {
    'buffer_warmup_timeout_sec': 2.0,
    'buffer_warmup_interval_sec': 0.1,
    'skip_buffer_warmup': False,  # 跳过 buffer 预热（用于测试或已知 TF2 可用的场景）
    'retry_interval_sec': 1.0,
    'max_retry_interval_sec': 30.0,
    'backoff_multiplier': 2.0,
    'max_retries': -1,  # -1 表示无限重试
}

DEFAULT_TRANSFORM_CONFIG = {
    'source_frame': 'base_link',
    'target_frame': 'odom',
}


class TF2InjectionManager:
    """
    TF2 注入管理器
    
    职责:
    - 管理 TF2 回调到 CoordTransformer 的注入
    - 处理注入失败的重试逻辑
    - 提供线程安全的状态查询
    
    使用示例:
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=controller_bridge.manager,
        )
        
        # 初始注入
        if injection_manager.inject(blocking=True):
            print("TF2 injected successfully")
        
        # 在控制循环中检查是否需要重试
        injection_manager.try_reinjection_if_needed()
    """
    
    def __init__(
        self,
        tf_bridge: Any,
        controller_manager: Any,
        config: Optional[Dict[str, Any]] = None,
        transform_config: Optional[Dict[str, Any]] = None,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        get_time_func: Optional[Callable[[], float]] = None,
    ):
        """
        初始化 TF2 注入管理器
        
        Args:
            tf_bridge: TF2 桥接实例 (TFBridge)
            controller_manager: 控制器管理器实例 (ControllerManager)
            config: 注入配置字典
            transform_config: 坐标变换配置字典
            log_info: 信息日志函数
            log_warn: 警告日志函数
            get_time_func: 获取当前时间的函数
        """
        self._tf_bridge = tf_bridge
        self._controller_manager = controller_manager
        
        # 合并配置
        cfg = {**DEFAULT_CONFIG, **(config or {})}
        tf_cfg = {**DEFAULT_TRANSFORM_CONFIG, **(transform_config or {})}
        
        # 配置参数
        self._buffer_warmup_timeout_sec = cfg['buffer_warmup_timeout_sec']
        self._buffer_warmup_interval_sec = cfg['buffer_warmup_interval_sec']
        self._skip_buffer_warmup = cfg['skip_buffer_warmup']
        self._initial_retry_interval = cfg['retry_interval_sec']
        self._max_retry_interval = cfg['max_retry_interval_sec']
        self._backoff_multiplier = cfg['backoff_multiplier']
        self._max_retries = cfg['max_retries']
        
        # 坐标系配置
        self._source_frame = tf_cfg['source_frame']
        self._target_frame = tf_cfg['target_frame']
        
        # 日志函数
        self._log_info = log_info or logger.info
        self._log_warn = log_warn or logger.warning
        
        # 时间函数
        self._get_time = get_time_func or time.monotonic
        
        # 线程安全状态 (使用 Event 实现无锁读取)
        self._injected_event = threading.Event()
        self._injection_attempted_event = threading.Event()
        
        # 需要锁保护的状态
        self._lock = threading.Lock()
        self._retry_count = 0
        self._last_retry_time: Optional[float] = None
        self._current_retry_interval = self._initial_retry_interval
    
    @property
    def is_injected(self) -> bool:
        """TF2 是否已成功注入 (线程安全，无锁)"""
        return self._injected_event.is_set()
    
    @property
    def injection_attempted(self) -> bool:
        """是否已尝试过注入 (线程安全，无锁)"""
        return self._injection_attempted_event.is_set()
    
    @property
    def retry_count(self) -> int:
        """重试次数"""
        with self._lock:
            return self._retry_count
    
    def inject(self, blocking: bool = True) -> bool:
        """
        执行 TF2 注入
        
        Args:
            blocking: 是否阻塞等待 TF buffer 预热
        
        Returns:
            是否注入成功
        """
        self._injection_attempted_event.set()
        
        # 检查前置条件
        if self._tf_bridge is None:
            self._log_warn("TF2 injection failed: tf_bridge is None")
            return False
        
        if not getattr(self._tf_bridge, 'is_initialized', False):
            self._log_warn("TF2 injection failed: tf_bridge not initialized")
            return False
        
        if self._controller_manager is None:
            self._log_warn("TF2 injection failed: controller_manager is None")
            return False
        
        coord_transformer = getattr(self._controller_manager, 'coord_transformer', None)
        if coord_transformer is None:
            self._log_warn("TF2 injection failed: coord_transformer not available")
            return False
        
        if not hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            self._log_warn("TF2 injection failed: coord_transformer missing set_tf2_lookup_callback")
            return False
        
        # 可选: 等待 TF buffer 预热
        # 注意: 即使预热失败，我们仍然注入回调，让 RobustCoordinateTransformer 的 fallback 机制处理
        if blocking and not self._skip_buffer_warmup:
            if not self._wait_for_buffer_warmup():
                self._log_warn(
                    "TF2 buffer warmup timeout. Injecting callback anyway - "
                    "RobustCoordinateTransformer will use fallback until TF2 is ready."
                )
        
        # 执行注入
        try:
            coord_transformer.set_tf2_lookup_callback(self._tf_bridge.lookup_transform)
            self._injected_event.set()
            
            # 重置重试状态
            with self._lock:
                self._current_retry_interval = self._initial_retry_interval
            
            self._log_info(
                f"TF2 callback injected successfully "
                f"({self._source_frame} -> {self._target_frame})"
            )
            return True
            
        except Exception as e:
            self._log_warn(f"TF2 injection failed with exception: {e}")
            return False
    
    def _wait_for_buffer_warmup(self) -> bool:
        """
        等待 TF buffer 预热
        
        Returns:
            是否在超时前完成预热
        """
        if not hasattr(self._tf_bridge, 'can_transform'):
            return True  # 无法检查，假设已就绪
        
        start_time = self._get_time()
        timeout = self._buffer_warmup_timeout_sec
        interval = self._buffer_warmup_interval_sec
        
        while self._get_time() - start_time < timeout:
            try:
                if self._tf_bridge.can_transform(
                    self._target_frame, 
                    self._source_frame,
                    timeout_sec=0.01
                ):
                    return True
            except Exception:
                pass
            time.sleep(interval)
        
        return False
    
    def try_reinjection_if_needed(self) -> bool:
        """
        检查是否需要重试注入，如果需要则执行
        
        此方法设计为在控制循环中调用，开销极低。
        
        Returns:
            是否执行了重试（不代表重试成功）
        """
        # 快速路径: 已注入或未尝试过，无需重试
        if self._injected_event.is_set():
            return False
        
        if not self._injection_attempted_event.is_set():
            return False
        
        # 检查重试条件
        now = self._get_time()
        
        with self._lock:
            # 检查最大重试次数
            if self._max_retries >= 0 and self._retry_count >= self._max_retries:
                return False
            
            # 检查重试间隔
            if self._last_retry_time is not None:
                elapsed = now - self._last_retry_time
                if elapsed < self._current_retry_interval:
                    return False
            
            # 更新重试状态
            self._retry_count += 1
            self._last_retry_time = now
            
            # 指数退避
            self._current_retry_interval = min(
                self._current_retry_interval * self._backoff_multiplier,
                self._max_retry_interval
            )
            retry_count = self._retry_count
            current_interval = self._current_retry_interval
        
        # 在锁外执行注入
        self._log_info(
            f"TF2 reinjection attempt #{retry_count} "
            f"(next interval: {current_interval:.1f}s)"
        )
        
        success = self.inject(blocking=False)
        
        if success:
            self._log_info(f"TF2 reinjection succeeded after {retry_count} attempts")
        
        return True
    
    def reset(self) -> None:
        """
        重置重试状态
        
        注意: 不重置注入状态，因为回调仍然绑定在 coord_transformer 上
        """
        with self._lock:
            self._last_retry_time = None
            # 不重置 retry_count，保留历史记录
            # 不重置 current_retry_interval，保持当前退避级别
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取注入状态
        
        Returns:
            状态字典
        """
        with self._lock:
            return {
                'injected': self._injected_event.is_set(),
                'injection_attempted': self._injection_attempted_event.is_set(),
                'retry_count': self._retry_count,
                'current_retry_interval_sec': self._current_retry_interval,
                'source_frame': self._source_frame,
                'target_frame': self._target_frame,
                'tf_bridge_available': self._tf_bridge is not None,
                'tf_bridge_initialized': (
                    getattr(self._tf_bridge, 'is_initialized', False)
                    if self._tf_bridge else False
                ),
            }
