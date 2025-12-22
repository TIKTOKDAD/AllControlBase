"""超时监控器"""
from typing import Dict, Any, Optional
import time

from ..core.data_types import TimeoutStatus


def _get_monotonic_time() -> float:
    """
    获取单调时钟时间（秒）
    
    使用 time.monotonic() 而非 time.time()，避免系统时间跳变
    （如 NTP 同步）导致的时间间隔计算错误。
    """
    return time.monotonic()


# 表示"从未收到消息"的超时值 (毫秒)
# 使用大的有限值代替无穷大，避免 JSON 序列化问题
# 1e9 ms ≈ 11.5 天，足够表示"非常长时间"
NEVER_RECEIVED_AGE_MS = 1e9


class TimeoutMonitor:
    """超时监控器
    
    注意：所有时间戳使用单调时钟 (time.monotonic())，避免系统时间跳变影响。
    消息的 stamp 字段仅用于日志记录，不参与超时计算。
    """
    
    def __init__(self, config: Dict[str, Any]):
        watchdog_config = config.get('watchdog', {})
        self.odom_timeout_ms = watchdog_config.get('odom_timeout_ms', 200)
        self.traj_timeout_ms = watchdog_config.get('traj_timeout_ms', 200)
        self.traj_grace_ms = watchdog_config.get('traj_grace_ms', 100)
        self.imu_timeout_ms = watchdog_config.get('imu_timeout_ms', 100)
        self.startup_grace_ms = watchdog_config.get('startup_grace_ms', 1000)
        
        self._startup_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
        self._last_traj_time: Optional[float] = None
        self._last_imu_time: Optional[float] = None
        self._traj_timeout_start: Optional[float] = None
    
    def update_odom(self, stamp: float) -> None:
        """更新 odom 时间戳
        
        Args:
            stamp: 消息时间戳（秒），仅用于日志，内部使用接收时间
        """
        receive_time = _get_monotonic_time()
        self._last_odom_time = receive_time
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def update_trajectory(self, stamp: float) -> None:
        """更新轨迹时间戳"""
        receive_time = _get_monotonic_time()
        self._last_traj_time = receive_time
        self._traj_timeout_start = None
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def update_imu(self, stamp: float) -> None:
        """更新 IMU 时间戳"""
        receive_time = _get_monotonic_time()
        self._last_imu_time = receive_time
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def check(self, current_time: float = None) -> TimeoutStatus:
        """检查所有超时状态
        
        Args:
            current_time: 当前时间（秒），如果为 None 则使用单调时钟
                         注意：为保持一致性，建议不传入此参数
        """
        # 使用单调时钟，忽略外部传入的时间
        monotonic_now = _get_monotonic_time()
        
        in_startup_grace = False
        if self._startup_time is None:
            in_startup_grace = True
        else:
            startup_elapsed_ms = (monotonic_now - self._startup_time) * 1000
            in_startup_grace = startup_elapsed_ms < self.startup_grace_ms
        
        if in_startup_grace:
            return TimeoutStatus(
                odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
                imu_timeout=False, last_odom_age_ms=0.0, last_traj_age_ms=0.0,
                last_imu_age_ms=0.0, in_startup_grace=True
            )
        
        odom_age_ms = self._compute_age_ms(self._last_odom_time, monotonic_now)
        traj_age_ms = self._compute_age_ms(self._last_traj_time, monotonic_now)
        imu_age_ms = self._compute_age_ms(self._last_imu_time, monotonic_now)
        
        odom_timeout = odom_age_ms > self.odom_timeout_ms
        imu_timeout = imu_age_ms > self.imu_timeout_ms
        
        traj_timeout = traj_age_ms > self.traj_timeout_ms
        traj_grace_exceeded = False
        
        if traj_timeout:
            if self._traj_timeout_start is None:
                self._traj_timeout_start = monotonic_now
            grace_elapsed_ms = (monotonic_now - self._traj_timeout_start) * 1000
            traj_grace_exceeded = grace_elapsed_ms > self.traj_grace_ms
        else:
            self._traj_timeout_start = None
        
        return TimeoutStatus(
            odom_timeout=odom_timeout,
            traj_timeout=traj_timeout,
            traj_grace_exceeded=traj_grace_exceeded,
            imu_timeout=imu_timeout,
            last_odom_age_ms=odom_age_ms,
            last_traj_age_ms=traj_age_ms,
            last_imu_age_ms=imu_age_ms,
            in_startup_grace=False
        )
    
    def _compute_age_ms(self, last_time: Optional[float], current_time: float) -> float:
        if last_time is None:
            return NEVER_RECEIVED_AGE_MS
        return (current_time - last_time) * 1000
    
    def reset(self) -> None:
        self._last_odom_time = None
        self._last_traj_time = None
        self._last_imu_time = None
        self._traj_timeout_start = None
        self._startup_time = None
