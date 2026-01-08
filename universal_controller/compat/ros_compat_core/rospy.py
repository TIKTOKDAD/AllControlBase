"""
Standalone implementation of rospy for non-ROS environments.
"""
import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)

class StandaloneRospy:
    """
    rospy 模块的独立运行替代实现
    
    提供 Time, Duration 类和日志函数，用于非 ROS 环境。
    """
    
    class Time:
        """ROS Time 的替代实现"""
        
        def __init__(self, secs: float = 0, nsecs: int = 0):
            self._secs = secs
            self._nsecs = nsecs
        
        @classmethod
        def now(cls) -> 'StandaloneRospy.Time':
            """获取当前时间"""
            t = time.time()
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            return cls(secs, nsecs)
        
        @classmethod
        def from_sec(cls, sec: float) -> 'StandaloneRospy.Time':
            """从秒数创建时间对象"""
            secs = int(sec)
            nsecs = int((sec - secs) * 1e9)
            return cls(secs, nsecs)
        
        def to_sec(self) -> float:
            """转换为秒数"""
            return self._secs + self._nsecs * 1e-9
        
        def __sub__(self, other: 'StandaloneRospy.Time') -> 'StandaloneRospy.Duration':
            """时间相减得到时长"""
            diff = self.to_sec() - other.to_sec()
            return StandaloneRospy.Duration.from_sec(diff)
        
        def __add__(self, other: 'StandaloneRospy.Duration') -> 'StandaloneRospy.Time':
            """时间加时长得到新时间"""
            return StandaloneRospy.Time.from_sec(self.to_sec() + other.to_sec())
        
        def __repr__(self) -> str:
            return f"Time({self.to_sec():.6f})"
    
    class Duration:
        """ROS Duration 的替代实现"""
        
        def __init__(self, secs: float = 0, nsecs: int = 0):
            self._secs = secs
            self._nsecs = nsecs
        
        @classmethod
        def from_sec(cls, sec: float) -> 'StandaloneRospy.Duration':
            """从秒数创建时长对象"""
            secs = int(sec)
            nsecs = int((sec - secs) * 1e9)
            return cls(secs, nsecs)
        
        def to_sec(self) -> float:
            """转换为秒数"""
            return self._secs + self._nsecs * 1e-9
        
        def __repr__(self) -> str:
            return f"Duration({self.to_sec():.6f})"
    
    # 日志函数
    @staticmethod
    def loginfo(msg: str):
        """记录信息日志"""
        logger.info(msg)
    
    @staticmethod
    def logwarn(msg: str):
        """记录警告日志"""
        logger.warning(msg)
    
    @staticmethod
    def logerr(msg: str):
        """记录错误日志"""
        logger.error(msg)
    
    @staticmethod
    def logdebug(msg: str):
        """记录调试日志"""
        logger.debug(msg)
    
    @staticmethod
    def logwarn_throttle(period: float, msg: str):
        """节流警告日志（简化实现：不做节流）"""
        logger.warning(msg)
    
    @staticmethod
    def loginfo_throttle(period: float, msg: str):
        """节流信息日志（简化实现：不做节流）"""
        logger.info(msg)
    
    @staticmethod
    def logwarn_once(msg: str):
        """单次警告日志（简化实现：不做去重）"""
        logger.warning(msg)
