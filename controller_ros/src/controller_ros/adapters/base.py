"""
适配器基类接口

支持 ROS1 和 ROS2 双版本。
"""
from abc import ABC, abstractmethod
from typing import Any, TypeVar, Optional, Callable
import time as _time

from ..utils.ros_compat import ROS_VERSION, ros_time_to_sec, sec_to_ros_time

T_ROS = TypeVar('T_ROS')  # ROS 消息类型
T_UC = TypeVar('T_UC')    # UC 数据类型


class IMsgConverter(ABC):
    """
    消息转换器接口
    
    定义 ROS 消息与 universal_controller 数据类型之间的双向转换。
    支持 ROS1 和 ROS2。
    """
    
    def __init__(self, get_time_func: Optional[Callable[[], float]] = None):
        """
        初始化消息转换器
        
        Args:
            get_time_func: 获取当前时间的函数，用于支持仿真时间
                          如果为 None，使用系统时间
        """
        self._get_time_func = get_time_func
    
    @abstractmethod
    def to_uc(self, ros_msg: T_ROS) -> T_UC:
        """
        ROS 消息 → UC 数据类型
        
        Args:
            ros_msg: ROS 消息对象
        
        Returns:
            universal_controller 数据类型对象
        """
        pass
    
    def to_ros(self, uc_data: T_UC) -> T_ROS:
        """
        UC 数据类型 → ROS 消息
        
        Args:
            uc_data: universal_controller 数据类型对象
        
        Returns:
            ROS 消息对象
        
        Note:
            默认实现抛出 NotImplementedError。
            只有需要输出的适配器（如 OutputAdapter）需要重写此方法。
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support to_ros()")
    
    def _get_current_time(self) -> float:
        """
        获取当前时间（秒）
        
        处理仿真时间未初始化（返回 0）的情况，回退到系统时间。
        """
        if self._get_time_func is not None:
            t = self._get_time_func()
            # 仿真时间未初始化时可能返回 0，回退到系统时间
            if t > 0:
                return t
        return _time.time()
    
    def _ros_time_to_sec(self, stamp) -> float:
        """ROS 时间戳转秒 - 委托给 ros_compat"""
        return ros_time_to_sec(stamp)
    
    def _sec_to_ros_time(self, sec: float):
        """秒转 ROS 时间戳 - 委托给 ros_compat"""
        return sec_to_ros_time(sec)
