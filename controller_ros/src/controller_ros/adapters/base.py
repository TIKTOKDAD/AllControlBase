"""
适配器基类接口

支持 ROS1 和 ROS2 双版本。
"""
from abc import ABC, abstractmethod
from typing import Any, TypeVar

T_ROS = TypeVar('T_ROS')  # ROS 消息类型
T_UC = TypeVar('T_UC')    # UC 数据类型

# 检测 ROS 版本
_ROS_VERSION = 0
try:
    import rclpy
    _ROS_VERSION = 2
except ImportError:
    try:
        import rospy
        _ROS_VERSION = 1
    except ImportError:
        pass


class IMsgConverter(ABC):
    """
    消息转换器接口
    
    定义 ROS 消息与 universal_controller 数据类型之间的双向转换。
    支持 ROS1 和 ROS2。
    """
    
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
    
    @abstractmethod
    def to_ros(self, uc_data: T_UC) -> T_ROS:
        """
        UC 数据类型 → ROS 消息
        
        Args:
            uc_data: universal_controller 数据类型对象
        
        Returns:
            ROS 消息对象
        """
        pass
    
    def _ros_time_to_sec(self, stamp) -> float:
        """
        ROS 时间戳转秒
        
        支持 ROS1 (secs/nsecs) 和 ROS2 (sec/nanosec) 格式
        """
        if _ROS_VERSION == 1:
            # ROS1: rospy.Time 有 to_sec() 方法
            if hasattr(stamp, 'to_sec'):
                return stamp.to_sec()
            # 或者直接访问属性
            return stamp.secs + stamp.nsecs * 1e-9
        else:
            # ROS2: builtin_interfaces/Time
            return stamp.sec + stamp.nanosec * 1e-9
    
    def _sec_to_ros_time(self, sec: float):
        """
        秒转 ROS 时间戳
        
        支持 ROS1 和 ROS2
        """
        if _ROS_VERSION == 1:
            import rospy
            return rospy.Time.from_sec(sec)
        else:
            from builtin_interfaces.msg import Time
            t = Time()
            t.sec = int(sec)
            t.nanosec = int((sec - t.sec) * 1e9)
            return t
