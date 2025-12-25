"""
速度数据适配器

从 Odometry 和 UnifiedCmd 提取速度数据。
"""
from typing import Any
from ..models import VelocityData


class VelocityAdapter:
    """
    速度数据适配器
    
    从 ROS 消息中提取速度数据。
    """
    
    def __init__(self):
        pass
    
    def from_odom(self, ros_msg: Any, timestamp: float = 0.0) -> VelocityData:
        """
        从 Odometry 消息提取实际速度
        
        Args:
            ros_msg: nav_msgs/Odometry 消息
            timestamp: 时间戳 (秒)
        
        Returns:
            VelocityData
        """
        return VelocityData(
            linear_x=ros_msg.twist.twist.linear.x,
            linear_y=ros_msg.twist.twist.linear.y,
            angular_z=ros_msg.twist.twist.angular.z,
            timestamp=timestamp,
        )
    
    def from_unified_cmd(self, ros_msg: Any, timestamp: float = 0.0) -> VelocityData:
        """
        从 UnifiedCmd 消息提取目标速度
        
        Args:
            ros_msg: controller_ros/UnifiedCmd 消息
            timestamp: 时间戳 (秒)
        
        Returns:
            VelocityData
        """
        return VelocityData(
            linear_x=ros_msg.vx,
            linear_y=ros_msg.vy,
            angular_z=ros_msg.omega,
            timestamp=timestamp,
        )
    
    def from_twist(self, ros_msg: Any, timestamp: float = 0.0) -> VelocityData:
        """
        从 Twist 消息提取速度
        
        Args:
            ros_msg: geometry_msgs/Twist 消息
            timestamp: 时间戳 (秒)
        
        Returns:
            VelocityData
        """
        return VelocityData(
            linear_x=ros_msg.linear.x,
            linear_y=ros_msg.linear.y,
            angular_z=ros_msg.angular.z,
            timestamp=timestamp,
        )
