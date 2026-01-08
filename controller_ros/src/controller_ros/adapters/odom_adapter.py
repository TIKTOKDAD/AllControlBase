"""
里程计适配器

ROS 消息: nav_msgs/Odometry
UC 数据类型: universal_controller.core.data_types.Odometry
"""
from typing import Any

from universal_controller.core.data_types import (
    Odometry as UcOdometry, Header, Point3D
)
from .base import IMsgConverter


class OdomAdapter(IMsgConverter):
    """
    Odometry 消息适配器
    
    将 ROS nav_msgs/Odometry 转换为 UC Odometry 数据类型。
    注意: Odometry 是输入数据，不需要 to_ros() 方法。

    坐标系说明:
    ROS nav_msgs/Odometry 定义 twist.angular 为 Child Frame (base_link) 下的角速度。
    控制器内部同样期望机体坐标系下的角速度。
    对于地面机器人（Z轴垂直于地面），世界系角速度 (dTheta/dt) 与机体系角速度 Z 分量数值相等。
    因此，直接传递 angular.z 符合 ROS 标准和控制器需求。
    """
    
    def __init__(self):
        """初始化里程计适配器"""
        super().__init__()
    
    def to_uc(self, ros_msg: Any) -> UcOdometry:
        """ROS Odometry → UC Odometry"""
        return UcOdometry(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            pose_position=Point3D(
                x=ros_msg.pose.pose.position.x,
                y=ros_msg.pose.pose.position.y,
                z=ros_msg.pose.pose.position.z
            ),
            pose_orientation=(
                ros_msg.pose.pose.orientation.x,
                ros_msg.pose.pose.orientation.y,
                ros_msg.pose.pose.orientation.z,
                ros_msg.pose.pose.orientation.w
            ),
            twist_linear=(
                ros_msg.twist.twist.linear.x,
                ros_msg.twist.twist.linear.y,
                ros_msg.twist.twist.linear.z
            ),
            twist_angular=(
                ros_msg.twist.twist.angular.x,
                ros_msg.twist.twist.angular.y,
                ros_msg.twist.twist.angular.z
            )
        )
