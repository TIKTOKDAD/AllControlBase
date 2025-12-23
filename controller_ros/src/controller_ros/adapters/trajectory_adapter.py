"""
轨迹适配器

ROS 消息: controller_ros/LocalTrajectoryV4
UC 数据类型: universal_controller.core.data_types.Trajectory
"""
from typing import Any
import logging
import numpy as np

from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode
from .base import IMsgConverter

logger = logging.getLogger(__name__)


class TrajectoryAdapter(IMsgConverter):
    """
    轨迹消息适配器
    
    将 ROS LocalTrajectoryV4 转换为 UC Trajectory 数据类型。
    """
    
    def to_uc(self, ros_msg: Any) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory"""
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in ros_msg.points
        ]
        num_points = len(points)
        
        # 转换速度数组
        velocities = None
        soft_enabled = ros_msg.soft_enabled
        if soft_enabled and len(ros_msg.velocities_flat) > 0:
            # 从扁平数组重建 [N, 4] 数组
            flat_len = len(ros_msg.velocities_flat)
            
            # 检查长度是否为 4 的倍数
            if flat_len % 4 != 0:
                logger.warning(
                    f"velocities_flat length {flat_len} is not a multiple of 4, truncating"
                )
                flat_len = (flat_len // 4) * 4
            
            if flat_len > 0:
                velocities = np.array(ros_msg.velocities_flat[:flat_len]).reshape(-1, 4)
                num_vel_points = velocities.shape[0]
                
                # 检查速度点数与位置点数是否匹配
                if num_vel_points != num_points:
                    logger.warning(
                        f"Velocity points ({num_vel_points}) != position points ({num_points}), "
                        f"adjusting velocities"
                    )
                    if num_vel_points > num_points:
                        # 速度点多于位置点，截断
                        velocities = velocities[:num_points]
                    else:
                        # 速度点少于位置点，用零填充
                        padding = np.zeros((num_points - num_vel_points, 4))
                        velocities = np.vstack([velocities, padding])
            else:
                # 无有效速度数据，禁用 soft 模式
                soft_enabled = False
        elif soft_enabled:
            # soft_enabled=True 但没有速度数据，禁用 soft 模式
            logger.debug("soft_enabled=True but no velocity data, disabling soft mode")
            soft_enabled = False
        
        # 转换轨迹模式
        try:
            mode = TrajectoryMode(ros_msg.mode)
        except ValueError:
            mode = TrajectoryMode.MODE_TRACK
        
        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=ros_msg.dt_sec,
            confidence=ros_msg.confidence,
            mode=mode,
            soft_enabled=soft_enabled
        )
    
    def to_ros(self, uc_data: UcTrajectory) -> Any:
        """UC Trajectory → ROS LocalTrajectoryV4"""
        # 延迟导入避免循环依赖
        try:
            from controller_ros.msg import LocalTrajectoryV4
            from geometry_msgs.msg import Point
        except ImportError:
            raise ImportError("ROS messages not available")
        
        ros_msg = LocalTrajectoryV4()
        ros_msg.header.stamp = self._sec_to_ros_time(uc_data.header.stamp)
        ros_msg.header.frame_id = uc_data.header.frame_id
        ros_msg.mode = int(uc_data.mode.value) if hasattr(uc_data.mode, 'value') else int(uc_data.mode)
        ros_msg.dt_sec = float(uc_data.dt_sec)
        ros_msg.confidence = float(uc_data.confidence)
        ros_msg.soft_enabled = uc_data.soft_enabled
        
        ros_msg.points = [
            Point(x=float(p.x), y=float(p.y), z=float(p.z))
            for p in uc_data.points
        ]
        
        if uc_data.velocities is not None:
            ros_msg.velocities_flat = uc_data.velocities.flatten().tolist()
        else:
            ros_msg.velocities_flat = []
        
        return ros_msg
