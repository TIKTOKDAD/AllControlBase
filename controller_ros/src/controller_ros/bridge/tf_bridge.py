"""
TF2 桥接层

管理 TF2 Buffer 和 Listener，提供坐标变换查询。
支持 ROS1 和 ROS2。
"""
from typing import Optional
import logging

from ..utils.ros_compat import (
    ROS_VERSION, TF2_AVAILABLE, TF2Compat
)

logger = logging.getLogger(__name__)


class TFBridge:
    """
    TF2 桥接层
    
    职责:
    - 管理 TF2 Buffer 和 Listener
    - 提供坐标变换查询
    - 将变换注入到 universal_controller
    
    支持 ROS1 和 ROS2。
    """
    
    def __init__(self, node=None):
        """
        初始化 TF 桥接
        
        Args:
            node: ROS2 节点实例 (ROS1 不需要)
        """
        self._node = node
        self._tf2_compat = TF2Compat(node)
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time=None, timeout_sec: float = 0.01) -> Optional[dict]:
        """
        查询坐标变换
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 时间戳 (None 表示最新)
            timeout_sec: 超时时间 (秒)
        
        Returns:
            变换字典 {'translation': (x, y, z), 'rotation': (x, y, z, w)} 或 None
        """
        return self._tf2_compat.lookup_transform(
            target_frame, source_frame, time, timeout_sec
        )
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.01) -> bool:
        """检查是否可以进行坐标变换"""
        return self._tf2_compat.can_transform(
            target_frame, source_frame, time, timeout_sec
        )
    
    def inject_to_transformer(self, coord_transformer) -> bool:
        """
        将 TF2 变换注入到 universal_controller 的坐标变换器
        
        Args:
            coord_transformer: RobustCoordinateTransformer 实例
        
        Returns:
            是否成功注入
        """
        if not self.is_initialized:
            logger.warning("TF2 not initialized, cannot inject to transformer")
            return False
        
        if coord_transformer is None:
            logger.warning("Coordinate transformer is None")
            return False
        
        # 检查 coord_transformer 是否支持 TF2 注入
        if hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            coord_transformer.set_tf2_lookup_callback(self.lookup_transform)
            logger.info("TF2 lookup callback injected to coordinate transformer")
            return True
        else:
            logger.warning("Coordinate transformer does not support TF2 injection")
            return False
    
    @property
    def is_initialized(self) -> bool:
        """TF2 是否已初始化"""
        return self._tf2_compat.is_initialized
    
    @property
    def buffer(self):
        """获取 TF2 Buffer (用于高级操作)"""
        return self._tf2_compat.buffer
