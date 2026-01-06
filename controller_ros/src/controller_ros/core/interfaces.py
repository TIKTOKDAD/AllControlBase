"""
ROS 控制器核心接口定义

使用 Protocol 定义静态接口，避免运行时继承耦合。
"""
from typing import Protocol, Any, Dict, Optional, Tuple, runtime_checkable
from universal_controller.core.data_types import ControlOutput, AttitudeCommand, Trajectory

@runtime_checkable
class ITFBridge(Protocol):
    """TF2 桥接接口协议"""
    
    @property
    def is_initialized(self) -> bool:
        """TF2 是否已初始化"""
        ...
        
    def lookup_transform(self, target_frame: str, source_frame: str, 
                         time: Any = None, timeout_sec: Any = None) -> Any:
        """查找坐标变换"""
        ...
        
    def can_transform(self, target_frame: str, source_frame: str, 
                      time: Any = None, timeout_sec: float = 0.0) -> bool:
        """检查是否可转换"""
        ...

@runtime_checkable
class IPublishers(Protocol):
    """发布器包装接口协议"""
    
    def publish_cmd(self, cmd: ControlOutput) -> None:
        """发布控制命令"""
        ...
        
    def publish_stop_cmd(self) -> None:
        """发布停止命令"""
        ...
        
    def publish_diagnostics(self, diag: Dict[str, Any]) -> None:
        """发布诊断信息"""
        ...
        
    def publish_attitude_cmd(self, attitude_cmd: AttitudeCommand, 
                           yaw_mode: int, is_hovering: bool) -> None:
        """
        发布姿态命令 (可选，仅四旋翼)
        注意：即使不是四旋翼，发布器也可能实现此接口（为空操作）
        """
        ...
        
    def publish_predicted_path(self, trajectory: Trajectory, frame_id: str) -> None:
        """发布预测路径"""
        ...
