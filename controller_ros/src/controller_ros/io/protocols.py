from typing import Protocol, Dict, Any, List, Optional
from universal_controller.core.data_types import ControlOutput, Trajectory, AttitudeCommand

class PublisherProtocol(Protocol):
    """Protocol for Publisher Managers (ROS1 & ROS2)"""
    
    def publish_cmd(self, cmd: ControlOutput) -> None:
        """Publish control command"""
        ...
        
    def publish_stop_cmd(self) -> None:
        """Publish stop command"""
        ...
        
    def publish_diagnostics(self, diag: Dict[str, Any], force: bool = False) -> None:
        """Publish diagnostics"""
        ...
        
    def publish_attitude_cmd(self, attitude_cmd: AttitudeCommand, 
                           yaw_mode: int = 0, is_hovering: bool = False) -> None:
        """Publish attitude command (for quadrotors)"""
        ...
        
    def publish_predicted_path(self, predicted_states: list, frame_id: str = 'odom') -> None:
        """Publish MPC predicted path"""
        ...
        
    def publish_debug_path(self, trajectory: Trajectory) -> None:
        """Publish debug path"""
        ...
        
    def shutdown(self) -> None:
        """Shutdown publishers"""
        ...
        
    def get_stats(self) -> Dict[str, Any]:
        """Get publisher statistics"""
        ...

class ServiceProtocol(Protocol):
    """Protocol for Service Managers"""
    
    def shutdown(self) -> None:
        """Shutdown services"""
        ...
