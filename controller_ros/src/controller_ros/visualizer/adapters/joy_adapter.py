"""
手柄消息适配器

ROS 消息: sensor_msgs/Joy
可视化模型: JoystickState
"""
from typing import Any, Dict
from ..models import JoystickState


class JoyAdapter:
    """
    手柄消息适配器
    
    将 ROS sensor_msgs/Joy 转换为 JoystickState。
    支持配置摇杆轴和按钮映射。
    """
    
    # 默认 Xbox 手柄映射 (可能因驱动不同而变化)
    DEFAULT_AXIS_MAP = {
        'left_x': 0,       # 左摇杆 X
        'left_y': 1,       # 左摇杆 Y
        'right_x': 3,      # 右摇杆 X
        'right_y': 4,      # 右摇杆 Y
        'lt': 2,           # 左扳机
        'rt': 5,           # 右扳机
    }
    
    DEFAULT_BUTTON_MAP = {
        'a': 0,
        'b': 1,
        'x': 2,
        'y': 3,
        'lb': 4,           # 左肩键 (使能键)
        'rb': 5,           # 右肩键
        'back': 6,
        'start': 7,
        'guide': 8,
        'left_stick': 9,
        'right_stick': 10,
    }
    
    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化手柄适配器
        
        Args:
            config: 手柄配置，包含:
                - enable_button: 使能键索引 (默认 4, LB)
                - linear_axis: 线速度轴索引 (默认 1, 左摇杆 Y)
                - angular_axis: 角速度轴索引 (默认 3, 右摇杆 X)
                - deadzone: 摇杆死区 (默认 0.1)
        """
        config = config or {}
        self._enable_button = config.get('enable_button', 4)
        self._linear_axis = config.get('linear_axis', 1)
        self._angular_axis = config.get('angular_axis', 3)
        self._deadzone = config.get('deadzone', 0.1)
        
        # 轴映射
        self._left_x_axis = config.get('left_x_axis', 0)
        self._left_y_axis = config.get('left_y_axis', 1)
        self._right_x_axis = config.get('right_x_axis', 3)
        self._right_y_axis = config.get('right_y_axis', 4)
    
    def _apply_deadzone(self, value: float) -> float:
        """应用死区"""
        if abs(value) < self._deadzone:
            return 0.0
        return value
    
    def to_model(self, ros_msg: Any) -> JoystickState:
        """
        ROS Joy → JoystickState
        
        Args:
            ros_msg: sensor_msgs/Joy 消息
        
        Returns:
            JoystickState 数据模型
        """
        axes = ros_msg.axes if ros_msg.axes else []
        buttons = list(ros_msg.buttons) if ros_msg.buttons else []
        
        # 安全获取轴值
        def get_axis(idx: int) -> float:
            if 0 <= idx < len(axes):
                return self._apply_deadzone(axes[idx])
            return 0.0
        
        # 安全获取按钮值
        def get_button(idx: int) -> bool:
            if 0 <= idx < len(buttons):
                return buttons[idx] == 1
            return False
        
        return JoystickState(
            connected=True,
            left_x=get_axis(self._left_x_axis),
            left_y=get_axis(self._left_y_axis),
            right_x=get_axis(self._right_x_axis),
            right_y=get_axis(self._right_y_axis),
            enable_pressed=get_button(self._enable_button),
            buttons=buttons,
        )
    
    @property
    def enable_button_index(self) -> int:
        """获取使能键索引"""
        return self._enable_button
    
    @property
    def linear_axis_index(self) -> int:
        """获取线速度轴索引"""
        return self._linear_axis
    
    @property
    def angular_axis_index(self) -> int:
        """获取角速度轴索引"""
        return self._angular_axis
