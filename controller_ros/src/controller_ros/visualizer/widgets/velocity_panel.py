"""
速度监控面板

显示线速度和角速度的实时数值和进度条。
"""
from typing import Optional

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QProgressBar, QFrame, QGridLayout
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from ..models import VelocityData


class VelocityGauge(QWidget):
    """速度仪表组件
    
    显示:
    - 主数值 (大字): 命令速度
    - 参考值 (小字): 反馈速度
    """
    
    def __init__(self, title: str, unit: str, min_val: float, max_val: float, 
                 color: str = "#00ff00", parent=None):
        super().__init__(parent)
        
        self._title = title
        self._unit = unit
        self._min_val = min_val
        self._max_val = max_val
        self._color = color
        self._current_val = 0.0
        self._feedback_val = 0.0
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)
        
        # 标题
        title_label = QLabel(self._title)
        title_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        layout.addWidget(title_label)
        
        # 进度条
        self._progress = QProgressBar()
        self._progress.setMinimum(0)
        self._progress.setMaximum(100)
        self._progress.setValue(50)  # 中间位置
        self._progress.setTextVisible(False)
        self._progress.setFixedHeight(20)
        self._progress.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #555555;
                border-radius: 3px;
                background-color: #333333;
            }}
            QProgressBar::chunk {{
                background-color: {self._color};
                border-radius: 2px;
            }}
        """)
        layout.addWidget(self._progress)
        
        # 数值显示
        value_layout = QHBoxLayout()
        
        self._current_label = QLabel("0.00")
        self._current_label.setStyleSheet(f"color: {self._color}; font-size: 14px; font-weight: bold;")
        value_layout.addWidget(self._current_label)
        
        unit_label = QLabel(self._unit)
        unit_label.setStyleSheet("color: #888888; font-size: 11px;")
        value_layout.addWidget(unit_label)
        
        value_layout.addStretch()
        
        self._feedback_label = QLabel("反馈: 0.00")
        self._feedback_label.setStyleSheet("color: #888888; font-size: 10px;")
        value_layout.addWidget(self._feedback_label)
        
        layout.addLayout(value_layout)
    
    def set_value(self, current: float, feedback: float = None):
        """设置命令值和反馈值
        
        Args:
            current: 命令速度 (主显示)
            feedback: 反馈速度 (参考显示)
        """
        self._current_val = current
        if feedback is not None:
            self._feedback_val = feedback
        
        # 更新进度条 (将值映射到 0-100)
        range_val = self._max_val - self._min_val
        if range_val > 0:
            # 对于角速度，0 在中间
            if self._min_val < 0:
                percent = int(50 + (current / self._max_val) * 50)
            else:
                percent = int((current - self._min_val) / range_val * 100)
            self._progress.setValue(max(0, min(100, percent)))
        
        # 更新标签
        self._current_label.setText(f"{current:.2f}")
        if feedback is not None:
            self._feedback_label.setText(f"反馈: {feedback:.2f}")


class VelocityPanel(QWidget):
    """
    速度监控面板
    
    显示:
    - 线速度 (vx) 当前值和目标值
    - 角速度 (ω) 当前值和目标值
    """
    
    def __init__(self, max_linear: float = 0.5, max_angular: float = 1.0, parent=None):
        super().__init__(parent)
        
        self._max_linear = max_linear
        self._max_angular = max_angular
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 标题
        title = QLabel("底盘速度监控")
        title.setStyleSheet("color: #ffffff; font-size: 13px; font-weight: bold;")
        layout.addWidget(title)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #555555;")
        layout.addWidget(line)
        
        # 线速度仪表
        self._linear_gauge = VelocityGauge(
            "线速度 (vx)", "m/s",
            -self._max_linear, self._max_linear,
            color="#00ff88"
        )
        layout.addWidget(self._linear_gauge)
        
        # 角速度仪表
        self._angular_gauge = VelocityGauge(
            "角速度 (ω)", "rad/s",
            -self._max_angular, self._max_angular,
            color="#ffaa00"
        )
        layout.addWidget(self._angular_gauge)
        
        layout.addStretch()
        
        # 设置背景
        self.setStyleSheet("""
            VelocityPanel {
                background-color: #2a2a2a;
                border: 1px solid #444444;
                border-radius: 5px;
            }
        """)
    
    def update_velocity(self, actual: VelocityData, target: VelocityData = None):
        """
        更新速度显示
        
        显示策略:
        - 主数值: 命令速度 (target) - 控制器输出的目标速度
        - 小字: 反馈速度 (actual) - 底盘里程计反馈的实际速度
        
        原因: TurtleBot1 的 /odom 话题中角速度反馈不准确（几乎为0），
        显示命令速度更能反映控制器的实际输出。
        
        Args:
            actual: 实际速度 (来自 /odom)
            target: 目标速度 (来自 /cmd_unified)
        """
        # 命令速度作为主显示，反馈速度作为参考
        cmd_vx = target.linear_x if target else 0.0
        cmd_omega = target.angular_z if target else 0.0
        feedback_vx = actual.linear_x if actual else None
        feedback_omega = actual.angular_z if actual else None
        
        self._linear_gauge.set_value(cmd_vx, feedback_vx)
        self._angular_gauge.set_value(cmd_omega, feedback_omega)
