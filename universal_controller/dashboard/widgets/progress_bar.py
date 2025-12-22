"""
自定义进度条控件 - 支持动态颜色
"""

from PyQt5.QtWidgets import QProgressBar, QWidget, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..styles import get_progress_color, COLORS


class ColorProgressBar(QWidget):
    """带颜色变化的进度条"""
    
    def __init__(self, parent=None, show_text=True, show_percent=True):
        super().__init__(parent)
        self.show_text = show_text
        self.show_percent = show_percent
        self._value = 0
        self._max_value = 100
        self._threshold = None
        self._unit = ''
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # 进度条
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(16)
        layout.addWidget(self.progress, 1)
        
        # 数值标签
        if self.show_text:
            self.value_label = QLabel()
            self.value_label.setMinimumWidth(140)
            self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            layout.addWidget(self.value_label)
    
    def set_value(self, value: float, max_value: float = None, threshold: float = None):
        """设置值"""
        self._value = value
        if max_value is not None:
            self._max_value = max_value
        if threshold is not None:
            self._threshold = threshold
        
        # 计算百分比
        if self._max_value > 0:
            ratio = min(value / self._max_value, 1.0)
        else:
            ratio = 0
        
        # 更新进度条
        self.progress.setMaximum(1000)
        self.progress.setValue(int(ratio * 1000))
        
        # 更新颜色
        color = get_progress_color(ratio)
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #3D3D3D;
                border-radius: 3px;
                background-color: #1E1E1E;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        
        # 更新文本
        if self.show_text:
            if self._threshold:
                text = f"{value:.2f}{self._unit} / {self._threshold:.2f}"
            else:
                text = f"{value:.2f}{self._unit}"
            
            if self.show_percent:
                text += f" ({ratio*100:.1f}%)"
            
            self.value_label.setText(text)
    
    def set_unit(self, unit: str):
        """设置单位"""
        self._unit = unit


class SimpleProgressBar(QProgressBar):
    """简单的彩色进度条"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTextVisible(True)
        self.setFixedHeight(18)
    
    def set_value_with_color(self, value: float, max_value: float = 100):
        """设置值并更新颜色"""
        ratio = value / max_value if max_value > 0 else 0
        ratio = min(ratio, 1.0)
        
        self.setMaximum(int(max_value * 10))
        self.setValue(int(value * 10))
        
        color = get_progress_color(ratio)
        self.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #3D3D3D;
                border-radius: 3px;
                background-color: #1E1E1E;
                text-align: center;
                color: white;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 2px;
            }}
        """)
        self.setFormat(f"{value:.1f} / {max_value:.1f}")
