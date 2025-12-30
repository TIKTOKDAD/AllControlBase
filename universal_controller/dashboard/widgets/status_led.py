"""
状态指示灯控件

提供多种状态指示：
- 成功/失败/警告/不可用
- 带文本标签
- 支持动态更新
"""

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..styles import COLORS


class StatusLED(QWidget):
    """状态指示灯
    
    支持状态：
    - True: 成功/正常（绿色）
    - False: 失败/错误（红色）
    - None: 不可用（灰色）
    - warning: 警告（黄色）
    """
    
    def __init__(self, text: str = '', parent=None):
        super().__init__(parent)
        self._text = text  # 保存原始文本
        self._status = None
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        
        # LED 指示灯
        self.led = QLabel()
        self.led.setFixedSize(14, 14)
        self.led.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.led)
        
        # 文本标签
        self.label = QLabel(self._text)
        layout.addWidget(self.label)
        layout.addStretch()
    
    def set_status(self, status: bool, text: str = None):
        """
        设置状态
        
        Args:
            status: True=成功, False=失败, None=不可用
            text: 显示文本 (如果为 None，保留原始文本)
        """
        self._status = status
        
        # 只有明确传入非 None 且非 '无数据' 的文本时才更新标签
        # '无数据' 是状态描述，不应替换功能名称
        if text is not None and text != '无数据':
            self._text = text
            self.label.setText(text)
        
        if status is None:
            # 数据不可用状态 - 保留功能名称，只改变样式
            color = COLORS.get('unavailable', COLORS['disabled'])
            symbol = '?'
            self.label.setStyleSheet(f'color: {color};')
            border_color = '#4D4D4D'
        elif status:
            color = COLORS['success']
            symbol = '✓'
            self.label.setStyleSheet('color: #FFFFFF;')
            border_color = COLORS['success']
        else:
            color = COLORS['error']
            symbol = '✗'
            self.label.setStyleSheet('color: #FFFFFF;')
            border_color = COLORS['error']
        
        # 添加发光效果
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border: 2px solid {border_color};
                border-radius: 7px;
                color: white;
                font-size: 9px;
                font-weight: bold;
            }}
        """)
        self.led.setText(symbol)
    
    def set_warning(self, text: str = None):
        """设置警告状态"""
        if text:
            self._text = text
            self.label.setText(text)
        
        color = COLORS['warning']
        self.label.setStyleSheet(f'color: {color};')
        
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border: 2px solid {color};
                border-radius: 7px;
                color: black;
                font-size: 9px;
                font-weight: bold;
            }}
        """)
        self.led.setText('!')


class StatusIndicator(QWidget):
    """状态指示器 (带值显示)
    
    用于显示带有数值的状态，如：
    - 温度: 45°C (正常)
    - 延迟: 12ms (警告)
    """
    
    def __init__(self, label: str = '', parent=None):
        super().__init__(parent)
        self._label = label
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        
        # 标签
        self.name_label = QLabel(self._label)
        self.name_label.setFixedWidth(100)
        layout.addWidget(self.name_label)
        
        # LED
        self.led = QLabel()
        self.led.setFixedSize(16, 16)
        self.led.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.led)
        
        # 值
        self.value_label = QLabel()
        layout.addWidget(self.value_label)
        layout.addStretch()
    
    def set_value(self, status: bool, value_text: str, detail_text: str = ''):
        """设置值"""
        # 更新 LED
        if status is None:
            color = COLORS['disabled']
            border_color = '#4D4D4D'
        elif status:
            color = COLORS['success']
            border_color = COLORS['success']
        else:
            color = COLORS['error']
            border_color = COLORS['error']
        
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border: 2px solid {border_color};
                border-radius: 8px;
            }}
        """)
        
        # 更新值文本
        text = value_text
        if detail_text:
            text += f"  ({detail_text})"
        self.value_label.setText(text)
