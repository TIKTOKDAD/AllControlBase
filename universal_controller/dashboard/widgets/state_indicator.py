"""
状态指示器控件 - 用于显示 7 级状态
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt
from ..styles import COLORS, STATE_NAMES, STATE_DESCRIPTIONS


class StateIndicator(QWidget):
    """大型状态指示器"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_state = 0
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # 状态显示框
        self.state_frame = QFrame()
        self.state_frame.setFixedSize(200, 100)
        self.state_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {COLORS['INIT']};
                border-radius: 10px;
            }}
        """)
        
        frame_layout = QVBoxLayout(self.state_frame)
        
        # 状态名称
        self.state_name = QLabel('INIT')
        self.state_name.setAlignment(Qt.AlignCenter)
        self.state_name.setStyleSheet("""
            QLabel {
                font-size: 24px;
                font-weight: bold;
                color: white;
            }
        """)
        frame_layout.addWidget(self.state_name)
        
        # 状态描述
        self.state_desc = QLabel('初始化')
        self.state_desc.setAlignment(Qt.AlignCenter)
        self.state_desc.setStyleSheet("""
            QLabel {
                font-size: 12px;
                color: rgba(255, 255, 255, 0.8);
            }
        """)
        frame_layout.addWidget(self.state_desc)
        
        layout.addWidget(self.state_frame, alignment=Qt.AlignCenter)
    
    def set_state(self, state: int):
        """设置状态"""
        self._current_state = state
        
        if state in STATE_NAMES:
            name, desc = STATE_NAMES[state]
            color = COLORS.get(name, COLORS['disabled'])
        else:
            name = f'UNKNOWN({state})'
            desc = '未知状态'
            color = COLORS['disabled']
        
        self.state_name.setText(name)
        self.state_desc.setText(desc)
        self.state_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {color};
                border-radius: 10px;
            }}
        """)


class StateMachineIndicator(QWidget):
    """状态机指示器 - 显示所有 7 个状态"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_state = 0
        self._state_items = {}
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)
        
        for state_id, (name, desc) in STATE_NAMES.items():
            item = self._create_state_item(state_id, name, desc)
            layout.addWidget(item)
            self._state_items[state_id] = item
    
    def _create_state_item(self, state_id: int, name: str, desc: str) -> QWidget:
        """创建状态项"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(8)
        
        # 状态指示灯
        led = QLabel('○')
        led.setFixedWidth(20)
        led.setAlignment(Qt.AlignCenter)
        led.setObjectName('led')
        layout.addWidget(led)
        
        # 状态编号和名称
        name_label = QLabel(f'{state_id}. {name}')
        name_label.setFixedWidth(140)
        name_label.setObjectName('name')
        layout.addWidget(name_label)
        
        # 状态描述
        desc_label = QLabel(desc)
        desc_label.setStyleSheet('color: #808080;')
        desc_label.setObjectName('desc')
        layout.addWidget(desc_label)
        
        layout.addStretch()
        
        return widget
    
    def set_state(self, state: int):
        """设置当前状态"""
        self._current_state = state
        
        for state_id, widget in self._state_items.items():
            led = widget.findChild(QLabel, 'led')
            name_label = widget.findChild(QLabel, 'name')
            
            if state_id == state:
                # 当前状态
                name, _ = STATE_NAMES[state_id]
                color = COLORS.get(name, COLORS['disabled'])
                led.setText('●')
                led.setStyleSheet(f'color: {color}; font-size: 16px;')
                name_label.setStyleSheet(f'color: {color}; font-weight: bold;')
            else:
                # 非当前状态
                led.setText('○')
                led.setStyleSheet('color: #606060; font-size: 16px;')
                name_label.setStyleSheet('color: #808080;')
