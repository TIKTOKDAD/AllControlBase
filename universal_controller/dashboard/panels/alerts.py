"""
警告/提醒面板
"""

from PyQt5.QtWidgets import (QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, 
                             QTextEdit, QPushButton, QComboBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QTextCursor
from datetime import datetime
from collections import deque
from ..styles import COLORS


class AlertsPanel(QGroupBox):
    """警告/提醒面板"""
    
    def __init__(self, parent=None):
        super().__init__('警告/提醒 (Alerts)', parent)
        self._alerts = deque(maxlen=100)
        self._filter = 'all'
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        
        # 日志显示区域
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(150)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1E1E1E;
                border: 1px solid #3D3D3D;
                border-radius: 3px;
                font-family: Consolas, monospace;
                font-size: 11px;
            }
        """)
        layout.addWidget(self.log_text)
        
        # 控制按钮
        btn_layout = QHBoxLayout()
        
        clear_btn = QPushButton('清除日志')
        clear_btn.clicked.connect(self.clear_logs)
        btn_layout.addWidget(clear_btn)
        
        export_btn = QPushButton('导出日志')
        export_btn.clicked.connect(self.export_logs)
        btn_layout.addWidget(export_btn)
        
        btn_layout.addWidget(QLabel('筛选:'))
        
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(['全部', '信息', '警告', '错误'])
        self.filter_combo.currentTextChanged.connect(self._on_filter_changed)
        btn_layout.addWidget(self.filter_combo)
        
        btn_layout.addStretch()
        
        layout.addLayout(btn_layout)
    
    def add_alert(self, level: str, message: str):
        """添加警告"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        # 图标和颜色
        if level == 'info':
            icon = 'ℹ️'
            color = COLORS['info']
        elif level == 'success':
            icon = '✓'
            color = COLORS['success']
        elif level == 'warning':
            icon = '⚠️'
            color = COLORS['warning']
        elif level == 'error':
            icon = '❌'
            color = COLORS['error']
        else:
            icon = '•'
            color = COLORS['text']
        
        alert = {
            'timestamp': timestamp,
            'level': level,
            'icon': icon,
            'message': message,
            'color': color,
        }
        
        self._alerts.appendleft(alert)
        self._refresh_display()
    
    def _refresh_display(self):
        """刷新显示"""
        self.log_text.clear()
        
        for alert in self._alerts:
            # 筛选
            if self._filter != 'all':
                filter_map = {'信息': 'info', '警告': 'warning', '错误': 'error'}
                if alert['level'] != filter_map.get(self._filter, ''):
                    if not (self._filter == '信息' and alert['level'] == 'success'):
                        continue
            
            # 格式化
            line = f"<span style='color: #808080;'>{alert['timestamp']}</span>  "
            line += f"<span style='color: {alert['color']};'>{alert['icon']}</span>  "
            line += f"<span style='color: {alert['color']};'>{alert['message']}</span>"
            
            self.log_text.append(line)
    
    def _on_filter_changed(self, text: str):
        """筛选变化"""
        filter_map = {'全部': 'all', '信息': 'info', '警告': 'warning', '错误': 'error'}
        self._filter = filter_map.get(text, 'all')
        self._refresh_display()
    
    def clear_logs(self):
        """清除日志"""
        self._alerts.clear()
        self.log_text.clear()
    
    def export_logs(self):
        """导出日志"""
        from PyQt5.QtWidgets import QFileDialog
        
        filename, _ = QFileDialog.getSaveFileName(
            self, '导出日志', 'controller_logs.txt', 'Text Files (*.txt)'
        )
        
        if filename:
            with open(filename, 'w', encoding='utf-8') as f:
                for alert in reversed(list(self._alerts)):
                    f.write(f"{alert['timestamp']}  {alert['icon']}  {alert['message']}\n")

    def update_display(self, data):
        """使用统一数据模型更新显示 (AlertsPanel 不需要定期更新，只需要 check_alerts)"""
        pass

    def check_alerts(self, data, last_data=None):
        """使用统一数据模型检查并生成警告"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return
        if last_data is None or not isinstance(last_data, DisplayData):
            return

        # 状态变化
        state = data.controller.state
        last_state = last_data.controller.state
        if state != last_state:
            from ..styles import STATE_NAMES
            old_name = STATE_NAMES.get(last_state, ('UNKNOWN', ''))[0]
            new_name = STATE_NAMES.get(state, ('UNKNOWN', ''))[0]

            if state > last_state:
                self.add_alert('warning', f'状态降级: {old_name} → {new_name}')
            else:
                self.add_alert('success', f'状态恢复: {old_name} → {new_name}')

        # MPC 求解时间警告
        solve_time = data.mpc_health.solve_time_ms
        if solve_time > 12:
            self.add_alert('warning', f'MPC 求解时间 {solve_time:.1f}ms，接近警告阈值 (15ms)')

        # 超时警告
        timeout = data.timeout
        last_timeout = last_data.timeout

        if timeout.odom_timeout and not last_timeout.odom_timeout:
            self.add_alert('error', 'Odom 超时')
        elif not timeout.odom_timeout and last_timeout.odom_timeout:
            self.add_alert('success', 'Odom 恢复正常')

        if timeout.traj_timeout and not last_timeout.traj_timeout:
            self.add_alert('error', 'Traj 超时')
        elif not timeout.traj_timeout and last_timeout.traj_timeout:
            self.add_alert('success', 'Traj 恢复正常')

        # α_soft 警告
        alpha = data.consistency.alpha_soft
        last_alpha = last_data.consistency.alpha_soft

        if alpha < 0.1 and last_alpha >= 0.1:
            self.add_alert('warning', f'α_soft 降至 {alpha:.2f}，低于阈值 0.1，禁用 Soft Head')
        elif alpha >= 0.1 and last_alpha < 0.1:
            self.add_alert('success', f'α_soft 恢复至 {alpha:.2f}，重新启用 Soft Head')
