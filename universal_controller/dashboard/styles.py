"""
样式定义 - 颜色方案和样式常量

Dashboard UI 统一样式配置，包含：
- 状态颜色（7级状态机）
- 通用颜色（成功/警告/错误等）
- 背景色和文字颜色
- 进度条颜色阈值
- 主窗口样式表
"""

# 状态颜色 - 7级状态机
COLORS = {
    'INIT': '#9E9E9E',          # 灰色 - 初始化
    'NORMAL': '#4CAF50',        # 绿色 - 正常运行
    'SOFT_DISABLED': '#FFC107', # 黄色 - Soft禁用
    'MPC_DEGRADED': '#FF9800',  # 橙色 - MPC降级
    'BACKUP_ACTIVE': '#FF5722', # 橙红色 - 备用激活
    'STOPPING': '#F44336',      # 红色 - 停车中
    'STOPPED': '#B71C1C',       # 深红色 - 已停车
    
    # 通用颜色
    'success': '#4CAF50',       # 成功/正常
    'success_light': '#81C784', # 浅绿色
    'warning': '#FFC107',       # 警告
    'warning_light': '#FFD54F', # 浅黄色
    'error': '#F44336',         # 错误
    'error_light': '#E57373',   # 浅红色
    'info': '#2196F3',          # 信息/标题
    'info_light': '#64B5F6',    # 浅蓝色
    'disabled': '#9E9E9E',      # 禁用
    'unavailable': '#757575',   # 数据不可用
    
    # 特殊指标颜色
    'excellent': '#00E676',     # 优秀 - 亮绿色
    'good': '#4CAF50',          # 良好 - 绿色
    'fair': '#FFC107',          # 一般 - 黄色
    'poor': '#FF5722',          # 较差 - 橙红色
    'critical': '#F44336',      # 严重 - 红色
    
    # 背景色
    'bg_dark': '#1E1E1E',       # 主背景
    'bg_panel': '#2D2D2D',      # 面板背景
    'bg_header': '#3D3D3D',     # 头部背景
    'bg_highlight': '#424242',  # 高亮背景
    'bg_unavailable': '#252525',# 数据不可用背景
    
    # 文字颜色
    'text': '#FFFFFF',          # 主文字
    'text_secondary': '#B0B0B0',# 次要文字
    'text_muted': '#808080',    # 弱化文字
    'text_unavailable': '#606060',# 不可用文字
    
    # 边框颜色
    'border': '#3D3D3D',        # 普通边框
    'border_light': '#4D4D4D',  # 浅边框
    'border_accent': '#2196F3', # 强调边框
}

# 数据不可用显示文本
UNAVAILABLE_TEXT = '无数据'
UNAVAILABLE_VALUE = '--'

# 进度条颜色阈值
def get_progress_color(ratio: float, inverted: bool = False) -> str:
    """根据占比返回颜色
    
    Args:
        ratio: 当前值与最大值的比例 [0, 1]
        inverted: 是否反转颜色（用于"越高越好"的指标）
    
    Returns:
        颜色代码
    """
    if inverted:
        # 反转模式：值越高越好（如跟踪质量评分）
        if ratio >= 0.8:
            return COLORS['success']
        elif ratio >= 0.5:
            return COLORS['warning']
        else:
            return COLORS['error']
    else:
        # 正常模式：值越低越好（如误差、延迟）
        if ratio < 0.5:
            return COLORS['success']
        elif ratio < 0.8:
            return COLORS['warning']
        else:
            return COLORS['error']


def get_quality_color(score: float) -> str:
    """根据质量评分返回颜色
    
    Args:
        score: 质量评分 [0, 100]
    
    Returns:
        颜色代码
    """
    if score >= 90:
        return COLORS['excellent']
    elif score >= 75:
        return COLORS['good']
    elif score >= 50:
        return COLORS['fair']
    elif score >= 25:
        return COLORS['poor']
    else:
        return COLORS['critical']


def get_health_color(healthy: bool, warning: bool = False) -> str:
    """根据健康状态返回颜色
    
    Args:
        healthy: 是否健康
        warning: 是否有警告
    
    Returns:
        颜色代码
    """
    if not healthy:
        return COLORS['error']
    elif warning:
        return COLORS['warning']
    else:
        return COLORS['success']

# 状态名称映射
STATE_NAMES = {
    0: ('INIT', '初始化'),
    1: ('NORMAL', '正常运行'),
    2: ('SOFT_DISABLED', 'Soft禁用'),
    3: ('MPC_DEGRADED', 'MPC降级'),
    4: ('BACKUP_ACTIVE', '备用激活'),
    5: ('STOPPING', '停车中'),
    6: ('STOPPED', '已停车'),
}

# 状态描述
STATE_DESCRIPTIONS = {
    0: '系统启动中，等待数据',
    1: 'MPC 正常工作，Soft 启用',
    2: 'α < 0.1，仅使用 Hard 轨迹',
    3: 'Horizon 降低，性能下降',
    4: 'Pure Pursuit 接管控制',
    5: '正在平滑减速',
    6: '速度为零，等待恢复',
}

# 主窗口样式
MAIN_STYLE = """
QMainWindow {
    background-color: #1E1E1E;
}
QWidget {
    background-color: #1E1E1E;
    color: #FFFFFF;
    font-family: "Microsoft YaHei", "Segoe UI", "PingFang SC", sans-serif;
    font-size: 12px;
}
QGroupBox {
    background-color: #2D2D2D;
    border: 1px solid #3D3D3D;
    border-radius: 8px;
    margin-top: 12px;
    padding-top: 12px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px;
    color: #2196F3;
    font-size: 13px;
}
QLabel {
    color: #FFFFFF;
}
QProgressBar {
    border: 1px solid #3D3D3D;
    border-radius: 4px;
    background-color: #1E1E1E;
    text-align: center;
    height: 18px;
}
QProgressBar::chunk {
    border-radius: 3px;
}
QScrollArea {
    border: none;
    background-color: transparent;
}
QScrollBar:vertical {
    background-color: #1E1E1E;
    width: 10px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background-color: #4D4D4D;
    border-radius: 5px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background-color: #5D5D5D;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QTextEdit {
    background-color: #1E1E1E;
    border: 1px solid #3D3D3D;
    border-radius: 4px;
    color: #FFFFFF;
    selection-background-color: #2196F3;
}
QPushButton {
    background-color: #3D3D3D;
    border: 1px solid #4D4D4D;
    border-radius: 4px;
    padding: 6px 16px;
    color: #FFFFFF;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #4D4D4D;
    border-color: #5D5D5D;
}
QPushButton:pressed {
    background-color: #2D2D2D;
}
QPushButton:disabled {
    background-color: #252525;
    color: #606060;
    border-color: #3D3D3D;
}
QComboBox {
    background-color: #3D3D3D;
    border: 1px solid #4D4D4D;
    border-radius: 4px;
    padding: 4px 8px;
    color: #FFFFFF;
}
QComboBox:hover {
    border-color: #5D5D5D;
}
QComboBox::drop-down {
    border: none;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #2D2D2D;
    border: 1px solid #4D4D4D;
    selection-background-color: #2196F3;
}
QToolTip {
    background-color: #3D3D3D;
    color: #FFFFFF;
    border: 1px solid #4D4D4D;
    border-radius: 4px;
    padding: 4px 8px;
}
"""


# 面板标题样式
PANEL_TITLE_STYLE = 'color: #2196F3; font-weight: bold; font-size: 12px; border-bottom: 1px solid #3D3D3D; padding-bottom: 4px; margin-bottom: 4px;'

# 子标题样式
SUB_TITLE_STYLE = 'color: #64B5F6; font-weight: 500; font-size: 11px;'

# 值标签样式
VALUE_LABEL_STYLE = 'color: #FFFFFF; font-weight: 500;'

# 次要值标签样式
SECONDARY_VALUE_STYLE = 'color: #B0B0B0;'

# 不可用样式
UNAVAILABLE_STYLE = 'color: #757575;'
