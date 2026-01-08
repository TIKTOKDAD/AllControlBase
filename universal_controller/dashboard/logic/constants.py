"""
Dashboard constants
"""

# Platform name mapping
PLATFORM_NAMES = {
    'differential': '差速车',
    'ackermann': '阿克曼',
    'omni': '全向车',
    'quadrotor': '四旋翼',
}

# State name and description mapping
# Key: ControllerStateEnum value (int)
# Value: (Name, Description)
STATE_INFO = {
    0: ('INIT', '初始化'),
    1: ('NORMAL', '正常运行'),
    2: ('SOFT_DISABLED', 'Soft禁用'),
    3: ('MPC_DEGRADED', 'MPC降级'),
    4: ('BACKUP_ACTIVE', '备用激活'),
    5: ('STOPPING', '停车中'),
    6: ('STOPPED', '已停车'),
}
