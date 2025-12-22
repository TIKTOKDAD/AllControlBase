"""姿态控制配置

四旋翼无人机姿态控制器的配置参数 (F14)：
- 物理参数
- 姿态角限制
- 悬停检测参数
- 推力限制
"""

ATTITUDE_CONFIG = {
    # 物理参数
    'mass': 1.5,                    # 质量 (kg)
    # 注意: 重力加速度统一使用 system.gravity
    
    # F14.2: 姿态角速度限制
    'roll_rate_max': 3.0,           # 最大滚转角速度 (rad/s)
    'pitch_rate_max': 3.0,          # 最大俯仰角速度 (rad/s)
    'yaw_rate_max': 2.0,            # 最大偏航角速度 (rad/s)
    
    # 姿态角限制
    'roll_max': 0.5,                # 最大滚转角 (rad, ~30°)
    'pitch_max': 0.5,               # 最大俯仰角 (rad, ~30°)
    
    # 速度控制增益
    'kp_vx': 0.5,                   # X 方向速度增益
    'kp_vy': 0.5,                   # Y 方向速度增益
    'kp_vz': 1.0,                   # Z 方向速度增益
    
    # F14.3: 悬停 yaw 漂移补偿
    'hover_yaw_compensation': True,  # 是否启用悬停 yaw 补偿
    'hover_speed_thresh': 0.1,       # 悬停水平速度阈值 (m/s)
    'hover_vz_thresh': 0.05,         # 悬停垂直速度阈值 (m/s)
    'yaw_drift_rate': 0.001,         # yaw 漂移率 (rad/s)
    
    # 悬停检测滞后参数
    'hover_enter_factor': 1.0,       # 进入悬停的阈值因子
    'hover_exit_factor': 1.5,        # 退出悬停的阈值因子
    'hover_cmd_exit_factor': 2.0,    # 命令速度退出悬停的阈值因子
    'hover_debounce_time': 0.1,      # 悬停状态切换去抖动时间 (秒)
    
    # F14.4: 位置-姿态解耦
    'position_attitude_decoupled': False,
    
    # 推力限制 (归一化到悬停推力)
    'thrust_min': 0.1,               # 最小推力
    'thrust_max': 2.0,               # 最大推力
    'thrust_rate_max': 2.0,          # 推力变化率限制 (每秒)
    'min_thrust_factor': 0.1,        # 最小推力加速度因子
    'attitude_factor_min': 0.1,      # 姿态角饱和后推力重计算的最小因子
    
    # 其他参数
    'invert_pitch_sign': True,       # pitch 符号反转
    'dt': 0.02,                      # 时间步长 (秒)
}

# 姿态控制配置验证规则
ATTITUDE_VALIDATION_RULES = {
    'attitude.mass': (0.01, 100.0, '质量 (kg)'),
    'attitude.roll_max': (0.01, 1.57, '最大滚转角 (rad)'),
    'attitude.pitch_max': (0.01, 1.57, '最大俯仰角 (rad)'),
    'attitude.thrust_min': (0.01, 1.0, '最小推力'),
    'attitude.thrust_max': (1.0, 10.0, '最大推力'),
}
