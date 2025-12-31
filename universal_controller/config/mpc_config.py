"""MPC 配置

模型预测控制器的配置参数：
- 预测时域
- 代价函数权重
- 健康监控参数
- Fallback 求解器参数

注意:
=====
ACADOS 求解器参数 (MPC_QP_SOLVER, MPC_INTEGRATOR_TYPE, MPC_NLP_SOLVER_TYPE, MPC_NLP_MAX_ITER)
是算法标准配置，定义在 core/constants.py 中。这些参数：
- 基于数值优化理论选择
- 修改需要深入理解 ACADOS 内部机制
- 不应由用户配置

健康监控的数值稳定性阈值 (CONDITION_NUMBER_THRESH, KKT_RESIDUAL_THRESH 等)
也定义在 core/constants.py 中，因为这些是数值分析的标准阈值。
"""

MPC_CONFIG = {
    'horizon': 20,                # MPC 预测时域
    'horizon_degraded': 10,       # 降级时的预测时域
    'dt': 0.1,                    # 时间步长 (秒) - 主配置，trajectory.default_dt_sec 自动继承
    'horizon_change_min_interval': 1.0,  # Horizon 调整节流间隔 (秒) - 防止频繁重新初始化求解器
    
    # 代价函数权重
    'weights': {
        'position': 10.0,         # 位置跟踪权重 (Q_pos)
        'velocity': 1.0,          # 速度跟踪权重 (Q_vel)
        'heading': 5.0,           # 航向跟踪权重 (Q_heading)
        # 控制输入权重 (R 矩阵) - 惩罚控制输入的大小，实现平滑控制
        'control_accel': 0.1,     # 加速度控制权重 (用于 ax, ay, az)
        'control_alpha': 0.1,     # 角加速度控制权重 (用于 alpha)
    },
    
    # MPC 健康监控参数 (用户可调的部分)
    'health_monitor': {
        'time_warning_thresh_ms': 8,       # 求解时间警告阈值 (ms)
        'time_critical_thresh_ms': 15,     # 求解时间临界阈值 (ms)
        'time_recovery_thresh_ms': 6,      # 求解时间恢复阈值 (ms)
        'consecutive_warning_limit': 3,    # 连续警告次数限制
        'consecutive_recovery_limit': 5,   # 连续恢复次数限制
        # 以下参数是算法内部参数，通常不需要修改
        'recovery_multiplier': 2.0,        # 备选恢复条件的倍数
        'consecutive_good_for_decay': 2,   # 连续良好次数达到此值后开始衰减
        'timeout_decay_rate': 2,           # 恢复区超时计数衰减速率
        'middle_zone_decay_rate': 1,       # 中间区超时计数衰减速率 (比恢复区慢，提供滞后)
        # 注意: 数值稳定性阈值 (CONDITION_NUMBER_THRESH, KKT_RESIDUAL_THRESH 等)
        # 是数值分析标准阈值，定义在 core/constants.py 中，不可配置
    },
    
    # Fallback 求解器参数
    # 注意: heading_kp 已统一到 backup.kp_heading，确保与 Pure Pursuit 一致
    # MPC 特有参数保留在 mpc.fallback 中
    'fallback': {
        'lookahead_steps': 3,              # MPC fallback 特有: 前瞻步数
    },
    
    # 注意: ACADOS 求解器参数定义在 core/constants.py 中
    # 包括: MPC_QP_SOLVER, MPC_INTEGRATOR_TYPE, MPC_NLP_SOLVER_TYPE, MPC_NLP_MAX_ITER
    # 这些是算法标准配置，不可配置
}

# MPC 配置验证规则
MPC_VALIDATION_RULES = {
    'mpc.horizon': (1, 100, 'MPC 预测时域'),
    'mpc.horizon_degraded': (1, 100, 'MPC 降级预测时域'),
    'mpc.dt': (0.001, 1.0, 'MPC 时间步长 (秒)'),
    'mpc.horizon_change_min_interval': (0.0, 60.0, 'Horizon 调整节流间隔 (秒)'),
    # Fallback 求解器参数 (MPC 特有)
    'mpc.fallback.lookahead_steps': (1, 50, 'Fallback 前瞻步数'),
    # MPC 权重 (必须为非负数)
    'mpc.weights.position': (0.0, None, 'MPC 位置权重'),
    'mpc.weights.velocity': (0.0, None, 'MPC 速度权重'),
    'mpc.weights.heading': (0.0, None, 'MPC 航向权重'),
    'mpc.weights.control_accel': (0.0, None, 'MPC 加速度控制权重'),
    'mpc.weights.control_alpha': (0.0, None, 'MPC 角加速度控制权重'),
    # MPC 健康监控参数
    'mpc.health_monitor.time_warning_thresh_ms': (0.1, 1000, '求解时间警告阈值 (ms)'),
    'mpc.health_monitor.time_critical_thresh_ms': (0.1, 1000, '求解时间临界阈值 (ms)'),
    'mpc.health_monitor.time_recovery_thresh_ms': (0.1, 1000, '求解时间恢复阈值 (ms)'),
    'mpc.health_monitor.consecutive_warning_limit': (1, 100, '连续警告次数限制'),
    'mpc.health_monitor.consecutive_recovery_limit': (1, 100, '连续恢复次数限制'),
    # 注意: ACADOS 求解器参数已移至 constants.py，不再需要验证
}
