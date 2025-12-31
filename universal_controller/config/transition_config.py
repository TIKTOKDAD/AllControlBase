"""平滑过渡配置

控制器状态切换时的平滑过渡参数：
- 过渡类型 (指数/线性)
- 时间常数
- 最大过渡时长

注意:
=====
过渡完成阈值 (completion_threshold) 是数学收敛判定标准，
已移至 core/constants.py 中的 TRANSITION_COMPLETION_THRESHOLD。
"""

# 平滑过渡配置
TRANSITION_CONFIG = {
    'type': 'exponential',            # 过渡类型: 'exponential' 或 'linear'
    'tau': 0.1,                       # 指数过渡时间常数 (秒)
    'max_duration': 0.5,              # 最大过渡时长 (秒)
    'duration': 0.2,                  # 线性过渡时长 (秒)
    # 注意: completion_threshold 已移至 constants.py (TRANSITION_COMPLETION_THRESHOLD)
}

# 过渡配置验证规则
TRANSITION_VALIDATION_RULES = {
    'transition.tau': (0.001, 10.0, '过渡时间常数 (秒)'),
    'transition.max_duration': (0.01, 10.0, '最大过渡时长 (秒)'),
    'transition.duration': (0.01, 10.0, '线性过渡时长 (秒)'),
    # 注意: completion_threshold 已移至 constants.py，不再需要验证
}

__all__ = [
    'TRANSITION_CONFIG',
    'TRANSITION_VALIDATION_RULES',
]
