"""坐标变换配置

坐标变换器的配置参数：
- 坐标系名称
- 超时参数
- 漂移估计参数
- 延迟补偿参数

坐标系说明 (不需要建图/定位):
==================================

    base_link (机体坐标系)              odom (里程计坐标系)
    ┌───────────────────┐               ┌─────────────────────┐
    │       ↑ X         │               │                     │
    │       │           │    坐标变换    │    机器人轨迹       │
    │    ←──┼──→        │  ───────────→ │    ○──○──○──○       │
    │     Y │           │  base_link→odom│                     │
    │       ↓           │               │    启动位置 ●       │
    └───────────────────┘               └─────────────────────┘

- base_link: 网络输出轨迹的坐标系 (局部坐标，当前位置为原点)
- odom: 控制器工作坐标系 (里程计坐标系，从启动位置累积)
"""

# 坐标变换配置
TRANSFORM_CONFIG = {
    # 坐标系配置
    'target_frame': 'odom',           # 目标坐标系 (控制器工作坐标系)
    'source_frame': 'base_link',      # 源坐标系 (网络输出轨迹的坐标系)
    'timeout_ms': 10,                 # TF2 查询超时 (ms)
    
    # 降级检测参数
    'fallback_duration_limit_ms': 500,   # 降级持续限制 (ms)
    'fallback_critical_limit_ms': 1000,  # 临界降级限制 (ms)
    
    # 漂移估计参数
    'drift_estimation_enabled': False,    # 漂移估计开关
    'recovery_correction_enabled': True,  # 恢复校正开关
    'max_accumulated_drift': 1.0,         # 漂移累积上限 (米)
    'drift_rate': 0.01,                   # 漂移率 (米/秒)
    'drift_velocity_factor': 0.1,         # 速度漂移因子
    'max_drift_dt': 0.5,                  # 漂移估计最大时间间隔 (秒)
    'drift_correction_thresh': 0.01,      # 漂移校正阈值 (米/弧度)
    
    # 延迟补偿参数
    'max_delay_compensation_sec': 0.5,    # 最大延迟补偿时间 (秒)
    
    # 坐标系验证
    'expected_source_frames': ['base_link', 'base_footprint', 'base_link_0', '', 'odom'],
    'warn_unexpected_frame': True,        # 对非期望坐标系发出警告
}

# 坐标变换配置验证规则
TRANSFORM_VALIDATION_RULES = {
    'transform.timeout_ms': (1, 1000, 'TF2 查询超时 (ms)'),
    'transform.fallback_duration_limit_ms': (0, 10000, 'TF2 降级持续限制 (ms)'),
    'transform.fallback_critical_limit_ms': (0, 30000, 'TF2 临界降级限制 (ms)'),
    'transform.drift_rate': (0.0, 1.0, '漂移率 (米/秒)'),
    'transform.drift_velocity_factor': (0.0, 1.0, '速度漂移因子'),
    'transform.max_drift_dt': (0.01, 5.0, '漂移估计最大时间间隔 (秒)'),
    'transform.drift_correction_thresh': (0.0, 1.0, '漂移校正阈值 (米/弧度)'),
    'transform.max_delay_compensation_sec': (0.0, 2.0, '最大延迟补偿时间 (秒)'),
}

__all__ = [
    'TRANSFORM_CONFIG',
    'TRANSFORM_VALIDATION_RULES',
]
