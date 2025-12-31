"""
通用常量和基础数学函数定义

本模块定义了整个控制器系统使用的通用常量和不依赖其他模块的基础数学函数。

=============================================================================
设计原则
=============================================================================

本模块只包含以下类型的常量：

1. 物理常量 (Physical Constants)
   - 基于物理定律的固定值，如重力加速度
   - 在地球表面几乎不变，不需要配置

2. 数学/几何常量 (Mathematical/Geometric Constants)
   - 基于数学定义的边界值，如置信度范围 [0, 1]
   - 基于几何原理的角度阈值，如 π/2, π/3

3. 数值稳定性常量 (Numerical Stability Constants)
   - 基于 IEEE 754 双精度浮点数特性选择
   - 用于避免除零、数值溢出等问题

4. 算法标准参数 (Algorithm Standard Parameters)
   - 基于数值分析理论的标准阈值
   - 基于控制理论的标准配置
   - 修改需要深入理解算法原理，不应由用户配置

=============================================================================
不放在本模块的内容
=============================================================================

以下内容应放在 config/*.py 配置文件中：

- 可调优的控制参数（如 MPC 权重、前瞻距离）
- 平台相关的约束参数（如最大速度、最大加速度）
- 超时和阈值参数（如 watchdog 超时）
- 任何可能需要根据实际情况调整的参数

=============================================================================
常量分类
=============================================================================

1. 物理常量 (Physical Constants)
   - DEFAULT_GRAVITY: 标准重力加速度

2. 数学边界常量 (Mathematical Bounds)
   - CONFIDENCE_MIN/MAX: 置信度定义域
   - QUATERNION_NORM_SQ_MIN/MAX: 四元数有效性范围

3. 数值稳定性常量 (Numerical Stability)
   - EPSILON, EPSILON_SMALL, EPSILON_ANGLE, EPSILON_VELOCITY
   - MIN_DENOMINATOR, MIN_SEGMENT_LENGTH, MIN_RELATIVE_CROSS

4. 几何常量 (Geometric Constants)
   - PURE_PURSUIT_ANGLE_THRESH: π/3，Pure Pursuit 经典切换角度
   - HEADING_CONTROL_ANGLE_THRESH: π/2，正交方向
   - REAR_ANGLE_THRESH: 正后方检测阈值

5. 算法标准参数 (Algorithm Standard Parameters)
   - MPC 求解器配置: MPC_QP_SOLVER, MPC_INTEGRATOR_TYPE 等
   - 数值稳定性阈值: CONDITION_NUMBER_THRESH, KKT_RESIDUAL_THRESH 等

=============================================================================
使用示例
=============================================================================

    from universal_controller.core.constants import (
        EPSILON, EPSILON_SMALL, EPSILON_ANGLE,
        DEFAULT_GRAVITY, NEVER_RECEIVED_TIME_MS,
        normalize_angle, angle_difference
    )
    
    # 避免除零
    if denominator > EPSILON:
        result = numerator / denominator
    
    # 角度归一化
    theta = normalize_angle(theta + delta)
    
    # 角度差计算
    error = angle_difference(target, current)
"""

import numpy as np


# =============================================================================
# 基础数学函数 (不依赖其他模块，避免循环导入)
# =============================================================================

def normalize_angle(angle: float) -> float:
    """
    将角度归一化到 [-π, π] 范围
    
    使用 arctan2(sin, cos) 方法，数值稳定且高效。
    
    Args:
        angle: 输入角度 (弧度)
    
    Returns:
        归一化后的角度 (弧度)，范围 [-π, π]
    
    Examples:
        >>> normalize_angle(3 * np.pi)  # 约等于 -π
        >>> normalize_angle(-2 * np.pi)  # 约等于 0
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最短差值
    
    结果表示从 angle2 到 angle1 的最短旋转方向和角度。
    正值表示逆时针旋转，负值表示顺时针旋转。
    
    Args:
        angle1: 目标角度 (弧度)
        angle2: 起始角度 (弧度)
    
    Returns:
        角度差 (弧度)，范围 [-π, π]，表示从 angle2 到 angle1 的最短旋转
    
    Examples:
        >>> angle_difference(0.1, -0.1)  # 约等于 0.2
        >>> angle_difference(-np.pi + 0.1, np.pi - 0.1)  # 约等于 0.2 (跨越 ±π)
    """
    return normalize_angle(angle1 - angle2)

# =============================================================================
# 数值稳定性常量 (Numerical Stability Constants)
# =============================================================================

# 通用小量阈值
# 用于一般的数值比较和避免除零
# 选择依据: 1e-6 是 float32 精度的合理阈值，对于 float64 更加安全
EPSILON = 1e-6

# 更严格的小量阈值
# 用于需要更高精度的计算，如协方差矩阵操作
# 选择依据: 1e-9 接近 float32 的机器精度，对于 float64 仍有足够余量
EPSILON_SMALL = 1e-9

# 角度计算阈值
# 用于角度比较和归一化
# 选择依据: 1e-9 rad ≈ 5.7e-8 度，足够精确
EPSILON_ANGLE = 1e-9

# 速度计算最小阈值
# 用于避免低速时的数值不稳定（如航向角计算）
# 选择依据: 1e-6 m/s 是实际不可能的速度，可安全用于除法
EPSILON_VELOCITY = 1e-6

# 最小分母值
# 用于通用的除法保护
# 选择依据: 与 EPSILON_SMALL 相同，确保除法结果在合理范围内
MIN_DENOMINATOR = 1e-9

# 最小线段长度
# 用于几何计算中判断点是否重合
# 选择依据: 1e-6 m = 1 微米，远小于任何实际测量精度
MIN_SEGMENT_LENGTH = 1e-6

# 最小相对叉积
# 用于判断向量是否共线
# 选择依据: 相对于分母的比例阈值，1e-6 表示非常接近共线
MIN_RELATIVE_CROSS = 1e-6


# =============================================================================
# 物理常量 (Physical Constants)
# 这些是不可配置的物理常量，在代码中固定使用
# =============================================================================

# 标准重力加速度 (m/s²)
# 地球表面重力加速度范围: 9.78-9.83 m/s²
# 使用标准值 9.81，不作为可配置参数
# 理由: 重力加速度是物理常量，在实际应用中几乎不变
DEFAULT_GRAVITY = 9.81


# =============================================================================
# 时间常量 (Time Constants)
# =============================================================================

# 表示"从未收到消息"的超时值 (毫秒)
# 使用大的有限值代替无穷大，避免 JSON 序列化问题
# 1e9 ms ≈ 11.5 天，足够表示"非常长时间"
NEVER_RECEIVED_TIME_MS = 1e9


# =============================================================================
# 四元数验证常量 (Quaternion Validation Constants)
# =============================================================================

# 四元数范数平方的有效范围
# 
# 设计说明：
# - 单位四元数的范数平方应该为 1.0
# - 由于数值误差，允许一定的偏差
# - 超出范围的四元数被认为无效
#
# 阈值选择：
# - MIN: 0.25 (范数 > 0.5) - 低于此值无法可靠归一化
# - MAX: 4.0 (范数 < 2.0) - 高于此值认为数据明显错误
#
# 范围 [0.5, 2.0] 的理由：
# - 正常的数值误差不会导致范数偏离 1.0 超过 2 倍
# - 如果范数 < 0.5 或 > 2.0，说明数据本身有问题
QUATERNION_NORM_SQ_MIN = 0.25
QUATERNION_NORM_SQ_MAX = 4.0


# =============================================================================
# 置信度边界常量 (Confidence Bounds)
# 这些是数学定义，不可配置
# =============================================================================

# 置信度最小值
# 数学定义: 置信度是 [0, 1] 区间的概率值
CONFIDENCE_MIN = 0.0

# 置信度最大值
CONFIDENCE_MAX = 1.0


# =============================================================================
# 协方差矩阵常量 (Covariance Matrix Constants)
# =============================================================================

# 协方差矩阵最小特征值
# 用于保证协方差矩阵正定，防止数值奇异
# 选择依据: 1e-6 是 float64 精度下的安全阈值
COVARIANCE_MIN_EIGENVALUE = 1e-6

# 协方差矩阵初始值
# 用于初始化协方差矩阵对角线
COVARIANCE_INITIAL_VALUE = 0.1


# =============================================================================
# 加速度计算常量 (Acceleration Calculation Constants)
# =============================================================================

# 加速度计算的最小时间间隔 (秒)
# 用于避免除零，当 dt < 此值时跳过加速度计算
# 选择依据: 1ms 是实际控制系统中不可能的时间间隔
MIN_DT_FOR_ACCEL = 0.001

# 加速度计算的最大时间间隔 (秒)
# 超过此值认为数据不连续，不计算加速度
MAX_DT_FOR_ACCEL = 1.0


# =============================================================================
# 过渡动画常量 (Transition Animation Constants)
# =============================================================================

# 过渡完成阈值
# 指数衰减到此比例时认为过渡完成
# 数学依据: 1 - e^(-3) ≈ 0.95，即 3 个时间常数后
TRANSITION_COMPLETION_THRESHOLD = 0.95


# =============================================================================
# 几何常量 (Geometric Constants)
# 基于控制理论和几何原理的固定角度阈值
# 这些是 Pure Pursuit 算法的标准参数，不应由用户配置
# =============================================================================

# Pure Pursuit 模式角度阈值 (rad, π/3 ≈ 60°)
# 当目标点角度小于此值时，使用标准 Pure Pursuit 曲率控制
# 选择依据: 60° 是 Pure Pursuit 算法的经典切换角度，基于控制理论
# 数学意义: cos(60°) = 0.5，此时曲率控制仍然有效
PURE_PURSUIT_ANGLE_THRESH = 1.047197551  # π/3，精确值

# 航向控制模式角度阈值 (rad, π/2 = 90°)
# 当目标点角度大于此值时，切换到航向误差控制模式
# 选择依据: 90° 是正交方向，几何意义明确
# 数学意义: cos(90°) = 0，Pure Pursuit 曲率公式在此点失效
HEADING_CONTROL_ANGLE_THRESH = 1.570796327  # π/2，精确值

# 正后方检测阈值 (rad, ~162°)
# 当目标点角度大于此值时，认为目标在正后方
# 选择依据: π - 18° ≈ 162°，留出一定的判断余量避免边界震荡
# 数学意义: 接近 180° 但留有余量，防止 ±π 跳变导致的不稳定
REAR_ANGLE_THRESH = 2.827433388  # π - π/10，精确值


# =============================================================================
# MPC 求解器标准配置 (MPC Solver Standard Configuration)
# 这些是 ACADOS 求解器的标准配置，基于数值优化理论
# 修改需要深入理解 ACADOS 内部机制，不应由用户配置
# =============================================================================

# QP 求解器类型
# PARTIAL_CONDENSING_HPIPM 是 ACADOS 推荐的高性能求解器
# 基于 HPIPM (High-Performance Interior Point Method) 算法
# 适用于中等规模的 MPC 问题，平衡了求解速度和数值稳定性
MPC_QP_SOLVER = "PARTIAL_CONDENSING_HPIPM"

# 积分器类型
# ERK (Explicit Runge-Kutta) 适用于非刚性系统
# 对于机器人运动学模型，ERK 提供了足够的精度和效率
# 刚性系统应使用 IRK (Implicit Runge-Kutta)
MPC_INTEGRATOR_TYPE = "ERK"

# NLP 求解器类型
# SQP_RTI (Sequential Quadratic Programming - Real-Time Iteration)
# 专为实时控制应用设计，每个控制周期只执行一次 QP 迭代
# 牺牲了收敛精度换取确定性的计算时间
MPC_NLP_SOLVER_TYPE = "SQP_RTI"

# NLP 求解器最大迭代次数
# 对于 SQP_RTI，此参数主要影响初始化阶段
# 50 次迭代足以处理大多数初始化场景
MPC_NLP_MAX_ITER = 50


# =============================================================================
# 数值稳定性阈值 (Numerical Stability Thresholds)
# 这些是数值优化算法的标准阈值，基于数值分析理论
# 修改需要理解数值优化原理，不应由用户配置
# =============================================================================

# 条件数警告阈值
# 矩阵条件数超过此值时发出警告，表明数值稳定性可能受影响
# 选择依据: 1e8 是数值分析中常用的"病态"阈值
# 条件数 κ(A) = ||A|| * ||A^(-1)||，反映矩阵对扰动的敏感度
CONDITION_NUMBER_THRESH = 1.0e8

# 条件数恢复阈值
# 矩阵条件数低于此值时认为恢复正常
# 选择依据: 1e5 提供了足够的滞后，避免在阈值边界频繁切换
# 恢复阈值 < 警告阈值，形成滞后区间 [1e5, 1e8]
CONDITION_NUMBER_RECOVERY = 1.0e5

# KKT 残差阈值
# 优化问题的 KKT (Karush-Kuhn-Tucker) 条件残差阈值
# 选择依据: 1e-3 是实时控制中常用的收敛精度
# 更严格的阈值会增加计算时间，更宽松的阈值会降低控制精度
KKT_RESIDUAL_THRESH = 1.0e-3


# =============================================================================
# 安全裕度常量 (Safety Margin Constants)
# 这些是安全工程的标准裕度值，基于工程实践经验
# 不应由用户随意修改，以确保系统安全性
# =============================================================================

# 速度限制裕度
# 安全检查时允许的速度超限比例
# 选择依据: 10% 裕度是工程实践中的标准值，允许短暂的超调
# 过小会导致频繁触发安全限制，过大会降低安全性
SAFETY_VELOCITY_MARGIN = 1.1

# 加速度限制裕度
# 安全检查时允许的加速度超限比例
# 选择依据: 50% 裕度考虑了加速度测量的噪声和滤波延迟
# 加速度是速度的导数，噪声放大效应明显，需要更大裕度
SAFETY_ACCEL_MARGIN = 1.5

# 加速度预热裕度上限
# 滤波器预热期间裕度倍数的硬性上限
# 选择依据: 2.0 确保即使配置错误，安全检查也不会过于宽松
# 这是一个防御性设计，防止用户配置导致安全隐患
SAFETY_ACCEL_WARMUP_MARGIN_MAX = 2.0

# 绝对加速度上限倍数
# 无论任何情况都不能超过的加速度上限（相对于 a_max）
# 选择依据: 2.0 倍是硬性安全限制，防止任何情况下的危险加速度
# 即使在滤波器未收敛时也能捕获极端情况
SAFETY_ACCEL_ABSOLUTE_MAX_MULTIPLIER = 2.0


# =============================================================================
# EKF 算法常量 (EKF Algorithm Constants)
# 这些是扩展卡尔曼滤波器的数值稳定性参数
# 基于数值分析理论和控制理论，不应由用户配置
# =============================================================================

# 最大倾斜角 (rad, ~60°)
# IMU 姿态角有效性检查阈值
# 选择依据: 60° 是地面机器人合理的最大倾斜角
# 超过此角度认为 IMU 数据异常或机器人已翻倒
EKF_MAX_TILT_ANGLE = 1.047197551  # π/3

# Jacobian 计算的最小速度阈值 (m/s)
# 用于避免低速时 Jacobian 矩阵的数值不稳定
# 选择依据: 0.01 m/s 是实际静止状态，低于此值航向角计算无意义
EKF_MIN_VELOCITY_FOR_JACOBIAN = 0.01

# 协方差爆炸检测阈值
# 当协方差矩阵范数超过此值时认为估计器发散
# 选择依据: 1000.0 是经验值，正常运行时协方差范数远小于此值
EKF_COVARIANCE_EXPLOSION_THRESH = 1000.0

# 创新度异常检测阈值
# 当测量创新度超过此值时认为测量异常
# 选择依据: 10.0 约等于 3σ (假设标准差为 3)，统计学上的异常值判定
EKF_INNOVATION_ANOMALY_THRESH = 10.0


# =============================================================================
# 轨迹验证常量 (Trajectory Validation Constants)
# 这些是轨迹数据有效性检查的边界值
# 基于数值稳定性和物理合理性，不应由用户配置
# =============================================================================

# 最小时间步长 (秒)
# 轨迹点之间的最小时间间隔
# 选择依据: 10ms 是实际控制系统的最小合理时间步长
# 低于此值会导致数值不稳定（速度/加速度计算）
TRAJECTORY_MIN_DT_SEC = 0.01

# 最大时间步长 (秒)
# 轨迹点之间的最大时间间隔
# 选择依据: 1.0s 是轨迹连续性的上限
# 超过此值认为轨迹不连续，无法进行有效插值
TRAJECTORY_MAX_DT_SEC = 1.0

# 最大合理坐标值 (米)
# 轨迹点坐标的有效范围上限
# 选择依据: 100m 是局部轨迹的合理范围
# 超过此值认为数据异常（可能是坐标系错误或数据损坏）
TRAJECTORY_MAX_COORD = 100.0


# =============================================================================
# 一致性检查常量 (Consistency Check Constants)
# 这些是轨迹一致性检查的安全策略参数
# =============================================================================

# 数据无效时的保守置信度
# 当轨迹数据无效时使用的保守 alpha 值
# 选择依据: 0.5 是中性值，既不完全信任也不完全拒绝
# 这是一个安全策略，确保异常情况下系统行为可预测
CONSISTENCY_INVALID_DATA_CONFIDENCE = 0.5


# =============================================================================
# 姿态控制常量 (Attitude Control Constants)
# 这些是四旋翼姿态控制的物理约束参数
# 基于飞行动力学原理，不应由用户配置
# =============================================================================

# 最小推力因子
# 推力加速度相对于重力的最小比例
# 选择依据: 0.1 确保无人机不会进入自由落体或倒飞状态
# 物理意义: 最小推力 = 0.1 * g，足以维持基本的姿态控制
ATTITUDE_MIN_THRUST_FACTOR = 0.1

# 姿态角饱和后推力重计算的最小因子
# 当 cos(roll) * cos(pitch) 小于此值时使用简化计算
# 选择依据: 0.1 对应约 84° 的组合倾斜角，此时推力计算不可靠
# 物理意义: 极端姿态下使用保守的推力估计
ATTITUDE_FACTOR_MIN = 0.1


# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    # 数值稳定性常量
    'EPSILON',
    'EPSILON_SMALL',
    'EPSILON_ANGLE',
    'EPSILON_VELOCITY',
    'MIN_DENOMINATOR',
    'MIN_SEGMENT_LENGTH',
    'MIN_RELATIVE_CROSS',
    # 物理常量
    'DEFAULT_GRAVITY',
    # 时间常量
    'NEVER_RECEIVED_TIME_MS',
    # 四元数验证常量
    'QUATERNION_NORM_SQ_MIN',
    'QUATERNION_NORM_SQ_MAX',
    # 置信度边界常量
    'CONFIDENCE_MIN',
    'CONFIDENCE_MAX',
    # 协方差矩阵常量
    'COVARIANCE_MIN_EIGENVALUE',
    'COVARIANCE_INITIAL_VALUE',
    # 加速度计算常量
    'MIN_DT_FOR_ACCEL',
    'MAX_DT_FOR_ACCEL',
    # 过渡动画常量
    'TRANSITION_COMPLETION_THRESHOLD',
    # Pure Pursuit 几何常量
    'PURE_PURSUIT_ANGLE_THRESH',
    'HEADING_CONTROL_ANGLE_THRESH',
    'REAR_ANGLE_THRESH',
    # MPC 求解器常量
    'MPC_QP_SOLVER',
    'MPC_INTEGRATOR_TYPE',
    'MPC_NLP_SOLVER_TYPE',
    'MPC_NLP_MAX_ITER',
    # 数值稳定性阈值常量
    'CONDITION_NUMBER_THRESH',
    'CONDITION_NUMBER_RECOVERY',
    'KKT_RESIDUAL_THRESH',
    # 安全裕度常量
    'SAFETY_VELOCITY_MARGIN',
    'SAFETY_ACCEL_MARGIN',
    'SAFETY_ACCEL_WARMUP_MARGIN_MAX',
    'SAFETY_ACCEL_ABSOLUTE_MAX_MULTIPLIER',
    # EKF 算法常量
    'EKF_MAX_TILT_ANGLE',
    'EKF_MIN_VELOCITY_FOR_JACOBIAN',
    'EKF_COVARIANCE_EXPLOSION_THRESH',
    'EKF_INNOVATION_ANOMALY_THRESH',
    # 轨迹验证常量
    'TRAJECTORY_MIN_DT_SEC',
    'TRAJECTORY_MAX_DT_SEC',
    'TRAJECTORY_MAX_COORD',
    # 一致性检查常量
    'CONSISTENCY_INVALID_DATA_CONFIDENCE',
    # 姿态控制常量
    'ATTITUDE_MIN_THRUST_FACTOR',
    'ATTITUDE_FACTOR_MIN',
    # 基础数学函数
    'normalize_angle',
    'angle_difference',
]
