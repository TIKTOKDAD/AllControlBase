# 控制器配置参考文档

本文档列出所有配置参数及其说明。

## 配置架构

### 配置文件结构

```
controller_ros/config/
├── base/
│   ├── controller_params.yaml    # ROS 层基础配置 (话题、时钟)
│   └── internal_params.yaml      # 内部实现参数 (用户不应修改)
├── platforms/
│   ├── _template.yaml            # 用户配置模板
│   ├── differential.yaml         # 差速车配置
│   ├── omni.yaml                 # 全向车配置
│   ├── ackermann.yaml            # 阿克曼车配置
│   ├── quadrotor.yaml            # 四旋翼配置
│   └── turtlebot1.yaml           # TurtleBot1 专用配置
└── tools/
    └── visualizer_params.yaml    # 可视化工具配置

universal_controller/config/
├── *.py                          # Python 默认配置 (安全网)
└── README.md                     # 配置系统说明
```

### 配置加载顺序 (从低到高)

```
1. universal_controller/config/*.py  - Python 默认配置 (安全网)
2. base/controller_params.yaml       - ROS 层基础配置 (话题、时钟)
3. base/internal_params.yaml         - 内部实现参数 (用户不应修改)
4. platforms/{platform}.yaml         - 平台特定配置 (用户可调参数)
5. ROS 参数服务器                     - 运行时覆盖
```

### 配置文件职责

| 文件 | 职责 | 用户是否应修改 |
|------|------|----------------|
| `controller_params.yaml` | ROS 话题默认值、时钟配置 | 通常不需要 |
| `internal_params.yaml` | 内部实现参数 (EKF、滤波器、状态机等) | **不应修改** |
| `platforms/*.yaml` | 平台特定的可调参数 | **主要配置入口** |
| `_template.yaml` | 创建新平台配置的模板 | 复制后修改 |

### 参数分类原则

| 分类 | 定义位置 | 说明 | 示例 |
|------|----------|------|------|
| **物理常量** | `core/constants.py` | 不可配置的物理/数学常量 | 重力加速度、EPSILON |
| **平台运动学** | `platform_config.py` | 平台运动学模型 (不可配置) | 状态维度、约束类型 |
| **用户可调参数** | `platforms/*.yaml` | 需要根据平台调整的参数 | MPC 权重、速度约束、超时 |
| **内部实现参数** | `internal_params.yaml` | 内部算法参数，用户不应修改 | EKF 噪声、滤波器系数 |
| **ROS 层特有参数** | `controller_params.yaml` | ROS 特有配置 | topics、clock |

### 设计原则

1. **物理常量在代码中固定**：重力加速度、数值精度等
2. **用户可调参数在平台配置中显式列出**：所有需要根据平台特性调整的参数
3. **内部实现参数独立存放**：在 `internal_params.yaml` 中，用户不应修改
4. **Python 默认值作为安全网**：当 YAML 缺失某项时使用默认值
5. **用户只需面对 controller_ros 包**：所有配置都在这个 ROS 包中统一管理

### 配置分类表

| 配置块 | 定义位置 | 说明 |
|--------|----------|------|
| `system` | platforms/*.yaml | 系统基础配置 |
| `constraints` | platforms/*.yaml | 运动约束 |
| `mpc` (用户参数) | platforms/*.yaml | MPC 控制器配置 |
| `mpc` (内部参数) | internal_params.yaml | MPC 求解器内部参数 |
| `safety` (用户参数) | platforms/*.yaml | 安全配置 |
| `safety` (内部参数) | internal_params.yaml | 安全裕度内部参数 |
| `backup` (用户参数) | platforms/*.yaml | 备份控制器配置 |
| `backup` (内部参数) | internal_params.yaml | 备份控制器内部参数 |
| `trajectory` (用户参数) | platforms/*.yaml | 轨迹配置 |
| `trajectory` (内部参数) | internal_params.yaml | 轨迹验证内部参数 |
| `consistency` (用户参数) | platforms/*.yaml | 一致性检查配置 |
| `consistency` (内部参数) | internal_params.yaml | 一致性检查内部参数 |
| `tracking` (用户参数) | platforms/*.yaml | 跟踪质量评估配置 |
| `tracking` (内部参数) | internal_params.yaml | 跟踪质量评估内部参数 |
| `watchdog` | platforms/*.yaml | 超时配置 |
| `diagnostics` (用户参数) | platforms/*.yaml | 诊断配置 |
| `diagnostics` (内部参数) | internal_params.yaml | 诊断内部参数 |
| `transform` (用户参数) | platforms/*.yaml | 坐标变换配置 |
| `transform` (内部参数) | internal_params.yaml | 坐标变换内部参数 |
| `ekf` | internal_params.yaml | EKF 状态估计器 |
| `transition` | internal_params.yaml | 状态过渡配置 |
| `attitude` (用户参数) | platforms/*.yaml | 姿态控制配置 (仅四旋翼) |
| `attitude` (内部参数) | internal_params.yaml | 姿态控制内部参数 |
| `topics` | controller_params.yaml | ROS 话题配置 |
| `cmd_vel_adapter` | platforms/*.yaml | cmd_vel 适配器配置 |
| `clock` | controller_params.yaml | 时钟配置 |

### 内部实现参数 (在 internal_params.yaml 中定义)

以下参数是内部实现细节，定义在 `base/internal_params.yaml` 中，用户不应修改：

| 模块 | 参数 | 理由 |
|------|------|------|
| `ekf.*` | EKF 状态估计器参数 | 参数高度耦合，需要专业知识 |
| `transition.*` | 状态过渡参数 | 内部控制逻辑 |
| `mpc.solver.*` | ACADOS 求解器参数 | 需要优化器专业知识 |
| `mpc.health_monitor.condition_number_*` | 数值稳定性参数 | 内部健康监控 |
| `safety.accel_filter_*` | 加速度滤波参数 | 内部安全机制 |
| `safety.state_machine.*` (高级) | 状态机高级参数 | 内部控制逻辑 |
| `backup.heading_mode` | 航向控制模式 | 内部控制逻辑 |
| `backup.*_angle_thresh` | 角度阈值参数 | 内部控制逻辑 |
| `transform.drift_*` | 漂移估计参数 | 内部补偿机制 |
| `tracking.weights.*` | Dashboard 权重 | 仅影响显示 |
| `tracking.rating.*` | Dashboard 评级 | 仅影响显示 |
| `diagnostics.*` (高级) | 诊断内部参数 | 内部日志控制 |
| `attitude.*` (高级) | 姿态控制内部参数 | 内部控制逻辑 |

### 配置文件结构

```
universal_controller/config/
├── platform_config.py      # 平台运动学配置（物理模型，不可配置）
├── system_config.py        # 系统基础配置
├── mpc_config.py           # MPC 控制器配置
├── safety_config.py        # 安全和约束配置
├── ekf_config.py           # EKF 状态估计器配置（高级参数）
├── attitude_config.py      # 姿态控制配置（四旋翼）
├── trajectory_config.py    # 轨迹配置
├── consistency_config.py   # 一致性检查配置
├── transform_config.py     # 坐标变换配置（算法层参数）
├── transition_config.py    # 平滑过渡配置（高级参数）
├── backup_config.py        # 备份控制器配置
├── mock_config.py          # Mock 配置（测试用，内部参数）
├── default_config.py       # 配置聚合器
└── validation.py           # 配置验证逻辑

controller_ros/config/
├── base/controller_params.yaml  # ROS 层基础配置 (话题默认值)
└── platforms/
    ├── _template.yaml           # 完整配置模板
    ├── differential.yaml        # 差速车配置
    ├── omni.yaml                # 全向车配置
    ├── ackermann.yaml           # 阿克曼车配置
    ├── quadrotor.yaml           # 四旋翼配置
    └── turtlebot1.yaml          # TurtleBot1 专用配置

controller_ros/src/controller_ros/utils/param_loader.py
└── ROS 层特有配置默认值:
    ├── TOPICS_DEFAULTS          # 话题默认值
    ├── TRANSFORM_ROS_DEFAULTS   # TF2 参数默认值
    ├── CMD_VEL_ADAPTER_DEFAULTS # cmd_vel 适配器默认值
    └── CLOCK_DEFAULTS           # 时钟配置默认值
```

---

## 物理常量 (代码中定义，不可配置)

这些常量定义在 `universal_controller/core/constants.py` 中：

| 常量 | 值 | 说明 |
|------|-----|------|
| `DEFAULT_GRAVITY` | 9.81 | 重力加速度 (m/s²) |
| `EPSILON` | 1e-6 | 通用数值精度阈值 |
| `EPSILON_SMALL` | 1e-9 | 高精度数值阈值 |
| `EPSILON_ANGLE` | 1e-9 | 角度计算精度阈值 |
| `EPSILON_VELOCITY` | 1e-6 | 速度计算精度阈值 |
| `MIN_DENOMINATOR` | 1e-9 | 除法保护最小分母 |
| `MIN_SEGMENT_LENGTH` | 1e-6 | 最小线段长度 (m) |
| `NEVER_RECEIVED_TIME_MS` | 1e9 | 表示"从未收到"的时间值 |

---

## 平台运动学配置 (代码中定义，不可配置)

定义在 `universal_controller/config/platform_config.py` 中，描述各平台的物理运动学特性：

| 平台 | 活跃状态维度 | 控制维度 | 运动学约束 |
|------|-------------|----------|-----------|
| differential | px, py, vx, theta, omega | vx, omega | vy=0 (非完整约束) |
| ackermann | px, py, vx, theta | vx, omega | vy=0, 曲率约束 |
| omni | px, py, vx, vy, theta, omega | vx, vy, omega | 无 |
| quadrotor | 全部 8 维 | vx, vy, vz, omega | 无 |

---

## 可配置参数

以下所有参数都应在平台配置文件 (`platforms/*.yaml`) 中显式列出。


### 1. system - 系统配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `ctrl_freq` | 50 | [1, 1000] | 控制频率 (Hz) |
| `platform` | "differential" | - | 平台类型: differential/omni/ackermann/quadrotor |

### 2. watchdog - 超时配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `odom_timeout_ms` | 500 | <=0 禁用 | 里程计超时 (ms) |
| `traj_timeout_ms` | 1000 | <=0 禁用 | 轨迹超时 (ms) |
| `traj_grace_ms` | 500 | [0, ∞) | 轨迹宽限期 (ms) |
| `imu_timeout_ms` | -1 | <=0 禁用 | IMU 超时 (ms)，四旋翼必须启用 |
| `startup_grace_ms` | 5000 | [0, ∞) | 启动宽限期 (ms) |

**超时检测时序**：
```
t=0ms    收到轨迹
t=traj_timeout_ms 轨迹超时，进入宽限期
t=traj_timeout_ms + traj_grace_ms 宽限期结束，触发安全停止
```

### 3. diagnostics - 诊断配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `publish_rate` | 10 | [1, 100] | 诊断发布降频率 (每 N 次控制循环发布一次) |

### 4. mpc - MPC 控制器配置

#### 4.1 预测时域

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `horizon` | 20 | [1, 100] | MPC 预测时域，必须 < 轨迹点数 |
| `horizon_degraded` | 10 | [1, 100] | 降级模式预测时域，必须 <= horizon |
| `dt` | 0.1 | [0.001, 1.0] | 时间步长 (秒)，trajectory.default_dt_sec 自动继承此值 |

#### 4.2 代价函数权重

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `weights.position` | 10.0 | [0, ∞) | 位置跟踪权重，增大提高位置精度 |
| `weights.velocity` | 1.0 | [0, ∞) | 速度跟踪权重，增大提高速度精度 |
| `weights.heading` | 5.0 | [0, ∞) | 航向跟踪权重，增大提高航向精度 |
| `weights.control_accel` | 0.1 | [0, ∞) | 加速度控制权重，增大使控制更平滑 |
| `weights.control_alpha` | 0.1 | [0, ∞) | 角加速度控制权重，增大使转向更平滑 |

#### 4.3 健康监控

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `health_monitor.time_warning_thresh_ms` | 8 | [0.1, 1000] | 求解时间警告阈值 (ms) |
| `health_monitor.time_critical_thresh_ms` | 15 | [0.1, 1000] | 求解时间临界阈值 (ms)，超过触发降级 |
| `health_monitor.time_recovery_thresh_ms` | 6 | [0.1, 1000] | 求解时间恢复阈值 (ms) |
| `health_monitor.consecutive_warning_limit` | 3 | [1, 100] | 连续警告次数限制 |
| `health_monitor.consecutive_recovery_limit` | 5 | [1, 100] | 连续恢复次数限制 |

#### 4.4 Fallback 求解器

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `fallback.lookahead_steps` | 3 | [1, 50] | 前瞻步数 |

### 5. constraints - 运动约束

#### 5.1 通用约束

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `v_max` | 2.0 | (0, 100] | 最大线速度 (m/s) |
| `v_min` | 0.0 | [-100, v_max] | 最小线速度 (m/s)，负值允许倒车 |
| `omega_max` | 2.0 | (0, 50] | 最大角速度 (rad/s) |
| `omega_max_low` | 1.0 | (0, omega_max] | 低速时最大角速度 (rad/s) |
| `v_low_thresh` | 0.1 | [0, v_max] | 低速阈值 (m/s) |
| `a_max` | 1.5 | (0, 50] | 最大加速度 (m/s²) |
| `alpha_max` | 3.0 | (0, 100] | 最大角加速度 (rad/s²) |

#### 5.2 全向车/四旋翼约束

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `vx_max` | 1.5 | (0, 100] | X 方向最大速度 (m/s) |
| `vx_min` | -1.5 | [-100, vx_max] | X 方向最小速度 (m/s) |
| `vy_max` | 1.5 | (0, 100] | Y 方向最大速度 (m/s) |
| `vy_min` | -1.5 | [-100, vy_max] | Y 方向最小速度 (m/s) |

#### 5.3 四旋翼约束

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `vz_max` | 2.0 | (0, 100] | Z 方向最大速度 (m/s) |
| `az_max` | 1.0 | (0, 50] | 最大垂直加速度 (m/s²) |

### 6. trajectory - 轨迹配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `low_speed_thresh` | 0.1 | [0, 10] | 低速阈值 (m/s)，用于角速度计算和一致性检查 |
| `min_points` | 2 | [1, 100] | 最小轨迹点数 |
| `max_points` | 100 | [2, 1000] | 最大轨迹点数，必须 > mpc.horizon |
| `max_point_distance` | 10.0 | [0.1, 100] | 相邻点最大距离 (m) |

### 7. consistency - 一致性检查配置

这些参数影响 alpha 值的计算，进而影响 soft/hard velocities 的混合比例。

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `alpha_min` | 0.1 | [0, 1] | α 最小值，低于此值禁用 soft 模式 |
| `kappa_thresh` | 0.5 | [0, 10] | 曲率一致性阈值 |
| `v_dir_thresh` | 0.8 | [0, 1] | 速度方向一致性阈值 |
| `temporal_smooth_thresh` | 0.5 | [0, 10] | 时序平滑度阈值 |
| `max_curvature` | 10.0 | [0.1, 100] | 曲率计算最大值限制 (1/m) |
| `temporal_window_size` | 10 | [2, 100] | 时序平滑度滑动窗口大小 |
| `weights.kappa` | 1.0 | [0, ∞) | 曲率一致性权重 |
| `weights.velocity` | 1.5 | [0, ∞) | 速度方向一致性权重 |
| `weights.temporal` | 0.8 | [0, ∞) | 时序平滑度权重 |

### 8. tracking - 跟踪质量评估配置

用于 Dashboard 显示，不影响控制器行为。

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `lateral_thresh` | 0.3 | [0.01, 10] | 横向误差阈值 (m) |
| `longitudinal_thresh` | 0.5 | [0.01, 10] | 纵向误差阈值 (m) |
| `heading_thresh` | 0.5 | [0.01, π] | 航向误差阈值 (rad) |

### 9. safety - 安全配置

#### 9.1 停车参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `emergency_decel` | 3.0 | (0, 50] | 紧急减速度 (m/s²) |
| `v_stop_thresh` | 0.05 | [0, 1] | 停车速度阈值 (m/s) |
| `vz_stop_thresh` | 0.1 | [0, 1] | 垂直停车速度阈值 (m/s)，四旋翼使用 |
| `stopping_timeout` | 5.0 | [0.1, 60] | 停车超时 (秒) |

#### 9.2 状态机配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `state_machine.alpha_disable_thresh` | 0.0 | [0, 1] | α 禁用阈值，0 表示禁用 α 检查 |
| `state_machine.mpc_recovery_thresh` | 5 | [1, 100] | MPC 恢复计数阈值 |
| `state_machine.mpc_recovery_tolerance` | 0 | [0, 100] | MPC 恢复容错次数 |
| `state_machine.mpc_fail_window_size` | 10 | [1, 100] | MPC 失败检测滑动窗口大小 |
| `state_machine.mpc_fail_thresh` | 3 | [1, 100] | 窗口内失败次数阈值 |

### 10. backup - 备份控制器配置 (Pure Pursuit)

MPC 失败时的备份控制策略。

#### 10.1 前瞻距离

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `lookahead_dist` | 1.0 | [0.1, 10] | 前瞻距离 (m) |
| `min_lookahead` | 0.5 | [0.01, 5] | 最小前瞻距离 (m) |
| `max_lookahead` | 3.0 | [0.5, 20] | 最大前瞻距离 (m) |
| `lookahead_ratio` | 0.5 | [0, 5] | 前瞻比例，速度越快前瞻越远 |

#### 10.2 控制增益

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `kp_heading` | 1.5 | [0.1, 10] | 航向增益 |
| `kp_z` | 1.0 | [0, 10] | Z 方向增益 (四旋翼使用) |

#### 10.3 控制参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `heading_error_thresh` | 1.047 | [0.1, π] | 航向误差阈值 (rad, ~60°) |
| `max_curvature` | 5.0 | [0.1, 20] | 最大曲率限制 (1/m) |
| `default_speed_ratio` | 0.5 | [0, 1] | 无 soft 速度时的默认速度比例 |
| `min_turn_speed` | 0.1 | [0, 1] | 阿克曼车辆最小转向速度 (m/s) |
| `min_distance_thresh` | 0.1 | [0.001, 1] | 最小距离阈值 (m)，目标点距离小于此值时停止 |

### 11. transform - 坐标变换配置

#### 11.1 算法层参数 (对应 Python DEFAULT_CONFIG)

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `source_frame` | "base_link" | - | 源坐标系 (网络输出轨迹的坐标系) |
| `target_frame` | "odom" | - | 目标坐标系 (控制器工作坐标系) |
| `timeout_ms` | 10 | [1, 1000] | TF2 查询超时 (ms) |

#### 11.2 ROS TF2 参数 (ROS 层特有，Python 中无对应)

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `buffer_warmup_timeout_sec` | 2.0 | [0.1, 60] | TF buffer 预热超时 (秒) |
| `buffer_warmup_interval_sec` | 0.1 | [0.01, 1] | TF buffer 预热检查间隔 (秒) |
| `retry_interval_sec` | 1.0 | [0.1, 60] | TF2 初始重试间隔 (秒) |
| `max_retry_interval_sec` | 30.0 | [1, 300] | TF2 最大重试间隔 (秒) |
| `backoff_multiplier` | 2.0 | [1, 10] | TF2 重试退避倍数 |
| `max_retries` | -1 | - | TF2 最大重试次数 (-1 = 无限) |

### 12. cmd_vel_adapter - cmd_vel 适配器配置

速度/加速度限制统一从 `constraints.*` 读取，不在此处重复定义。

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `publish_rate` | 20.0 | [1, 100] | 命令发布频率 (Hz) |
| `cmd_timeout` | 0.5 | [0.1, 10] | 命令超时 (秒) |
| `enable_rate_limit` | true | - | 启用速度变化率限制 |
| `joy_topic` | "/joy_cmd_vel" | - | 手柄输入话题 |
| `mode_topic` | "/visualizer/control_mode" | - | 模式切换话题 |
| `output_topic` | "/cmd_vel" | - | 输出话题 |

### 13. attitude - 姿态控制配置 (仅四旋翼)

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `mass` | 1.5 | [0.01, 100] | 质量 (kg) |
| `roll_max` | 0.5 | [0.01, π/2] | 最大滚转角 (rad, ~30°) |
| `pitch_max` | 0.5 | [0.01, π/2] | 最大俯仰角 (rad, ~30°) |
| `roll_rate_max` | 3.0 | (0, 10] | 最大滚转角速度 (rad/s) |
| `pitch_rate_max` | 3.0 | (0, 10] | 最大俯仰角速度 (rad/s) |
| `yaw_rate_max` | 2.0 | (0, 10] | 最大偏航角速度 (rad/s) |
| `kp_vx` | 0.5 | [0, 10] | X 方向速度增益 |
| `kp_vy` | 0.5 | [0, 10] | Y 方向速度增益 |
| `kp_vz` | 1.0 | [0, 10] | Z 方向速度增益 |
| `hover_yaw_compensation` | true | - | 悬停 yaw 补偿 |
| `hover_speed_thresh` | 0.1 | [0, 1] | 悬停水平速度阈值 (m/s) |
| `hover_vz_thresh` | 0.05 | [0, 1] | 悬停垂直速度阈值 (m/s) |
| `thrust_min` | 0.1 | [0.01, 1] | 最小推力 |
| `thrust_max` | 2.0 | [1, 10] | 最大推力 |
| `thrust_rate_max` | 2.0 | (0, 10] | 推力变化率限制 (每秒) |

---

## ROS 层配置

以下参数仅在 ROS 层使用，定义在 `base/controller_params.yaml` 中。

### topics - 话题配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topics.odom` | "/controller/input/odom" | 里程计输入话题 |
| `topics.imu` | "" | IMU 输入话题 (空字符串禁用) |
| `topics.trajectory` | "/controller/input/trajectory" | 轨迹输入话题 |
| `topics.emergency_stop` | "/controller/emergency_stop" | 紧急停止话题 |
| `topics.cmd_unified` | "/controller/cmd" | 统一控制命令输出话题 |
| `topics.diagnostics` | "/controller/diagnostics" | 诊断输出话题 |
| `topics.state` | "/controller/state" | 状态输出话题 |
| `topics.attitude_cmd` | "/controller/attitude_cmd" | 姿态命令输出话题 (四旋翼) |
| `topics.debug_path` | "/controller/debug_path" | 调试路径输出话题 |

### clock - 时钟配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `clock.jitter_tolerance` | 0.001 | 时钟抖动容忍度 (秒) |
| `clock.jump_threshold` | 1.0 | 时钟大幅跳变阈值 (秒) |
| `clock.max_events` | 10 | 最多保留的时钟跳变事件数 |

---

## ROS 层特有配置说明

以下配置是 ROS 层特有的，Python 算法库中没有对应的默认值。
这些配置的默认值定义在 `controller_ros/src/controller_ros/utils/param_loader.py` 中。

### 配置定义位置

| 配置块 | 默认值定义位置 | 说明 |
|--------|----------------|------|
| `topics` | `TOPICS_DEFAULTS` | ROS 话题名称 |
| `cmd_vel_adapter` | `CMD_VEL_ADAPTER_DEFAULTS` | cmd_vel 适配器配置 |
| `clock` | `CLOCK_DEFAULTS` | 时钟跳变检测配置 |
| `transform` (ROS TF2 部分) | `TRANSFORM_ROS_DEFAULTS` | TF2 重试策略参数 |

### 为什么这些配置不在 Python 算法库中

1. **topics**: 话题名称是 ROS 通信机制特有的概念，纯算法库不需要
2. **cmd_vel_adapter**: 这是一个独立的 ROS 节点，负责命令转发和模式切换
3. **clock**: 时钟跳变检测是 ROS 仿真环境特有的需求
4. **transform (TF2 部分)**: TF2 buffer 预热、重试策略是 ROS TF2 库特有的
