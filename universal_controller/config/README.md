# 配置系统说明

## 设计原则

本配置系统遵循以下原则：

1. **常量与配置分离**
   - 物理/数学/几何常量定义在 `core/constants.py`，不可配置
   - 可调参数定义在 `config/*.py`，可通过 YAML 覆盖

2. **单一数据源**
   - 每个参数只在一个地方定义默认值
   - `low_speed_thresh` 统一定义在 `trajectory_config.py`
   - 其他模块从 trajectory 配置读取，确保一致性

3. **分层配置**
   - Python 配置：算法库默认值，可独立运行
   - YAML 配置：ROS 层覆盖值，面向用户

4. **职责单一**
   - 每个配置文件只负责一个功能模块
   - 文件命名清晰表达其职责

## 架构概览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         配置层次结构                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  core/constants.py - 物理/数学/几何/算法常量 (不可配置)               │   │
│  │  • 物理常量: DEFAULT_GRAVITY                                         │   │
│  │  • 数学边界: CONFIDENCE_MIN/MAX                                      │   │
│  │  • 几何常量: PURE_PURSUIT_ANGLE_THRESH, HEADING_CONTROL_ANGLE_THRESH │   │
│  │  • 算法标准参数: MPC_QP_SOLVER, CONDITION_NUMBER_THRESH              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              ↓                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  config/*.py - Python 默认配置 (算法库内部)                           │   │
│  │  • 每个模块一个文件，职责单一                                          │   │
│  │  • 包含: 默认值 + 验证规则                                            │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                              ↓                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  controller_ros/config/ - YAML 配置 (ROS 层)                         │   │
│  │  • base/controller_params.yaml - ROS 话题、时钟                       │   │
│  │  • base/internal_params.yaml - 高级参数覆盖                           │   │
│  │  • platforms/*.yaml - 平台配置 (用户入口)                             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 配置文件职责

| 文件 | 职责 | 用户是否应修改 |
|------|------|----------------|
| `controller_params.yaml` | ROS 话题默认值、时钟配置 | 通常不需要 |
| `internal_params.yaml` | 内部实现参数 (EKF、滤波器、状态机等) | **不应修改** |
| `platforms/*.yaml` | 平台特定的可调参数 | **主要配置入口** |
| `_template.yaml` | 创建新平台配置的模板 | 复制后修改 |

**设计说明**：
- **用户只需面对 `controller_ros` 包**，所有配置都在 YAML 中统一管理
- **`universal_controller` 不对用户开放**，是纯算法库的内部实现
- Python 默认值让算法库可以**独立运行**（不依赖 ROS，如单元测试）
- YAML 配置是 ROS 部署时的**完整配置入口**，必须显式列出所有用户可调参数
- 内部实现参数独立存放在 `internal_params.yaml`，用户不应修改

### 什么放在代码中，什么放在配置文件中

**代码中固定（物理规则，不需要配置）**：
- 物理常量：如重力加速度 `DEFAULT_GRAVITY = 9.81`
- 数值稳定性常量：如 `EPSILON = 1e-6`
- 置信度边界：`CONFIDENCE_MIN = 0.0`, `CONFIDENCE_MAX = 1.0`
- 协方差矩阵常量：`COVARIANCE_MIN_EIGENVALUE`, `COVARIANCE_INITIAL_VALUE`
- 加速度计算常量：`MIN_DT_FOR_ACCEL`, `MAX_DT_FOR_ACCEL`
- 过渡动画常量：`TRANSITION_COMPLETION_THRESHOLD`
- Pure Pursuit 几何常量：`PURE_PURSUIT_ANGLE_THRESH`, `HEADING_CONTROL_ANGLE_THRESH`, `REAR_ANGLE_THRESH`
- MPC 求解器常量：`MPC_QP_SOLVER`, `MPC_INTEGRATOR_TYPE`, `MPC_NLP_SOLVER_TYPE`, `MPC_NLP_MAX_ITER`
- 数值稳定性阈值：`CONDITION_NUMBER_THRESH`, `KKT_RESIDUAL_THRESH`

**平台配置（platform_config.py）**：
- `type`: 平台类型枚举
- `velocity_heading_coupled`: 速度方向是否与航向耦合
- `is_ground_vehicle`: 是否为地面车辆
- `can_rotate_in_place`: 是否可原地旋转
- `output_type`: 输出类型 (differential/omni/3d)
- `output_frame`: 输出坐标系 (base_link/world)

**YAML 配置文件中（用户可调参数，必须显式列出）**：
- 控制参数：MPC 权重、预测时域、控制增益
- 约束参数：速度/加速度限制
- 超时参数：watchdog 配置
- 安全参数：紧急减速度、停车阈值
- 平台特有参数：坐标系名称、话题名称

**internal_params.yaml 中（内部实现参数，用户不应修改）**：
- EKF 噪声、滤波器系数、状态机细节等
- 这些参数经过精心调优，修改需要专业知识

### 内部实现参数 (在 internal_params.yaml 中)

以下参数是内部实现细节，定义在 `controller_ros/config/base/internal_params.yaml` 中：

| 模块 | 参数 | 理由 |
|------|------|------|
| `ekf.*` | EKF 状态估计器 | 参数高度耦合，需要专业知识 |
| `transition.*` | 状态过渡 | 内部控制逻辑 |
| `mpc.solver.*` | ACADOS 求解器 | 需要优化器专业知识 |
| `safety.accel_filter_*` | 加速度滤波 | 内部安全机制 |
| `backup.heading_mode` | 航向控制模式 | 内部控制逻辑 |
| `transform.drift_*` | 漂移估计 | 内部补偿机制 |

## 配置设计原则

1. **用户配置入口唯一**：用户只需关注 `controller_ros/config/platforms/*.yaml`
2. **物理常量在代码中定义**：如重力加速度 `DEFAULT_GRAVITY`，定义在 `core/constants.py`
3. **平台运动学在代码中定义**：如活跃状态维度、控制维度，定义在 `platform_config.py`
4. **所有用户可调参数在 YAML 中显式列出**：平台配置文件包含所有需要调优的参数，分类清晰
5. **Python 默认值作为安全网**：当 YAML 缺失某项时使用默认值，防止系统崩溃
6. **文件职责单一**：每个配置文件只负责一个功能模块的配置

## 配置优先级

从低到高：
1. `universal_controller/config/*.py` - 默认配置（算法库内部，安全网）
2. YAML 配置文件 - 平台特定配置（面向用户的完整配置入口）
3. ROS 参数服务器 - 运行时覆盖

## 配置文件结构

```
universal_controller/config/
├── __init__.py             # 配置模块入口，导出主要接口
├── default_config.py       # 配置聚合，合并所有子模块
├── platform_config.py      # 平台运动学配置（物理模型，不可配置）
├── system_config.py        # 系统基础配置 (ctrl_freq, watchdog, diagnostics, tracking)
├── mpc_config.py           # MPC 控制器配置
├── safety_config.py        # 安全和约束配置
├── ekf_config.py           # EKF 状态估计器配置
├── attitude_config.py      # 姿态控制配置 (无人机)
├── trajectory_config.py    # 轨迹配置
├── consistency_config.py   # 一致性检查配置
├── transform_config.py     # 坐标变换配置
├── transition_config.py    # 平滑过渡配置
├── backup_config.py        # 备份控制器配置
├── mock_config.py          # Mock 配置 (Dashboard 调试用)
├── modules_config.py       # 兼容性导入 (已拆分到独立文件)
└── validation.py           # 配置验证逻辑

controller_ros/config/
├── base/
│   ├── controller_params.yaml  # ROS 层基础配置 (话题、时钟)
│   └── internal_params.yaml    # 内部实现参数 (用户不应修改)
├── platforms/
│   ├── _template.yaml          # 用户配置模板
│   ├── differential.yaml       # 差速车配置
│   ├── omni.yaml               # 全向车配置
│   ├── ackermann.yaml          # 阿克曼车配置
│   ├── quadrotor.yaml          # 四旋翼配置
│   └── turtlebot1.yaml         # TurtleBot1 专用配置
└── tools/
    └── visualizer_params.yaml  # 可视化工具配置
```

## 配置分层设计

### 配置分类

| 配置块 | 用途 | 定义位置 |
|--------|------|----------|
| `system` | 系统基础配置 | Python + YAML |
| `constraints` | 运动约束 | Python + YAML |
| `mpc` | MPC 控制器配置 | Python + YAML |
| `safety` | 安全配置 | Python + YAML |
| `backup` | 备份控制器配置 | Python + YAML |
| `trajectory` | 轨迹配置 | Python + YAML |
| `consistency` | 一致性检查配置 | Python + YAML |
| `tracking` | 跟踪质量评估配置 | Python + YAML |
| `watchdog` | 超时配置 | Python + YAML |
| `diagnostics` | 诊断配置 | Python + YAML |
| `transform` | 坐标变换配置 | Python (算法层) + YAML (算法层 + ROS TF2) |
| `ekf` | EKF 状态估计器 | Python + YAML (高级) |
| `transition` | 状态过渡配置 | Python + YAML (高级) |
| `attitude` | 姿态控制配置 | Python + YAML (仅四旋翼) |
| `mock` | 模拟数据配置 | Python only (开发调试) |
| `topics` | ROS 话题配置 | YAML only (ROS 层特有) |
| `cmd_vel_adapter` | cmd_vel 适配器配置 | YAML only (ROS 层特有) |
| `clock` | 时钟配置 | YAML only (ROS 层特有) |

### transform 配置说明

`transform` 配置是混合配置，包含算法层参数和 ROS TF2 参数：

```yaml
transform:
  # 算法层参数 (对应 Python DEFAULT_CONFIG['transform'])
  source_frame: "base_link"       # 源坐标系
  target_frame: "odom"            # 目标坐标系
  timeout_ms: 10                  # TF 查询超时
  
  # ROS TF2 参数 (ROS 层特有，定义在 param_loader.py)
  buffer_warmup_timeout_sec: 2.0  # TF buffer 预热超时
  buffer_warmup_interval_sec: 0.1 # TF buffer 预热检查间隔
  retry_interval_sec: 1.0         # TF2 重试间隔
  max_retries: -1                 # 最大重试次数
```

### 配置访问方式

推荐使用 `get_config_value` 函数：

```python
from universal_controller.config import get_config_value, DEFAULT_CONFIG

# 获取配置值，支持点分隔路径
horizon = get_config_value(config, 'mpc.horizon', default=20)

# 使用 fallback_config 获取默认值
value = get_config_value(config, 'mpc.horizon', fallback_config=DEFAULT_CONFIG)
```

## 配置验证

### 范围验证

每个配置模块定义自己的验证规则：

```python
MPC_VALIDATION_RULES = {
    'mpc.horizon': (1, 100, 'MPC 预测时域'),
    'mpc.dt': (0.001, 1.0, 'MPC 时间步长 (秒)'),
}
```

### 逻辑一致性验证

`validate_logical_consistency()` 检查配置参数之间的逻辑关系：
- `min_lookahead < max_lookahead`
- `horizon_degraded <= horizon`
- `v_min <= v_max`

### 使用验证

```python
from universal_controller.config import validate_config, ConfigValidationError

try:
    errors = validate_config(config, raise_on_error=True)
except ConfigValidationError as e:
    print(f"配置错误: {e}")
```

## 添加新配置项

1. 在对应的 `*_config.py` 中添加默认值（算法库内部）
2. 在同一文件的 `*_VALIDATION_RULES` 中添加验证规则
3. **在 `controller_ros/config/platforms/_template.yaml` 中添加对应配置**（面向用户）
4. 确保 Python 和 YAML 中的键名一致
5. 更新相关文档

## 特殊配置说明

### low_speed_thresh

低速阈值的唯一定义点是 `trajectory.low_speed_thresh`。
`consistency` 模块会自动从 `trajectory` 配置读取此值，确保一致性。

### traj_grace_ms

轨迹宽限期是超时**之后**的额外等待时间：
- 安全停止延迟 = `traj_timeout_ms` + `traj_grace_ms`
- 例如: 1000ms 超时 + 500ms 宽限 = 1.5秒后触发停止

### expected_source_frames

预期的轨迹源坐标系列表，用于验证网络输出：
- `base_link`: 标准机体坐标系
- `base_footprint`: TurtleBot 等平台使用
- `base_link_0`: 推理时刻冻结的机体坐标系
- `''`: 空字符串，使用默认 source_frame
- `odom`: 已经在里程计坐标系的轨迹
