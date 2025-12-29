# 配置参数指南

**文档版本**: 1.0  
**日期**: 2024-12-29

---

## 概述

本文档说明 `controller_ros` 中各类常量和参数的配置策略，帮助开发者理解哪些参数应该配置化，哪些应该保持为代码常量。

---

## 配置分类

### 1. 协议常量 (Protocol Constants)

**定义**: 由通信协议或数据格式定义的固定值

**是否配置化**: ❌ **不应该**

**示例**:
```python
# trajectory_adapter.py
VELOCITY_DIMENSION = 4  # [vx, vy, vz, wz] 是 ROS 消息格式的固定定义
```

**原因**:
- 改变这些值会导致消息解析错误
- 这是系统间的约定，不是可调参数
- 修改需要同时修改消息定义和所有相关代码

---

### 2. 默认值 (Default Values)

**定义**: 当外部输入缺失或无效时使用的备用值

**是否配置化**: ⚠️ **可选**

**示例**:
```python
# trajectory_adapter.py
DEFAULT_TRAJECTORY_FRAME_ID = 'base_link'  # 当 ROS 消息的 frame_id 为空时使用
DEFAULT_DT_SEC = 0.1                       # 当 dt_sec 无效时使用
MIN_DT_SEC = 0.001                         # dt_sec 的最小有效值
MAX_DT_SEC = 10.0                          # dt_sec 的最大有效值
```

**配置建议**:
- **通常不需要配置**: 这些默认值适用于大多数场景
- **如果需要配置**: 应该在统一的配置文件中管理（如 `tf` 配置）
- **优先级**: 外部输入 > 配置文件 > 代码默认值

---

### 3. 算法参数 (Algorithm Parameters)

**定义**: 影响算法行为的可调参数

**是否配置化**: ✅ **应该考虑**

**示例**:
```python
# trajectory_adapter.py
VELOCITY_DECAY_THRESHOLD = 0.1  # 速度衰减填充阈值 (m/s)
```

**配置策略**:

#### 3.1 核心算法参数 → 必须配置化

**特征**:
- 直接影响控制性能
- 不同机器人需要不同的值
- 用户经常需要调整

**示例**:
```python
# universal_controller/config/default_config.py
'constraints': {
    'v_max': 2.0,        # 最大线速度
    'omega_max': 2.0,    # 最大角速度
    'a_max': 1.5,        # 最大线加速度
}
```

#### 3.2 边缘情况参数 → 可选配置化

**特征**:
- 影响边缘情况的处理
- 默认值适用于大多数场景
- 高级用户可能需要调整

**示例**:
```python
# trajectory_adapter.py
VELOCITY_DECAY_THRESHOLD = 0.1  # 轨迹末端速度填充阈值
```

**当前实现**:
```python
class TrajectoryAdapter(IMsgConverter):
    def __init__(self, config: dict = None):
        # 支持通过 config 传入，但默认使用常量
        self._velocity_decay_threshold = config.get(
            'velocity_decay_threshold', 
            VELOCITY_DECAY_THRESHOLD
        ) if config else VELOCITY_DECAY_THRESHOLD
```

**使用方式**:
```python
# 默认使用（推荐）
adapter = TrajectoryAdapter()

# 自定义阈值（高级用户）
adapter = TrajectoryAdapter(config={'velocity_decay_threshold': 0.15})
```

#### 3.3 内部实现细节 → 不应配置化

**特征**:
- 算法内部的实现细节
- 修改可能导致不可预测的行为
- 用户不应该关心

**示例**:
```python
# 不好的例子：将内部循环次数配置化
MAX_ITERATIONS = config.get('max_iterations', 100)  # ❌ 不推荐
```

---

## 配置层次

### 层次 1: 代码常量

**位置**: 模块顶部的常量定义

**适用于**:
- 协议常量
- 算法内部参数
- 默认值

**示例**:
```python
# trajectory_adapter.py
VELOCITY_DIMENSION = 4
VELOCITY_DECAY_THRESHOLD = 0.1
```

### 层次 2: 类初始化参数

**位置**: 类的 `__init__` 方法

**适用于**:
- 可选的算法参数
- 需要在运行时动态调整的参数

**示例**:
```python
class TrajectoryAdapter:
    def __init__(self, config: dict = None):
        self._velocity_decay_threshold = config.get(
            'velocity_decay_threshold', 
            VELOCITY_DECAY_THRESHOLD
        ) if config else VELOCITY_DECAY_THRESHOLD
```

### 层次 3: 配置文件

**位置**: `universal_controller/config/default_config.py` 或 ROS 参数服务器

**适用于**:
- 核心算法参数
- 机器人特定参数
- 用户经常需要调整的参数

**示例**:
```yaml
# config/turtlebot1.yaml
constraints:
  v_max: 0.5
  omega_max: 1.0
  a_max: 0.3
```

---

## 决策流程图

```
是否应该配置化？
│
├─ 是协议定义的固定值？
│  └─ 是 → ❌ 不配置化（代码常量）
│
├─ 是算法核心参数？
│  └─ 是 → ✅ 必须配置化（配置文件）
│
├─ 是默认值或边缘情况参数？
│  ├─ 用户经常需要调整？
│  │  └─ 是 → ✅ 配置化（配置文件）
│  └─ 否 → ⚠️ 可选配置化（类参数 + 代码常量）
│
└─ 是内部实现细节？
   └─ 是 → ❌ 不配置化（代码常量）
```

---

## 最佳实践

### ✅ 推荐做法

1. **分层注释**: 在常量定义处明确说明是否应该配置化
   ```python
   # ============================================================================
   # 协议常量 (不应修改)
   # ============================================================================
   VELOCITY_DIMENSION = 4
   
   # ============================================================================
   # 算法参数 (内部使用，通常不需要修改)
   # ============================================================================
   VELOCITY_DECAY_THRESHOLD = 0.1
   ```

2. **提供配置接口但保持可选**: 允许高级用户自定义，但默认使用合理值
   ```python
   def __init__(self, config: dict = None):
       self._threshold = config.get('threshold', DEFAULT_THRESHOLD) if config else DEFAULT_THRESHOLD
   ```

3. **文档说明**: 在常量定义处说明如何配置
   ```python
   # 如果需要调整此参数，可以在创建时传入配置：
   #   adapter = TrajectoryAdapter(config={'velocity_decay_threshold': 0.15})
   VELOCITY_DECAY_THRESHOLD = 0.1
   ```

### ❌ 避免做法

1. **过度配置化**: 将所有常量都放入配置文件
   ```python
   # ❌ 不好
   VELOCITY_DIMENSION = config.get('velocity_dimension', 4)  # 这是协议定义，不应配置
   ```

2. **配置层次混乱**: 同一参数在多个地方定义
   ```python
   # ❌ 不好
   # 在 3 个地方都定义了 v_max
   V_MAX = 2.0                           # 代码常量
   config['v_max'] = 1.5                 # 配置文件
   self.v_max = kwargs.get('v_max', 1.0) # 类参数
   ```

3. **缺少默认值**: 强制要求配置
   ```python
   # ❌ 不好
   def __init__(self, config: dict):
       self._threshold = config['threshold']  # 如果缺少会崩溃
   
   # ✅ 好
   def __init__(self, config: dict = None):
       self._threshold = config.get('threshold', DEFAULT_THRESHOLD) if config else DEFAULT_THRESHOLD
   ```

---

## 当前实现状态

### trajectory_adapter.py

| 参数 | 类型 | 配置化 | 说明 |
|------|------|--------|------|
| `VELOCITY_DIMENSION` | 协议常量 | ❌ | ROS 消息格式定义 |
| `DEFAULT_TRAJECTORY_FRAME_ID` | 默认值 | ❌ | 备用坐标系名称 |
| `DEFAULT_DT_SEC` | 默认值 | ❌ | 备用时间间隔 |
| `MIN_DT_SEC` / `MAX_DT_SEC` | 验证参数 | ❌ | 数据有效性边界 |
| `VELOCITY_DECAY_THRESHOLD` | 算法参数 | ⚠️ | 可选配置（类参数） |

### ros_data_source.py

| 参数 | 类型 | 配置化 | 说明 |
|------|------|--------|------|
| `MIN_DATA_STALE_THRESHOLD_MS` | 算法参数 | ❌ | 数据过期判断阈值 |

---

## 总结

**配置化的核心原则**:
1. **协议常量**: 永远不配置化
2. **核心参数**: 必须配置化
3. **边缘参数**: 可选配置化，提供合理默认值
4. **内部细节**: 不配置化

**当前策略**:
- `VELOCITY_DECAY_THRESHOLD` 采用**可选配置化**策略
- 提供类参数接口，但默认使用代码常量
- 适用于大多数场景，高级用户可自定义

**未来改进**:
- 如果用户反馈需要频繁调整 `VELOCITY_DECAY_THRESHOLD`，可以将其添加到配置文件
- 目前保持简单，避免过度配置

