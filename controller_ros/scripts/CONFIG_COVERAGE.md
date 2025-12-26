# 诊断脚本配置覆盖对照表

本文档列出了 `diagnose_and_tune.sh` 和 `full_diagnostics.py` 脚本覆盖的所有配置项。

## 诊断模式

| 模式 | 需要启动 | 诊断内容 |
|-----|---------|---------|
| **基础模式** | turtlebot_bringup + 网络轨迹 | 话题频率、轨迹特性、底盘估算 |
| **底盘测试** (`--test-chassis`) | turtlebot_bringup | 实际最大速度、加速度、响应时间 |
| **运行时调优** (`--runtime-tuning`) | turtlebot_bringup + 网络 + 控制器 | MPC 求解时间、跟踪误差、调优建议 |

## 使用方法

```bash
# 1. 基础诊断（生成初始配置）
rosrun controller_ros full_diagnostics.py --output initial.yaml

# 2. 底盘测试（更精确的速度/加速度限制）
rosrun controller_ros full_diagnostics.py --test-chassis --output tuned.yaml

# 3. 运行时调优（需要控制器运行，用于调整 MPC 权重）
rosrun controller_ros full_diagnostics.py --runtime-tuning --duration 30
```

## 配置覆盖状态

| 配置模块 | 配置项 | Bash 脚本 | Python 脚本 | 诊断依据 |
|---------|--------|-----------|-------------|----------|
| **system** | ctrl_freq | ✅ | ✅ | 里程计频率 |
| | platform | ✅ | ✅ | 固定值 |
| **topics** | odom | ✅ | ✅ | 命令行参数 |
| | imu | ✅ | ✅ | 命令行参数 |
| | trajectory | ✅ | ✅ | 命令行参数 |
| | cmd_vel | ✅ | ✅ | 命令行参数 |
| **tf** | source_frame | ✅ | ✅ | 轨迹 frame_id |
| | target_frame | ✅ | ✅ | 固定值 |
| | timeout_ms | ✅ | ✅ | 里程计延迟 |
| | buffer_warmup_* | ✅ | ✅ | 固定值 |
| **watchdog** | odom_timeout_ms | ✅ | ✅ | 里程计频率 |
| | traj_timeout_ms | ✅ | ✅ | 轨迹频率 |
| | traj_grace_ms | ✅ | ✅ | 轨迹频率 |
| | imu_timeout_ms | ✅ | ✅ | IMU 频率 |
| | startup_grace_ms | ✅ | ✅ | 固定值 |
| **mpc** | horizon | ✅ | ✅ | 轨迹点数 |
| | horizon_degraded | ✅ | ✅ | horizon/2 |
| | dt | ✅ | ✅ | 轨迹 dt_sec |
| | weights.* | ✅ | ✅ | 默认值 (需运行时调优) |
| | solver.* | ✅ | ✅ | 固定值 |
| | health_monitor.* | ✅ | ✅ | 控制频率 |
| | fallback.* | ✅ | ✅ | 固定值 |
| **constraints** | v_max | ✅ | ✅ | 底盘最大速度 × 80% |
| | v_min | ✅ | ✅ | 固定值 |
| | omega_max | ✅ | ✅ | 底盘最大角速度 × 80% |
| | omega_max_low | ✅ | ✅ | omega_max × 50% |
| | a_max | ✅ | ✅ | 底盘最大加速度 × 80% |
| | alpha_max | ✅ | ✅ | 底盘最大角加速度 × 80% |
| | v_low_thresh | ✅ | ✅ | 固定值 |
| **consistency** | kappa_thresh | ✅ | ✅ | 固定值 |
| | v_dir_thresh | ✅ | ✅ | 固定值 |
| | temporal_smooth_thresh | ✅ | ✅ | 轨迹抖动 |
| | alpha_min | ✅ | ✅ | 固定值 |
| | max_curvature | ✅ | ✅ | 固定值 |
| | temporal_window_size | ✅ | ✅ | 固定值 |
| | weights.* | ✅ | ✅ | 固定值 |
| **safety** | v_stop_thresh | ✅ | ✅ | 固定值 |
| | stopping_timeout | ✅ | ✅ | 固定值 |
| | emergency_decel | ✅ | ✅ | a_max × 1.5 |
| | low_speed.* | ✅ | ✅ | omega_max × 50% |
| | margins.* | ✅ | ✅ | 固定值 |
| | accel_filter_* | ✅ | ✅ | 固定值 |
| | state_machine.alpha_* | ✅ | ✅ | 轨迹是否有速度 |
| | state_machine.mpc_* | ✅ | ✅ | 固定值 |
| **ekf** | use_odom_orientation_fallback | ✅ | ✅ | IMU 可用性 |
| | imu_motion_compensation | ✅ | ✅ | IMU 可用性 |
| | process_noise.* | ✅ | ✅ | 里程计抖动 |
| | measurement_noise.* | ✅ | ✅ | 里程计抖动 |
| | adaptive.* | ✅ | ✅ | 固定值 |
| | anomaly_detection.* | ✅ | ✅ | 固定值 |
| **transform** | timeout_ms | ✅ | ✅ | 里程计延迟 |
| | fallback_*_limit_ms | ✅ | ✅ | 固定值 |
| | drift_* | ✅ | ✅ | 固定值 |
| **transition** | type | ✅ | ✅ | 固定值 |
| | tau | ✅ | ✅ | 控制周期 × 2 |
| | max_duration | ✅ | ✅ | 固定值 |
| | completion_threshold | ✅ | ✅ | 固定值 |
| | duration | ✅ | ✅ | 固定值 |
| **backup** | lookahead_dist | ✅ | ✅ | v_max × 响应时间 |
| | min_lookahead | ✅ | ✅ | lookahead × 0.6 |
| | max_lookahead | ✅ | ✅ | lookahead × 3 |
| | kp_heading | ✅ | ✅ | 固定值 |
| | heading_error_thresh | ✅ | ✅ | 固定值 |
| | pure_pursuit_angle_thresh | ✅ | ✅ | 固定值 |
| | max_curvature | ✅ | ✅ | 固定值 |
| | min_turn_speed | ✅ | ✅ | 固定值 |
| | default_speed_ratio | ✅ | ✅ | 固定值 |
| **tracking** | lateral_thresh | ✅ | ✅ | 轨迹长度 |
| | longitudinal_thresh | ✅ | ✅ | 轨迹长度 |
| | heading_thresh | ✅ | ✅ | 固定值 |
| | weights.* | ✅ | ✅ | 固定值 |
| | rating.* | ✅ | ✅ | 固定值 |
| **cmd_vel_adapter** | publish_rate | ✅ | ✅ | 控制频率 |
| | joy_timeout | ✅ | ✅ | 固定值 |
| | max_linear | ✅ | ✅ | v_max |
| | max_angular | ✅ | ✅ | omega_max |
| | max_*_accel | ✅ | ✅ | 固定值 |
| | output_topic | ✅ | ✅ | 命令行参数 |

## 诊断数据来源

| 数据 | 获取方式 | 用途 |
|------|---------|------|
| 里程计频率 | `rostopic hz` | 控制频率、超时 |
| 里程计抖动 | `rostopic hz` std dev | EKF 噪声 |
| 里程计延迟 | `rostopic delay` | TF 超时 |
| 轨迹频率 | `rostopic hz` | 超时配置 |
| 轨迹点数 | `rostopic echo` | MPC horizon |
| 轨迹 dt_sec | `rostopic echo` | MPC dt |
| 轨迹速度信息 | `rostopic echo` | alpha 检查 |
| IMU 频率 | `rostopic hz` | IMU 超时 |
| 底盘最大速度 | 里程计分析 / 底盘测试 | 速度约束 |
| 底盘最大角速度 | 里程计分析 / 底盘测试 | 角速度约束 |
| 底盘最大加速度 | 底盘测试 (Python) | 加速度约束 |
| 底盘响应时间 | 底盘测试 (Python) | 前瞻距离 |

## 需要运行时调优的参数

以下参数需要根据实际运行效果进行调优：

1. **MPC 权重** (`mpc.weights.*`)
   - 跟踪误差大 → 增加 `position` 权重
   - 控制抖动 → 增加 `control_accel` 和 `control_alpha`
   - 响应慢 → 减小 `control_accel`

2. **一致性权重** (`consistency.weights.*`)
   - 根据轨迹质量调整各项权重

3. **跟踪阈值** (`tracking.*_thresh`)
   - 根据实际跟踪精度要求调整

## 使用建议

1. **首次部署**: 
   - 启动 `turtlebot_bringup` + 网络轨迹
   - 运行 `full_diagnostics.py --output initial.yaml` 生成基础配置

2. **精确调优**: 
   - 启动 `turtlebot_bringup`
   - 运行 `full_diagnostics.py --test-chassis --output tuned.yaml` 进行底盘测试

3. **运行时调优** (MPC 权重调整):
   - 启动 `turtlebot_bringup` + 网络轨迹 + `controller_ros`
   - 让机器人跟踪轨迹运动
   - 运行 `full_diagnostics.py --runtime-tuning --duration 30`
   - 根据输出的调优建议修改 MPC 权重

4. **问题排查**: 使用 `--verbose` 选项查看详细诊断信息

## 运行时调优输出示例

```
Controller Runtime Statistics:
  MPC solve time: 8.5ms avg, 15.2ms max
  MPC success rate: 98.5%
  Backup active ratio: 1.2%
  Lateral error: 3.2cm avg, 8.5cm max
  Heading error: 2.3° avg
  Alpha (consistency): 0.85 avg, 0.42 min

Runtime Tuning Suggestions:
  [OK] MPC solve time is good (8.5ms < 16.7ms)
  [OK] MPC success rate is good (98%)
  [WARN] High lateral tracking error (3.2cm)
    → Increase mpc.weights.position (try 15-20)
    → Decrease mpc.weights.control_accel (try 0.1)
```
