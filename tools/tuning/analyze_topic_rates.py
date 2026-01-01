#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
话题帧率分析与参数调优工具 v1.0

分析 ROS 话题的实际帧率，并基于帧率生成参数调优建议。

需要监控的话题 (基于 turtlebot1.yaml 和 controller_params.yaml):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
输入话题:
  - /odom                              → watchdog.odom_timeout_ms
  - /controller/input/trajectory       → watchdog.traj_timeout_ms, traj_grace_ms
  - /mobile_base/sensors/imu_data      → watchdog.imu_timeout_ms
  - TF2 (base_footprint → odom)        → transform.timeout_ms

输出话题:
  - /controller/diagnostics            → diagnostics.publish_rate
  - /cmd_unified                       → cmd_vel_adapter.publish_rate
  - /controller/state                  → 状态监控

使用方法:
  # 实时分析 (需要 ROS 环境)
  python -m tools.tuning.analyze_topic_rates --live --duration 30
  
  # 从 ROS bag 分析
  python -m tools.tuning.analyze_topic_rates --bag /path/to/recording.bag
  
  # 生成调优建议
  python -m tools.tuning.analyze_topic_rates --live --duration 30 --suggest
"""

import argparse
import time
import json
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from collections import defaultdict
import numpy as np


# 话题配置 - 基于 turtlebot1.yaml 和 controller_params.yaml
TOPIC_CONFIG = {
    # 输入话题
    'odom': {
        'topic': '/odom',
        'param': 'watchdog.odom_timeout_ms',
        'type': 'input',
        'expected_hz': 50,  # TurtleBot 通常 50Hz
        'timeout_margin': 2.0,  # 超时应该是周期的 2 倍
    },
    'trajectory': {
        'topic': '/controller/input/trajectory',
        'param': 'watchdog.traj_timeout_ms',
        'type': 'input',
        'expected_hz': 10,  # 轨迹发布器建议 >= 10Hz
        'timeout_margin': 2.0,
        'grace_param': 'watchdog.traj_grace_ms',
        'grace_margin': 1.5,
    },
    'imu': {
        'topic': '/mobile_base/sensors/imu_data',
        'param': 'watchdog.imu_timeout_ms',
        'type': 'input',
        'expected_hz': 100,  # IMU 通常 100Hz
        'timeout_margin': 2.0,
    },
    # 输出话题
    'diagnostics': {
        'topic': '/controller/diagnostics',
        'param': 'diagnostics.publish_rate',
        'type': 'output',
        'expected_hz': 4,  # ctrl_freq(20) / publish_rate(5) = 4Hz
    },
    'cmd_unified': {
        'topic': '/cmd_unified',
        'param': 'cmd_vel_adapter.publish_rate',
        'type': 'output',
        'expected_hz': 20,  # 与 publish_rate 一致
    },
    'state': {
        'topic': '/controller/state',
        'param': None,
        'type': 'output',
        'expected_hz': 4,  # 与 diagnostics 相同
    },
}

# TF2 配置
TF_CONFIG = {
    'source_frame': 'base_footprint',
    'target_frame': 'odom',
    'param': 'transform.timeout_ms',
    'expected_hz': 50,  # TF 通常与 odom 同频
    'timeout_margin': 2.0,
}


@dataclass
class TopicStats:
    """话题统计"""
    topic_name: str
    message_count: int = 0
    timestamps: List[float] = field(default_factory=list)
    
    @property
    def duration(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        return self.timestamps[-1] - self.timestamps[0]
    
    @property
    def avg_hz(self) -> float:
        if self.duration <= 0 or self.message_count < 2:
            return 0.0
        return (self.message_count - 1) / self.duration
    
    @property
    def intervals(self) -> List[float]:
        if len(self.timestamps) < 2:
            return []
        return [self.timestamps[i+1] - self.timestamps[i] 
                for i in range(len(self.timestamps)-1)]
    
    @property
    def min_hz(self) -> float:
        intervals = self.intervals
        if not intervals:
            return 0.0
        max_interval = max(intervals)
        return 1.0 / max_interval if max_interval > 0 else 0.0
    
    @property
    def max_hz(self) -> float:
        intervals = self.intervals
        if not intervals:
            return 0.0
        min_interval = min(intervals)
        return 1.0 / min_interval if min_interval > 0 else float('inf')
    
    @property
    def std_hz(self) -> float:
        intervals = self.intervals
        if len(intervals) < 2:
            return 0.0
        hz_values = [1.0/i for i in intervals if i > 0]
        return float(np.std(hz_values)) if hz_values else 0.0
    
    @property
    def p95_interval_ms(self) -> float:
        intervals = self.intervals
        if not intervals:
            return 0.0
        return float(np.percentile(intervals, 95)) * 1000


@dataclass
class TuningResult:
    """调优建议"""
    param: str
    current_value: Any
    suggested_value: Any
    reason: str
    severity: str  # 'critical', 'warning', 'info'


class TopicRateAnalyzer:
    """话题帧率分析器"""
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}
        self.topic_stats: Dict[str, TopicStats] = {}
        self.tf_stats: Optional[TopicStats] = None
        self.tuning_results: List[TuningResult] = []
    
    def add_topic_sample(self, topic_name: str, timestamp: float):
        """添加话题样本"""
        if topic_name not in self.topic_stats:
            self.topic_stats[topic_name] = TopicStats(topic_name=topic_name)
        
        stats = self.topic_stats[topic_name]
        stats.message_count += 1
        stats.timestamps.append(timestamp)
    
    def add_tf_sample(self, timestamp: float):
        """添加 TF 样本"""
        if self.tf_stats is None:
            self.tf_stats = TopicStats(topic_name="TF2")
        self.tf_stats.message_count += 1
        self.tf_stats.timestamps.append(timestamp)

    
    def analyze(self) -> List[TuningResult]:
        """分析并生成调优建议"""
        self.tuning_results = []
        
        # 分析各话题
        for key, topic_cfg in TOPIC_CONFIG.items():
            topic_name = topic_cfg['topic']
            if topic_name in self.topic_stats:
                self._analyze_topic(key, topic_cfg, self.topic_stats[topic_name])
            else:
                # 话题未收到消息
                if topic_cfg['type'] == 'input':
                    self.tuning_results.append(TuningResult(
                        param=topic_cfg.get('param', topic_name),
                        current_value='N/A',
                        suggested_value='检查话题',
                        reason=f"未收到 {topic_name} 消息，请检查话题是否正确发布",
                        severity='critical'
                    ))
        
        # 分析 TF
        if self.tf_stats:
            self._analyze_tf()
        
        return self.tuning_results
    
    def _analyze_topic(self, key: str, cfg: Dict, stats: TopicStats):
        """分析单个话题"""
        actual_hz = stats.avg_hz
        expected_hz = cfg.get('expected_hz', 10)
        param = cfg.get('param')
        
        if actual_hz <= 0:
            return
        
        # 计算建议的超时值
        actual_period_ms = 1000.0 / actual_hz
        p95_interval_ms = stats.p95_interval_ms
        
        if cfg['type'] == 'input' and param:
            # 获取当前配置值
            current_timeout = self._get_config_value(param, 500)
            margin = cfg.get('timeout_margin', 2.0)
            
            # 建议超时 = max(p95间隔, 平均周期) * margin
            suggested_timeout = int(max(p95_interval_ms, actual_period_ms) * margin)
            
            # 检查是否需要调整
            if current_timeout < suggested_timeout * 0.8:
                self.tuning_results.append(TuningResult(
                    param=param,
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"话题 {cfg['topic']} 实际帧率 {actual_hz:.1f}Hz (p95间隔 {p95_interval_ms:.0f}ms)，当前超时 {current_timeout}ms 可能过短",
                    severity='warning'
                ))
            
            # 检查 grace 参数
            grace_param = cfg.get('grace_param')
            if grace_param:
                current_grace = self._get_config_value(grace_param, 600)
                grace_margin = cfg.get('grace_margin', 1.5)
                suggested_grace = int(max(p95_interval_ms, actual_period_ms) * grace_margin)
                
                if current_grace < suggested_grace * 0.8:
                    self.tuning_results.append(TuningResult(
                        param=grace_param,
                        current_value=current_grace,
                        suggested_value=suggested_grace,
                        reason=f"话题 {cfg['topic']} 实际帧率 {actual_hz:.1f}Hz，当前宽限期 {current_grace}ms 可能过短",
                        severity='warning'
                    ))
        
        # 检查帧率是否过低
        if actual_hz < expected_hz * 0.5:
            self.tuning_results.append(TuningResult(
                param=cfg['topic'],
                current_value=f"{actual_hz:.1f} Hz",
                suggested_value=f">= {expected_hz} Hz",
                reason=f"话题 {cfg['topic']} 帧率过低 ({actual_hz:.1f}Hz < 期望 {expected_hz}Hz)，可能导致控制性能下降",
                severity='critical' if cfg['type'] == 'input' else 'warning'
            ))

    
    def _analyze_tf(self):
        """分析 TF2"""
        if not self.tf_stats or self.tf_stats.avg_hz <= 0:
            return
        
        actual_hz = self.tf_stats.avg_hz
        p95_interval_ms = self.tf_stats.p95_interval_ms
        actual_period_ms = 1000.0 / actual_hz
        
        current_timeout = self._get_config_value(TF_CONFIG['param'], 50)
        margin = TF_CONFIG.get('timeout_margin', 2.0)
        suggested_timeout = int(max(p95_interval_ms, actual_period_ms) * margin)
        
        if current_timeout < suggested_timeout * 0.8:
            self.tuning_results.append(TuningResult(
                param=TF_CONFIG['param'],
                current_value=current_timeout,
                suggested_value=suggested_timeout,
                reason=f"TF2 ({TF_CONFIG['source_frame']} → {TF_CONFIG['target_frame']}) 实际帧率 {actual_hz:.1f}Hz，当前超时 {current_timeout}ms 可能过短",
                severity='warning'
            ))
    
    def _get_config_value(self, param_path: str, default: Any) -> Any:
        """从配置中获取参数值"""
        parts = param_path.split('.')
        value = self.config
        for part in parts:
            if isinstance(value, dict) and part in value:
                value = value[part]
            else:
                return default
        return value
    
    def print_report(self):
        """打印分析报告"""
        print("=" * 70)
        print("话题帧率分析报告")
        print("=" * 70)
        
        # 输入话题
        print("\n【输入话题帧率】")
        for key, cfg in TOPIC_CONFIG.items():
            if cfg['type'] != 'input':
                continue
            topic = cfg['topic']
            if topic in self.topic_stats:
                stats = self.topic_stats[topic]
                status = "✓" if stats.avg_hz >= cfg.get('expected_hz', 10) * 0.8 else "⚠"
                print(f"  {status} {topic}")
                print(f"      帧率: {stats.avg_hz:.1f} Hz (期望 >= {cfg.get('expected_hz', 10)} Hz)")
                print(f"      消息数: {stats.message_count}")
                print(f"      p95间隔: {stats.p95_interval_ms:.1f} ms")
            else:
                print(f"  ✗ {topic} - 未收到消息")
        
        # TF2
        print("\n【TF2 帧率】")
        if self.tf_stats and self.tf_stats.avg_hz > 0:
            print(f"  {TF_CONFIG['source_frame']} → {TF_CONFIG['target_frame']}")
            print(f"      帧率: {self.tf_stats.avg_hz:.1f} Hz")
            print(f"      p95间隔: {self.tf_stats.p95_interval_ms:.1f} ms")
        else:
            print("  未收到 TF 数据")
        
        # 输出话题
        print("\n【输出话题帧率】")
        for key, cfg in TOPIC_CONFIG.items():
            if cfg['type'] != 'output':
                continue
            topic = cfg['topic']
            if topic in self.topic_stats:
                stats = self.topic_stats[topic]
                print(f"  {topic}: {stats.avg_hz:.1f} Hz")
            else:
                print(f"  {topic}: 未收到消息")
        
        # 调优建议
        if self.tuning_results:
            print("\n" + "=" * 70)
            print("调优建议")
            print("=" * 70)
            
            critical = [r for r in self.tuning_results if r.severity == 'critical']
            warnings = [r for r in self.tuning_results if r.severity == 'warning']
            
            if critical:
                print("\n🔴 严重问题:")
                for r in critical:
                    print(f"  [{r.param}]")
                    print(f"    当前: {r.current_value} → 建议: {r.suggested_value}")
                    print(f"    原因: {r.reason}")
            
            if warnings:
                print("\n🟡 警告:")
                for r in warnings:
                    print(f"  [{r.param}]")
                    print(f"    当前: {r.current_value} → 建议: {r.suggested_value}")
                    print(f"    原因: {r.reason}")
        else:
            print("\n✅ 未发现需要调优的问题")
        
        print()


def collect_live(duration_sec: float, config: Dict[str, Any]) -> TopicRateAnalyzer:
    """实时收集话题数据"""
    try:
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Empty
        import tf2_ros
    except ImportError:
        print("错误: 需要 ROS 环境")
        print("请确保已 source ROS 工作空间")
        return None
    
    # 尝试导入自定义消息
    try:
        from controller_ros.msg import Trajectory, DiagnosticsV2, UnifiedCmd
        has_custom_msgs = True
    except ImportError:
        print("警告: controller_ros 消息不可用，部分话题将无法监控")
        has_custom_msgs = False
    
    analyzer = TopicRateAnalyzer(config)
    
    # 初始化 ROS 节点
    try:
        rospy.init_node('topic_rate_analyzer', anonymous=True)
    except rospy.exceptions.ROSException:
        pass
    
    # 订阅回调
    def odom_cb(msg):
        analyzer.add_topic_sample('/odom', rospy.Time.now().to_sec())
    
    def imu_cb(msg):
        analyzer.add_topic_sample('/mobile_base/sensors/imu_data', rospy.Time.now().to_sec())
    
    def traj_cb(msg):
        analyzer.add_topic_sample('/controller/input/trajectory', rospy.Time.now().to_sec())
    
    def diag_cb(msg):
        analyzer.add_topic_sample('/controller/diagnostics', rospy.Time.now().to_sec())
    
    def cmd_cb(msg):
        analyzer.add_topic_sample('/cmd_unified', rospy.Time.now().to_sec())
    
    def state_cb(msg):
        analyzer.add_topic_sample('/controller/state', rospy.Time.now().to_sec())
    
    # 创建订阅者
    subs = []
    subs.append(rospy.Subscriber('/odom', Odometry, odom_cb))
    subs.append(rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, imu_cb))
    
    if has_custom_msgs:
        subs.append(rospy.Subscriber('/controller/input/trajectory', Trajectory, traj_cb))
        subs.append(rospy.Subscriber('/controller/diagnostics', DiagnosticsV2, diag_cb))
        subs.append(rospy.Subscriber('/cmd_unified', UnifiedCmd, cmd_cb))
    
    # TF2 监听
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    print(f"开始收集数据 ({duration_sec}秒)...")
    print("监控话题:")
    for key, cfg in TOPIC_CONFIG.items():
        print(f"  - {cfg['topic']}")
    print(f"  - TF2: {TF_CONFIG['source_frame']} → {TF_CONFIG['target_frame']}")
    print("-" * 40)
    
    start_time = time.time()
    rate = rospy.Rate(50)  # 50Hz 检查频率
    
    while not rospy.is_shutdown():
        elapsed = time.time() - start_time
        if elapsed >= duration_sec:
            break
        
        # 尝试获取 TF
        try:
            tf_buffer.lookup_transform(
                TF_CONFIG['target_frame'],
                TF_CONFIG['source_frame'],
                rospy.Time(0),
                rospy.Duration(0.01)
            )
            analyzer.add_tf_sample(rospy.Time.now().to_sec())
        except:
            pass
        
        # 打印进度
        if int(elapsed) % 5 == 0 and elapsed > 0:
            print(f"  已收集 {int(elapsed)}秒...")
        
        rate.sleep()
    
    # 清理
    for sub in subs:
        sub.unregister()
    
    return analyzer


def collect_from_bag(bag_path: str, config: Dict[str, Any]) -> TopicRateAnalyzer:
    """从 ROS bag 收集数据"""
    try:
        import rosbag
    except ImportError:
        print("错误: 需要安装 rosbag")
        print("请运行: pip install rosbag bagpy")
        return None
    
    analyzer = TopicRateAnalyzer(config)
    
    # 话题映射
    topic_map = {cfg['topic']: cfg['topic'] for cfg in TOPIC_CONFIG.values()}
    
    print(f"读取 bag 文件: {bag_path}")
    
    try:
        bag = rosbag.Bag(bag_path, 'r')
        
        for topic, msg, t in bag.read_messages():
            timestamp = t.to_sec()
            
            if topic in topic_map:
                analyzer.add_topic_sample(topic, timestamp)
            elif topic == '/tf' or topic == '/tf_static':
                # 简化处理 TF
                analyzer.add_tf_sample(timestamp)
        
        bag.close()
        
    except Exception as e:
        print(f"读取 bag 文件失败: {e}")
        return None
    
    return analyzer


def load_config(config_path: str = None) -> Dict[str, Any]:
    """加载配置文件"""
    import yaml
    
    config = {}
    
    # 默认配置路径
    if config_path is None:
        config_path = 'controller_ros/config/platforms/turtlebot1.yaml'
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}
        print(f"已加载配置: {config_path}")
    except Exception as e:
        print(f"警告: 无法加载配置文件 {config_path}: {e}")
    
    return config


def main():
    parser = argparse.ArgumentParser(description='话题帧率分析与参数调优工具')
    parser.add_argument('--live', action='store_true', help='实时收集数据')
    parser.add_argument('--bag', type=str, help='ROS bag 文件路径')
    parser.add_argument('--duration', type=float, default=30, help='实时收集持续时间(秒)')
    parser.add_argument('--config', type=str, help='配置文件路径')
    parser.add_argument('--suggest', action='store_true', help='生成调优建议')
    args = parser.parse_args()
    
    # 加载配置
    config = load_config(args.config)
    
    # 收集数据
    analyzer = None
    
    if args.live:
        analyzer = collect_live(args.duration, config)
    elif args.bag:
        analyzer = collect_from_bag(args.bag, config)
    else:
        print("请指定数据源: --live 或 --bag")
        print("\n使用示例:")
        print("  python -m tools.tuning.analyze_topic_rates --live --duration 30")
        print("  python -m tools.tuning.analyze_topic_rates --bag recording.bag")
        return
    
    if analyzer is None:
        return
    
    # 分析
    analyzer.analyze()
    
    # 打印报告
    analyzer.print_report()


if __name__ == '__main__':
    main()
