#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
帧率和话题分析工具

分析诊断数据中的话题帧率、轨迹点数、MPC 降级原因等信息。
这是对现有调优工具的补充，专注于帧率相关的分析。

使用方法:
    python -m tools.tuning.analyze_frame_rate --json tuning_output/collected_diagnostics.json
"""

import json
import argparse
import numpy as np
from pathlib import Path
from typing import Dict, List, Any, Optional
from collections import defaultdict
from dataclasses import dataclass, field


@dataclass
class FrameRateStats:
    """帧率统计"""
    topic_name: str
    sample_count: int = 0
    intervals: List[float] = field(default_factory=list)
    
    @property
    def avg_interval_ms(self) -> float:
        if not self.intervals:
            return 0.0
        return np.mean(self.intervals) * 1000
    
    @property
    def avg_hz(self) -> float:
        if not self.intervals or np.mean(self.intervals) == 0:
            return 0.0
        return 1.0 / np.mean(self.intervals)
    
    @property
    def min_hz(self) -> float:
        if not self.intervals:
            return 0.0
        max_interval = np.max(self.intervals)
        return 1.0 / max_interval if max_interval > 0 else 0.0
    
    @property
    def max_hz(self) -> float:
        if not self.intervals:
            return 0.0
        min_interval = np.min(self.intervals)
        return 1.0 / min_interval if min_interval > 0 else 0.0
    
    @property
    def std_hz(self) -> float:
        if len(self.intervals) < 2:
            return 0.0
        hz_values = [1.0/i for i in self.intervals if i > 0]
        return np.std(hz_values) if hz_values else 0.0


@dataclass
class MPCDegradationAnalysis:
    """MPC 降级分析"""
    total_samples: int = 0
    degraded_samples: int = 0
    
    # 降级原因统计
    no_solve_count: int = 0           # mpc_solve_time_ms == 0
    safety_failed_count: int = 0       # safety_check_passed == False
    traj_timeout_count: int = 0        # traj_timeout == True
    traj_grace_exceeded_count: int = 0 # traj_grace_exceeded == True
    tf2_unavailable_count: int = 0     # tf2_available == False
    
    # 轨迹延迟统计
    traj_ages_during_degradation: List[float] = field(default_factory=list)
    
    @property
    def degradation_rate(self) -> float:
        return self.degraded_samples / self.total_samples * 100 if self.total_samples > 0 else 0.0


def analyze_frame_rate(samples: List[Dict[str, Any]]) -> Dict[str, FrameRateStats]:
    """分析诊断消息的帧率
    
    通过分析连续消息的时间戳差异来计算帧率。
    """
    if len(samples) < 2:
        return {}
    
    # 诊断话题帧率
    diag_stats = FrameRateStats(topic_name="/controller/diagnostics")
    
    prev_stamp = None
    for sample in samples:
        stamp = sample.get('header', {}).get('stamp', 0)
        if prev_stamp is not None and stamp > prev_stamp:
            interval = stamp - prev_stamp
            if 0 < interval < 10:  # 过滤异常值
                diag_stats.intervals.append(interval)
        prev_stamp = stamp
        diag_stats.sample_count += 1
    
    return {'diagnostics': diag_stats}


def analyze_input_topic_rates(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """分析输入话题的有效帧率
    
    通过分析 last_xxx_age_ms 字段的变化来推断输入话题的帧率。
    """
    odom_ages = []
    traj_ages = []
    imu_ages = []
    
    for sample in samples:
        timeout = sample.get('timeout', {})
        if 'last_odom_age_ms' in timeout:
            odom_ages.append(timeout['last_odom_age_ms'])
        if 'last_traj_age_ms' in timeout:
            traj_ages.append(timeout['last_traj_age_ms'])
        if 'last_imu_age_ms' in timeout:
            imu_ages.append(timeout['last_imu_age_ms'])
    
    result = {}
    
    if odom_ages:
        odom_arr = np.array(odom_ages)
        result['odom'] = {
            'avg_age_ms': np.mean(odom_arr),
            'max_age_ms': np.max(odom_arr),
            'p95_age_ms': np.percentile(odom_arr, 95),
            'estimated_hz': 1000.0 / np.mean(odom_arr) if np.mean(odom_arr) > 0 else 0,
        }
    
    if traj_ages:
        traj_arr = np.array(traj_ages)
        result['trajectory'] = {
            'avg_age_ms': np.mean(traj_arr),
            'max_age_ms': np.max(traj_arr),
            'p95_age_ms': np.percentile(traj_arr, 95),
            'estimated_hz': 1000.0 / np.mean(traj_arr) if np.mean(traj_arr) > 0 else 0,
        }
    
    if imu_ages:
        imu_arr = np.array(imu_ages)
        result['imu'] = {
            'avg_age_ms': np.mean(imu_arr),
            'max_age_ms': np.max(imu_arr),
            'p95_age_ms': np.percentile(imu_arr, 95),
            'estimated_hz': 1000.0 / np.mean(imu_arr) if np.mean(imu_arr) > 0 else 0,
        }
    
    return result


def analyze_mpc_degradation(samples: List[Dict[str, Any]]) -> MPCDegradationAnalysis:
    """分析 MPC 降级原因"""
    analysis = MPCDegradationAnalysis()
    
    # ControllerState 枚举值
    STATE_MPC_DEGRADED = 3
    STATE_STOPPING = 5
    STATE_STOPPED = 6
    
    for sample in samples:
        analysis.total_samples += 1
        state = sample.get('state', 0)
        
        # 检查是否处于降级相关状态
        if state == STATE_MPC_DEGRADED:
            analysis.degraded_samples += 1
            
            # 分析降级原因
            mpc_solve_time = sample.get('mpc_solve_time_ms', 0)
            if mpc_solve_time == 0:
                analysis.no_solve_count += 1
            
            if not sample.get('safety_check_passed', True):
                analysis.safety_failed_count += 1
            
            timeout = sample.get('timeout', {})
            if timeout.get('traj_timeout', False):
                analysis.traj_timeout_count += 1
            if timeout.get('traj_grace_exceeded', False):
                analysis.traj_grace_exceeded_count += 1
            
            traj_age = timeout.get('last_traj_age_ms', 0)
            if traj_age > 0:
                analysis.traj_ages_during_degradation.append(traj_age)
            
            transform = sample.get('transform', {})
            if not transform.get('tf2_available', True):
                analysis.tf2_unavailable_count += 1
    
    return analysis


def analyze_state_transitions(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """分析状态转换"""
    state_names = {
        0: 'INIT',
        1: 'NORMAL',
        2: 'SOFT_DISABLED',
        3: 'MPC_DEGRADED',
        4: 'BACKUP_ACTIVE',
        5: 'STOPPING',
        6: 'STOPPED',
    }
    
    state_counts = defaultdict(int)
    transitions = defaultdict(int)
    
    prev_state = None
    for sample in samples:
        state = sample.get('state', 0)
        state_counts[state] += 1
        
        if prev_state is not None and state != prev_state:
            transition_key = f"{state_names.get(prev_state, str(prev_state))} -> {state_names.get(state, str(state))}"
            transitions[transition_key] += 1
        
        prev_state = state
    
    return {
        'state_distribution': {state_names.get(k, str(k)): v for k, v in state_counts.items()},
        'transitions': dict(transitions),
    }


def print_analysis_report(
    frame_rates: Dict[str, FrameRateStats],
    input_rates: Dict[str, Any],
    degradation: MPCDegradationAnalysis,
    state_analysis: Dict[str, Any],
):
    """打印分析报告"""
    print("=" * 70)
    print("帧率和话题分析报告")
    print("=" * 70)
    
    # 诊断话题帧率
    print("\n【诊断话题帧率】")
    if 'diagnostics' in frame_rates:
        stats = frame_rates['diagnostics']
        print(f"  样本数: {stats.sample_count}")
        print(f"  平均帧率: {stats.avg_hz:.1f} Hz")
        print(f"  最小帧率: {stats.min_hz:.1f} Hz")
        print(f"  最大帧率: {stats.max_hz:.1f} Hz")
        print(f"  帧率标准差: {stats.std_hz:.2f} Hz")
    
    # 输入话题延迟
    print("\n【输入话题延迟分析】")
    for topic, data in input_rates.items():
        print(f"\n  {topic}:")
        print(f"    平均延迟: {data['avg_age_ms']:.1f} ms")
        print(f"    最大延迟: {data['max_age_ms']:.1f} ms")
        print(f"    95%分位延迟: {data['p95_age_ms']:.1f} ms")
        print(f"    估计帧率: {data['estimated_hz']:.1f} Hz")
    
    # MPC 降级分析
    print("\n【MPC 降级分析】")
    print(f"  总样本数: {degradation.total_samples}")
    print(f"  降级样本数: {degradation.degraded_samples} ({degradation.degradation_rate:.1f}%)")
    
    if degradation.degraded_samples > 0:
        print("\n  降级原因统计:")
        print(f"    MPC 未执行求解 (solve_time=0): {degradation.no_solve_count} 次")
        print(f"    安全检查失败: {degradation.safety_failed_count} 次")
        print(f"    轨迹超时: {degradation.traj_timeout_count} 次")
        print(f"    轨迹宽限期超时: {degradation.traj_grace_exceeded_count} 次")
        print(f"    TF2 不可用: {degradation.tf2_unavailable_count} 次")
        
        if degradation.traj_ages_during_degradation:
            traj_arr = np.array(degradation.traj_ages_during_degradation)
            print(f"\n  降级时轨迹延迟:")
            print(f"    平均: {np.mean(traj_arr):.1f} ms")
            print(f"    最大: {np.max(traj_arr):.1f} ms")
    
    # 状态分布
    print("\n【状态分布】")
    for state, count in state_analysis['state_distribution'].items():
        pct = count / degradation.total_samples * 100
        print(f"  {state}: {count} ({pct:.1f}%)")
    
    # 状态转换
    if state_analysis['transitions']:
        print("\n【状态转换统计】")
        for transition, count in sorted(state_analysis['transitions'].items(), key=lambda x: -x[1]):
            print(f"  {transition}: {count} 次")
    
    # 问题诊断
    print("\n" + "=" * 70)
    print("问题诊断和建议")
    print("=" * 70)
    
    issues = []
    
    # 检查轨迹帧率
    if 'trajectory' in input_rates:
        traj_hz = input_rates['trajectory']['estimated_hz']
        if traj_hz < 5:
            issues.append(f"⚠️ 轨迹话题帧率过低 ({traj_hz:.1f} Hz)，建议检查轨迹发布器")
        
        traj_p95 = input_rates['trajectory']['p95_age_ms']
        if traj_p95 > 500:
            issues.append(f"⚠️ 轨迹延迟过高 (p95={traj_p95:.0f}ms)，可能导致 MPC 降级")
    
    # 检查 MPC 降级
    if degradation.degradation_rate > 5:
        issues.append(f"⚠️ MPC 降级率过高 ({degradation.degradation_rate:.1f}%)")
        
        if degradation.no_solve_count > degradation.degraded_samples * 0.5:
            issues.append("  → 主要原因: MPC 未执行求解，可能是轨迹点数不足")
        
        if degradation.traj_timeout_count > 0:
            issues.append("  → 存在轨迹超时，建议增加 watchdog.traj_timeout_ms")
        
        if degradation.traj_grace_exceeded_count > 0:
            issues.append("  → 存在宽限期超时，建议增加 watchdog.traj_grace_ms")
    
    # 检查诊断帧率
    if 'diagnostics' in frame_rates:
        diag_hz = frame_rates['diagnostics'].avg_hz
        if diag_hz < 15:
            issues.append(f"⚠️ 诊断帧率过低 ({diag_hz:.1f} Hz)，可能影响控制性能")
    
    if issues:
        for issue in issues:
            print(issue)
    else:
        print("✅ 未发现明显问题")
    
    print()


def main():
    parser = argparse.ArgumentParser(description='帧率和话题分析工具')
    parser.add_argument('--json', type=str, required=True, help='诊断数据 JSON 文件路径')
    args = parser.parse_args()
    
    # 加载数据
    json_path = Path(args.json)
    if not json_path.exists():
        print(f"错误: 文件不存在 {json_path}")
        return
    
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    samples = data.get('samples', data) if isinstance(data, dict) else data
    
    if not samples:
        print("错误: 没有找到诊断样本")
        return
    
    print(f"加载了 {len(samples)} 个诊断样本")
    
    # 执行分析
    frame_rates = analyze_frame_rate(samples)
    input_rates = analyze_input_topic_rates(samples)
    degradation = analyze_mpc_degradation(samples)
    state_analysis = analyze_state_transitions(samples)
    
    # 打印报告
    print_analysis_report(frame_rates, input_rates, degradation, state_analysis)


if __name__ == '__main__':
    main()
