#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强诊断模块 v2.0 - 统一架构重构版

此模块提供额外的诊断功能，用于分析影响轨迹跟踪但未被标准诊断覆盖的参数：
1. MPC 权重分析（position, velocity, heading, control_accel, control_alpha）
2. 一致性检查性能分析（alpha 拒绝率，kappa/v_dir 阈值）
3. 状态机切换分析（切换频率，MPC 失败检测）

架构改进：
- 统一使用消息 timestamp，不再使用 time.time()
- 统一变化率计算逻辑，避免重复
- 添加纵向跟踪误差分析
- 修正单位标注（m/s² 而非 m/s²/s）
- 优化建议生成逻辑

使用方法:
  # 在 unified_diagnostics.py 中导入使用
  from enhanced_diagnostics import EnhancedDiagnostics
  
  # 或作为独立脚本运行
  rosrun controller_ros enhanced_diagnostics.py --duration 60

作者: Kiro Auto-generated
版本: 2.0
"""

import sys
import os
import numpy as np
import yaml
from collections import deque
from typing import Dict, List, Any, Optional
from dataclasses import dataclass

# 修复 Windows 编码问题
if sys.platform == 'win32':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8')
    os.environ['PYTHONIOENCODING'] = 'utf-8'


@dataclass
class ControlSample:
    """控制样本数据结构"""
    timestamp: float  # 消息时间戳（秒）
    vx: float
    vy: float
    omega: float
    lateral_error: float
    longitudinal_error: float  # 新增：纵向误差
    heading_error: float
    alpha: float
    state: str
    mpc_success: bool


class EnhancedDiagnostics:
    """
    增强诊断分析器 v2.0
    
    统一架构，避免重复计算，提供完整的参数诊断
    """
    
    def __init__(self, window_size: int = 200):
        """
        初始化
        
        Args:
            window_size: 滑动窗口大小（默认200个样本）
        """
        self.window_size = window_size
        self.samples = deque(maxlen=window_size)
        
        # 统计数据
        self.state_transitions = []  # (timestamp, from_state, to_state)
        self.alpha_rejections = 0
        self.total_samples = 0
        
    def add_sample(self, diag: Dict[str, Any]):
        """
        添加诊断样本
        
        Args:
            diag: DiagnosticsV2 消息字典，必须包含 timestamp
        """
        # 使用消息自带的 timestamp，而不是 time.time()
        timestamp = diag.get('timestamp', 0.0)
        if timestamp == 0.0:
            # 如果消息没有 timestamp，使用 header.stamp
            header = diag.get('header')
            if header:
                timestamp = header.stamp.to_sec() if hasattr(header.stamp, 'to_sec') else float(header.stamp.secs) + float(header.stamp.nsecs) * 1e-9
        
        sample = ControlSample(
            timestamp=timestamp,
            vx=diag.get('cmd_vx', 0.0),
            vy=diag.get('cmd_vy', 0.0),
            omega=diag.get('cmd_omega', 0.0),
            lateral_error=diag.get('tracking_lateral_error', 0.0),
            longitudinal_error=diag.get('tracking_longitudinal_error', 0.0),
            heading_error=diag.get('tracking_heading_error', 0.0),
            alpha=diag.get('alpha', 1.0),
            state=diag.get('state', 'UNKNOWN'),
            mpc_success=diag.get('mpc_success', False)
        )
        
        # 检测状态切换
        if len(self.samples) > 0:
            last_state = self.samples[-1].state
            if last_state != sample.state:
                self.state_transitions.append((sample.timestamp, last_state, sample.state))
        
        # 统计 alpha 拒绝（alpha < 0.5 表示一致性检查拒绝）
        if sample.alpha < 0.5:
            self.alpha_rejections += 1
        
        self.samples.append(sample)
        self.total_samples += 1
    
    def _calculate_control_derivatives(self) -> Dict[str, Any]:
        """
        统一计算控制输出的变化率（加速度和角加速度）
        
        Returns:
            包含所有变化率指标的字典
        """
        if len(self.samples) < 2:
            return None
        
        vx_changes = []
        omega_changes = []
        
        for i in range(1, len(self.samples)):
            dt = self.samples[i].timestamp - self.samples[i-1].timestamp
            
            # 过滤异常时间间隔
            if dt <= 0 or dt > 0.5:
                continue
            
            # 计算变化率（加速度 = dv/dt）
            dvx = abs(self.samples[i].vx - self.samples[i-1].vx) / dt
            domega = abs(self.samples[i].omega - self.samples[i-1].omega) / dt
            
            vx_changes.append(dvx)
            omega_changes.append(domega)
        
        if not vx_changes:
            return None
        
        return {
            'avg_accel': np.mean(vx_changes),
            'max_accel': np.max(vx_changes),
            'avg_angular_accel': np.mean(omega_changes),
            'max_angular_accel': np.max(omega_changes),
            'vx_std': np.std([s.vx for s in self.samples]),
            'omega_std': np.std([s.omega for s in self.samples])
        }
    
    def analyze_mpc_weights(self) -> Dict[str, Any]:
        """
        分析 MPC 权重是否合理
        
        检测 5 个权重参数：position, velocity, heading, control_accel, control_alpha
        通过跟踪误差和控制平滑性的权衡来判断权重设置
        
        Returns:
            分析结果和建议
        """
        if len(self.samples) < 10:
            return {"status": "insufficient_data", "message": "需要更多数据（至少10个样本）"}
        
        # 计算跟踪误差
        lateral_errors = [abs(s.lateral_error) for s in self.samples]
        longitudinal_errors = [abs(s.longitudinal_error) for s in self.samples]
        heading_errors = [abs(s.heading_error) for s in self.samples]
        
        avg_lateral = np.mean(lateral_errors)
        max_lateral = np.max(lateral_errors)
        avg_longitudinal = np.mean(longitudinal_errors)
        max_longitudinal = np.max(longitudinal_errors)
        avg_heading = np.mean(heading_errors)
        max_heading = np.max(heading_errors)
        
        # 计算控制平滑性
        derivatives = self._calculate_control_derivatives()
        if not derivatives:
            return {"status": "insufficient_data", "message": "无法计算控制变化率"}
        
        # 构建结果
        result = {
            "status": "ok",
            "metrics": {
                "avg_lateral_error": avg_lateral,
                "max_lateral_error": max_lateral,
                "avg_longitudinal_error": avg_longitudinal,
                "max_longitudinal_error": max_longitudinal,
                "avg_heading_error": avg_heading,
                "max_heading_error": max_heading,
                "avg_accel": derivatives['avg_accel'],
                "max_accel": derivatives['max_accel'],
                "avg_angular_accel": derivatives['avg_angular_accel'],
                "max_angular_accel": derivatives['max_angular_accel']
            },
            "suggestions": []
        }
        
        # 判断逻辑：误差大但控制平滑 → 跟踪权重过低
        # 阈值说明：
        # - 横向误差 > 0.15m 认为较大
        # - 纵向误差 > 0.20m 认为较大
        # - 航向误差 > 0.3 rad (17°) 认为较大
        # - 加速度 < 3.0 m/s² 认为平滑
        # - 角加速度 < 5.0 rad/s² 认为平滑
        
        if avg_lateral > 0.15 and derivatives['max_accel'] < 3.0:
            result["suggestions"].append({
                "parameter": "mpc.weights.position",
                "current_issue": f"横向误差较大 (avg={avg_lateral:.3f}m, max={max_lateral:.3f}m) 但控制很平滑",
                "suggestion": "增加 position 权重 (建议从 10.0 → 15.0)",
                "priority": "high"
            })
        
        if avg_longitudinal > 0.20 and derivatives['max_accel'] < 3.0:
            result["suggestions"].append({
                "parameter": "mpc.weights.velocity",
                "current_issue": f"纵向误差较大 (avg={avg_longitudinal:.3f}m, max={max_longitudinal:.3f}m) 但控制很平滑",
                "suggestion": "增加 velocity 权重 (建议从 5.0 → 8.0)",
                "priority": "high"
            })
        
        if avg_heading > 0.3 and derivatives['max_angular_accel'] < 5.0:
            result["suggestions"].append({
                "parameter": "mpc.weights.heading",
                "current_issue": f"航向误差较大 (avg={np.rad2deg(avg_heading):.1f}°, max={np.rad2deg(max_heading):.1f}°) 但控制很平滑",
                "suggestion": "增加 heading 权重 (建议从 5.0 → 8.0)",
                "priority": "high"
            })
        
        # 判断逻辑：控制抖动大 → 平滑权重过低
        # 阈值说明：
        # - 加速度 > 8.0 m/s² 认为抖动大
        # - 角加速度 > 15.0 rad/s² 认为抖动大
        
        if derivatives['max_accel'] > 8.0:
            result["suggestions"].append({
                "parameter": "mpc.weights.control_accel",
                "current_issue": f"加速度变化过大 (avg={derivatives['avg_accel']:.2f} m/s², max={derivatives['max_accel']:.2f} m/s²)",
                "suggestion": "增加 control_accel 权重 (建议从 0.2 → 0.5)",
                "priority": "high"
            })
        
        if derivatives['max_angular_accel'] > 15.0:
            result["suggestions"].append({
                "parameter": "mpc.weights.control_alpha",
                "current_issue": f"角加速度变化过大 (avg={derivatives['avg_angular_accel']:.2f} rad/s², max={derivatives['max_angular_accel']:.2f} rad/s²)",
                "suggestion": "增加 control_alpha 权重 (建议从 0.2 → 0.5)",
                "priority": "high"
            })
        
        # 判断逻辑：误差小且控制平滑 → 权重设置良好
        if avg_lateral < 0.10 and avg_longitudinal < 0.15 and derivatives['max_accel'] < 3.0:
            result["suggestions"].append({
                "parameter": "mpc.weights",
                "current_issue": "无",
                "suggestion": "权重设置良好，跟踪精度和平滑性平衡",
                "priority": "info"
            })
        
        return result
    
    def analyze_consistency_check(self) -> Dict[str, Any]:
        """
        分析一致性检查性能
        
        检测参数：kappa_thresh, v_dir_thresh
        通过 alpha 拒绝率判断阈值是否过严
        
        Returns:
            分析结果和建议
        """
        if self.total_samples < 10:
            return {"status": "insufficient_data", "message": "需要更多数据（至少10个样本）"}
        
        rejection_rate = self.alpha_rejections / self.total_samples
        
        # 统计 alpha 分布
        alpha_values = [s.alpha for s in self.samples]
        avg_alpha = np.mean(alpha_values)
        min_alpha = np.min(alpha_values)
        
        result = {
            "status": "ok",
            "metrics": {
                "rejection_rate": rejection_rate,
                "rejection_count": self.alpha_rejections,
                "total_samples": self.total_samples,
                "avg_alpha": avg_alpha,
                "min_alpha": min_alpha
            },
            "suggestions": []
        }
        
        # 判断逻辑：拒绝率过高 → 阈值过严
        # 阈值说明：
        # - 拒绝率 > 10% 认为过高
        # - 拒绝率 5-10% 认为偏高
        # - 拒绝率 < 5% 认为良好
        
        if rejection_rate > 0.1:
            result["suggestions"].append({
                "parameter": "consistency.kappa_thresh / v_dir_thresh",
                "current_issue": f"一致性检查拒绝率过高 ({rejection_rate*100:.1f}%)",
                "suggestion": "放宽一致性阈值 (kappa_thresh: 0.5→0.7, v_dir_thresh: 0.8→0.9)",
                "priority": "high"
            })
        elif rejection_rate > 0.05:
            result["suggestions"].append({
                "parameter": "consistency.kappa_thresh / v_dir_thresh",
                "current_issue": f"一致性检查拒绝率偏高 ({rejection_rate*100:.1f}%)",
                "suggestion": "考虑适当放宽阈值 (kappa_thresh: +0.1, v_dir_thresh: +0.05)",
                "priority": "medium"
            })
        else:
            result["suggestions"].append({
                "parameter": "consistency check",
                "current_issue": "无",
                "suggestion": f"一致性检查性能良好 (拒绝率 {rejection_rate*100:.1f}%)",
                "priority": "info"
            })
        
        # 判断逻辑：min_alpha 很低 → 轨迹质量问题
        # 阈值说明：
        # - min_alpha < 0.2 认为严重问题
        # - min_alpha < 0.3 认为需要关注
        
        if min_alpha < 0.2:
            result["suggestions"].append({
                "parameter": "trajectory quality",
                "current_issue": f"检测到严重的轨迹一致性问题 (min_alpha={min_alpha:.2f})",
                "suggestion": "检查轨迹发布节点，可能存在数据质量问题或网络延迟",
                "priority": "critical"
            })
        elif min_alpha < 0.3:
            result["suggestions"].append({
                "parameter": "trajectory quality",
                "current_issue": f"检测到轨迹一致性偏低 (min_alpha={min_alpha:.2f})",
                "suggestion": "建议检查轨迹发布频率和质量",
                "priority": "medium"
            })
        
        return result
    
    def analyze_state_machine(self) -> Dict[str, Any]:
        """
        分析状态机切换频率
        
        检测参数：mpc_fail_thresh, mpc_recovery_thresh
        通过切换频率和类型判断状态机参数是否合理
        
        Returns:
            分析结果和建议
        """
        if len(self.state_transitions) == 0:
            return {
                "status": "ok",
                "metrics": {
                    "transition_count": 0,
                    "transition_rate": 0.0,
                    "transition_types": {}
                },
                "suggestions": [{
                    "parameter": "state machine",
                    "current_issue": "无",
                    "suggestion": "无状态切换，控制器稳定运行",
                    "priority": "info"
                }]
            }
        
        # 统计切换频率
        if len(self.samples) > 1:
            duration = self.samples[-1].timestamp - self.samples[0].timestamp
            transition_rate = len(self.state_transitions) / max(duration, 1.0)
        else:
            transition_rate = 0.0
        
        # 统计切换类型
        transition_types = {}
        mpc_to_backup_count = 0
        backup_to_mpc_count = 0
        
        for _, from_state, to_state in self.state_transitions:
            key = f"{from_state} → {to_state}"
            transition_types[key] = transition_types.get(key, 0) + 1
            
            # 统计 MPC 失败切换
            if "MPC" in from_state and "BACKUP" in to_state:
                mpc_to_backup_count += 1
            elif "BACKUP" in from_state and "MPC" in to_state:
                backup_to_mpc_count += 1
        
        result = {
            "status": "ok",
            "metrics": {
                "transition_count": len(self.state_transitions),
                "transition_rate": transition_rate,
                "transition_types": transition_types,
                "mpc_to_backup_count": mpc_to_backup_count,
                "backup_to_mpc_count": backup_to_mpc_count
            },
            "suggestions": []
        }
        
        # 判断逻辑：切换频繁 → 状态机参数不当
        # 阈值说明：
        # - 切换频率 > 0.5 次/秒 认为频繁
        # - 切换频率 > 0.1 次/秒 认为偏高
        
        if transition_rate > 0.5:
            result["suggestions"].append({
                "parameter": "safety.state_machine.mpc_fail_thresh / mpc_recovery_thresh",
                "current_issue": f"状态切换频繁 ({transition_rate:.2f} 次/秒)",
                "suggestion": "调整状态机参数，减少不必要的切换 (mpc_fail_thresh: 3→5, mpc_recovery_thresh: 5→3)",
                "priority": "high"
            })
        elif transition_rate > 0.1:
            result["suggestions"].append({
                "parameter": "safety.state_machine",
                "current_issue": f"状态切换偏频繁 ({transition_rate:.2f} 次/秒)",
                "suggestion": "考虑调整状态机参数",
                "priority": "medium"
            })
        
        # 判断逻辑：MPC 失败切换 → MPC 性能或备用控制器问题
        if mpc_to_backup_count > 0:
            result["suggestions"].append({
                "parameter": "MPC performance / backup controller",
                "current_issue": f"检测到 {mpc_to_backup_count} 次 MPC 失败切换到备用控制器",
                "suggestion": "检查 MPC 求解器健康度，或优化备用控制器参数",
                "priority": "high" if mpc_to_backup_count > 5 else "medium"
            })
        
        return result
    
    def generate_report(self) -> str:
        """
        生成完整的增强诊断报告
        
        Returns:
            格式化的报告文本
        """
        report = []
        report.append("\n" + "="*70)
        report.append("  增强诊断报告 (Enhanced Diagnostics Report v2.0)")
        report.append("="*70)
        
        # 1. MPC 权重分析
        report.append("\n【1. MPC 权重分析】")
        mpc_result = self.analyze_mpc_weights()
        if mpc_result["status"] == "ok":
            m = mpc_result["metrics"]
            report.append(f"  横向误差: avg={m['avg_lateral_error']:.3f}m, max={m['max_lateral_error']:.3f}m")
            report.append(f"  纵向误差: avg={m['avg_longitudinal_error']:.3f}m, max={m['max_longitudinal_error']:.3f}m")
            report.append(f"  航向误差: avg={np.rad2deg(m['avg_heading_error']):.1f}°, max={np.rad2deg(m['max_heading_error']):.1f}°")
            report.append(f"  加速度: avg={m['avg_accel']:.2f} m/s², max={m['max_accel']:.2f} m/s²")
            report.append(f"  角加速度: avg={m['avg_angular_accel']:.2f} rad/s², max={m['max_angular_accel']:.2f} rad/s²")
            
            if mpc_result["suggestions"]:
                report.append("\n  建议:")
                for sug in mpc_result["suggestions"]:
                    icon = {"critical": "🔴", "high": "⚠️", "medium": "ℹ️", "info": "✅"}.get(sug["priority"], "•")
                    report.append(f"    {icon} {sug['parameter']}")
                    report.append(f"       问题: {sug['current_issue']}")
                    report.append(f"       建议: {sug['suggestion']}")
        else:
            report.append(f"  {mpc_result['message']}")
        
        # 2. 一致性检查分析
        report.append("\n【2. 一致性检查分析】")
        consistency_result = self.analyze_consistency_check()
        if consistency_result["status"] == "ok":
            m = consistency_result["metrics"]
            report.append(f"  拒绝率: {m['rejection_rate']*100:.1f}% ({m['rejection_count']}/{m['total_samples']})")
            report.append(f"  Alpha: avg={m['avg_alpha']:.3f}, min={m['min_alpha']:.3f}")
            
            if consistency_result["suggestions"]:
                report.append("\n  建议:")
                for sug in consistency_result["suggestions"]:
                    icon = {"critical": "🔴", "high": "⚠️", "medium": "ℹ️", "info": "✅"}.get(sug["priority"], "•")
                    report.append(f"    {icon} {sug['parameter']}")
                    report.append(f"       问题: {sug['current_issue']}")
                    report.append(f"       建议: {sug['suggestion']}")
        else:
            report.append(f"  {consistency_result['message']}")
        
        # 3. 状态机切换分析
        report.append("\n【3. 状态机切换分析】")
        state_result = self.analyze_state_machine()
        m = state_result["metrics"]
        report.append(f"  切换次数: {m['transition_count']}")
        if m['transition_count'] > 0:
            report.append(f"  切换频率: {m['transition_rate']:.2f} 次/秒")
            report.append(f"  MPC→Backup: {m['mpc_to_backup_count']} 次")
            report.append(f"  Backup→MPC: {m['backup_to_mpc_count']} 次")
            report.append("  切换类型:")
            for trans_type, count in m['transition_types'].items():
                report.append(f"    {trans_type}: {count} 次")
        
        if state_result["suggestions"]:
            report.append("\n  建议:")
            for sug in state_result["suggestions"]:
                icon = {"critical": "🔴", "high": "⚠️", "medium": "ℹ️", "info": "✅"}.get(sug["priority"], "•")
                report.append(f"    {icon} {sug['parameter']}")
                report.append(f"       问题: {sug['current_issue']}")
                report.append(f"       建议: {sug['suggestion']}")
        
        report.append("\n" + "="*70)
        
        return "\n".join(report)
    
    def get_all_suggestions(self) -> List[Dict[str, Any]]:
        """
        获取所有建议（用于配置文件生成）
        
        Returns:
            按优先级排序的建议列表
        """
        all_suggestions = []
        
        # 收集所有分析的建议
        for analysis_func in [
            self.analyze_mpc_weights,
            self.analyze_consistency_check,
            self.analyze_state_machine
        ]:
            result = analysis_func()
            if result.get("status") == "ok" and "suggestions" in result:
                all_suggestions.extend(result["suggestions"])
        
        # 按优先级排序
        priority_order = {"critical": 0, "high": 1, "medium": 2, "info": 3}
        all_suggestions.sort(key=lambda x: priority_order.get(x["priority"], 99))
        
        return all_suggestions


# 独立运行时的主函数
if __name__ == '__main__':
    try:
        import rospy
        from controller_ros.msg import DiagnosticsV2
    except ImportError:
        print("错误: 无法导入 ROS 或 controller_ros 消息")
        print("请确保在 ROS 环境中运行此脚本")
        sys.exit(1)
    
    import argparse
    
    parser = argparse.ArgumentParser(description='增强诊断工具 - 独立运行模式')
    parser.add_argument('--duration', type=float, default=60.0, help='诊断持续时间(秒)')
    parser.add_argument('--output', type=str, default='enhanced_diagnostics_suggestions.yaml', 
                        help='建议输出文件')
    args = parser.parse_args()
    
    print("="*70)
    print("  增强诊断工具 v2.0 - 独立运行模式")
    print("="*70)
    print(f"\n订阅 /controller/diagnostics 进行分析...")
    print(f"持续时间: {args.duration} 秒\n")
    
    rospy.init_node('enhanced_diagnostics', anonymous=True)
    
    analyzer = EnhancedDiagnostics(window_size=200)
    
    def diagnostics_callback(msg):
        """诊断消息回调"""
        diag_dict = {
            'header': msg.header,
            'cmd_vx': msg.cmd_vx,
            'cmd_vy': msg.cmd_vy,
            'cmd_omega': msg.cmd_omega,
            'tracking_lateral_error': msg.tracking_lateral_error,
            'tracking_longitudinal_error': msg.tracking_longitudinal_error,
            'tracking_heading_error': msg.tracking_heading_error,
            'alpha': msg.consistency_alpha_soft,  # 修复：使用正确的字段名
            'state': msg.state,
            'mpc_success': msg.mpc_success
        }
        analyzer.add_sample(diag_dict)
    
    sub = rospy.Subscriber('/controller/diagnostics', DiagnosticsV2, diagnostics_callback)
    
    print(f"收集数据 {args.duration} 秒...")
    rospy.sleep(args.duration)
    
    # 生成报告
    print(analyzer.generate_report())
    
    # 保存建议到文件
    suggestions = analyzer.get_all_suggestions()
    if suggestions:
        print(f"\n保存建议到 {args.output}...")
        with open(args.output, 'w', encoding='utf-8') as f:
            yaml.dump({'suggestions': suggestions}, f, default_flow_style=False, allow_unicode=True)
        print("完成!")
    else:
        print("\n无建议需要保存")
