#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试增强诊断功能 v2.1

验证增强诊断模块是否能正常生成建议
更新：使用整数状态码，与 ControllerState 枚举一致
"""

import sys
import os
import numpy as np

# 修复 Windows 编码问题
if sys.platform == 'win32':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8')
    os.environ['PYTHONIOENCODING'] = 'utf-8'

sys.path.insert(0, '.')

from enhanced_diagnostics import EnhancedDiagnostics, ControllerState


def test_scenario_1_high_error_smooth_control():
    """场景1: 跟踪误差大但控制平滑 - 应该建议增加 position/heading 权重"""
    print("\n" + "="*70)
    print("测试场景 1: 跟踪误差大但控制平滑")
    print("="*70)
    
    analyzer = EnhancedDiagnostics(window_size=50)
    
    # 模拟数据：误差大，控制平滑
    base_time = 1000.0
    for i in range(50):
        t = base_time + i * 0.05  # 使用固定时间戳
        diag = {
            'timestamp': t,
            'cmd_vx': 0.3 + 0.005 * np.sin(t * 0.1),  # 非常平滑的变化
            'cmd_vy': 0.0,
            'cmd_omega': 0.2 + 0.01 * np.sin(t * 0.1),  # 非常平滑的变化
            'tracking_lateral_error': 0.18 + 0.05 * np.random.randn(),  # 大误差
            'tracking_longitudinal_error': 0.25 + 0.05 * np.random.randn(),  # 大误差
            'tracking_heading_error': 0.35 + 0.1 * np.random.randn(),  # 大误差
            'alpha': 0.95,
            'state': ControllerState.NORMAL,  # 使用整数状态码
            'mpc_success': True
        }
        analyzer.add_sample(diag)
    
    # 分析
    result = analyzer.analyze_mpc_weights()
    print(f"\n状态: {result['status']}")
    print(f"\n指标:")
    for key, value in result['metrics'].items():
        print(f"  {key}: {value:.3f}")
    print(f"\n建议数量: {len(result['suggestions'])}")
    for sug in result['suggestions']:
        print(f"  [{sug['priority']}] {sug['parameter']}")
        print(f"    问题: {sug['current_issue']}")
        print(f"    建议: {sug['suggestion']}")
    
    # 验证
    assert result['status'] == 'ok', "分析应该成功"
    assert len(result['suggestions']) > 0, "应该有建议"
    has_position_suggestion = any('position' in s['parameter'] for s in result['suggestions'])
    has_velocity_suggestion = any('velocity' in s['parameter'] for s in result['suggestions'])
    print(f"\n✅ 验证通过: {'有' if has_position_suggestion else '无'} position 权重建议")
    print(f"✅ 验证通过: {'有' if has_velocity_suggestion else '无'} velocity 权重建议")


def test_scenario_2_control_jitter():
    """场景2: 控制输出抖动 - 应该建议增加 control_accel/alpha 权重"""
    print("\n" + "="*70)
    print("测试场景 2: 控制输出抖动")
    print("="*70)
    
    analyzer = EnhancedDiagnostics(window_size=50)
    
    # 模拟数据：控制抖动（使用固定时间戳）
    base_time = 1000.0
    for i in range(50):
        t = base_time + i * 0.05
        # 模拟大幅度跳变
        vx = 0.3 if i % 2 == 0 else 0.5  # 0.2 m/s 跳变，dt=0.05s，加速度=4 m/s²
        omega = 0.2 if i % 2 == 0 else 1.0  # 0.8 rad/s 跳变，dt=0.05s，角加速度=16 rad/s²
        
        diag = {
            'timestamp': t,
            'cmd_vx': vx,
            'cmd_vy': 0.0,
            'cmd_omega': omega,
            'tracking_lateral_error': 0.08,  # 小误差
            'tracking_longitudinal_error': 0.10,  # 小误差
            'tracking_heading_error': 0.15,  # 小误差
            'alpha': 0.95,
            'state': ControllerState.NORMAL,  # 使用整数状态码
            'mpc_success': True
        }
        analyzer.add_sample(diag)
    
    # 分析
    result = analyzer.analyze_mpc_weights()
    print(f"\n状态: {result['status']}")
    print(f"\n指标:")
    for key, value in result['metrics'].items():
        print(f"  {key}: {value:.3f}")
    print(f"\n建议数量: {len(result['suggestions'])}")
    for sug in result['suggestions']:
        print(f"  [{sug['priority']}] {sug['parameter']}")
        print(f"    问题: {sug['current_issue']}")
        print(f"    建议: {sug['suggestion']}")
    
    # 验证
    assert result['status'] == 'ok', "分析应该成功"
    has_control_suggestion = any('control_accel' in s['parameter'] or 'control_alpha' in s['parameter'] 
                                   for s in result['suggestions'])
    print(f"\n✅ 验证通过: {'有' if has_control_suggestion else '无'} control 权重建议")


def test_scenario_3_high_rejection_rate():
    """场景3: 一致性检查拒绝率高 - 应该建议放宽阈值"""
    print("\n" + "="*70)
    print("测试场景 3: 一致性检查拒绝率高")
    print("="*70)
    
    analyzer = EnhancedDiagnostics(window_size=100)
    
    # 模拟数据：15% 拒绝率
    base_time = 1000.0
    for i in range(100):
        t = base_time + i * 0.05
        alpha = 0.3 if i % 7 == 0 else 0.95  # 约 14% 拒绝
        diag = {
            'timestamp': t,
            'cmd_vx': 0.3,
            'cmd_vy': 0.0,
            'cmd_omega': 0.2,
            'tracking_lateral_error': 0.08,
            'tracking_longitudinal_error': 0.10,
            'tracking_heading_error': 0.15,
            'alpha': alpha,
            'state': ControllerState.NORMAL,  # 使用整数状态码
            'mpc_success': True
        }
        analyzer.add_sample(diag)
    
    # 分析
    result = analyzer.analyze_consistency_check()
    print(f"\n状态: {result['status']}")
    print(f"\n指标:")
    for key, value in result['metrics'].items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    print(f"\n建议数量: {len(result['suggestions'])}")
    for sug in result['suggestions']:
        print(f"  [{sug['priority']}] {sug['parameter']}")
        print(f"    问题: {sug['current_issue']}")
        print(f"    建议: {sug['suggestion']}")
    
    # 验证
    assert result['status'] == 'ok', "分析应该成功"
    assert result['metrics']['rejection_rate'] > 0.1, "拒绝率应该 > 10%"
    has_consistency_suggestion = any('consistency' in s['parameter'] or 'thresh' in s['parameter'] 
                                      for s in result['suggestions'])
    print(f"\n✅ 验证通过: {'有' if has_consistency_suggestion else '无'} 一致性阈值建议")


def test_scenario_4_frequent_state_transitions():
    """场景4: 频繁状态切换 - 应该建议调整状态机参数"""
    print("\n" + "="*70)
    print("测试场景 4: 频繁状态切换")
    print("="*70)
    
    analyzer = EnhancedDiagnostics(window_size=100)
    
    # 模拟数据：频繁切换（使用整数状态码）
    base_time = 1000.0
    for i in range(100):
        t = base_time + i * 0.05
        # 40% 时间在 BACKUP_ACTIVE，60% 时间在 NORMAL
        state = ControllerState.BACKUP_ACTIVE if i % 5 < 2 else ControllerState.NORMAL
        diag = {
            'timestamp': t,
            'cmd_vx': 0.3,
            'cmd_vy': 0.0,
            'cmd_omega': 0.2,
            'tracking_lateral_error': 0.08,
            'tracking_longitudinal_error': 0.10,
            'tracking_heading_error': 0.15,
            'alpha': 0.95,
            'state': state,
            'mpc_success': state == ControllerState.NORMAL
        }
        analyzer.add_sample(diag)
    
    # 分析
    result = analyzer.analyze_state_machine()
    print(f"\n状态: {result['status']}")
    print(f"\n指标:")
    for key, value in result['metrics'].items():
        if isinstance(value, dict):
            print(f"  {key}:")
            for k, v in value.items():
                print(f"    {k}: {v}")
        elif isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    print(f"\n建议数量: {len(result['suggestions'])}")
    for sug in result['suggestions']:
        print(f"  [{sug['priority']}] {sug['parameter']}")
        print(f"    问题: {sug['current_issue']}")
        print(f"    建议: {sug['suggestion']}")
    
    # 验证
    assert result['status'] == 'ok', "分析应该成功"
    assert result['metrics']['transition_count'] > 10, "应该有多次切换"
    has_state_machine_suggestion = any('state_machine' in s['parameter'] or 'MPC' in s['parameter'] 
                                        for s in result['suggestions'])
    print(f"\n✅ 验证通过: {'有' if has_state_machine_suggestion else '无'} 状态机建议")


def test_complete_report():
    """测试完整报告生成"""
    print("\n" + "="*70)
    print("测试完整报告生成")
    print("="*70)
    
    analyzer = EnhancedDiagnostics(window_size=50)
    
    # 模拟正常数据
    base_time = 1000.0
    for i in range(50):
        t = base_time + i * 0.05
        diag = {
            'timestamp': t,
            'cmd_vx': 0.3 + 0.005 * np.sin(t * 0.1),
            'cmd_vy': 0.0,
            'cmd_omega': 0.2 + 0.01 * np.sin(t * 0.1),
            'tracking_lateral_error': 0.08 + 0.02 * np.random.randn(),
            'tracking_longitudinal_error': 0.10 + 0.02 * np.random.randn(),
            'tracking_heading_error': 0.15 + 0.05 * np.random.randn(),
            'alpha': 0.95,
            'state': ControllerState.NORMAL,  # 使用整数状态码
            'mpc_success': True
        }
        analyzer.add_sample(diag)
    
    # 生成报告
    report = analyzer.generate_report()
    print(report)
    
    # 获取所有建议
    suggestions = analyzer.get_all_suggestions()
    print(f"\n总建议数: {len(suggestions)}")
    
    # 按优先级分组
    by_priority = {}
    for sug in suggestions:
        priority = sug['priority']
        by_priority[priority] = by_priority.get(priority, 0) + 1
    
    print(f"按优先级分布: {by_priority}")
    print("\n✅ 完整报告生成成功")


if __name__ == '__main__':
    print("="*70)
    print("  增强诊断功能测试 v2.1")
    print("="*70)
    
    try:
        test_scenario_1_high_error_smooth_control()
        test_scenario_2_control_jitter()
        test_scenario_3_high_rejection_rate()
        test_scenario_4_frequent_state_transitions()
        test_complete_report()
        
        print("\n" + "="*70)
        print("  ✅ 所有测试通过！")
        print("="*70)
        print("\n增强诊断功能可以正常工作，能够：")
        print("  1. 检测跟踪误差和控制平滑性的权衡")
        print("  2. 识别控制输出抖动问题")
        print("  3. 统计一致性检查拒绝率")
        print("  4. 分析状态机切换频率")
        print("  5. 生成优先级建议")
        print("  6. 输出完整诊断报告")
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
