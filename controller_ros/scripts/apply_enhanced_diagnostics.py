#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动应用增强诊断补丁到 unified_diagnostics.py

此脚本会自动修改 unified_diagnostics.py 文件，添加增强诊断功能。

使用方法:
    python3 apply_enhanced_diagnostics.py

作者: Kiro Auto-generated
"""

import os
import sys
import shutil
from datetime import datetime


def backup_file(filepath):
    """备份原文件"""
    backup_path = f"{filepath}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(filepath, backup_path)
    print(f"✅ 已备份原文件到: {backup_path}")
    return backup_path


def apply_patch(filepath):
    """应用增强诊断补丁"""
    
    print(f"📝 读取文件: {filepath}")
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # 检查是否已经应用过补丁
    content = ''.join(lines)
    if 'from enhanced_diagnostics import EnhancedDiagnostics' in content:
        print("⚠️  增强诊断补丁已经应用过，跳过")
        return False
    
    modified_lines = lines.copy()
    modifications = []
    
    # ========================================================================
    # 1. 添加导入语句 (在其他导入之后，约第 75 行)
    # ========================================================================
    import_added = False
    for i, line in enumerate(modified_lines):
        if 'ROS_VERSION = 2' in line and not import_added:
            # 在 ROS 导入块之后添加
            insert_pos = i + 1
            # 找到下一个空行
            while insert_pos < len(modified_lines) and modified_lines[insert_pos].strip():
                insert_pos += 1
            
            import_code = """
# 增强诊断模块
try:
    from enhanced_diagnostics import EnhancedDiagnostics
    ENHANCED_DIAGNOSTICS_AVAILABLE = True
except ImportError:
    ENHANCED_DIAGNOSTICS_AVAILABLE = False
    print("警告: enhanced_diagnostics 模块不可用，部分高级诊断功能将被禁用")

"""
            modified_lines.insert(insert_pos, import_code)
            modifications.append(f"第 {insert_pos} 行: 添加增强诊断导入")
            import_added = True
            break
    
    if not import_added:
        print("❌ 无法找到合适的位置添加导入语句")
        return False
    
    # ========================================================================
    # 2. 在 __init__ 方法中添加增强分析器 (查找 self.diag_monitor = None)
    # ========================================================================
    init_added = False
    for i, line in enumerate(modified_lines):
        if 'self.diag_monitor = None' in line and not init_added:
            insert_pos = i + 1
            init_code = "        self.enhanced_analyzer = None  # 增强诊断分析器\n"
            modified_lines.insert(insert_pos, init_code)
            modifications.append(f"第 {insert_pos} 行: 添加增强分析器初始化")
            init_added = True
            break
    
    if not init_added:
        print("⚠️  警告: 无法找到 __init__ 方法中的 diag_monitor 初始化")
    
    # ========================================================================
    # 3. 修改 _run_controller_diagnostics 方法
    # ========================================================================
    method_modified = False
    for i, line in enumerate(modified_lines):
        if 'def _run_controller_diagnostics(self):' in line:
            # 找到方法结束位置
            method_start = i
            indent_level = len(line) - len(line.lstrip())
            method_end = method_start + 1
            
            while method_end < len(modified_lines):
                next_line = modified_lines[method_end]
                if next_line.strip() and not next_line.startswith(' ' * (indent_level + 4)):
                    break
                method_end += 1
            
            # 在 "收集控制器诊断" 之后添加增强分析器初始化
            for j in range(method_start, method_end):
                if '收集控制器诊断' in modified_lines[j]:
                    insert_pos = j - 1
                    enhanced_init = """
    # 初始化增强诊断分析器
    if ENHANCED_DIAGNOSTICS_AVAILABLE:
        self.enhanced_analyzer = EnhancedDiagnostics(window_size=200)
        self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 增强诊断分析器已启用")
    
"""
                    modified_lines.insert(insert_pos, enhanced_init)
                    modifications.append(f"第 {insert_pos} 行: 添加增强分析器初始化代码")
                    method_modified = True
                    break
            
            # 在数据收集循环中添加实时分析
            for j in range(method_start, method_end + 10):  # +10 因为我们插入了代码
                if 'time.sleep(self.args.duration)' in modified_lines[j]:
                    # 替换为循环收集
                    modified_lines[j] = """    # 收集数据并实时分析
    start_time = time.time()
    while time.time() - start_time < self.args.duration:
        time.sleep(0.1)
        
        # 如果有增强分析器，实时添加样本
        if self.enhanced_analyzer and self.diag_monitor.diagnostics:
            latest_diag = self.diag_monitor.diagnostics[-1]
            diag_dict = {
                'cmd_vx': latest_diag.cmd_vx,
                'cmd_vy': latest_diag.cmd_vy,
                'cmd_omega': latest_diag.cmd_omega,
                'tracking_lateral_error': latest_diag.tracking_lateral_error,
                'tracking_heading_error': latest_diag.tracking_heading_error,
                'alpha': latest_diag.alpha,
                'state': latest_diag.state,
                'mpc_success': latest_diag.mpc_success
            }
            self.enhanced_analyzer.add_sample(diag_dict)
    
"""
                    modifications.append(f"第 {j} 行: 修改为循环收集并实时分析")
                    break
            
            # 在获取统计信息后添加增强分析
            for j in range(method_start, method_end + 30):
                if 'controller_stats = self.diag_monitor.get_stats()' in modified_lines[j]:
                    # 找到 if controller_stats: 块
                    for k in range(j, min(j + 10, len(modified_lines))):
                        if 'self.results[\'controller\'] = controller_stats' in modified_lines[k]:
                            insert_pos = k + 2  # 在 _log 之后
                            enhanced_analysis = """        
        # 运行增强分析
        if self.enhanced_analyzer:
            self._log(f"\\n  {Colors.CYAN}运行增强诊断分析...{Colors.NC}")
            # 注意: analyze_control_smoothness 已合并到 analyze_mpc_weights 中
            self.results['enhanced_diagnostics'] = {
                'mpc_weights': self.enhanced_analyzer.analyze_mpc_weights(),
                'consistency_check': self.enhanced_analyzer.analyze_consistency_check(),
                'state_machine': self.enhanced_analyzer.analyze_state_machine()
            }
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 增强诊断分析完成")
"""
                            modified_lines.insert(insert_pos, enhanced_analysis)
                            modifications.append(f"第 {insert_pos} 行: 添加增强分析执行代码")
                            break
                    break
            
            break
    
    if not method_modified:
        print("⚠️  警告: 无法完全修改 _run_controller_diagnostics 方法")
    
    # ========================================================================
    # 4. 在 _show_tuning_results 方法中添加增强诊断显示
    # ========================================================================
    show_added = False
    for i, line in enumerate(modified_lines):
        if '运行时调优建议:' in line:
            # 找到这个部分的结束
            insert_pos = i
            while insert_pos < len(modified_lines) and '生成配置文件' not in modified_lines[insert_pos]:
                insert_pos += 1
            
            enhanced_display = """
        # 增强诊断结果
        if 'enhanced_diagnostics' in self.results and self.enhanced_analyzer:
            self._log(f"\\n{Colors.CYAN}增强诊断分析:{Colors.NC}")
            
            # 显示完整报告
            report = self.enhanced_analyzer.generate_report()
            for line in report.split('\\n'):
                self._log(line)
            
            # 收集所有高优先级建议
            all_suggestions = self.enhanced_analyzer.get_all_suggestions()
            high_priority_suggestions = [s for s in all_suggestions if s['priority'] in ['critical', 'high']]
            
            if high_priority_suggestions:
                self._log(f"\\n{Colors.RED}⚠️  高优先级配置建议:{Colors.NC}")
                for idx, sug in enumerate(high_priority_suggestions, 1):
                    self._log(f"  {idx}. {sug['parameter']}")
                    self._log(f"     问题: {sug['current_issue']}")
                    self._log(f"     建议: {sug['suggestion']}")
        
"""
            modified_lines.insert(insert_pos, enhanced_display)
            modifications.append(f"第 {insert_pos} 行: 添加增强诊断显示代码")
            show_added = True
            break
    
    if not show_added:
        print("⚠️  警告: 无法找到 _show_tuning_results 方法中的合适位置")
    
    # ========================================================================
    # 保存修改后的文件
    # ========================================================================
    print(f"\n📝 应用的修改:")
    for mod in modifications:
        print(f"  ✅ {mod}")
    
    print(f"\n💾 保存修改到: {filepath}")
    with open(filepath, 'w', encoding='utf-8') as f:
        f.writelines(modified_lines)
    
    return True


def main():
    """主函数"""
    print("="*70)
    print("  增强诊断补丁应用工具")
    print("="*70)
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    target_file = os.path.join(script_dir, 'unified_diagnostics.py')
    
    if not os.path.exists(target_file):
        print(f"❌ 错误: 找不到文件 {target_file}")
        sys.exit(1)
    
    print(f"\n目标文件: {target_file}")
    
    # 备份原文件
    backup_path = backup_file(target_file)
    
    # 应用补丁
    try:
        success = apply_patch(target_file)
        
        if success:
            print("\n" + "="*70)
            print("  ✅ 增强诊断补丁应用成功!")
            print("="*70)
            print("\n下一步:")
            print("  1. 确保 enhanced_diagnostics.py 在同一目录")
            print("  2. 运行增强诊断:")
            print("     rosrun controller_ros unified_diagnostics.py --mode tuning --runtime-tuning --duration 60")
            print("\n如果需要恢复原文件:")
            print(f"  cp {backup_path} {target_file}")
        else:
            print("\n⚠️  补丁应用失败或已经应用过")
            print(f"如需重新应用，请先恢复备份文件:")
            print(f"  cp {backup_path} {target_file}")
    
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        print(f"\n恢复备份文件:")
        print(f"  cp {backup_path} {target_file}")
        sys.exit(1)


if __name__ == '__main__':
    main()
