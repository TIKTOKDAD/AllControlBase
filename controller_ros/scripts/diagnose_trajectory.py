#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[已废弃] 轨迹诊断脚本 - 请使用 unified_diagnostics.py

此脚本已被 unified_diagnostics.py 替代，保留此文件仅为向后兼容。
所有功能已合并到 unified_diagnostics.py 中。

使用方法:
    # 推荐: 使用统一诊断工具
    rosrun controller_ros unified_diagnostics.py --mode realtime
    
    # 此脚本会自动重定向到 unified_diagnostics.py
    rosrun controller_ros diagnose_trajectory.py

迁移说明:
    diagnose_trajectory.py  →  unified_diagnostics.py --mode realtime
    
作者: Kiro Auto-generated
版本: 3.0 (已废弃，重定向到 unified_diagnostics.py)
"""
import sys
import os
import warnings

# 发出废弃警告
warnings.warn(
    "\n" + "="*70 + "\n"
    "  [已废弃] diagnose_trajectory.py 已被 unified_diagnostics.py 替代\n"
    "  请使用: rosrun controller_ros unified_diagnostics.py --mode realtime\n"
    + "="*70,
    DeprecationWarning,
    stacklevel=2
)

def main():
    """重定向到 unified_diagnostics.py"""
    print("="*70)
    print("  [已废弃] diagnose_trajectory.py")
    print("  此脚本已被 unified_diagnostics.py 替代")
    print("="*70)
    print()
    print("正在重定向到 unified_diagnostics.py --mode realtime ...")
    print()
    
    # 获取 unified_diagnostics.py 的路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    unified_script = os.path.join(script_dir, 'unified_diagnostics.py')
    
    if not os.path.exists(unified_script):
        print(f"错误: 找不到 {unified_script}")
        print("请手动运行: rosrun controller_ros unified_diagnostics.py --mode realtime")
        return 1
    
    # 构建新的命令行参数
    new_args = [sys.executable, unified_script, '--mode', 'realtime']
    
    # 传递原有参数（如果有）
    for arg in sys.argv[1:]:
        if arg.startswith('_'):
            # ROS 参数格式转换: _topic:=/xxx → --xxx-topic /xxx
            if ':=' in arg:
                param_name, param_value = arg[1:].split(':=', 1)
                if 'topic' in param_name:
                    new_args.extend([f'--{param_name.replace("_", "-")}', param_value])
        else:
            new_args.append(arg)
    
    # 执行 unified_diagnostics.py
    os.execv(sys.executable, new_args)


if __name__ == '__main__':
    sys.exit(main() or 0)
