#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[已废弃] 系统调优工具 - 请使用 unified_diagnostics.py

此脚本已被 unified_diagnostics.py 替代，保留此文件仅为向后兼容。
所有功能已合并到 unified_diagnostics.py 中。

使用方法:
    # 推荐: 使用统一诊断工具
    rosrun controller_ros unified_diagnostics.py --mode tuning --output config.yaml
    
    # 底盘测试
    rosrun controller_ros unified_diagnostics.py --mode tuning --test-chassis
    
    # 此脚本会自动重定向到 unified_diagnostics.py
    rosrun controller_ros full_diagnostics.py

迁移说明:
    full_diagnostics.py                    →  unified_diagnostics.py --mode tuning
    full_diagnostics.py --output x.yaml    →  unified_diagnostics.py --mode tuning --output x.yaml
    full_diagnostics.py --test-chassis     →  unified_diagnostics.py --mode tuning --test-chassis
    full_diagnostics.py --runtime-tuning   →  unified_diagnostics.py --mode tuning --runtime-tuning
    
作者: Kiro Auto-generated
版本: 2.0 (已废弃，重定向到 unified_diagnostics.py)
"""
import sys
import os
import warnings

# 发出废弃警告
warnings.warn(
    "\n" + "="*70 + "\n"
    "  [已废弃] full_diagnostics.py 已被 unified_diagnostics.py 替代\n"
    "  请使用: rosrun controller_ros unified_diagnostics.py --mode tuning\n"
    + "="*70,
    DeprecationWarning,
    stacklevel=2
)

def main():
    """重定向到 unified_diagnostics.py"""
    print("="*70)
    print("  [已废弃] full_diagnostics.py")
    print("  此脚本已被 unified_diagnostics.py 替代")
    print("="*70)
    print()
    print("正在重定向到 unified_diagnostics.py --mode tuning ...")
    print()
    
    # 获取 unified_diagnostics.py 的路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    unified_script = os.path.join(script_dir, 'unified_diagnostics.py')
    
    if not os.path.exists(unified_script):
        print(f"错误: 找不到 {unified_script}")
        print("请手动运行: rosrun controller_ros unified_diagnostics.py --mode tuning")
        return 1
    
    # 构建新的命令行参数
    new_args = [sys.executable, unified_script, '--mode', 'tuning']
    
    # 参数映射
    arg_mapping = {
        '--odom-topic': '--odom-topic',
        '--imu-topic': '--imu-topic',
        '--traj-topic': '--traj-topic',
        '--cmd-vel-topic': '--cmd-vel-topic',
        '--diag-topic': '--diag-topic',
        '--duration': '--duration',
        '--output': '--output',
        '-o': '--output',
        '--test-chassis': '--test-chassis',
        '--runtime-tuning': '--runtime-tuning',
    }
    
    # 传递原有参数
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in arg_mapping:
            new_args.append(arg_mapping[arg])
            # 检查是否有值参数
            if arg not in ['--test-chassis', '--runtime-tuning'] and i + 1 < len(sys.argv):
                i += 1
                new_args.append(sys.argv[i])
        elif arg.startswith('--'):
            # 未知参数直接传递
            new_args.append(arg)
            if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith('--'):
                i += 1
                new_args.append(sys.argv[i])
        i += 1
    
    # 执行 unified_diagnostics.py
    os.execv(sys.executable, new_args)


if __name__ == '__main__':
    sys.exit(main() or 0)
