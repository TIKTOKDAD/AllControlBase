#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径设置模块

统一管理 controller_ros 脚本的 Python 路径设置。
消除硬编码路径，支持多种工作空间配置。

使用方法:
    在脚本开头导入此模块:
    
    import os
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from setup_paths import setup_controller_ros_paths
    setup_controller_ros_paths()
"""
import os
import sys
from typing import Optional, List


def _find_catkin_devel_path() -> Optional[str]:
    """
    自动查找 catkin devel 路径
    
    搜索策略:
    1. 环境变量 CONTROLLER_ROS_DEVEL_PATH
    2. 环境变量 ROS_PACKAGE_PATH 中的 devel 路径
    3. 常见的 catkin 工作空间位置
    4. 相对于脚本的父目录查找
    
    Returns:
        找到的 devel 路径，或 None
    """
    # 策略 1: 环境变量
    env_path = os.environ.get('CONTROLLER_ROS_DEVEL_PATH')
    if env_path and os.path.exists(env_path):
        return env_path
    
    # 策略 2: 从 ROS_PACKAGE_PATH 推断
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
    for path in ros_package_path.split(':'):
        if '/src/' in path:
            # 从 .../ws/src/pkg 推断 .../ws/devel/lib/python3/dist-packages
            ws_root = path.split('/src/')[0]
            devel_path = os.path.join(ws_root, 'devel', 'lib', 'python3', 'dist-packages')
            if os.path.exists(devel_path):
                return devel_path
    
    # 策略 3: 常见工作空间位置
    home = os.path.expanduser('~')
    common_workspaces = [
        os.path.join(home, 'catkin_ws'),
        os.path.join(home, 'ros_ws'),
        os.path.join(home, 'turtlebot_ws'),
        '/opt/ros/noetic',
    ]
    
    for ws in common_workspaces:
        devel_path = os.path.join(ws, 'devel', 'lib', 'python3', 'dist-packages')
        if os.path.exists(devel_path):
            return devel_path
    
    # 策略 4: 相对于脚本目录查找
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 向上查找 devel 目录
    current = script_dir
    for _ in range(5):  # 最多向上 5 级
        parent = os.path.dirname(current)
        if parent == current:
            break
        devel_path = os.path.join(parent, 'devel', 'lib', 'python3', 'dist-packages')
        if os.path.exists(devel_path):
            return devel_path
        current = parent
    
    return None


def _find_src_path() -> Optional[str]:
    """
    查找 controller_ros/src 路径
    
    Returns:
        src 路径，或 None
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(script_dir, '..', 'src')
    if os.path.exists(src_path):
        return os.path.abspath(src_path)
    return None


def _find_universal_controller_path() -> Optional[str]:
    """
    查找 universal_controller 模块路径
    
    Returns:
        universal_controller 所在目录，或 None
    """
    # 尝试从已安装的包导入
    try:
        import universal_controller
        return None  # 已安装，不需要添加路径
    except ImportError:
        pass
    
    # 查找相对路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 可能的相对位置
    possible_paths = [
        os.path.join(script_dir, '..', '..'),  # controller_ros 同级
        os.path.join(script_dir, '..', '..', '..'),  # 工作空间 src 目录
        os.path.join(script_dir, '..', '..', '..', 'src'),
    ]
    
    for base_path in possible_paths:
        uc_path = os.path.join(base_path, 'universal_controller')
        if os.path.exists(uc_path) and os.path.isdir(uc_path):
            return os.path.abspath(base_path)
    
    return None


def setup_controller_ros_paths(verbose: bool = False) -> List[str]:
    """
    设置 controller_ros 所需的 Python 路径
    
    添加顺序 (优先级从高到低):
    1. catkin devel 路径 (controller_ros.msg 等)
    2. controller_ros/src 路径 (controller_ros.node 等)
    3. universal_controller 路径 (如果未安装)
    
    Args:
        verbose: 是否打印调试信息
    
    Returns:
        添加的路径列表
    """
    added_paths = []
    
    # 1. 添加 catkin devel 路径 (最高优先级，用于 ROS 消息)
    devel_path = _find_catkin_devel_path()
    if devel_path:
        if devel_path not in sys.path:
            sys.path.insert(0, devel_path)
            added_paths.append(devel_path)
        if verbose:
            print(f"[setup_paths] Added devel path: {devel_path}")
    elif verbose:
        print("[setup_paths] Warning: catkin devel path not found")
    
    # 2. 添加 src 路径
    src_path = _find_src_path()
    if src_path:
        if src_path not in sys.path:
            # 插入到 devel 之后
            insert_pos = 1 if devel_path else 0
            sys.path.insert(insert_pos, src_path)
            added_paths.append(src_path)
        if verbose:
            print(f"[setup_paths] Added src path: {src_path}")
    
    # 3. 添加 universal_controller 路径 (如果需要)
    uc_path = _find_universal_controller_path()
    if uc_path:
        if uc_path not in sys.path:
            sys.path.append(uc_path)  # 添加到末尾，优先使用已安装的版本
            added_paths.append(uc_path)
        if verbose:
            print(f"[setup_paths] Added universal_controller path: {uc_path}")
    
    return added_paths


def check_imports(verbose: bool = True) -> dict:
    """
    检查必要的导入是否可用
    
    Args:
        verbose: 是否打印结果
    
    Returns:
        导入状态字典
    """
    status = {
        'rospy': False,
        'controller_ros.msg': False,
        'universal_controller': False,
    }
    
    try:
        import rospy
        status['rospy'] = True
    except ImportError:
        pass
    
    try:
        from controller_ros.msg import LocalTrajectoryV4
        status['controller_ros.msg'] = True
    except ImportError:
        pass
    
    try:
        import universal_controller
        status['universal_controller'] = True
    except ImportError:
        pass
    
    if verbose:
        print("[setup_paths] Import status:")
        for name, available in status.items():
            symbol = "✓" if available else "✗"
            print(f"  {symbol} {name}")
    
    return status


# 如果直接运行此脚本，执行诊断
if __name__ == '__main__':
    print("=" * 50)
    print("controller_ros 路径设置诊断")
    print("=" * 50)
    print()
    
    print("1. 设置路径...")
    paths = setup_controller_ros_paths(verbose=True)
    print()
    
    print("2. 检查导入...")
    check_imports(verbose=True)
    print()
    
    print("3. 当前 sys.path:")
    for i, p in enumerate(sys.path[:10]):
        print(f"  [{i}] {p}")
    if len(sys.path) > 10:
        print(f"  ... ({len(sys.path) - 10} more)")
