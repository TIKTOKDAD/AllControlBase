#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路径设置入口 (供 scripts/ 目录下的脚本使用)

使用方法:
    在脚本开头添加:
    
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    from setup_paths import setup_controller_ros_paths
    setup_controller_ros_paths()

或者更简洁的方式:
    
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
    import controller_ros._path_manager
"""
import os
import sys
from typing import List

# 添加 src 目录到路径，以便导入 _path_manager
_script_dir = os.path.dirname(os.path.abspath(__file__))
_src_path = os.path.abspath(os.path.join(_script_dir, '..', 'src'))
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)


def setup_controller_ros_paths(verbose: bool = False) -> List[str]:
    """
    设置 controller_ros 所需的 Python 路径
    
    这是对 controller_ros._path_manager.setup_paths() 的包装，
    提供向后兼容的接口。
    
    Args:
        verbose: 是否打印调试信息
    
    Returns:
        添加的路径列表
    """
    try:
        from controller_ros._path_manager import setup_paths
        return setup_paths(verbose=verbose)
    except ImportError:
        # 如果无法导入 _path_manager，使用内联的简化版本
        return _setup_paths_fallback(verbose)


def _setup_paths_fallback(verbose: bool = False) -> List[str]:
    """
    备用路径设置 (当 _path_manager 不可用时)
    """
    added = []
    
    # 1. 查找 devel 路径
    devel_path = _find_devel_path()
    if devel_path and devel_path not in sys.path:
        sys.path.insert(0, devel_path)
        added.append(devel_path)
        if verbose:
            print(f"[setup_paths] Added devel: {devel_path}")
    
    # 2. src 路径已经在模块级别添加了
    if _src_path not in sys.path:
        sys.path.append(_src_path)  # 添加到末尾，确保 devel 优先
        added.append(_src_path)
        if verbose:
            print(f"[setup_paths] Added src: {_src_path}")
    
    # 3. 查找 universal_controller
    uc_path = _find_universal_controller_path()
    if uc_path and uc_path not in sys.path:
        sys.path.append(uc_path)
        added.append(uc_path)
        if verbose:
            print(f"[setup_paths] Added universal_controller: {uc_path}")
    
    return added


def _find_devel_path():
    """查找 catkin devel 路径"""
    # 从环境变量
    env_path = os.environ.get('CONTROLLER_ROS_DEVEL_PATH')
    if env_path and os.path.exists(env_path):
        return env_path
    
    # 从 ROS_PACKAGE_PATH 推断
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
    for path in ros_package_path.split(os.pathsep):
        if '/src/' in path:
            ws_root = path.split('/src/')[0]
            devel_path = os.path.join(ws_root, 'devel', 'lib', 'python3', 'dist-packages')
            if os.path.exists(devel_path):
                return devel_path
    
    # 常见位置
    home = os.path.expanduser('~')
    for ws in ['turtlebot_ws', 'catkin_ws', 'ros_ws']:
        devel_path = os.path.join(home, ws, 'devel', 'lib', 'python3', 'dist-packages')
        if os.path.exists(devel_path):
            return devel_path
    
    return None


def _find_universal_controller_path():
    """查找 universal_controller 路径"""
    try:
        import universal_controller
        return None
    except ImportError:
        pass
    
    # 相对于 controller_ros 查找
    pkg_root = os.path.abspath(os.path.join(_script_dir, '..'))
    parent = os.path.dirname(pkg_root)
    uc_path = os.path.join(parent, 'universal_controller')
    if os.path.exists(uc_path):
        return parent
    
    return None


def check_imports(verbose: bool = True) -> dict:
    """检查必要的导入是否可用"""
    try:
        from controller_ros._path_manager import check_imports as _check
        _, status = _check()
        if verbose:
            print("[setup_paths] Import status:")
            for name, ok in status.items():
                symbol = "✓" if ok else "✗"
                print(f"  {symbol} {name}")
        return status
    except ImportError:
        # 简化版本
        status = {}
        for name in ['rospy', 'controller_ros.msg', 'universal_controller']:
            try:
                __import__(name.replace('.', '_') if '.' in name else name)
                status[name] = True
            except ImportError:
                status[name] = False
        if verbose:
            for name, ok in status.items():
                print(f"  {'✓' if ok else '✗'} {name}")
        return status


# 如果直接运行此脚本，执行诊断
if __name__ == '__main__':
    print("=" * 60)
    print("controller_ros 路径设置诊断")
    print("=" * 60)
    print()
    
    print("1. 设置路径...")
    paths = setup_controller_ros_paths(verbose=True)
    print()
    
    print("2. 检查导入...")
    check_imports(verbose=True)
    print()
    
    print("3. 当前 sys.path (前 10 项):")
    for i, p in enumerate(sys.path[:10]):
        print(f"  [{i}] {p}")
    if len(sys.path) > 10:
        print(f"  ... ({len(sys.path) - 10} more)")
