#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一路径管理器

解决 controller_ros 包的 Python 路径问题：
1. catkin devel 路径 (controller_ros.msg) - 最高优先级
2. controller_ros/src 路径 (controller_ros.node 等) - 次优先级
3. universal_controller 路径 - 最低优先级

设计原则：
- devel 路径必须在 src 路径之前，确保编译后的消息类型优先
- 使用环境变量和自动探测相结合
- 支持开发环境和部署环境

使用方法：
    # 方式 1: 在脚本开头 (推荐)
    import controller_ros._path_manager
    
    # 方式 2: 显式调用
    from controller_ros._path_manager import setup_paths
    setup_paths()
"""
import os
import sys
from typing import Optional, List, Tuple

# 模块级别的状态
_paths_configured = False
_added_paths: List[str] = []


def _get_package_root() -> str:
    """获取 controller_ros 包的根目录"""
    # 本文件位于 controller_ros/src/controller_ros/_path_manager.py
    # 包根目录是 controller_ros/
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def _find_catkin_devel_path() -> Optional[str]:
    """
    查找 catkin devel 路径
    
    搜索策略 (按优先级):
    1. 环境变量 CONTROLLER_ROS_DEVEL_PATH
    2. 从 ROS_PACKAGE_PATH 推断
    3. 从 PYTHONPATH 中查找已存在的 devel 路径
    4. 常见工作空间位置
    5. 相对于包目录向上查找
    """
    # 策略 1: 环境变量
    env_path = os.environ.get('CONTROLLER_ROS_DEVEL_PATH')
    if env_path and os.path.exists(env_path):
        return env_path
    
    # 策略 2: 从 ROS_PACKAGE_PATH 推断
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
    for path in ros_package_path.split(os.pathsep):
        if '/src/' in path or '\\src\\' in path:
            # 从 .../ws/src/pkg 推断 .../ws/devel/lib/python3/dist-packages
            ws_root = path.split('/src/')[0].split('\\src\\')[0]
            devel_path = os.path.join(ws_root, 'devel', 'lib', 'python3', 'dist-packages')
            if os.path.exists(devel_path):
                return devel_path
    
    # 策略 3: 从 PYTHONPATH 中查找
    for path in sys.path:
        if 'devel' in path and 'dist-packages' in path:
            if os.path.exists(os.path.join(path, 'controller_ros', 'msg')):
                return path
    
    # 策略 4: 常见工作空间位置
    home = os.path.expanduser('~')
    common_workspaces = [
        os.path.join(home, 'turtlebot_ws'),
        os.path.join(home, 'catkin_ws'),
        os.path.join(home, 'ros_ws'),
    ]
    
    for ws in common_workspaces:
        devel_path = os.path.join(ws, 'devel', 'lib', 'python3', 'dist-packages')
        if os.path.exists(devel_path):
            # 验证 controller_ros.msg 存在
            if os.path.exists(os.path.join(devel_path, 'controller_ros', 'msg')):
                return devel_path
    
    # 策略 5: 相对于包目录向上查找
    pkg_root = _get_package_root()
    current = pkg_root
    for _ in range(5):
        parent = os.path.dirname(current)
        if parent == current:
            break
        devel_path = os.path.join(parent, 'devel', 'lib', 'python3', 'dist-packages')
        if os.path.exists(devel_path):
            return devel_path
        current = parent
    
    return None


def _find_src_path() -> Optional[str]:
    """查找 controller_ros/src 路径"""
    pkg_root = _get_package_root()
    src_path = os.path.join(pkg_root, 'src')
    if os.path.exists(src_path):
        return os.path.abspath(src_path)
    return None


def _find_universal_controller_path() -> Optional[str]:
    """
    查找 universal_controller 模块路径
    
    Returns:
        universal_controller 所在的父目录，或 None (如果已安装)
    """
    # 检查是否已经可以导入
    try:
        import universal_controller
        return None  # 已安装，不需要添加路径
    except ImportError:
        pass
    
    # 查找相对路径 (controller_ros 和 universal_controller 同级)
    pkg_root = _get_package_root()
    possible_paths = [
        os.path.dirname(pkg_root),  # controller_ros 的父目录
        os.path.join(os.path.dirname(pkg_root), 'src'),  # 工作空间 src 目录
    ]
    
    for base_path in possible_paths:
        uc_path = os.path.join(base_path, 'universal_controller')
        if os.path.exists(uc_path) and os.path.isdir(uc_path):
            # 验证是有效的 Python 包
            if os.path.exists(os.path.join(uc_path, '__init__.py')):
                return os.path.abspath(base_path)
    
    return None


def setup_paths(verbose: bool = False) -> List[str]:
    """
    设置 controller_ros 所需的 Python 路径
    
    路径优先级 (从高到低):
    1. catkin devel 路径 - controller_ros.msg 等编译生成的模块
    2. controller_ros/src 路径 - controller_ros.node 等源码模块
    3. universal_controller 路径 - 核心算法库
    
    Args:
        verbose: 是否打印调试信息
    
    Returns:
        添加的路径列表
    """
    global _paths_configured, _added_paths
    
    if _paths_configured:
        if verbose:
            print(f"[PathManager] Already configured, paths: {_added_paths}")
        return _added_paths
    
    added = []
    
    # 1. 添加 catkin devel 路径 (最高优先级)
    devel_path = _find_catkin_devel_path()
    if devel_path:
        if devel_path not in sys.path:
            sys.path.insert(0, devel_path)
            added.append(('devel', devel_path))
        if verbose:
            print(f"[PathManager] Added devel: {devel_path}")
    elif verbose:
        print("[PathManager] Warning: catkin devel path not found")
    
    # 2. 添加 src 路径 (添加到 devel 之后)
    src_path = _find_src_path()
    if src_path:
        if src_path not in sys.path:
            # 找到 devel 路径的位置，插入到其后面
            insert_pos = 1 if devel_path else 0
            # 但如果 devel 不在 sys.path 中，就插入到开头
            if devel_path and devel_path in sys.path:
                insert_pos = sys.path.index(devel_path) + 1
            sys.path.insert(insert_pos, src_path)
            added.append(('src', src_path))
        if verbose:
            print(f"[PathManager] Added src: {src_path}")
    
    # 3. 添加 universal_controller 路径 (最低优先级)
    uc_path = _find_universal_controller_path()
    if uc_path:
        if uc_path not in sys.path:
            sys.path.append(uc_path)  # 添加到末尾
            added.append(('universal_controller', uc_path))
        if verbose:
            print(f"[PathManager] Added universal_controller: {uc_path}")
    
    _paths_configured = True
    _added_paths = [p for _, p in added]
    
    return _added_paths


def check_imports() -> Tuple[bool, dict]:
    """
    检查必要的导入是否可用
    
    Returns:
        (all_ok, status_dict)
    """
    status = {
        'rospy': False,
        'controller_ros.msg': False,
        'controller_ros.node': False,
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
        from controller_ros.node import base_node
        status['controller_ros.node'] = True
    except ImportError:
        pass
    
    try:
        import universal_controller
        status['universal_controller'] = True
    except ImportError:
        pass
    
    all_ok = all(status.values())
    return all_ok, status


def diagnose(verbose: bool = True) -> dict:
    """
    运行完整诊断
    
    Args:
        verbose: 是否打印结果
    
    Returns:
        诊断结果字典
    """
    result = {
        'package_root': _get_package_root(),
        'devel_path': _find_catkin_devel_path(),
        'src_path': _find_src_path(),
        'uc_path': _find_universal_controller_path(),
        'paths_configured': _paths_configured,
        'added_paths': _added_paths,
        'imports': {},
    }
    
    # 设置路径
    setup_paths(verbose=verbose)
    
    # 检查导入
    _, result['imports'] = check_imports()
    
    if verbose:
        print("\n" + "=" * 60)
        print("controller_ros 路径诊断")
        print("=" * 60)
        print(f"包根目录: {result['package_root']}")
        print(f"Devel 路径: {result['devel_path'] or '未找到'}")
        print(f"Src 路径: {result['src_path'] or '未找到'}")
        print(f"UC 路径: {result['uc_path'] or '已安装/未找到'}")
        print()
        print("导入状态:")
        for name, ok in result['imports'].items():
            symbol = "✓" if ok else "✗"
            print(f"  {symbol} {name}")
        print()
        print("sys.path (前 10 项):")
        for i, p in enumerate(sys.path[:10]):
            print(f"  [{i}] {p}")
        if len(sys.path) > 10:
            print(f"  ... ({len(sys.path) - 10} more)")
        print("=" * 60)
    
    return result


# 模块导入时自动设置路径
setup_paths()
