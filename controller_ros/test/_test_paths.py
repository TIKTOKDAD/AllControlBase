#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试路径配置模块

此模块在测试文件导入时自动设置 Python 路径，
确保可以正确导入 controller_ros 和 universal_controller 模块。

使用方法:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(__file__))
    import _test_paths  # 自动配置路径
"""
import sys
import os

# 获取当前文件所在目录 (controller_ros/test)
_current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取 controller_ros 根目录
_controller_ros_root = os.path.dirname(_current_dir)

# 获取 controller_ros/src 目录 (包含 controller_ros 包)
_controller_ros_src = os.path.join(_controller_ros_root, 'src')

# 获取项目根目录 (包含 universal_controller)
_project_root = os.path.dirname(_controller_ros_root)

# 添加路径到 sys.path
_paths_to_add = [
    _controller_ros_src,  # controller_ros.xxx
    _project_root,        # universal_controller.xxx
    _current_dir,         # fixtures 等测试辅助模块
]

for path in _paths_to_add:
    if path not in sys.path:
        sys.path.insert(0, path)

# 验证导入（可选，用于调试）
def verify_imports():
    """验证关键模块可以正确导入"""
    try:
        import universal_controller
        print(f"✓ universal_controller: {universal_controller.__file__}")
    except ImportError as e:
        print(f"✗ universal_controller: {e}")
    
    try:
        import controller_ros
        print(f"✓ controller_ros: {controller_ros.__file__}")
    except ImportError as e:
        print(f"✗ controller_ros: {e}")


if __name__ == '__main__':
    print("Test paths configured:")
    for p in _paths_to_add:
        print(f"  - {p}")
    print()
    verify_imports()
