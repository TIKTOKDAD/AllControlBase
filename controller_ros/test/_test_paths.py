#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试路径设置模块

在测试文件开头导入此模块即可自动设置所有必要路径。

使用方法:
    import sys, os
    sys.path.insert(0, os.path.dirname(__file__))
    import _test_paths
"""
import sys
import os

# 获取测试目录和 src 目录
_test_dir = os.path.dirname(os.path.abspath(__file__))
_src_path = os.path.abspath(os.path.join(_test_dir, '..', 'src'))

# 添加 src 目录
if _src_path not in sys.path:
    sys.path.insert(0, _src_path)

# 添加测试目录（用于 fixtures）
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)

# 导入路径管理器（会自动设置 devel 和 universal_controller 路径）
try:
    import controller_ros._path_manager
except ImportError:
    # 如果路径管理器不可用，尝试手动设置
    pass
