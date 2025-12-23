#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 Noetic setup.py (用于 catkin_make)

依赖:
- universal_controller: 纯算法库，需要在 PYTHONPATH 中
  export PYTHONPATH=$PYTHONPATH:/path/to/universal_controller/..
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'controller_ros',
        'controller_ros.adapters',
        'controller_ros.bridge',
        'controller_ros.io',
        'controller_ros.node',
        'controller_ros.utils',
    ],
    package_dir={'': 'src'},
    install_requires=[
        'numpy>=1.20.0',
        'scipy>=1.7.0',
    ],
)

setup(**setup_args)
