#!/usr/bin/env python3
"""
TurtleBot1 运行可视化节点启动脚本

功能:
1. 轨迹可视化 - 在俯视图上显示网络输出的轨迹
2. 速度监控 - 实时显示底盘线速度和角速度
3. 手柄控制 - Xbox 手柄控制机器人，LB 键切换控制模式

使用方法 (ROS1):
    rosrun controller_ros visualizer_node.py

使用方法 (ROS2):
    ros2 run controller_ros visualizer_node.py
"""
import sys
import os

# 添加包路径
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(os.path.dirname(script_dir), 'src')
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from controller_ros.visualizer import create_visualizer_node


def main(args=None):
    """主入口"""
    try:
        # 创建节点
        node = create_visualizer_node()()
        
        # 启动 GUI (阻塞，直到窗口关闭)
        exit_code = node.start_gui()
        
    except KeyboardInterrupt:
        exit_code = 0
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
