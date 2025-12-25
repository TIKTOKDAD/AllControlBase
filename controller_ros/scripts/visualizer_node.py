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

# 先测试 ROS 消息是否可用（在修改 sys.path 之前）
try:
    from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2
    _msgs_available = True
except ImportError:
    _msgs_available = False

# 添加包路径（用于导入 visualizer 模块）
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(os.path.dirname(script_dir), 'src')
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from controller_ros.visualizer import create_visualizer_node


def load_config_from_ros():
    """从 ROS 参数服务器加载配置"""
    try:
        import rospy
        
        # 从参数服务器读取配置
        config = {
            'topics': {
                'odom': rospy.get_param('~topics/odom', rospy.get_param('topics/odom', '/odom')),
                'trajectory': rospy.get_param('~topics/trajectory', rospy.get_param('topics/trajectory', '/nn/local_trajectory')),
                'cmd_unified': rospy.get_param('~topics/cmd_unified', rospy.get_param('topics/cmd_unified', '/cmd_unified')),
                'diagnostics': rospy.get_param('~topics/diagnostics', rospy.get_param('topics/diagnostics', '/controller/diagnostics')),
                'camera_image': rospy.get_param('~topics/camera_image', rospy.get_param('topics/camera_image', '')),
                'joy': rospy.get_param('~topics/joy', rospy.get_param('topics/joy', '/joy')),
                'cmd_vel_output': rospy.get_param('~topics/cmd_vel_output', rospy.get_param('topics/cmd_vel_output', '/joy_cmd_vel')),
                'control_mode': rospy.get_param('~topics/control_mode', rospy.get_param('topics/control_mode', '/visualizer/control_mode')),
                'emergency_stop': rospy.get_param('~topics/emergency_stop', rospy.get_param('topics/emergency_stop', '/controller/emergency_stop')),
            },
            'display': {
                'update_rate': rospy.get_param('~display/update_rate', rospy.get_param('display/update_rate', 30)),
                'velocity_history_sec': rospy.get_param('~display/velocity_history_sec', rospy.get_param('display/velocity_history_sec', 10)),
            },
            'joystick': {
                'enable_button': rospy.get_param('~joystick/enable_button', rospy.get_param('joystick/enable_button', 4)),
                'linear_axis': rospy.get_param('~joystick/linear_axis', rospy.get_param('joystick/linear_axis', 1)),
                'angular_axis': rospy.get_param('~joystick/angular_axis', rospy.get_param('joystick/angular_axis', 3)),
                'max_linear': rospy.get_param('~joystick/max_linear', rospy.get_param('joystick/max_linear', 0.5)),
                'max_angular': rospy.get_param('~joystick/max_angular', rospy.get_param('joystick/max_angular', 1.0)),
                'deadzone': rospy.get_param('~joystick/deadzone', rospy.get_param('joystick/deadzone', 0.1)),
            },
            'constraints': {
                'v_max': rospy.get_param('~constraints/v_max', rospy.get_param('constraints/v_max', 0.5)),
                'omega_max': rospy.get_param('~constraints/omega_max', rospy.get_param('constraints/omega_max', 1.0)),
            },
            'camera': {
                'use_camera': rospy.get_param('~camera/use_camera', rospy.get_param('camera/use_camera', False)),
                'calibration_file': rospy.get_param('~camera/calibration_file', rospy.get_param('camera/calibration_file', '')),
            },
        }
        
        # 如果指定了相机话题，自动启用相机模式
        if config['topics']['camera_image']:
            config['camera']['use_camera'] = True
            rospy.loginfo(f"[Visualizer] Camera topic: {config['topics']['camera_image']}")
            rospy.loginfo(f"[Visualizer] Calibration file: {config['camera']['calibration_file']}")
        
        return config
        
    except ImportError:
        # ROS2 或非 ROS 环境
        return None


def main(args=None):
    """主入口"""
    try:
        # 从 ROS 参数加载配置
        config = load_config_from_ros()
        
        # 创建节点
        node_class = create_visualizer_node()
        node = node_class(config=config)
        
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
