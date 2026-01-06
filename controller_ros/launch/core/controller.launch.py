"""
控制器 ROS2 启动文件

启动 universal_controller 控制器节点。

使用方法:
    ros2 launch controller_ros core/controller.launch.py
    ros2 launch controller_ros core/controller.launch.py platform:=omni
    ros2 launch controller_ros core/controller.launch.py dashboard:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """动态生成节点配置，支持平台特定配置加载"""
    # 获取包路径
    pkg_share = FindPackageShare('controller_ros')

    # 解析 LaunchConfiguration 的实际值
    platform = LaunchConfiguration('platform').perform(context)
    ctrl_freq = LaunchConfiguration('ctrl_freq').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # 配置文件路径
    base_config = PathJoinSubstitution([
        pkg_share, 'config', 'base', 'controller_params.yaml'
    ])

    internal_config = PathJoinSubstitution([
        pkg_share, 'config', 'base', 'internal_params.yaml'
    ])

    # 动态加载平台特定配置
    platform_config = PathJoinSubstitution([
        pkg_share, 'config', 'platforms', f'{platform}.yaml'
    ])

    # 控制器节点 - 加载所有三层配置
    controller_node = Node(
        package='controller_ros',
        executable='controller_node.py',
        name='universal_controller_node',
        output='screen',
        parameters=[
            base_config,
            internal_config,
            platform_config,  # 添加平台特定配置
            {
                'system.platform': platform,
                'system.ctrl_freq': int(ctrl_freq),
                'use_sim_time': use_sim_time == 'true',
            }
        ],
    )

    # Dashboard 节点 (可选)
    dashboard_node = Node(
        package='controller_ros',
        executable='dashboard_node.py',
        name='controller_dashboard',
        output='screen',
        parameters=[base_config],
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return [controller_node, dashboard_node]


def generate_launch_description():
    # 声明参数
    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='differential',
        description='Platform type: differential, omni, ackermann, quadrotor'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    ctrl_freq_arg = DeclareLaunchArgument(
        'ctrl_freq',
        default_value='50',
        description='Control frequency (Hz)'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='false',
        description='Launch Dashboard GUI for monitoring'
    )

    return LaunchDescription([
        platform_arg,
        use_sim_time_arg,
        ctrl_freq_arg,
        dashboard_arg,
        OpaqueFunction(function=launch_setup),
    ])
