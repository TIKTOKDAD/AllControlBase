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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('controller_ros')
    
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
    
    # 配置文件路径
    # 配置加载顺序 (从低到高):
    # 1. base/controller_params.yaml  - ROS 层基础配置 (话题、时钟)
    # 2. base/internal_params.yaml    - 内部实现参数 (用户不应修改)
    # 3. platforms/{platform}.yaml    - 平台特定配置 (用户可调参数)
    base_config = PathJoinSubstitution([
        pkg_share, 'config', 'base', 'controller_params.yaml'
    ])
    
    internal_config = PathJoinSubstitution([
        pkg_share, 'config', 'base', 'internal_params.yaml'
    ])
    
    # 注意: ROS2 中动态平台配置需要通过 LaunchConfiguration 处理
    # 这里使用 base_config 和 internal_config，平台配置通过参数覆盖
    
    # 控制器节点
    controller_node = Node(
        package='controller_ros',
        executable='controller_node.py',
        name='universal_controller_node',
        output='screen',
        parameters=[
            base_config,
            internal_config,
            {
                'system.platform': LaunchConfiguration('platform'),
                'system.ctrl_freq': LaunchConfiguration('ctrl_freq'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
    
    return LaunchDescription([
        platform_arg,
        use_sim_time_arg,
        ctrl_freq_arg,
        dashboard_arg,
        controller_node,
        dashboard_node,
    ])
