"""
可视化模块 - TurtleBot1 运行可视化界面

模块结构:
- adapters/: 数据适配器 (ROS 消息 <-> 可视化数据)
- widgets/: UI 组件 (PyQt5)
- handlers/: 业务逻辑处理器
- node/: ROS 节点

功能:
1. 轨迹可视化 - 在相机图像或俯视图上显示轨迹
2. 速度监控 - 实时显示底盘线速度和角速度
3. 手柄控制 - Xbox 手柄控制机器人，LB 键切换控制模式

架构设计:
- 遵循 controller_ros 现有架构风格
- 使用适配器模式隔离 ROS 消息和 UI 数据
- 使用处理器模式封装业务逻辑
- 支持 ROS1 和 ROS2
"""

# 延迟导入，避免在非 GUI/ROS 环境下导入失败
def create_visualizer_node():
    """创建可视化节点的工厂函数"""
    from .node import get_visualizer_node
    return get_visualizer_node()

__all__ = [
    'create_visualizer_node',
]
