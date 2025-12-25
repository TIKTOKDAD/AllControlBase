"""
可视化器 ROS 节点层
"""
# 延迟导入，避免在非 ROS 环境下导入失败
def get_visualizer_node():
    from .visualizer_node import VisualizerNode
    return VisualizerNode

__all__ = ['get_visualizer_node']
