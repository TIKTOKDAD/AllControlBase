"""
ROS 兼容层实现 - 独立运行模式

提供 rospy, tf2_ros 等 ROS 模块的替代实现，用于非 ROS 环境下的独立运行。

重要说明:
=========
这是**生产代码**的一部分，不是测试 mock。
本文件已重构，核心逻辑移动至 ros_compat_core 包。

- StandaloneRospy: rospy 模块的替代实现
- StandaloneTF2Buffer: tf2_ros.Buffer 的替代实现
- StandaloneTF2Ros: tf2_ros 模块的替代实现
- StandaloneTFTransformations: tf.transformations 的替代实现

这些类在非 ROS 环境下提供完整的功能支持，使 universal_controller 可以独立运行。

命名约定:
=========
- "Standalone" 前缀表示这是独立运行模式的实现
- 不使用 "Mock" 前缀，因为这不是测试 mock，而是生产代码的替代实现

向后兼容:
=========
为保持向后兼容，保留 Mock* 别名，但建议使用 Standalone* 名称。
"""

# Re-export from ros_compat_core
from .ros_compat_core import (
    LookupException, 
    ExtrapolationException, 
    ConnectivityException,
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    do_transform_pose
)

# =============================================================================
# 向后兼容别名 (已弃用，建议使用 Standalone* 名称)
# =============================================================================

# 保留 Mock* 别名以保持向后兼容
MockRospy = StandaloneRospy
MockTF2BufferCore = StandaloneTF2Buffer
MockTF2Ros = StandaloneTF2Ros
MockTFTransformations = StandaloneTFTransformations
