"""
Standalone implementation of tf2_ros module wrapper.
"""
from .tf2_buffer import StandaloneTF2Buffer
from .exceptions import LookupException, ExtrapolationException, ConnectivityException

class StandaloneTF2Ros:
    """tf2_ros 模块的独立运行替代实现"""
    Buffer = StandaloneTF2Buffer
    TransformListener = lambda self, buffer: None
    TransformBroadcaster = lambda: None
    StaticTransformBroadcaster = lambda: None
    LookupException = LookupException
    ExtrapolationException = ExtrapolationException
    ConnectivityException = ConnectivityException
