"""
TF2 Exception types for Standalone ROS compatibility.
"""

class LookupException(Exception):
    """TF2 查找异常 - 找不到指定的坐标变换"""
    pass


class ExtrapolationException(Exception):
    """TF2 外推异常 - 请求的时间超出缓存范围"""
    pass


class ConnectivityException(Exception):
    """TF2 连接异常 - 坐标系之间没有连接路径"""
    pass
