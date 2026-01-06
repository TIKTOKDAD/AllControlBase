"""配置工具函数"""
from typing import Dict, Any
import logging
import collections.abc

logger = logging.getLogger(__name__)

def deep_update(source: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    """
    深度递归更新字典
    
    将 overrides 中的配置合并到 source 中。
    如果有嵌套字典，则递归合并，而不是直接替换。
    
    Args:
        source: 基础配置字典 (将被原地修改，同时也作为返回值)
        overrides: 覆盖配置字典
        
    Returns:
        Dict[str, Any]: 更新后的 source 字典
    """
    for key, value in overrides.items():
        if isinstance(value, collections.abc.Mapping) and value:
            # 如果源值不是字典（例如是基本类型或 None），则初始化为空字典以便进行合并
            target = source.get(key, {})
            if not isinstance(target, collections.abc.Mapping):
                target = {}
            
            returned = deep_update(target, value)
            source[key] = returned
        else:
            source[key] = overrides[key]
    return source
