"""
参数加载器

从 ROS 参数服务器加载配置。
支持 ROS1 和 ROS2，以及非 ROS 环境。

配置映射说明:
- ROS 参数 (YAML) -> universal_controller 配置
- system.ctrl_freq -> system.ctrl_freq
- system.platform -> system.platform
- watchdog.* -> watchdog.*
- mpc.* -> mpc.*
- attitude.* -> attitude.*

重构说明:
- 使用策略模式统一 ROS1/ROS2 参数加载逻辑
- 参数定义集中管理，避免重复
"""
from typing import Dict, Any, Optional
from abc import ABC, abstractmethod
import logging

from universal_controller.config.default_config import DEFAULT_CONFIG
from .ros_compat import ROS_VERSION

logger = logging.getLogger(__name__)


# =============================================================================
# 参数定义 (ROS1 和 ROS2 共享)
# =============================================================================
PARAM_DEFINITIONS = {
    'system': {
        'ctrl_freq': 50,
        'platform': 'differential',
    },
    'node': {
        'use_sim_time': False,
    },
    'topics': {
        'odom': '/odom',
        'imu': '/imu',
        'trajectory': '/nn/local_trajectory',
        'cmd_unified': '/cmd_unified',
        'diagnostics': '/controller/diagnostics',
        'state': '/controller/state',
        'emergency_stop': '/controller/emergency_stop',
        'attitude_cmd': '/controller/attitude_cmd',
    },
    'tf': {
        'source_frame': 'base_link',
        'target_frame': 'odom',
        'timeout_ms': 10,
        'buffer_warmup_timeout_sec': 2.0,
        'buffer_warmup_interval_sec': 0.1,
    },
    'watchdog': {
        'odom_timeout_ms': 200,
        'traj_timeout_ms': 500,
        'imu_timeout_ms': 100,
        'startup_grace_ms': 1000,
    },
    'diagnostics': {
        'publish_rate': 5,
    },
    'mpc': {
        'horizon': 20,
        'horizon_degraded': 10,
        'dt': 0.1,
    },
    'attitude': {
        'mass': 1.5,
        'roll_max': 0.5,
        'pitch_max': 0.5,
        'hover_yaw_compensation': True,
    },
}


# =============================================================================
# 参数加载策略接口
# =============================================================================
class ParamLoaderStrategy(ABC):
    """参数加载策略基类"""
    
    @abstractmethod
    def get_param(self, group: str, key: str, default: Any) -> Any:
        """获取单个参数"""
        pass
    
    def load_group(self, group: str) -> Dict[str, Any]:
        """加载一组参数"""
        if group not in PARAM_DEFINITIONS:
            return {}
        
        result = {}
        for key, default in PARAM_DEFINITIONS[group].items():
            result[key] = self.get_param(group, key, default)
        return result
    
    def load_all(self) -> Dict[str, Any]:
        """加载所有参数"""
        params = {}
        for group in PARAM_DEFINITIONS:
            params[group] = self.load_group(group)
        return params


class ROS2ParamStrategy(ParamLoaderStrategy):
    """ROS2 参数加载策略"""
    
    def __init__(self, node):
        self._node = node
        self._declare_all_params()
    
    def _declare_all_params(self) -> None:
        """
        声明所有 ROS2 参数
        
        使用 has_parameter() 检查参数是否已声明，避免重复声明异常。
        这比使用类变量跟踪更可靠，因为：
        1. 不依赖全局状态
        2. 正确处理节点重建场景
        3. 避免内存泄漏
        """
        for group, params in PARAM_DEFINITIONS.items():
            for key, default in params.items():
                param_name = f"{group}.{key}"
                self._safe_declare(param_name, default)
    
    def _safe_declare(self, name: str, default: Any) -> None:
        """
        安全声明参数
        
        如果参数已存在则跳过，避免重复声明异常。
        """
        try:
            # 检查参数是否已声明
            if self._node.has_parameter(name):
                return
            self._node.declare_parameter(name, default)
        except Exception:
            # 忽略声明失败（可能是参数已存在或其他原因）
            pass
    
    def get_param(self, group: str, key: str, default: Any) -> Any:
        """获取 ROS2 参数"""
        param_name = f"{group}.{key}"
        return self._node.get_parameter(param_name).value


class ROS1ParamStrategy(ParamLoaderStrategy):
    """ROS1 参数加载策略"""
    
    def __init__(self):
        import rospy
        self._rospy = rospy
    
    def get_param(self, group: str, key: str, default: Any) -> Any:
        """获取 ROS1 参数"""
        # ROS1 使用 / 分隔符，特殊处理 use_sim_time
        if group == 'node' and key == 'use_sim_time':
            return self._rospy.get_param('/use_sim_time', default)
        return self._rospy.get_param(f'{group}/{key}', default)


class DefaultParamStrategy(ParamLoaderStrategy):
    """默认参数策略 (非 ROS 环境)"""
    
    def get_param(self, group: str, key: str, default: Any) -> Any:
        """返回默认值"""
        return default


# =============================================================================
# 参数加载器
# =============================================================================
class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    支持 ROS1、ROS2 和非 ROS 环境。
    """
    
    @staticmethod
    def _get_strategy(node=None) -> ParamLoaderStrategy:
        """获取参数加载策略"""
        if ROS_VERSION == 2 and node is not None:
            return ROS2ParamStrategy(node)
        elif ROS_VERSION == 1:
            return ROS1ParamStrategy()
        else:
            return DefaultParamStrategy()
    
    @staticmethod
    def load(node=None) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
        
        Returns:
            合并后的配置字典
        """
        config = ParamLoader._deep_copy(DEFAULT_CONFIG)
        
        strategy = ParamLoader._get_strategy(node)
        ros_params = strategy.load_all()
        ParamLoader._merge_params(config, ros_params)
        
        logger.info(
            f"Loaded config: platform={config.get('system', {}).get('platform', 'unknown')}, "
            f"ctrl_freq={config.get('system', {}).get('ctrl_freq', 50)}Hz"
        )
        return config
    
    @staticmethod
    def _merge_params(config: Dict[str, Any], ros_params: Dict[str, Any]):
        """
        合并 ROS 参数到配置
        
        直接映射策略:
        - system.* -> system.*
        - watchdog.* -> watchdog.*
        - mpc.* -> mpc.*
        - attitude.* -> attitude.*
        - tf.* -> transform.*
        """
        # 直接映射的组
        direct_groups = ['system', 'watchdog', 'mpc', 'attitude', 'diagnostics']
        for group in direct_groups:
            if group in ros_params:
                config.setdefault(group, {})
                for key, value in ros_params[group].items():
                    config[group][key] = value
        
        # TF 配置 -> transform 配置
        if 'tf' in ros_params:
            config.setdefault('transform', {})
            config['transform']['source_frame'] = ros_params['tf'].get('source_frame', 'base_link')
            config['transform']['target_frame'] = ros_params['tf'].get('target_frame', 'odom')
    
    @staticmethod
    def _deep_copy(d: Dict[str, Any]) -> Dict[str, Any]:
        """深拷贝字典"""
        import copy
        return copy.deepcopy(d)
    
    @staticmethod
    def get_topics(node=None) -> Dict[str, str]:
        """获取话题配置"""
        strategy = ParamLoader._get_strategy(node)
        return strategy.load_group('topics')
    
    @staticmethod
    def get_tf_config(node=None) -> Dict[str, Any]:
        """获取 TF 配置"""
        strategy = ParamLoader._get_strategy(node)
        return strategy.load_group('tf')
