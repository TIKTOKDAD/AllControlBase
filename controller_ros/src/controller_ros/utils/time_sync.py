"""
时间同步工具

处理数据时间戳和超时检测。

键名约定:
- 输入键名: 'odom', 'trajectory', 'imu'
- 输出超时键名: 'odom_timeout', 'trajectory_timeout', 'imu_timeout'
- 为保持向后兼容，也支持 'traj_timeout' 作为 'trajectory_timeout' 的别名
"""
from typing import Dict, Optional, List


# 特殊值：表示禁用超时检测
TIMEOUT_DISABLED = -1


class TimeSync:
    """
    时间同步工具
    
    职责:
    - 检查数据新鲜度
    - 处理超时
    
    键名约定:
    - 内部和外部统一使用 'odom', 'trajectory', 'imu'
    - 输出超时键为 'odom_timeout', 'trajectory_timeout', 'imu_timeout'
    - 为保持向后兼容，check_freshness 返回的字典同时包含 'traj_timeout' 别名
    
    特殊值:
    - 超时值为 0 或负数时，表示禁用该数据的超时检测
    """
    
    def __init__(self, max_odom_age_ms: float = 100,
                 max_traj_age_ms: float = 200,
                 max_imu_age_ms: float = 50):
        """
        初始化时间同步
        
        Args:
            max_odom_age_ms: 里程计最大年龄 (毫秒)，0 或负数表示禁用
            max_traj_age_ms: 轨迹最大年龄 (毫秒)，0 或负数表示禁用
            max_imu_age_ms: IMU 最大年龄 (毫秒)，0 或负数表示禁用
        """
        # 处理禁用情况：0 或负数表示禁用，使用 None 表示
        self._max_ages = {
            'odom': max_odom_age_ms / 1000.0 if max_odom_age_ms > 0 else None,
            'trajectory': max_traj_age_ms / 1000.0 if max_traj_age_ms > 0 else None,
            'imu': max_imu_age_ms / 1000.0 if max_imu_age_ms > 0 else None,
        }
        
        # 记录哪些数据源被禁用
        self._disabled = {
            'odom': max_odom_age_ms <= 0,
            'trajectory': max_traj_age_ms <= 0,
            'imu': max_imu_age_ms <= 0,
        }
    
    def check_freshness(self, ages: Dict[str, float]) -> Dict[str, bool]:
        """
        检查数据新鲜度
        
        Args:
            ages: 各数据的年龄 (秒)，键为 'odom', 'trajectory', 'imu'
        
        Returns:
            各数据是否超时的字典，键为 'odom_timeout', 'trajectory_timeout', 'imu_timeout'
            为保持向后兼容，也包含 'traj_timeout' 作为 'trajectory_timeout' 的别名
            
        Note:
            禁用的数据源永远不会报告超时 (返回 False)
        """
        timeouts = {}
        for key, max_age in self._max_ages.items():
            # 如果该数据源被禁用，永远不超时
            if max_age is None:
                timeouts[f'{key}_timeout'] = False
            else:
                age = ages.get(key, float('inf'))
                timeouts[f'{key}_timeout'] = age > max_age
        
        # 向后兼容：添加 'traj_timeout' 别名
        timeouts['traj_timeout'] = timeouts['trajectory_timeout']
        
        return timeouts
    
    def is_all_fresh(self, ages: Dict[str, float], 
                     required: Optional[List[str]] = None) -> bool:
        """
        检查所有必需数据是否新鲜
        
        Args:
            ages: 各数据的年龄 (秒)
            required: 必需的数据列表，默认 ['odom', 'trajectory']
        
        Returns:
            所有必需数据都新鲜返回 True
            
        Note:
            禁用的数据源会被视为新鲜 (不影响结果)
        """
        if required is None:
            required = ['odom', 'trajectory']
        
        for key in required:
            # 如果该数据源被禁用，跳过检查
            if self._disabled.get(key, False):
                continue
            
            max_age = self._max_ages.get(key)
            if max_age is None:
                continue
            
            age = ages.get(key, float('inf'))
            if age > max_age:
                return False
        return True
    
    def is_disabled(self, key: str) -> bool:
        """
        检查指定数据源的超时检测是否被禁用
        
        Args:
            key: 数据源键名 ('odom', 'trajectory', 'imu')
        
        Returns:
            是否被禁用
        """
        return self._disabled.get(key, False)
    
    def get_max_ages(self) -> Dict[str, Optional[float]]:
        """
        获取最大年龄配置
        
        Returns:
            字典，值为 None 表示该数据源被禁用
        """
        return self._max_ages.copy()
