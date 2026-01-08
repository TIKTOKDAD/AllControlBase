"""
数据源接口 - 统一数据管理
(重构版本 - 逻辑分离到 logic 包)

从 ControllerManager 获取数据并转换为统一的 DisplayData 模型。
所有数据处理逻辑集中在此（代理给 builder 和 statistics），面板只负责显示。

注意：此模块仅用于直接访问 ControllerManager 的场景。
ROS 模式下请使用 ros_data_source.py 中的 ROSDashboardDataSource。
"""

from typing import Dict, Any, Optional

from .models import DisplayData
from .logic.constants import PLATFORM_NAMES, STATE_INFO
from .logic.statistics import StatisticsManager
from .logic.builders import StatusBuilder

# 尝试导入 ROS 相关模块 (保留兼容性)
try:
    from ..core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE
except ImportError:
    ROS_AVAILABLE = False
    TF2_AVAILABLE = False

# 尝试检测 ACADOS (保留兼容性)
try:
    from acados_template import AcadosOcp
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False


class DashboardDataSource:
    """
    Dashboard 数据源 - 统一数据管理
    
    职责：
    1. 从 ControllerManager 获取原始数据
    2. 转换为统一的 DisplayData 模型
    3. 处理所有数据逻辑 (通过代理类实现)
    4. 维护历史数据和统计信息 (通过 StatisticsManager 实现)
    
    使用场景：
    - 直接嵌入到使用 ControllerManager 的应用中
    - 需要传入有效的 controller_manager 实例
    
    ROS 模式：
    - 请使用 ROSDashboardDataSource (ros_data_source.py)
    """

    def __init__(self, controller_manager=None, config: Dict[str, Any] = None):
        """
        初始化数据源
        
        Args:
            controller_manager: ControllerManager 实例（必需）
            config: 配置字典
        """
        self.manager = controller_manager
        self.config = config or {}

        # 初始化代理组件
        self.statistics_manager = StatisticsManager()
        self.status_builder = StatusBuilder(self.config, self.manager)
        
        self._cached_data: Optional[DisplayData] = None

    def get_display_data(self) -> DisplayData:
        """
        获取统一的显示数据
        
        这是面板获取数据的唯一入口，所有数据处理逻辑集中在此。
        当没有真实数据时，返回带有 availability 标记的空数据，
        而不是生成模拟数据。
        """
        # 获取原始数据
        raw_diagnostics = self._get_raw_diagnostics()
        
        # 更新统计
        self.statistics_manager.update(raw_diagnostics)

        # 构建统一数据模型
        data = DisplayData()
        
        # 使用 StatusBuilder 构建各个状态
        data.availability = self.status_builder.build_availability(raw_diagnostics)
        data.environment = self.status_builder.build_environment_status()
        data.platform = self.status_builder.build_platform_config()
        data.controller = self.status_builder.build_controller_status(raw_diagnostics)
        data.mpc_health = self.status_builder.build_mpc_health(raw_diagnostics)
        data.consistency = self.status_builder.build_consistency(raw_diagnostics)
        data.timeout = self.status_builder.build_timeout_status(raw_diagnostics)
        data.tracking = self.status_builder.build_tracking_status(raw_diagnostics)
        data.estimator = self.status_builder.build_estimator_status(raw_diagnostics)
        data.transform = self.status_builder.build_transform_status(raw_diagnostics)
        data.safety = self.status_builder.build_safety_status(raw_diagnostics)
        data.command = self.status_builder.build_control_command(raw_diagnostics)
        data.trajectory = self.status_builder.build_trajectory_data()
        
        # 从 StatisticsManager 获取统计数据
        data.statistics = self.statistics_manager.get_statistics_data()

        # 元信息 - 从 __version__ 获取版本号
        from .. import __version__
        data.version = f'v{__version__}'
        data.transition_progress = raw_diagnostics.get('transition_progress', 1.0)

        self._cached_data = data
        return data

    def _get_raw_diagnostics(self) -> Dict[str, Any]:
        """获取原始诊断数据"""
        if self.manager:
            diag = self.manager.get_last_published_diagnostics()
            if diag is None:
                return {}
            if isinstance(diag, dict):
                return diag
            if hasattr(diag, 'to_dict'):
                return diag.to_dict()
            if hasattr(diag, 'to_ros_msg'):
                return diag.to_ros_msg()
            return {}
        
        # 没有 manager 时返回空数据
        return {}

    def get_history(self) -> Dict[str, Any]:
        """获取历史数据 (用于曲线图)"""
        return self.statistics_manager.get_history()

    # 为了保持向后兼容性或测试兼容性，如果需要访问内部历史 buffer:
    @property
    def _solve_time_history(self):
        return self.statistics_manager._solve_time_history
        
    @property
    def _lateral_error_history(self):
        return self.statistics_manager._lateral_error_history
        
    @property
    def _alpha_history(self):
        return self.statistics_manager._alpha_history
    
    @property
    def _cycle_times(self):
        return self.statistics_manager._cycle_times
