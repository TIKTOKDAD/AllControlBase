"""集成测试 - 符合 v3.17.6 需求"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, Point3D, Header, Odometry, Imu
)
from universal_controller.core.enums import ControllerState
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.manager.controller_manager import ControllerManager

# 使用 mock 模块中的测试数据生成器
from universal_controller.mock.test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu
)


def test_controller_manager_initialize():
    """测试 ControllerManager 初始化"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    
    # 使用默认组件初始化
    manager.initialize_default_components()
    
    assert manager.state_estimator is not None
    assert manager.mpc_tracker is not None
    assert manager.backup_tracker is not None
    assert manager.consistency_checker is not None
    assert manager.safety_monitor is not None
    assert manager.state_machine is not None
    
    print("✓ test_controller_manager_initialize passed")


def test_controller_manager_update():
    """测试 ControllerManager 主循环"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(x=0.0, y=0.0, vx=1.0)
    trajectory = create_test_trajectory()
    
    # 执行更新
    cmd = manager.update(odom, trajectory)
    
    assert isinstance(cmd, ControlOutput)
    assert cmd.frame_id != ""
    
    print("✓ test_controller_manager_update passed")


def test_controller_manager_with_imu():
    """测试带 IMU 的更新"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(x=0.0, y=0.0, vx=1.0)
    trajectory = create_test_trajectory()
    imu = create_test_imu()
    
    # 执行更新
    cmd = manager.update(odom, trajectory, imu)
    
    assert isinstance(cmd, ControlOutput)
    
    print("✓ test_controller_manager_with_imu passed")


def test_diagnostics_publish():
    """测试诊断发布 (v3.17.6)"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom()
    trajectory = create_test_trajectory()
    
    manager.update(odom, trajectory)
    
    # 验证 _last_published_diagnostics 已设置
    assert manager._last_published_diagnostics is not None
    assert isinstance(manager._last_published_diagnostics, dict)
    assert 'state' in manager._last_published_diagnostics
    assert 'mpc_success' in manager._last_published_diagnostics
    assert 'timeout' in manager._last_published_diagnostics
    
    print("✓ test_diagnostics_publish passed")


def test_component_delayed_binding():
    """测试组件延迟绑定 (v3.17.5)"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    
    from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
    from universal_controller.transform.robust_transformer import RobustCoordinateTransformer
    
    transformer = RobustCoordinateTransformer(config)
    estimator = AdaptiveEKFEstimator(config)
    
    # 先注入 coord_transformer，再注入 state_estimator
    manager.initialize_components(coord_transformer=transformer)
    manager.initialize_components(state_estimator=estimator)
    
    # 验证 transformer 获得了 estimator 引用
    assert manager.coord_transformer.state_estimator is estimator
    
    print("✓ test_component_delayed_binding passed")


def test_state_transition():
    """测试状态转换"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 初始状态
    assert manager.get_state() == ControllerState.INIT
    
    # 发送有效数据后应转换到 NORMAL
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(3):
        manager.update(odom, trajectory)
    
    # 状态应该已经改变
    state = manager.get_state()
    assert state != ControllerState.INIT or state == ControllerState.NORMAL
    
    print("✓ test_state_transition passed")


def test_reset_and_shutdown():
    """测试重置和关闭"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    manager.update(odom, trajectory)
    
    # 重置
    manager.reset()
    assert manager.get_state() == ControllerState.INIT
    assert manager._last_mpc_cmd is None
    
    # 关闭
    manager.shutdown()
    
    print("✓ test_reset_and_shutdown passed")


def test_tracking_error_computation():
    """测试跟踪误差计算"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(x=0.5, y=0.3, vx=1.0)
    trajectory = create_test_trajectory()
    
    manager.update(odom, trajectory)
    
    # 检查跟踪误差
    assert manager._last_tracking_error is not None
    assert 'lateral_error' in manager._last_tracking_error
    assert 'longitudinal_error' in manager._last_tracking_error
    assert 'heading_error' in manager._last_tracking_error
    
    print("✓ test_tracking_error_computation passed")


def test_mpc_degraded_horizon():
    """测试 MPC 降级时 horizon 调整"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 模拟进入 MPC_DEGRADED 状态
    manager._on_state_changed(ControllerState.NORMAL, ControllerState.MPC_DEGRADED)
    
    assert manager._current_horizon == manager.horizon_degraded
    
    # 恢复
    manager._on_state_changed(ControllerState.MPC_DEGRADED, ControllerState.NORMAL)
    
    assert manager._current_horizon == manager.horizon_normal
    
    print("✓ test_mpc_degraded_horizon passed")


if __name__ == '__main__':
    test_controller_manager_initialize()
    test_controller_manager_update()
    test_controller_manager_with_imu()
    test_diagnostics_publish()
    test_component_delayed_binding()
    test_state_transition()
    test_reset_and_shutdown()
    test_tracking_error_computation()
    test_mpc_degraded_horizon()
    print("\n✅ All integration tests passed!")
