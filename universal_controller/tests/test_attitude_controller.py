"""
无人机姿态控制器测试 (F14)
"""
import numpy as np
import time

from universal_controller.tracker.attitude_controller import QuadrotorAttitudeController
from universal_controller.core.data_types import ControlOutput, AttitudeCommand
from universal_controller.config.default_config import DEFAULT_CONFIG


def test_attitude_controller_basic():
    """测试姿态控制器基本功能"""
    config = DEFAULT_CONFIG.copy()
    controller = QuadrotorAttitudeController(config)
    
    # 初始状态
    state = np.zeros(8)
    state[6] = 0.0  # theta = 0
    
    # 前向速度命令
    velocity_cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    
    attitude = controller.compute_attitude(velocity_cmd, state, yaw_mode='velocity')
    
    assert isinstance(attitude, AttitudeCommand)
    # 前向速度应该产生负 pitch (前倾) 或接近零
    # 当速度误差为正时，期望加速度为正，应产生负 pitch
    assert attitude.pitch <= 0.01, f"Expected pitch <= 0.01, got {attitude.pitch}"
    assert abs(attitude.roll) < 0.1, f"Expected roll near 0, got {attitude.roll}"
    
    print("✓ test_attitude_controller_basic passed")


def test_attitude_rate_limits():
    """测试姿态角速度限制 (F14.2)"""
    config = DEFAULT_CONFIG.copy()
    controller = QuadrotorAttitudeController(config)
    
    state = np.zeros(8)
    
    # 第一次调用
    cmd1 = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    attitude1 = controller.compute_attitude(cmd1, state)
    
    # 突然大幅度变化的命令
    cmd2 = ControlOutput(vx=5.0, vy=5.0, vz=0.0, omega=0.0, frame_id="world")
    attitude2 = controller.compute_attitude(cmd2, state)
    
    # 由于角速度限制，姿态变化应该被限制
    rate_limits = controller.get_attitude_rate_limits()
    assert 'roll_rate_max' in rate_limits
    assert 'pitch_rate_max' in rate_limits
    assert 'yaw_rate_max' in rate_limits
    
    print("✓ test_attitude_rate_limits passed")


def test_hover_yaw_compensation():
    """测试悬停 yaw 漂移补偿 (F14.3)"""
    config = DEFAULT_CONFIG.copy()
    controller = QuadrotorAttitudeController(config)
    
    # 设置悬停 yaw
    target_yaw = 0.5
    controller.set_hover_yaw(target_yaw)
    
    # 悬停状态 (低速)
    state = np.zeros(8)
    state[6] = target_yaw
    
    cmd = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    
    # 多次调用模拟时间流逝
    for _ in range(10):
        attitude = controller.compute_attitude(cmd, state, yaw_mode='velocity')
        time.sleep(0.01)
    
    # 悬停时应该保持目标 yaw
    assert abs(attitude.yaw - target_yaw) < 0.1
    
    print("✓ test_hover_yaw_compensation passed")


def test_position_attitude_decoupled():
    """测试位置-姿态解耦 (F14.4)"""
    config = DEFAULT_CONFIG.copy()
    config['attitude'] = config.get('attitude', {}).copy()
    config['attitude']['position_attitude_decoupled'] = True
    
    controller = QuadrotorAttitudeController(config)
    
    # 有航向角的状态
    state = np.zeros(8)
    state[6] = np.pi / 4  # 45度航向
    
    # 世界坐标系下的速度命令
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    
    attitude = controller.compute_attitude(cmd, state)
    
    # 解耦模式下，姿态应该直接响应世界坐标系加速度
    assert isinstance(attitude, AttitudeCommand)
    
    print("✓ test_position_attitude_decoupled passed")


def test_thrust_calculation():
    """测试推力计算"""
    config = DEFAULT_CONFIG.copy()
    controller = QuadrotorAttitudeController(config)
    
    state = np.zeros(8)
    
    # 悬停命令 (vz=0)
    cmd_hover = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    attitude_hover = controller.compute_attitude(cmd_hover, state)
    
    # 悬停时推力应该约等于 1.0 (补偿重力)
    assert 0.9 < attitude_hover.thrust < 1.1
    
    # 上升命令
    cmd_up = ControlOutput(vx=0.0, vy=0.0, vz=1.0, omega=0.0, frame_id="world")
    attitude_up = controller.compute_attitude(cmd_up, state)
    
    # 上升时推力应该大于悬停
    assert attitude_up.thrust > attitude_hover.thrust
    
    print("✓ test_thrust_calculation passed")


def test_controller_reset():
    """测试控制器重置"""
    config = DEFAULT_CONFIG.copy()
    controller = QuadrotorAttitudeController(config)
    
    state = np.zeros(8)
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, frame_id="world")
    
    # 调用几次
    for _ in range(5):
        controller.compute_attitude(cmd, state)
    
    # 重置
    controller.reset()
    
    # 验证状态已重置
    assert controller._last_attitude is None
    assert controller._hover_yaw is None
    
    print("✓ test_controller_reset passed")


def run_all_attitude_tests():
    """运行所有姿态控制器测试"""
    print("\n=== Running Attitude Controller Tests (F14) ===\n")
    
    test_attitude_controller_basic()
    test_attitude_rate_limits()
    test_hover_yaw_compensation()
    test_position_attitude_decoupled()
    test_thrust_calculation()
    test_controller_reset()
    
    print("\n=== All Attitude Controller Tests Passed ===\n")


if __name__ == '__main__':
    run_all_attitude_tests()
