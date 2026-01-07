"""配置验证测试"""
import pytest
from universal_controller.config.default_config import (
    DEFAULT_CONFIG, 
    validate_config, 
    ConfigValidationError,
    get_config_value
)


def test_default_config_valid():
    """测试默认配置应该通过验证"""
    errors = validate_config(DEFAULT_CONFIG, raise_on_error=False)
    assert len(errors) == 0, f"Default config has errors: {errors}"


def test_validate_config_invalid_horizon():
    """测试无效的 MPC horizon"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = -1
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('horizon' in key for key, _ in errors)


def test_validate_config_invalid_ctrl_freq():
    """测试无效的控制频率"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['ctrl_freq'] = 0
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('ctrl_freq' in key for key, _ in errors)


def test_validate_config_logical_consistency_horizon():
    """测试 horizon 逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = 10
    config['mpc']['horizon_degraded'] = 20  # 降级 horizon 大于正常 horizon
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('horizon_degraded' in key for key, _ in errors)


def test_validate_config_logical_consistency_lookahead():
    """测试前瞻距离逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['min_lookahead'] = 5.0
    config['backup']['max_lookahead'] = 2.0  # min > max
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('min_lookahead' in key for key, _ in errors)


def test_validate_config_logical_consistency_velocity():
    """测试速度约束逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['v_min'] = 5.0
    config['constraints']['v_max'] = 2.0  # min > max
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('v_min' in key for key, _ in errors)


def test_validate_config_raise_on_error():
    """测试 raise_on_error 参数"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = -1
    
    with pytest.raises(ConfigValidationError):
        validate_config(config, raise_on_error=True)


def test_validate_config_type_error():
    """测试类型错误检测"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = "invalid"  # 应该是数字
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('类型错误' in msg for _, msg in errors)


def test_get_config_value_nested():
    """测试嵌套配置值获取"""
    value = get_config_value(DEFAULT_CONFIG, 'mpc.weights.position')
    assert value == 10.0


def test_get_config_value_default():
    """测试默认值获取"""
    value = get_config_value(DEFAULT_CONFIG, 'nonexistent.key', default=42)
    assert value == 42


def test_get_config_value_from_default():
    """测试从备选配置获取缺失值"""
    config = {'system': {'ctrl_freq': 100}}  # 缺少其他配置
    # 使用 fallback_config 参数显式指定备选配置
    value = get_config_value(config, 'mpc.horizon', fallback_config=DEFAULT_CONFIG)
    assert value == 20  # 从 DEFAULT_CONFIG 获取


def test_validate_config_alpha_max_constraint():
    """测试角加速度约束验证 - alpha_max 必须大于 0"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['alpha_max'] = 0  # 无效值
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('alpha_max' in key for key, _ in errors)
    
    # 测试负值
    config['constraints']['alpha_max'] = -1.0
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('alpha_max' in key for key, _ in errors)


def test_validate_config_az_max_constraint():
    """测试垂直加速度约束验证 - az_max 必须大于 0"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['az_max'] = 0  # 无效值
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('az_max' in key for key, _ in errors)
    
    # 测试负值
    config['constraints']['az_max'] = -5.0
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('az_max' in key for key, _ in errors)


# =============================================================================
# 新增: 枚举值验证测试
# =============================================================================

def test_validate_config_enum_transition_type():
    """测试过渡类型枚举验证"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['transition'] = DEFAULT_CONFIG['transition'].copy()
    config['transition']['type'] = 'invalid_type'
    
    errors = validate_logical_consistency(config)
    assert len(errors) > 0
    assert any('transition.type' in key for key, _, _ in errors)


def test_validate_config_enum_heading_mode():
    """测试航向模式枚举验证"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['heading_mode'] = 'invalid_mode'
    
    errors = validate_logical_consistency(config)
    assert len(errors) > 0
    assert any('heading_mode' in key for key, _, _ in errors)


def test_validate_config_enum_platform():
    """测试平台类型枚举验证"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['platform'] = 'invalid_platform'
    
    errors = validate_logical_consistency(config)
    assert len(errors) > 0
    assert any('platform' in key for key, _, _ in errors)


def test_validate_config_enum_turn_direction():
    """测试转向方向枚举验证"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['default_turn_direction'] = 'up'  # 无效值
    
    errors = validate_logical_consistency(config)
    assert len(errors) > 0
    assert any('turn_direction' in key for key, _, _ in errors)


# =============================================================================
# 新增: MPC 健康监控阈值交叉验证测试
# =============================================================================

def test_validate_config_mpc_health_thresholds():
    """测试 MPC 健康监控阈值一致性"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['health_monitor'] = DEFAULT_CONFIG['mpc']['health_monitor'].copy()
    
    # 警告阈值 >= 临界阈值 (错误)
    config['mpc']['health_monitor']['time_warning_thresh_ms'] = 20
    config['mpc']['health_monitor']['time_critical_thresh_ms'] = 15
    
    errors = validate_logical_consistency(config)
    assert any('time_warning_thresh_ms' in key for key, _, _ in errors)


def test_validate_config_mpc_recovery_threshold():
    """测试 MPC 恢复阈值一致性"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['health_monitor'] = DEFAULT_CONFIG['mpc']['health_monitor'].copy()
    
    # 恢复阈值 >= 警告阈值 (错误)
    config['mpc']['health_monitor']['time_recovery_thresh_ms'] = 10
    config['mpc']['health_monitor']['time_warning_thresh_ms'] = 8
    
    errors = validate_logical_consistency(config)
    assert any('time_recovery_thresh_ms' in key for key, _, _ in errors)


# =============================================================================
# 新增: 状态机参数交叉验证测试
# =============================================================================

def test_validate_config_mpc_fail_window():
    """测试 MPC 失败检测窗口参数一致性"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['safety'] = DEFAULT_CONFIG['safety'].copy()
    config['safety']['state_machine'] = DEFAULT_CONFIG['safety']['state_machine'].copy()
    
    # 失败阈值 > 窗口大小 (错误)
    config['safety']['state_machine']['mpc_fail_window_size'] = 10
    config['safety']['state_machine']['mpc_fail_thresh'] = 15
    
    errors = validate_logical_consistency(config)
    assert any('mpc_fail_thresh' in key for key, _, _ in errors)


def test_validate_config_mpc_fail_ratio():
    """测试 MPC 失败率阈值范围"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['safety'] = DEFAULT_CONFIG['safety'].copy()
    config['safety']['state_machine'] = DEFAULT_CONFIG['safety']['state_machine'].copy()
    
    # 失败率 > 1 (错误)
    config['safety']['state_machine']['mpc_fail_ratio_thresh'] = 1.5
    
    errors = validate_logical_consistency(config)
    assert any('mpc_fail_ratio_thresh' in key for key, _, _ in errors)


def test_validate_config_mpc_recovery_tolerance():
    """测试 MPC 恢复容错参数一致性"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['safety'] = DEFAULT_CONFIG['safety'].copy()
    config['safety']['state_machine'] = DEFAULT_CONFIG['safety']['state_machine'].copy()
    
    # 容错次数 >= 最近检查次数 (错误)
    config['safety']['state_machine']['mpc_recovery_recent_count'] = 5
    config['safety']['state_machine']['mpc_recovery_tolerance'] = 5
    
    errors = validate_logical_consistency(config)
    assert any('mpc_recovery_tolerance' in key for key, _, _ in errors)


# =============================================================================
# 新增: EKF 噪声参数验证测试
# =============================================================================

def test_validate_config_ekf_measurement_noise():
    """测试 EKF 测量噪声必须为正数"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
    config['ekf']['measurement_noise'] = DEFAULT_CONFIG['ekf']['measurement_noise'].copy()
    config['ekf']['measurement_noise']['odom_position'] = -0.01  # 负值
    
    errors = validate_logical_consistency(config)
    assert any('measurement_noise' in key for key, _, _ in errors)


def test_validate_config_ekf_process_noise():
    """测试 EKF 过程噪声必须为正数"""
    from universal_controller.config.validation import validate_logical_consistency
    
    config = DEFAULT_CONFIG.copy()
    config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
    config['ekf']['process_noise'] = DEFAULT_CONFIG['ekf']['process_noise'].copy()
    config['ekf']['process_noise']['velocity'] = 0  # 零值
    
    errors = validate_logical_consistency(config)
    assert any('process_noise' in key for key, _, _ in errors)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
