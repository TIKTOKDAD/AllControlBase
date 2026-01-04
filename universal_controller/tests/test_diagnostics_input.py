"""DiagnosticsInput 数据类测试"""
import pytest
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.core.enums import ControllerState
from universal_controller.core.data_types import MPCHealthStatus


def test_diagnostics_input_defaults():
    """测试 DiagnosticsInput 默认值"""
    diag = DiagnosticsInput()
    
    assert diag.alpha == 1.0
    assert diag.data_valid == True
    assert diag.mpc_health is None
    assert diag.mpc_success == False
    assert diag.odom_timeout == False
    assert diag.traj_timeout_exceeded == False
    assert diag.v_horizontal == 0.0
    assert diag.vz == 0.0
    assert diag.has_valid_data == False
    assert diag.tf2_critical == False
    assert diag.safety_failed == False
    assert diag.current_state == ControllerState.INIT
    
    print("[PASS] test_diagnostics_input_defaults passed")


def test_diagnostics_input_to_dict():
    """测试 DiagnosticsInput.to_dict()"""
    mpc_health = MPCHealthStatus(
        healthy=True, degradation_warning=False, can_recover=True,
        consecutive_near_timeout=0, kkt_residual=0.001, condition_number=100.0
    )
    
    diag = DiagnosticsInput(
        alpha=0.8,
        data_valid=True,
        mpc_health=mpc_health,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        v_horizontal=1.5,
        vz=0.1,
        has_valid_data=True,
        tf2_critical=False,
        safety_failed=False,
        current_state=ControllerState.NORMAL
    )
    
    d = diag.to_dict()
    
    assert d['alpha'] == 0.8
    assert d['data_valid'] == True
    assert d['mpc_health'] == mpc_health
    assert d['mpc_success'] == True
    assert d['v_horizontal'] == 1.5
    assert d['vz'] == 0.1
    assert d['has_valid_data'] == True
    assert d['current_state'] == ControllerState.NORMAL
    
    print("[PASS] test_diagnostics_input_to_dict passed")


def test_diagnostics_input_from_dict():
    """测试 DiagnosticsInput.from_dict()"""
    data = {
        'alpha': 0.5,
        'data_valid': False,
        'mpc_health': None,
        'mpc_success': False,
        'odom_timeout': True,
        'traj_timeout_exceeded': True,
        'v_horizontal': 0.0,
        'vz': 0.0,
        'has_valid_data': False,
        'tf2_critical': True,
        'safety_failed': True,
        'current_state': ControllerState.STOPPING
    }
    
    diag = DiagnosticsInput.from_dict(data)
    
    assert diag.alpha == 0.5
    assert diag.data_valid == False
    assert diag.mpc_health is None
    assert diag.mpc_success == False
    assert diag.odom_timeout == True
    assert diag.traj_timeout_exceeded == True
    assert diag.v_horizontal == 0.0
    assert diag.vz == 0.0
    assert diag.has_valid_data == False
    assert diag.tf2_critical == True
    assert diag.safety_failed == True
    assert diag.current_state == ControllerState.STOPPING
    
    print("[PASS] test_diagnostics_input_from_dict passed")


def test_diagnostics_input_from_dict_with_defaults():
    """测试 DiagnosticsInput.from_dict() 使用默认值"""
    # 空字典应该使用默认值
    diag = DiagnosticsInput.from_dict({})
    
    assert diag.alpha == 1.0
    assert diag.data_valid == True
    assert diag.mpc_success == False
    assert diag.current_state == ControllerState.INIT
    
    # 部分字典应该使用默认值填充缺失字段
    diag2 = DiagnosticsInput.from_dict({'alpha': 0.3, 'mpc_success': True})
    
    assert diag2.alpha == 0.3
    assert diag2.mpc_success == True
    assert diag2.data_valid == True  # 默认值
    assert diag2.odom_timeout == False  # 默认值
    
    print("[PASS] test_diagnostics_input_from_dict_with_defaults passed")


def test_diagnostics_input_roundtrip():
    """测试 DiagnosticsInput to_dict/from_dict 往返转换"""
    original = DiagnosticsInput(
        alpha=0.75,
        data_valid=True,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        v_horizontal=2.0,
        vz=0.5,
        has_valid_data=True,
        tf2_critical=False,
        safety_failed=False,
        current_state=ControllerState.MPC_DEGRADED
    )
    
    # 转换为字典再转回来
    d = original.to_dict()
    restored = DiagnosticsInput.from_dict(d)
    
    assert restored.alpha == original.alpha
    assert restored.data_valid == original.data_valid
    assert restored.mpc_success == original.mpc_success
    assert restored.odom_timeout == original.odom_timeout
    assert restored.traj_timeout_exceeded == original.traj_timeout_exceeded
    assert restored.v_horizontal == original.v_horizontal
    assert restored.vz == original.vz
    assert restored.has_valid_data == original.has_valid_data
    assert restored.tf2_critical == original.tf2_critical
    assert restored.safety_failed == original.safety_failed
    assert restored.current_state == original.current_state
    
    print("[PASS] test_diagnostics_input_roundtrip passed")


if __name__ == '__main__':
    test_diagnostics_input_defaults()
    test_diagnostics_input_to_dict()
    test_diagnostics_input_from_dict()
    test_diagnostics_input_from_dict_with_defaults()
    test_diagnostics_input_roundtrip()
    print("\nAll DiagnosticsInput tests passed!")
