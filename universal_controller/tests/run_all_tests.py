"""运行所有测试"""
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
# 添加测试目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("=" * 60)
print("Universal Controller v3.17.7 - Test Suite")
print("=" * 60)

print("\n[1/5] Running Core Tests...")
print("-" * 40)
from .test_core import *
test_control_output_copy()
test_trajectory_get_velocities_unified()
test_diagnostics_v2_to_ros_msg()
test_default_config_completeness()
test_get_config_value()
test_transform_status()
test_trajectory_copy()

print("\n[2/5] Running Component Tests...")
print("-" * 40)
from .test_components import *
test_adaptive_ekf()
test_temporal_valid_requires_two_samples()
test_state_machine_counter_reset()
test_timeout_monitor()
test_safety_monitor()
test_mpc_horizon_dynamic_adjustment()
test_pure_pursuit_shutdown()
test_mpc_health_monitor()
test_consistency_analyzer()

print("\n[3/5] Running TF2 Transform Tests (F3)...")
print("-" * 40)
from .test_tf2_transform import *
test_tf2_transform_basic()
test_tf2_fallback_degradation()
test_tf2_recovery_correction()
test_tf2_transform_with_velocity()
test_tf2_status_reporting()
test_tf2_reset()

print("\n[4/5] Running Attitude Controller Tests (F14)...")
print("-" * 40)
from .test_attitude_controller import *
test_attitude_controller_basic()
test_attitude_rate_limits()
test_hover_yaw_compensation()
test_position_attitude_decoupled()
test_thrust_calculation()
test_controller_reset()

print("\n[5/5] Running Integration Tests...")
print("-" * 40)
from .test_integration import *
test_controller_manager_initialize()
test_controller_manager_update()
test_controller_manager_with_imu()
test_diagnostics_publish()
test_component_delayed_binding()
test_state_transition()
test_reset_and_shutdown()
test_tracking_error_computation()
test_mpc_degraded_horizon()

print("\n" + "=" * 60)
print("✅ ALL TESTS PASSED - v3.17.7 Requirements Verified")
print("=" * 60)
