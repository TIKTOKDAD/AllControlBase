#!/usr/bin/env python3
"""
Dashboard 模拟数据测试脚本

仅用于测试和开发目的，使用模拟数据运行 Dashboard。
生产环境请使用 ROS 模式：roslaunch controller_ros controller.launch dashboard:=true

用法:
    python -m universal_controller.tests.run_dashboard_mock
    
或者:
    python universal_controller/tests/run_dashboard_mock.py
"""

import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from PyQt5.QtWidgets import QApplication
from universal_controller.dashboard.main_window import DashboardWindow
from universal_controller.dashboard.models import DisplayData
from universal_controller.config.default_config import DEFAULT_CONFIG


class MockDashboardDataSource:
    """
    模拟数据源 - 仅用于测试
    
    生成模拟的诊断数据用于 Dashboard 界面测试。
    """
    
    def __init__(self, config=None):
        self.config = config or DEFAULT_CONFIG
        self._start_time = __import__('time').time()
    
    def get_display_data(self) -> DisplayData:
        """获取模拟的显示数据"""
        from universal_controller.tests.fixtures.mock_diagnostics import generate_mock_diagnostics
        from universal_controller.dashboard.models import (
            EnvironmentStatus, PlatformConfig, ControllerStatus,
            MPCHealthStatus, ConsistencyStatus, TimeoutStatus, TrackingStatus,
            EstimatorStatus, TransformStatus, ControlCommand, TrajectoryData,
            StatisticsData, SafetyStatus, ControllerStateEnum
        )
        import time
        import math
        
        # 生成模拟诊断数据
        diag = generate_mock_diagnostics(self._start_time)
        
        data = DisplayData()
        
        # 环境状态 - 标记为模拟模式
        data.environment = EnvironmentStatus(
            ros_available=False,
            tf2_available=False,
            acados_available=True,
            imu_available=False,
            is_mock_mode=True,
        )
        
        # 平台配置
        platform = self.config.get('system', {}).get('platform', 'differential')
        data.platform = PlatformConfig(
            platform=platform,
            platform_display={'differential': '差速车', 'omni': '全向车'}.get(platform, platform),
            ctrl_freq=self.config.get('system', {}).get('ctrl_freq', 50),
            mpc_horizon=self.config.get('mpc', {}).get('horizon', 20),
            mpc_horizon_degraded=self.config.get('mpc', {}).get('horizon_degraded', 10),
            mpc_dt=self.config.get('mpc', {}).get('dt', 0.02),
        )
        
        # 控制器状态
        state = diag.get('state', 0)
        state_info = {
            0: ('INIT', '初始化'), 1: ('NORMAL', '正常运行'),
            2: ('SOFT_DISABLED', 'Soft禁用'), 3: ('MPC_DEGRADED', 'MPC降级'),
            4: ('BACKUP_ACTIVE', '备用激活'), 5: ('STOPPING', '停车中'),
            6: ('STOPPED', '已停车'),
        }
        state_name, state_desc = state_info.get(state, ('UNKNOWN', '未知'))
        
        consistency = diag.get('consistency', {})
        alpha = consistency.get('alpha_soft', 0) if isinstance(consistency, dict) else 0
        
        data.controller = ControllerStatus(
            state=ControllerStateEnum(state) if 0 <= state <= 6 else ControllerStateEnum.INIT,
            state_name=state_name,
            state_desc=state_desc,
            mpc_success=diag.get('mpc_success', False),
            backup_active=diag.get('backup_active', False),
            current_controller='Backup' if diag.get('backup_active', False) else 'MPC',
            soft_head_enabled=alpha > 0.1,
            alpha_soft=alpha,
        )
        
        # MPC 健康
        health = diag.get('mpc_health', {})
        data.mpc_health = MPCHealthStatus(
            kkt_residual=health.get('kkt_residual', 0),
            condition_number=health.get('condition_number', 0),
            solve_time_ms=diag.get('mpc_solve_time_ms', 0),
            consecutive_near_timeout=health.get('consecutive_near_timeout', 0),
            degradation_warning=health.get('degradation_warning', False),
            can_recover=health.get('can_recover', True),
            healthy=diag.get('mpc_success', False),
        )
        
        # 一致性
        cons = diag.get('consistency', {})
        data.consistency = ConsistencyStatus(
            curvature=cons.get('curvature', 0),
            velocity_dir=cons.get('velocity_dir', 0),
            temporal=cons.get('temporal', 0),
            alpha_soft=cons.get('alpha_soft', 0),
            data_valid=cons.get('data_valid', True),
        )
        
        # 超时状态
        timeout = diag.get('timeout', {})
        data.timeout = TimeoutStatus(
            odom_timeout=timeout.get('odom_timeout', False),
            traj_timeout=timeout.get('traj_timeout', False),
            traj_grace_exceeded=timeout.get('traj_grace_exceeded', False),
            imu_timeout=timeout.get('imu_timeout', False),
            last_odom_age_ms=timeout.get('last_odom_age_ms', 0),
            last_traj_age_ms=timeout.get('last_traj_age_ms', 0),
            last_imu_age_ms=timeout.get('last_imu_age_ms', 0),
            in_startup_grace=timeout.get('in_startup_grace', False),
        )
        
        # 跟踪状态
        tracking = diag.get('tracking', {})
        data.tracking = TrackingStatus(
            lateral_error=tracking.get('lateral_error', 0),
            longitudinal_error=tracking.get('longitudinal_error', 0),
            heading_error=tracking.get('heading_error', 0),
            prediction_error=tracking.get('prediction_error', 0),
        )
        
        # 状态估计器
        est = diag.get('estimator_health', {})
        bias = est.get('imu_bias', [0, 0, 0])
        data.estimator = EstimatorStatus(
            covariance_norm=est.get('covariance_norm', 0),
            innovation_norm=est.get('innovation_norm', 0),
            slip_probability=est.get('slip_probability', 0),
            imu_drift_detected=est.get('imu_drift_detected', False),
            imu_bias=(bias[0] if len(bias) > 0 else 0, 
                     bias[1] if len(bias) > 1 else 0, 
                     bias[2] if len(bias) > 2 else 0),
            imu_available=False,
            ekf_enabled=True,
            slip_detection_enabled=True,
            drift_correction_enabled=True,
            heading_fallback_enabled=True,
        )
        
        # 坐标变换
        transform = diag.get('transform', {})
        data.transform = TransformStatus(
            tf2_available=False,
            fallback_active=True,
            fallback_duration_ms=transform.get('fallback_duration_ms', 0),
            accumulated_drift=transform.get('accumulated_drift', 0),
            target_frame='odom',
            output_frame='base_link',
        )
        
        # 安全状态
        cmd = diag.get('cmd', {})
        vx = cmd.get('vx', 0)
        vy = cmd.get('vy', 0)
        current_v = math.sqrt(vx ** 2 + vy ** 2)
        data.safety = SafetyStatus(
            v_max=self.config.get('constraints', {}).get('v_max', 2.0),
            omega_max=self.config.get('constraints', {}).get('omega_max', 2.0),
            a_max=self.config.get('constraints', {}).get('a_max', 1.5),
            current_v=current_v,
            current_omega=abs(cmd.get('omega', 0)),
            low_speed_protection_active=current_v < 0.1,
            safety_check_passed=True,
            emergency_stop=diag.get('emergency_stop', False),
        )
        
        # 控制命令
        data.command = ControlCommand(
            vx=cmd.get('vx', 0),
            vy=cmd.get('vy', 0),
            vz=cmd.get('vz', 0),
            omega=cmd.get('omega', 0),
            frame_id=cmd.get('frame_id', 'base_link'),
        )
        
        # 轨迹数据 - 空
        data.trajectory = TrajectoryData()
        
        # 统计数据
        elapsed = time.time() - self._start_time
        data.statistics = StatisticsData(
            elapsed_time=elapsed,
            elapsed_time_str=f'{int(elapsed//3600):02d}:{int((elapsed%3600)//60):02d}:{int(elapsed%60):02d}',
            total_cycles=int(elapsed * 50),
            actual_freq=50.0,
            avg_cycle_ms=20.0,
            max_cycle_ms=25.0,
            min_cycle_ms=18.0,
            mpc_success_rate=95.0,
            state_counts={i: 0 for i in range(7)},
            backup_switch_count=0,
            safety_limit_count=0,
            tf2_fallback_count=0,
            soft_disable_count=0,
        )
        
        data.version = 'v3.17.12 [测试模式]'
        data.transition_progress = 1.0
        
        return data
    
    def get_history(self):
        """获取历史数据"""
        return {'solve_time': [], 'lateral_error': [], 'alpha': []}


def main():
    """主函数 - 仅用于测试"""
    print("=" * 60)
    print("  Universal Controller Dashboard [测试模式]")
    print("  版本: v3.17.12")
    print("=" * 60)
    print()
    print("⚠️  警告: 当前使用模拟数据，仅用于界面测试！")
    print("    生产环境请使用: roslaunch controller_ros controller.launch dashboard:=true")
    print()
    
    app = QApplication(sys.argv)
    
    # 创建模拟数据源
    data_source = MockDashboardDataSource(config=DEFAULT_CONFIG)
    
    # 创建主窗口
    window = DashboardWindow(data_source)
    window.setWindowTitle('Universal Controller Dashboard [测试模式 - 模拟数据]')
    window.show()
    
    print("Dashboard 已启动!")
    print("按 Ctrl+C 或关闭窗口退出")
    print()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
