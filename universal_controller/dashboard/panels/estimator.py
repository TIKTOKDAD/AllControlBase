"""
状态估计面板

显示 EKF 状态估计器的健康状态和关键指标：
- 位置/航向/速度估计
- 滤波器健康（协方差范数、新息范数）
- 打滑检测
- IMU 状态和 Bias 估计
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QFrame
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS, PANEL_TITLE_STYLE


class EstimatorPanel(QGroupBox):
    """状态估计 (EKF) 面板"""
    
    def __init__(self, parent=None):
        super().__init__('状态估计 (EKF)', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 位置估计
        pos_title = QLabel('位置估计')
        pos_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(pos_title)
        
        pos_grid = QGridLayout()
        pos_grid.setSpacing(3)
        
        pos_grid.addWidget(QLabel('x:'), 0, 0)
        self.x_label = QLabel('0.00 m')
        pos_grid.addWidget(self.x_label, 0, 1)
        
        pos_grid.addWidget(QLabel('y:'), 1, 0)
        self.y_label = QLabel('0.00 m')
        pos_grid.addWidget(self.y_label, 1, 1)
        
        pos_grid.addWidget(QLabel('z:'), 2, 0)
        self.z_label = QLabel('0.00 m')
        pos_grid.addWidget(self.z_label, 2, 1)
        
        layout.addLayout(pos_grid)
        
        layout.addSpacing(5)
        
        # 航向估计
        heading_title = QLabel('航向估计')
        heading_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(heading_title)
        
        heading_grid = QGridLayout()
        heading_grid.setSpacing(3)
        
        heading_grid.addWidget(QLabel('θ:'), 0, 0)
        self.theta_label = QLabel('0.00 rad (0.0°)')
        heading_grid.addWidget(self.theta_label, 0, 1)
        
        heading_grid.addWidget(QLabel('来源:'), 1, 0)
        self.source_label = QLabel('EKF 融合')
        heading_grid.addWidget(self.source_label, 1, 1)
        
        heading_grid.addWidget(QLabel('备选:'), 2, 0)
        self.fallback_label = QLabel('○ 未使用')
        self.fallback_label.setStyleSheet('color: #B0B0B0;')
        heading_grid.addWidget(self.fallback_label, 2, 1)
        
        layout.addLayout(heading_grid)
        
        layout.addSpacing(5)
        
        # 速度估计
        vel_title = QLabel('速度估计 (世界坐标系)')
        vel_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(vel_title)
        
        vel_grid = QGridLayout()
        vel_grid.setSpacing(3)
        
        vel_grid.addWidget(QLabel('vx:'), 0, 0)
        self.vx_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vx_label, 0, 1)
        
        vel_grid.addWidget(QLabel('vy:'), 1, 0)
        self.vy_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vy_label, 1, 1)
        
        vel_grid.addWidget(QLabel('vz:'), 2, 0)
        self.vz_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vz_label, 2, 1)
        
        vel_grid.addWidget(QLabel('omega:'), 3, 0)
        self.omega_label = QLabel('0.00 rad/s')
        vel_grid.addWidget(self.omega_label, 3, 1)
        
        layout.addLayout(vel_grid)
        
        layout.addSpacing(5)
        
        # 滤波器健康
        health_title = QLabel('滤波器健康')
        health_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(health_title)
        
        # 协方差范数 - 带进度条
        cov_row = QVBoxLayout()
        cov_row.setSpacing(2)
        cov_label_row = QHBoxLayout()
        cov_label_row.addWidget(QLabel('协方差范数:'))
        self.cov_label = QLabel('0.000')
        cov_label_row.addWidget(self.cov_label)
        cov_label_row.addStretch()
        cov_row.addLayout(cov_label_row)
        self.cov_progress = ColorProgressBar(show_text=False)
        cov_row.addWidget(self.cov_progress)
        layout.addLayout(cov_row)
        
        # 新息范数 - 带进度条
        innov_row = QVBoxLayout()
        innov_row.setSpacing(2)
        innov_label_row = QHBoxLayout()
        innov_label_row.addWidget(QLabel('新息范数:'))
        self.innov_label = QLabel('0.000')
        innov_label_row.addWidget(self.innov_label)
        innov_label_row.addStretch()
        innov_row.addLayout(innov_label_row)
        self.innov_progress = ColorProgressBar(show_text=False)
        innov_row.addWidget(self.innov_progress)
        layout.addLayout(innov_row)
        
        # 滤波器状态指示
        self.cov_warning_led = StatusLED('协方差状态')
        layout.addWidget(self.cov_warning_led)
        
        self.innov_warning_led = StatusLED('新息状态')
        layout.addWidget(self.innov_warning_led)
        
        layout.addSpacing(5)
        
        # 打滑检测
        slip_title = QLabel('打滑检测')
        slip_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(slip_title)
        
        slip_row = QHBoxLayout()
        slip_row.addWidget(QLabel('打滑概率:'))
        self.slip_label = QLabel('0.0%')
        slip_row.addWidget(self.slip_label)
        slip_row.addStretch()
        layout.addLayout(slip_row)
        
        self.slip_progress = ColorProgressBar(show_text=False)
        layout.addWidget(self.slip_progress)
        
        self.high_slip_led = StatusLED('高打滑警告')
        layout.addWidget(self.high_slip_led)
        
        layout.addSpacing(5)
        
        # IMU 状态
        imu_title = QLabel('IMU 状态')
        imu_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(imu_title)
        
        self.imu_avail_led = StatusLED('IMU 可用')
        layout.addWidget(self.imu_avail_led)
        
        self.imu_drift_led = StatusLED('IMU 漂移')
        layout.addWidget(self.imu_drift_led)
        
        layout.addSpacing(5)
        
        # IMU Bias 估计
        bias_title = QLabel('IMU Bias 估计')
        bias_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(bias_title)
        
        bias_grid = QGridLayout()
        bias_grid.setSpacing(3)
        
        bias_grid.addWidget(QLabel('bias_ax:'), 0, 0)
        self.bias_ax_label = QLabel('0.00 m/s²')
        self.bias_ax_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_ax_label, 0, 1)
        
        bias_grid.addWidget(QLabel('bias_ay:'), 1, 0)
        self.bias_ay_label = QLabel('0.00 m/s²')
        self.bias_ay_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_ay_label, 1, 1)
        
        bias_grid.addWidget(QLabel('bias_az:'), 2, 0)
        self.bias_az_label = QLabel('0.00 m/s²')
        self.bias_az_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_az_label, 2, 1)
        
        layout.addLayout(bias_grid)
        
        layout.addStretch()

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.estimator_data_available:
            self._show_unavailable()
            return

        est = data.estimator
        traj = data.trajectory

        # 位置估计
        pos = traj.current_position
        self.x_label.setText(f'{pos[0]:.2f} m')
        self.y_label.setText(f'{pos[1]:.2f} m')
        self.z_label.setText(f'{pos[2]:.2f} m')

        # 航向估计
        import math
        heading = traj.current_heading
        self.theta_label.setText(f'{heading:.2f} rad ({math.degrees(heading):.1f}°)')

        # 航向来源
        if est.heading_fallback_enabled:
            self.source_label.setText('Odom 备选')
            self.fallback_label.setText('● 使用中')
            self.fallback_label.setStyleSheet(f'color: #FFC107;')
        else:
            self.source_label.setText('EKF 融合' if est.ekf_enabled else 'Odom')
            self.fallback_label.setText('○ 未使用')
            self.fallback_label.setStyleSheet('color: #B0B0B0;')

        # 速度估计
        vel = traj.current_velocity
        self.vx_label.setText(f'{vel[0]:.2f} m/s')
        self.vy_label.setText(f'{vel[1]:.2f} m/s')
        self.vz_label.setText('0.00 m/s')
        self.omega_label.setText('0.00 rad/s')

        # 滤波器健康 - 协方差范数
        cov_norm = est.covariance_norm
        self.cov_label.setText(f'{cov_norm:.4f}')
        # 协方差范数阈值：1.0 为警告阈值
        self.cov_progress.set_value(cov_norm, 1.0, 1.0)
        if est.covariance_warning:
            self.cov_label.setStyleSheet(f'color: {COLORS["warning"]}; font-weight: bold;')
            self.cov_warning_led.set_warning('⚠ 偏高')
        else:
            self.cov_label.setStyleSheet('')
            self.cov_warning_led.set_status(True, '✓ 正常')
        
        # 滤波器健康 - 新息范数
        innov_norm = est.innovation_norm
        self.innov_label.setText(f'{innov_norm:.4f}')
        # 新息范数阈值：0.5 为警告阈值
        self.innov_progress.set_value(innov_norm, 0.5, 0.5)
        if est.innovation_warning:
            self.innov_label.setStyleSheet(f'color: {COLORS["warning"]}; font-weight: bold;')
            self.innov_warning_led.set_warning('⚠ 偏高')
        else:
            self.innov_label.setStyleSheet('')
            self.innov_warning_led.set_status(True, '✓ 正常')

        # 打滑概率
        slip = est.slip_probability * 100
        self.slip_label.setText(f'{slip:.1f}%')
        self.slip_progress.set_value(slip, 100)
        
        # 高打滑警告（打滑概率 > 30%）
        if slip > 30:
            self.slip_label.setStyleSheet(f'color: {COLORS["warning"]}; font-weight: bold;')
            self.high_slip_led.set_warning('⚠ 高打滑')
        else:
            self.slip_label.setStyleSheet('')
            self.high_slip_led.set_status(True, '✓ 正常')

        # IMU 状态
        self.imu_avail_led.set_status(est.imu_available, 
                                       '✓ 是' if est.imu_available else '✗ 否')
        self.imu_drift_led.set_status(not est.imu_drift_detected, 
                                       '○ 未检测' if not est.imu_drift_detected else '⚠ 检测到')

        # IMU Bias
        bias = est.imu_bias
        self.bias_ax_label.setText(f'{bias[0]:.3f} m/s²')
        self.bias_ay_label.setText(f'{bias[1]:.3f} m/s²')
        self.bias_az_label.setText(f'{bias[2]:.3f} m/s²')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 位置显示不可用
        self.x_label.setText('无数据')
        self.x_label.setStyleSheet(unavailable_style)
        self.y_label.setText('无数据')
        self.y_label.setStyleSheet(unavailable_style)
        self.z_label.setText('无数据')
        self.z_label.setStyleSheet(unavailable_style)
        
        # 航向显示不可用
        self.theta_label.setText('无数据')
        self.theta_label.setStyleSheet(unavailable_style)
        self.source_label.setText('无数据')
        self.source_label.setStyleSheet(unavailable_style)
        self.fallback_label.setText('无数据')
        self.fallback_label.setStyleSheet(unavailable_style)
        
        # 速度显示不可用
        self.vx_label.setText('无数据')
        self.vx_label.setStyleSheet(unavailable_style)
        self.vy_label.setText('无数据')
        self.vy_label.setStyleSheet(unavailable_style)
        self.vz_label.setText('无数据')
        self.vz_label.setStyleSheet(unavailable_style)
        self.omega_label.setText('无数据')
        self.omega_label.setStyleSheet(unavailable_style)
        
        # 滤波器健康显示不可用
        self.cov_label.setText('无数据')
        self.cov_label.setStyleSheet(unavailable_style)
        self.cov_progress.set_value(0, 1.0)
        self.cov_warning_led.set_status(None, '无数据')
        
        self.innov_label.setText('无数据')
        self.innov_label.setStyleSheet(unavailable_style)
        self.innov_progress.set_value(0, 0.5)
        self.innov_warning_led.set_status(None, '无数据')
        
        # 打滑概率显示不可用
        self.slip_label.setText('无数据')
        self.slip_label.setStyleSheet(unavailable_style)
        self.slip_progress.set_value(0, 100)
        self.high_slip_led.set_status(None, '无数据')
        
        # IMU 状态显示不可用
        self.imu_avail_led.set_status(None, '无数据')
        self.imu_drift_led.set_status(None, '无数据')
        
        # IMU Bias 显示不可用
        self.bias_ax_label.setText('无数据')
        self.bias_ax_label.setStyleSheet(unavailable_style)
        self.bias_ay_label.setText('无数据')
        self.bias_ay_label.setStyleSheet(unavailable_style)
        self.bias_az_label.setText('无数据')
        self.bias_az_label.setStyleSheet(unavailable_style)
