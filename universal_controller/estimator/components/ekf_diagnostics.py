from typing import List
import numpy as np
import logging
from ...core.indices import StateIdx

logger = logging.getLogger(__name__)

class EKFDiagnosticsMixin:
    """
    Mixin for EKF diagnostics and anomaly detection.
    """

    def _get_adaptive_slip_threshold(self) -> float:
        current_velocity = np.linalg.norm(self.x[StateIdx.VX : StateIdx.VZ + 1])
        return self.base_slip_thresh + self.slip_velocity_factor * current_velocity
    
    def _is_stationary(self) -> bool:
        return np.linalg.norm(self.x[StateIdx.VX : StateIdx.VZ + 1]) < self.stationary_thresh
    
    def _compute_slip_probability(self, accel_diff: float) -> float:
        slip_thresh = self._get_adaptive_slip_threshold()
        k = self.slip_probability_k_factor / slip_thresh
        probability = 1.0 / (1.0 + np.exp(-k * (accel_diff - slip_thresh * 0.8)))
        self.slip_history.append(probability)
        return np.mean(self.slip_history)

    def _detect_anomalies_unlocked(self) -> List[str]:
        """检测估计器异常（内部无锁版本）

        检测以下异常情况:
        - SLIP_DETECTED: 检测到轮子打滑
        - IMU_DRIFT: IMU 漂移（静止时陀螺仪有输出）
        - ODOM_JUMP: 里程计位置跳变
        - IMU_UNAVAILABLE: IMU 数据不可用
        - COVARIANCE_EXPLOSION: 协方差矩阵发散
        - INNOVATION_ANOMALY: 测量创新度异常大

        Returns:
            异常标识列表
        """
        anomalies = []

        if self.slip_detected:
            anomalies.append("SLIP_DETECTED")

        # Fix: Check Odom freshness before using _raw_odom_twist_norm
        # If Odom is stale, we cannot rely on it to determine if we are stopped.
        # Assumption: If odom timeout, is_linear_stationary is effectively Unknown.
        # Strict logic: We only flag IMU_DRIFT if we are CONFIDENT we are stationary.
        odom_is_fresh = self._time_since_last_odom < self.accel_freshness_thresh * 5.0 # e.g. 0.5s

        is_linear_stationary = False
        if odom_is_fresh:
            is_linear_stationary = self._raw_odom_twist_norm < self.stationary_thresh
        
        is_angular_stationary = True
        if hasattr(self, '_raw_odom_angular_velocity') and odom_is_fresh:
             is_angular_stationary = abs(self._raw_odom_angular_velocity) < self.stationary_thresh
             
        is_physically_stationary = is_linear_stationary and is_angular_stationary
        
        if is_physically_stationary and abs(self.gyro_z) > self.drift_thresh:
            anomalies.append("IMU_DRIFT")
            self._imu_drift_detected = True
        else:
            self._imu_drift_detected = False

        if self.position_jump > self.jump_thresh:
            anomalies.append("ODOM_JUMP")

        if not self._imu_available:
            anomalies.append("IMU_UNAVAILABLE")

        # 协方差爆炸检测
        covariance_norm = np.linalg.norm(self.P[:8, :8])
        if covariance_norm > self.covariance_explosion_thresh:
            anomalies.append("COVARIANCE_EXPLOSION")
            logger.warning(f"Covariance explosion detected: norm={covariance_norm:.2f}")

        # 创新度异常检测
        if self.last_innovation_norm > self.innovation_anomaly_thresh:
            anomalies.append("INNOVATION_ANOMALY")

        return anomalies

    def detect_anomalies(self) -> List[str]:
        """检测估计器异常（线程安全）

        检测以下异常情况:
        - SLIP_DETECTED: 检测到轮子打滑
        - IMU_DRIFT: IMU 漂移（静止时陀螺仪有输出）
        - ODOM_JUMP: 里程计位置跳变
        - IMU_UNAVAILABLE: IMU 数据不可用
        - COVARIANCE_EXPLOSION: 协方差矩阵发散
        - INNOVATION_ANOMALY: 测量创新度异常大

        Returns:
            异常标识列表
        """
        self._acquire_lock()
        try:
            return self._detect_anomalies_unlocked()
        finally:
            self._release_lock()
