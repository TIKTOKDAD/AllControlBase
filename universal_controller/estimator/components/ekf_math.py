from typing import Tuple, List, Optional
import numpy as np
from ...core.ros_compat import normalize_angle
from ...core.indices import StateIdx

class EKFMathMixin:
    """
    EKF Math Mixin containing generic Kalman Filter operations.
    Expected attributes on self:
        x (np.ndarray): State vector
        P (np.ndarray): Covariance matrix
        min_eigenvalue (float): Minimum eigenvalue for covariance repair
        initial_covariance (float): Fallback covariance value
    """

    def _kalman_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray, 
                       angle_indices: list = None,
                       buffers: tuple = None,
                       y_buffer: np.ndarray = None) -> None:
        """
        卡尔曼更新
        
        Args:
            z: 观测向量
            H: 观测矩阵
            R: 测量噪声矩阵
            angle_indices: 观测向量中角度元素的索引列表，用于角度归一化
            buffers: 预分配的 Buffer 元组，用于性能优化
            y_buffer: 创新 (Innovation) 向量 y 的 buffer，避免 H @ x 分配
        """
        if y_buffer is not None:
            # Zero-alloc impl
            # 1. Hx = H @ x (Store in y_buffer temporarily)
            np.matmul(H, self.x, out=y_buffer)
            # 2. y = z - Hx (Store result in y_buffer)
            np.subtract(z, y_buffer, out=y_buffer)
            y = y_buffer
        else:
            # Fallback
            y = z - H @ self.x 
        
        # 对角度元素进行归一化
        if angle_indices:
            for idx in angle_indices:
                y[idx] = normalize_angle(y[idx])
        
        self._apply_kalman_gain(y, H, R, buffers)
    
    def _kalman_update_with_expected(self, z: np.ndarray, z_expected: np.ndarray,
                                     H: np.ndarray, R: np.ndarray,
                                     buffers: tuple = None,
                                     y_buffer: np.ndarray = None):
        if y_buffer is not None:
            np.subtract(z, z_expected, out=y_buffer)
            y = y_buffer
        else:
            y = z - z_expected
             
        self._apply_kalman_gain(y, H, R, buffers)
    
    def _apply_kalman_gain(self, y: np.ndarray, H: np.ndarray, R: np.ndarray,
                          buffers: tuple = None) -> None:
        """应用卡尔曼增益 (零内存分配版)"""
        
        if buffers is None:
            # Fallback for simplicity or legacy calls
            self._apply_kalman_gain_legacy(y, H, R)
            return

        # 解包 Buffers
        # buffers=(temp_H_P, temp_H_P_HT, temp_K, temp_K_y, temp_PHt, temp_K_S, temp_P_update)
        # 注意: 传入的 buffers 需要根据 H 的维度匹配
        (t_HP, t_HPHT, t_K, t_Ky, t_PHt, t_KS, t_P_upd) = buffers
        
        # 1. S = H @ P @ H.T + R
        # Step 1.1: t_HP = H @ P
        np.matmul(H, self.P, out=t_HP)
        # Step 1.2: t_HPHT = t_HP @ H.T
        np.matmul(t_HP, H.T, out=t_HPHT)
        # Step 1.3: S = t_HPHT + R (S reuse t_HPHT)
        # 注意: R 是对角/对称阵，这里直接加
        np.add(t_HPHT, R, out=t_HPHT) # Now t_HPHT holds S
        S = t_HPHT
        
        # 2. 计算 Kalman Gain K
        # K = P @ H.T @ S_inv
        
        # Step 2.1: t_PHt = P @ H.T
        # 这个其实就是 t_HP.T，因为 P 对称。但为了稳健还是乘一下
        # 优化: t_PHt = t_HP.T (Zero copy view if shape matches?) 
        # t_HP shape (M, N), t_PHt shape (N, M). 
        # 直接拿 t_HP.T 并在后续使用 solve 可能更快，但这里为了利用 buffer显式计算
        np.matmul(self.P, H.T, out=t_PHt)
        
        # Step 2.2: Compute K
        # 由于无法做 inplace solve (S_inv)，我们仍需解方程
        # K = t_PHt @ inv(S) -> K @ S = t_PHt -> K = solve(S.T, t_PHt.T).T
        # 这里的 solve 依然会分配内存，这很难避免，除非手写 Cholesky
        try:
            # 使用 Cholesky 分解 (比 inv 快)
            # L = cholesky(S)
            # K = t_PHt @ inv(S)
            # solve is: x = solve(a, b) -> ax = b
            # We want K = P H' S^-1 => K S = P H'
            # S is symmetric
            # scipy.linalg.solve(S, (P H').T, assume_a='pos').T
            # 这里用 numpy standard solve: K = solve(S, t_PHt.T).T
            # 分配不可避免: S_inv or intermediate
            K_trans = np.linalg.solve(S, t_PHt.T)
            t_K[:] = K_trans.T # Store in t_K
        except np.linalg.LinAlgError:
            # Fallback
            K_legacy = self.P @ H.T @ np.linalg.pinv(S)
            t_K[:] = K_legacy
             
        # 3. 更新状态 x = x + K @ y
        np.matmul(t_K, y, out=t_Ky)
        np.add(self.x, t_Ky, out=self.x)
        
        # 4. 更新协方差 P (Joseph Form: P = (I - KH)P(I-KH)' + KRK')
        # Simplified: P = P - K @ S @ K.T (Optimal for symmetric)
        # Step 4.1: t_KS = K @ S
        np.matmul(t_K, S, out=t_KS)
        # Step 4.2: term = t_KS @ K.T
        np.matmul(t_KS, t_K.T, out=t_P_upd)
        
        # Step 4.3: P = P - term
        np.subtract(self.P, t_P_upd, out=self.P)
        
        self._ensure_positive_definite()
        self.last_innovation_norm = np.linalg.norm(y)
        self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

    
    def _apply_kalman_gain_legacy(self, y: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        """应用卡尔曼增益 (legacy implementation)"""
        # S = H P H.T + R
        S = H @ self.P @ H.T + R
        
        try:
            # 使用 Cholesky 分解求逆更加数值稳定且快
            L = np.linalg.cholesky(S)
            # K = P H.T S^-1
            S_inv = np.linalg.solve(L.T, np.linalg.solve(L, np.eye(len(S))))
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            # 退化回伪逆
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # 更新状态 x = x + K y
        self.x = self.x + K @ y
        
        # 更新协方差 P = P - K @ S @ K.T
        term = K @ S @ K.T
        self.P = self.P - term
        
        self._ensure_positive_definite()
        self.last_innovation_norm = np.linalg.norm(y)
        
        self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

    def _ensure_positive_definite(self) -> None:
        """确保协方差矩阵正定
        
        优化策略:
        1. 首先强制对称性（使用预分配缓冲区避免临时分配）
        2. 尝试 Cholesky 分解检测正定性（比特征值分解快）
        3. 只有在 Cholesky 失败时才进行特征值分解修复
        
        这种分层策略在正常情况下避免了昂贵的特征值分解，
        只在协方差矩阵出现问题时才进行完整修复。
        """
        # 强制对称性（零分配版本）
        # 注意: self.P 和 self.P.T 是同一内存的重叠视图，
        # 直接 np.add(self.P, self.P.T, out=self.P) 会导致数据竞争
        # 因此需要使用预分配的临时缓冲区
        if hasattr(self, '_temp_P_sym'):
             np.copyto(self._temp_P_sym, self.P.T)      # 复制转置到临时缓冲区
             np.add(self.P, self._temp_P_sym, out=self.P)  # P = P + P^T
        else:
             self.P = self.P + self.P.T

        self.P *= 0.5                              # P = P / 2
        
        # 尝试 Cholesky 分解检测正定性
        # Cholesky 分解比特征值分解快约 3 倍
        try:
            np.linalg.cholesky(self.P)
            # Cholesky 成功，矩阵已经正定，无需进一步处理
            return
        except np.linalg.LinAlgError:
            # Cholesky 失败，需要修复
            pass
        
        # 使用特征值分解修复非正定矩阵
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(self.P)
            if np.any(eigenvalues < self.min_eigenvalue):
                eigenvalues = np.maximum(eigenvalues, self.min_eigenvalue)
                self.P = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        except np.linalg.LinAlgError:
            # 特征值分解也失败，重置为初始协方差
            self.P = np.eye(11) * self.initial_covariance
