"""
Standalone implementation of tf.transformations.
"""
import numpy as np

class StandaloneTFTransformations:
    """tf.transformations 模块的独立运行替代实现"""
    
    @staticmethod
    def euler_from_quaternion(q):
        """
        从四元数计算欧拉角 (roll, pitch, yaw)
        
        Args:
            q: 四元数 (x, y, z, w)
        
        Returns:
            (roll, pitch, yaw) 弧度
        """
        import numpy as np
        
        x, y, z, w = q
        
        # 归一化四元数，确保数值稳定性
        norm_sq = x*x + y*y + z*z + w*w
        if norm_sq < 1e-10:
            # 无效四元数，返回零欧拉角
            return (0.0, 0.0, 0.0)
        if abs(norm_sq - 1.0) > 1e-6:
            # 需要归一化
            norm = np.sqrt(norm_sq)
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (带数值稳定性保护)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)  # 确保在有效范围内
        if abs(sinp) >= 1.0 - 1e-9:
            # 万向节锁情况
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        从欧拉角计算四元数 (x, y, z, w)
        
        Args:
            roll, pitch, yaw: 欧拉角 (弧度)
        
        Returns:
            四元数 (x, y, z, w)
        """
        import numpy as np
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (x, y, z, w)
