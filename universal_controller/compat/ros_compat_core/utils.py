"""
Standalone geometry utilities.
"""
import numpy as np
from .constants import EPSILON

def do_transform_pose(pose, transform):
    """
    执行 Pose 变换（独立模式实现）
    
    Args:
        pose: 带有 position 和 orientation 的对象
        transform: 带有 translation 和 rotation 的 TransformStamped 对象
    
    Returns:
        变换后的 pose 对象
    """
    from ...core.data_types import Pose, Point3D, Quaternion
    
    # 提取变换参数
    t = transform.transform
    
    # 旋转矩阵
    q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    
    # 归一化四元数
    norm_sq = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2
    if norm_sq < 1e-10:
        # 无效四元数，使用单位四元数
        q = (0.0, 0.0, 0.0, 1.0)
    elif abs(norm_sq - 1.0) > EPSILON:
        norm = np.sqrt(norm_sq)
        q = (q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm)
    
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    
    # 变换位置
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    translation = np.array([t.translation.x, t.translation.y, t.translation.z])
    new_pos = R @ p + translation
    
    # 变换方向（四元数乘法）
    pq = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    
    # 四元数乘法: q * pq
    new_w = w*pq[3] - x*pq[0] - y*pq[1] - z*pq[2]
    new_x = w*pq[0] + x*pq[3] + y*pq[2] - z*pq[1]
    new_y = w*pq[1] - x*pq[2] + y*pq[3] + z*pq[0]
    new_z = w*pq[2] + x*pq[1] - y*pq[0] + z*pq[3]
    
    # 构建结果
    result = Pose(
        position=Point3D(x=new_pos[0], y=new_pos[1], z=new_pos[2]),
        orientation=Quaternion(x=new_x, y=new_y, z=new_z, w=new_w)
    )
    
    return result
