"""
验证最近修复的 Bug 的测试
"""
import numpy as np
import pytest
from universal_controller.core.data_types import Trajectory, Header
from universal_controller.safety.safety_monitor import BasicSafetyMonitor


class TestFixesVerification:
    """验证所有修复的正确性"""
    
    def test_trajectory_points_indexing(self):
        """
        验证修复 #1: Pure Pursuit TARGET_POINT 模式的索引访问
        
        之前的 Bug: 使用 end_pt.y 和 end_pt.x 访问 numpy 数组会失败
        修复后: 使用 end_pt[1] 和 end_pt[0] 索引访问
        """
        points = np.array([[0, 0, 0], [1, 2, 0], [3, 5, 0]], dtype=np.float64)
        traj = Trajectory(header=Header(), points=points, velocities=None, dt_sec=0.1)
        
        # 模拟 control_law.py:221-222 中的访问方式 (修复后)
        end_pt = traj.points[-1]
        
        # 验证这是 numpy 数组，不是 Point3D 对象
        assert isinstance(end_pt, np.ndarray), "end_pt should be numpy array"
        
        # 使用索引访问 (修复后的方式)
        x = end_pt[0]
        y = end_pt[1]
        
        assert x == 3.0, f"Expected x=3.0, got {x}"
        assert y == 5.0, f"Expected y=5.0, got {y}"
        
        # 验证 arctan2 可以正常计算
        target = np.arctan2(end_pt[1], end_pt[0])
        assert np.isfinite(target), "arctan2 should return finite value"
    
    def test_safety_monitor_reset_clears_all_state(self):
        """
        验证修复 #2: SafetyMonitor.reset() 清除 _last_filtered_accel
        
        之前的 Bug: reset() 没有清除 _last_filtered_accel，可能导致使用旧值
        修复后: reset() 正确设置 _last_filtered_accel = None
        """
        config = {'constraints': {'v_max': 2.0, 'omega_max': 2.0}, 'safety': {}}
        platform_config = {'type': 'differential', 'is_ground_vehicle': True}
        sm = BasicSafetyMonitor(config, platform_config)
        
        # 模拟有历史加速度值
        sm._last_filtered_accel = (1.0, 2.0, 0.0, 0.5)
        
        # 调用 reset
        sm.reset()
        
        # 验证所有状态都被清除
        assert sm._last_cmd is None, "_last_cmd should be None"
        assert sm._filtered_ax is None, "_filtered_ax should be None"
        assert sm._filtered_ay is None, "_filtered_ay should be None"
        assert sm._filtered_az is None, "_filtered_az should be None"
        assert sm._filtered_alpha is None, "_filtered_alpha should be None"
        assert sm._last_filtered_accel is None, "_last_filtered_accel should be None"  # 关键修复
        assert sm._filter_warmup_count == 0, "_filter_warmup_count should be 0"
    
    def test_trajectory_cache_single_key_update(self):
        """
        验证修复 #3: get_hard_velocities 不再冗余更新 cache_key
        
        之前的问题: cache_key 被写入两次 (line 455 和 line 505)
        修复后: cache_key 只在最后写入一次
        
        验证方式: 确保缓存机制正常工作
        """
        points = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=np.float64)
        traj = Trajectory(header=Header(), points=points, velocities=None, dt_sec=0.1)
        
        # 第一次调用
        v1 = traj.get_hard_velocities()
        
        # 第二次调用应该返回缓存的同一对象
        v2 = traj.get_hard_velocities()
        
        assert v1 is v2, "Cache should return same object"
        
        # 验证速度值正确
        assert v1.shape == (4, 4), f"Expected shape (4, 4), got {v1.shape}"
        # 期望速度: dx=1, dy=0, dz=0 / dt=0.1 = vx=10, vy=0, vz=0
        assert np.isclose(v1[0, 0], 10.0), f"Expected vx=10.0, got {v1[0, 0]}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
