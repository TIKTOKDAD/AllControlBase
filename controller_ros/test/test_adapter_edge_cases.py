"""
适配器边界条件测试

测试 ROS 适配器在各种边界条件下的行为。

测试覆盖:
1. 空数据处理
2. 无效数据处理
3. 数据维度不匹配
4. 坐标系处理
"""
import pytest
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.core.enums import TrajectoryMode


# Mock 类定义
class MockTime:
    def __init__(self, sec=0, nsec=0):
        self.secs = sec
        self.nsecs = nsec


class MockHeader:
    def __init__(self, stamp=None, frame_id=""):
        self.stamp = stamp or MockTime()
        self.frame_id = frame_id


class MockPoint:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockQuaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class MockVector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockPose:
    def __init__(self):
        self.position = MockPoint()
        self.orientation = MockQuaternion()


class MockTwist:
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()


class MockPoseWithCov:
    def __init__(self):
        self.pose = MockPose()


class MockTwistWithCov:
    def __init__(self):
        self.twist = MockTwist()


class MockOdometry:
    def __init__(self):
        self.header = MockHeader()
        self.pose = MockPoseWithCov()
        self.twist = MockTwistWithCov()


class MockTrajectory:
    def __init__(self):
        self.header = MockHeader(frame_id="base_link")  # 使用有效的 frame_id
        self.points = []
        self.velocities_flat = []
        self.dt_sec = 0.1
        self.confidence = 0.9
        self.mode = 0
        self.soft_enabled = False


class TestTrajectoryAdapterEdgeCases:
    """测试轨迹适配器边界条件"""
    
    def test_empty_trajectory(self):
        """测试空轨迹"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = []
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert len(uc_traj.points) == 0
        assert uc_traj.mode == TrajectoryMode.MODE_STOP
    
    def test_single_point_trajectory(self):
        """测试单点轨迹"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        import numpy as np
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(1.0, 2.0, 0.0)]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert len(uc_traj.points) == 1
        # points 现在是 numpy 数组，使用索引访问
        if isinstance(uc_traj.points, np.ndarray):
            assert uc_traj.points[0, 0] == 1.0  # x
            assert uc_traj.points[0, 1] == 2.0  # y
        else:
            # Legacy Point3D 列表格式
            assert uc_traj.points[0].x == 1.0
    
    def test_velocity_dimension_mismatch(self):
        """测试速度维度不匹配"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(5)]
        # 速度数组长度不是 4 的倍数
        ros_msg.velocities_flat = [0.5, 0.0, 0.0]  # 只有 3 个元素
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该禁用 soft 模式或截断
        # 具体行为取决于实现
        assert uc_traj is not None
    
    def test_more_velocities_than_points(self):
        """测试速度点多于位置点 - 严格校验策略下丢弃速度数据"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(3)]
        ros_msg.velocities_flat = [0.5, 0.0, 0.0, 0.0] * 10  # 10 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 严格校验: 数量不匹配时丢弃速度数据
        assert uc_traj.velocities is None or not uc_traj.soft_enabled
    
    def test_fewer_velocities_than_points(self):
        """测试速度点少于位置点 - 严格校验策略下丢弃速度数据"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(10)]
        ros_msg.velocities_flat = [0.5, 0.0, 0.0, 0.0] * 3  # 只有 3 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 严格校验: 数量不匹配时丢弃速度数据，禁用 soft 模式
        assert uc_traj.velocities is None or not uc_traj.soft_enabled
    
    def test_exact_velocity_count_match(self):
        """测试速度点数与位置点数完全匹配"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(5)]
        ros_msg.velocities_flat = [0.5, 0.0, 0.0, 0.0] * 5  # 正好 5 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 数量匹配时保留速度数据
        assert uc_traj.velocities is not None
        assert uc_traj.velocities.shape[0] == 5
        assert uc_traj.soft_enabled
    
    def test_empty_frame_id(self):
        """测试空 frame_id 会抛出异常 (安全性改进)"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.header.frame_id = ""
        ros_msg.points = [MockPoint(0, 0, 0)]
        
        # 空 frame_id 应该抛出 ValueError (安全性改进: 拒绝隐式坐标系)
        with pytest.raises(ValueError, match="valid frame_id"):
            adapter.to_uc(ros_msg)
    
    def test_invalid_dt_sec(self):
        """测试无效 dt_sec"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.dt_sec = -0.1  # 无效值
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该使用默认值
        assert uc_traj.dt_sec > 0
    
    def test_invalid_confidence(self):
        """测试无效置信度"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.confidence = 1.5  # 超出范围
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该被裁剪到 [0, 1]
        assert 0.0 <= uc_traj.confidence <= 1.0
    
    def test_nan_point_coordinates(self):
        """测试 NaN 坐标"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [
            MockPoint(0, 0, 0),
            MockPoint(float('nan'), 0, 0),  # NaN 坐标
            MockPoint(0.2, 0, 0),
        ]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # NaN 点应该被过滤掉
        assert len(uc_traj.points) == 2
    
    def test_inf_point_coordinates(self):
        """测试 Inf 坐标"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [
            MockPoint(0, 0, 0),
            MockPoint(float('inf'), 0, 0),  # Inf 坐标
            MockPoint(0.2, 0, 0),
        ]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # Inf 点应该被过滤掉
        assert len(uc_traj.points) == 2
    
    def test_unknown_trajectory_mode(self):
        """测试未知轨迹模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.mode = 999  # 未知模式
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该使用默认模式
        assert uc_traj.mode == TrajectoryMode.MODE_TRACK


class TestOdomAdapterEdgeCases:
    """测试里程计适配器边界条件"""
    
    def test_basic_odom_conversion(self):
        """测试基本里程计转换"""
        from controller_ros.adapters.odom_adapter import OdomAdapter
        
        adapter = OdomAdapter()
        
        ros_msg = MockOdometry()
        ros_msg.pose.pose.position.x = 1.0
        ros_msg.pose.pose.position.y = 2.0
        ros_msg.twist.twist.linear.x = 0.5
        
        uc_odom = adapter.to_uc(ros_msg)
        
        assert uc_odom.pose_position.x == 1.0
        assert uc_odom.pose_position.y == 2.0
        assert uc_odom.twist_linear[0] == 0.5
    
    def test_odom_with_rotation(self):
        """测试带旋转的里程计"""
        from controller_ros.adapters.odom_adapter import OdomAdapter
        
        adapter = OdomAdapter()
        
        ros_msg = MockOdometry()
        ros_msg.pose.pose.orientation.z = 0.707
        ros_msg.pose.pose.orientation.w = 0.707
        ros_msg.twist.twist.angular.z = 0.5
        
        uc_odom = adapter.to_uc(ros_msg)
        
        assert uc_odom.pose_orientation[2] == 0.707
        assert uc_odom.twist_angular[2] == 0.5


class TestImuAdapterEdgeCases:
    """测试 IMU 适配器边界条件"""
    
    def test_basic_imu_conversion(self):
        """测试基本 IMU 转换"""
        from controller_ros.adapters.imu_adapter import ImuAdapter
        
        adapter = ImuAdapter()
        
        class MockImu:
            def __init__(self):
                self.header = MockHeader()
                self.orientation = MockQuaternion()
                self.angular_velocity = MockVector3(0, 0, 0.5)
                self.linear_acceleration = MockVector3(0, 0, 9.8)
        
        ros_msg = MockImu()
        uc_imu = adapter.to_uc(ros_msg)
        
        assert uc_imu.angular_velocity[2] == 0.5
        assert uc_imu.linear_acceleration[2] == 9.8


class TestOutputAdapterEdgeCases:
    """测试输出适配器边界条件"""
    
    def test_control_output_conversion(self):
        """测试控制输出转换"""
        from controller_ros.adapters.output_adapter import OutputAdapter
        from universal_controller.core.data_types import ControlOutput
        
        adapter = OutputAdapter()
        
        uc_cmd = ControlOutput(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            omega=0.5,
            frame_id="base_link",
            success=True
        )
        
        # 转换为 ROS 消息需要 ROS 环境，这里只测试接口存在
        assert hasattr(adapter, 'to_ros')
    
    def test_control_output_with_extras(self):
        """测试带额外数据的控制输出"""
        from controller_ros.adapters.output_adapter import OutputAdapter
        from universal_controller.core.data_types import ControlOutput
        
        adapter = OutputAdapter()
        
        uc_cmd = ControlOutput(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            omega=0.5,
            frame_id="base_link",
            success=True,
            extras={'attitude_cmd': {'roll': 0.1, 'pitch': 0.2}}
        )
        
        assert 'attitude_cmd' in uc_cmd.extras


class TestStrictVelocityValidation:
    """测试严格速度校验策略"""
    
    def test_velocity_mismatch_disables_soft(self):
        """速度数量不匹配时禁用 soft 模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(10)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 5  # 只有 5 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 严格校验: 不匹配时禁用 soft
        assert not uc_traj.soft_enabled or uc_traj.velocities is None
    
    def test_exact_match_preserves_soft(self):
        """速度数量完全匹配时保留 soft 模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(10)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 10  # 正好 10 个
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert uc_traj.soft_enabled
        assert uc_traj.velocities is not None
        assert uc_traj.velocities.shape == (10, 4)
    
    def test_empty_velocities_disables_soft(self):
        """空速度数组禁用 soft 模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(5)]
        ros_msg.velocities_flat = []
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert not uc_traj.soft_enabled
    
    def test_invalid_velocity_dimension_disables_soft(self):
        """速度维度不是 4 的倍数时禁用 soft 模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(5)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0]  # 3 个元素，不是 4 的倍数
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert not uc_traj.soft_enabled


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
