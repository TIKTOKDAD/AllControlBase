#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹发布器 - 将神经网络输出转换为 LocalTrajectoryV4 消息

功能:
- 将神经网络输出的轨迹转换为 controller_ros/LocalTrajectoryV4 格式
- 发布到 /nn/local_trajectory 话题
- 支持带速度和不带速度的轨迹

使用方法:
    1. 修改 YourNetwork 类为你的实际网络
    2. rosrun controller_ros trajectory_publisher.py

话题:
    发布: /nn/local_trajectory (controller_ros/LocalTrajectoryV4)

重要:
    - 轨迹坐标系必须是 base_link (机器人当前位置为原点，X轴朝前)
    - 轨迹点数建议 10-30 个
    - 时间间隔建议 0.1s (与 MPC dt 匹配)
"""

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header

try:
    from controller_ros.msg import LocalTrajectoryV4
except ImportError:
    rospy.logerr("无法导入 controller_ros.msg，请确保 controller_ros 已编译")
    raise


# =============================================================================
# 轨迹模式常量
# =============================================================================
MODE_TRACK = 0      # 跟踪模式 (正常跟踪轨迹)
MODE_STOP = 1       # 停止模式 (减速停车)
MODE_HOVER = 2      # 悬停模式 (仅四旋翼)
MODE_EMERGENCY = 3  # 紧急模式 (立即停止)


# =============================================================================
# TODO: 替换为你的神经网络类
# =============================================================================
class YourNetwork:
    """
    你的神经网络类 (示例)
    
    TODO: 替换为你的实际网络实现
    """

    def __init__(self):
        """初始化网络"""
        # TODO: 加载你的模型
        # self.model = load_model('your_model.pth')
        rospy.loginfo("YourNetwork 初始化 (示例模式)")
    
    def predict(self, sensor_data=None):
        """
        网络推理
        
        Args:
            sensor_data: 传感器数据 (根据你的网络需求)
        
        Returns:
            dict: 包含以下字段:
                - positions: numpy array, shape (N, 2), [x, y] 坐标 (base_link 坐标系)
                - velocities: numpy array, shape (N, 2), [vx, vy] 速度 (可选)
                - confidence: float, 轨迹置信度 [0, 1] (可选)
                - mode: int, 轨迹模式 (可选, 默认 MODE_TRACK)
        
        示例输出:
            {
                'positions': np.array([[0.2, 0.0], [0.4, 0.0], ...]),
                'velocities': np.array([[0.3, 0.0], [0.3, 0.0], ...]),  # 可选
                'confidence': 0.95,  # 可选
                'mode': MODE_TRACK,  # 可选
            }
        """
        # TODO: 替换为你的网络推理代码
        # output = self.model(sensor_data)
        # return output
        
        # ===== 示例: 生成直线轨迹 (向前 2 米) =====
        num_points = 15
        positions = np.array([[i * 0.15, 0.0] for i in range(num_points)])
        
        # 可选: 生成速度
        velocities = np.array([[0.3, 0.0] for _ in range(num_points)])
        
        return {
            'positions': positions,
            'velocities': velocities,
            'confidence': 1.0,
            'mode': MODE_TRACK,
        }


# =============================================================================
# 轨迹发布器
# =============================================================================
class TrajectoryPublisher:
    """轨迹发布器类"""
    
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=False)
        
        # 参数
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        self.dt = rospy.get_param('~dt', 0.1)  # 轨迹点时间间隔
        output_topic = rospy.get_param('~output_topic', '/nn/local_trajectory')
        
        # 发布器
        self.pub = rospy.Publisher(output_topic, LocalTrajectoryV4, queue_size=1)
        
        # 初始化网络
        self.network = YourNetwork()
        
        # 统计
        self.publish_count = 0
        
        rospy.loginfo(f"TrajectoryPublisher 已启动:")
        rospy.loginfo(f"  发布话题: {output_topic}")
        rospy.loginfo(f"  发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"  轨迹时间间隔: {self.dt} s")

    def create_trajectory_msg(self, positions, velocities=None, 
                               confidence=1.0, mode=MODE_TRACK):
        """
        创建 LocalTrajectoryV4 消息
        
        Args:
            positions: numpy array, shape (N, 2) 或 (N, 3)
                       [x, y] 或 [x, y, z] 坐标
                       坐标系: base_link (机器人当前位置为原点，X轴朝前)
            velocities: numpy array, shape (N, 2) 或 (N, 4), 可选
                        [vx, vy] 或 [vx, vy, vz, wz] 速度
            confidence: float, 轨迹置信度 [0, 1]
            mode: int, 轨迹模式 (MODE_TRACK, MODE_STOP, MODE_HOVER, MODE_EMERGENCY)
        
        Returns:
            LocalTrajectoryV4: ROS 消息
        """
        msg = LocalTrajectoryV4()
        
        # Header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"  # 重要：必须是 base_link
        
        # 轨迹模式
        msg.mode = mode
        
        # 轨迹点
        msg.points = []
        for i in range(len(positions)):
            p = Point()
            p.x = float(positions[i, 0])
            p.y = float(positions[i, 1])
            p.z = float(positions[i, 2]) if positions.shape[1] > 2 else 0.0
            msg.points.append(p)
        
        # 速度 (可选)
        if velocities is not None and len(velocities) > 0:
            msg.velocities_flat = []
            for i in range(len(velocities)):
                if velocities.shape[1] >= 4:
                    # 完整速度: [vx, vy, vz, wz]
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        float(velocities[i, 1]),
                        float(velocities[i, 2]),
                        float(velocities[i, 3]),
                    ])
                elif velocities.shape[1] >= 2:
                    # 只有 [vx, vy]
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        float(velocities[i, 1]),
                        0.0,  # vz
                        0.0,  # wz
                    ])
                else:
                    # 只有 vx
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        0.0, 0.0, 0.0,
                    ])
            msg.soft_enabled = True
        else:
            msg.velocities_flat = []
            msg.soft_enabled = False
        
        # 时间间隔
        msg.dt_sec = self.dt
        
        # 置信度
        msg.confidence = confidence
        
        return msg
    
    def publish_trajectory(self, network_output):
        """
        发布轨迹
        
        Args:
            network_output: dict, 网络输出，包含 positions, velocities (可选), 
                           confidence (可选), mode (可选)
        """
        positions = network_output['positions']
        velocities = network_output.get('velocities', None)
        confidence = network_output.get('confidence', 1.0)
        mode = network_output.get('mode', MODE_TRACK)
        
        msg = self.create_trajectory_msg(positions, velocities, confidence, mode)
        self.pub.publish(msg)
        
        self.publish_count += 1
        if self.publish_count % 50 == 0:
            rospy.logdebug(f"已发布 {self.publish_count} 条轨迹")

    def run(self):
        """主循环"""
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo("TrajectoryPublisher 正在运行...")
        
        while not rospy.is_shutdown():
            try:
                # 获取传感器数据 (根据你的需求)
                sensor_data = self.get_sensor_data()
                
                # 网络推理
                network_output = self.network.predict(sensor_data)
                
                # 发布轨迹
                self.publish_trajectory(network_output)
                
            except Exception as e:
                rospy.logerr(f"轨迹发布异常: {e}")
            
            rate.sleep()
    
    def get_sensor_data(self):
        """
        获取传感器数据
        
        TODO: 根据你的网络需求实现
        
        Returns:
            传感器数据 (格式根据你的网络需求)
        """
        # TODO: 订阅并获取传感器数据
        # 例如: 图像、激光雷达、里程计等
        return None


# =============================================================================
# 停止轨迹发布器 (用于紧急停止)
# =============================================================================
class StopTrajectoryPublisher:
    """发布停止轨迹"""
    
    def __init__(self):
        rospy.init_node('stop_trajectory_publisher', anonymous=True)
        self.pub = rospy.Publisher('/nn/local_trajectory', LocalTrajectoryV4, queue_size=1)
        rospy.sleep(0.5)  # 等待连接
    
    def publish_stop(self):
        """发布停止轨迹"""
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.mode = MODE_STOP
        msg.points = [Point(x=0.0, y=0.0, z=0.0)]
        msg.velocities_flat = []
        msg.soft_enabled = False
        msg.dt_sec = 0.1
        msg.confidence = 1.0
        
        self.pub.publish(msg)
        rospy.loginfo("已发布停止轨迹")


# =============================================================================
# 主函数
# =============================================================================
def main():
    try:
        publisher = TrajectoryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TrajectoryPublisher 异常: {e}")
        raise


if __name__ == '__main__':
    main()
