#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹可视化节点 - 在相机图像上叠加显示网络输出的轨迹点

功能:
- 订阅相机图像和轨迹数据
- 将轨迹点从 base_footprint 坐标系投影到图像平面
- 实时显示叠加后的图像

使用方法:
    roslaunch controller_ros trajectory_visualizer.launch
    或
    rosrun controller_ros trajectory_visualizer.py
"""

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from controller_ros.msg import LocalTrajectoryV4

class TrajectoryVisualizer:
    """轨迹可视化器 - 在图像上叠加轨迹点"""
    
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous=False)
        
        # 参数
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/nn/local_trajectory')
        self.output_topic = rospy.get_param('~output_topic', '/trajectory_overlay/image')
        self.camera_frame = rospy.get_param('~camera_frame', 'usb_cam')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        
        # 相机内参
        self.fx = rospy.get_param('~fx', 525.0)
        self.fy = rospy.get_param('~fy', 525.0)
        self.cx = rospy.get_param('~cx', 319.5)
        self.cy = rospy.get_param('~cy', 239.5)
        
        # 相机外参 (相对于 base_footprint)
        self.cam_x = rospy.get_param('~cam_x', 0.08)    # 前方距离 (m)
        self.cam_y = rospy.get_param('~cam_y', 0.0)     # 左右偏移 (m)
        self.cam_z = rospy.get_param('~cam_z', 0.50)    # 高度 (m)
        self.cam_pitch = rospy.get_param('~cam_pitch', 0.0)  # 俯仰角 (rad)
        
        # 显示参数
        self.show_window = rospy.get_param('~show_window', True)
        self.point_radius = rospy.get_param('~point_radius', 8)
        self.line_thickness = rospy.get_param('~line_thickness', 3)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 数据缓存
        self.latest_trajectory = None
        self.trajectory_stamp = None
        
        # 订阅
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        self.traj_sub = rospy.Subscriber(
            self.trajectory_topic, LocalTrajectoryV4, self.trajectory_callback, queue_size=1
        )
        
        # 发布
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        
        # 统计
        self.frame_count = 0
        
        rospy.loginfo(f"TrajectoryVisualizer 已启动:")
        rospy.loginfo(f"  图像输入: {self.image_topic}")
        rospy.loginfo(f"  轨迹输入: {self.trajectory_topic}")
        rospy.loginfo(f"  图像输出: {self.output_topic}")
        rospy.loginfo(f"  相机坐标系: {self.camera_frame}")
        rospy.loginfo(f"  基准坐标系: {self.base_frame}")
        rospy.loginfo(f"  显示窗口: {self.show_window}")
    
    def trajectory_callback(self, msg):
        """轨迹回调"""
        self.latest_trajectory = msg
        self.trajectory_stamp = rospy.Time.now()
    
    def image_callback(self, msg):
        """图像回调 - 主处理函数"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge 错误: {e}")
            return
        
        # 叠加轨迹
        if self.latest_trajectory is not None:
            cv_image = self.overlay_trajectory(cv_image, self.latest_trajectory)
        
        # 添加状态信息
        cv_image = self.draw_status(cv_image)
        
        # 发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(f"发布图像错误: {e}")
        
        # 显示窗口
        if self.show_window:
            cv2.imshow('Trajectory Overlay', cv_image)
            cv2.waitKey(1)
        
        self.frame_count += 1
    
    def overlay_trajectory(self, image, trajectory):
        """在图像上叠加轨迹点 - 简化版，直接使用2D投影"""
        # 直接使用简化的2D投影，起始点固定在底部中间
        points_2d = self.project_points_manual(trajectory.points, image.shape)
        
        # 绘制轨迹
        if points_2d:
            self.draw_trajectory_on_image(image, points_2d)
        
        return image
    
    def project_points_manual(self, points, image_shape):
        """
        标准 3D→2D 投影 - 使用齐次变换矩阵
        
        坐标系定义:
        - base_footprint: x前, y左, z上, 原点在地面两轮中心
        - camera optical: x右, y下, z前 (OpenCV/ROS 标准)
        
        变换流程:
        1. P_base: 轨迹点在 base_footprint 坐标系的坐标
        2. P_cam = T_cam_base @ P_base: 变换到相机坐标系
        3. p = K @ P_cam: 针孔投影到图像平面
        """
        h, w = image_shape[:2]
        points_2d = []
        
        # ============ 构建相机内参矩阵 K ============
        K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # ============ 构建外参变换矩阵 T_cam_base ============
        # 相机在 base_footprint 中的位置
        t_cam_in_base = np.array([self.cam_x, self.cam_y, self.cam_z])
        
        # 相机姿态: 先绕 base 的 y 轴旋转 pitch (向下为正)
        # 然后坐标系转换: base(x前,y左,z上) → cam_optical(x右,y下,z前)
        
        # 坐标系转换矩阵 (不含俯仰)
        # base_x(前) → cam_z(前)
        # base_y(左) → cam_-x(右的负方向，即左)  
        # base_z(上) → cam_-y(下的负方向，即上)
        R_base_to_cam_optical = np.array([
            [0, -1, 0],   # cam_x = -base_y
            [0, 0, -1],   # cam_y = -base_z
            [1, 0, 0]     # cam_z = base_x
        ])
        
        # 俯仰旋转矩阵 (绕相机的 x 轴，即 base 的 -y 轴)
        # 正 pitch = 相机向下看
        cos_p = np.cos(self.cam_pitch)
        sin_p = np.sin(self.cam_pitch)
        R_pitch = np.array([
            [1, 0, 0],
            [0, cos_p, -sin_p],
            [0, sin_p, cos_p]
        ])
        
        # 组合旋转: 先坐标系转换，再俯仰
        R_cam_base = R_pitch @ R_base_to_cam_optical
        
        # 平移: 相机看到的点 = 点在base中的位置 - 相机在base中的位置
        # 然后旋转到相机坐标系
        
        for pt in points:
            # 轨迹点在 base_footprint 坐标系 (地面点, z 通常为 0)
            P_base = np.array([pt.x, pt.y, pt.z])
            
            # 变换到相机坐标系
            P_rel = P_base - t_cam_in_base  # 相对于相机的位置
            P_cam = R_cam_base @ P_rel      # 旋转到相机坐标系
            
            # 相机坐标系中: x右, y下, z前
            x_cam, y_cam, z_cam = P_cam
            
            # 只投影相机前方的点 (z > 0)
            if z_cam > 0.01:
                # 针孔相机投影
                u = self.fx * x_cam / z_cam + self.cx
                v = self.fy * y_cam / z_cam + self.cy
                
                u_int = int(round(u))
                v_int = int(round(v))
                
                # 允许稍微超出图像范围
                if -100 <= u_int < w + 100 and -100 <= v_int < h + 100:
                    u_int = max(0, min(w - 1, u_int))
                    v_int = max(0, min(h - 1, v_int))
                    points_2d.append((u_int, v_int))
        
        return points_2d
    
    def draw_trajectory_on_image(self, image, points_2d, depths=None):
        """在图像上绘制轨迹"""
        if not points_2d:
            return
        
        # 颜色渐变: 近处绿色 -> 远处红色
        num_points = len(points_2d)
        
        # 绘制连线
        for i in range(len(points_2d) - 1):
            # 颜色渐变
            ratio = i / max(num_points - 1, 1)
            color = (
                int(255 * ratio),      # B: 0 -> 255
                int(255 * (1 - ratio)), # G: 255 -> 0
                0                       # R: 0
            )
            cv2.line(image, points_2d[i], points_2d[i + 1], color, self.line_thickness)
        
        # 绘制点
        for i, pt in enumerate(points_2d):
            ratio = i / max(num_points - 1, 1)
            
            if i == 0:
                # 第一个点 (当前位置) - 红色大圆
                color = (0, 0, 255)
                radius = self.point_radius + 4
            else:
                # 其他点 - 渐变色
                color = (
                    int(255 * ratio),
                    int(255 * (1 - ratio)),
                    0
                )
                radius = self.point_radius
            
            cv2.circle(image, pt, radius, color, -1)
            cv2.circle(image, pt, radius, (255, 255, 255), 1)  # 白色边框
            
            # 显示点序号
            cv2.putText(image, str(i), (pt[0] + 10, pt[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def draw_status(self, image):
        """绘制状态信息"""
        h, w = image.shape[:2]
        
        # 背景半透明矩形
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (250, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, image, 0.5, 0, image)
        
        # 状态文字
        status_lines = [
            f"Frame: {self.frame_count}",
            f"Traj points: {len(self.latest_trajectory.points) if self.latest_trajectory else 0}",
            f"Mode: {'TRACK' if self.latest_trajectory and self.latest_trajectory.mode == 1 else 'STOP' if self.latest_trajectory else 'N/A'}"
        ]
        
        y_offset = 25
        for line in status_lines:
            cv2.putText(image, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 20
        
        return image
    
    def run(self):
        """运行"""
        rospy.loginfo("TrajectoryVisualizer 正在运行...")
        rospy.spin()
        
        # 清理
        if self.show_window:
            cv2.destroyAllWindows()


def main():
    try:
        visualizer = TrajectoryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TrajectoryVisualizer 异常: {e}")
        raise


if __name__ == '__main__':
    main()
