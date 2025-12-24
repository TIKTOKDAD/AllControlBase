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
        精准 3D→2D 投影 - 将地面轨迹点投影到图像平面
        
        原理:
        1. 轨迹点在 base_footprint 坐标系 (x前, y左, z上, 原点在地面两轮中心)
        2. 转换到相机坐标系 (需要知道相机的位置和姿态)
        3. 使用针孔相机模型投影到图像平面
        """
        h, w = image_shape[:2]
        points_2d = []
        
        # 使用配置的相机外参
        cam_x = self.cam_x
        cam_y = self.cam_y
        cam_z = self.cam_z
        cam_pitch = self.cam_pitch
        
        # 预计算旋转矩阵 (绕 y 轴旋转 pitch)
        cos_p = np.cos(cam_pitch)
        sin_p = np.sin(cam_pitch)
        
        for pt in points:
            # ============ Step 1: base_footprint → 相机坐标系 ============
            # 轨迹点相对于相机的位置 (在 base_footprint 坐标系下)
            dx = pt.x - cam_x  # 前方距离
            dy = pt.y - cam_y  # 左右距离
            dz = pt.z - cam_z  # 高度差 (轨迹点在地面, z=0, 所以 dz = -cam_z)
            
            # ============ Step 2: 转换到相机光学坐标系 ============
            # 相机光学坐标系: x右, y下, z前
            # base_footprint: x前, y左, z上
            
            # 先应用俯仰旋转 (绕相机的 y 轴, 即 base 的 -y 轴)
            # 旋转后: x' = x*cos(p) + z*sin(p), z' = -x*sin(p) + z*cos(p)
            dx_rot = dx * cos_p - dz * sin_p
            dz_rot = dx * sin_p + dz * cos_p
            
            # 坐标系转换: base → camera optical
            x_cam = -dy        # base的y左 → cam的x右 (取负)
            y_cam = -dz_rot    # base的z上 → cam的y下 (取负)
            z_cam = dx_rot     # base的x前 → cam的z前
            
            # ============ Step 3: 针孔相机投影 ============
            # 只投影相机前方的点
            if z_cam > 0.01:  # 至少 1cm 前方
                # 投影公式: u = fx * x/z + cx, v = fy * y/z + cy
                u = self.fx * x_cam / z_cam + self.cx
                v = self.fy * y_cam / z_cam + self.cy
                
                # 转为整数像素坐标
                u_int = int(round(u))
                v_int = int(round(v))
                
                # 检查是否在图像范围内 (允许稍微超出)
                if -50 <= u_int < w + 50 and -50 <= v_int < h + 50:
                    # 限制到图像边界
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
