#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动底盘测试脚本 - 用于滞空测试时手动调整参数

使用方法:
1. 底盘滞空
2. 运行此脚本
3. 观察轮子转速，手动记录
4. 根据经验调整配置

rosrun controller_ros test_chassis_manual.py
"""
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class ManualChassisTest:
    def __init__(self):
        rospy.init_node('manual_chassis_test')
        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.last_odom = None
        
    def odom_callback(self, msg):
        self.last_odom = msg
    
    def test_linear_speed(self, target_speeds=[0.2, 0.4, 0.6, 0.8]):
        """测试不同线速度"""
        print("\n=== 线速度测试 ===")
        print("观察轮子转速，判断是否合理\n")
        
        for v in target_speeds:
            print(f"测试速度: {v} m/s")
            cmd = Twist()
            cmd.linear.x = v
            
            # 发送命令3秒
            start = time.time()
            max_reported = 0
            while time.time() - start < 3.0:
                self.cmd_pub.publish(cmd)
                if self.last_odom:
                    reported_v = self.last_odom.twist.twist.linear.x
                    max_reported = max(max_reported, abs(reported_v))
                time.sleep(0.05)
            
            # 停止
            self.cmd_pub.publish(Twist())
            time.sleep(1.0)
            
            print(f"  命令: {v} m/s")
            print(f"  odom报告: {max_reported:.3f} m/s")
            print(f"  建议实际值: {max_reported * 0.6:.3f} m/s (打6折)")
            
            input("  按Enter继续下一个测试...")
    
    def test_angular_speed(self, target_omegas=[0.5, 1.0, 1.5, 2.0]):
        """测试不同角速度"""
        print("\n=== 角速度测试 ===")
        print("观察轮子转速差，判断转向能力\n")
        
        for omega in target_omegas:
            print(f"测试角速度: {omega} rad/s")
            cmd = Twist()
            cmd.angular.z = omega
            
            # 发送命令2秒
            start = time.time()
            max_reported = 0
            while time.time() - start < 2.0:
                self.cmd_pub.publish(cmd)
                if self.last_odom:
                    reported_omega = self.last_odom.twist.twist.angular.z
                    max_reported = max(max_reported, abs(reported_omega))
                time.sleep(0.05)
            
            # 停止
            self.cmd_pub.publish(Twist())
            time.sleep(1.0)
            
            print(f"  命令: {omega} rad/s")
            print(f"  odom报告: {max_reported:.3f} rad/s")
            print(f"  建议实际值: {max_reported * 0.7:.3f} rad/s (打7折)")
            
            input("  按Enter继续下一个测试...")
    
    def run(self):
        print("手动底盘测试")
        print("=" * 50)
        print("注意: 请确保底盘已滞空，周围安全")
        print("=" * 50)
        
        input("\n按Enter开始线速度测试...")
        self.test_linear_speed()
        
        input("\n按Enter开始角速度测试...")
        self.test_angular_speed()
        
        print("\n测试完成！")
        print("\n建议配置:")
        print("根据上面的测试结果，选择合适的最大值")
        print("一般建议:")
        print("  v_max: 0.3-0.5 m/s (TurtleBot典型值)")
        print("  omega_max: 1.0-1.5 rad/s")
        print("  a_max: 0.5-1.0 m/s^2 (不要用滞空测出的13.97!)")

if __name__ == '__main__':
    try:
        tester = ManualChassisTest()
        time.sleep(1.0)  # 等待订阅建立
        tester.run()
    except rospy.ROSInterruptException:
        pass
