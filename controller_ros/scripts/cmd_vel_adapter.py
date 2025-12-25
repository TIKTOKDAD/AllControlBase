#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel 适配器 - 将 UnifiedCmd 转换为 TurtleBot 的 cmd_vel

功能:
- 订阅 /cmd_unified (controller_ros/UnifiedCmd)
- 发布 /cmd_vel (geometry_msgs/Twist)
- 应用速度限制保护机器人
- 可选的速度变化率限制（平滑处理）

使用方法:
    rosrun controller_ros cmd_vel_adapter.py
    
    或通过 launch 文件启动:
    roslaunch controller_ros turtlebot1.launch

参数:
    ~max_linear: 最大线速度 (m/s), 默认 0.5
    ~max_angular: 最大角速度 (rad/s), 默认 1.0
    ~max_linear_accel: 最大线加速度 (m/s²), 默认 0 (禁用)
    ~max_angular_accel: 最大角加速度 (rad/s²), 默认 0 (禁用)
    ~input_topic: 输入话题, 默认 /cmd_unified
    ~output_topic: 输出话题, 默认 /cmd_vel

注意:
    速度变化率限制（max_linear_accel, max_angular_accel）默认禁用，
    因为平滑控制应该由上游 MPC 控制器负责。
    仅在特殊情况下（如 MPC 输出不平滑）才需要启用。
"""

# 注意：不要在这里修改 sys.path！
# PYTHONPATH 已经由 source devel/setup.bash 正确设置

import rospy
from geometry_msgs.msg import Twist

try:
    from controller_ros.msg import UnifiedCmd
except ImportError:
    rospy.logerr("无法导入 controller_ros.msg，请确保 controller_ros 已编译")
    raise


class CmdVelAdapter:
    """cmd_vel 适配器类"""
    
    def __init__(self):
        rospy.init_node('cmd_vel_adapter', anonymous=False)
        
        # 获取参数
        self.max_linear = rospy.get_param('~max_linear', 0.5)
        self.max_angular = rospy.get_param('~max_angular', 1.0)
        
        # 可选的速度变化率限制（0 表示禁用）
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 0.0)
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 0.0)
        
        input_topic = rospy.get_param('~input_topic', '/cmd_unified')
        output_topic = rospy.get_param('~output_topic', '/cmd_vel')
        
        # 订阅控制器输出
        self.sub = rospy.Subscriber(input_topic, UnifiedCmd, self.callback, queue_size=1)
        
        # 发布 cmd_vel
        self.pub = rospy.Publisher(output_topic, Twist, queue_size=1)
        
        # 状态（用于速度变化率限制）
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time = None
        
        # 统计
        self.msg_count = 0
        self.clamp_count = 0
        self.rate_limit_count = 0
        
        rospy.loginfo(f"CmdVelAdapter 已启动:")
        rospy.loginfo(f"  输入话题: {input_topic}")
        rospy.loginfo(f"  输出话题: {output_topic}")
        rospy.loginfo(f"  最大线速度: {self.max_linear} m/s")
        rospy.loginfo(f"  最大角速度: {self.max_angular} rad/s")
        if self.max_linear_accel > 0:
            rospy.loginfo(f"  最大线加速度: {self.max_linear_accel} m/s² (启用)")
        if self.max_angular_accel > 0:
            rospy.loginfo(f"  最大角加速度: {self.max_angular_accel} rad/s² (启用)")

    def callback(self, msg: UnifiedCmd):
        """
        处理 UnifiedCmd 消息，转换为 Twist 并发布
        
        Args:
            msg: UnifiedCmd 消息，包含 vx, vy, vz, omega 等字段
        """
        twist = Twist()
        
        # 差速车只使用 vx 和 omega
        # 保存原始值用于日志
        original_linear_x = msg.vx
        original_angular_z = msg.omega
        
        linear_x = msg.vx
        angular_z = msg.omega
        
        # 0. 数据有效性检查 - 检测 NaN 和 Inf
        import math
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            rospy.logwarn_throttle(1.0, 
                f"收到无效速度命令: vx={linear_x}, omega={angular_z}，发布零速度")
            # 发布零速度作为安全措施
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            # 重置状态，避免下次使用无效的 last 值
            self.last_linear_x = 0.0
            self.last_angular_z = 0.0
            return
        
        # 1. 应用速度变化率限制（如果启用）
        if self.max_linear_accel > 0 or self.max_angular_accel > 0:
            linear_x, angular_z = self._apply_rate_limit(linear_x, angular_z)
        
        # 2. 应用速度限制（安全边界）
        linear_x_clamped = self._clamp(linear_x, -self.max_linear, self.max_linear)
        angular_z_clamped = self._clamp(angular_z, -self.max_angular, self.max_angular)
        
        # 检查是否被限制 (与原始输入值比较，而非变化率限制后的值)
        if abs(linear_x_clamped - original_linear_x) > 0.001 or abs(angular_z_clamped - original_angular_z) > 0.001:
            self.clamp_count += 1
            if self.clamp_count % 100 == 1:  # 每 100 次打印一次警告
                rospy.logwarn(f"速度被限制: vx {original_linear_x:.2f}->{linear_x_clamped:.2f}, "
                             f"omega {original_angular_z:.2f}->{angular_z_clamped:.2f}")
        
        # 更新状态
        self.last_linear_x = linear_x_clamped
        self.last_angular_z = angular_z_clamped
        
        # 设置 Twist 消息
        twist.linear.x = linear_x_clamped
        twist.linear.y = 0.0  # 差速车不支持横向移动
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z_clamped
        
        # 发布
        self.pub.publish(twist)
        
        self.msg_count += 1
        if self.msg_count % 200 == 0:  # 每 200 条消息打印一次状态
            rospy.logdebug(f"已处理 {self.msg_count} 条消息, "
                          f"限制次数: {self.clamp_count}, "
                          f"变化率限制次数: {self.rate_limit_count}")
    
    def _apply_rate_limit(self, linear_x: float, angular_z: float) -> tuple:
        """
        应用速度变化率限制
        
        Args:
            linear_x: 目标线速度
            angular_z: 目标角速度
        
        Returns:
            (limited_linear_x, limited_angular_z): 限制后的速度
        """
        now = rospy.Time.now()
        
        if self.last_time is None:
            self.last_time = now
            return linear_x, angular_z
        
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        
        if dt <= 0:
            return linear_x, angular_z
        
        rate_limited = False
        
        # 线速度变化率限制
        if self.max_linear_accel > 0:
            max_delta_linear = self.max_linear_accel * dt
            delta_linear = linear_x - self.last_linear_x
            if abs(delta_linear) > max_delta_linear:
                linear_x = self.last_linear_x + max_delta_linear * (1 if delta_linear > 0 else -1)
                rate_limited = True
        
        # 角速度变化率限制
        if self.max_angular_accel > 0:
            max_delta_angular = self.max_angular_accel * dt
            delta_angular = angular_z - self.last_angular_z
            if abs(delta_angular) > max_delta_angular:
                angular_z = self.last_angular_z + max_delta_angular * (1 if delta_angular > 0 else -1)
                rate_limited = True
        
        if rate_limited:
            self.rate_limit_count += 1
        
        return linear_x, angular_z
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制值在指定范围内"""
        return max(min_val, min(max_val, value))
    
    def run(self):
        """运行适配器"""
        rospy.loginfo("CmdVelAdapter 正在运行...")
        rospy.spin()


def main():
    try:
        adapter = CmdVelAdapter()
        adapter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"CmdVelAdapter 异常: {e}")
        raise


if __name__ == '__main__':
    main()
