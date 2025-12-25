#!/usr/bin/env python3
"""
诊断发布调试脚本

检查为什么 /controller/diagnostics 话题没有数据
"""

import sys
import time
import rospy

def main():
    rospy.init_node('debug_diagnostics', anonymous=True)
    
    print("=" * 60)
    print("  诊断发布调试")
    print("=" * 60)
    
    # 1. 检查话题信息
    print("\n[1] 检查话题信息...")
    try:
        from rostopic import get_topic_type
        topic_type, _, _ = get_topic_type('/controller/diagnostics')
        print(f"  话题类型: {topic_type}")
        
        # 检查发布者数量
        import rostopic
        pubs = rostopic.get_info_text('/controller/diagnostics')
        print(f"  话题信息:\n{pubs}")
    except Exception as e:
        print(f"  获取话题信息失败: {e}")
    
    # 2. 检查消息类型
    print("\n[2] 检查消息类型...")
    try:
        from controller_ros.msg import DiagnosticsV2
        print(f"  ✓ DiagnosticsV2 可用")
        print(f"  字段: {DiagnosticsV2.__slots__[:5]}...")
    except ImportError as e:
        print(f"  ✗ DiagnosticsV2 不可用: {e}")
        return 1
    
    # 3. 尝试手动创建发布器
    print("\n[3] 测试手动发布...")
    try:
        from controller_ros.msg import DiagnosticsV2
        test_pub = rospy.Publisher('/controller/diagnostics_test', DiagnosticsV2, queue_size=10)
        rospy.sleep(0.5)  # 等待连接
        
        msg = DiagnosticsV2()
        msg.state = 99  # 测试值
        msg.mpc_success = True
        test_pub.publish(msg)
        print(f"  ✓ 测试消息已发布到 /controller/diagnostics_test")
        print(f"  运行: rostopic echo /controller/diagnostics_test -n 1")
    except Exception as e:
        print(f"  ✗ 发布失败: {e}")
    
    # 4. 检查 ROS1PublisherManager
    print("\n[4] 检查 ROS1PublisherManager...")
    try:
        from controller_ros.io import ROS1PublisherManager
        print(f"  ✓ ROS1PublisherManager 可导入")
        
        # 尝试创建实例
        topics = {
            'cmd_unified': '/cmd_unified_test',
            'diagnostics': '/controller/diagnostics_test2',
            'state': '/controller/state_test',
        }
        pm = ROS1PublisherManager(topics=topics, diag_publish_rate=1)
        print(f"  ✓ ROS1PublisherManager 实例创建成功")
        print(f"  _diag_pub: {pm._diag_pub}")
        print(f"  _DiagnosticsV2: {pm._DiagnosticsV2}")
        
        # 测试发布
        test_diag = {
            'state': 1,
            'mpc_success': True,
            'mpc_solve_time_ms': 5.0,
        }
        pm.publish_diagnostics(test_diag, force=True)
        print(f"  ✓ 诊断已发布到 /controller/diagnostics_test2")
        
    except Exception as e:
        import traceback
        print(f"  ✗ 错误: {e}")
        traceback.print_exc()
    
    # 5. 订阅原始话题
    print("\n[5] 订阅 /controller/diagnostics (等待 10 秒)...")
    
    received = {'count': 0, 'last': None}
    
    def callback(msg):
        received['count'] += 1
        received['last'] = msg
        print(f"  收到消息 #{received['count']}: state={msg.state}")
    
    sub = rospy.Subscriber('/controller/diagnostics', DiagnosticsV2, callback)
    
    start = time.time()
    while time.time() - start < 10 and not rospy.is_shutdown():
        rospy.sleep(0.1)
        if received['count'] > 0:
            break
    
    if received['count'] > 0:
        print(f"  ✓ 收到 {received['count']} 条消息")
    else:
        print(f"  ✗ 未收到任何消息")
        print("\n  可能原因:")
        print("  1. 控制器节点的 _publishers._diag_pub 为 None")
        print("  2. 控制器节点的 _DiagnosticsV2 为 None")
        print("  3. 节流器阻止了发布")
        print("  4. fill_diagnostics_msg 抛出异常")
    
    print("\n" + "=" * 60)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
