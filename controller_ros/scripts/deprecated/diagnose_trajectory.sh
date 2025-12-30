#!/bin/bash
# 轨迹方向诊断脚本 (数据采集)
# 
# 注意: 此脚本仅用于快速采集原始数据
# 如需完整诊断分析，请使用:
#   rosrun controller_ros unified_diagnostics.py --mode realtime
#
# 用法: ./diagnose_trajectory.sh

echo "=============================================="
echo "轨迹方向诊断工具 (数据采集)"
echo "=============================================="
echo ""
echo "提示: 如需完整诊断分析，请使用:"
echo "  rosrun controller_ros unified_diagnostics.py --mode realtime"
echo ""

# 检查 ROS 环境
if ! command -v rostopic &> /dev/null; then
    echo "错误: ROS 环境未配置，请先 source setup.bash"
    exit 1
fi

echo "[1/4] 采集网络轨迹 (5条消息)..."
echo "----------------------------------------------"
rostopic echo -n 5 /nn/local_trajectory 2>/dev/null || echo "话题 /nn/local_trajectory 无数据"

echo ""
echo "[2/4] 采集控制器输出 (5条消息)..."
echo "----------------------------------------------"
rostopic echo -n 5 /cmd_unified 2>/dev/null || echo "话题 /cmd_unified 无数据"

echo ""
echo "[3/4] 采集控制器诊断 (3条消息)..."
echo "----------------------------------------------"
rostopic echo -n 3 /controller/diagnostics 2>/dev/null || echo "话题 /controller/diagnostics 无数据"

echo ""
echo "[4/4] 采集底盘速度命令 (5条消息)..."
echo "----------------------------------------------"
rostopic echo -n 5 /mobile_base/commands/velocity 2>/dev/null || echo "话题 /mobile_base/commands/velocity 无数据"

echo ""
echo "=============================================="
echo "诊断完成，请将以上输出发给开发者分析"
echo "=============================================="
