#!/bin/bash
# TurtleBot + ViNT 一键启动脚本
# 优化版: 支持环境变量配置，移除硬编码路径

SESSION="turtlebot"

# 配置路径 (支持环境变量覆盖)
VINT_DIR="${VINT_DIR:-$HOME/visualnav-transformer}"
# controller_ros 通过 ROS 包路径查找，无需硬编码

# 检查 VINT 目录是否存在
if [ ! -d "$VINT_DIR" ]; then
    echo "Warning: VINT directory not found at $VINT_DIR"
    echo "Please set VINT_DIR environment variable or check path."
    read -p "Press Enter to continue anyway..."
fi

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -c ~

# 启用鼠标支持
tmux set-option -t $SESSION mouse on

# 创建 4 个窗格
tmux split-window -t $SESSION -c ~
tmux split-window -t $SESSION -c ~
tmux split-window -t $SESSION -c ~

# 强制 tiled 布局 (四方格)
tmux select-layout -t $SESSION tiled

# 窗格0: ViNT - 先cd，分两步执行
tmux send-keys -t ${SESSION}:0.0 "cd ${VINT_DIR}/deployment/src" Enter
tmux send-keys -t ${SESSION}:0.0 'sudo modprobe gspca_kinect'

# 窗格1: explore - 先cd
tmux send-keys -t ${SESSION}:0.1 "cd ${VINT_DIR}/deployment/src" Enter
tmux send-keys -t ${SESSION}:0.1 'python explore_new.py'

# 窗格2: controller
# 使用 roslaunch 包名 启动文件 的方式，利用 ROS_PACKAGE_PATH 查找
tmux send-keys -t ${SESSION}:0.2 'roslaunch controller_ros turtlebot1.launch'

# 窗格3: trajectory
# 使用 rosrun 启动脚本
tmux send-keys -t ${SESSION}:0.3 'rosrun controller_ros trajectory_publisher.py'

tmux select-pane -t ${SESSION}:0.0

echo "=========================================="
echo "  执行顺序 (每个窗格按回车执行):"
echo "  1. 左上: modprobe -> 输密码 -> 手动输入: roslaunch vint_locobot.launch"
echo "  2. 右上: python explore_new.py"
echo "  3. 左下: roslaunch controller_ros"
echo "  4. 右下: rosrun trajectory_publisher"
echo "=========================================="
tmux attach -t $SESSION
