#!/bin/bash
# TurtleBot + ViNT 一键启动脚本

SESSION_NAME="turtlebot"

# 关闭已存在的 session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 创建 session 和 4 个窗格
tmux new-session -d -s $SESSION_NAME

# 分割成 4 个窗格 (2x2)
tmux split-window -h -t $SESSION_NAME
tmux split-window -v -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.2

# 窗格 0 (左上): ViNT 启动
tmux send-keys -t $SESSION_NAME:0.0 'cd ~/visualnav-transformer/deployment/src && sudo modprobe gspca_kinect && roslaunch vint_locobot.launch'

# 窗格 1 (左下): controller_ros
tmux send-keys -t $SESSION_NAME:0.1 'roslaunch controller_ros turtlebot1.launch'

# 窗格 2 (右上): explore_new.py
tmux send-keys -t $SESSION_NAME:0.2 'cd ~/visualnav-transformer/deployment/src && python explore_new.py'

# 窗格 3 (右下): trajectory_publisher
tmux send-keys -t $SESSION_NAME:0.3 'rosrun controller_ros trajectory_publisher.py'

# 选中左上窗格
tmux select-pane -t $SESSION_NAME:0.0

echo "=========================================="
echo "  按回车执行命令，Ctrl+B+方向键 切换窗格"
echo "=========================================="
echo "  执行顺序:"
echo "  1. 左上: ViNT (先执行，等启动完成)"
echo "  2. 左下: controller_ros"
echo "  3. 右上: explore_new.py"
echo "  4. 右下: trajectory_publisher"
echo "=========================================="

tmux attach-session -t $SESSION_NAME
