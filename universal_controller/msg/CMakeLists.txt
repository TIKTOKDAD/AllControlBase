# CMakeLists.txt for controller_msgs
# 
# 用于生成 ROS 消息类型
# 
# 使用方法:
# 1. 将此目录复制到 catkin 工作空间的 src 目录
# 2. 重命名为 controller_msgs
# 3. 运行 catkin_make 或 catkin build

cmake_minimum_required(VERSION 3.0.2)
project(controller_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

# 添加消息文件
add_message_files(
  FILES
  DiagnosticsV2.msg
  LocalTrajectoryV4.msg
  UnifiedCmd.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
