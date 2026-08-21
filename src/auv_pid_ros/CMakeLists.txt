cmake_minimum_required(VERSION 3.8)
project(auv_pid_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# 原来的 PID 节点
add_executable(
  auv_pid_node
  src/auv_pid_node.cpp
)

ament_target_dependencies(
  auv_pid_node
  rclcpp
  std_msgs
)

# 受力版：只负责计算 PID 控制量
add_executable(
  pid_controller_only_node
  src/pid_controller_only_node.cpp
)

ament_target_dependencies(
  pid_controller_only_node
  rclcpp
  std_msgs
)

# 受力版：读取 Gazebo 实际深度
add_executable(
  force_depth_feedback_node
  src/force_depth_feedback_node.cpp
)

ament_target_dependencies(
  force_depth_feedback_node
  rclcpp
  std_msgs
  gazebo_msgs
)

# 原来的展示版：根据 ROS 深度强制修改 Gazebo 位置
add_executable(
  gazebo_pose_sync_node
  src/gazebo_pose_sync_node.cpp
)

ament_target_dependencies(
  gazebo_pose_sync_node
  rclcpp
  std_msgs
  gazebo_msgs
)

# 受力版：把控制输出转换成 Gazebo 外力
add_executable(
  gazebo_force_node
  src/gazebo_force_node.cpp
)

ament_target_dependencies(
  gazebo_force_node
  rclcpp
  std_msgs
  gazebo_msgs
)

# 安装所有可执行节点
install(TARGETS
  auv_pid_node
  pid_controller_only_node
  gazebo_pose_sync_node
  force_depth_feedback_node
  gazebo_force_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()