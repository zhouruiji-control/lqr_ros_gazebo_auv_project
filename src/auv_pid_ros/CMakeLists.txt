cmake_minimum_required(VERSION 3.8)
project(auv_pid_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(auv_pid_node src/auv_pid_node.cpp)
ament_target_dependencies(auv_pid_node
  rclcpp
  std_msgs
)

add_executable(gazebo_pose_sync_node src/gazebo_pose_sync_node.cpp)
ament_target_dependencies(gazebo_pose_sync_node
  rclcpp
  std_msgs
  gazebo_msgs
)

install(TARGETS
  auv_pid_node
  gazebo_pose_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
