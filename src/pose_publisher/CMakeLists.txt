cmake_minimum_required(VERSION 3.8)
project(pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加发布者的可执行文件
add_executable(pose_publisher src/pose_publisher.cpp)
add_executable(pose_publisher_dynamic src/pose_publisher_dynamic.cpp)
add_executable(pose_publisher_task1 src/pose_publisher_task1.cpp)
add_executable(pose_publisher_task2 src/pose_publisher_task2.cpp)
add_executable(pose_publisher_task1_random src/pose_publisher_task1_random.cpp)
add_executable(pose_publisher_task3 src/pose_publisher_task3.cpp)
add_executable(pose_publisher_task3_follow src/pose_publisher_task3_follow.cpp)
# 链接依赖
ament_target_dependencies(pose_publisher rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_dynamic rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_task1 rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_task2 rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_task1_random rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_task3 rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_task3_follow rclcpp geometry_msgs)
# 安装目标
install(TARGETS
  pose_publisher pose_publisher_dynamic pose_publisher_task1 pose_publisher_task2 pose_publisher_task3 pose_publisher_task3_follow pose_publisher_task1_random
  DESTINATION lib/${PROJECT_NAME})

ament_package()

