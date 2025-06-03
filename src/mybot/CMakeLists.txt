cmake_minimum_required(VERSION 3.22)
project(mybot)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# 添加可执行文件
add_executable(move_group_interface src/move_group_interface.cpp)
add_executable(end_effector_feedback src/end_effector_feedback.cpp)
add_executable(arm_execute_circle src/arm_execute_circle.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_circle_trajectory src/arm_execute_circle_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory src/arm_execute_total_trajectory.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(arm_execute_total_trajectory_final src/arm_execute_total_trajectory_final.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(mpc_based_test src/mpc_based_test.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(mpc_based_dynamic src/mpc_based_dynamic.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(dynamic_grasping src/dynamic_grasping.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)

add_executable(trajectory_buffer_node src/trajectory_buffer.cpp)
add_executable(move_trajectory_buffer src/move_trajectory_buffer.cpp)
add_executable(rrt_base src/rrt_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(BFMTk_base src/BFMTk_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(BiEST_base src/BiEST_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(BiTRRT_base src/BiTRRT_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(PRMstar_base src/PRMstar_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(SPARStwo_base src/SPARStwo_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(BFPIECE_base src/BFPIECE_base.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(task1_hmpc src/task1_hmpc.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(task2_hmpc src/task2_hmpc.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(task3_hmpc src/task3_hmpc.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(task3_PRMstar src/task3_PRMstar.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)
add_executable(task3_compare src/task3_compare.cpp src/arm_control.cpp src/Communication.cpp src/Trajectory.cpp)

# 链接必要的 ROS 2 和 MoveIt! 库
ament_target_dependencies(trajectory_buffer_node
        rclcpp
        moveit_msgs
        moveit_ros_planning_interface
        trajectory_msgs  # 添加这一行
)

ament_target_dependencies(move_trajectory_buffer
        rclcpp
        moveit_msgs
        moveit_ros_planning_interface
        trajectory_msgs  # 添加这一行
)

ament_target_dependencies(move_group_interface
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(end_effector_feedback
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_circle
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_circle_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(arm_execute_total_trajectory
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(mpc_based_test
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(mpc_based_dynamic
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(dynamic_grasping
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(arm_execute_total_trajectory_final
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)


ament_target_dependencies(rrt_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)

ament_target_dependencies(BFMTk_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(BiEST_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(BiTRRT_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(PRMstar_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(BFPIECE_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(SPARStwo_base
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(task1_hmpc
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(task2_hmpc
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(task3_hmpc
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(task3_PRMstar
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
ament_target_dependencies(task3_compare
        rclcpp
        moveit_core
        moveit_ros_planning_interface
        geometry_msgs
        Eigen3
        moveit_visual_tools
        moveit_msgs
        tf2
        tf2_eigen
        tf2_ros
)
# 安装可执行文件
install(TARGETS move_group_interface end_effector_feedback trajectory_buffer_node move_trajectory_buffer  arm_execute_total_trajectory_final arm_execute_total_trajectory dynamic_grasping mpc_based_dynamic mpc_based_test arm_execute_circle_trajectory arm_execute_circle rrt_base BFMTk_base BiEST_base BiTRRT_base PRMstar_base BFPIECE_base SPARStwo_base task1_hmpc task2_hmpc task3_compare task3_hmpc task3_PRMstar
        DESTINATION lib/${PROJECT_NAME})

# 安装src目录中的所有文件
install(DIRECTORY src/ DESTINATION share/${PROJECT_NAME}/src
        FILES_MATCHING PATTERN "*.cpp" PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.c"
)

# 安装config目录中的所有文件
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 安装launch目录和其他文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
        PATTERN "setup_assistant.launch" EXCLUDE)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()