# Install script for directory: /home/fins/myrobot_move/src/mybot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/move_group_interface")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_group_interface")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/move_group_interface.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/end_effector_feedback")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/end_effector_feedback")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/end_effector_feedback.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/trajectory_buffer_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/trajectory_buffer_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/trajectory_buffer_node.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/move_trajectory_buffer")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/move_trajectory_buffer")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/move_trajectory_buffer.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/arm_execute_total_trajectory_final")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory_final")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/arm_execute_total_trajectory_final.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/arm_execute_total_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_total_trajectory")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/arm_execute_total_trajectory.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/dynamic_grasping")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/dynamic_grasping")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/dynamic_grasping.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/mpc_based_dynamic")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_dynamic")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/mpc_based_dynamic.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/mpc_based_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/mpc_based_test")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/mpc_based_test.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/arm_execute_circle_trajectory")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle_trajectory")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/arm_execute_circle_trajectory.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/arm_execute_circle")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/arm_execute_circle")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/arm_execute_circle.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/rrt_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/rrt_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/rrt_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/BFMTk_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFMTk_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/BFMTk_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/BiEST_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiEST_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/BiEST_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/BiTRRT_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BiTRRT_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/BiTRRT_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/PRMstar_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/PRMstar_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/PRMstar_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/BFPIECE_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/BFPIECE_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/BFPIECE_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/SPARStwo_base")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/SPARStwo_base")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/SPARStwo_base.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/task1_hmpc")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task1_hmpc")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/task1_hmpc.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/task2_hmpc")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task2_hmpc")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/task2_hmpc.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/task3_compare")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_compare")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/task3_compare.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/task3_hmpc")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_hmpc")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/task3_hmpc.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mybot" TYPE EXECUTABLE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/task3_PRMstar")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar"
         OLD_RPATH "/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning_interface/lib:/home/fins/Workspace/moveit2_ws/install/moveit_visual_tools/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_move_group/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_warehouse/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_planning/lib:/home/fins/Workspace/moveit2_ws/install/moveit_ros_occupancy_map_monitor/lib:/home/fins/Workspace/moveit2_ws/install/moveit_core/lib:/home/fins/Workspace/moveit2_ws/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mybot/task3_PRMstar")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/fins/myrobot_move/src/mybot/cmake-build-debug/CMakeFiles/task3_PRMstar.dir/install-cxx-module-bmi-Debug.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/src" TYPE DIRECTORY FILES "/home/fins/myrobot_move/src/mybot/src/" FILES_MATCHING REGEX "/[^/]*\\.cpp$" REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/[^/]*\\.c$")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/config" TYPE DIRECTORY FILES "/home/fins/myrobot_move/src/mybot/config/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE DIRECTORY FILES "/home/fins/myrobot_move/src/mybot/launch" REGEX "/setup\\_assistant\\.launch$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/.setup_assistant")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/mybot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/mybot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/environment" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_index/share/ament_index/resource_index/packages/mybot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot/cmake" TYPE FILE FILES
    "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_core/mybotConfig.cmake"
    "/home/fins/myrobot_move/src/mybot/cmake-build-debug/ament_cmake_core/mybotConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mybot" TYPE FILE FILES "/home/fins/myrobot_move/src/mybot/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/fins/myrobot_move/src/mybot/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
