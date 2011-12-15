cmake_minimum_required(VERSION 2.4.6)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_library( pr2_mechanism_controllers
  src/laser_scanner_traj_controller.cpp
  src/caster_controller.cpp
  src/base_kinematics.cpp
  src/pr2_odometry.cpp
  src/pr2_base_controller.cpp
  src/pr2_base_controller2.cpp
  src/pr2_gripper_controller.cpp
  src/trajectory.cpp
  )
rosbuild_link_boost(pr2_mechanism_controllers thread)
rosbuild_remove_compile_flags(pr2_mechanism_controllers -W)

