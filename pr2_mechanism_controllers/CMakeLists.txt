cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_library( pr2_mechanism_controllers
  src/laser_scanner_traj_controller.cpp
  src/caster_controller.cpp
  src/base_kinematics.cpp
  src/pr2_odometry.cpp
  src/pr2_base_controller.cpp
  src/pr2_gripper_controller.cpp
  )
rosbuild_link_boost(pr2_mechanism_controllers thread)
rosbuild_remove_compile_flags(pr2_mechanism_controllers -W)

