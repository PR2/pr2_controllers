cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rospack(pr2_mechanism_controllers)
genmsg()
gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library( pr2_mechanism_controllers
  src/laser_scanner_traj_controller.cpp
  src/caster_controller.cpp
  src/base_kinematics.cpp
  src/pr2_odometry.cpp
  src/pr2_base_controller.cpp
  )
rospack_link_boost(pr2_mechanism_controllers thread)
rospack_remove_compile_flags(pr2_mechanism_controllers -W)

