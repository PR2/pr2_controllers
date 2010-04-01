cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()
rosbuild_add_library(robot_mechanism_controllers
    src/cartesian_pose_controller.cpp
    src/cartesian_twist_controller.cpp
    src/cartesian_wrench_controller.cpp
    src/joint_effort_controller.cpp
    src/joint_position_controller.cpp
    src/joint_velocity_controller.cpp
    src/joint_spline_trajectory_controller.cpp
    src/joint_trajectory_action_controller.cpp
)


rosbuild_remove_compile_flags(robot_mechanism_controllers -W)
target_link_libraries(robot_mechanism_controllers ltdl)
