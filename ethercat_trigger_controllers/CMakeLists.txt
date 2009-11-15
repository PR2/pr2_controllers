cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
set(_srcs
    src/trigger_controller.cpp
    src/multi_trigger_controller.cpp
    src/projector_controller.cpp
)

# For some reason, the endeffector_constraint_controller won't build on OSX
rospack_add_library(ethercat_trigger_controllers ${_srcs})
rospack_remove_compile_flags(ethercat_trigger_controllers -W)
target_link_libraries(ethercat_trigger_controllers ltdl)

# A hack to remove a library by an old name.
execute_process(COMMAND cmake -E remove ${PROJECT_SOURCE_DIR}/lib/librobot_mechanism_controllers.so)
