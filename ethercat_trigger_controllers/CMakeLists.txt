# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ethercat_trigger_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS pr2_controller_interface realtime_tools roscpp diagnostic_msgs pluginlib message_generation std_msgs)

add_message_files(
   DIRECTORY msg
   FILES MultiWaveform.msg MultiWaveformTransition.msg
)
add_service_files(
   DIRECTORY srv
   FILES SetMultiWaveform.srv SetWaveform.srv
)
set(_srcs
    src/trigger_controller.cpp
    src/multi_trigger_controller.cpp
    src/projector_controller.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS pr2_controller_interface realtime_tools roscpp diagnostic_msgs pluginlib message_runtime std_msgs
    INCLUDE_DIRS include
    LIBRARIES ethercat_trigger_controllers
)

add_definitions(-O3)

add_library(ethercat_trigger_controllers ${_srcs})
target_link_libraries(ethercat_trigger_controllers ltdl ${catkin_LIBRARIES})
add_dependencies(ethercat_trigger_controllers ${ethercat_trigger_controllers_EXPORTED_TARGETS})
pr2_enable_rpath(ethercat_trigger_controllers)

install(TARGETS ethercat_trigger_controllers
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
