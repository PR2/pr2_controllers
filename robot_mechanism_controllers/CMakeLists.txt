# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(robot_mechanism_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS  roscpp angles filters tf tf_conversions  eigen_conversions pluginlib actionlib std_msgs geometry_msgs diagnostic_msgs trajectory_msgs pr2_controllers_msgs pr2_controller_interface pr2_controller_manager kdl_parser pr2_mechanism_model control_toolbox realtime_tools message_filters control_msgs message_generation)


add_message_files(
   DIRECTORY msg
   FILES
   JTCartesianControllerState.msg
)

find_package(Boost REQUIRED)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
   CATKIN_DEPENDS  roscpp angles filters tf tf_conversions  eigen_conversions pluginlib actionlib std_msgs geometry_msgs diagnostic_msgs trajectory_msgs pr2_controllers_msgs pr2_controller_interface pr2_controller_manager kdl_parser pr2_mechanism_model control_toolbox realtime_tools message_filters control_msgs message_runtime
   INCLUDE_DIRS include
   LIBRARIES robot_mechanism_controllers
)

string(REPLACE "." ";" VERSION_LIST ${actionlib_VERSION})
list(GET VERSION_LIST 0 actionlib_VERSION_MAJOR)
list(GET VERSION_LIST 1 actionlib_VERSION_MINOR)
list(GET VERSION_LIST 2 actionlib_VERSION_PATCH)
add_definitions(-O3 -Dactionlib_VERSION=${actionlib_VERSION} -Dactionlib_VERSION_MAJOR=${actionlib_VERSION_MAJOR} -Dactionlib_VERSION_MINOR=${actionlib_VERSION_MINOR} -Dactionlib_VERSION_PATCH=${actionlib_VERSION_PATCH})

add_library(robot_mechanism_controllers
    src/cartesian_pose_controller.cpp
    src/cartesian_twist_controller.cpp
    src/cartesian_wrench_controller.cpp
    src/joint_effort_controller.cpp
    src/joint_position_controller.cpp
    src/joint_velocity_controller.cpp
    src/joint_spline_trajectory_controller.cpp
    src/joint_trajectory_action_controller.cpp
    src/jt_cartesian_controller.cpp
)
target_link_libraries(robot_mechanism_controllers ltdl ${Boost_LIBRARIES}
   ${catkin_LIBRARIES})
add_dependencies(robot_mechanism_controllers 
   ${robot_mechanism_controllers_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(robot_mechanism_controllers)

install(TARGETS robot_mechanism_controllers
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/effort.py posture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
