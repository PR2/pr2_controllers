# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_mechanism_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs pr2_mechanism_msgs nav_msgs pr2_msgs visualization_msgs diagnostic_msgs pr2_controller_interface roscpp pr2_controllers_msgs pr2_mechanism_model realtime_tools robot_mechanism_controllers  pluginlib rosconsole tf angles control_toolbox filters diagnostic_updater message_generation)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_message_files(
   DIRECTORY msg
   FILES
   BaseControllerState2.msg
   BaseControllerState.msg
   BaseOdometryState.msg
   DebugInfo.msg
   Odometer.msg
   OdometryMatrix.msg
   TrackLinkCmd.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   SetProfile.srv
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS})

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    DEPENDS EIGEN3
    CATKIN_DEPENDS std_msgs geometry_msgs pr2_mechanism_msgs nav_msgs pr2_msgs visualization_msgs diagnostic_msgs pr2_controller_interface roscpp pr2_controllers_msgs pr2_mechanism_model realtime_tools robot_mechanism_controllers  pluginlib rosconsole tf angles control_toolbox filters diagnostic_updater message_runtime
    INCLUDE_DIRS include
    LIBRARIES pr2_mechanism_controllers
)

add_definitions(-O3)

add_library( pr2_mechanism_controllers
  src/laser_scanner_traj_controller.cpp
  src/caster_controller.cpp
  src/base_kinematics.cpp
  src/pr2_odometry.cpp
  src/pr2_base_controller.cpp
  src/pr2_base_controller2.cpp
  src/pr2_gripper_controller.cpp
  src/trajectory.cpp
  )
find_package(PkgConfig)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
message(STATUS "Find URDFDOM ${URDFDOM_VERSION}")
target_compile_definitions(pr2_mechanism_controllers PRIVATE -DURDFDOM_1_0_0_API)
target_link_libraries(pr2_mechanism_controllers ${Boost_LIBRARIES} 
   ${catkin_LIBRARIES})
add_dependencies(pr2_mechanism_controllers
   ${pr2_mechanism_controllers_EXPORTED_TARGETS} ${pr2_msgs_EXPORTED_TARGETS})
pr2_enable_rpath(pr2_mechanism_controllers)

# TODO: tests

install(TARGETS pr2_mechanism_controllers
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
