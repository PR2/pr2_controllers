# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_calibration_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs std_srvs pr2_controller_interface roscpp pr2_mechanism_model realtime_tools robot_mechanism_controllers pr2_mechanism_controllers pluginlib rosconsole angles control_toolbox)

add_library(${PROJECT_NAME}
  src/caster_calibration_controller.cpp
  src/gripper_calibration_controller.cpp
  src/joint_calibration_controller.cpp
  src/wrist_calibration_controller.cpp
  src/fake_calibration_controller.cpp
)


catkin_package(
    CATKIN_DEPENDS std_msgs std_srvs pr2_controller_interface roscpp pr2_mechanism_model realtime_tools robot_mechanism_controllers pr2_mechanism_controllers pluginlib rosconsole angles control_toolbox
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
