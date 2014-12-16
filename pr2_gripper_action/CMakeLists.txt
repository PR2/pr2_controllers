# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib pr2_mechanism_controllers robot_mechanism_controllers pr2_controllers_msgs pr2_mechanism_model)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(pr2_gripper_action src/pr2_gripper_action.cpp)
target_link_libraries(pr2_gripper_action ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS pr2_gripper_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
