# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(joint_trajectory_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp trajectory_msgs actionlib_msgs pr2_controllers_msgs actionlib)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp trajectory_msgs actionlib_msgs pr2_controllers_msgs actionlib
)

add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action ${Boost_LIBRARIES})

install(TARGETS joint_trajectory_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
