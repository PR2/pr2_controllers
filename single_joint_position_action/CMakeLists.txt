# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(single_joint_position_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp pr2_controllers_msgs actionlib)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(single_joint_position_action src/single_joint_position_action.cpp)
target_link_libraries(single_joint_position_action ${Boost_LIBRARIES} 
   ${catkin_LIBRARIES})
add_dependencies(single_joint_position_action ${catkin_EXPORTED_TARGETS})

install(TARGETS single_joint_position_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
