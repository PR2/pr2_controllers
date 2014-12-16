# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_head_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   geometry_msgs 
   trajectory_msgs 
   sensor_msgs 
   pr2_controllers_msgs 
   roscpp 
   message_filters 
   tf 
   tf_conversions 
   actionlib 
   kdl_parser)

find_package(orocos_kdl REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS}
   ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(pr2_head_action src/pr2_point_frame.cpp)
target_link_libraries(pr2_head_action ${Boost_LIBRARIES} 
   ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pr2_head_action ${catkin_EXPORTED_TARGETS})

install(TARGETS pr2_head_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES head_action.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
