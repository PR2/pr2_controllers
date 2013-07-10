# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_controllers_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS actionlib_msgs trajectory_msgs geometry_msgs message_generation)

add_message_files(
   DIRECTORY msg
   FILES
   JointControllerState.msg
   JointTrajectoryControllerState.msg
   Pr2GripperCommand.msg
)
add_service_files(
   DIRECTORY srv
   FILES QueryCalibrationState.srv QueryTrajectoryState.srv
)
add_action_files(
   DIRECTORY action
   FILES
   JointTrajectory.action
   PointHead.action
   Pr2GripperCommand.action
   SingleJointPosition.action
)
## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES trajectory_msgs std_msgs geometry_msgs
   actionlib_msgs)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs trajectory_msgs geometry_msgs message_runtime
)
