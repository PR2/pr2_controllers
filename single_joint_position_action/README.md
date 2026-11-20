See also
[JointSplineTrajectoryController](https://index.ros.org/p/robot_mechanism_controllers),
which is used by this action.

![attachment:sjposition_action_topics.png](doc/sjposition_action_topics.png)

## Usage

You can find an example of using the single joint position action on the
PR2 in the
[Moving the Torso tutorial](https://index.ros.org/p/pr2_controllers).

## ROS API

The ROS API consists of three parts: private parameters, an action
server that achieves position goals, and an interface to a trajectory
controller.

### Parameters

#### Parameters

- `~joint` (`string`, default: `Required`): The joint to command
- `~goal_threshold` (`double`, default: `0.1`): The maximum distance
  from the goal at which to report success.

### Action interface

The joint trajectory action provides an action server (see
[actionlib](https://index.ros.org/p/actionlib)) that takes in goals of
the type
[pr2_controllers_msgs/msg/SingleJointPositionGoal](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/SingleJointPositionGoal.html).

#### Subscribed Topics

- `~position_joint_action/goal`
  ([pr2_controllers_msgs/msg/SingleJointPositionActionGoal](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/SingleJointPositionActionGoal.html)):
  The goal describes the position for the robot to achieve
- `~position_joint_action/cancel`
  ([actionlib_msgs/msg/GoalID](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalID.html)):
  A request to cancel a specific goal.

#### Published Topics

- `~position_joint_action/feedback`
  ([pr2_controllers_msgs/msg/SingleJointPositionActionFeedback](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/SingleJointPositionActionFeedback.html)):
  Feedback describing the progress the mechanism is making on reaching
  the goal position
- `~position_joint_action/status`
  ([actionlib_msgs/msg/GoalStatusArray](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalStatusArray.html)):
  Provides status information on the goals that are sent to the action.
- `~position_joint_action/result`
  ([pr2_controllers_msgs/msg/SingleJointPositionActionResult](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/SingleJointPositionActionResult.html)):
  empty

### Controller interface

#### Subscribed Topics

- `~state`
  ([pr2_controllers_msgs/msg/JointTrajectoryControllerState](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/JointTrajectoryControllerState.html)):
  Listens to the state of the controller.

#### Published Topics

- `~command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Sends joint trajectories to the controller.
