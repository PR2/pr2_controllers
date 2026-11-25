![attachment:gripper_action_topics.png](doc/gripper_action_topics.png)

Users interested in grasping more delicate objects may want to checkout
the alternative
[pr2_gripper_sensor_action](https://index.ros.org/p/pr2_gripper_sensor_action)
package.

## Usage

You can see an example of using the pr2 gripper action on the PR2 in the
[Moving the Gripper tutorial](https://index.ros.org/p/pr2).

## ROS API

The ROS API consists of three parts: private parameters, an action
server that achieves gripper goals, and an interface to the gripper
controller.

The [Moving the Gripper tutorial](https://index.ros.org/p/pr2) provides
more context on how the
[action and controller are launched](https://index.ros.org/p/pr2) and
how [action goals are interpreted](https://index.ros.org/p/pr2).

### Parameters

#### Parameters

- `~goal_threshold` (`double`, default: `0.01`): The distance from the
  desired position at which the result is considered successful.
- `~stall_timeout` (`double`, default: `0.1`): The amount of time (in
  seconds) the joint must be stationary to be considered a stall.
- `~stall_velocity_threshold` (`double`, default: `1e-6`): Parameter
  used when stall velocity oscillates (e.g. simulation jitter).

### Action interface

#### Subscribed Topics

- `~gripper_action/goal`
  ([pr2_controllers_msgs/msg/Pr2GripperCommandActionGoal](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/Pr2GripperCommandActionGoal.html)):
  The position/force for the gripper to achieve.
- `~gripper_action/cancel`
  ([actionlib_msgs/msg/GoalID](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalID.html)):
  A request to stop trying to move the gripper.

#### Published Topics

- `~gripper_action/status`
  ([actionlib_msgs/msg/GoalStatusArray](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalStatusArray.html)):
  Provides status information on the goals that are sent to the
  `action` action.
- `~gripper_action/result`
  ([pr2_controllers_msgs/msg/Pr2GripperActionResult](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/Pr2GripperActionResult.html)):
  Describes why the gripper stopped (stalled or reached the final
  position).

### Controller interface

#### Subscribed Topics

- `~state`
  ([pr2_controllers_msgs/msg/JointControllerState](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/JointControllerState.html)):
  Listens to the state of the controller.

#### Published Topics

- `~command`
  ([pr2_controllers_msgs/msg/Pr2GripperCommand](https://docs.ros.org/en/rolling/p/pr2_controllers_msgs/msg/Pr2GripperCommand.html)):
  Sends grip commands to the controller.
