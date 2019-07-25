/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef PR2_GRIPPER_CONTROLLER_H__
#define PR2_GRIPPER_CONTROLLER_H__

/**
   @class pr2_controller_interface::Pr2GripperController
   @brief Joint Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "Pr2GripperController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (pr2_controllers_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <pr2_controllers_msgs/JointControllerState.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

namespace controller
{

class Pr2GripperController : public pr2_controller_interface::Controller
{
public:

  Pr2GripperController();
  ~Pr2GripperController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  virtual void starting() {
    using namespace pr2_controllers_msgs;
    Pr2GripperCommandPtr c(new Pr2GripperCommand);
    c->position = joint_state_->position_;
    c->max_effort = 0.0;
    command_box_.set(c);
  }

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  pr2_mechanism_model::JointState *joint_state_;
  realtime_tools::RealtimeBox<pr2_controllers_msgs::Pr2GripperCommandConstPtr> command_box_;

private:
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_;
  control_toolbox::Pid pid_;
  ros::Time last_time_;

  ros::NodeHandle node_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;
  void commandCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg);
};

} // namespace

#endif
