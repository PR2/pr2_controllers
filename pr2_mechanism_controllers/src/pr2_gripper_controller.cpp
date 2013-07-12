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

#include "pr2_mechanism_controllers/pr2_gripper_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( controller::Pr2GripperController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

Pr2GripperController::Pr2GripperController()
: joint_state_(NULL),
  loop_count_(0), robot_(NULL), last_time_(0)
{
}

Pr2GripperController::~Pr2GripperController()
{
  sub_command_.shutdown();
}

bool Pr2GripperController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;

  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot_->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint named \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (joint_state_->joint_->type != urdf::Joint::PRISMATIC)
  {
    ROS_ERROR("The joint \"%s\" was not prismatic (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s is not calibrated (namespace: %s)",
              joint_state_->joint_->name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!pid_.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<pr2_controllers_msgs::Pr2GripperCommand>(
    "command", 1, &Pr2GripperController::commandCB, this);

  return true;
}

void Pr2GripperController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  ros::Time time = robot_->getTime();
  assert(joint_state_->joint_);
  ros::Duration dt = time - last_time_;

  pr2_controllers_msgs::Pr2GripperCommandConstPtr command;
  command_box_.get(command);
  assert(command);

  // Computes the position error
  error = command->position - joint_state_->position_;

  // Sets the effort (limited)
  double effort = pid_.computeCommand(error, 0.0 - joint_state_->velocity_, dt);
  if (command->max_effort >= 0.0)
  {
    effort = std::max(-command->max_effort, std::min(effort, command->max_effort));
  }
  joint_state_->commanded_effort_ = effort;

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command->position;
      controller_state_publisher_->msg_.process_value = joint_state_->position_;
      controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt.toSec();
      controller_state_publisher_->msg_.command = effort;

      double dummy;
      pid_.getGains(controller_state_publisher_->msg_.p,
                    controller_state_publisher_->msg_.i,
                    controller_state_publisher_->msg_.d,
                    controller_state_publisher_->msg_.i_clamp,
                    dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
}

void Pr2GripperController::commandCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg)
{
  command_box_.set(msg);
}

}
